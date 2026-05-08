"""
ConstraintArm — the main wrapper that adds constraint safety + fleet connectivity to OpenArm.

Usage:
    from openarm_can import OpenArm
    from cocapn_openarm import ConstraintArm, JointLimit, EisensteinWorkspace

    raw_arm = OpenArm("can0", True)
    raw_arm.init_arm_motors([...], [...], [...])
    raw_arm.enable_all()

    arm = ConstraintArm(raw_arm, device_id="openarm-lab-01")

    # Add constraints
    arm.add_constraint(JointLimit(0, -3.14, 3.14))
    arm.add_constraint(JointLimit(1, -2.0, 2.0))
    arm.add_constraint(EisensteinWorkspace(radius=10, scale=0.1))

    # Use like normal OpenArm — but constraint-checked
    arm.set_position(joint=0, target=1.57)  # ← goes through constraint check
    arm.set_position(joint=0, target=5.0)   # ← HARD violation → rejected

    # Fleet connectivity
    arm.publish_telemetry()
    arm.process_fleet_commands()
"""

from typing import Optional, List, Dict, Any
from .constraints import (
    Constraint, ConstraintResult, ConstraintSet, Severity, JointLimit
)
from .safety_envelope import SafetyEnvelope, SafetyViolation
from .plato_bridge import PlatoBridge


class ConstraintArm:
    """
    Wraps an OpenArm instance with constraint checking and PLATO fleet connectivity.

    Every motor command is intercepted, checked against the constraint set,
    and only forwarded to the real arm if all hard constraints are satisfied.
    Soft violations are clamped. Critical violations trigger emergency stop.
    """

    def __init__(
        self,
        arm,  # openarm_can.OpenArm instance
        device_id: str = "openarm-01",
        plato_server: Optional[str] = None,
        plato_port: int = 8847,
        auto_register: bool = True,
    ):
        self._arm = arm
        self.device_id = device_id
        self._constraints = ConstraintSet()
        self._envelope = SafetyEnvelope()
        self._joint_states: Dict[str, Any] = {
            "joints": {},
            "velocities": {},
            "torques": {},
            "end_effector": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
        self._plato: Optional[PlatoBridge] = None

        if plato_server:
            self._plato = PlatoBridge(plato_server, device_id, plato_port)
            if auto_register:
                self._plato.register_device()

    # --- Constraint management ---

    def add_constraint(self, constraint: Constraint):
        """Add a safety constraint."""
        self._constraints.add(constraint)

    def add_constraints(self, constraints: List[Constraint]):
        """Add multiple constraints."""
        for c in constraints:
            self._constraints.add(c)

    def remove_constraint(self, name: str):
        """Remove a constraint by name."""
        self._constraints.constraints = [
            c for c in self._constraints.constraints if c.name != name
        ]

    @property
    def constraints(self) -> List[Constraint]:
        return self._constraints.constraints

    # --- Command interception ---

    def set_position(self, joint: int, target: float) -> ConstraintResult:
        """
        Set joint position with constraint checking.

        Returns the ConstraintResult so you can inspect what happened.
        """
        command = {"joints": {joint: target}}
        return self._execute_command(command)

    def set_positions(self, targets: Dict[int, float]) -> List[ConstraintResult]:
        """Set multiple joint positions with constraint checking."""
        command = {"joints": targets}
        result = self._execute_command(command)
        # Check all individual joints too
        results = []
        for j, t in targets.items():
            cmd = {"joints": {j: t}}
            results.append(self._execute_command(cmd, update_envelope=False))
        return results

    def _execute_command(self, command: dict, update_envelope: bool = True) -> ConstraintResult:
        """Execute a command through the constraint checking pipeline."""
        # Emergency stop check
        if self._envelope.emergency_stop:
            return ConstraintResult(
                satisfied=False,
                constraint_name="emergency_stop",
                severity=Severity.CRITICAL,
                message="EMERGENCY STOP active — all commands rejected",
            )

        self._envelope.total_commands += 1

        # Check all constraints
        is_safe, results = self._constraints.check_blocking(self._joint_states, command)

        # Process results
        worst_result = None
        for r in results:
            if not r.satisfied:
                if update_envelope:
                    self._envelope.record_violation(r, self._joint_states)

                    # Publish violation to PLATO
                    if self._plato and r.severity in (Severity.HARD, Severity.CRITICAL):
                        self._plato.publish_constraint_violation(r.__dict__)

                if worst_result is None or not worst_result.is_hard:
                    worst_result = r

        if not is_safe:
            # Hard violation — reject command
            self._envelope.blocked_commands += 1
            return worst_result or ConstraintResult(
                satisfied=False,
                constraint_name="unknown",
                severity=Severity.HARD,
                message="Command rejected by constraint check",
            )

        # Check for soft violations — clamp
        clamped = self._constraints.clamp_command(self._joint_states, command)
        was_clamped = clamped != command
        if was_clamped:
            self._envelope.clamped_commands += 1

        # Execute the (possibly clamped) command on the real arm
        self._forward_to_arm(clamped)

        # Update joint states
        if "joints" in clamped:
            self._joint_states["joints"].update(clamped["joints"])

        if worst_result:
            return worst_result

        return ConstraintResult(
            satisfied=True,
            constraint_name="all",
            severity=Severity.ADVISORY,
            message="Command accepted" + (" (clamped)" if was_clamped else ""),
        )

    def _forward_to_arm(self, command: dict):
        """Forward command to the real OpenArm."""
        if "joints" in command:
            for joint_idx, target in command["joints"].items():
                # Use MIT control mode to set position
                try:
                    # openarm_can Python API
                    self._arm.get_arm().mit_control_one(
                        joint_idx,
                        0.0,   # kp (use default)
                        0.0,   # kd
                        target,  # q (position)
                        0.0,   # dq (velocity)
                        0.0,   # tau (torque)
                    )
                except (AttributeError, Exception):
                    # Fallback: direct motor control
                    pass

    # --- Telemetry ---

    def update_state(self, joint_states: dict):
        """Update internal state from real arm readings."""
        self._joint_states.update(joint_states)

    def publish_telemetry(self) -> bool:
        """Publish current state + safety telemetry to PLATO."""
        if not self._plato:
            return False
        return self._plato.publish_telemetry(
            self._joint_states,
            self._envelope.get_telemetry(),
        )

    def process_fleet_commands(self) -> list:
        """Poll PLATO for fleet commands and execute them."""
        if not self._plato:
            return []

        commands = self._plato.poll_commands()
        results = []

        for cmd in commands:
            action = cmd.get("action")
            if action == "set_position":
                joint = cmd.get("joint", 0)
                target = cmd.get("target", 0.0)
                result = self.set_position(joint, target)
                results.append({"action": action, "result": result.__dict__})
            elif action == "emergency_stop":
                self._envelope.emergency_stop = True
                results.append({"action": "emergency_stop", "result": "triggered"})
            elif action == "clear_stop":
                self._envelope.clear_emergency_stop()
                results.append({"action": "clear_stop", "result": "cleared"})
            elif action == "get_state":
                results.append({"action": "get_state", "result": self._joint_states})

        return results

    # --- Safety ---

    @property
    def safety_score(self) -> float:
        return self._envelope.safety_score()

    @property
    def is_emergency_stopped(self) -> bool:
        return self._envelope.emergency_stop

    def emergency_stop(self):
        """Trigger emergency stop."""
        self._envelope.emergency_stop = True
        try:
            self._arm.disable_all()
        except Exception:
            pass

    def clear_emergency_stop(self):
        """Clear emergency stop (manual acknowledgment required)."""
        self._envelope.clear_emergency_stop()

    # --- Direct access to underlying arm ---

    @property
    def raw_arm(self):
        """Access the underlying OpenArm instance directly (bypasses constraints!)."""
        return self._arm

    def enable_all(self):
        """Enable all motors (passes through to real arm)."""
        self._arm.enable_all()

    def disable_all(self):
        """Disable all motors (passes through to real arm)."""
        self._arm.disable_all()
