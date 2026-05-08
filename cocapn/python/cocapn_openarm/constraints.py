"""
Constraint types for OpenArm safety envelope.

Each constraint checks a condition and returns ConstraintResult.
Severity levels determine what happens on violation:
  - advisory: log warning, command proceeds
  - soft: command clamped to safe boundary
  - hard: command rejected
  - critical: emergency stop triggered
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import math


class Severity(Enum):
    ADVISORY = "advisory"
    SOFT = "soft"
    HARD = "hard"
    CRITICAL = "critical"


@dataclass
class ConstraintResult:
    """Result of checking a single constraint."""
    satisfied: bool
    constraint_name: str
    severity: Severity
    message: str = ""
    actual_value: float = 0.0
    safe_value: Optional[float] = None  # For soft violations: clamped value
    margin: float = 0.0  # How far from violation (negative = violated)

    @property
    def is_critical(self) -> bool:
        return not self.satisfied and self.severity == Severity.CRITICAL

    @property
    def is_hard(self) -> bool:
        return not self.satisfied and self.severity in (Severity.HARD, Severity.CRITICAL)


class Constraint:
    """Base class for all constraints."""
    name: str = "base"
    severity: Severity = Severity.HARD

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        """Check constraint against current state + proposed command."""
        raise NotImplementedError

    def eisenstein_norm(self, a: int, b: int) -> int:
        """Eisenstein integer norm: a² - ab + b²."""
        return a * a - a * b + b * b


@dataclass
class JointLimit(Constraint):
    """Joint position limit constraint."""
    joint_index: int
    min_pos: float
    max_pos: float
    severity: Severity = Severity.HARD
    name: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = f"joint_{self.joint_index}_limit"

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        joints = command.get("joints", joint_states.get("joints", {}))
        target = joints.get(self.joint_index, 0.0)
        current = joint_states.get("joints", {}).get(self.joint_index, 0.0)
        margin_min = target - self.min_pos
        margin_max = self.max_pos - target

        if margin_min >= 0 and margin_max >= 0:
            return ConstraintResult(
                satisfied=True,
                constraint_name=self.name,
                severity=self.severity,
                actual_value=target,
                margin=min(margin_min, margin_max),
            )

        violated_margin = min(margin_min, margin_max)
        safe_value = max(self.min_pos, min(self.max_pos, target))

        return ConstraintResult(
            satisfied=False,
            constraint_name=self.name,
            severity=self.severity,
            message=f"Joint {self.joint_index}: {target:.3f} outside [{self.min_pos:.3f}, {self.max_pos:.3f}]",
            actual_value=target,
            safe_value=safe_value,
            margin=violated_margin,
        )


@dataclass
class TorqueLimit(Constraint):
    """Joint torque limit constraint."""
    joint_index: int
    max_torque: float
    severity: Severity = Severity.HARD
    name: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = f"joint_{self.joint_index}_torque"

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        torque = joint_states.get("torques", {}).get(self.joint_index, 0.0)
        margin = self.max_torque - abs(torque)

        if margin >= 0:
            return ConstraintResult(
                satisfied=True,
                constraint_name=self.name,
                severity=self.severity,
                actual_value=torque,
                margin=margin,
            )

        return ConstraintResult(
            satisfied=False,
            constraint_name=self.name,
            severity=self.severity,
            message=f"Joint {self.joint_index} torque: {torque:.2f} exceeds {self.max_torque:.2f}",
            actual_value=torque,
            margin=margin,
        )


@dataclass
class VelocityLimit(Constraint):
    """Joint velocity limit constraint."""
    joint_index: int
    max_velocity: float
    severity: Severity = Severity.ADVISORY
    name: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = f"joint_{self.joint_index}_velocity"

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        velocity = joint_states.get("velocities", {}).get(self.joint_index, 0.0)
        margin = self.max_velocity - abs(velocity)

        if margin >= 0:
            return ConstraintResult(
                satisfied=True,
                constraint_name=self.name,
                severity=self.severity,
                actual_value=velocity,
                margin=margin,
            )

        return ConstraintResult(
            satisfied=False,
            constraint_name=self.name,
            severity=self.severity,
            message=f"Joint {self.joint_index} velocity: {velocity:.3f} exceeds {self.max_velocity:.3f}",
            actual_value=velocity,
            margin=margin,
        )


@dataclass
class WorkspaceBoundary(Constraint):
    """Cartesian workspace boundary (sphere)."""
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 1.0
    severity: Severity = Severity.CRITICAL
    name: str = "workspace_boundary"

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        # Get end-effector position from state
        ee = joint_states.get("end_effector", {})
        x = ee.get("x", 0.0) - self.center[0]
        y = ee.get("y", 0.0) - self.center[1]
        z = ee.get("z", 0.0) - self.center[2]
        dist = math.sqrt(x * x + y * y + z * z)
        margin = self.radius - dist

        if margin >= 0:
            return ConstraintResult(
                satisfied=True,
                constraint_name=self.name,
                severity=self.severity,
                actual_value=dist,
                margin=margin,
            )

        return ConstraintResult(
            satisfied=False,
            constraint_name=self.name,
            severity=self.severity,
            message=f"End effector at distance {dist:.3f} exceeds workspace radius {self.radius:.3f}",
            actual_value=dist,
            margin=margin,
        )


@dataclass
class EisensteinWorkspace(Constraint):
    """
    Eisenstein lattice workspace constraint.

    Maps the hex lattice geometry to workspace bounds using
    Eisenstein integer norms. This gives 6-fold symmetric
    workspace boundaries that naturally align with joint space
    geometry for 6-DOF and 7-DOF arms.

    The constraint checks that the end-effector position,
    mapped to Eisenstein coordinates, lies within a disk
    of the given radius in the Eisenstein lattice.
    """
    radius: int = 10  # Eisenstein norm radius
    center_a: int = 0
    center_b: int = 0
    scale: float = 0.1  # meters per Eisenstein unit
    severity: Severity = Severity.CRITICAL
    name: str = "eisenstein_workspace"

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        ee = joint_states.get("end_effector", {})
        x = ee.get("x", 0.0)
        y = ee.get("y", 0.0)

        # Map Cartesian to Eisenstein coordinates
        # Using the basis vectors: e1 = (1, 0), e2 = (1/2, √3/2)
        a = round(x / self.scale - y / (self.scale * math.sqrt(3)))
        b = round(2 * y / (self.scale * math.sqrt(3)))

        # Eisenstein norm
        da = a - self.center_a
        db = b - self.center_b
        norm = self.eisenstein_norm(da, db)
        norm_sqrt = math.sqrt(norm)
        margin = self.radius - norm_sqrt

        if margin >= 0:
            return ConstraintResult(
                satisfied=True,
                constraint_name=self.name,
                severity=self.severity,
                actual_value=norm_sqrt,
                margin=margin,
                message=f"Eisenstein norm {norm_sqrt:.2f} within radius {self.radius}",
            )

        return ConstraintResult(
            satisfied=False,
            constraint_name=self.name,
            severity=self.severity,
            message=f"Eisenstein workspace violation: norm {norm_sqrt:.2f} > radius {self.radius}",
            actual_value=norm_sqrt,
            margin=margin,
        )


class ConstraintSet:
    """Collection of constraints with bulk checking."""

    def __init__(self):
        self.constraints: List[Constraint] = []

    def add(self, constraint: Constraint):
        self.constraints.append(constraint)

    def check_all(self, joint_states: dict, command: dict) -> List[ConstraintResult]:
        """Check all constraints. Returns results."""
        return [c.check(joint_states, command) for c in self.constraints]

    def check_blocking(self, joint_states: dict, command: dict) -> Tuple[bool, List[ConstraintResult]]:
        """Check constraints, return (is_safe, results)."""
        results = self.check_all(joint_states, command)
        is_safe = not any(r.is_hard for r in results)
        return is_safe, results

    def clamp_command(self, joint_states: dict, command: dict) -> dict:
        """For soft violations, clamp the command to safe values."""
        results = self.check_all(joint_states, command)
        clamped = dict(command)

        for r in results:
            if not r.satisfied and r.severity == Severity.SOFT and r.safe_value is not None:
                # Apply clamping
                if "joints" in clamped:
                    # Find the joint index from the constraint name
                    for c in self.constraints:
                        if c.name == r.constraint_name and hasattr(c, 'joint_index'):
                            clamped["joints"][c.joint_index] = r.safe_value

        return clamped
