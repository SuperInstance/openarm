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
class SplineSnapConstraint(Constraint):
    """
    Smooth workspace boundary using spline snap-to-manifold.

    Instead of hard rejection at the boundary, this snaps the target
    to the nearest point on a smooth spline envelope. Uses a
    deadband-aware offset — small violations within tolerance are
    smoothly corrected, not abruptly rejected.

    The snap function uses Pythagorean triple manifold anchoring:
    map the angle to the nearest Pythagorean triple angle, then
    compute the smooth offset within the deadband.
    """
    joint_index: int
    boundary_min: float
    boundary_max: float
    deadband: float = 0.1  # fraction of range (0.1 = 10% tolerance)
    snap_strength: float = 2.0  # how quickly snap activates (higher = sharper)
    severity: Severity = Severity.SOFT
    name: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = f"joint_{self.joint_index}_spline_snap"

    @property
    def range_size(self) -> float:
        return self.boundary_max - self.boundary_min

    @property
    def deadband_abs(self) -> float:
        """Absolute deadband in position units."""
        return self.deadband * self.range_size

    def snap_value(self, target: float) -> float:
        """
        Apply smooth spline snap to a target value.

        Inside safe zone (away from boundaries): zero correction.
        Approaching boundary within deadband: smooth increasing correction.
        Outside boundary: full snap to boundary.

        The correction uses a power-law profile within the deadband:
          correction = -sign(diff) * deadband_abs * (|diff|/deadband_abs)^snap_strength
        """
        safe_min = self.boundary_min + self.deadband_abs
        safe_max = self.boundary_max - self.deadband_abs
        db = self.deadband_abs

        if db <= 0:
            # Degenerate case: hard clamp
            return max(self.boundary_min, min(self.boundary_max, target))

        # Hard clamp first (outside boundary)
        if target <= self.boundary_min:
            return self.boundary_min
        if target >= self.boundary_max:
            return self.boundary_max

        # Lower deadband zone: target in [boundary_min, safe_min]
        if target < safe_min:
            diff = target - self.boundary_min  # distance from boundary (positive)
            ratio = diff / db  # 0 at boundary, 1 at safe_min edge
            correction = db * (1.0 - ratio ** self.snap_strength)
            return target + correction

        # Upper deadband zone: target in [safe_max, boundary_max]
        if target > safe_max:
            diff = self.boundary_max - target  # distance from boundary (positive)
            ratio = diff / db  # 0 at boundary, 1 at safe_max edge
            correction = db * (1.0 - ratio ** self.snap_strength)
            return target - correction

        # Well inside safe zone: no correction
        return target

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        joints = command.get("joints", joint_states.get("joints", {}))
        target = joints.get(self.joint_index, 0.0)
        snapped = self.snap_value(target)

        # Compute margin (distance to nearest boundary)
        margin_min = target - self.boundary_min
        margin_max = self.boundary_max - target
        margin = min(margin_min, margin_max)

        satisfied = abs(snapped - target) < 1e-9

        return ConstraintResult(
            satisfied=satisfied,
            constraint_name=self.name,
            severity=self.severity,
            message=f"Joint {self.joint_index}: target={target:.3f}, snapped={snapped:.3f}"
                     if not satisfied else "",
            actual_value=target,
            safe_value=snapped,
            margin=margin,
        )


@dataclass
class SplineSnapWorkspace(Constraint):
    """
    Smooth Eisenstein workspace boundary using spline snap.

    Instead of hard rejection at the hex boundary, applies a smooth
    inward correction vector whose magnitude increases as the end-effector
    approaches the boundary. Uses the Eisenstein norm to measure distance
    to the hex workspace boundary.
    """
    radius: int = 10  # Eisenstein norm radius
    center_a: int = 0
    center_b: int = 0
    scale: float = 0.1  # meters per Eisenstein unit
    deadband: float = 0.15  # fraction of radius for smooth zone
    snap_strength: float = 2.0
    severity: Severity = Severity.SOFT
    name: str = "spline_snap_workspace"

    def _to_eisenstein(self, x: float, y: float) -> Tuple[int, int]:
        a = round(x / self.scale - y / (self.scale * math.sqrt(3)))
        b = round(2 * y / (self.scale * math.sqrt(3)))
        return a, b

    def _to_cartesian(self, a: int, b: int) -> Tuple[float, float]:
        x = (a + b * 0.5) * self.scale
        y = b * (math.sqrt(3) / 2) * self.scale
        return x, y

    def snap_position(self, x: float, y: float) -> Tuple[float, float]:
        """
        Apply smooth snap to keep (x, y) within the Eisenstein workspace.

        Returns the snapped (x, y) position.
        """
        a, b = self._to_eisenstein(x, y)
        da = a - self.center_a
        db = b - self.center_b
        norm = da * da - da * db + db * db
        dist = math.sqrt(norm)

        if dist <= 0:
            return x, y

        db_zone = self.deadband * self.radius
        safe_radius = self.radius - db_zone

        if dist <= safe_radius:
            # Well inside safe zone
            return x, y

        if dist >= self.radius:
            # Hard snap to boundary
            scale_factor = self.radius / dist
            a_new = self.center_a + da * scale_factor
            b_new = self.center_b + db * scale_factor
            return self._to_cartesian(int(round(a_new)), int(round(b_new)))

        # Deadband zone: smooth correction
        ratio = (self.radius - dist) / db_zone  # 1 at safe edge, 0 at boundary
        correction_strength = 1.0 - ratio ** self.snap_strength

        # Push inward
        scale_factor = (dist - db_zone * correction_strength) / dist
        a_new = self.center_a + da * scale_factor
        b_new = self.center_b + db * scale_factor
        return self._to_cartesian(int(round(a_new)), int(round(b_new)))

    def check(self, joint_states: dict, command: dict) -> ConstraintResult:
        ee = joint_states.get("end_effector", {})
        x = ee.get("x", 0.0)
        y = ee.get("y", 0.0)

        a, b = self._to_eisenstein(x, y)
        da = a - self.center_a
        db = b - self.center_b
        norm = self.eisenstein_norm(da, db)
        dist = math.sqrt(norm)
        margin = self.radius - dist

        sx, sy = self.snap_position(x, y)
        correction = math.sqrt((sx - x) ** 2 + (sy - y) ** 2)
        satisfied = correction < 1e-9

        return ConstraintResult(
            satisfied=satisfied,
            constraint_name=self.name,
            severity=self.severity,
            message=f"Workspace snap: ({x:.3f},{y:.3f}) -> ({sx:.3f},{sy:.3f})"
                     if not satisfied else "",
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

    def _find_constraint(self, name: str) -> Optional[Constraint]:
        """Find a constraint by name."""
        for c in self.constraints:
            if c.name == name:
                return c
        return None

    def clamp_command(self, joint_states: dict, command: dict) -> dict:
        """For soft violations, apply smooth snap (preferred) or hard clamp."""
        results = self.check_all(joint_states, command)
        clamped = dict(command)

        for r in results:
            if r.severity != Severity.SOFT:
                continue

            constraint = self._find_constraint(r.constraint_name)
            if constraint is None:
                continue

            # Use smooth snap for SplineSnapConstraint
            if isinstance(constraint, SplineSnapConstraint):
                joints = clamped.get("joints", command.get("joints", joint_states.get("joints", {})))
                if constraint.joint_index in joints:
                    joints = dict(joints)
                    joints[constraint.joint_index] = constraint.snap_value(
                        joints[constraint.joint_index]
                    )
                    clamped["joints"] = joints
            # Use smooth snap for SplineSnapWorkspace (end-effector)
            elif isinstance(constraint, SplineSnapWorkspace):
                ee = dict(clamped.get("end_effector",
                            joint_states.get("end_effector", {})))
                x, y = ee.get("x", 0.0), ee.get("y", 0.0)
                sx, sy = constraint.snap_position(x, y)
                ee["x"] = sx
                ee["y"] = sy
                clamped["end_effector"] = ee
            # Fallback: hard clamp for other soft constraints
            elif not r.satisfied and r.safe_value is not None:
                if "joints" in clamped and hasattr(constraint, 'joint_index'):
                    clamped["joints"] = dict(clamped["joints"])
                    clamped["joints"][constraint.joint_index] = r.safe_value

        return clamped
