"""
Cocapn OpenArm — constraint-aware safety envelope + fleet connectivity.

Drop-in enhancement for any OpenArm installation.
Wraps openarm_can with Eisenstein constraint checking and PLATO fleet connectivity.

Usage:
    from openarm_can import OpenArm
    from cocapn_openarm import ConstraintArm

    raw = OpenArm("can0", True)
    arm = ConstraintArm(raw, plato_server="192.168.1.100:8847")

    arm.add_constraint(JointLimit(0, -3.14, 3.14, severity="hard"))
    arm.add_constraint(TorqueLimit(0, 5.0, severity="hard"))
    arm.add_constraint(EisensteinWorkspace(radius=10, center=(0,0,0)))

    arm.set_position(joint=0, target=1.57)  # ← constraint-checked
    arm.publish_telemetry()                  # → PLATO
"""

__version__ = "0.1.0"
__author__ = "Cocapn Fleet — SuperInstance"

from .constraints import (
    Constraint,
    ConstraintResult,
    Severity,
    JointLimit,
    TorqueLimit,
    VelocityLimit,
    WorkspaceBoundary,
    EisensteinWorkspace,
    ConstraintSet,
)
from .safety_envelope import SafetyEnvelope, SafetyViolation
from .constraint_arm import ConstraintArm
from .plato_bridge import PlatoBridge

__all__ = [
    "ConstraintArm",
    "Constraint",
    "ConstraintResult",
    "Severity",
    "JointLimit",
    "TorqueLimit",
    "VelocityLimit",
    "WorkspaceBoundary",
    "EisensteinWorkspace",
    "ConstraintSet",
    "SafetyEnvelope",
    "SafetyViolation",
    "PlatoBridge",
]
