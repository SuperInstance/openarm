"""
Safety envelope — manages constraint checking lifecycle.

Tracks violation history, computes safety scores,
and triggers emergency stops on critical violations.
"""

from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime
import json


@dataclass
class SafetyViolation:
    """Record of a constraint violation."""
    timestamp: str
    constraint_name: str
    severity: str
    message: str
    actual_value: float
    margin: float
    joint_states: dict  # Snapshot at violation time

    def to_dict(self) -> dict:
        return {
            "timestamp": self.timestamp,
            "constraint": self.constraint_name,
            "severity": self.severity,
            "message": self.message,
            "actual": self.actual_value,
            "margin": self.margin,
        }


class SafetyEnvelope:
    """
    Manages the safety envelope for an OpenArm.

    Tracks:
    - Current constraint satisfaction state
    - Violation history (last N violations)
    - Safety score (0.0 = all violated, 1.0 = all satisfied)
    - Emergency stop state
    """

    def __init__(self, max_violation_history: int = 1000):
        self.max_history = max_violation_history
        self.violation_history: List[SafetyViolation] = []
        self.last_results: List[dict] = []
        self.emergency_stop = False
        self.total_commands = 0
        self.blocked_commands = 0
        self.clamped_commands = 0

    def record_violation(self, result, joint_states: dict):
        """Record a constraint violation."""
        violation = SafetyViolation(
            timestamp=datetime.utcnow().isoformat(),
            constraint_name=result.constraint_name,
            severity=result.severity.value if hasattr(result.severity, 'value') else str(result.severity),
            message=result.message,
            actual_value=result.actual_value,
            margin=result.margin,
            joint_states=dict(joint_states),  # Shallow copy
        )
        self.violation_history.append(violation)

        # Trim history
        if len(self.violation_history) > self.max_history:
            self.violation_history = self.violation_history[-self.max_history:]

        # Emergency stop on critical violations
        sev = result.severity.value if hasattr(result.severity, 'value') else str(result.severity)
        if sev == "critical":
            self.emergency_stop = True

    def safety_score(self) -> float:
        """Compute current safety score based on recent violations."""
        if not self.last_results:
            return 1.0

        total = len(self.last_results)
        satisfied = sum(1 for r in self.last_results if r.get("satisfied", True))
        return satisfied / total if total > 0 else 1.0

    def violation_rate(self) -> float:
        """Fraction of commands that were blocked or clamped."""
        if self.total_commands == 0:
            return 0.0
        return (self.blocked_commands + self.clamped_commands) / self.total_commands

    def clear_emergency_stop(self):
        """Clear emergency stop (requires manual acknowledgment)."""
        self.emergency_stop = False

    def get_telemetry(self) -> dict:
        """Get safety telemetry for PLATO publishing."""
        return {
            "safety_score": self.safety_score(),
            "emergency_stop": self.emergency_stop,
            "total_commands": self.total_commands,
            "blocked_commands": self.blocked_commands,
            "clamped_commands": self.clamped_commands,
            "violation_rate": self.violation_rate(),
            "recent_violations": [v.to_dict() for v in self.violation_history[-10:]],
            "constraint_count": len(self.last_results),
        }

    def get_ensign_tile(self, device_id: str) -> dict:
        """Generate a PLATO ensign tile for this arm's safety state."""
        return {
            "room": device_id,
            "domain": "safety",
            "question": "envelope",
            "answer": json.dumps(self.get_telemetry()),
        }
