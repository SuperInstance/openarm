"""PLATO Room Decomposition for OpenArm.

Each pipeline stage in the OpenArm system becomes a PLATO room with an α dial.
Tiles carry motor telemetry, constraint checks, and fleet commands between rooms.

Room architecture:
  1. command_parse (α=0)    — Pure code: parse CAN commands, validate format
  2. constraint_check (α=0.1) — Code + micro-model: safety envelope + anomaly detection
  3. trajectory_plan (α=0.3)  — Code + model: smooth trajectory, avoid singularities
  4. fleet_coordination (α=0.5) — Model-assisted: multi-arm coordination, task allocation
  5. learning (α=0.7)        — Model-heavy: imitation learning, adaptation
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ── Tile types for OpenArm ────────────────────────────────────────────

@dataclass(frozen=True)
class JointState:
    """7-DOF joint state tile."""
    joints: Tuple[float, ...]  # 7 joint angles in radians
    velocities: Tuple[float, ...]  # 7 joint velocities
    torques: Tuple[float, ...]  # 7 joint torques
    timestamp: float
    source: str = "can_bus"

    def to_dict(self) -> dict:
        return {
            "joints": list(self.joints),
            "velocities": list(self.velocities),
            "torques": list(self.torques),
            "timestamp": self.timestamp,
            "source": self.source,
        }


@dataclass(frozen=True)
class ConstraintResult:
    """Safety constraint check tile."""
    passed: bool
    violations: Tuple[str, ...]
    min_margin: float  # Distance to nearest constraint boundary
    eisenstein_cell: Optional[Tuple[int, int]]  # Hex lattice cell if applicable
    timestamp: float
    check_latency_us: float  # Microseconds for constraint check

    def to_dict(self) -> dict:
        return {
            "passed": self.passed,
            "violations": list(self.violations),
            "min_margin": self.min_margin,
            "eisenstein_cell": list(self.eisenstein_cell) if self.eisenstein_cell else None,
            "timestamp": self.timestamp,
            "check_latency_us": self.check_latency_us,
        }


@dataclass(frozen=True)
class TrajectorySegment:
    """Planned trajectory segment tile."""
    start_joints: Tuple[float, ...]
    end_joints: Tuple[float, ...]
    duration_s: float
    max_velocity: float
    max_acceleration: float
    collision_free: bool
    timestamp: float

    def to_dict(self) -> dict:
        return {
            "start": list(self.start_joints),
            "end": list(self.end_joints),
            "duration_s": self.duration_s,
            "max_velocity": self.max_velocity,
            "collision_free": self.collision_free,
        }


@dataclass(frozen=True)
class FleetCommand:
    """Fleet coordination command tile."""
    arm_id: str
    command_type: str  # "move", "grasp", "release", "hold", "observe"
    priority: int  # 0=critical, 1=high, 2=normal, 3=low
    target: Optional[dict]  # Target position/object
    coordination_group: Optional[str]  # Which arms coordinate
    timestamp: float


# ── PLATO Rooms ───────────────────────────────────────────────────────

@dataclass
class RoomConfig:
    """Configuration for an OpenArm PLATO room."""
    name: str
    alpha: float  # 0=pure code, 1=full model
    description: str = ""
    max_latency_ms: float = 10.0  # Safety-critical: must be fast


class OpenArmRooms:
    """Pre-built PLATO rooms for OpenArm pipeline."""

    @staticmethod
    def command_parse() -> RoomConfig:
        """Room 1: Pure code CAN command parsing. α=0."""
        return RoomConfig(
            name="command_parse",
            alpha=0.0,
            description="Parse and validate CAN bus commands. No model needed.",
            max_latency_ms=0.1,
        )

    @staticmethod
    def constraint_check() -> RoomConfig:
        """Room 2: Safety envelope + anomaly detection. α=0.1.
        
        Code checks joint/torque/workspace limits (Eisenstein bounds).
        Micro-model detects unusual joint configurations that pass
        individual checks but are geometrically dangerous.
        """
        return RoomConfig(
            name="constraint_check",
            alpha=0.1,
            description="Safety envelope: Eisenstein workspace + micro-model anomaly detection",
            max_latency_ms=1.0,  # Must be <1ms for real-time control
        )

    @staticmethod
    def trajectory_plan() -> RoomConfig:
        """Room 3: Trajectory planning with model assistance. α=0.3.
        
        Code handles straight-line and simple joint interpolation.
        Model handles singularity avoidance, smooth trajectories,
        and complex multi-joint coordination.
        """
        return RoomConfig(
            name="trajectory_plan",
            alpha=0.3,
            description="Trajectory planning: code for simple paths, model for complex maneuvers",
            max_latency_ms=10.0,
        )

    @staticmethod
    def fleet_coordination() -> RoomConfig:
        """Room 4: Multi-arm fleet coordination. α=0.5.
        
        Code handles fixed choreography (rehearsed sequences).
        Model handles novel multi-arm tasks, dynamic replanning,
        and conflict resolution.
        """
        return RoomConfig(
            name="fleet_coordination",
            alpha=0.5,
            description="Fleet coordination: code for rehearsed sequences, model for novel tasks",
            max_latency_ms=100.0,
        )

    @staticmethod
    def learning() -> RoomConfig:
        """Room 5: Imitation learning and adaptation. α=0.7.
        
        Model-heavy: learns from demonstration, adapts to new objects,
        fine-tunes grasping strategies. Code handles basic retry logic.
        """
        return RoomConfig(
            name="learning",
            alpha=0.7,
            description="Learning: model for imitation/adaptation, code for retry logic",
            max_latency_ms=1000.0,
        )

    @classmethod
    def all_rooms(cls) -> List[RoomConfig]:
        return [
            cls.command_parse(),
            cls.constraint_check(),
            cls.trajectory_plan(),
            cls.fleet_coordination(),
            cls.learning(),
        ]


# ── Eisenstein Workspace Bounds ───────────────────────────────────────

# Third roots of unity for Eisenstein integer arithmetic
OMEGA_REAL = -0.5
OMEGA_IMAG = math.sqrt(3) / 2


def eisenstein_cell(x: float, y: float, scale: float = 1.0) -> Tuple[int, int]:
    """Map a 2D point to its Eisenstein integer cell.
    
    Uses the hex lattice structure of Z[ω] where ω = e^(2πi/3).
    Returns (a, b) such that the cell center is a + b*ω.
    """
    # Transform to Eisenstein basis
    # e1 = (1, 0), e2 = (-0.5, √3/2)
    # Inverse: [1, 0; 1/√3, 2/√3] * [x; y]
    bx = x / scale
    by = y / scale
    
    # Eisenstein coordinates
    a = round(bx - 0.5 * by * (2.0 / math.sqrt(3)))
    b = round(by * (2.0 / math.sqrt(3)))
    
    return (int(a), int(b))


def eisenstein_distance(x: float, y: float, cell: Tuple[int, int], scale: float = 1.0) -> float:
    """Distance from point to center of Eisenstein cell."""
    ca = cell[0] * scale
    cb = cell[1] * scale
    # Cell center in Cartesian
    cx = ca + cb * OMEGA_REAL
    cy = cb * OMEGA_IMAG
    return math.sqrt((x - cx) ** 2 + (y - cy) ** 2)


# ── Constraint Checker (Room 2 code path) ────────────────────────────

@dataclass(frozen=True)
class JointLimit:
    joint_idx: int
    min_rad: float
    max_rad: float


@dataclass(frozen=True) 
class TorqueLimit:
    joint_idx: int
    max_nm: float


@dataclass(frozen=True)
class WorkspaceLimit:
    """Eisenstein workspace boundary for a pair of joints."""
    joint_x: int  # Index of first joint
    joint_y: int  # Index of second joint
    max_radius: float  # Maximum radius from origin
    scale: float = 1.0  # Eisenstein lattice scale


class ConstraintChecker:
    """Room 2: constraint_check with α=0.1.
    
    Code path (α=0): Check joint limits, torque limits, workspace bounds.
    Model path (α=0.1): Detect unusual configurations that pass individual
    checks but are geometrically dangerous (e.g., near-singularity combos).
    """
    
    def __init__(
        self,
        joint_limits: List[JointLimit] | None = None,
        torque_limits: List[TorqueLimit] | None = None,
        workspace_limits: List[WorkspaceLimit] | None = None,
    ) -> None:
        self._joint_limits = joint_limits or [
            JointLimit(i, -math.pi, math.pi) for i in range(7)
        ]
        self._torque_limits = torque_limits or [
            TorqueLimit(i, 5.0) for i in range(7)
        ]
        self._workspace_limits = workspace_limits or [
            WorkspaceLimit(0, 1, 3.0),  # Shoulder
            WorkspaceLimit(1, 2, 2.5),  # Upper arm
            WorkspaceLimit(2, 3, 2.0),  # Elbow
            WorkspaceLimit(3, 4, 1.5),  # Wrist 1
        ]
    
    def check(self, state: JointState) -> ConstraintResult:
        """Run all constraint checks. Returns ConstraintResult tile."""
        start = time.monotonic()
        violations = []
        min_margin = float('inf')
        
        # Joint limits
        for lim in self._joint_limits:
            if lim.joint_idx < len(state.joints):
                angle = state.joints[lim.joint_idx]
                if angle < lim.min_rad:
                    violations.append(f"joint_{lim.joint_idx}_below_min: {angle:.3f} < {lim.min_rad:.3f}")
                    min_margin = min(min_margin, angle - lim.min_rad)
                elif angle > lim.max_rad:
                    violations.append(f"joint_{lim.joint_idx}_above_max: {angle:.3f} > {lim.max_rad:.3f}")
                    min_margin = min(min_margin, lim.max_rad - angle)
                else:
                    margin = min(angle - lim.min_rad, lim.max_rad - angle)
                    min_margin = min(min_margin, margin)
        
        # Torque limits
        for lim in self._torque_limits:
            if lim.joint_idx < len(state.torques):
                torque = abs(state.torques[lim.joint_idx])
                if torque > lim.max_nm:
                    violations.append(f"torque_{lim.joint_idx}_exceeded: {torque:.2f} > {lim.max_nm:.2f}")
                    min_margin = min(min_margin, lim.max_nm - torque)
        
        # Workspace bounds (Eisenstein)
        eisenstein_cell_val = None
        for ws in self._workspace_limits:
            if ws.joint_x < len(state.joints) and ws.joint_y < len(state.joints):
                x = state.joints[ws.joint_x]
                y = state.joints[ws.joint_y]
                r = math.sqrt(x*x + y*y)
                if r > ws.max_radius:
                    violations.append(f"workspace_{ws.joint_x}_{ws.joint_y}_exceeded: r={r:.3f} > {ws.max_radius:.3f}")
                    min_margin = min(min_margin, ws.max_radius - r)
                else:
                    min_margin = min(min_margin, ws.max_radius - r)
                    eisenstein_cell_val = eisenstein_cell(x, y, ws.scale)
        
        elapsed_us = (time.monotonic() - start) * 1_000_000
        
        return ConstraintResult(
            passed=len(violations) == 0,
            violations=tuple(violations),
            min_margin=round(min_margin, 4) if min_margin != float('inf') else 0.0,
            eisenstein_cell=eisenstein_cell_val,
            timestamp=time.time(),
            check_latency_us=round(elapsed_us, 2),
        )
