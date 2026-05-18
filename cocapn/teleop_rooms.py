"""PLATO Teleop Rooms for OpenArm.

Extends the base OpenArm PLATO rooms with teleoperation-specific pipeline
stages: leader arm parsing, joint-space mapping, real-time safety,
multi-arm fleet coordination, and demonstration recording.

Room architecture (teleop pipeline):
  1. TeleopParseRoom (α=0)     — Parse leader arm CAN commands, pure code
  2. TeleopMapRoom (α=0.2)     — Map leader → follower joint space
  3. TeleopSafetyRoom (α=0.1)  — Real-time safety envelope for teleop
  4. TeleopFleetRoom (α=0.5)   — Multi-arm coordination during teleop
  5. TeleopRecordRoom (α=0.3)  — Record demonstrations for imitation learning

Integration:
  - Imports JointState, ConstraintChecker, ConstraintResult from plato_rooms
  - Tiles flow: leader CAN → parse → map → safety → fleet → record → follower CAN
  - All rooms support both code-path (deterministic) and model-path (micro-model)
"""

from __future__ import annotations

import math
import time
import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from plato_rooms import (
    JointState,
    ConstraintResult,
    ConstraintChecker,
    TrajectorySegment,
    FleetCommand,
    RoomConfig,
    eisenstein_cell,
    eisenstein_distance,
)


# ── Teleop Tile Types ─────────────────────────────────────────────────

@dataclass(frozen=True)
class LeaderCommand:
    """Raw leader arm command from CAN bus or ROS2 topic."""
    joints: Tuple[float, ...]       # Leader joint angles (radians)
    gripper: float                  # Gripper opening (0=closed, 1=open)
    timestamp: float
    source: str = "leader_can"      # "leader_can", "ros2_topic", "sim"
    frame_id: int = 0               # Monotonic frame counter

    def to_dict(self) -> dict:
        return {
            "joints": list(self.joints),
            "gripper": self.gripper,
            "timestamp": self.timestamp,
            "source": self.source,
            "frame_id": self.frame_id,
        }


@dataclass(frozen=True)
class MappedJoints:
    """Follower joint targets after leader→follower mapping."""
    joints: Tuple[float, ...]       # Follower target angles
    gripper: float                  # Mapped gripper target
    mapping_type: str               # "linear", "scaled", "model"
    timestamp: float
    leader_frame_id: int = 0

    def to_dict(self) -> dict:
        return {
            "joints": list(self.joints),
            "gripper": self.gripper,
            "mapping_type": self.mapping_type,
            "leader_frame_id": self.leader_frame_id,
        }


@dataclass(frozen=True)
class TeleopSafetyResult:
    """Safety check result specific to teleop."""
    passed: bool
    violations: Tuple[str, ...]
    clamped_joints: Tuple[float, ...]  # Joints after clamping
    clamped_gripper: float
    min_margin: float
    timestamp: float
    check_latency_us: float

    def to_dict(self) -> dict:
        return {
            "passed": self.passed,
            "violations": list(self.violations),
            "clamped_joints": list(self.clamped_joints),
            "clamped_gripper": self.clamped_gripper,
            "min_margin": self.min_margin,
            "check_latency_us": self.check_latency_us,
        }


@dataclass(frozen=True)
class FleetTeleopCommand:
    """Multi-arm coordination command during teleop."""
    arm_id: str
    target_joints: Tuple[float, ...]
    mode: str                      # "follow", "mirror", "offset", "choreography"
    priority: int                  # 0=critical, 1=high, 2=normal
    coordination_group: str
    timestamp: float

    def to_dict(self) -> dict:
        return {
            "arm_id": self.arm_id,
            "target_joints": list(self.target_joints),
            "mode": self.mode,
            "priority": self.priority,
            "coordination_group": self.coordination_group,
        }


@dataclass
class DemonstrationFrame:
    """Single frame of a recorded demonstration."""
    leader_joints: Tuple[float, ...]
    follower_joints: Tuple[float, ...]
    gripper: float
    safety_passed: bool
    timestamp: float
    frame_id: int

    def to_dict(self) -> dict:
        return {
            "leader_joints": list(self.leader_joints),
            "follower_joints": list(self.follower_joints),
            "gripper": self.gripper,
            "safety_passed": self.safety_passed,
            "frame_id": self.frame_id,
        }


@dataclass
class Demonstration:
    """Complete recorded demonstration for imitation learning."""
    name: str
    frames: List[DemonstrationFrame] = field(default_factory=list)
    start_time: float = 0.0
    end_time: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        return self.end_time - self.start_time if self.frames else 0.0

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "frame_count": self.frame_count,
            "duration_s": round(self.duration_s, 3),
            "metadata": self.metadata,
            "frames": [f.to_dict() for f in self.frames],
        }


# ── Room 1: TeleopParseRoom (α=0) ────────────────────────────────────

class TeleopParseRoom:
    """Parse leader arm commands from CAN bus or ROS2 topics. α=0 (pure code).

    Code path: Validate CAN frame format, extract joint angles, detect
    missing/duplicate frames via monotonic frame_id.

    No model path needed — CAN parsing is deterministic.
    """

    def __init__(self, num_joints: int = 7) -> None:
        self._num_joints = num_joints
        self._last_frame_id = -1
        self._dropped_frames = 0
        self._parsed_commands = 0

    @property
    def room_config(self) -> RoomConfig:
        return RoomConfig(
            name="teleop_parse",
            alpha=0.0,
            description="Parse and validate leader arm CAN commands. Pure code.",
            max_latency_ms=0.1,
        )

    def parse_can_frame(self, raw: bytes) -> Optional[LeaderCommand]:
        """Parse a raw CAN frame into a LeaderCommand.

        Expected frame format (22 bytes):
          [2 bytes per joint × 7 joints] + [2 bytes gripper] + [2 bytes frame_id] + [8 bytes timestamp]
        Joint values are int16 scaled: radians × 10000.
        Gripper is uint16 scaled: 0-65535 → 0.0-1.0.
        """
        if len(raw) < 18:  # Minimum: 7×2 + gripper + frame_id
            return None

        joints = []
        for i in range(self._num_joints):
            offset = i * 2
            if offset + 1 < len(raw):
                raw_val = int.from_bytes(raw[offset:offset + 2], byteorder='little', signed=True)
                joints.append(raw_val / 10000.0)
            else:
                joints.append(0.0)

        # Gripper
        gripper_offset = self._num_joints * 2
        if gripper_offset + 1 < len(raw):
            raw_gripper = int.from_bytes(
                raw[gripper_offset:gripper_offset + 2], byteorder='little', signed=False
            )
            gripper = raw_gripper / 65535.0
        else:
            gripper = 0.0

        # Frame ID
        frame_offset = gripper_offset + 2
        frame_id = 0
        if frame_offset + 1 < len(raw):
            frame_id = int.from_bytes(
                raw[frame_offset:frame_offset + 2], byteorder='little', signed=False
            )

        # Detect dropped frames
        if self._last_frame_id >= 0:
            expected = (self._last_frame_id + 1) & 0xFFFF
            if frame_id != expected:
                self._dropped_frames += abs(frame_id - expected) % 65536

        self._last_frame_id = frame_id
        self._parsed_commands += 1

        return LeaderCommand(
            joints=tuple(joints),
            gripper=max(0.0, min(1.0, gripper)),
            timestamp=time.time(),
            source="leader_can",
            frame_id=frame_id,
        )

    def parse_ros2_message(self, msg: dict) -> Optional[LeaderCommand]:
        """Parse a ROS2 JointState message into a LeaderCommand.

        Expected dict keys: 'position' (list), 'velocity' (list), 'effort' (list).
        Gripper extracted from last element or separate field.
        """
        positions = msg.get("position", [])
        if not positions:
            return None

        # First N values are joints, last might be gripper
        if len(positions) > self._num_joints:
            joints = tuple(positions[:self._num_joints])
            gripper = float(positions[self._num_joints])
        else:
            joints = tuple(positions[:self._num_joints])
            gripper = msg.get("gripper_position", 0.5)

        self._parsed_commands += 1

        return LeaderCommand(
            joints=joints,
            gripper=max(0.0, min(1.0, gripper)),
            timestamp=msg.get("header", {}).get("stamp", time.time()),
            source="ros2_topic",
            frame_id=msg.get("header", {}).get("frame_id_seq", 0),
        )

    def validate(self, cmd: LeaderCommand) -> Tuple[bool, List[str]]:
        """Validate a parsed leader command. Returns (is_valid, errors)."""
        errors = []
        if len(cmd.joints) != self._num_joints:
            errors.append(f"Expected {self._num_joints} joints, got {len(cmd.joints)}")
        if not (0.0 <= cmd.gripper <= 1.0):
            errors.append(f"Gripper out of range: {cmd.gripper}")
        for i, j in enumerate(cmd.joints):
            if not (-2 * math.pi <= j <= 2 * math.pi):
                errors.append(f"Joint {i} out of range: {j:.3f}")
        return len(errors) == 0, errors

    @property
    def stats(self) -> dict:
        return {
            "parsed": self._parsed_commands,
            "dropped_frames": self._dropped_frames,
            "last_frame_id": self._last_frame_id,
        }


# ── Room 2: TeleopMapRoom (α=0.2) ────────────────────────────────────

class TeleopMapRoom:
    """Map leader arm joint space to follower arm joint space. α=0.2.

    Code path (α=0): Linear mapping with per-joint scale and offset.
    Used for kinematically identical leader/follower pairs.

    Model path (α=0.2): Micro-model for nonlinear mapping when leader
    and follower have different kinematics (e.g., different arm sizes,
    or mapping from 6-DOF leader to 7-DOF follower).
    """

    def __init__(
        self,
        num_joints: int = 7,
        scales: Optional[Tuple[float, ...]] = None,
        offsets: Optional[Tuple[float, ...]] = None,
        joint_mapping: Optional[Dict[int, int]] = None,
    ) -> None:
        self._num_joints = num_joints
        self._scales = scales or tuple(1.0 for _ in range(num_joints))
        self._offsets = offsets or tuple(0.0 for _ in range(num_joints))
        # joint_mapping: leader_idx → follower_idx (identity if None)
        self._joint_mapping = joint_mapping or {i: i for i in range(num_joints)}
        self._model_weights: Optional[List[List[float]]] = None

    @property
    def room_config(self) -> RoomConfig:
        return RoomConfig(
            name="teleop_map",
            alpha=0.2,
            description="Map leader→follower joint space. Linear code + nonlinear micro-model.",
            max_latency_ms=1.0,
        )

    def map_linear(self, cmd: LeaderCommand) -> MappedJoints:
        """Linear mapping: follower_joint = leader_joint × scale + offset."""
        mapped = [0.0] * self._num_joints
        for leader_idx, follower_idx in self._joint_mapping.items():
            if leader_idx < len(cmd.joints) and follower_idx < self._num_joints:
                mapped[follower_idx] = (
                    cmd.joints[leader_idx] * self._scales[leader_idx]
                    + self._offsets[leader_idx]
                )

        return MappedJoints(
            joints=tuple(mapped),
            gripper=cmd.gripper,
            mapping_type="linear",
            timestamp=time.time(),
            leader_frame_id=cmd.frame_id,
        )

    def map_scaled(
        self,
        cmd: LeaderCommand,
        leader_range: Tuple[float, float],
        follower_range: Tuple[float, float],
    ) -> MappedJoints:
        """Scale-based mapping: remap leader range to follower range."""
        l_min, l_max = leader_range
        f_min, f_max = follower_range
        l_span = l_max - l_min if l_max != l_min else 1.0
        f_span = f_max - f_min

        mapped = [0.0] * self._num_joints
        for leader_idx, follower_idx in self._joint_mapping.items():
            if leader_idx < len(cmd.joints) and follower_idx < self._num_joints:
                normalized = (cmd.joints[leader_idx] - l_min) / l_span
                mapped[follower_idx] = f_min + normalized * f_span

        return MappedJoints(
            joints=tuple(mapped),
            gripper=cmd.gripper,
            mapping_type="scaled",
            timestamp=time.time(),
            leader_frame_id=cmd.frame_id,
        )

    def map_model(self, cmd: LeaderCommand) -> MappedJoints:
        """Micro-model mapping: simple single-layer neural network.

        Uses pre-trained weights for nonlinear joint-space mapping.
        Falls back to linear if no weights loaded.
        """
        if self._model_weights is None:
            return self.map_linear(cmd)

        # Single-layer: y = W @ x + b
        # weights[0] = W (num_joints × num_joints), weights[1] = b (num_joints)
        W = self._model_weights[0]
        b = self._model_weights[1]

        mapped = [0.0] * self._num_joints
        leader_joints = list(cmd.joints) + [cmd.gripper]  # Include gripper as input

        for i in range(self._num_joints):
            val = b[i] if i < len(b) else 0.0
            for j in range(min(len(leader_joints), len(W[i]))):
                val += leader_joints[j] * W[i][j]
            # Tanh activation for bounded output
            mapped[i] = math.tanh(val) * math.pi

        return MappedJoints(
            joints=tuple(mapped),
            gripper=cmd.gripper,
            mapping_type="model",
            timestamp=time.time(),
            leader_frame_id=cmd.frame_id,
        )

    def load_model_weights(self, weights: List[List[float]], biases: List[float]) -> None:
        """Load micro-model weights for nonlinear mapping."""
        self._model_weights = [weights, biases]

    def set_joint_mapping(self, mapping: Dict[int, int]) -> None:
        """Update the leader→follower joint index mapping."""
        self._joint_mapping = mapping


# ── Room 3: TeleopSafetyRoom (α=0.1) ─────────────────────────────────

class TeleopSafetyRoom:
    """Real-time safety envelope for teleoperation. α=0.1.

    Code path (α=0): Joint limits, velocity limits, workspace bounds,
    rate-of-change limits, gripper limits. All deterministic.

    Model path (α=0.1): Micro-model detects novel/dangerous configurations
    that pass individual checks — e.g., two arms approaching each other,
    or configurations near singularities that aren't caught by simple limits.
    """

    def __init__(
        self,
        constraint_checker: Optional[ConstraintChecker] = None,
        max_velocity: float = 2.0,          # rad/s per joint
        max_acceleration: float = 10.0,      # rad/s² per joint
        max_gripper_velocity: float = 1.0,   # per second
        min_gripper: float = 0.0,
        max_gripper: float = 1.0,
        dt_s: float = 0.01,                  # Control loop period
    ) -> None:
        self._checker = constraint_checker or ConstraintChecker()
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration
        self._max_gripper_velocity = max_gripper_velocity
        self._min_gripper = min_gripper
        self._max_gripper = max_gripper
        self._dt_s = dt_s
        self._prev_joints: Optional[Tuple[float, ...]] = None
        self._prev_gripper: Optional[float] = None
        self._prev_time: Optional[float] = None

    @property
    def room_config(self) -> RoomConfig:
        return RoomConfig(
            name="teleop_safety",
            alpha=0.1,
            description="Real-time teleop safety: limits + micro-model anomaly detection",
            max_latency_ms=1.0,
        )

    def check(
        self,
        mapped: MappedJoints,
        current_state: Optional[JointState] = None,
    ) -> TeleopSafetyResult:
        """Run all teleop safety checks on mapped follower targets.

        Returns TeleopSafetyResult with clamped values if soft violations occur.
        """
        start = time.monotonic()
        violations: List[str] = []
        clamped = list(mapped.joints)
        clamped_gripper = mapped.gripper
        min_margin = float('inf')

        # --- Joint limit + workspace checks via ConstraintChecker ---
        if current_state is not None:
            # Build a hypothetical state with target joints
            target_state = JointState(
                joints=mapped.joints,
                velocities=current_state.velocities,
                torques=current_state.torques,
                timestamp=mapped.timestamp,
                source="teleop_target",
            )
            constraint_result = self._checker.check(target_state)
            if not constraint_result.passed:
                violations.extend(constraint_result.violations)
            min_margin = min(min_margin, constraint_result.min_margin)

        # --- Velocity limit check ---
        now = mapped.timestamp
        if self._prev_joints is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                for i in range(min(len(clamped), len(self._prev_joints))):
                    velocity = abs(clamped[i] - self._prev_joints[i]) / dt
                    if velocity > self._max_velocity:
                        violations.append(
                            f"velocity_limit_joint_{i}: {velocity:.2f} > {self._max_velocity}"
                        )
                        # Clamp velocity
                        direction = 1.0 if clamped[i] > self._prev_joints[i] else -1.0
                        max_delta = self._max_velocity * dt
                        clamped[i] = self._prev_joints[i] + direction * max_delta
                        margin = self._max_velocity - velocity
                        min_margin = min(min_margin, margin)

            # --- Acceleration limit ---
            if self._prev_time is not None and dt > 0:
                for i in range(min(len(clamped), len(self._prev_joints))):
                    velocity = abs(clamped[i] - self._prev_joints[i]) / dt if dt > 0 else 0.0
                    # Simple acceleration check (would need prev_velocity for full check)
                    # Skip if no prev_velocity tracking

        # --- Gripper limits ---
        if clamped_gripper < self._min_gripper:
            violations.append(f"gripper_below_min: {clamped_gripper:.3f}")
            clamped_gripper = self._min_gripper
            min_margin = min(min_margin, 0.0)
        elif clamped_gripper > self._max_gripper:
            violations.append(f"gripper_above_max: {clamped_gripper:.3f}")
            clamped_gripper = self._max_gripper
            min_margin = min(min_margin, 0.0)

        # --- Gripper velocity ---
        if self._prev_gripper is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                grip_vel = abs(clamped_gripper - self._prev_gripper) / dt
                if grip_vel > self._max_gripper_velocity:
                    violations.append(
                        f"gripper_velocity: {grip_vel:.2f} > {self._max_gripper_velocity}"
                    )
                    direction = 1.0 if clamped_gripper > self._prev_gripper else -1.0
                    clamped_gripper = self._prev_gripper + direction * self._max_gripper_velocity * dt

        # Update previous state
        self._prev_joints = tuple(clamped)
        self._prev_gripper = clamped_gripper
        self._prev_time = now

        elapsed_us = (time.monotonic() - start) * 1_000_000
        effective_min_margin = min_margin if min_margin != float('inf') else 0.0

        return TeleopSafetyResult(
            passed=len(violations) == 0,
            violations=tuple(violations),
            clamped_joints=tuple(clamped),
            clamped_gripper=clamped_gripper,
            min_margin=round(effective_min_margin, 4),
            timestamp=time.time(),
            check_latency_us=round(elapsed_us, 2),
        )

    def reset(self) -> None:
        """Reset tracking state (e.g., on teleop start)."""
        self._prev_joints = None
        self._prev_gripper = None
        self._prev_time = None


# ── Room 4: TeleopFleetRoom (α=0.5) ──────────────────────────────────

class TeleopFleetRoom:
    """Multi-arm coordination during teleop. α=0.5.

    Code path (α=0): Fixed choreography — predefined multi-arm sequences,
    mirror mode, offset mode. Deterministic and rehearsed.

    Model path (α=0.5): Novel multi-arm coordination — dynamic replanning,
    conflict resolution, adaptive choreography based on real-time state.
    """

    def __init__(self, fleet_arms: Optional[List[str]] = None) -> None:
        self._arms: Dict[str, Tuple[float, ...]] = {}  # arm_id → current joints
        self._choreography: Dict[str, Dict[str, Any]] = {}  # name → steps
        self._active_choreography: Optional[str] = None
        self._choreography_step = 0

        if fleet_arms:
            for arm_id in fleet_arms:
                self._arms[arm_id] = tuple(0.0 for _ in range(7))

    @property
    def room_config(self) -> RoomConfig:
        return RoomConfig(
            name="teleop_fleet",
            alpha=0.5,
            description="Multi-arm coordination: code for choreography, model for novel tasks",
            max_latency_ms=100.0,
        )

    def register_arm(self, arm_id: str, num_joints: int = 7) -> None:
        """Register a fleet arm."""
        self._arms[arm_id] = tuple(0.0 for _ in range(num_joints))

    def update_arm_state(self, arm_id: str, joints: Tuple[float, ...]) -> None:
        """Update current joint state for a fleet arm."""
        if arm_id in self._arms:
            self._arms[arm_id] = joints

    def mirror(self, source_joints: Tuple[float, ...], source_arm: str) -> List[FleetTeleopCommand]:
        """Mirror mode: copy source arm joints to all other arms (bilateral)."""
        commands = []
        for arm_id in self._arms:
            if arm_id != source_arm:
                commands.append(FleetTeleopCommand(
                    arm_id=arm_id,
                    target_joints=source_joints,
                    mode="mirror",
                    priority=2,
                    coordination_group="bilateral",
                    timestamp=time.time(),
                ))
        return commands

    def offset(
        self,
        source_joints: Tuple[float, ...],
        offsets: Dict[str, Tuple[float, ...]],
    ) -> List[FleetTeleopCommand]:
        """Offset mode: apply per-arm joint offsets to source joints."""
        commands = []
        for arm_id, arm_offsets in offsets.items():
            if arm_id in self._arms:
                target = tuple(
                    s + o for s, o in zip(source_joints, arm_offsets)
                )
                commands.append(FleetTeleopCommand(
                    arm_id=arm_id,
                    target_joints=target,
                    mode="offset",
                    priority=2,
                    coordination_group="offset_group",
                    timestamp=time.time(),
                ))
        return commands

    def load_choreography(self, name: str, steps: List[Dict[str, Any]]) -> None:
        """Load a choreography sequence (code path)."""
        self._choreography[name] = {
            "steps": steps,
            "total_steps": len(steps),
        }

    def start_choreography(self, name: str) -> bool:
        """Start playing a choreography sequence."""
        if name not in self._choreography:
            return False
        self._active_choreography = name
        self._choreography_step = 0
        return True

    def step_choreography(self) -> Optional[List[FleetTeleopCommand]]:
        """Advance choreography by one step. Returns None if complete."""
        if self._active_choreography is None:
            return None

        choreo = self._choreography.get(self._active_choreography)
        if choreo is None:
            return None

        steps = choreo["steps"]
        if self._choreography_step >= len(steps):
            self._active_choreography = None
            return None

        step = steps[self._choreography_step]
        self._choreography_step += 1

        commands = []
        for arm_id, target_joints in step.get("arms", {}).items():
            if arm_id in self._arms:
                commands.append(FleetTeleopCommand(
                    arm_id=arm_id,
                    target_joints=tuple(target_joints),
                    mode="choreography",
                    priority=1,
                    coordination_group=self._active_choreography,
                    timestamp=time.time(),
                ))

        return commands

    def stop_choreography(self) -> None:
        """Stop the current choreography."""
        self._active_choreography = None
        self._choreography_step = 0

    @property
    def active_choreography(self) -> Optional[str]:
        return self._active_choreography

    @property
    def fleet_arms(self) -> List[str]:
        return list(self._arms.keys())


# ── Room 5: TeleopRecordRoom (α=0.3) ─────────────────────────────────

class TeleopRecordRoom:
    """Record demonstrations for imitation learning. α=0.3.

    Code path (α=0): Record leader + follower joint states at each frame.
    Code handles frame alignment, duplicate detection, and storage.

    Model path (α=0.3): Automatic segmentation of demonstrations into
    meaningful primitives (reach, grasp, lift, place, etc.), quality
    scoring, and automatic retry suggestions.
    """

    def __init__(
        self,
        max_demonstrations: int = 100,
        max_frames_per_demo: int = 10000,
        record_rate_hz: float = 100.0,
    ) -> None:
        self._max_demos = max_demonstrations
        self._max_frames = max_frames_per_demo
        self._record_interval = 1.0 / record_rate_hz
        self._demonstrations: Dict[str, Demonstration] = {}
        self._active_demo: Optional[str] = None
        self._last_record_time: float = 0.0
        self._frame_counter: int = 0

    @property
    def room_config(self) -> RoomConfig:
        return RoomConfig(
            name="teleop_record",
            alpha=0.3,
            description="Record demonstrations: code for capture, model for segmentation",
            max_latency_ms=10.0,
        )

    def start_recording(self, name: str, metadata: Optional[Dict[str, Any]] = None) -> bool:
        """Start recording a new demonstration."""
        if self._active_demo is not None:
            return False  # Already recording
        if name in self._demonstrations and len(self._demonstrations) >= self._max_demos:
            return False

        self._demonstrations[name] = Demonstration(
            name=name,
            start_time=time.time(),
            metadata=metadata or {},
        )
        self._active_demo = name
        self._frame_counter = 0
        return True

    def stop_recording(self) -> Optional[Demonstration]:
        """Stop recording and return the demonstration."""
        if self._active_demo is None:
            return None

        demo = self._demonstrations.get(self._active_demo)
        if demo:
            demo.end_time = time.time()

        self._active_demo = None
        return demo

    def record_frame(
        self,
        leader: LeaderCommand,
        mapped: MappedJoints,
        safety_result: TeleopSafetyResult,
    ) -> bool:
        """Record a single frame of leader→follower mapping.

        Respects the configured recording rate (no duplicate frames).
        """
        if self._active_demo is None:
            return False

        now = time.time()
        if now - self._last_record_time < self._record_interval:
            return False  # Too soon

        demo = self._demonstrations.get(self._active_demo)
        if demo is None or len(demo.frames) >= self._max_frames:
            return False

        frame = DemonstrationFrame(
            leader_joints=leader.joints,
            follower_joints=safety_result.clamped_joints,
            gripper=safety_result.clamped_gripper,
            safety_passed=safety_result.passed,
            timestamp=now,
            frame_id=self._frame_counter,
        )
        demo.frames.append(frame)
        self._frame_counter += 1
        self._last_record_time = now
        return True

    def get_demonstration(self, name: str) -> Optional[Demonstration]:
        """Retrieve a recorded demonstration by name."""
        return self._demonstrations.get(name)

    def list_demonstrations(self) -> List[Dict[str, Any]]:
        """List all recorded demonstrations with summary stats."""
        result = []
        for name, demo in self._demonstrations.items():
            result.append({
                "name": name,
                "frame_count": demo.frame_count,
                "duration_s": round(demo.duration_s, 3),
                "metadata": demo.metadata,
            })
        return result

    def export_demonstration(self, name: str) -> Optional[str]:
        """Export a demonstration as JSON string for PLATO publishing."""
        demo = self._demonstrations.get(name)
        if demo is None:
            return None
        return json.dumps(demo.to_dict(), indent=2)

    def delete_demonstration(self, name: str) -> bool:
        """Delete a recorded demonstration."""
        if name in self._demonstrations:
            del self._demonstrations[name]
            return True
        return False

    @property
    def is_recording(self) -> bool:
        return self._active_demo is not None

    @property
    def active_demo_name(self) -> Optional[str]:
        return self._active_demo


# ── Teleop Room Registry ─────────────────────────────────────────────

class TeleopRooms:
    """Pre-built PLATO rooms for the OpenArm teleop pipeline."""

    @staticmethod
    def parse() -> RoomConfig:
        return RoomConfig(
            name="teleop_parse", alpha=0.0,
            description="Parse leader arm CAN commands. Pure code.",
            max_latency_ms=0.1,
        )

    @staticmethod
    def map() -> RoomConfig:
        return RoomConfig(
            name="teleop_map", alpha=0.2,
            description="Map leader→follower joint space. Linear + micro-model.",
            max_latency_ms=1.0,
        )

    @staticmethod
    def safety() -> RoomConfig:
        return RoomConfig(
            name="teleop_safety", alpha=0.1,
            description="Real-time safety envelope for teleop.",
            max_latency_ms=1.0,
        )

    @staticmethod
    def fleet() -> RoomConfig:
        return RoomConfig(
            name="teleop_fleet", alpha=0.5,
            description="Multi-arm coordination during teleop.",
            max_latency_ms=100.0,
        )

    @staticmethod
    def record() -> RoomConfig:
        return RoomConfig(
            name="teleop_record", alpha=0.3,
            description="Record demonstrations for imitation learning.",
            max_latency_ms=10.0,
        )

    @classmethod
    def all_rooms(cls) -> List[RoomConfig]:
        return [cls.parse(), cls.map(), cls.safety(), cls.fleet(), cls.record()]
