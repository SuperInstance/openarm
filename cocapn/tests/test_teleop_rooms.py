"""Tests for PLATO teleop rooms — OpenArm teleoperation pipeline.

Covers all 5 rooms: Parse, Map, Safety, Fleet, Record.
At least 15 tests covering code paths, edge cases, and integration.
"""

import math
import time
import pytest

from teleop_rooms import (
    LeaderCommand,
    MappedJoints,
    TeleopSafetyResult,
    FleetTeleopCommand,
    DemonstrationFrame,
    Demonstration,
    TeleopParseRoom,
    TeleopMapRoom,
    TeleopSafetyRoom,
    TeleopFleetRoom,
    TeleopRecordRoom,
    TeleopRooms,
)
from plato_rooms import JointState, ConstraintChecker, RoomConfig


# ── Helpers ───────────────────────────────────────────────────────────

def make_leader(joints=None, gripper=0.5, frame_id=1):
    """Create a test LeaderCommand."""
    if joints is None:
        joints = tuple(0.0 for _ in range(7))
    return LeaderCommand(
        joints=joints,
        gripper=gripper,
        timestamp=time.time(),
        source="test",
        frame_id=frame_id,
    )


def make_mapped(joints=None, gripper=0.5, frame_id=1):
    """Create a test MappedJoints."""
    if joints is None:
        joints = tuple(0.0 for _ in range(7))
    return MappedJoints(
        joints=joints,
        gripper=gripper,
        mapping_type="linear",
        timestamp=time.time(),
        leader_frame_id=frame_id,
    )


def make_can_frame(joints=None, gripper=0.5, frame_id=1):
    """Build a fake CAN frame (22 bytes: 7×2 joints + 2 gripper + 2 frame_id + 4 timestamp)."""
    if joints is None:
        joints = [0.0] * 7
    raw = bytearray()
    for j in joints:
        raw.extend(int(j * 10000).to_bytes(2, 'little', signed=True))
    raw.extend(int(gripper * 65535).to_bytes(2, 'little', signed=False))
    raw.extend(frame_id.to_bytes(2, 'little', signed=False))
    raw.extend(int(time.time()).to_bytes(4, 'little', signed=False))
    return bytes(raw)


# ── Room 1: TeleopParseRoom Tests ────────────────────────────────────

class TestTeleopParseRoom:

    def test_parse_can_frame_valid(self):
        """Parse a well-formed CAN frame into LeaderCommand."""
        room = TeleopParseRoom(num_joints=7)
        joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        raw = make_can_frame(joints=joints, gripper=0.75, frame_id=42)
        cmd = room.parse_can_frame(raw)

        assert cmd is not None
        assert len(cmd.joints) == 7
        for i, expected in enumerate(joints):
            assert abs(cmd.joints[i] - expected) < 0.001, f"Joint {i}: {cmd.joints[i]} vs {expected}"
        assert abs(cmd.gripper - 0.75) < 0.01
        assert cmd.frame_id == 42
        assert cmd.source == "leader_can"

    def test_parse_can_frame_too_short(self):
        """Reject CAN frames that are too short."""
        room = TeleopParseRoom(num_joints=7)
        cmd = room.parse_can_frame(b'\x00\x01\x02')
        assert cmd is None

    def test_parse_ros2_message(self):
        """Parse a ROS2-style dict message."""
        room = TeleopParseRoom(num_joints=7)
        msg = {
            "position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
            "header": {"stamp": 1000.0, "frame_id_seq": 5},
        }
        cmd = room.parse_ros2_message(msg)
        assert cmd is not None
        assert len(cmd.joints) == 7
        assert abs(cmd.gripper - 0.8) < 0.01
        assert cmd.source == "ros2_topic"

    def test_validate_good_command(self):
        """Valid leader command passes validation."""
        room = TeleopParseRoom(num_joints=7)
        cmd = make_leader(joints=tuple(0.5 * i for i in range(7)), gripper=0.5)
        ok, errors = room.validate(cmd)
        assert ok
        assert errors == []

    def test_validate_bad_joint_range(self):
        """Leader command with out-of-range joints fails validation."""
        room = TeleopParseRoom(num_joints=7)
        bad_joints = [0.0] * 7
        bad_joints[3] = 99.0
        cmd = make_leader(joints=tuple(bad_joints))
        ok, errors = room.validate(cmd)
        assert not ok
        assert any("Joint 3" in e for e in errors)

    def test_frame_drop_detection(self):
        """Detect dropped frames via non-monotonic frame_id."""
        room = TeleopParseRoom(num_joints=7)
        room.parse_can_frame(make_can_frame(frame_id=1))
        room.parse_can_frame(make_can_frame(frame_id=2))
        room.parse_can_frame(make_can_frame(frame_id=5))  # Dropped 3,4
        assert room.stats["dropped_frames"] == 2  # Frames 3 and 4 = 2 dropped


# ── Room 2: TeleopMapRoom Tests ──────────────────────────────────────

class TestTeleopMapRoom:

    def test_linear_mapping_identity(self):
        """Identity mapping: scales=1.0, offsets=0.0."""
        room = TeleopMapRoom(num_joints=7)
        leader = make_leader(joints=tuple(0.5 * i for i in range(7)))
        mapped = room.map_linear(leader)
        assert mapped.mapping_type == "linear"
        for i in range(7):
            assert abs(mapped.joints[i] - 0.5 * i) < 1e-6

    def test_linear_mapping_with_scale_offset(self):
        """Linear mapping with per-joint scale and offset."""
        scales = tuple(2.0 for _ in range(7))
        offsets = tuple(-0.5 for _ in range(7))
        room = TeleopMapRoom(num_joints=7, scales=scales, offsets=offsets)
        leader = make_leader(joints=tuple(1.0 for _ in range(7)))
        mapped = room.map_linear(leader)
        for i in range(7):
            assert abs(mapped.joints[i] - 1.5) < 1e-6  # 1.0*2.0 - 0.5

    def test_scaled_mapping(self):
        """Scale-based mapping remaps leader range to follower range."""
        room = TeleopMapRoom(num_joints=7)
        leader = make_leader(joints=tuple(0.0 for _ in range(7)))
        mapped = room.map_scaled(
            leader,
            leader_range=(-math.pi, math.pi),
            follower_range=(0.0, 1.0),
        )
        # 0.0 in [-π, π] → 0.5 in [0.0, 1.0]
        assert mapped.mapping_type == "scaled"
        for i in range(7):
            assert abs(mapped.joints[i] - 0.5) < 1e-6

    def test_model_mapping_fallback_to_linear(self):
        """Model mapping falls back to linear when no weights loaded."""
        room = TeleopMapRoom(num_joints=7)
        leader = make_leader(joints=tuple(0.3 for _ in range(7)))
        mapped = room.map_model(leader)
        assert mapped.mapping_type == "linear"  # Fallback

    def test_model_mapping_with_weights(self):
        """Model mapping uses loaded weights."""
        room = TeleopMapRoom(num_joints=7)
        # Single-layer weights: 7×8 (7 joints + gripper input, 7 outputs)
        weights = [[0.1] * 8 for _ in range(7)]
        biases = [0.0] * 7
        room.load_model_weights(weights, biases)
        leader = make_leader(joints=tuple(1.0 for _ in range(7)))
        mapped = room.map_model(leader)
        assert mapped.mapping_type == "model"
        # With these weights, output = tanh(sum(inputs * 0.1)) * π
        # Each output ≈ tanh(0.8) * π ≈ 0.6640 * 3.14159 ≈ 2.086
        for i in range(7):
            assert -math.pi <= mapped.joints[i] <= math.pi

    def test_custom_joint_mapping(self):
        """Custom leader→follower joint mapping."""
        mapping = {0: 6, 1: 5, 2: 4, 3: 3, 4: 2, 5: 1, 6: 0}  # Reverse
        room = TeleopMapRoom(num_joints=7, joint_mapping=mapping)
        leader = make_leader(joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))
        mapped = room.map_linear(leader)
        assert abs(mapped.joints[0] - 7.0) < 1e-6  # Leader 6 → Follower 0
        assert abs(mapped.joints[6] - 1.0) < 1e-6  # Leader 0 → Follower 6


# ── Room 3: TeleopSafetyRoom Tests ───────────────────────────────────

class TestTeleopSafetyRoom:

    def test_safe_command_passes(self):
        """Safe mapped joints pass all checks."""
        room = TeleopSafetyRoom(max_velocity=10.0)
        mapped = make_mapped(joints=tuple(0.1 * i for i in range(7)))
        result = room.check(mapped)
        assert result.passed
        assert len(result.violations) == 0

    def test_gripper_limit_clamped(self):
        """Gripper values outside [0, 1] are clamped."""
        room = TeleopSafetyRoom()
        mapped = make_mapped(gripper=1.5)
        result = room.check(mapped)
        assert not result.passed
        assert result.clamped_gripper == 1.0
        assert any("gripper_above_max" in v for v in result.violations)

    def test_velocity_limit_triggers(self):
        """Rapid joint movement triggers velocity limit."""
        room = TeleopSafetyRoom(max_velocity=0.5)
        # First command (establishes prev state)
        mapped1 = make_mapped(joints=tuple(0.0 for _ in range(7)))
        room.check(mapped1)

        # Second command with large jump (same timestamp range → high velocity)
        mapped2 = make_mapped(joints=tuple(1.0 for _ in range(7)))
        mapped2 = MappedJoints(
            joints=tuple(1.0 for _ in range(7)),
            gripper=0.5,
            mapping_type="linear",
            timestamp=mapped1.timestamp + 0.01,  # 10ms later
        )
        result = room.check(mapped2)
        assert not result.passed
        assert any("velocity_limit" in v for v in result.violations)

    def test_clamped_joints_preserve_safe_values(self):
        """Clamped joints should be within velocity bounds from previous."""
        room = TeleopSafetyRoom(max_velocity=1.0)
        mapped1 = make_mapped(joints=tuple(0.0 for _ in range(7)))
        room.check(mapped1)

        mapped2 = MappedJoints(
            joints=tuple(5.0 for _ in range(7)),
            gripper=0.5,
            mapping_type="linear",
            timestamp=mapped1.timestamp + 0.01,
        )
        result = room.check(mapped2)
        # Clamped joints should be near prev + max_velocity * dt
        for j in result.clamped_joints:
            assert abs(j) <= 1.0 * 0.01 + 0.01  # max_vel * dt + epsilon

    def test_reset_clears_previous_state(self):
        """Reset clears velocity tracking."""
        room = TeleopSafetyRoom(max_velocity=0.5)
        mapped = make_mapped(joints=tuple(0.0 for _ in range(7)))
        room.check(mapped)
        room.reset()

        # After reset, large jump should be fine (no prev state)
        mapped2 = make_mapped(joints=tuple(5.0 for _ in range(7)))
        result = room.check(mapped2)
        assert result.passed

    def test_safety_latency_is_fast(self):
        """Safety check completes in < 1ms (room requirement)."""
        room = TeleopSafetyRoom()
        mapped = make_mapped()
        result = room.check(mapped)
        assert result.check_latency_us < 1000  # < 1ms


# ── Room 4: TeleopFleetRoom Tests ────────────────────────────────────

class TestTeleopFleetRoom:

    def test_mirror_mode(self):
        """Mirror mode copies joints to all other arms."""
        room = TeleopFleetRoom(fleet_arms=["arm_left", "arm_right", "arm_center"])
        source = tuple(0.5 * i for i in range(7))
        commands = room.mirror(source, "arm_left")

        assert len(commands) == 2
        arm_ids = {c.arm_id for c in commands}
        assert "arm_left" not in arm_ids
        assert "arm_right" in arm_ids
        assert "arm_center" in arm_ids
        for c in commands:
            assert c.mode == "mirror"
            assert c.target_joints == source

    def test_offset_mode(self):
        """Offset mode applies per-arm offsets."""
        room = TeleopFleetRoom(fleet_arms=["arm_A", "arm_B"])
        source = tuple(1.0 for _ in range(7))
        offsets = {
            "arm_A": tuple(0.5 for _ in range(7)),
            "arm_B": tuple(-0.5 for _ in range(7)),
        }
        commands = room.offset(source, offsets)

        assert len(commands) == 2
        for c in commands:
            if c.arm_id == "arm_A":
                assert all(abs(j - 1.5) < 1e-6 for j in c.target_joints)
            elif c.arm_id == "arm_B":
                assert all(abs(j - 0.5) < 1e-6 for j in c.target_joints)

    def test_choreography_playback(self):
        """Choreography plays predefined steps in order."""
        room = TeleopFleetRoom(fleet_arms=["arm_L", "arm_R"])
        room.load_choreography("wave", [
            {"arms": {"arm_L": [0.0]*7, "arm_R": [0.5]*7}},
            {"arms": {"arm_L": [0.5]*7, "arm_R": [0.0]*7}},
            {"arms": {"arm_L": [0.0]*7, "arm_R": [0.0]*7}},
        ])

        assert room.start_choreography("wave")
        assert room.active_choreography == "wave"

        step1 = room.step_choreography()
        assert len(step1) == 2
        assert step1[0].mode == "choreography"

        step2 = room.step_choreography()
        assert len(step2) == 2

        step3 = room.step_choreography()
        assert len(step3) == 2

        # Choreography complete
        step4 = room.step_choreography()
        assert step4 is None
        assert room.active_choreography is None

    def test_stop_choreography(self):
        """Stopping choreography resets state."""
        room = TeleopFleetRoom(fleet_arms=["arm_1"])
        room.load_choreography("test", [{"arms": {"arm_1": [0.0]*7}}])
        room.start_choreography("test")
        assert room.active_choreography == "test"
        room.stop_choreography()
        assert room.active_choreography is None

    def test_unknown_choreography(self):
        """Starting an unknown choreography fails."""
        room = TeleopFleetRoom(fleet_arms=["arm_1"])
        assert not room.start_choreography("nonexistent")


# ── Room 5: TeleopRecordRoom Tests ───────────────────────────────────

class TestTeleopRecordRoom:

    def test_start_stop_recording(self):
        """Start and stop recording produces a demonstration."""
        room = TeleopRecordRoom(record_rate_hz=10000.0)  # Very fast for testing
        assert room.start_recording("test_demo", metadata={"task": "pick"})
        assert room.is_recording
        assert room.active_demo_name == "test_demo"

        demo = room.stop_recording()
        assert demo is not None
        assert demo.name == "test_demo"
        assert not room.is_recording

    def test_record_frames(self):
        """Record frames during a demonstration."""
        room = TeleopRecordRoom(record_rate_hz=10000.0)
        room.start_recording("frame_test")

        leader = make_leader(joints=tuple(0.1 * i for i in range(7)))
        mapped = make_mapped(joints=tuple(0.2 * i for i in range(7)))
        safety = TeleopSafetyResult(
            passed=True, violations=(),
            clamped_joints=tuple(0.2 * i for i in range(7)),
            clamped_gripper=0.5, min_margin=0.5,
            timestamp=time.time(), check_latency_us=10.0,
        )

        # Record a few frames
        for _ in range(5):
            time.sleep(0.001)  # Ensure different timestamps
            safety = TeleopSafetyResult(
                passed=True, violations=(),
                clamped_joints=tuple(0.2 * i for i in range(7)),
                clamped_gripper=0.5, min_margin=0.5,
                timestamp=time.time(), check_latency_us=10.0,
            )
            room.record_frame(leader, mapped, safety)

        demo = room.stop_recording()
        assert demo is not None
        assert demo.frame_count >= 1
        assert demo.duration_s > 0

    def test_cannot_double_record(self):
        """Cannot start a second recording while one is active."""
        room = TeleopRecordRoom()
        room.start_recording("demo1")
        assert not room.start_recording("demo2")
        room.stop_recording()

    def test_export_as_json(self):
        """Export demonstration as valid JSON."""
        room = TeleopRecordRoom(record_rate_hz=10000.0)
        room.start_recording("export_test")

        leader = make_leader()
        mapped = make_mapped()
        safety = TeleopSafetyResult(
            passed=True, violations=(),
            clamped_joints=tuple(0.0 for _ in range(7)),
            clamped_gripper=0.5, min_margin=1.0,
            timestamp=time.time(), check_latency_us=5.0,
        )
        room.record_frame(leader, mapped, safety)
        room.stop_recording()

        json_str = room.export_demonstration("export_test")
        assert json_str is not None
        parsed = json.loads(json_str)
        assert parsed["name"] == "export_test"
        assert parsed["frame_count"] >= 1

    def test_list_and_delete_demonstrations(self):
        """List and delete demonstrations."""
        room = TeleopRecordRoom(record_rate_hz=10000.0)
        room.start_recording("temp_demo")
        room.stop_recording()

        demos = room.list_demonstrations()
        assert len(demos) == 1
        assert demos[0]["name"] == "temp_demo"

        assert room.delete_demonstration("temp_demo")
        assert room.list_demonstrations() == []
        assert not room.delete_demonstration("nonexistent")


# ── Integration Tests ────────────────────────────────────────────────

class TestTeleopPipeline:
    """End-to-end teleop pipeline tests."""

    def test_full_pipeline_single_frame(self):
        """Parse → Map → Safety → Record for a single frame."""
        # Parse
        parse_room = TeleopParseRoom(num_joints=7)
        raw = make_can_frame(
            joints=[0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.0],
            gripper=0.5, frame_id=1,
        )
        leader = parse_room.parse_can_frame(raw)
        assert leader is not None

        # Map
        map_room = TeleopMapRoom(num_joints=7)
        mapped = map_room.map_linear(leader)
        assert mapped.mapping_type == "linear"

        # Safety
        safety_room = TeleopSafetyRoom(max_velocity=10.0)
        safety_result = safety_room.check(mapped)
        assert safety_result.passed

        # Record
        record_room = TeleopRecordRoom(record_rate_hz=10000.0)
        record_room.start_recording("pipeline_test")
        record_room.record_frame(leader, mapped, safety_result)
        demo = record_room.stop_recording()
        assert demo is not None
        assert demo.frame_count == 1

    def test_full_pipeline_with_fleet_mirror(self):
        """Pipeline with fleet mirror coordination."""
        fleet_room = TeleopFleetRoom(fleet_arms=["arm_left", "arm_right"])
        source_joints = tuple(0.5 for _ in range(7))
        commands = fleet_room.mirror(source_joints, "arm_left")
        assert len(commands) == 1
        assert commands[0].arm_id == "arm_right"
        assert commands[0].target_joints == source_joints

    def test_room_configs(self):
        """All teleop rooms have correct alpha values."""
        rooms = TeleopRooms.all_rooms()
        assert len(rooms) == 5

        alpha_map = {r.name: r.alpha for r in rooms}
        assert alpha_map["teleop_parse"] == 0.0
        assert alpha_map["teleop_map"] == 0.2
        assert alpha_map["teleop_safety"] == 0.1
        assert alpha_map["teleop_fleet"] == 0.5
        assert alpha_map["teleop_record"] == 0.3


# Need json import for export test
import json
