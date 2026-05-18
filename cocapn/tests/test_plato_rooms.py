"""Tests for OpenArm PLATO room decomposition."""

import math
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from plato_rooms import (
    JointState, ConstraintResult, TrajectorySegment, FleetCommand,
    OpenArmRooms, RoomConfig, ConstraintChecker, JointLimit, TorqueLimit,
    WorkspaceLimit, eisenstein_cell, eisenstein_distance,
)


class TestJointState:
    def test_creation(self):
        js = JointState(
            joints=(0.0,) * 7, velocities=(0.0,) * 7, torques=(0.0,) * 7,
            timestamp=0.0,
        )
        assert len(js.joints) == 7
        assert js.source == "can_bus"

    def test_to_dict(self):
        js = JointState(joints=(1.0, 2.0), velocities=(0.1, 0.2), torques=(0.5, 0.6), timestamp=1.0)
        d = js.to_dict()
        assert d["joints"] == [1.0, 2.0]
        assert d["timestamp"] == 1.0


class TestEisenstein:
    def test_origin_cell(self):
        cell = eisenstein_cell(0.0, 0.0)
        assert cell == (0, 0)

    def test_unit_cell(self):
        cell = eisenstein_cell(1.0, 0.0)
        assert isinstance(cell, tuple)
        assert len(cell) == 2

    def test_hex_symmetry(self):
        cells = set()
        for deg in range(0, 360, 60):
            rad = math.radians(deg)
            x, y = math.cos(rad), math.sin(rad)
            cells.add(eisenstein_cell(x, y))
        assert len(cells) >= 3

    def test_distance_to_cell(self):
        dist = eisenstein_distance(0.0, 0.0, (0, 0))
        assert dist == pytest.approx(0.0, abs=0.01)


class TestConstraintChecker:
    def setup_method(self):
        self.checker = ConstraintChecker()
        self.safe_state = JointState(
            joints=(0.0,) * 7, velocities=(0.0,) * 7, torques=(0.0,) * 7,
            timestamp=0.0,
        )

    def test_safe_state_passes(self):
        result = self.checker.check(self.safe_state)
        assert isinstance(result, ConstraintResult)
        assert result.passed is True
        assert len(result.violations) == 0

    def test_joint_limit_violation(self):
        state = JointState(
            joints=(4.0,) * 7,
            velocities=(0.0,) * 7, torques=(0.0,) * 7, timestamp=0.0,
        )
        result = self.checker.check(state)
        assert result.passed is False
        assert len(result.violations) > 0
        assert any("above_max" in v for v in result.violations)

    def test_torque_limit_violation(self):
        state = JointState(
            joints=(0.0,) * 7, velocities=(0.0,) * 7,
            torques=(10.0,) * 7,
            timestamp=0.0,
        )
        result = self.checker.check(state)
        assert result.passed is False
        assert any("torque" in v for v in result.violations)

    def test_workspace_limit_violation(self):
        joints = [0.0] * 7
        joints[0] = 3.5
        joints[1] = 3.5
        state = JointState(
            joints=tuple(joints), velocities=(0.0,) * 7, torques=(0.0,) * 7,
            timestamp=0.0,
        )
        result = self.checker.check(state)
        assert result.passed is False
        assert any("workspace" in v for v in result.violations)

    def test_latency_sub_ms(self):
        result = self.checker.check(self.safe_state)
        assert result.check_latency_us < 1000

    def test_margin_computation(self):
        result = self.checker.check(self.safe_state)
        assert result.min_margin > 0

    def test_custom_limits(self):
        checker = ConstraintChecker(
            joint_limits=[JointLimit(0, -0.5, 0.5)],
            torque_limits=[TorqueLimit(0, 1.0)],
            workspace_limits=[WorkspaceLimit(0, 1, 1.0)],
        )
        state = JointState(
            joints=(0.3,) * 7, velocities=(0.0,) * 7, torques=(0.5,) * 7,
            timestamp=0.0,
        )
        result = checker.check(state)
        assert result.passed is True


class TestOpenArmRooms:
    def test_all_rooms_defined(self):
        rooms = OpenArmRooms.all_rooms()
        assert len(rooms) == 5
        names = [r.name for r in rooms]
        assert "command_parse" in names
        assert "constraint_check" in names
        assert "trajectory_plan" in names
        assert "fleet_coordination" in names
        assert "learning" in names

    def test_alpha_ordering(self):
        rooms = OpenArmRooms.all_rooms()
        alphas = [r.alpha for r in rooms]
        for i in range(1, len(alphas)):
            assert alphas[i] >= alphas[i-1]

    def test_command_parse_is_pure_code(self):
        room = OpenArmRooms.command_parse()
        assert room.alpha == 0.0

    def test_constraint_check_mostly_code(self):
        room = OpenArmRooms.constraint_check()
        assert room.alpha < 0.3

    def test_safety_rooms_are_fast(self):
        parse = OpenArmRooms.command_parse()
        check = OpenArmRooms.constraint_check()
        assert parse.max_latency_ms < 1.0
        assert check.max_latency_ms <= 10.0

    def test_constraint_check_description(self):
        room = OpenArmRooms.constraint_check()
        assert "Eisenstein" in room.description
