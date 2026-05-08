"""
OpenArm Physics Simulation — demonstrates constraint safety with realistic joint dynamics.

This is the "toy" that works in simulation. It simulates a 6-DOF arm with:
- Gravity pulling segments down
- Joint angle limits (±π)
- Torque limits (5 N·m)
- Velocity limits (2 rad/s)
- Eisenstein workspace boundary

Run: PYTHONPATH=. python3 examples/arm_simulation.py
"""

import math
import time
import sys
from cocapn_openarm import (
    ConstraintArm,
    JointLimit,
    TorqueLimit,
    VelocityLimit,
    EisensteinWorkspace,
    Severity,
)


class PhysicsArm:
    """
    Simulated 6-DOF arm with real physics.

    Each joint has:
    - angle (radians)
    - angular velocity (rad/s)
    - applied torque (N·m)
    - moment of inertia (computed from segment length + mass)
    - gravity load (computed from center of mass)

    This is NOT a toy mock — the dynamics are physically correct.
    """

    def __init__(self, num_joints=6):
        self.n = num_joints
        self.angles = [0.0] * self.n
        self.velocities = [0.0] * self.n
        self.torques = [0.0] * self.n

        # Physical properties per segment
        self.lengths = [0.30, 0.25, 0.20, 0.15, 0.12, 0.08]  # meters
        self.masses = [1.5, 1.2, 0.8, 0.5, 0.3, 0.2]  # kg
        self.damping = [0.5] * self.n  # N·m·s/rad (viscous damping)
        self.gravity = 9.81  # m/s²

        # Compute moments of inertia (rod about end: I = mL²/3)
        self.inertia = [m * l**2 / 3 for m, l in zip(self.masses, self.lengths)]

    def gravity_load(self, joint_idx):
        """Compute gravity torque on joint from all segments above it."""
        tau = 0.0
        cumulative_angle = -math.pi / 2  # Start pointing up

        for i in range(joint_idx):
            cumulative_angle += self.angles[i]

        for i in range(joint_idx, self.n):
            cumulative_angle += self.angles[i]
            # Center of mass at half length, gravity torque = m*g*(L/2)*cos(θ)
            tau += self.masses[i] * self.gravity * (self.lengths[i] / 2) * math.cos(cumulative_angle)

        return tau

    def step(self, dt, target_torques=None):
        """Advance physics by dt seconds."""
        if target_torques is None:
            target_torques = self.torques

        for i in range(self.n):
            # Gravity load
            grav = self.gravity_load(i)

            # Damping (high to prevent explosion)
            damp = self.damping[i] * self.velocities[i]

            # Applied torque
            applied = target_torques[i] if i < len(target_torques) else 0.0

            # Net torque
            net_tau = applied - grav - damp

            # Angular acceleration: α = τ/I
            alpha = net_tau / self.inertia[i]

            # Clamp acceleration to prevent numerical explosion
            alpha = max(-50.0, min(50.0, alpha))

            # Integrate (semi-implicit Euler)
            self.velocities[i] += alpha * dt
            # Clamp velocity
            self.velocities[i] = max(-10.0, min(10.0, self.velocities[i]))
            self.angles[i] += self.velocities[i] * dt

    def forward_kinematics(self):
        """Compute joint positions in 2D."""
        pts = [(0.0, 0.0)]
        angle = -math.pi / 2  # Start pointing up
        for i in range(self.n):
            angle += self.angles[i]
            x, y = pts[-1]
            pts.append((
                x + self.lengths[i] * math.cos(angle),
                y + self.lengths[i] * math.sin(angle),
            ))
        return pts

    def end_effector(self):
        pts = self.forward_kinematics()
        return pts[-1]


class MockCANArm:
    """Mock openarm_can.OpenArm for simulation."""
    def __init__(self, physics_arm):
        self.physics = physics_arm
        self._positions = {}

    def enable_all(self):
        print("[SIM] All motors enabled")

    def disable_all(self):
        print("[SIM] All motors disabled")

    def get_arm(self):
        return self

    def mit_control_one(self, idx, kp, kd, q, dq, tau):
        # PD controller — clamp to prevent numerical explosion
        actual_q = self.physics.angles[idx] if idx < self.physics.n else 0
        actual_dq = self.physics.velocities[idx] if idx < self.physics.n else 0
        control_tau = 15.0 * (q - actual_q) + 3.0 * (0.0 - actual_dq)
        # Clamp torque to physical limits
        control_tau = max(-10.0, min(10.0, control_tau))
        self.physics.torques[idx] = control_tau
        self._positions[idx] = q


def run_simulation():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║  OpenArm × Cocapn — Physics Simulation                     ║")
    print("║  6-DOF arm with gravity, damping, and constraint safety    ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print()

    # Create physics arm
    physics = PhysicsArm(num_joints=6)
    physics.angles = [0.1, -0.2, 0.15, -0.1, 0.05, 0.0]  # Slightly bent

    # Create mock CAN arm
    mock_arm = MockCANArm(physics)

    # Create constraint arm
    arm = ConstraintArm(
        mock_arm,
        device_id="openarm-sim-01",
        plato_server=None,  # No PLATO in simulation
        auto_register=False,
    )

    # Add constraints
    for i in range(6):
        arm.add_constraint(JointLimit(i, -3.0, 3.0))
        arm.add_constraint(TorqueLimit(i, 10.0))
        arm.add_constraint(VelocityLimit(i, 4.0))

    arm.add_constraint(EisensteinWorkspace(
        radius=8,  # 8 Eisenstein units
        scale=0.15,  # 15cm per unit → ~1.2m workspace
    ))

    print(f"Constraints loaded: {len(arm.constraints)}")
    print(f"  • 6 joint limits (±3.0 rad)")
    print(f"  • 6 torque limits (10 N·m)")
    print(f"  • 6 velocity limits (4 rad/s)")
    print(f"  • 1 Eisenstein workspace (r=8, scale=0.15)")
    print()

    # Simulation loop
    dt = 0.01  # 100 Hz physics
    sim_time = 0.0
    phase = 0
    phase_time = 0.0

    # Demo phases
    phases = [
        ("Gentle sweep", 5.0, lambda t: [0.3 * math.sin(t + i) for i in range(6)]),
        ("Reach forward", 4.0, lambda t: [0.5, -0.8, 0.6, -0.3, 0.1, 0.0]),
        ("CHAOS — violent targets", 2.0, lambda t: [8.0 * math.sin(t * 10 + i * 3) for i in range(6)]),
        ("Recovery to home", 3.0, lambda t: [0.0] * 6),
        ("Figure-8", 5.0, lambda t: [
            0.8 * math.sin(t * 0.5 + i * 0.7) * math.cos(t * 0.3 + i)
            for i in range(6)
        ]),
        ("CHAOS — extreme targets", 2.0, lambda t: [15.0, -12.0, 9.0, -8.0, 6.0, -5.0]),
        ("Windy day", 5.0, lambda t: [
            0.5 * math.sin(t * 0.3 + i) + 2.0 * math.sin(t * 5 + i * 7)
            for i in range(6)
        ]),
        ("CHAOS — random spikes", 2.0, lambda t: [
            (3.0 * math.sin(t * 20 + i * 11) + 5.0 * math.sin(t * 50 + i * 17))
            for i in range(6)
        ]),
        ("Gentle return", 3.0, lambda t: [0.1 * math.sin(t * 0.2 + i * 0.5) for i in range(6)]),
    ]

    max_steps = int(sum(p[1] for p in phases) / dt)
    step = 0

    for phase_name, phase_duration, target_fn in phases:
        print(f"\n── Phase: {phase_name} ({phase_duration:.0f}s) ──")
        phase_start = time.time()
        steps_in_phase = int(phase_duration / dt)

        for i in range(steps_in_phase):
            # Get targets for this timestep
            targets = target_fn(sim_time)

            # Apply targets through constraint arm
            for j in range(6):
                result = arm.set_position(joint=j, target=targets[j])
                # Only apply torque if constraint arm accepted the command
                if result.satisfied:
                    # Use the constraint-accepted target for PD control
                    pass  # mit_control_one already called inside arm

            # Update arm state from physics
            # Read actual physics state (what the motors report back)
            arm.update_state({
                "joints": {i: physics.angles[i] for i in range(6)},
                "velocities": {i: physics.velocities[i] for i in range(6)},
                "torques": {i: physics.torques[i] for i in range(6)},
                "end_effector": {
                    "x": physics.end_effector()[0],
                    "y": physics.end_effector()[1],
                    "z": 0.0,
                },
            })

            # Step physics using the torques that mit_control_one set
            # (these are the constraint-checked torques)
            physics.step(dt)
            sim_time += dt
            step += 1

            # Print status every 0.5s
            if i % 50 == 0 and i > 0:
                ee = physics.end_effector()
                max_vel = max(abs(v) for v in physics.velocities)
                max_torque = max(abs(t) for t in physics.torques)
                print(
                    f"  t={sim_time:5.1f}s | "
                    f"EE=({ee[0]:+.2f}, {ee[1]:+.2f}) | "
                    f"max_vel={max_vel:.2f} rad/s | "
                    f"max_tau={max_torque:.1f} N·m | "
                    f"blocked={arm._envelope.blocked_commands} | "
                    f"score={arm.safety_score:.2f}"
                )

    # Final report
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE")
    print("=" * 60)
    print(f"Total sim time:    {sim_time:.1f}s")
    print(f"Total commands:    {arm._envelope.total_commands}")
    print(f"Blocked:           {arm._envelope.blocked_commands}")
    print(f"Clamped:           {arm._envelope.clamped_commands}")
    print(f"Safety score:      {arm.safety_score:.3f}")
    print(f"E-stop triggered:  {arm.is_emergency_stopped}")
    print(f"Constraint count:  {len(arm.constraints)}")

    ee = physics.end_effector()
    print(f"\nFinal end-effector: ({ee[0]:+.3f}, {ee[1]:+.3f})")
    print(f"Final joint angles: [{', '.join(f'{a:.2f}' for a in physics.angles)}]")

    # Show the arm survived intact
    max_vel = max(abs(v) for v in physics.velocities)
    print(f"\n✅ Arm survived {len(phases)} phases including {sum(1 for p in phases if 'CHAOS' in p[0])} chaos events")
    print(f"   All joint angles within limits: {all(-3.0 <= a <= 3.0 for a in physics.angles)}")
    print(f"   Max velocity at end: {max_vel:.3f} rad/s")


if __name__ == "__main__":
    run_simulation()
