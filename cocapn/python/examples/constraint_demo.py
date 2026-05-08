"""
Demo: Constraint-aware OpenArm with fleet connectivity.

This demonstrates the killer app — every motor command
goes through Eisenstein constraint checking, and the arm
publishes telemetry to the PLATO fleet.
"""

from cocapn_openarm import (
    ConstraintArm,
    JointLimit,
    TorqueLimit,
    VelocityLimit,
    EisensteinWorkspace,
)


def main():
    print("╔══════════════════════════════════════════════╗")
    print("║  Cocapn OpenArm — Constraint Safety Demo     ║")
    print("╚══════════════════════════════════════════════╝")
    print()

    # Mock arm (replace with real OpenArm when hardware available)
    class MockArm:
        def __init__(self, iface, fd):
            self.iface = iface
            self.fd = fd
            print(f"[CAN] Connected to {iface} (FD={fd})")

        def enable_all(self):
            print("[CAN] All motors enabled")

        def disable_all(self):
            print("[CAN] All motors disabled")

        def get_arm(self):
            return self

        def mit_control_one(self, idx, kp, kd, q, dq, tau):
            print(f"[CAN] Motor {idx} → position {q:.3f}")

    arm = ConstraintArm(
        MockArm("can0", True),
        device_id="openarm-demo-01",
        plato_server=None,  # No PLATO server in demo
        auto_register=False,
    )

    # Define safety constraints for a 7-DOF arm
    # Joint limits (typical for a human-scale arm)
    for i in range(7):
        arm.add_constraint(JointLimit(i, -3.14, 3.14))
    arm.add_constraint(TorqueLimit(0, 5.0))
    arm.add_constraint(VelocityLimit(0, 2.0))

    # Eisenstein workspace constraint (hex-lattice boundary)
    arm.add_constraint(EisensteinWorkspace(
        radius=10,  # 10 Eisenstein units
        scale=0.1,  # 0.1m per unit = 1m workspace radius
    ))

    arm.enable_all()
    print()

    # Test 1: Safe command
    print("── Test 1: Safe command ──")
    result = arm.set_position(joint=0, target=1.57)
    print(f"  Result: {'✅ ACCEPTED' if result.satisfied else '❌ REJECTED'}")
    print(f"  Margin: {result.margin:.3f}")
    print()

    # Test 2: Joint limit violation
    print("── Test 2: Joint limit violation ──")
    result = arm.set_position(joint=0, target=5.0)
    print(f"  Result: {'✅ ACCEPTED' if result.satisfied else '❌ REJECTED'}")
    print(f"  Message: {result.message}")
    print(f"  Margin: {result.margin:.3f}")
    print()

    # Test 3: Within limits but close to boundary
    print("── Test 3: Near boundary ──")
    result = arm.set_position(joint=0, target=3.0)
    print(f"  Result: {'✅ ACCEPTED' if result.satisfied else '❌ REJECTED'}")
    print(f"  Margin: {result.margin:.3f}")
    print()

    # Test 4: Multiple joints
    print("── Test 4: Multiple joints ──")
    results = arm.set_positions({0: 0.5, 1: -1.0, 2: 0.8})
    for r in results:
        print(f"  {r.constraint_name}: {'✅' if r.satisfied else '❌'} margin={r.margin:.3f}")
    print()

    # Safety report
    print("── Safety Report ──")
    print(f"  Commands: {arm._envelope.total_commands}")
    print(f"  Blocked:  {arm._envelope.blocked_commands}")
    print(f"  Clamped:  {arm._envelope.clamped_commands}")
    print(f"  Score:    {arm.safety_score:.2f}")
    print(f"  E-Stop:   {arm.is_emergency_stopped}")

    # Constraint count
    print(f"\n  Active constraints: {len(arm.constraints)}")
    for c in arm.constraints:
        print(f"    • {c.name} (severity={c.severity.value})")


if __name__ == "__main__":
    main()
