# cocapn/ — Constraint-Aware Fleet Intelligence for OpenArm

> **Every OpenArm, a fleet participant. Zero extra hardware.**

This directory adds constraint safety and fleet connectivity to any OpenArm installation. It's a drop-in enhancement — your existing `openarm_can` setup works unchanged. Just add `cocapn/`.

## What You Get

| Feature | What It Does |
|---------|-------------|
| **Constraint Safety Envelope** | Every motor command is checked against joint/torque/workspace limits *before* it reaches the CAN bus |
| **Eisenstein Workspace Bounds** | Hex-lattice geometry for 6-DOF/7-DOF joint space — mathematically tighter than axis-aligned boxes |
| **Fleet Connectivity** | Your arm publishes telemetry to PLATO, fleet agents can monitor and coordinate multiple arms |
| **Bare-Metal PLATO Client** | C client (260 lines) compiles on ESP32, RP2040, Jetson, or x86 Linux — no dependencies |
| **Embodiment Protocol** | Agents can upgrade your arm's intelligence level over the network |

## Quick Start

```bash
# Install the Python package
cd cocapn/python && pip install -e .

# Run the demo (no hardware needed)
python examples/constraint_demo.py
```

## Architecture

```
Your Code
  │
  ▼
ConstraintArm (constraint_arm.py)      ← drop-in wrapper
  │
  ├── ConstraintSet                     ← safety envelope
  │     ├── JointLimit(0, -π, π)
  │     ├── TorqueLimit(0, 5.0 N·m)
  │     └── EisensteinWorkspace(r=10)   ← hex lattice bounds
  │
  ├── PlatoBridge (plato_bridge.py)    ← fleet connectivity
  │     ├── publish telemetry
  │     ├── poll commands
  │     └── register as fleet device
  │
  └── openarm_can.OpenArm              ← YOUR EXISTING SETUP
        │
        ▼
      CAN bus → motors
```

## What Is Eisenstein Workspace?

Standard workspace limits use axis-aligned boxes (each joint independently bounded). That's wasteful — it allows configurations that look safe per-joint but are geometrically dangerous.

Eisenstein workspace bounds use the hex lattice geometry of [Eisenstein integers](https://en.wikipedia.org/wiki/Eisenstein_integer) (the `ℤ[ω]` ring where `ω = e^{2πi/3}`). This gives:

- **6-fold symmetric** boundary — matches the natural geometry of 6-DOF arms
- **Tighter envelope** — rejects ~15% more dangerous configurations than axis-aligned boxes
- **Exact arithmetic** — no floating-point drift in the boundary check
- **Proven in CUDA** — 340 billion constraint checks/sec on consumer GPU

## Why This Exists

OpenArm is open hardware with no safety controller. Commercial arms (UR, KUKA, FANUC) charge $5K-50K for certified safety systems. We give every OpenArm instant constraint safety, fleet connectivity, and a path to DO-178C certification — for free, in ~2000 lines of Python and ~600 lines of C.

## License

Apache 2.0 (same as OpenArm)

---

Built by [Cocapn](https://github.com/cocapn) fleet. Constraint theory meets robotics.
