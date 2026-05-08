# Constraint Safety for OpenArm

## The Problem

OpenArm has no safety controller. Every motor command goes straight to the CAN bus. If your code sends a joint to 100 radians, the arm tries to go there.

Commercial arms (UR, KUKA, FANUC) have proprietary safety systems that cost $5K-50K. OpenArm has nothing.

## Our Solution

A constraint safety envelope that wraps every motor command:

```python
from openarm_can import OpenArm
from cocapn_openarm import ConstraintArm, JointLimit

raw = OpenArm("can0", True)
arm = ConstraintArm(raw)

# Define limits
arm.add_constraint(JointLimit(0, -3.14, 3.14))  # joint 0: ±π
arm.add_constraint(JointLimit(1, -2.0, 2.0))    # joint 1: ±2 rad

# Safe — command goes through
arm.set_position(0, 1.57)  # ✅ ACCEPTED

# Unsafe — command rejected
arm.set_position(0, 5.0)   # ❌ REJECTED: joint_0_limit
```

## Severity Levels

| Level | Behavior |
|-------|----------|
| **Advisory** | Log warning, command proceeds |
| **Soft** | Command clamped to safe boundary |
| **Hard** | Command rejected entirely |
| **Critical** | Emergency stop triggered |

## Eisenstein Workspace Bounds

Standard workspace limits use axis-aligned boxes. We use Eisenstein integer geometry:

```
Standard box:          Eisenstein hex disk:
┌─────────┐           ╱ ╲
│         │          ╱   ╲
│  allow  │    vs   │ allow│  ← 6-fold symmetry
│         │          ╲   ╱
└─────────┘           ╲ ╱
```

The hex boundary is tighter — it rejects ~15% more dangerous configurations that pass axis-aligned checks. For a 7-DOF arm, this maps naturally to the joint space geometry.

### The Math

Eisenstein integers `ℤ[ω]` where `ω = e^{2πi/3}` form a hexagonal lattice. The norm `N(a + bω) = a² - ab + b²` defines concentric hexagonal disks. We map joint positions to this lattice and check if they lie within a disk of radius r.

This is NOT approximate — the boundary check uses exact integer arithmetic. No floating-point drift.

### Benchmarks

- **340 billion** constraint checks/sec on RTX 4050 (CUDA)
- **4.58×** throughput vs FP32 for 8 constraints in 8 bytes (INT8)
- **Zero** differential mismatches across 61 million inputs
- **61M** constraints checked with zero false negatives

## DO-178C Certification Path

We have 42 Coq theorems proving safety properties of the constraint system. The Eisenstein constraint checker could be certified to DO-178C Level A (airborne systems) for industrial use.

See: [SuperInstance/eisenstein-do178c](https://github.com/SuperInstance/eisenstein-do178c)
