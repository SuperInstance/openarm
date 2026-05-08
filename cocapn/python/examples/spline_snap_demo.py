#!/usr/bin/env python3
"""
Spline Snap Demo — Hard Clamp vs Smooth Snap

Generates 100 random targets across a joint range and compares:
  1. Original (raw) target values
  2. Hard-clamped values (simple min/max)
  3. Smooth-snapped values (SplineSnapConstraint)

The text plot shows how smooth snap produces natural-looking corrections
while hard clamp creates sharp discontinuities at the boundary.
"""

import sys
import os
import random
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cocapn_openarm.constraints import (
    SplineSnapConstraint,
    Severity,
)


def hard_clamp(value: float, lo: float, hi: float) -> float:
    """Simple min/max clamping."""
    return max(lo, min(hi, value))


def text_plot(values: list[float], width: int = 60, lo: float = -2.0, hi: float = 2.0):
    """Convert a list of values to text sparkline chars."""
    chars = []
    for v in values:
        frac = (v - lo) / (hi - lo)
        pos = int(frac * (width - 1))
        pos = max(0, min(width - 1, pos))
        chars.append(pos)
    return chars


def fmt_val(v: float) -> str:
    return f"{v:+.3f}"


def main():
    random.seed(42)

    # Joint parameters
    boundary_min = -1.5
    boundary_max = 1.5
    deadband = 0.15  # 15% of range
    snap_strength = 2.5

    snap = SplineSnapConstraint(
        joint_index=0,
        boundary_min=boundary_min,
        boundary_max=boundary_max,
        deadband=deadband,
        snap_strength=snap_strength,
        severity=Severity.SOFT,
    )

    # Generate 100 random targets, some intentionally near/outside boundaries
    targets = []
    for _ in range(70):
        targets.append(random.uniform(boundary_min - 0.3, boundary_max + 0.3))
    for _ in range(15):
        targets.append(random.uniform(boundary_min - 0.5, boundary_min + 0.3))
    for _ in range(15):
        targets.append(random.uniform(boundary_max - 0.3, boundary_max + 0.5))
    targets.sort()

    # Compute clamped and snapped
    hard_clamped = [hard_clamp(t, boundary_min, boundary_max) for t in targets]
    soft_snapped = [snap.snap_value(t) for t in targets]

    full_range = boundary_min - 0.5
    full_hi = boundary_max + 0.5

    print("=" * 78)
    print("SPLINE SNAP DEMO — Hard Clamp vs Smooth Snap")
    print("=" * 78)
    print(f"  Boundary: [{boundary_min:.2f}, {boundary_max:.2f}]")
    print(f"  Deadband: {deadband*100:.0f}% of range ({snap.deadband_abs:.3f} units)")
    print(f"  Snap strength: {snap_strength}")
    print(f"  Targets: {len(targets)} (sorted)")
    print()

    # Show boundary markers
    width = 60
    bmin_pos = int((boundary_min - full_range) / (full_hi - full_range) * (width - 1))
    bmax_pos = int((boundary_max - full_range) / (full_hi - full_range) * (width - 1))

    def make_line(positions: list[int], marker: str = "│") -> str:
        line = [" "] * width
        for p in positions:
            line[p] = marker
        return "".join(line)

    def val_to_pos(v: float) -> int:
        frac = (v - full_range) / (full_hi - full_range)
        return max(0, min(width - 1, int(frac * (width - 1))))

    # Draw boundary line
    boundary_line = list("·" * width)
    for i in range(bmin_pos, bmax_pos + 1):
        boundary_line[i] = "─"
    boundary_line[bmin_pos] = "┃"
    boundary_line[bmax_pos] = "┃"

    print("  Scale:")
    print(f"  {''.join(boundary_line)}")
    print(f"  {full_range:+.1f}{' ' * (width - 12)}{full_hi:+.1f}")
    print(f"  {' ':>{bmin_pos}}└─ safe zone ─┘")
    print()

    # Sample 25 evenly spaced targets for the text plot
    step = max(1, len(targets) // 25)
    sample_indices = list(range(0, len(targets), step))[:25]

    print("  Sampled targets (25 of 100):")
    print()
    print(f"  {'Index':>5}  {'Target':>7}  {'Hard':>7}  {'Snap':>7}  {'HΔ':>7}  {'SΔ':>7}  Profile")
    print("  " + "-" * 76)

    for idx in sample_indices:
        t = targets[idx]
        hc = hard_clamped[idx]
        ss = soft_snapped[idx]
        delta_h = hc - t
        delta_s = ss - t

        # Build mini profile bar
        t_pos = val_to_pos(t)
        hc_pos = val_to_pos(hc)
        ss_pos = val_to_pos(ss)

        bar = list(" " * 30)
        bar[0] = "|"  # min boundary marker
        bar[29] = "|"  # max boundary marker

        # Map positions to 30-char bar
        def to_bar(v):
            frac = (v - full_range) / (full_hi - full_range)
            return max(0, min(29, int(frac * 29)))

        tp = to_bar(t)
        hp = to_bar(hc)
        sp = to_bar(ss)

        bar[tp] = "●"  # target
        bar[hp] = "□"  # hard clamp
        bar[sp] = "◇"  # snap

        print(f"  {idx:>5}  {fmt_val(t)}  {fmt_val(hc)}  {fmt_val(ss)}  {fmt_val(delta_h)}  {fmt_val(delta_s)}  {''.join(bar)}")

    print()
    print("  Legend: ●=target  □=hard_clamp  ◇=smooth_snap  |=boundary")
    print()

    # Statistics
    corrections_hard = [abs(hc - t) for t, hc in zip(targets, hard_clamped)]
    corrections_snap = [abs(ss - t) for t, ss in zip(targets, soft_snapped)]
    jumps_hard = [abs(hard_clamped[i] - hard_clamped[i-1]) for i in range(1, len(hard_clamped))]
    jumps_snap = [abs(soft_snapped[i] - soft_snapped[i-1]) for i in range(1, len(soft_snapped))]

    n_corrected_hard = sum(1 for c in corrections_hard if c > 1e-6)
    n_corrected_snap = sum(1 for c in corrections_snap if c > 1e-6)

    print("  STATISTICS")
    print("  " + "-" * 50)
    print(f"  {'Metric':<30} {'Hard Clamp':>10} {'Smooth Snap':>10}")
    print(f"  {'-'*50}")
    print(f"  {'Values corrected':<30} {n_corrected_hard:>10} {n_corrected_snap:>10}")
    print(f"  {'Max correction':<30} {max(corrections_hard):>10.4f} {max(corrections_snap):>10.4f}")
    print(f"  {'Mean correction':<30} {sum(corrections_hard)/len(corrections_hard):>10.4f} {sum(corrections_snap)/len(corrections_snap):>10.4f}")
    print(f"  {'Max jump (adjacent)':<30} {max(jumps_hard):>10.4f} {max(jumps_snap):>10.4f}")
    print(f"  {'Mean jump (adjacent)':<30} {sum(jumps_hard)/len(jumps_hard):>10.4f} {sum(jumps_snap)/len(jumps_snap):>10.4f}")
    print()

    # Smoothness metric: sum of squared second differences (lower = smoother)
    def smoothness(vals):
        return sum((vals[i] - 2*vals[i-1] + vals[i-2])**2
                   for i in range(2, len(vals)))

    s_hard = smoothness(hard_clamped)
    s_snap = smoothness(soft_snapped)
    print(f"  Smoothness (Σ² 2nd diff, lower=better):")
    print(f"    Hard clamp:  {s_hard:.4f}")
    print(f"    Smooth snap: {s_snap:.4f}")
    if s_snap < s_hard:
        print(f"    ✓ Smooth snap is {s_hard/s_snap:.1f}x smoother")
    print()
    print("=" * 78)
    print("Demo complete. Smooth snap eliminates discontinuities at boundaries.")
    print("=" * 78)


if __name__ == "__main__":
    main()
