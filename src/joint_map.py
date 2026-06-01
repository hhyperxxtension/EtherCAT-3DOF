"""
Joint angle ↔ motor encoder counts conversion. Pure math — no hardware —
so it can be unit-tested offline.

Chain from a joint angle to the number the drive wants in 0x607A:

    joint_angle [rad]
      → joint revolutions       = angle / 2π
      → motor revolutions       = joint_rev × GEAR_RATIO        (harmonic drive)
      → encoder counts          = motor_rev × ENCODER_COUNTS_PER_REV

The drive's own electronic gear (0x6091) reads 1:1 on these Y7s, so the
harmonic-drive ratio is applied here in software, not by the drive.

Two per-joint calibration facts come from the hardware (see config.py):
  - JOINT_DIR  : +1 / -1, whether a rising count means a CCW-positive angle.
  - home anchor: which raw count corresponds to which joint angle. The Y7
    absolute encoder reports an arbitrary raw count at power-up, so the
    controller captures `home_counts` (the raw 0x6064 reading at a known
    pose) and passes it in; that pose's angle is JOINT_ZERO_RAD. With the
    default home_counts=0 the model degenerates to "raw count 0 ⇔
    JOINT_ZERO_RAD", which is what the offline tests use.

Conversion (per joint j, raw drive count `c`):
    angle = JOINT_ZERO_RAD[j]
          + JOINT_DIR[j] · (c − home_counts[j]) / counts_per_joint_rev[j] · 2π
and its exact inverse, rounded to the nearest integer count (0x607A is DINT).
"""

import math

try:
    from config import (
        GEAR_RATIO, ENCODER_COUNTS_PER_REV, JOINT_DIR, JOINT_ZERO_RAD,
    )
except ImportError:  # allow `import joint_map` without the full config
    GEAR_RATIO = (100.0, 200.0)
    ENCODER_COUNTS_PER_REV = 1 << 23
    JOINT_DIR = (1, 1)
    JOINT_ZERO_RAD = (0.0, 0.0)

TAU = 2.0 * math.pi

# 0x607A / 0x6064 are 32-bit signed. Commanding past this wraps the drive.
INT32_MIN = -(1 << 31)
INT32_MAX = (1 << 31) - 1

N_JOINTS = len(GEAR_RATIO)


def counts_per_joint_rev(joint):
    """Encoder counts for one full revolution of the *joint output* (after
    the harmonic drive). This is the scale factor that turns a joint angle
    into drive counts."""
    return GEAR_RATIO[joint] * ENCODER_COUNTS_PER_REV


def counts_per_rad(joint):
    """Signed encoder counts per radian of joint motion (sign = JOINT_DIR)."""
    return JOINT_DIR[joint] * counts_per_joint_rev(joint) / TAU


def deg_per_count(joint):
    """Joint degrees represented by a single encoder count — the angular
    resolution at the joint. Handy for sanity-checking precision."""
    return 360.0 / counts_per_joint_rev(joint)


def angle_to_counts(angle, joint, home_counts=0):
    """Joint angle [rad] → raw drive count (int), about `home_counts`."""
    delta = JOINT_DIR[joint] * (angle - JOINT_ZERO_RAD[joint]) / TAU
    return home_counts + int(round(delta * counts_per_joint_rev(joint)))


def counts_to_angle(counts, joint, home_counts=0):
    """Raw drive count → joint angle [rad], about `home_counts`."""
    rev = JOINT_DIR[joint] * (counts - home_counts) / counts_per_joint_rev(joint)
    return JOINT_ZERO_RAD[joint] + rev * TAU


def _home_vec(home_counts):
    if home_counts is None:
        return [0] * N_JOINTS
    if len(home_counts) != N_JOINTS:
        raise ValueError(f"home_counts must have {N_JOINTS} entries")
    return home_counts


def angles_to_counts(q, home_counts=None):
    """Joint vector [rad] → tuple of raw drive counts (one per joint)."""
    if len(q) != N_JOINTS:
        raise ValueError(f"expected {N_JOINTS} joint angles, got {len(q)}")
    home = _home_vec(home_counts)
    return tuple(angle_to_counts(q[j], j, home[j]) for j in range(N_JOINTS))


def counts_to_angles(counts, home_counts=None):
    """Tuple of raw drive counts → joint vector [rad]."""
    if len(counts) != N_JOINTS:
        raise ValueError(f"expected {N_JOINTS} counts, got {len(counts)}")
    home = _home_vec(home_counts)
    return tuple(counts_to_angle(counts[j], j, home[j]) for j in range(N_JOINTS))


def counts_in_int32(counts):
    """True if a raw count fits the drive's 32-bit signed position word."""
    return INT32_MIN <= counts <= INT32_MAX
