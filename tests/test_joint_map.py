"""Offline tests for joint_map.py — run with plain `python`, no pytest."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import joint_map as jm

TAU = 2.0 * math.pi


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_scale_matches_config():
    # counts per joint rev = gear ratio × counts per motor rev (from config,
    # so this stays correct as calibration changes the gear ratios).
    for j in range(jm.N_JOINTS):
        expected = jm.GEAR_RATIO[j] * jm.ENCODER_COUNTS_PER_REV
        assert jm.counts_per_joint_rev(j) == expected, j
        # One full joint revolution maps to that many counts, signed by JOINT_DIR.
        assert jm.angle_to_counts(TAU, j) == round(jm.JOINT_DIR[j] * expected), j
    print("test_scale_matches_config OK")


def test_zero_and_home():
    # At JOINT_ZERO_RAD the count equals home_counts (default 0).
    for j in range(jm.N_JOINTS):
        assert jm.angle_to_counts(jm.JOINT_ZERO_RAD[j], j) == 0
    # Non-zero home anchor: zero angle sits at the captured raw count.
    assert jm.angle_to_counts(jm.JOINT_ZERO_RAD[0], 0, home_counts=123456) == 123456
    assert _close(jm.counts_to_angle(123456, 0, home_counts=123456),
                  jm.JOINT_ZERO_RAD[0])
    print("test_zero_and_home OK")


def test_roundtrip_angle_counts():
    # angle → counts → angle recovers the angle within one count of slack.
    for j in range(jm.N_JOINTS):
        tol = jm.deg_per_count(j) * math.pi / 180.0 * 1.001  # ≤ 1 count
        for k in range(-20, 21):
            a = k * 0.1
            c = jm.angle_to_counts(a, j, home_counts=500)
            back = jm.counts_to_angle(c, j, home_counts=500)
            assert abs(back - a) <= tol, (j, a, back, tol)
    print("test_roundtrip_angle_counts OK")


def test_direction_sign():
    # JOINT_DIR flips which way counts grow with angle.
    saved = jm.JOINT_DIR
    try:
        jm.JOINT_DIR = (1, 1)
        pos = jm.angle_to_counts(0.5, 0)
        jm.JOINT_DIR = (-1, 1)
        neg = jm.angle_to_counts(0.5, 0)
        assert pos == -neg and pos > 0, (pos, neg)
    finally:
        jm.JOINT_DIR = saved
    print("test_direction_sign OK")


def test_vector_helpers():
    q = (0.3, -0.7)
    home = (1000, -2000)
    counts = jm.angles_to_counts(q, home)
    assert len(counts) == jm.N_JOINTS
    back = jm.counts_to_angles(counts, home)
    for a, b in zip(q, back):
        assert _close(a, b, 1e-6), (a, b)
    # Wrong-length inputs are rejected.
    for bad in [(0.1,), (0.1, 0.2, 0.3)]:
        try:
            jm.angles_to_counts(bad)
            assert False, "expected ValueError"
        except ValueError:
            pass
    print("test_vector_helpers OK")


def test_int32_guard():
    assert jm.counts_in_int32(0)
    assert jm.counts_in_int32(jm.INT32_MAX)
    assert not jm.counts_in_int32(jm.INT32_MAX + 1)
    assert not jm.counts_in_int32(jm.INT32_MIN - 1)
    print("test_int32_guard OK")


if __name__ == "__main__":
    test_scale_matches_config()
    test_zero_and_home()
    test_roundtrip_angle_counts()
    test_direction_sign()
    test_vector_helpers()
    test_int32_guard()
    print("\nAll joint_map tests passed.")
