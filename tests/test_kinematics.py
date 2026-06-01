"""Offline tests for kinematics.py — run with plain `python`, no pytest."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import kinematics as kin

LINKS = (0.30, 0.25)
TOL = 1e-9


def _close(a, b, tol=1e-7):
    return abs(a - b) <= tol


def test_fk_known_poses():
    # Fully stretched along +X.
    x, z = kin.forward((0.0, 0.0), LINKS)
    assert _close(x, 0.55) and _close(z, 0.0), (x, z)
    # Shoulder straight up, elbow straight (both links along +Z).
    x, z = kin.forward((math.pi / 2, 0.0), LINKS)
    assert _close(x, 0.0) and _close(z, 0.55), (x, z)
    # Shoulder along +X, elbow folded back 180° → tip at L1−L2 on +X.
    x, z = kin.forward((0.0, math.pi), LINKS)
    assert _close(x, 0.05) and _close(z, 0.0), (x, z)
    print("test_fk_known_poses OK")


def test_ik_roundtrip_both_branches():
    # Sweep reachable joint space, FK→point→IK, expect to recover angles.
    n = 0
    for q0 in [x * 0.2 for x in range(-7, 8)]:
        for q1 in [x * 0.2 for x in range(1, 13)]:   # q1>0 → "up" branch
            x, z = kin.forward((q0, q1), LINKS)
            r0, r1 = kin.inverse(x, z, LINKS, "up")
            rx, rz = kin.forward((r0, r1), LINKS)
            assert _close(rx, x, 1e-6) and _close(rz, z, 1e-6), (q0, q1, rx, rz)
            n += 1
    print(f"test_ik_roundtrip_both_branches OK ({n} poses)")


def test_both_branches_agree_on_ee():
    x, z = 0.4, 0.2
    sols = kin.inverse_both(x, z, LINKS)
    for name, q in sols.items():
        rx, rz = kin.forward(q, LINKS)
        assert _close(rx, x, 1e-7) and _close(rz, z, 1e-7), (name, rx, rz)
    # Branches must differ in elbow sign.
    assert sols["up"][1] >= 0.0 >= sols["down"][1]
    print("test_both_branches_agree_on_ee OK")


def test_reachability_and_unreachable():
    rmin, rmax = kin.workspace_bounds(LINKS)
    assert _close(rmin, 0.05) and _close(rmax, 0.55)
    assert kin.reachable(0.55, 0.0, LINKS)
    assert kin.reachable(0.05, 0.0, LINKS)
    assert not kin.reachable(0.6, 0.0, LINKS)      # past outer radius
    assert not kin.reachable(0.0, 0.0, LINKS)      # inside inner radius
    try:
        kin.inverse(0.7, 0.0, LINKS)
        assert False, "expected Unreachable"
    except kin.Unreachable:
        pass
    print("test_reachability_and_unreachable OK")


def test_joint_limits():
    limits = ((-1.0, 1.0), (-1.0, 1.0))
    assert kin.within_joint_limits((0.5, -0.5), limits)
    assert not kin.within_joint_limits((1.5, 0.0), limits)
    print("test_joint_limits OK")


if __name__ == "__main__":
    test_fk_known_poses()
    test_ik_roundtrip_both_branches()
    test_both_branches_agree_on_ee()
    test_reachability_and_unreachable()
    test_joint_limits()
    print("\nAll kinematics tests passed.")
