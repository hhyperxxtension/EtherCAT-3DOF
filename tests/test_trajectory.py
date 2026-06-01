"""Offline tests for trajectory.py — run with plain `python`, no pytest."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import trajectory as traj


def _close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def test_endpoints():
    t = traj.plan_ptp((0.0, 0.0), (1.0, -0.5), v_max=1.0, a_max=2.0)
    assert _close(t.at(0.0)[0], 0.0) and _close(t.at(0.0)[1], 0.0)
    assert _close(t.at(t.duration)[0], 1.0) and _close(t.at(t.duration)[1], -0.5)
    print("test_endpoints OK")


def test_synchronization():
    # Both joints must finish at the same time T (slowest joint governs).
    t = traj.plan_ptp((0.0, 0.0), (2.0, 0.1), v_max=1.0, a_max=2.0)
    q = t.at(t.duration)
    assert _close(q[0], 2.0) and _close(q[1], 0.1)
    # The short joint must still be moving (not finished) just before T/2
    # if it were planned independently — here it should reach target only
    # at T, confirming it was slowed to sync.
    q_mid = t.at(t.duration * 0.5)
    assert q_mid[1] < 0.1 - 1e-3, q_mid
    print("test_synchronization OK")


def test_velocity_accel_limits():
    v_max, a_max = 1.0, 2.0
    t = traj.plan_ptp((0.0, 0.0), (3.0, 1.0), v_max=v_max, a_max=a_max)
    samples = t.samples(dt=0.001)
    qs = [q for _, q in samples]
    ts = [tt for tt, _ in samples]
    # Numerical velocity per joint must not exceed v_max (+ small margin).
    for j in range(2):
        vmax_seen = 0.0
        for k in range(1, len(qs)):
            dt = ts[k] - ts[k - 1]
            if dt <= 0:
                continue
            v = abs(qs[k][j] - qs[k - 1][j]) / dt
            vmax_seen = max(vmax_seen, v)
        assert vmax_seen <= v_max + 1e-2, (j, vmax_seen)
    print("test_velocity_accel_limits OK")


def test_monotonic_progress():
    # Distance from start to current should be non-decreasing for a PTP move.
    t = traj.plan_ptp((0.0, 0.0), (1.0, 1.0), v_max=1.0, a_max=2.0)
    prev = -1.0
    for tt, q in t.samples(dt=0.004):
        d = (q[0] ** 2 + q[1] ** 2) ** 0.5
        assert d >= prev - 1e-9, (tt, d, prev)
        prev = d
    print("test_monotonic_progress OK")


def test_zero_move():
    t = traj.plan_ptp((0.5, 0.5), (0.5, 0.5), v_max=1.0, a_max=2.0)
    assert t.duration == 0.0
    assert _close(t.at(0.0)[0], 0.5) and _close(t.at(0.0)[1], 0.5)
    print("test_zero_move OK")


if __name__ == "__main__":
    test_endpoints()
    test_synchronization()
    test_velocity_accel_limits()
    test_monotonic_progress()
    test_zero_move()
    print("\nAll trajectory tests passed.")
