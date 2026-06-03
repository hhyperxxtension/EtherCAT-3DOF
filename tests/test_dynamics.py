"""Offline tests for dynamics.py + trajectory derivatives — plain `python`."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import dynamics as dyn
import gravity_model as gm
from trajectory import plan_ptp


def _close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def test_trajectory_derivatives():
    # q̇ integrates to the move; q̈ is +/-a then 0; endpoints zero.
    tr = plan_ptp((0.0, 0.0), (1.0, -0.5), v_max=1.0, a_max=2.0)
    q0, qd0, qdd0 = tr.at_full(0.0)
    assert qd0 == (0.0, 0.0) and qdd0 == (0.0, 0.0)
    # Numerical derivative of position matches analytic velocity.
    t, h = 0.15, 1e-6
    qa = tr.at(t + h); qb = tr.at(t - h)
    for j in range(2):
        num = (qa[j] - qb[j]) / (2 * h)
        assert _close(num, tr.at_full(t)[1][j], 1e-4), (j, num)
    print("test_trajectory_derivatives OK")


def _synth_tau(grav_truth, dyn_truth, q, qd, qdd):
    g = [sum(c * b for c, b in zip(grav_truth[j], gm._basis(j, q)))
         for j in range(dyn.N_JOINTS)]
    d = [sum(c * b for c, b in zip(dyn_truth[j], dyn.dyn_basis(j, q, qd, qdd)))
         for j in range(dyn.N_JOINTS)]
    return tuple(g[j] + d[j] for j in range(dyn.N_JOINTS))


def test_fit_recovers_dynamic_coeffs():
    grav_truth = [[180., 0., 90., 0., 10.], [70., 0., 5.]]
    dyn_truth = [[120., 40., 15., 100., 6.], [38., 12., 49., 4.]]
    g = gm.GravityModel(coeffs=[list(c) for c in grav_truth])
    # Rich excitation is REQUIRED for identifiability: vary q, q̇ AND q̈ on
    # BOTH joints independently (a rank-deficient set, e.g. q̈1 held constant,
    # would fit tau perfectly yet leave the coeffs non-unique).
    import random
    random.seed(1)
    samples = []
    for _ in range(120):
        q = (random.uniform(-1, 1), random.uniform(-1, 1))
        qd = (random.uniform(-1, 1), random.uniform(-1, 1))
        qdd = (random.uniform(-2, 2), random.uniform(-2, 2))
        tau = _synth_tau(grav_truth, dyn_truth, q, qd, qdd)
        samples.append((q, qd, qdd, tau))
    model = dyn.DynamicsModel(g)
    assert not model.is_identified()
    rms = model.fit(samples)
    for j in range(dyn.N_JOINTS):
        assert rms[j] < 1e-6, (j, rms[j])
        for got, exp in zip(model.coeffs[j], dyn_truth[j]):
            assert _close(got, exp, 1e-3), (j, got, exp)
    print(f"test_fit_recovers_dynamic_coeffs OK (rms={['%.1e' % r for r in rms]})")


def test_payload_residual():
    grav_truth = [[180., 0., 90., 0., 10.], [70., 0., 5.]]
    dyn_truth = [[120., 40., 15., 100., 6.], [38., 12., 49., 4.]]
    g = gm.GravityModel(coeffs=[list(c) for c in grav_truth])
    model = dyn.DynamicsModel(g, coeffs=[list(c) for c in dyn_truth])
    q, qd, qdd = (0.5, -0.3), (0.4, -0.2), (1.0, 0.5)
    tau0 = _synth_tau(grav_truth, dyn_truth, q, qd, qdd)
    # No payload -> residual ~ 0.
    r = model.payload_residual(q, qd, qdd, tau0)
    assert all(_close(ri, 0.0, 1e-6) for ri in r), r
    # Add a known payload offset -> residual equals it.
    tau1 = (tau0[0] + 17.0, tau0[1] + 8.0)
    r = model.payload_residual(q, qd, qdd, tau1)
    assert _close(r[0], 17.0) and _close(r[1], 8.0), r
    print("test_payload_residual OK")


def test_static_reduces_to_gravity():
    # q̇=q̈=0 -> dyn part zero -> predict == gravity.
    g = gm.GravityModel(coeffs=[[180., 0., 90., 0., 10.], [70., 0., 5.]])
    model = dyn.DynamicsModel(g, coeffs=[[120., 40., 15., 100., 6.], [38., 12., 49., 4.]])
    q = (0.7, -0.4); z = (0.0, 0.0)
    assert model.predict(q, z, z) == g.predict(q)
    print("test_static_reduces_to_gravity OK")


if __name__ == "__main__":
    test_trajectory_derivatives()
    test_fit_recovers_dynamic_coeffs()
    test_payload_residual()
    test_static_reduces_to_gravity()
    print("\nAll dynamics tests passed.")
