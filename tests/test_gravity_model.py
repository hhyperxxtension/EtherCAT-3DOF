"""Offline tests for gravity_model.py — run with plain `python`, no pytest."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import gravity_model as gm


def _close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _synthetic_tau(coeffs, q):
    """Ground-truth load torque from known coeffs (same basis as the model)."""
    return tuple(sum(c * b for c, b in zip(coeffs[j], gm._basis(j, q)))
                 for j in range(gm.N_JOINTS))


def test_fit_recovers_known_coeffs():
    # Ground truth: shoulder dominated by cos terms, elbow by cos(q0+q1),
    # with a small sensor bias and a tiny COM-offset sin term.
    truth = [[300.0, 5.0, 120.0, -3.0, 8.0],   # joint 0
             [90.0, 2.0, 4.0]]                  # joint 1
    # Build a grid of poses spanning the workspace.
    samples = []
    for q0 in [x * 0.3 for x in range(-5, 6)]:
        for q1 in [x * 0.3 for x in range(-6, 7)]:
            q = (q0, q1)
            samples.append((q, _synthetic_tau(truth, q)))
    model = gm.GravityModel()
    assert not model.is_identified()
    rms = model.fit(samples)
    for j in range(gm.N_JOINTS):
        assert rms[j] < 1e-6, (j, rms[j])
        for got, exp in zip(model.coeffs[j], truth[j]):
            assert _close(got, exp, 1e-4), (j, got, exp)
    assert model.is_identified()
    print(f"test_fit_recovers_known_coeffs OK (rms={['%.2e' % r for r in rms]})")


def test_predict_matches_truth():
    truth = [[250.0, 0.0, 100.0, 0.0, 0.0], [80.0, 0.0, 0.0]]
    model = gm.GravityModel(coeffs=[list(c) for c in truth])
    for q in [(0.0, 0.0), (0.5, -0.3), (-0.8, 1.1)]:
        pred = model.predict(q)
        ref = _synthetic_tau(truth, q)
        assert all(_close(a, b) for a, b in zip(pred, ref)), (q, pred, ref)
    # Sanity: at q=(0,0) shoulder load = c00+c02, elbow load = c10.
    assert _close(model.predict((0.0, 0.0))[0], 250.0 + 100.0)
    assert _close(model.predict((0.0, 0.0))[1], 80.0)
    print("test_predict_matches_truth OK")


def test_save_load_roundtrip():
    path = os.path.join(os.path.dirname(__file__), "_test_grav.json")
    truth = [[111.0, 1.0, 22.0, 2.0, 3.0], [44.0, 4.0, 5.0]]
    gm.GravityModel(coeffs=[list(c) for c in truth]).save(path)
    loaded = gm.GravityModel.load(path)
    os.remove(path)
    assert loaded is not None
    for j in range(gm.N_JOINTS):
        assert all(_close(a, b) for a, b in zip(loaded.coeffs[j], truth[j]))
    assert gm.GravityModel.load(os.path.join(os.path.dirname(__file__), "_nope.json")) is None
    print("test_save_load_roundtrip OK")


if __name__ == "__main__":
    test_fit_recovers_known_coeffs()
    test_predict_matches_truth()
    test_save_load_roundtrip()
    print("\nAll gravity_model tests passed.")
