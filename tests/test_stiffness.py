"""Offline tests for stiffness.py — run with plain `python`, no pytest."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import stiffness as st


def _close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _synth_dz(K, q, dm):
    """Tip droop the model would produce for known K, pose, load increment."""
    J = st.tip_jacobian(q)
    return sum(J[j] * dm[j] / K[j] for j in range(st.N_JOINTS))


def test_fit_recovers_known_stiffness():
    K_true = [1500.0, 600.0]   # per-mille/rad
    # A few poses with different geometry + per-joint load increments.
    poses = [(0.3, -0.5), (0.8, 0.2), (1.2, -0.9), (0.5, 0.6)]
    dms = [[80.0, 30.0], [120.0, 45.0], [60.0, 20.0], [100.0, 38.0]]
    samples = [(q, dm, _synth_dz(K_true, q, dm)) for q, dm in zip(poses, dms)]
    model = st.StiffnessModel()
    assert not model.is_identified()
    rms = model.fit(samples)
    assert rms < 1e-9, rms
    for got, exp in zip(model.k, K_true):
        assert _close(got, exp, 1e-3), (got, exp)
    assert model.is_identified()
    print(f"test_fit_recovers_known_stiffness OK (K={[round(k,1) for k in model.k]})")


def test_deflection_direction():
    model = st.StiffnessModel(k=[1000.0, 500.0])
    # delta = load / K
    d = model.deflection((0.5, 0.3), (200.0, 50.0))
    assert _close(d[0], 0.2) and _close(d[1], 0.1), d
    print("test_deflection_direction OK")


def test_jacobian_known_pose():
    # At q=(0,0): x_tip = L1+L2, elbow x = L1 -> J1 = L2.
    l1, l2 = kin_links()
    J = st.tip_jacobian((0.0, 0.0))
    assert _close(J[0], l1 + l2, 1e-6) and _close(J[1], l2, 1e-6), J
    print("test_jacobian_known_pose OK")


def kin_links():
    import kinematics as kin
    return kin.LINK_LENGTHS


def test_save_load_roundtrip():
    path = os.path.join(os.path.dirname(__file__), "_test_stiff.json")
    st.StiffnessModel(k=[1234.0, 567.0]).save(path)
    loaded = st.StiffnessModel.load(path)
    os.remove(path)
    assert loaded is not None and loaded.k == [1234.0, 567.0]
    assert st.StiffnessModel.load(os.path.join(os.path.dirname(__file__), "_nope.json")) is None
    print("test_save_load_roundtrip OK")


if __name__ == "__main__":
    test_fit_recovers_known_stiffness()
    test_deflection_direction()
    test_jacobian_known_pose()
    test_save_load_roundtrip()
    print("\nAll stiffness tests passed.")
