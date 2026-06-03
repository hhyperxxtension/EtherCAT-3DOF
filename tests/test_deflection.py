"""Offline tests for deflection.py — run with plain `python`, no pytest."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import gravity_model as gm
import stiffness as st
from deflection import DeflectionCompensator


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_correction_is_load_over_k():
    # Gravity coeffs so G is easy to reason about; constant K.
    g = gm.GravityModel(coeffs=[[100.0, 0.0, 50.0, 0.0, 0.0], [40.0, 0.0, 0.0]])
    s = st.StiffnessModel(k=[2000.0, 800.0])
    comp = DeflectionCompensator(g, s)
    q = (0.0, 0.0)
    G = g.predict(q)                  # (150, 40)
    d = comp.correction(q)
    assert _close(d[0], G[0] / 2000.0) and _close(d[1], G[1] / 800.0), (d, G)
    # compensate() adds the correction to the target.
    q_cmd, delta = comp.compensate(q)
    assert _close(q_cmd[0], q[0] + d[0]) and _close(q_cmd[1], q[1] + d[1])
    assert delta == d
    print("test_correction_is_load_over_k OK")


def test_unidentified_stiffness_gives_zero():
    g = gm.GravityModel(coeffs=[[100.0, 0.0, 50.0, 0.0, 0.0], [40.0, 0.0, 0.0]])
    s = st.StiffnessModel(k=[2000.0, None])   # elbow not identified
    comp = DeflectionCompensator(g, s)
    d = comp.correction((0.3, -0.2))
    assert d[1] == 0.0, d        # no correction where K is missing
    assert d[0] != 0.0
    print("test_unidentified_stiffness_gives_zero OK")


if __name__ == "__main__":
    test_correction_is_load_over_k()
    test_unidentified_stiffness_gives_zero()
    print("\nAll deflection tests passed.")
