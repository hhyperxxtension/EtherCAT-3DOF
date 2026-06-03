"""
Data-driven static gravity (load-torque) model for the planar 2DOF arm.

Instead of CAD masses, we identify each joint's static load torque directly
from MEASURED de-frictioned motor torque (0x6077, in per-mille of rated) as a
function of pose. Fitting against the motor signal folds the harmonic-drive
ratio N and efficiency η into the coefficients, and keeps every joint in its
OWN motor-torque units — so no cross-joint reconciliation is needed, and the
deflection step δ_i = m_i / K_eff,i stays self-consistent (K_eff is calibrated
in the same units).

Physics of a vertical-plane 2DOF arm (gravity along -Z): the static load
torque at a joint is the horizontal moment of everything distal to it, so it
is a sum of cos(sum of angles) terms. We use a model that is LINEAR in its
parameters, with sin terms (to absorb a center-of-mass offset from the link
axis) and a constant (sensor/zero-offset, residual brake drag):

    m0(q) = c00*cos(q0) + c01*sin(q0)
          + c02*cos(q0+q1) + c03*sin(q0+q1) + c04
    m1(q) = c10*cos(q0+q1) + c11*sin(q0+q1) + c12

Coefficients are found by least squares from (pose, measured-load-torque)
samples (see fit()), persisted to gravity_calib.json, and used at runtime by
predict() to feed the deflection correction. Units throughout: motor per-mille
(same as 0x6077), per joint.

A payload at the tip just adds to the same cos/sin(q0+q1) terms, so the
payload observer later nudges c02/c03 (joint 0) and c10/c11 (joint 1); the
structural fit here is done with the nominal (no extra) payload.
"""

import json
import math
import os

try:
    from config import GEAR_RATIO
    N_JOINTS = len(GEAR_RATIO)
except ImportError:
    N_JOINTS = 2

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gravity_calib.json")


def _basis(joint, q):
    """Regression basis (list of feature values) for a joint's load torque.
    Linear-in-parameters; the fitted coefficient vector dots with this."""
    q0, q1 = q[0], q[1]
    if joint == 0:   # shoulder: moments of link1, link2+payload
        s = q0 + q1
        return [math.cos(q0), math.sin(q0), math.cos(s), math.sin(s), 1.0]
    elif joint == 1:  # elbow: moment of link2+payload only
        s = q0 + q1
        return [math.cos(s), math.sin(s), 1.0]
    raise ValueError(f"joint {joint} out of range for this 2DOF model")


# Number of coefficients per joint, must match _basis lengths.
_NCOEF = (5, 3)


class GravityModel:
    """Per-joint static load-torque predictor (motor per-mille)."""

    def __init__(self, coeffs=None):
        # coeffs[j] = list of floats for joint j; zeros = "not yet identified".
        self.coeffs = coeffs or [[0.0] * _NCOEF[j] for j in range(N_JOINTS)]

    def is_identified(self):
        return any(any(c != 0.0 for c in cj) for cj in self.coeffs)

    def predict(self, q):
        """Predicted static load torque per joint (motor per-mille) at pose q."""
        out = []
        for j in range(N_JOINTS):
            b = _basis(j, q)
            out.append(sum(ci * bi for ci, bi in zip(self.coeffs[j], b)))
        return tuple(out)

    def fit(self, samples):
        """Least-squares fit from samples = [(q, tau_vec), ...] where tau_vec
        is the measured de-frictioned load torque per joint (motor per-mille).
        Returns per-joint RMS residual."""
        import numpy as np
        if len(samples) < max(_NCOEF):
            raise ValueError(f"need >= {max(_NCOEF)} samples to fit")
        rms = []
        for j in range(N_JOINTS):
            A = np.array([_basis(j, q) for q, _ in samples], dtype=float)
            y = np.array([tau[j] for _, tau in samples], dtype=float)
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            self.coeffs[j] = coef.tolist()
            resid = A @ coef - y
            rms.append(float(np.sqrt(np.mean(resid ** 2))))
        return rms

    # ── persistence ─────────────────────────────────────────────────────
    def save(self, path=None):
        path = path or CALIB_FILE
        with open(path, "w") as f:
            json.dump({"coeffs": self.coeffs, "n_joints": N_JOINTS}, f, indent=2)

    @classmethod
    def load(cls, path=None):
        """Load a persisted model; returns None if absent/invalid."""
        path = path or CALIB_FILE
        if not os.path.exists(path):
            return None
        try:
            with open(path) as f:
                data = json.load(f)
            coeffs = data["coeffs"]
            if len(coeffs) != N_JOINTS:
                return None
            return cls([[float(c) for c in cj] for cj in coeffs])
        except (OSError, ValueError, KeyError):
            return None
