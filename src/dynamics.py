"""
Rigid-body dynamic model for the planar 2DOF arm, used to predict the
"no-payload" motor torque during motion so the live payload can be observed as
the residual (see the adaptive compensation in controller/CSPBus).

Full per-joint torque (motor per-mille, each joint in its own units):

    tau_i = G_i(q)                              gravity (from gravity_model)
          + [inertia + Coriolis]_i(q,q̇,q̈)
          + f_i*sign(q̇_i) + nu_i*q̇_i           friction (Coulomb + viscous)
          + payload_i                           (observed online, not modelled here)

For a planar 2R arm (p1=I1+m1 lc1^2+m2 l1^2, p2=I2+m2 lc2^2, p3=m2 l1 lc2):

    tau0 - G0 = P_a*q̈0 + P_b*q̈1
              + P_c*[cos q1*(2 q̈0+q̈1) - sin q1*(2 q̇0 q̇1+q̇1^2)]
              + f0*sign(q̇0) + nu0*q̇0
    tau1 - G1 = Q_b*(q̈0+q̈1) + Q_c*[cos q1*q̈0 + sin q1*q̇0^2]
              + f1*sign(q̇1) + nu1*q̇1

(physically P_a=p1+p2, P_b=Q_b=p2, P_c=Q_c=p3, modulo the per-joint per-mille
scale; left free in the fit, with P_c/Q_c a consistency check on p3).

Linear in parameters. The trajectory supplies q,q̇,q̈ analytically (no noisy
differentiation), so identification (dyncal) is a clean least squares of the
residual tau - G(q) against the basis below, on excitation moves with VARYING
acceleration (and no extra payload). Gravity G(q) is reused as-is from gravcal.
"""

import json
import math
import os

from gravity_model import GravityModel

try:
    from config import GEAR_RATIO
    N_JOINTS = len(GEAR_RATIO)
except ImportError:
    N_JOINTS = 2

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dynamics_calib.json")


def _sgn(v):
    return (v > 0.0) - (v < 0.0)


def dyn_basis(joint, q, qd, qdd):
    """Non-gravity (inertia + Coriolis + friction) regressor row for a joint."""
    q1 = q[1]
    qd0, qd1 = qd[0], qd[1]
    qdd0, qdd1 = qdd[0], qdd[1]
    c, s = math.cos(q1), math.sin(q1)
    if joint == 0:
        return [qdd0,
                qdd1,
                c * (2 * qdd0 + qdd1) - s * (2 * qd0 * qd1 + qd1 * qd1),
                _sgn(qd0),
                qd0]
    elif joint == 1:
        return [qdd0 + qdd1,
                c * qdd0 + s * qd0 * qd0,
                _sgn(qd1),
                qd1]
    raise ValueError(f"joint {joint} out of range")


_NDYN = (5, 4)


class DynamicsModel:
    """Predicts no-payload torque = G(q) + inertia/Coriolis/friction."""

    def __init__(self, gravity, coeffs=None):
        self.gravity = gravity
        self.coeffs = coeffs or [[0.0] * _NDYN[j] for j in range(N_JOINTS)]

    def is_identified(self):
        return any(any(c != 0.0 for c in cj) for cj in self.coeffs)

    def dyn_torque(self, q, qd, qdd):
        """Inertia + Coriolis + friction contribution per joint (no gravity)."""
        out = []
        for j in range(N_JOINTS):
            b = dyn_basis(j, q, qd, qdd)
            out.append(sum(ci * bi for ci, bi in zip(self.coeffs[j], b)))
        return tuple(out)

    def predict(self, q, qd, qdd):
        """Full no-payload torque per joint (motor per-mille)."""
        G = self.gravity.predict(q)
        d = self.dyn_torque(q, qd, qdd)
        return tuple(G[j] + d[j] for j in range(N_JOINTS))

    def payload_residual(self, q, qd, qdd, tau_meas):
        """tau_meas - predict(): the part attributable to an unmodelled tip
        payload, per joint (motor per-mille)."""
        p = self.predict(q, qd, qdd)
        return tuple(tau_meas[j] - p[j] for j in range(N_JOINTS))

    def fit(self, samples):
        """samples = [(q, qd, qdd, tau), ...] from excitation moves with NO
        extra payload. Fits the dynamic coeffs to the residual tau - G(q).
        Returns per-joint RMS residual [per-mille]."""
        import numpy as np
        if len(samples) < max(_NDYN):
            raise ValueError(f"need >= {max(_NDYN)} samples")
        rms = []
        for j in range(N_JOINTS):
            A = np.array([dyn_basis(j, q, qd, qdd) for q, qd, qdd, _ in samples], float)
            y = np.array([tau[j] - self.gravity.predict(q)[j]
                          for q, _, _, tau in samples], float)
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            self.coeffs[j] = coef.tolist()
            rms.append(float(np.sqrt(np.mean((A @ coef - y) ** 2))))
        return rms

    # ── persistence ─────────────────────────────────────────────────────
    def save(self, path=None):
        path = path or CALIB_FILE
        with open(path, "w") as f:
            json.dump({"coeffs": self.coeffs, "n_joints": N_JOINTS}, f, indent=2)

    @classmethod
    def load(cls, gravity=None, path=None):
        """Load dynamic coeffs; needs a GravityModel (loads one if not given).
        Returns None if absent/invalid."""
        path = path or CALIB_FILE
        if gravity is None:
            gravity = GravityModel.load()
        if gravity is None or not gravity.is_identified():
            return None
        if not os.path.exists(path):
            return None
        try:
            with open(path) as f:
                data = json.load(f)
            coeffs = data["coeffs"]
            if len(coeffs) != N_JOINTS:
                return None
            return cls(gravity, [[float(c) for c in cj] for cj in coeffs])
        except (OSError, ValueError, KeyError):
            return None
