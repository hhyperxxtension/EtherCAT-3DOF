"""
Lumped joint stiffness K_eff identification and deflection prediction.

Only the motor-side encoder exists, so elastic deflection (harmonic-drive
torsion + 3D-printed link bending, lumped per joint) is not measurable online
-- it is modelled. K_eff,i [motor per-mille / rad] relates a joint's load
torque (in the same motor per-mille units as gravity_model / 0x6077) to its
angular deflection:  delta_i = load_i / K_eff,i  [rad].

Calibration uses a known added tip load and a dial indicator AT THE TIP. The
tip vertical droop is the sum of both joints' deflections through the tip
Jacobian:

    dz_tip = J0*delta0 + J1*delta1
      J0 = d z_tip / d q0 = x_tip                  (horizontal reach to tip)
      J1 = d z_tip / d q1 = L2*cos(q0+q1)          (horizontal elbow->tip)
      delta_i = dm_i / K_i      (dm_i = per-mille load increment from the load)

  => dz_tip = (J0*dm0)*(1/K0) + (J1*dm1)*(1/K1)

linear in (1/K0, 1/K1). Probing several poses (different J/dm weighting)
separates the two stiffnesses from tip-only droop. fit() solves it; K_eff is
persisted to stiffness_calib.json; deflection() is used at runtime to bias the
joint target by the predicted sag.
"""

import json
import os

import kinematics as kin

try:
    from config import GEAR_RATIO
    N_JOINTS = len(GEAR_RATIO)
except ImportError:
    N_JOINTS = 2

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stiffness_calib.json")


def tip_jacobian(q):
    """(d z_tip/d q0, d z_tip/d q1) at pose q [mm/rad].
    J0 = x_tip; J1 = horizontal elbow->tip distance = L2*cos(q0+q1)."""
    x_tip = kin.forward(q)[0]
    x_elbow = kin.elbow_position(q)[0]
    return (x_tip, x_tip - x_elbow)


class StiffnessModel:
    """Per-joint lumped stiffness K_eff [per-mille / rad]."""

    def __init__(self, k=None):
        self.k = list(k) if k else [None] * N_JOINTS

    def is_identified(self):
        # At least one joint identified — allows PARTIAL compensation (a joint
        # whose K is None just gets zero deflection, see deflection()). This is
        # useful when e.g. only the dominant shoulder stiffness is measurable.
        return any(ki is not None for ki in self.k)

    def deflection(self, q, load_torque):
        """Predicted joint deflection [rad] under load_torque [per-mille] at
        pose q. Returns one value per joint (0 where K not identified)."""
        out = []
        for j in range(N_JOINTS):
            kj = self.k[j]
            out.append(0.0 if not kj else load_torque[j] / kj)
        return tuple(out)

    def fit(self, samples):
        """samples = [(q, dm_vec, dz_tip), ...] where dm_vec is the per-mille
        load increment per joint from the added tip load and dz_tip is the
        measured tip droop [mm]. Solves dz = sum_i (J_i*dm_i)*(1/K_i) for the
        per-joint K_eff. Returns RMS residual [mm]."""
        import numpy as np
        if len(samples) < N_JOINTS:
            raise ValueError(f"need >= {N_JOINTS} poses to separate {N_JOINTS} stiffnesses")
        A, y = [], []
        for q, dm, dz in samples:
            J = tip_jacobian(q)
            A.append([J[j] * dm[j] for j in range(N_JOINTS)])
            y.append(dz)
        A = np.array(A, dtype=float); y = np.array(y, dtype=float)
        inv_k, *_ = np.linalg.lstsq(A, y, rcond=None)
        self.k = [(float(1.0 / v) if v != 0 else None) for v in inv_k]
        resid = A @ inv_k - y
        return float(np.sqrt(np.mean(resid ** 2)))

    # ── persistence ─────────────────────────────────────────────────────
    def save(self, path=None):
        path = path or CALIB_FILE
        with open(path, "w") as f:
            json.dump({"k": self.k, "n_joints": N_JOINTS}, f, indent=2)

    @classmethod
    def load(cls, path=None):
        path = path or CALIB_FILE
        if not os.path.exists(path):
            return None
        try:
            with open(path) as f:
                data = json.load(f)
            k = data["k"]
            if len(k) != N_JOINTS:
                return None
            return cls(k)
        except (OSError, ValueError, KeyError):
            return None
