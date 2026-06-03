"""
Static elastic-deflection compensation.

Combines the identified gravity load model G(q) and the lumped joint stiffness
K_eff into a feedforward joint-angle correction. Under load each joint sags by

    delta_i = G_i(q) / K_eff,i        [rad]   (G in per-mille, K in per-mille/rad)

With only a motor-side encoder the link lags the motor by delta_i in the load
direction (torsional spring: theta_link = theta_motor - tau_load/K). So to land
the LINK at q_target we command the MOTOR to

    q_cmd = q_target + delta(q_target)

i.e. pre-bias the motor by the predicted sag. This is open-loop feedforward
from the model (stable, no torque-loop noise); the live 0x6077 (with its ~100
per-mille friction band) is NOT in this path.

NOTE the correction sign relies on G and K_eff sharing the same per-mille /
JOINT_DIR convention (they do — both fit against the same motor torque signal).
Validate on hardware: tip error at a loaded pose should shrink with comp ON;
if it grows, a sign is off.
"""

import joint_map as jm
from gravity_model import GravityModel
from stiffness import StiffnessModel, tip_jacobian
from dynamics import DynamicsModel

try:
    from config import OBS_LP_ALPHA, OBS_DELTA_MAX_RAD
except ImportError:
    OBS_LP_ALPHA, OBS_DELTA_MAX_RAD = 0.97, 0.20


class DeflectionCompensator:
    def __init__(self, gravity, stiffness):
        self.gravity = gravity
        self.stiffness = stiffness

    @classmethod
    def load(cls):
        """Build from persisted gravity + stiffness calibrations, or None if
        either is missing/unidentified."""
        g = GravityModel.load()
        s = StiffnessModel.load()
        if g is None or not g.is_identified() or s is None or not s.is_identified():
            return None
        return cls(g, s)

    def correction(self, q):
        """Predicted joint deflection delta_i [rad] at pose q (the amount to
        add to the motor command). 0 for any joint whose K is unidentified."""
        load = self.gravity.predict(q)
        return self.stiffness.deflection(q, load)

    def compensate(self, q_target):
        """Return (q_cmd, delta): the deflection-corrected joint command and
        the applied correction."""
        d = self.correction(q_target)
        return tuple(qi + di for qi, di in zip(q_target, d)), tuple(d)


class LiveCompensator:
    """Per-cycle ONLINE observer for adaptive compensation during motion.

    Each control cycle it: (1) de-inertias/de-frictions the measured torque
    with the dynamic model to get the live load residual, (2) estimates the tip
    payload as a single scalar c_p via the tip Jacobian, (3) low-pass filters
    c_p, (4) forms delta_i = [G_i(q) + c_p*J_i(q)] / K_eff,i, and returns it as a
    per-joint COUNT offset to add to the streamed setpoint. The trajectory's
    analytic (q, q̇, q̈) is supplied up front (samples_full), so no on-bus
    differentiation is needed; the torque used is the previous cycle's reading
    (one-cycle lag, negligible).

    Built to be handed to CSPBus.play_trajectory(corrector=...). Stateful
    (cycle index + filtered c_p), so create a fresh one per move.
    """

    def __init__(self, gravity, stiffness, dynamics, home_counts,
                 lp_alpha=OBS_LP_ALPHA, delta_max=OBS_DELTA_MAX_RAD,
                 ramp_s=0.4, dt=0.004):
        self.g = gravity
        self.s = stiffness
        self.d = dynamics
        self.home = home_counts
        self.alpha = lp_alpha
        self.delta_max = delta_max
        self.n = len(home_counts)
        self.ramp_cycles = max(1, int(ramp_s / dt))
        self.sf = []
        self.i = 0
        self.cp = 0.0                 # filtered payload scalar (persists)
        self.engaged = 0              # ramp counter (persists across moves)
        self.last_delta = tuple(0.0 for _ in range(self.n))
        self.last_cp = 0.0

    def new_move(self, samples_full):
        """Attach the trajectory of the next move; keeps cp + ramp so the
        correction does NOT drop to zero between moves."""
        self.sf = samples_full
        self.i = 0

    def __call__(self, status):
        """status: list of per-drive dicts with 'trq'. Returns per-joint count
        offsets (ints) to add to this cycle's nominal setpoint."""
        if not self.sf:
            return [0] * self.n
        k = self.i if self.i < len(self.sf) else len(self.sf) - 1
        self.i += 1
        _t, q, qd, qdd = self.sf[k]
        tau = [status[j]['trq'] for j in range(self.n)]
        # Live load attributable to payload = measured - model(no payload).
        r = self.d.payload_residual(q, qd, qdd, tau)
        J = tip_jacobian(q)
        denom = sum(j * j for j in J) or 1.0
        cp_raw = sum(J[j] * r[j] for j in range(self.n)) / denom
        self.cp = self.alpha * self.cp + (1.0 - self.alpha) * cp_raw
        self.last_cp = self.cp
        # Smooth engagement: ramp the correction in over ramp_cycles the first
        # time it runs (avoids a step when entering adaptive at a drooped hold);
        # stays at 1 for subsequent moves.
        gain = min(1.0, self.engaged / self.ramp_cycles)
        self.engaged += 1
        G = self.g.predict(q)
        offs, deltas = [], []
        for j in range(self.n):
            kj = self.s.k[j]
            load = G[j] + self.cp * J[j]
            delta = gain * (load / kj) if kj else 0.0    # rad
            if delta > self.delta_max:
                delta = self.delta_max
            elif delta < -self.delta_max:
                delta = -self.delta_max
            deltas.append(delta)
            offs.append(int(round(delta * jm.counts_per_rad(j))))
        self.last_delta = tuple(deltas)
        return offs

    @classmethod
    def available(cls):
        """True if all three models needed for live adaptation are present."""
        g = GravityModel.load()
        s = StiffnessModel.load()
        if g is None or not g.is_identified() or s is None or not s.is_identified():
            return False
        return DynamicsModel.load(g) is not None
