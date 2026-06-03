"""
Cartesian (X-Z) point-to-point controller for the planar 2DOF arm.

Wires the pieces together:
    (x, z)  --inverse kinematics-->  q_target
    q_start = current joint angles (from encoder counts)
    plan_ptp(q_start, q_target)      -> synchronized trapezoidal trajectory
    sample at the bus cycle time     -> q(t) per cycle
    joint_map.angles_to_counts       -> raw 0x607A counts per cycle
    CSPBus.play_trajectory           -> RT thread streams them to the drives

HOMING (required before any move): the absolute encoder reports an arbitrary
raw count at power-up, so the controller must be told which counts correspond
to a known pose. Put the arm at the JOINT_ZERO_RAD pose (config; default q=0,0
= link 1 along +X, elbow straight) and run `home`; the current counts are
captured as the reference. Joint angles are then computed relative to it.

CLI (python controller.py):
    home                set current pose as the JOINT_ZERO_RAD reference
    where               print current joint angles, (x, z), and raw counts
    move <x> <z>        move the end effector to (x, z) metres (blocks)
    elbow up|down       choose IK branch for subsequent moves
    speed <v> <a>       set joint vel/accel limits (rad/s, rad/s²)
    stop                halt motion, hold position
    q                   quit (disable drives, close bus)

SAFETY: a move is rejected before any motion if the target is unreachable,
violates a joint software limit, or would exceed the drive's 32-bit position
range. Speed is bounded by the planner (JOINT_VEL_MAX/ACC_MAX) and, underneath,
by CSPBus's hard per-cycle cap and the 0x60E0/0x60E1 torque limit.
"""

import json
import math
import os
import sys
import time

import kinematics as kin
import joint_map as jm
from trajectory import plan_ptp
from ethercat_csp import CSPBus
from gravity_model import GravityModel
from stiffness import StiffnessModel, tip_jacobian
from dynamics import DynamicsModel
from deflection import DeflectionCompensator, LiveCompensator
from config import (
    JOINT_LIMITS, JOINT_VEL_MAX, JOINT_ACC_MAX, ELBOW_CONFIG,
    JOINT_ZERO_RAD, PROBE_SWEEP_DEG, PROBE_SPEED_DPS,
    Z_FLOOR_MM, Z_FLOOR_TOL_MM, GRAVCAL_GRID_N, GRAVCAL_APPROACH_DPS,
    STIFFCAL_LOAD_KG, STIFFCAL_N, STIFFCAL_G_HEADROOM,
    STIFFCAL_SWEEP_DEG, STIFFCAL_SPEED_DPS,
    DYNCAL_GRID_N, DYNCAL_VMAX, DYNCAL_ACCELS,
    G_ACCEL, STIFF_DROOP_POSES_DEG, STIFF_SCALE_POSES_DEG,
)

SAMPLES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gravity_samples.json")
STIFF_SAMPLES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stiffness_samples.json")
DYN_SAMPLES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dynamics_samples.json")
STIFF_PHYS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stiffness_phys.json")

# Persisted home reference. The Y7 encoder is absolute: while the drives stay
# powered, raw 0x6064 counts are continuous, so a home captured in an earlier
# session is still valid (master may restart, drives must NOT have been
# powered off). Stored next to this module.
HOME_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "home_calib.json")


class NotHomed(RuntimeError):
    pass


class RobotController:
    def __init__(self, bus=None):
        self.bus = bus or CSPBus()
        self.home_counts = None
        self.elbow = ELBOW_CONFIG
        self.v_max = JOINT_VEL_MAX
        self.a_max = JOINT_ACC_MAX
        # Deflection compensation. comp_mode in {'off','static','adaptive'}:
        #   static   = feedforward q_cmd = q_target + G(q)/K_eff (own weight)
        #   adaptive = live payload observer during motion (G + dynamics + K)
        self.last_correction = None   # last static delta [rad] (reporting)
        self.last_live = None         # last LiveCompensator (adaptive reporting)
        self._live_comp = None        # persistent live observer (carries c_p/ramp)
        self._load_models()
        self.comp_mode = self._default_mode()

    def _load_models(self):
        self.gravity = GravityModel.load()
        self.stiffness = StiffnessModel.load()
        self.dynamics = DynamicsModel.load(self.gravity) if self.gravity else None
        self.compensator = DeflectionCompensator.load()

    def _static_ok(self):
        return self.compensator is not None

    def _adaptive_ok(self):
        return (self.gravity is not None and self.gravity.is_identified()
                and self.stiffness is not None and self.stiffness.is_identified()
                and self.dynamics is not None)

    def _default_mode(self):
        # Compensation is OFF by default — the user enables static/adaptive
        # explicitly via the `comp` command once calibration is trusted.
        return "off"

    # ── lifecycle ───────────────────────────────────────────────────────
    def connect(self):
        self.bus.connect()
        self._load_models()           # calib files may have changed
        # Downgrade the mode if its models are no longer available.
        if self.comp_mode == "adaptive" and not self._adaptive_ok():
            self.comp_mode = "static" if self._static_ok() else "off"
        elif self.comp_mode == "static" and not self._static_ok():
            self.comp_mode = "off"

    def disconnect(self):
        self.bus.disconnect()

    # ── homing / state ──────────────────────────────────────────────────
    def set_home(self, persist=True):
        """Capture current raw counts as the JOINT_ZERO_RAD reference and
        (by default) persist them so the next session can skip homing."""
        self.home_counts = list(self.bus.actual_counts())
        if persist:
            self._save_home()
        return self.home_counts

    def _save_home(self):
        try:
            with open(HOME_FILE, "w") as f:
                json.dump({"home_counts": self.home_counts,
                           "n": self.bus.n,
                           "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, f)
        except OSError as e:
            print(f"  [WARN] could not save home: {e}")

    def load_home(self):
        """Restore a persisted home if present and structurally valid.
        Returns (ok, info). Does NOT verify the drives kept power — call
        home_plausible() for a sanity check before trusting it."""
        if not os.path.exists(HOME_FILE):
            return False, "no saved home"
        try:
            with open(HOME_FILE) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            return False, f"unreadable home file: {e}"
        hc = data.get("home_counts")
        if not isinstance(hc, list) or len(hc) != self.bus.n:
            return False, "saved home does not match drive count"
        self.home_counts = [int(c) for c in hc]
        return True, data.get("saved", "?")

    def home_plausible(self):
        """True if the current pose under the (restored) home lands within the
        joint limits and inside the workspace — a guard against a stale home
        from a drive that lost position (was powered off / reset)."""
        if self.home_counts is None:
            return False
        q = jm.counts_to_angles(self.bus.actual_counts(), self.home_counts)
        return kin.within_joint_limits(q, JOINT_LIMITS) and kin.reachable(*kin.forward(q))

    def clear_home(self):
        self.home_counts = None
        try:
            os.remove(HOME_FILE)
        except OSError:
            pass

    def _require_home(self):
        if self.home_counts is None:
            raise NotHomed("not homed - run `home` with the arm at the zero pose first")

    def current_q(self):
        self._require_home()
        return jm.counts_to_angles(self.bus.actual_counts(), self.home_counts)

    def current_xz(self):
        return kin.forward(self.current_q())

    # ── motion ──────────────────────────────────────────────────────────
    def _compensate(self, q_target):
        """STATIC feedforward: q_cmd = q_target + G(q_target)/K_eff. Validates
        q_cmd against the joint limits (the bias can push a near-limit target
        past it). Records the correction in self.last_correction. Returns q_cmd."""
        q_cmd, delta = self.compensator.compensate(q_target)
        self.last_correction = delta
        if not kin.within_joint_limits(q_cmd, JOINT_LIMITS):
            raise ValueError(f"compensated target {_deg(q_cmd)} deg exceeds joint "
                             "limits (deflection bias near a limit)")
        return q_cmd

    def _sf_to_vectors(self, samples_full):
        """Count-vectors from (t,q,q̇,q̈) samples, with the INT32 guard."""
        vectors = []
        for _t, q, _qd, _qdd in samples_full:
            c = jm.angles_to_counts(q, self.home_counts)
            for ci in c:
                if not jm.counts_in_int32(ci):
                    raise ValueError(f"count {ci} exceeds drive 32-bit range")
            vectors.append(c)
        return vectors

    def _traj_to_vectors(self, traj):
        """Count-vectors from a trajectory (position only), with INT32 guard."""
        vectors = []
        for _t, q in traj.samples(dt=self.bus.cycle_time):
            c = jm.angles_to_counts(q, self.home_counts)
            for ci in c:
                if not jm.counts_in_int32(ci):
                    raise ValueError(f"count {ci} exceeds drive 32-bit range")
            vectors.append(c)
        return vectors

    def _go(self, q_target):
        """Plan and start a move to validated joint target q_target, applying
        the active compensation mode. Returns duration [s]. Non-blocking."""
        self.last_correction = None
        self.last_live = None
        q_start = self.current_q()
        if self.comp_mode == "adaptive" and self._adaptive_ok():
            if self._live_comp is None:       # persistent: carries c_p + ramp
                self._live_comp = LiveCompensator(self.gravity, self.stiffness,
                                                  self.dynamics, self.home_counts,
                                                  dt=self.bus.cycle_time)
            # The measured current_q already INCLUDES the correction applied at
            # the end of the last move (it is baked into the motor position).
            # Plan the nominal (raw) trajectory from the LINK position
            # (current_q - last applied delta) so that nominal[0] + corrector
            # offset == the current motor position -> NO step/jerk at start.
            last_d = self._live_comp.last_delta
            q_start_link = tuple(q_start[j] - last_d[j] for j in range(self.bus.n))
            traj = plan_ptp(q_start_link, q_target, self.v_max, self.a_max)
            sf = traj.samples_full(self.bus.cycle_time)
            vectors = self._sf_to_vectors(sf)
            self._live_comp.new_move(sf)
            self.last_live = self._live_comp
            self.bus.play_trajectory(vectors, corrector=self._live_comp)
            return traj.duration
        if self.comp_mode == "static" and self._static_ok():
            q_cmd = self._compensate(q_target)            # endpoint feedforward
            traj = plan_ptp(q_start, q_cmd, self.v_max, self.a_max)
        else:                                             # off
            traj = plan_ptp(q_start, q_target, self.v_max, self.a_max)
        self.bus.play_trajectory(self._sf_to_vectors(traj.samples_full(self.bus.cycle_time)))
        return traj.duration

    def plan_move(self, x, z):
        """Validate (x,z) and return (q_target). Raises if unsafe."""
        self._require_home()
        if not kin.reachable(x, z):
            rmin, rmax = kin.workspace_bounds()
            raise ValueError(f"({x:.1f}, {z:.1f}) mm unreachable - workspace r in [{rmin:.1f}, {rmax:.1f}] mm")
        q_target = kin.inverse(x, z, elbow=self.elbow)   # raises Unreachable
        if not kin.within_joint_limits(q_target, JOINT_LIMITS):
            raise ValueError(f"target joints {_deg(q_target)} violate joint limits")
        return q_target

    def move_to(self, x, z):
        """Plan, validate, and start a move to (x, z) [mm]. Returns duration
        [s]. Non-blocking — poll bus.is_busy() or call wait()."""
        return self._go(self.plan_move(x, z))

    def move_by(self, dx, dz):
        """Relative move: target = current (x, z) + (dx, dz), all in mm."""
        x0, z0 = self.current_xz()
        return self.move_to(x0 + dx, z0 + dz)

    def move_joints(self, q_target):
        """Direct joint-space move to absolute joint angles [rad]. Validates
        joint limits before motion. Returns duration [s]."""
        self._require_home()
        q_target = tuple(q_target)
        if len(q_target) != self.bus.n:
            raise ValueError(f"expected {self.bus.n} joint angles")
        if not kin.within_joint_limits(q_target, JOINT_LIMITS):
            raise ValueError(f"target joints {_deg(q_target)} deg violate joint limits")
        return self._go(q_target)

    def move_joints_by(self, dq):
        """Relative joint-space move by dq [rad] from the current angles."""
        q = self.current_q()
        return self.move_joints(tuple(qi + dqi for qi, dqi in zip(q, dq)))

    def correction_mm(self):
        """Tip Cartesian correction (dx, dz, magnitude) [mm] currently applied
        by the active compensation, at the current pose; None if no comp."""
        if self.comp_mode == "static" and self.last_correction:
            delta = self.last_correction
        elif self.comp_mode == "adaptive" and self.last_live is not None:
            delta = self.last_live.last_delta
        else:
            return None
        try:
            q = self.current_q()
        except NotHomed:
            return None
        dx, dz = kin.tip_offset(q, delta)
        return (dx, dz, math.hypot(dx, dz))

    def wait(self, poll=0.05):
        while self.bus.is_busy():
            time.sleep(poll)

    def stop(self):
        self.bus.freeze()

    # ── joint-space play + torque probe (deflection calibration) ─────────
    def _play_joints(self, q_from, q_to, v_max, a_max, sample_joint=None):
        """Play a joint-space PTP move and block until done. If sample_joint
        is set, collect that joint's torque (0x6077) over the cruise portion
        and return its mean; else return None."""
        traj = plan_ptp(q_from, q_to, v_max, a_max)
        self.bus.play_trajectory(self._traj_to_vectors(traj))
        torques = []
        while self.bus.is_busy():
            if sample_joint is not None:
                snap, *_ = self.bus.status()
                torques.append(snap[sample_joint]['trq'])
            time.sleep(0.01)
        if sample_joint is None:
            return None
        # Trim the accel/decel ends (keep middle 60%) so only ~constant-speed
        # samples enter the average.
        n = len(torques)
        mid = torques[int(0.2 * n):int(0.8 * n)] or torques
        return sum(mid) / len(mid)

    def probe_joint(self, j, sweep_deg=PROBE_SWEEP_DEG, speed_dps=PROBE_SPEED_DPS):
        """Bidirectional slow sweep of joint j about the current pose. Returns
        (load_torque, friction) in motor per-mille:
            load = (fwd + rev)/2   friction = (fwd - rev)/2
        Coulomb+viscous friction (odd in velocity) cancels in `load`."""
        self._require_home()
        q_c = list(self.current_q())
        sweep = math.radians(sweep_deg)
        cruise = math.radians(speed_dps)
        q_lo = list(q_c); q_lo[j] -= sweep
        q_hi = list(q_c); q_hi[j] += sweep
        # Never command past a soft joint limit, even if the pose is near one.
        lo_lim, hi_lim = JOINT_LIMITS[j]
        q_lo[j] = max(q_lo[j], lo_lim)
        q_hi[j] = min(q_hi[j], hi_lim)
        # Approach the low end at the normal slow speed.
        self._play_joints(q_c, q_lo, self.v_max, self.a_max)
        # Forward (lo->hi) and reverse (hi->lo) at the probe cruise speed.
        t_fwd = self._play_joints(q_lo, q_hi, cruise, self.a_max, sample_joint=j)
        t_rev = self._play_joints(q_hi, q_lo, cruise, self.a_max, sample_joint=j)
        # Back to the starting pose.
        self._play_joints(q_lo, q_c, self.v_max, self.a_max)
        return 0.5 * (t_fwd + t_rev), 0.5 * (t_fwd - t_rev)

    def probe_pose(self, sweep_deg=PROBE_SWEEP_DEG, speed_dps=PROBE_SPEED_DPS):
        """Probe every joint at the current pose. Returns (q_center, load_vec,
        friction_vec) — load_vec is the de-frictioned load torque per joint,
        ready to feed gravity_model.fit() as one (q, tau) sample."""
        self._require_home()
        q_c = self.current_q()
        load, fric = [], []
        for j in range(self.bus.n):
            l, f = self.probe_joint(j, sweep_deg, speed_dps)
            load.append(l); fric.append(f)
        return q_c, load, fric

    # ── gravity calibration (autonomous grid sweep) ─────────────────────
    @staticmethod
    def _pose_floor_ok(q, z_floor):
        """Tip AND elbow stay at/above the Cartesian floor (mm), within a small
        tolerance (the home pose sits exactly at z=0)."""
        lim = z_floor - Z_FLOOR_TOL_MM
        return kin.elbow_position(q)[1] >= lim and kin.forward(q)[1] >= lim

    def _pose_safe(self, q, z_floor, sweep):
        """Pose is within joint limits, above the floor, and its +/-sweep probe
        perturbations on each joint also stay above the floor."""
        if not kin.within_joint_limits(q, JOINT_LIMITS):
            return False
        if not self._pose_floor_ok(q, z_floor):
            return False
        for j in range(len(q)):
            for s in (sweep, -sweep):
                qp = list(q); qp[j] += s
                if not self._pose_floor_ok(qp, z_floor):
                    return False
        return True

    def _traj_floor_ok(self, traj, z_floor):
        return all(self._pose_floor_ok(q, z_floor)
                   for _t, q in traj.samples(dt=self.bus.cycle_time))

    def grav_grid(self, n0=GRAVCAL_GRID_N, n1=GRAVCAL_GRID_N,
                  z_floor=Z_FLOOR_MM, sweep_deg=PROBE_SWEEP_DEG):
        """Build the safe serpentine pose list for gravity calibration:
        n0 x n1 over the joint limits (shrunk by sweep+margin), keeping only
        poses whose pose AND probe-sweep extremes clear the floor."""
        sweep = math.radians(sweep_deg)
        margin = sweep + math.radians(1.0)
        (lo0, hi0), (lo1, hi1) = JOINT_LIMITS
        q0s = _linspace(lo0 + margin, hi0 - margin, n0)
        q1s = _linspace(lo1 + margin, hi1 - margin, n1)
        grid = []
        for i, q0 in enumerate(q0s):
            row = q1s if i % 2 == 0 else list(reversed(q1s))  # serpentine
            for q1 in row:
                q = (q0, q1)
                if self._pose_safe(q, z_floor, sweep):
                    grid.append(q)
        return grid

    def grav_calibrate(self, grid=None, sweep_deg=PROBE_SWEEP_DEG,
                       speed_dps=PROBE_SPEED_DPS, z_floor=Z_FLOOR_MM,
                       progress=None):
        """Visit each pose in `grid` (or a default grid), probe load+friction,
        fit the gravity model, persist it + the raw samples. Returns
        (model, rms, samples, raw). Skips any pose whose approach path would
        dip below the floor. Raises if too few usable samples."""
        self._require_home()
        if grid is None:
            grid = self.grav_grid(sweep_deg=sweep_deg, z_floor=z_floor)
        av = math.radians(GRAVCAL_APPROACH_DPS)
        samples, raw = [], []
        for k, q in enumerate(grid):
            if progress:
                progress(k, len(grid), q)
            # Approach with a floor-checked trajectory; skip if unsafe.
            traj = plan_ptp(self.current_q(), q, av, self.a_max)
            if not self._traj_floor_ok(traj, z_floor):
                continue
            self.bus.play_trajectory(self._traj_to_vectors(traj))
            self.wait()
            time.sleep(0.3)  # settle (3D-printed links ring a little)
            qc, load, fric = self.probe_pose(sweep_deg, speed_dps)
            samples.append((list(qc), list(load)))
            raw.append({"q": list(qc), "load": list(load), "friction": list(fric)})
        if len(samples) < 8:
            raise ValueError(f"only {len(samples)} usable poses - widen the "
                             "grid or relax the floor")
        model = GravityModel()
        rms = model.fit(samples)
        model.save()
        try:
            with open(SAMPLES_FILE, "w") as f:
                json.dump({"samples": raw, "n_joints": self.bus.n}, f, indent=2)
        except OSError as e:
            print(f"  [WARN] could not save raw samples: {e}")
        return model, rms, samples, raw

    # ── stiffness calibration (interactive, dial indicator) ─────────────
    def approach(self, q, z_floor=Z_FLOOR_MM):
        """Floor-checked joint-space move to q (reduced speed) + wait. Returns
        False without moving if the path would dip below the floor."""
        traj = plan_ptp(self.current_q(), q, math.radians(GRAVCAL_APPROACH_DPS), self.a_max)
        if not self._traj_floor_ok(traj, z_floor):
            return False
        self.bus.play_trajectory(self._traj_to_vectors(traj))
        self.wait()
        return True

    def stiffcal_poses(self, n=STIFFCAL_N, g_headroom=STIFFCAL_G_HEADROOM,
                       z_floor=Z_FLOOR_MM):
        """Auto-pick n safe, well-conditioned poses for stiffness calibration.
        Needs an identified gravity model (to keep arm-only load within
        headroom so probing WITH the tip load stays under the torque cap).
        Greedy farthest-point on normalized tip-Jacobian direction -> spread
        J0/J1 weighting so tip-only droop separates the two stiffnesses."""
        gmodel = GravityModel.load()
        if gmodel is None or not gmodel.is_identified():
            raise ValueError("run gravcal first - gravity model is required")
        cand = []
        for q0d in range(20, 131, 10):
            for q1d in range(-100, 101, 20):
                q = (math.radians(q0d), math.radians(q1d))
                if not kin.within_joint_limits(q, JOINT_LIMITS):
                    continue
                if not self._pose_safe(q, z_floor, math.radians(PROBE_SWEEP_DEG)):
                    continue
                if any(abs(g) > g_headroom for g in gmodel.predict(q)):
                    continue
                cand.append(q)
        if len(cand) < n:
            raise ValueError(f"only {len(cand)} candidate poses fit the headroom; "
                             "lower STIFFCAL_G_HEADROOM or n")
        # Select poses that make the regression design [J0_i, J1_i] well
        # conditioned, so BOTH joints are observable from tip-only droop.
        # Greedy: maximize the smallest singular value of the column-normalized
        # Jacobian matrix over the chosen poses (avoids near-singular sets such
        # as several poses with J1~0 at q0+q1=90deg).
        import numpy as np
        J = np.array([tip_jacobian(q) for q in cand], dtype=float)   # (m,2)
        scale = np.maximum(np.abs(J).max(axis=0), 1e-9)
        Jn = J / scale
        chosen = [int(np.argmax((Jn ** 2).sum(axis=1)))]
        while len(chosen) < n:
            best, best_sv = None, -1.0
            for i in range(len(cand)):
                if i in chosen:
                    continue
                sv = np.linalg.svd(Jn[chosen + [i]], compute_uv=False).min()
                if sv > best_sv:
                    best, best_sv = i, sv
            chosen.append(best)
        return [cand[i] for i in chosen]

    def measure_dm(self, gmodel, sweep_deg=STIFFCAL_SWEEP_DEG, speed_dps=STIFFCAL_SPEED_DPS):
        """Probe the current (loaded) pose and return (q, dm, load_with) where
        dm = load_with - G(q) is the per-mille load increment from the added
        tip load. Caller must have the known load mounted. The sweep is tiny
        (config STIFFCAL_SWEEP_DEG) so a small-range tip indicator stays in
        range during the probe."""
        qc, load_with, _fric = self.probe_pose(sweep_deg, speed_dps)
        g = gmodel.predict(qc)
        dm = [load_with[j] - g[j] for j in range(self.bus.n)]
        return qc, dm, load_with

    @staticmethod
    def fit_stiffness(samples):
        """Fit + persist a StiffnessModel from [(q, dm, dz_mm), ...]."""
        model = StiffnessModel()
        rms = model.fit(samples)
        model.save()
        return model, rms

    # ── two-experiment stiffness calibration ────────────────────────────
    def fit_droop_stiffness(self, samples):
        """Experiment 1: physical stiffness Kp [N*mm/rad] from STATIC tip droop
        under the known load. samples = [(q, dz_mm), ...] (>=2 poses, dz signed,
        down<0). Solves |dz| = F*(J0^2/Kp0 + J1^2/Kp1) for 1/Kp via LSQ; the
        (0,90)-type pose (J1=0) isolates the shoulder. Saves stiffness_phys.json,
        returns Kp."""
        import numpy as np
        if len(samples) < self.bus.n:
            raise ValueError(f"need >= {self.bus.n} droop poses")
        F = STIFFCAL_LOAD_KG * G_ACCEL                 # N
        A, y = [], []
        for q, dz_mm in samples:
            J = tip_jacobian(q)                        # mm
            A.append([J[j] ** 2 for j in range(self.bus.n)])
            y.append(abs(dz_mm) / F)
        A = np.array(A, float); y = np.array(y, float)
        # Non-negative least squares: 1/Kp >= 0 (stiffness can't be negative).
        # If the data is inconsistent (nonlinearity/backlash), a joint goes to
        # 1/Kp=0 (infinite stiffness, no comp for it) rather than negative.
        inv_kp = _nnls(A, y)
        Kp = [(1.0 / v if v > 1e-12 else None) for v in inv_kp]
        try:
            with open(STIFF_PHYS_FILE, "w") as f:
                json.dump({"Kp_Nmm": Kp, "F_N": F, "n_joints": self.bus.n,
                           "samples": [{"q": list(q), "dz_mm": dz} for q, dz in samples]},
                          f, indent=2)
        except OSError as e:
            print(f"  [WARN] could not save stiffness_phys.json: {e}")
        return Kp

    def finalize_stiffness(self, scale_samples):
        """Experiment 2: per-mille-per-torque scale s_i from de-frictioned load
        WITH minus WITHOUT the load, then K_i = s_i * Kp_i [per-mille/rad].
        scale_samples = [(q, dm_vec), ...] (dm in per-mille). Reads
        stiffness_phys.json (Kp), saves stiffness_calib.json, returns (K, s, Kp)."""
        if not os.path.exists(STIFF_PHYS_FILE):
            raise ValueError("run stiffdroop first (no stiffness_phys.json)")
        with open(STIFF_PHYS_FILE) as f:
            Kp = json.load(f)["Kp_Nmm"]
        F = STIFFCAL_LOAD_KG * G_ACCEL
        # s_j = mean over poses of dm_j / (-F*J_j), skipping tiny-lever poses.
        # The gravitational generalized force from the tip mass is -F*J_j (it
        # pulls the tip down), and the per-mille load increment dm_j = s_j*(-F*J_j);
        # the same -F*J convention is used for Kp in fit_droop_stiffness, so
        # K = s*Kp comes out POSITIVE and delta=G/K has the sag-reducing sign.
        num = [0.0] * self.bus.n
        cnt = [0] * self.bus.n
        for q, dm in scale_samples:
            J = tip_jacobian(q)
            for j in range(self.bus.n):
                if abs(J[j]) > 1.0:                    # mm: meaningful lever
                    num[j] += dm[j] / (-F * J[j]); cnt[j] += 1
        s = [num[j] / cnt[j] if cnt[j] else None for j in range(self.bus.n)]
        K = []
        for j in range(self.bus.n):
            if Kp[j] is None or s[j] is None or s[j] == 0:
                K.append(None)
            else:
                K.append(s[j] * Kp[j])
        StiffnessModel(K).save()
        return K, s, Kp

    # ── dynamic identification (dyncal) ─────────────────────────────────
    def dyncal(self, poses=None, vas=None, z_floor=Z_FLOOR_MM, progress=None):
        """Identify the dynamic model (inertia + Coriolis + friction) by running
        excitation moves with VARYING acceleration and logging per-cycle torque
        against the trajectory's analytic (q, q̇, q̈). Compensation is forced OFF
        (so commanded motion == trajectory) and NO payload must be mounted.
        Returns (model, rms, n_samples). Persists dynamics_calib.json + raw
        samples. Requires an identified gravity model (reused as-is)."""
        self._require_home()
        g = GravityModel.load()
        if g is None or not g.is_identified():
            raise ValueError("run gravcal first (gravity model needed)")
        if poses is None:
            poses = self.grav_grid(3, 3, z_floor=z_floor)
        if len(poses) < 2:
            raise ValueError("need >= 2 safe poses for an excitation tour")
        if vas is None:
            # (v_max [rad/s], a_max [rad/s^2]) — different accels excite inertia.
            vas = [(0.2, 0.5), (0.3, 1.0), (0.4, 2.0)]

        comp_was = self.comp_mode
        self.comp_mode = "off"                # uncompensated -> clean q,q̇,q̈
        data = []
        try:
            tour = list(poses) + list(reversed(poses))   # both velocity signs
            self.approach(tour[0], z_floor)
            total = len(vas) * (len(tour) - 1)
            k = 0
            for (v, a) in vas:
                for s in range(len(tour) - 1):
                    k += 1
                    if progress:
                        progress(k, total, v, a, tour[s + 1])
                    traj = plan_ptp(self.current_q(), tour[s + 1], v, a)
                    if not self._traj_floor_ok(traj, z_floor):
                        continue
                    sf = traj.samples_full(self.bus.cycle_time)
                    try:
                        vectors = [jm.angles_to_counts(q, self.home_counts)
                                   for (_t, q, _qd, _qdd) in sf]
                    except Exception:
                        continue
                    if any(not jm.counts_in_int32(c) for vec in vectors for c in vec):
                        continue
                    self.bus.play_trajectory(vectors, log_torque=True)
                    self.wait()
                    tlog = self.bus.torque_log()
                    m = min(len(sf), len(tlog))
                    for i in range(m):
                        _t, q, qd, qdd = sf[i]
                        data.append((q, qd, qdd, tuple(tlog[i])))
            if len(data) < 50:
                raise ValueError(f"only {len(data)} samples - too few")
            model = DynamicsModel(g)
            rms = model.fit(data)
            model.save()
            try:
                with open(DYN_SAMPLES_FILE, "w") as f:
                    json.dump({"samples": [{"q": q, "qd": qd, "qdd": qdd, "tau": list(t)}
                                           for q, qd, qdd, t in data],
                               "n_joints": self.bus.n}, f)
            except OSError as e:
                print(f"  [WARN] could not save raw dyn samples: {e}")
            return model, rms, len(data)
        finally:
            self.comp_mode = comp_was


# ── helpers ──────────────────────────────────────────────────────────────
def _deg(q):
    return tuple(round(math.degrees(a), 2) for a in q)


def _linspace(a, b, n):
    if n <= 1:
        return [0.5 * (a + b)]
    return [a + (b - a) * i / (n - 1) for i in range(n)]


def _nnls(A, y):
    """Non-negative least squares min ||A x - y|| s.t. x >= 0, by enumerating
    active sets. Exact and dependency-free for the small number of columns here
    (joints). Returns x as a numpy array."""
    import numpy as np
    from itertools import combinations
    m = A.shape[1]
    best_x = np.zeros(m); best_r = float(np.dot(y, y))
    idx = list(range(m))
    for k in range(1, m + 1):
        for cols in combinations(idx, k):
            sub = A[:, cols]
            sol, *_ = np.linalg.lstsq(sub, y, rcond=None)
            if np.any(sol < 0):
                continue                       # infeasible for this active set
            x = np.zeros(m)
            for c, v in zip(cols, sol):
                x[c] = v
            r = float(np.sum((A @ x - y) ** 2))
            if r < best_r:
                best_r, best_x = r, x
    return best_x


# ── CLI ────────────────────────────────────────────────────────────────
def _print_where(rc):
    try:
        q = rc.current_q()
        x, z = kin.forward(q)
        counts = rc.bus.actual_counts()
        print(f"  q = {_deg(q)} deg   (x, z) = ({x:.1f}, {z:.1f}) mm   counts = {counts}")
    except NotHomed:
        print("  not homed")


def _monitor_move(rc, dur, label):
    """Print progress (live torque) while a move plays; handle Ctrl+C abort."""
    print(f"  moving to {label} - {dur:.2f}s. Ctrl+C to abort.")
    try:
        while rc.bus.is_busy():
            snap, *_ = rc.bus.status()
            trq = ' '.join(f"D{i}={s['trq']:+d}" for i, s in enumerate(snap))
            extra = ""
            if rc.comp_mode == "adaptive" and rc.last_live is not None:
                cm = rc.correction_mm()
                mm = f" corr={cm[2]:.2f}mm" if cm else ""
                extra = f"  c_p={rc.last_live.last_cp:+.3f}{mm}"
            sys.stdout.write(f"\r  trq {trq}{extra}   ")
            sys.stdout.flush()
            time.sleep(0.1)
        print("\n  arrived.")
        cm = rc.correction_mm()
        if cm is not None:
            print(f"  deflection corrected by {cm[2]:.2f} mm at the tip "
                  f"(dx={cm[0]:+.2f}, dz={cm[1]:+.2f} mm)")
        _print_where(rc)
    except KeyboardInterrupt:
        rc.stop()
        print("\n  aborted - holding position.")


def main():
    print("=" * 70)
    print("X-Z Cartesian controller (CLI) - moves hardware")
    print("=" * 70)
    rc = RobotController()
    try:
        rc.connect()
    except Exception as e:
        print(f"connect failed: {e}")
        rc.disconnect()
        return

    # Try to restore a persisted home from a previous session (valid only if
    # the drives kept power since). Verify it before trusting it.
    ok, info = rc.load_home()
    if ok and rc.home_plausible():
        print(f"\nHome RESTORED from {info}.")
        _print_where(rc)
        print("If this pose does NOT match the arm's real position, run "
              "`rehome` (the drives may have lost power).")
    elif ok:
        rc.clear_home()
        print(f"\nSaved home from {info} looks INVALID (current pose out of "
              "range — drives likely lost power). Discarded; please `home`.")
    else:
        print(f"\nNOT homed ({info}). Put the arm at the zero pose, then `home`.")
    print("Commands (all distances in mm):")
    print("  home/rehome | clearhome | where")
    print("  move <x> <z>     absolute Cartesian target (mm)")
    print("  rmove <dx> <dz>  relative Cartesian (mm)")
    print("  jmove <q0> <q1>  absolute joint angles (deg)")
    print("  jrmove <d0> <d1> relative joint angles (deg)")
    print(f"  probe [<sweep_deg> <speed_dps>]  measure load torque + friction "
          f"(default {PROBE_SWEEP_DEG} deg, {PROBE_SPEED_DPS} deg/s)")
    print(f"  gravcal [<n0> <n1>]  autonomous grid sweep -> fit gravity model "
          f"(default {GRAVCAL_GRID_N}x{GRAVCAL_GRID_N})")
    print(f"  stiffdroop      EXP1: K_eff via static droop under {STIFFCAL_LOAD_KG} kg (indicator)")
    print(f"  stiffscale      EXP2: load-scale via with/without-load probe -> finalize K_eff")
    print(f"  stiffcal [<n>]  (legacy single-step K_eff cal)")
    print("  dyncal           autonomous excitation sweep -> fit dynamic model "
          "(inertia/Coriolis/friction); NO payload mounted")
    print("  comp [off|static|adaptive]   deflection compensation mode/status")
    print("  dyncal           identify dynamic model (for adaptive comp)")
    print("  elbow up|down | speed <v> <a> | stop | q")
    avail = []
    if rc._static_ok():
        avail.append("static")
    if rc._adaptive_ok():
        avail.append("adaptive")
    print(f"\nDeflection compensation mode: {rc.comp_mode.upper()}  "
          f"(available: {', '.join(avail) if avail else 'none - run gravcal/stiffcal/dyncal'})")
    try:
        while True:
            try:
                raw = input("\nxz> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if raw in ("q", "quit", "exit"):
                break
            if raw == "":
                continue
            parts = raw.split()
            cmd = parts[0]

            if cmd in ("home", "rehome"):
                hc = rc.set_home()
                print(f"  home set & saved: counts = {hc}  (this pose = {_deg(JOINT_ZERO_RAD)} deg)")
            elif cmd == "clearhome":
                rc.clear_home()
                print("  home cleared (saved file removed)")
            elif cmd == "where":
                _print_where(rc)
            elif cmd == "elbow" and len(parts) == 2 and parts[1] in ("up", "down"):
                rc.elbow = parts[1]
                print(f"  elbow branch = {rc.elbow}")
            elif cmd == "speed" and len(parts) == 3:
                try:
                    rc.v_max = float(parts[1]); rc.a_max = float(parts[2])
                    print(f"  v_max={rc.v_max} rad/s  a_max={rc.a_max} rad/s^2")
                except ValueError:
                    print("  speed <v> <a> - numbers")
            elif cmd == "probe":
                sweep, speed = PROBE_SWEEP_DEG, PROBE_SPEED_DPS
                if len(parts) == 3:
                    try:
                        sweep = float(parts[1]); speed = float(parts[2])
                    except ValueError:
                        print("  probe [<sweep_deg> <speed_dps>] - numbers")
                        continue
                elif len(parts) != 1:
                    print("  probe [<sweep_deg> <speed_dps>]")
                    continue
                try:
                    qc, load, fric = rc.probe_pose(sweep, speed)
                except (NotHomed, ValueError) as e:
                    print(f"  probe failed: {e}")
                    continue
                print(f"  probed at q = {_deg(qc)} deg "
                      f"(sweep +/-{sweep} deg @ {speed} deg/s):")
                for j in range(rc.bus.n):
                    print(f"    D{j}: load={load[j]:+.1f}  friction={fric[j]:+.1f}  (per-mille)")
            elif cmd == "gravcal":
                n0 = n1 = GRAVCAL_GRID_N
                if len(parts) == 3:
                    try:
                        n0 = int(parts[1]); n1 = int(parts[2])
                    except ValueError:
                        print("  gravcal [<n0> <n1>] - integers")
                        continue
                elif len(parts) != 1:
                    print("  gravcal [<n0> <n1>]")
                    continue
                try:
                    grid = rc.grav_grid(n0, n1)
                except NotHomed as e:
                    print(f"  {e}")
                    continue
                if len(grid) < 8:
                    print(f"  only {len(grid)} safe poses - too few; check limits")
                    continue
                print(f"  {len(grid)} safe poses. The robot will move AUTONOMOUSLY")
                print(f"  across the workspace (approach {GRAVCAL_APPROACH_DPS} deg/s, "
                      f"probe {PROBE_SWEEP_DEG} deg). Ctrl+C aborts.")
                if input("  proceed? [y/N] ").strip().lower() not in ("y", "yes"):
                    print("  cancelled")
                    continue

                def _prog(k, total, q):
                    print(f"  [{k+1}/{total}] pose {_deg(q)} deg")

                try:
                    model, rms, samples, _raw = rc.grav_calibrate(
                        grid=grid, progress=_prog)
                except KeyboardInterrupt:
                    rc.stop()
                    print("\n  aborted - holding position (no fit saved).")
                    continue
                except ValueError as e:
                    print(f"  gravcal failed: {e}")
                    continue
                print(f"  fitted from {len(samples)} poses. RMS residual "
                      f"(per-mille): {[round(r,1) for r in rms]}")
                for j in range(rc.bus.n):
                    print(f"    D{j} coeffs: {[round(c,2) for c in model.coeffs[j]]}")
                print("  saved gravity_calib.json + gravity_samples.json")
            elif cmd == "stiffdroop":
                # Experiment 1: physical stiffness Kp from static droop.
                poses_deg = STIFF_DROOP_POSES_DEG
                print(f"  EXP1 physical stiffness: static droop under the "
                      f"{STIFFCAL_LOAD_KG} kg load at poses {poses_deg} deg, NO sweep.")
                print("  Comp forced OFF. Ctrl+C aborts.")
                if input("  proceed? [y/N] ").strip().lower() not in ("y", "yes"):
                    print("  cancelled"); continue
                comp_was = rc.comp_mode; rc.comp_mode = "off"
                samples = []; ok = True
                try:
                    for (q0d, q1d) in poses_deg:
                        q = (math.radians(q0d), math.radians(q1d))
                        print(f"\n  moving to ({q0d}, {q1d}) deg ...")
                        if not rc.approach(q):
                            print("    unsafe path - aborting"); ok = False; break
                        input("    place & ZERO indicator at tip (NO load), Enter")
                        input(f"    hang the {STIFFCAL_LOAD_KG} kg load, Enter")
                        raw = input("    tip droop in hundredths (0.01mm, down=neg): ").strip()
                        try:
                            dz = float(raw) / 100.0
                        except ValueError:
                            print("    bad number - aborting"); ok = False; break
                        samples.append((q, dz))
                        print(f"    recorded dz={dz:+.3f} mm")
                        input("    remove load + indicator, Enter")
                    if ok and len(samples) >= rc.bus.n:
                        Kp = rc.fit_droop_stiffness(samples)
                        print(f"\n  Kp (N*mm/rad): {[round(k,1) if k else None for k in Kp]}")
                        if any(k is None or k <= 0 for k in Kp):
                            print("  [WARN] a Kp<=0 -> inconsistent droop (check signs/poses)")
                        else:
                            print("  saved stiffness_phys.json. Next: `stiffscale`")
                except KeyboardInterrupt:
                    rc.stop(); print("\n  aborted (nothing saved).")
                finally:
                    rc.comp_mode = comp_was
            elif cmd == "stiffscale":
                # Experiment 2: per-mille load scale s_i -> final K_eff.
                g = GravityModel.load()
                if g is None or not g.is_identified():
                    print("  run gravcal first"); continue
                if not os.path.exists(STIFF_PHYS_FILE):
                    print("  run stiffdroop first (no stiffness_phys.json)"); continue
                poses_deg = STIFF_SCALE_POSES_DEG
                tipmm = 2 * math.radians(PROBE_SWEEP_DEG) * sum(kin.LINK_LENGTHS)
                print(f"  EXP2 load scale: de-frictioned probe WITH/WITHOUT the "
                      f"{STIFFCAL_LOAD_KG} kg load at {poses_deg} deg.")
                print(f"  INDICATOR REMOVED (probe sweeps {PROBE_SWEEP_DEG} deg -> tip "
                      f"~{tipmm:.0f} mm). Comp OFF. Ctrl+C aborts.")
                if input("  proceed? [y/N] ").strip().lower() not in ("y", "yes"):
                    print("  cancelled"); continue
                comp_was = rc.comp_mode; rc.comp_mode = "off"
                scale = []; ok = True
                try:
                    for (q0d, q1d) in poses_deg:
                        q = (math.radians(q0d), math.radians(q1d))
                        print(f"\n  moving to ({q0d}, {q1d}) deg ...")
                        if not rc.approach(q):
                            print("    unsafe path - aborting"); ok = False; break
                        input("    ensure NO load mounted, Enter (probing baseline)")
                        _qc0, load0, _ = rc.probe_pose()
                        input(f"    hang the {STIFFCAL_LOAD_KG} kg load, Enter (probing loaded)")
                        qc1, load1, _ = rc.probe_pose()
                        if any(abs(l) > 320 for l in load1):
                            print(f"    [WARN] loaded torque {[round(l) for l in load1]} "
                                  "per-mille near cap - reading may be unreliable")
                        dm = [load1[j] - load0[j] for j in range(rc.bus.n)]
                        scale.append((list(qc1), dm))
                        print(f"    dm={[round(d,1) for d in dm]} per-mille")
                        input("    remove the load, Enter")
                    if ok and scale:
                        K, s, Kp = rc.finalize_stiffness(scale)
                        print(f"\n  s (per-mille/Nmm): {[round(si,5) if si else None for si in s]}")
                        print(f"  Kp (N*mm/rad):     {[round(k,1) if k else None for k in Kp]}")
                        print(f"  K_eff (per-mille/rad): {[round(k,1) if k else None for k in K]}")
                        if any(k is None or k <= 0 for k in K):
                            print("  [WARN] a K_eff<=0/None - check dm signs / Kp")
                        else:
                            print("  saved stiffness_calib.json. static/adaptive comp now usable.")
                except KeyboardInterrupt:
                    rc.stop(); print("\n  aborted (nothing saved).")
                finally:
                    rc.comp_mode = comp_was
            elif cmd == "stiffcal":
                n = STIFFCAL_N
                if len(parts) == 2:
                    try:
                        n = int(parts[1])
                    except ValueError:
                        print("  stiffcal [<n>] - integer")
                        continue
                elif len(parts) != 1:
                    print("  stiffcal [<n>]")
                    continue
                try:
                    gmodel = GravityModel.load()
                    if gmodel is None or not gmodel.is_identified():
                        print("  run gravcal first (gravity model required)")
                        continue
                    poses = rc.stiffcal_poses(n)
                except (NotHomed, ValueError) as e:
                    print(f"  {e}")
                    continue
                print(f"  {len(poses)} poses selected. For each: zero the indicator")
                print(f"  (no load), hang the {STIFFCAL_LOAD_KG} kg load, read droop in")
                print("  hundredths (0.01 mm), down = negative. Ctrl+C aborts.")
                print(f"  NOTE: probe sweeps {STIFFCAL_SWEEP_DEG} deg -> tip moves "
                      f"~{2*math.radians(STIFFCAL_SWEEP_DEG)*sum(kin.LINK_LENGTHS):.1f} mm; "
                      "keep the indicator within range.")
                if input("  proceed? [y/N] ").strip().lower() not in ("y", "yes"):
                    print("  cancelled")
                    continue
                samples = []
                try:
                    for k, q in enumerate(poses):
                        print(f"\n  [{k+1}/{len(poses)}] moving to {_deg(q)} deg ...")
                        if not rc.approach(q):
                            print("    unsafe path - skipped")
                            continue
                        input("    place indicator at tip, ZERO it (no load), Enter")
                        input(f"    hang the {STIFFCAL_LOAD_KG} kg load, Enter")
                        qc, dm, load_with = rc.measure_dm(gmodel)
                        if any(abs(l) > 320 for l in load_with):
                            print(f"    [WARN] load_with={[round(l) for l in load_with]} "
                                  "per-mille near torque cap - reading may be unreliable")
                        raw = input("    tip droop in hundredths (0.01mm, down=neg): ").strip()
                        try:
                            dz = float(raw) / 100.0
                        except ValueError:
                            print("    bad number - pose skipped")
                            input("    remove the load, Enter")
                            continue
                        samples.append((list(qc), dm, dz))
                        print(f"    recorded: dm={[round(d,1) for d in dm]} per-mille  dz={dz:+.3f} mm")
                        input("    remove the load, Enter")
                except KeyboardInterrupt:
                    rc.stop()
                    print("\n  aborted - holding position (no fit saved).")
                    continue
                # Persist raw samples first, so a bad fit can be diagnosed /
                # re-fit (with added poses) without re-running on hardware.
                try:
                    with open(STIFF_SAMPLES_FILE, "w") as f:
                        json.dump({"samples": [{"q": q, "dm": dm, "dz_mm": dz}
                                               for q, dm, dz in samples],
                                   "n_joints": rc.bus.n}, f, indent=2)
                except OSError as e:
                    print(f"  [WARN] could not save raw stiffness samples: {e}")
                if len(samples) < rc.bus.n:
                    print(f"  only {len(samples)} usable poses (<{rc.bus.n}) - not enough")
                    continue
                model, rms = rc.fit_stiffness(samples)
                print(f"\n  fitted K_eff from {len(samples)} poses. RMS residual {rms:.4f} mm")
                for j in range(rc.bus.n):
                    print(f"    K_eff[D{j}] = {model.k[j]:.1f} per-mille/rad")
                if any(k is None or k <= 0 for k in model.k):
                    print("  [WARN] a K_eff is non-positive -> that joint was NOT")
                    print("  identified (poor conditioning/noise). Re-run with more")
                    print("  poses (e.g. `stiffcal 6`); raw data is in stiffness_samples.json")
                else:
                    print("  saved stiffness_calib.json")
            elif cmd == "dyncal":
                if rc.compensator is None and GravityModel.load() is None:
                    print("  run gravcal first (gravity model needed)")
                    continue
                print("  Autonomous EXCITATION sweep (varying speed/accel) across")
                print("  the workspace. NO payload must be mounted. Ctrl+C aborts.")
                if input("  proceed? [y/N] ").strip().lower() not in ("y", "yes"):
                    print("  cancelled")
                    continue

                def _dprog(k, total, v, a, q):
                    print(f"  [{k}/{total}] v={v} a={a} -> {_deg(q)} deg")

                try:
                    model, rms, n = rc.dyncal(progress=_dprog)
                except KeyboardInterrupt:
                    rc.stop()
                    print("\n  aborted - holding position (no fit saved).")
                    continue
                except (ValueError, NotHomed) as e:
                    print(f"  dyncal failed: {e}")
                    continue
                print(f"  fitted dynamic model from {n} samples. "
                      f"RMS residual (per-mille): {[round(r,2) for r in rms]}")
                for j in range(rc.bus.n):
                    print(f"    D{j} dyn coeffs: {[round(c,2) for c in model.coeffs[j]]}")
                print("  saved dynamics_calib.json + dynamics_samples.json")
            elif cmd == "comp":
                if len(parts) == 2 and parts[1] in ("off", "static", "adaptive"):
                    want = parts[1]
                    if want == "static" and not rc._static_ok():
                        print("  static unavailable (need gravcal + stiffcal)")
                    elif want == "adaptive" and not rc._adaptive_ok():
                        print("  adaptive unavailable (need gravcal + stiffcal + dyncal)")
                    else:
                        rc.comp_mode = want
                        rc._live_comp = None      # fresh ramp on (re)entry
                        print(f"  deflection comp mode -> {want.upper()}")
                else:
                    av = [m for m, ok in (("static", rc._static_ok()),
                                          ("adaptive", rc._adaptive_ok())) if ok]
                    print(f"  comp mode: {rc.comp_mode.upper()}  "
                          f"(available: {', '.join(av) or 'none'})  "
                          f"usage: comp off|static|adaptive")
                    if rc._static_ok():
                        try:
                            d = rc.compensator.correction(rc.current_q())
                            print(f"  static correction at current pose: {_deg(d)} deg")
                        except NotHomed:
                            pass
            elif cmd == "stop":
                rc.stop()
                print("  -> stop / hold")
            elif cmd in ("move", "rmove", "jmove", "jrmove") and len(parts) == 3:
                try:
                    a = float(parts[1]); b = float(parts[2])
                except ValueError:
                    unit = "deg" if cmd.startswith("j") else "mm"
                    print(f"  {cmd} <a> <b> - numbers in {unit}")
                    continue
                try:
                    if cmd == "move":
                        dur = rc.move_to(a, b)
                        label = f"({a:.1f}, {b:.1f}) mm"
                    elif cmd == "rmove":
                        dur = rc.move_by(a, b)
                        label = f"relative ({a:+.1f}, {b:+.1f}) mm"
                    elif cmd == "jmove":
                        dur = rc.move_joints((math.radians(a), math.radians(b)))
                        label = f"joints ({a:.1f}, {b:.1f}) deg"
                    else:  # jrmove
                        dur = rc.move_joints_by((math.radians(a), math.radians(b)))
                        label = f"joints relative ({a:+.1f}, {b:+.1f}) deg"
                except (ValueError, kin.Unreachable, NotHomed) as e:
                    print(f"  rejected: {e}")
                    continue
                if rc.comp_mode == "static" and rc.last_correction:
                    print(f"  static comp applied: {_deg(rc.last_correction)} deg")
                elif rc.comp_mode == "adaptive":
                    print("  adaptive comp: live payload observer active")
                _monitor_move(rc, dur, label)
            else:
                print("  unknown command")
    finally:
        print()
        rc.disconnect()


if __name__ == "__main__":
    main()
