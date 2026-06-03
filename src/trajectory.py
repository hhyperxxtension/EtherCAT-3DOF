"""
Joint-space point-to-point (PTP) trajectory generation with a synchronized
trapezoidal velocity profile. Pure math — testable offline.

Every joint follows a symmetric trapezoid (accelerate → cruise → decelerate).
Joints are *time-synchronized*: the slowest joint sets the total move time T,
and the others are slowed to finish at the same instant, so the arm starts
and stops all joints together. This keeps the end-effector path repeatable
(though, being joint-space, that path is an arc — not a Cartesian straight
line; Cartesian interpolation can later be layered on the same sampler).

Profile maths (per joint, distance d ≥ 0, over total time T, accel a):
    cruise speed v solves   d = v·T − v²/a   →   v = (a·T − √((a·T)² − 4·a·d)) / 2
    accel/decel time        t_a = v / a
    s(t) = ½·a·t²                       0 ≤ t < t_a
         = ½·a·t_a² + v·(t − t_a)       t_a ≤ t < T − t_a
         = d − ½·a·(T − t)²             T − t_a ≤ t ≤ T
A purely triangular move (no cruise) falls out of the same formula with
t_a → T/2.
"""

import math

try:
    from config import JOINT_VEL_MAX, JOINT_ACC_MAX, EC_CYCLE_TIME
except ImportError:
    JOINT_VEL_MAX = 1.0
    JOINT_ACC_MAX = 2.0
    EC_CYCLE_TIME = 0.004


def _min_time(d, v_max, a_max):
    """Minimum time to move distance d (≥0) under v_max / a_max limits."""
    if d <= 0.0:
        return 0.0
    d_ramp = v_max * v_max / a_max          # distance covered by accel + decel
    if d <= d_ramp:                         # triangular — never reaches v_max
        return 2.0 * math.sqrt(d / a_max)
    return d / v_max + v_max / a_max        # trapezoidal


def _cruise_speed(d, T, a_max):
    """Cruise speed for a symmetric trapezoid of distance d over exactly
    time T using acceleration a_max. Assumes T ≥ min-time so a real solution
    exists; returns 0 for a zero-distance joint."""
    if d <= 0.0 or T <= 0.0:
        return 0.0
    disc = (a_max * T) ** 2 - 4.0 * a_max * d
    disc = max(0.0, disc)                   # guard tiny negative from rounding
    return (a_max * T - math.sqrt(disc)) / 2.0


class _JointProfile:
    """Scalar trapezoid for one joint, mapped onto a shared duration T."""

    def __init__(self, q0, q1, T, a_max):
        self.q0 = q0
        self.d = q1 - q0
        self.sign = 1.0 if self.d >= 0.0 else -1.0
        self.dist = abs(self.d)
        self.T = T
        self.a = a_max
        self.v = _cruise_speed(self.dist, T, a_max)
        self.t_a = (self.v / a_max) if a_max > 0.0 else 0.0

    def at(self, t):
        """Joint position at time t (clamped to [0, T])."""
        if t <= 0.0:
            return self.q0
        if t >= self.T:
            return self.q0 + self.d
        ta, T, a, v = self.t_a, self.T, self.a, self.v
        if t < ta:
            s = 0.5 * a * t * t
        elif t < T - ta:
            s = 0.5 * a * ta * ta + v * (t - ta)
        else:
            td = T - t
            s = self.dist - 0.5 * a * td * td
        return self.q0 + self.sign * s

    def vel(self, t):
        """Joint velocity at time t [rad/s] (0 outside [0, T])."""
        if t <= 0.0 or t >= self.T:
            return 0.0
        ta, T, a, v = self.t_a, self.T, self.a, self.v
        if t < ta:
            s = a * t
        elif t < T - ta:
            s = v
        else:
            s = a * (T - t)
        return self.sign * s

    def acc(self, t):
        """Joint acceleration at time t [rad/s^2] (0 outside [0, T] and during
        cruise; +/-a during accel/decel)."""
        if t <= 0.0 or t >= self.T:
            return 0.0
        ta, T, a = self.t_a, self.T, self.a
        if t < ta:
            s = a
        elif t < T - ta:
            s = 0.0
        else:
            s = -a
        return self.sign * s


class Trajectory:
    """A planned PTP move. Sample it with `at(t)` or materialise the whole
    thing with `samples()`."""

    def __init__(self, q_start, q_target, duration, profiles):
        self.q_start = tuple(q_start)
        self.q_target = tuple(q_target)
        self.duration = duration
        self._profiles = profiles

    def at(self, t):
        """Joint vector at time t [s] (tuple, length = #joints)."""
        return tuple(p.at(t) for p in self._profiles)

    def at_full(self, t):
        """(q, q̇, q̈) vectors at time t — analytic, no differentiation."""
        return (tuple(p.at(t) for p in self._profiles),
                tuple(p.vel(t) for p in self._profiles),
                tuple(p.acc(t) for p in self._profiles))

    def samples(self, dt=EC_CYCLE_TIME):
        """Materialise the trajectory as a list of (t, q-vector) at step dt.
        Always includes t=0 and the exact endpoint at t=duration."""
        out = []
        n = max(1, int(math.ceil(self.duration / dt)))
        for k in range(n):
            t = k * dt
            out.append((t, self.at(t)))
        out.append((self.duration, tuple(self.q_target)))
        return out

    def samples_full(self, dt=EC_CYCLE_TIME):
        """Like samples() but each entry is (t, q, q̇, q̈) — for dynamic
        identification / the inertial-torque term of the live observer."""
        out = []
        n = max(1, int(math.ceil(self.duration / dt)))
        for k in range(n):
            t = k * dt
            q, qd, qdd = self.at_full(t)
            out.append((t, q, qd, qdd))
        nz = tuple(0.0 for _ in self.q_target)
        out.append((self.duration, tuple(self.q_target), nz, nz))
        return out


def plan_ptp(q_start, q_target, v_max=JOINT_VEL_MAX, a_max=JOINT_ACC_MAX):
    """Plan a synchronized trapezoidal move from q_start to q_target.

    v_max, a_max are scalar joint limits (rad/s, rad/s²) applied to every
    joint; the dominant joint runs at the limit and the rest are slowed to
    match its duration. Returns a Trajectory.
    """
    if len(q_start) != len(q_target):
        raise ValueError("q_start and q_target must have the same length")
    if v_max <= 0.0 or a_max <= 0.0:
        raise ValueError("v_max and a_max must be positive")

    deltas = [abs(b - a) for a, b in zip(q_start, q_target)]
    T = max((_min_time(d, v_max, a_max) for d in deltas), default=0.0)
    profiles = [_JointProfile(a, b, T, a_max)
                for a, b in zip(q_start, q_target)]
    return Trajectory(q_start, q_target, T, profiles)
