"""
Planar 2DOF kinematics for the shoulder + forearm arm (RRR with the base
yaw joint omitted for now). Pure math — no hardware, no pysoem — so it can
be unit-tested offline.

Frame (see config.py): origin at the shoulder joint axis, X horizontal
(forward +), Z vertical (up +). The arm works in the X-Z plane.
    q[0] = shoulder angle from +X, CCW positive
    q[1] = elbow angle relative to link 1, CCW positive
End effector at the tip of link 2.

Forward kinematics:
    elbow = (L1·cosq0,            L1·sinq0)
    ee    = (L1·cosq0 + L2·cos(q0+q1),  L1·sinq0 + L2·sin(q0+q1))

Inverse kinematics (closed form, two branches):
    r²       = x² + z²
    cos(q1)  = (r² − L1² − L2²) / (2·L1·L2)
    q1       = ±acos(cos q1)            ("up" = +, "down" = −)
    q0       = atan2(z, x) − atan2(L2·sin q1, L1 + L2·cos q1)
"""

import math

try:
    from config import LINK_LENGTHS, ELBOW_CONFIG, JOINT_LIMITS
except ImportError:  # allow `import kinematics` without the full config
    LINK_LENGTHS = (0.300, 0.250)
    ELBOW_CONFIG = "up"
    JOINT_LIMITS = ((-math.pi, math.pi), (-math.pi, math.pi))


class Unreachable(ValueError):
    """Target point lies outside the arm's annular workspace."""


def forward(q, links=LINK_LENGTHS):
    """Joint angles (q0, q1) [rad] → end-effector (x, z) [same units as links]."""
    q0, q1 = q
    l1, l2 = links
    x = l1 * math.cos(q0) + l2 * math.cos(q0 + q1)
    z = l1 * math.sin(q0) + l2 * math.sin(q0 + q1)
    return (x, z)


def elbow_position(q, links=LINK_LENGTHS):
    """Joint angles → elbow (joint-2 axis) position (x, z). Handy for plots
    and for self-collision / limit visualisation."""
    q0, _ = q
    l1, _ = links
    return (l1 * math.cos(q0), l1 * math.sin(q0))


def reachable(x, z, links=LINK_LENGTHS, eps=1e-9):
    """True if (x, z) is inside the annulus [|L1−L2|, L1+L2]."""
    l1, l2 = links
    r = math.hypot(x, z)
    return (abs(l1 - l2) - eps) <= r <= (l1 + l2 + eps)


def inverse(x, z, links=LINK_LENGTHS, elbow=ELBOW_CONFIG):
    """End-effector (x, z) → joint angles (q0, q1) [rad].

    `elbow` selects the branch: "up" (q1 ≥ 0) or "down" (q1 ≤ 0).
    Raises Unreachable if the point is outside the workspace.
    """
    l1, l2 = links
    r2 = x * x + z * z
    c1 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    # Clamp tiny numerical overshoot before acos; reject genuine misses.
    if c1 < -1.0 - 1e-6 or c1 > 1.0 + 1e-6:
        raise Unreachable(f"point ({x:.4f}, {z:.4f}) outside workspace "
                          f"[{abs(l1 - l2):.4f}, {l1 + l2:.4f}]")
    c1 = max(-1.0, min(1.0, c1))
    q1 = math.acos(c1)
    if elbow == "down":
        q1 = -q1
    elif elbow != "up":
        raise ValueError(f"elbow must be 'up' or 'down', got {elbow!r}")
    q0 = math.atan2(z, x) - math.atan2(l2 * math.sin(q1), l1 + l2 * math.cos(q1))
    # Normalise q0 into (−π, π].
    q0 = math.atan2(math.sin(q0), math.cos(q0))
    return (q0, q1)


def inverse_both(x, z, links=LINK_LENGTHS):
    """Return both IK branches as {"up": (q0, q1), "down": (q0, q1)}.
    Raises Unreachable if the point is outside the workspace."""
    return {
        "up": inverse(x, z, links, "up"),
        "down": inverse(x, z, links, "down"),
    }


def within_joint_limits(q, limits=JOINT_LIMITS):
    """True if every joint angle is inside its (min, max) software limit."""
    return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, limits))


def workspace_bounds(links=LINK_LENGTHS):
    """Inner/outer radii of the reachable annulus (r_min, r_max)."""
    l1, l2 = links
    return (abs(l1 - l2), l1 + l2)
