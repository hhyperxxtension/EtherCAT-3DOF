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
import os
import sys
import time

import kinematics as kin
import joint_map as jm
from trajectory import plan_ptp
from ethercat_csp import CSPBus
from config import (
    JOINT_LIMITS, JOINT_VEL_MAX, JOINT_ACC_MAX, ELBOW_CONFIG,
    JOINT_ZERO_RAD,
)

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

    # ── lifecycle ───────────────────────────────────────────────────────
    def connect(self):
        self.bus.connect()

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
    def plan_move(self, x, z):
        """Validate and plan a move to (x, z). Returns (trajectory, q_target).
        Raises before any motion if the move is unsafe."""
        self._require_home()
        if not kin.reachable(x, z):
            rmin, rmax = kin.workspace_bounds()
            raise ValueError(f"({x:.1f}, {z:.1f}) mm unreachable - workspace r in [{rmin:.1f}, {rmax:.1f}] mm")
        q_target = kin.inverse(x, z, elbow=self.elbow)   # raises Unreachable
        if not kin.within_joint_limits(q_target, JOINT_LIMITS):
            raise ValueError(f"target joints {_deg(q_target)} violate joint limits")
        q_start = self.current_q()
        traj = plan_ptp(q_start, q_target, self.v_max, self.a_max)
        return traj, q_target

    def move_by(self, dx, dz):
        """Relative move: target = current (x, z) + (dx, dz), all in mm."""
        x0, z0 = self.current_xz()
        return self.move_to(x0 + dx, z0 + dz)

    def move_to(self, x, z):
        """Plan, validate, and start a move to (x, z) [mm]. Returns duration
        [s]. Non-blocking — poll bus.is_busy() or call wait()."""
        traj, _ = self.plan_move(x, z)
        vectors = []
        for _t, q in traj.samples(dt=self.bus.cycle_time):
            c = jm.angles_to_counts(q, self.home_counts)
            for ci in c:
                if not jm.counts_in_int32(ci):
                    raise ValueError(f"count {ci} exceeds drive 32-bit range")
            vectors.append(c)
        self.bus.play_trajectory(vectors)
        return traj.duration

    def wait(self, poll=0.05):
        while self.bus.is_busy():
            time.sleep(poll)

    def stop(self):
        self.bus.freeze()


# ── helpers ──────────────────────────────────────────────────────────────
def _deg(q):
    import math
    return tuple(round(math.degrees(a), 2) for a in q)


# ── CLI ────────────────────────────────────────────────────────────────
def _print_where(rc):
    try:
        q = rc.current_q()
        x, z = kin.forward(q)
        counts = rc.bus.actual_counts()
        print(f"  q = {_deg(q)} deg   (x, z) = ({x:.1f}, {z:.1f}) mm   counts = {counts}")
    except NotHomed:
        print("  not homed")


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
    print("  move <x> <z>     absolute target (mm)")
    print("  rmove <dx> <dz>  relative to current position (mm)")
    print("  elbow up|down | speed <v> <a> | stop | q")
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
            elif cmd == "stop":
                rc.stop()
                print("  -> stop / hold")
            elif cmd in ("move", "rmove") and len(parts) == 3:
                try:
                    a = float(parts[1]); b = float(parts[2])
                except ValueError:
                    print(f"  {cmd} <x> <z> - numbers in mm")
                    continue
                try:
                    if cmd == "rmove":
                        dur = rc.move_by(a, b)
                        label = f"relative ({a:+.1f}, {b:+.1f}) mm"
                    else:
                        dur = rc.move_to(a, b)
                        label = f"({a:.1f}, {b:.1f}) mm"
                except (ValueError, kin.Unreachable, NotHomed) as e:
                    print(f"  rejected: {e}")
                    continue
                print(f"  moving to {label} - {dur:.2f}s. Ctrl+C to abort.")
                try:
                    while rc.bus.is_busy():
                        snap, *_ = rc.bus.status()
                        trq = ' '.join(f"D{i}={s['trq']:+d}" for i, s in enumerate(snap))
                        sys.stdout.write(f"\r  trq {trq}   ")
                        sys.stdout.flush()
                        time.sleep(0.1)
                    print("\n  arrived.")
                    _print_where(rc)
                except KeyboardInterrupt:
                    rc.stop()
                    print("\n  aborted - holding position.")
            else:
                print("  unknown command")
    finally:
        print()
        rc.disconnect()


if __name__ == "__main__":
    main()
