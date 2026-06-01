"""
Generate figures for the thesis (docs/figures/*.png) from the real models and
config. Reproducible: re-run after re-calibration to refresh the plots.

    python docs/make_figures.py
"""
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
sys.path.insert(0, SRC)

import kinematics as kin
import gravity_model as gm
import stiffness as st

FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
L1, L2 = kin.LINK_LENGTHS
DEG = 180.0 / math.pi


def _save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, name), dpi=150)
    plt.close(fig)
    print("wrote", name)


# 1. Synchronized trapezoidal profile -------------------------------------
def fig_trapezoid():
    d, v, a = 1.4, 1.0, 2.0          # rad, rad/s, rad/s^2 (illustrative)
    d_ramp = v * v / a
    if d <= d_ramp:
        T = 2 * math.sqrt(d / a); v = a * T / 2
    else:
        T = d / v + v / a
    ta = v / a
    t = np.linspace(0, T, 500)
    s = np.where(t < ta, 0.5 * a * t**2,
        np.where(t < T - ta, 0.5 * a * ta**2 + v * (t - ta),
                 d - 0.5 * a * (T - t)**2))
    vel = np.where(t < ta, a * t, np.where(t < T - ta, v, a * (T - t)))
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 4.5), sharex=True)
    ax1.plot(t, vel, lw=2); ax1.set_ylabel("velocity, rad/s")
    ax1.axvline(ta, ls="--", c="gray"); ax1.axvline(T - ta, ls="--", c="gray")
    ax1.set_title("Trapezoidal velocity profile (one joint)")
    ax1.grid(alpha=.3)
    ax2.plot(t, s, lw=2, c="C1"); ax2.set_ylabel("position, rad")
    ax2.set_xlabel("time, s"); ax2.grid(alpha=.3)
    ax1.text(ta/2, v*0.4, "accel", ha="center"); ax1.text(T/2, v*1.05, "cruise", ha="center")
    _save(fig, "fig_trapezoid.png")


# 2. Workspace annulus + a sample pose ------------------------------------
def fig_workspace():
    rmin, rmax = abs(L1 - L2), L1 + L2
    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    th = np.linspace(0, 2*math.pi, 400)
    for r, c in ((rmax, "C0"), (rmin, "C3")):
        ax.plot(r*np.cos(th), r*np.sin(th), c=c, lw=1.5)
    ax.fill(np.r_[rmax*np.cos(th), rmin*np.cos(th[::-1])],
            np.r_[rmax*np.sin(th), rmin*np.sin(th[::-1])], color="C0", alpha=.08)
    q0, q1 = math.radians(45), math.radians(-60)
    ex, ez = kin.elbow_position((q0, q1)); tx, tz = kin.forward((q0, q1))
    ax.plot([0, ex, tx], [0, ez, tz], "-o", c="k", lw=2.5, ms=6)
    ax.annotate("shoulder", (0, 0), textcoords="offset points", xytext=(6, -12))
    ax.annotate("elbow", (ex, ez), textcoords="offset points", xytext=(6, 6))
    ax.annotate("tip", (tx, tz), textcoords="offset points", xytext=(6, 6))
    ax.axhline(0, c="gray", lw=.6); ax.axvline(0, c="gray", lw=.6)
    ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.set_xlabel("x, mm"); ax.set_ylabel("z, mm")
    ax.set_title(f"Workspace annulus  r in [{rmin:.0f}, {rmax:.0f}] mm")
    _save(fig, "fig_workspace.png")


# 3. Identified gravity load G(q) -----------------------------------------
def fig_gravity():
    g = gm.GravityModel.load()
    if g is None:
        print("skip gravity: no calib"); return
    q0 = np.linspace(0, math.pi, 200)
    fig, (axa, axb) = plt.subplots(1, 2, figsize=(9, 3.8))
    for q1 in (math.radians(-60), 0.0, math.radians(60)):
        m0 = [g.predict((a, q1))[0] for a in q0]
        m1 = [g.predict((a, q1))[1] for a in q0]
        axa.plot(q0*DEG, m0, label=f"q1={q1*DEG:.0f}deg")
        axb.plot(q0*DEG, m1, label=f"q1={q1*DEG:.0f}deg")
    for ax, t in ((axa, "shoulder load m0(q)"), (axb, "elbow load m1(q)")):
        ax.set_xlabel("q0, deg"); ax.set_ylabel("load torque, per-mille")
        ax.set_title(t); ax.grid(alpha=.3); ax.legend(fontsize=8); ax.axhline(0, c="gray", lw=.6)
    fig.suptitle("Identified static gravity load G(q) (from gravcal)")
    _save(fig, "fig_gravity.png")


# 4. Predicted deflection delta(q) = G/K_eff ------------------------------
def fig_deflection():
    g = gm.GravityModel.load(); s = st.StiffnessModel.load()
    if g is None or s is None:
        print("skip deflection: no calib"); return
    q0 = np.linspace(0, math.pi, 200)
    fig, ax = plt.subplots(figsize=(6, 3.8))
    for q1 in (math.radians(-60), 0.0, math.radians(60)):
        d = [s.deflection((a, q1), g.predict((a, q1))) for a in q0]
        ax.plot(q0*DEG, [di[0]*DEG for di in d], label=f"d0, q1={q1*DEG:.0f}")
    ax.set_xlabel("q0, deg"); ax.set_ylabel("shoulder deflection d0, deg")
    ax.set_title("Predicted elastic deflection (shoulder)  d = G/K_eff")
    ax.grid(alpha=.3); ax.legend(fontsize=8); ax.axhline(0, c="gray", lw=.6)
    _save(fig, "fig_deflection.png")


# 5. Friction / bidirectional de-biasing schematic ------------------------
def fig_friction():
    load = 60.0; coul = 100.0
    v = np.linspace(-1, 1, 400)
    tau = load + coul*np.sign(v) + 8*v   # Coulomb + small viscous
    fig, ax = plt.subplots(figsize=(6, 3.8))
    ax.plot(v, tau, lw=2)
    ax.axhline(load, ls="--", c="C2", label="load = (fwd+rev)/2")
    ax.axhline(load+coul, ls=":", c="C3"); ax.axhline(load-coul, ls=":", c="C3")
    ax.annotate("forward", (0.5, load+coul+12)); ax.annotate("reverse", (-0.8, load-coul-22))
    ax.set_xlabel("joint velocity (sign)"); ax.set_ylabel("motor torque, per-mille")
    ax.set_title("Bidirectional de-frictioning: friction cancels in the mean")
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    _save(fig, "fig_friction.png")


# 6. Elbow observability: J1 vanishes at q0+q1 = 90 deg -------------------
def fig_observability():
    s = np.linspace(-150, 200, 300)            # q0+q1 in deg
    J1 = L2*np.cos(np.radians(s))
    fig, ax = plt.subplots(figsize=(6, 3.4))
    ax.plot(s, J1, lw=2)
    ax.axhline(0, c="gray", lw=.6); ax.axvline(90, ls="--", c="C3")
    ax.annotate("J1=0 -> elbow invisible\nto tip height", (90, 0),
                textcoords="offset points", xytext=(8, 60),
                arrowprops=dict(arrowstyle="->", color="C3"))
    ax.set_xlabel("q0+q1, deg"); ax.set_ylabel("J1 = L2 cos(q0+q1), mm")
    ax.set_title("Stiffness observability (elbow column of the tip Jacobian)")
    ax.grid(alpha=.3)
    _save(fig, "fig_observability.png")


if __name__ == "__main__":
    fig_trapezoid(); fig_workspace(); fig_gravity()
    fig_deflection(); fig_friction(); fig_observability()
    print("done ->", FIG)
