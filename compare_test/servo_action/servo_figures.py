#!/usr/bin/env python3
"""Servo-action figures for the AIS4001 report.

Reads the compare_test logs (one folder per 4-way run), rebuilds what the
servos were actually told to do, and writes four figures:

  fig_servo_tradeoff.png   tracking error (IAE) vs servo travel, every run
  fig_servo_time.png       tilt command and servo a1 angle, step + centre hold
  fig_servo_psd.png        power spectrum of the tilt command
  fig_servo_bars.png       servo travel / update rate / RMS tilt per controller

The servo angles are not logged. They are rebuilt here from the logged tilt
command u_x, u_y with the same steps as Ball_Balance_Compare.ino:
  1. r_max = 15 deg clamp (already in the log),
  2. tilt_db = 0.25 deg deadband (a new target is sent only when the tilt
     changed by more than this on either axis),
  3. InverseKinematics(0, 0, hz_home, roll = -u_y, pitch = +u_x, 0),
     a Python port of the sketch's IK, same geometry.
So the angles are the commanded targets, not measured servo positions.

Usage:  python3 servo_figures.py LOGDIR [LOGDIR ...]   (newest run = last)
"""
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import welch

MM_PER_PX = 1.045
COLS = ["t_ms", "seg", "sp_x", "sp_y", "ball_x", "ball_y", "err_x", "err_y",
        "derr_x", "derr_y", "u_x", "u_y", "sat", "found", "ik"]
CTRLS = [("pid", "PID"), ("kf", "KF-PID"), ("lqr", "LQR"), ("kflqr", "KF-LQR")]
COLOR = {"PID": "#2a78d6", "KF-PID": "#eb6834", "LQR": "#1baf7a", "KF-LQR": "#c98500"}
LS = {"PID": "-", "KF-PID": "--", "LQR": "-", "KF-LQR": "--"}
MARK = {"PID": "o", "KF-PID": "s", "LQR": "^", "KF-LQR": "D"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

# ----------------------------------------------------------------- geometry (mm)
l0, d1, lf, m_, p1, p2 = 55.0, 55.0, 65.0, 12.7, 31.0, 150.0
hz_home, tilt_db = 145.0, 0.25
s3 = np.sqrt(3.0)
a10 = np.array([-d1, -l0, 0]); a20 = np.array([d1, -l0, 0])
b10 = np.array([l0 * s3 / 2 + d1 / 2, l0 / 2 - d1 * s3 / 2, 0])
b20 = np.array([l0 * s3 / 2 - d1 / 2, l0 / 2 + d1 * s3 / 2, 0])
c10 = np.array([-b20[0], b20[1], 0]); c20 = np.array([-b10[0], b10[1], 0])
ax_a, ax_b, ax_c = a20 - a10, b20 - b10, c20 - c10
na = np.array([0, -1, 0]); nb = np.array([s3 / 2, .5, 0]); nc = np.array([-s3 / 2, .5, 0])
t_ = lf / 2 + m_ * s3 / 2
w_ = lf * s3 / 2 - m_ / 2
H = {"a": np.array([0, -t_, 0]), "b": np.array([s3 * t_ / 2, t_ / 2, 0]),
     "c": np.array([-s3 * t_ / 2, t_ / 2, 0])}
ARM = [("a", np.array([-w_, 0, 0]), a10, na, ax_a, +1),
       ("a", np.array([w_, 0, 0]), a20, na, ax_a, -1),
       ("b", np.array([w_ / 2, -w_ * s3 / 2, 0]), b10, nb, ax_b, +1),
       ("b", np.array([-w_ / 2, w_ * s3 / 2, 0]), b20, nb, ax_b, -1),
       ("c", np.array([w_ / 2, w_ * s3 / 2, 0]), c10, nc, ax_c, +1),
       ("c", np.array([-w_ / 2, -w_ * s3 / 2, 0]), c20, nc, ax_c, -1)]
SERVO_ID = ["a1", "a2", "b1", "b2", "c1", "c2"]


def rot(roll, pitch):
    r, p = np.radians(roll), np.radians(pitch)
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return Ry @ Rx  # yaw = 0


def ik(roll, pitch):
    R = rot(roll, pitch)
    h = np.array([0, 0, hz_home])
    th = np.empty(6)
    for i, (side, off, base, n, axis, sgn) in enumerate(ARM):
        v = h + R @ H[side] + R @ off - base
        s = abs(v @ n)
        proj = v + n * s
        pm = np.linalg.norm(proj)
        p2p = np.sqrt(p2 * p2 - s * s)
        c1 = np.clip(sgn * (proj @ axis) / (2 * d1 * pm), -1, 1)
        c2 = np.clip((pm * pm + p1 * p1 - p2p * p2p) / (2 * pm * p1), -1, 1)
        th[i] = -np.degrees(np.arccos(c1) - np.arccos(c2))  # negative = up
    return th


# ----------------------------------------------------------------- loading
class Run:
    pass


def load(path):
    r = Run()
    r.segs, rows = [], []
    for line in open(path):
        line = line.strip()
        if line.startswith("# SEG"):
            p = line.split()
            d = {"i": int(p[2]), "name": p[3]}
            d.update({k: float(v) for k, v in (kv.split("=") for kv in p[4:])})
            r.segs.append(d)
        elif line and line[0].isdigit():
            parts = line.split(",")
            if len(parts) == len(COLS):
                rows.append([float(v) for v in parts])
    a = np.array(rows)
    r.d = {c: a[:, i] for i, c in enumerate(COLS)}
    r.t = r.d["t_ms"] / 1000
    ok = r.d["found"] == 1
    r.e = np.where(ok, np.hypot(r.d["ball_x"] - r.d["sp_x"], r.d["ball_y"] - r.d["sp_y"]), np.nan) * MM_PER_PX
    t0 = 0
    for s in r.segs:
        s["t0"], t0 = t0, t0 + s["dur"]
        s["t1"] = t0
    # deadband -> what was sent, then IK -> servo targets
    ux, uy = r.d["u_x"], r.d["u_y"]
    sx = np.zeros_like(ux); sy = np.zeros_like(uy); upd = np.zeros(len(ux), bool)
    lx = ly = 0.0
    for k in range(len(ux)):
        if abs(ux[k] - lx) > tilt_db or abs(uy[k] - ly) > tilt_db:
            lx, ly, upd[k] = ux[k], uy[k], True
        sx[k], sy[k] = lx, ly
    r.sx, r.sy, r.upd = sx, sy, upd
    r.theta = np.array([ik(-y, x) for x, y in zip(sx, sy)])  # roll=-u_y, pitch=+u_x
    return r


def metrics(r):
    T = r.t[-1] - r.t[0]
    dth = np.abs(np.diff(r.theta, axis=0))
    return {
        "iae": float(np.nansum(r.e[1:] * np.diff(r.t))),
        "travel": float(dth.sum(axis=0).mean() / T),        # deg/s per servo
        "tilt_rate": float(np.nansum(np.abs(np.diff(np.hypot(r.d["u_x"], r.d["u_y"])))) / T),
        "upd_pct": float(r.upd.mean() * 100),
        "rms_tilt": float(np.sqrt(np.mean(r.d["u_x"] ** 2 + r.d["u_y"] ** 2))),
        "rev_per_s": float(np.mean([np.sum(np.diff(np.sign(np.diff(r.theta[r.upd, j]))) != 0)
                                    for j in range(6)]) / T),
    }


def style(ax):
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.xaxis.label.set_color(INK); ax.yaxis.label.set_color(INK)


# ----------------------------------------------------------------- main
def main(dirs, outdir):
    runs = []  # list of dict ctrl -> Run, oldest first
    for d in dirs:
        stamp = os.path.basename(os.path.normpath(d))
        rr = {}
        for key, name in CTRLS:
            f = glob.glob(os.path.join(d, f"*_{key}.csv"))
            if f:
                rr[name] = load(f[0])
        runs.append((stamp, rr))
    latest = runs[-1][1]
    M = {stamp: {c: metrics(r) for c, r in rr.items()} for stamp, rr in runs}

    plt.rcParams.update({"font.size": 10, "axes.titlesize": 11, "font.family": "DejaVu Sans"})

    # 1. trade-off: IAE vs servo travel
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    for c in COLOR:
        xs = [M[s][c]["travel"] for s, _ in runs if c in M[s]]
        ys = [M[s][c]["iae"] for s, _ in runs if c in M[s]]
        ax.plot(xs, ys, color=COLOR[c], ls=":", lw=1, alpha=.6)
        ax.scatter(xs, ys, s=60, marker=MARK[c], color=COLOR[c], edgecolor="white", lw=1.5, zorder=3, label=c)
        ax.annotate(c, (np.mean(xs), np.mean(ys)), xytext=(8, 6), textcoords="offset points", fontsize=9, color=INK)
    ax.set_xlabel("Mean servo travel per servo [deg/s]")
    ax.set_ylabel("Total tracking IAE [mm·s]")
    ax.set_title("Tracking error vs. servo effort (one point per test run)", loc="left", color=INK)
    style(ax)
    ax.legend(frameon=False, fontsize=9, loc="upper right")
    ax.text(0.01, 0.02, "better ↙", transform=ax.transAxes, color=INK2, fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "fig_servo_tradeoff.png"), dpi=200); plt.close(fig)

    # 2. time series: step out_000 + back_000, and centre hold
    seg = {s["name"]: s for s in latest["PID"].segs}
    win = [("Step centre → +x edge and back", seg["out_000"]["t0"] - 0.3, seg["back_000"]["t1"]),
           ("Centre hold (last 3 s of test)", seg["end"]["t1"] - 3.0, seg["end"]["t1"])]
    fig, axs = plt.subplots(3, 2, figsize=(10, 7.2), sharex="col",
                            gridspec_kw={"height_ratios": [1, 1, 1]})
    th0 = ik(0, 0)
    for j, (title, ta, tb) in enumerate(win):
        for c, r in latest.items():
            mk = (r.t >= ta) & (r.t <= tb)
            t = r.t[mk] - ta
            axs[0, j].plot(t, r.d["ball_x"][mk] * MM_PER_PX, color=COLOR[c], ls=LS[c], lw=1.6, label=c)
            axs[1, j].plot(t, r.d["u_x"][mk], color=COLOR[c], ls=LS[c], lw=1.4, label=c)
            axs[2, j].step(t, r.theta[mk, 0] - th0[0], where="post", color=COLOR[c], ls=LS[c], lw=1.4, label=c)
        if j == 0:
            mk = (latest["PID"].t >= ta) & (latest["PID"].t <= tb)
            axs[0, j].plot(latest["PID"].t[mk] - ta, latest["PID"].d["sp_x"][mk] * MM_PER_PX,
                           color=INK2, lw=1, ls=(0, (1, 2)), label="set point")
        axs[0, j].set_title(title, loc="left", color=INK)
        axs[2, j].set_xlabel("Time [s]")
        for a in axs[:, j]:
            style(a)
    axs[0, 0].set_ylabel("Ball x [mm]")
    axs[1, 0].set_ylabel("Tilt cmd $u_x$ [deg]")
    axs[2, 0].set_ylabel("Servo a1 Δθ [deg]")
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, frameon=False, fontsize=9, ncol=5, loc="upper center")
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(outdir, "fig_servo_time.png"), dpi=200); plt.close(fig)

    # 3. PSD of the tilt command (whole test)
    fig, ax = plt.subplots(figsize=(6.2, 4.0))
    for c, r in latest.items():
        fs = 1 / np.median(np.diff(r.t))
        f, P = welch(r.d["u_x"], fs=fs, nperseg=512)
        f2, P2 = welch(r.d["u_y"], fs=fs, nperseg=512)
        ax.semilogy(f, P + P2, color=COLOR[c], ls=LS[c], lw=1.6, label=c)
        hf = (P + P2)[f > 5].sum() / (P + P2).sum() * 100
        print(f"{c}: {hf:.1f} % of tilt-command power above 5 Hz")
    ax.axvspan(5, fs / 2, color=GRID, alpha=0.35, lw=0)
    ax.text(5.3, ax.get_ylim()[1] * 0.4, "above ball-motion band (> 5 Hz)", fontsize=8, color=INK2, va="top")
    ax.set_xlim(0, fs / 2)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD of tilt command [deg²/Hz]")
    ax.set_title("Frequency content of the tilt command", loc="left", color=INK)
    style(ax)
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "fig_servo_psd.png"), dpi=200); plt.close(fig)

    # 4. bars: travel, update rate, RMS tilt (mean over runs, whiskers = min/max)
    keys = [("travel", "Servo travel [deg/s]"), ("rev_per_s", "Direction reversals [1/s]"),
            ("upd_pct", "Frames with new target [%]"), ("rms_tilt", "RMS tilt [deg]")]
    fig, axs = plt.subplots(1, 4, figsize=(11, 3.2))
    names = list(COLOR)
    for a, (k, lab) in zip(axs, keys):
        vals = [[M[s][c][k] for s, _ in runs if c in M[s]] for c in names]
        mean = [np.mean(v) for v in vals]
        lo = [m - min(v) for m, v in zip(mean, vals)]
        hi = [max(v) - m for m, v in zip(mean, vals)]
        a.bar(range(4), mean, color=[COLOR[c] for c in names], width=0.7, edgecolor="white", lw=2)
        if len(runs) > 1:
            a.errorbar(range(4), mean, yerr=[lo, hi], fmt="none", ecolor=INK2, capsize=3, lw=1)
        top = max(m + h for m, h in zip(mean, hi))
        for i, (m, h) in enumerate(zip(mean, hi)):
            a.text(i, m + h + 0.02 * top, f"{m:.1f}", ha="center", va="bottom", fontsize=8, color=INK)
        a.set_ylim(0, top * 1.15)
        a.set_xticks(range(4)); a.set_xticklabels(names, fontsize=8, rotation=20)
        a.set_title(lab, loc="left", fontsize=10, color=INK)
        style(a); a.grid(axis="x", visible=False)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "fig_servo_bars.png"), dpi=200); plt.close(fig)

    # table for the text
    with open(os.path.join(outdir, "servo_metrics.md"), "w") as fh:
        fh.write("| Run | Ctrl | IAE [mm s] | Servo travel [deg/s] | Reversals [1/s] | New target [%] | Tilt rate [deg/s] | RMS tilt [deg] |\n")
        fh.write("|---|---|---|---|---|---|---|---|\n")
        for s, _ in runs:
            for c in names:
                if c in M[s]:
                    q = M[s][c]
                    fh.write(f"| {s} | {c} | {q['iae']:.0f} | {q['travel']:.1f} | {q['rev_per_s']:.2f} | "
                             f"{q['upd_pct']:.0f} | {q['tilt_rate']:.1f} | {q['rms_tilt']:.2f} |\n")
    print(open(os.path.join(outdir, "servo_metrics.md")).read())


if __name__ == "__main__":
    dirs = sys.argv[1:]
    out = os.path.dirname(os.path.abspath(__file__))
    main(dirs, out)
