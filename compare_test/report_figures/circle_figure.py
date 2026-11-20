#!/usr/bin/env python3
"""Clearer circle-tracking figure (alternative to fig_xy.png, which is left untouched).

Top row: one panel per circle. The ball path is averaged over all runs of a controller
(first turn dropped), binned by set-point angle, so each controller is one clean line.
Bottom row: radius error (ball radius - set-point radius) against set-point angle,
which shows where on the plate each controller cuts inside or bulges out.

Usage: python3 circle_figure.py LOGDIR
Output: fig_circles.png next to this script.
"""
import glob, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MM = 1.045
CTRL = [("pid", "PID", "#2a78d6"), ("kf", "KF-PID", "#eb6834"),
        ("lqr", "LQR", "#1baf7a"), ("kflqr", "KF-LQR", "#c98500")]
CIRC = [("circle", "(a) r 63 mm, 4 s/turn (0.25 Hz)"),
        ("circle3", "(b) r 42 mm, 6 s/turn (0.17 Hz)"),
        ("circle2", "(c) r 42 mm, 2 s/turn (0.5 Hz)")]
NB = 72
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def read_log(path):
    segs, rows = {}, []
    for line in open(path, encoding="utf-8"):
        if line.startswith("# SEG"):
            p = line.split()
            d = {k: float(v) for k, v in (kv.split("=") for kv in p[4:])}
            segs[p[3]] = (int(p[2]), d)
        elif line[:1].isdigit():
            rows.append([float(x) for x in line.strip().split(",")])
    n = min(len(r) for r in rows)
    return segs, np.array([r[:n] for r in rows])


def circle_bins(a, segs, name):
    idx, d = segs[name]
    s = a[(a[:, 1] == idx) & (a[:, 13] == 1)]
    t = (s[:, 0] - s[0, 0]) / 1000
    s = s[t >= d["p"]]                      # drop first turn
    ang = np.mod(np.arctan2(s[:, 3], s[:, 2]), 2 * np.pi)
    b = (ang / (2 * np.pi) * NB).astype(int) % NB
    return b, s[:, 4] * MM, s[:, 5] * MM, d["R"] * MM


def main(logdir):
    data = {}
    for key, _, _ in CTRL:
        files = sorted(glob.glob(os.path.join(logdir, f"*_{key}.csv")))
        data[key] = [read_log(f) for f in files]

    fig, axs = plt.subplots(2, 3, figsize=(11, 7.2), gridspec_kw={"height_ratios": [1.25, 0.75]})
    th = np.linspace(0, 2 * np.pi, 300)
    centres = (np.arange(NB) + 0.5) / NB * 2 * np.pi
    for j, (name, title) in enumerate(CIRC):
        ax, ar = axs[0, j], axs[1, j]
        R = None
        for key, label, col in CTRL:
            B, X, Y = [], [], []
            for segs, a in data[key]:
                b, x, y, R = circle_bins(a, segs, name)
                B.append(b); X.append(x); Y.append(y)
            B, X, Y = map(np.concatenate, (B, X, Y))
            mx = np.array([X[B == k].mean() for k in range(NB)])
            my = np.array([Y[B == k].mean() for k in range(NB)])
            ax.plot(np.r_[mx, mx[0]], np.r_[my, my[0]], color=col, lw=1.8, label=label)
            rad = np.hypot(mx, my) - R
            deg = np.degrees(centres)
            ar.plot(np.r_[deg, 360], np.r_[rad, rad[0]], color=col, lw=1.6)
        ax.plot(R * np.cos(th), R * np.sin(th), "--", color=INK, lw=1.2, label="set point")
        ax.plot(0, 0, "+", color=INK2, ms=8)
        ax.set_xlim(-85, 85); ax.set_ylim(-85, 85); ax.set_aspect("equal")
        ax.set_title(title, loc="left", fontsize=10, color=INK)
        ax.set_xlabel("x [mm]"); ax.grid(color=GRID, lw=0.6)
        if j == 0:
            ax.set_ylabel("y [mm]")
        ar.axhline(0, color=INK, ls="--", lw=1.0)
        ar.set_xlim(0, 360); ar.set_xticks([0, 90, 180, 270, 360])
        ar.set_xticklabels(["+x\n0°", "+y\n90°", "−x\n180°", "−y\n270°", "+x\n360°"], fontsize=8)
        ar.set_ylim(-35, 25); ar.grid(color=GRID, lw=0.6)
        if j == 0:
            ar.set_ylabel("radius error [mm]")
        for axx in (ax, ar):
            for s in ("top", "right"):
                axx.spines[s].set_visible(False)
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=5, frameon=False, fontsize=9)
    axs[1, 1].set_xlabel("set-point angle on the plate")
    fig.subplots_adjust(left=0.07, right=0.98, top=0.90, bottom=0.10, wspace=0.25, hspace=0.35)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig_circles.png")
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == "__main__":
    main(sys.argv[1])
