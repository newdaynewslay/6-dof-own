#!/usr/bin/env python3
"""Figure of the scripted test sequence (Ball_Balance_Compare.ino buildSequence(), 2026-11-19 version).

Top: the set-point patterns drawn on the plate (steps, three circles, pentagram).
Bottom: set point x/y and the added disturbance tilt against time for one whole run (~94.5 s),
taken from a log with the disturbance part (r n keep seed 1).

Usage: python3 test_patterns.py LOG.csv
Output: fig_test_patterns.png next to this script.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

MM = 1.045                       # mm per px
EXT_R = 50                       # px, edge points
CIRCLES = [(60, 4.0, 2, "#eb6834", "-"), (40, 2.0, 3, "#c98500", "--"), (40, 6.0, 2, "#c98500", ":")]
STAR_R, STAR_V = 60, 60.0        # px, px/s
PLATE, NUT = 205.0, 85.0         # mm
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, AQUA, RED = "#2a78d6", "#eb6834", "#1baf7a", "#b4372f"


def plate(ax, title):
    ax.add_patch(Rectangle((-PLATE / 2, -PLATE / 2), PLATE, PLATE, fc="#f4f3f0", ec=INK2, lw=1.2))
    for sx in (-1, 1):
        for sy in (-1, 1):
            ax.add_patch(Circle((sx * NUT, sy * NUT), 5, fc="#c3c2b7", ec=INK2, lw=0.8))
    ax.plot(0, 0, "+", color=INK2, ms=10, mew=1)
    ax.set_xlim(-112, 112); ax.set_ylim(-112, 112)
    ax.set_aspect("equal")
    ax.set_xticks([-100, -50, 0, 50, 100]); ax.set_yticks([-100, -50, 0, 50, 100])
    ax.tick_params(colors=INK2, labelsize=8)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.set_xlabel("x [mm]", color=INK)


def arrow(ax, p, q, color, lw=1.6):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=12, color=color, lw=lw,
                                 shrinkA=0, shrinkB=0))


def star_vertex(k, R):
    order = [0, 2, 4, 1, 3]
    a = np.radians(90 + 72 * order[k % 5])
    return R * np.cos(a), R * np.sin(a)


def read_log(path):
    segs, rows = [], []
    for line in open(path, encoding="utf-8"):
        if line.startswith("# SEG"):
            p = line.split()
            d = {"name": p[3]}
            d.update({k: float(v) for k, v in (kv.split("=") for kv in p[4:])})
            segs.append(d)
        elif line[:1].isdigit():
            v = line.strip().split(",")
            rows.append([float(x) for x in v])
    n = min(len(r) for r in rows)
    return segs, np.array([r[:n] for r in rows])


def main(log):
    segs, a = read_log(log)
    fig = plt.figure(figsize=(10, 8))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1.0], hspace=0.35, wspace=0.12)
    gsb = gs[1, :].subgridspec(2, 1, height_ratios=[0.75, 0.25], hspace=0.08)

    # (a) steps
    ax = fig.add_subplot(gs[0, 0])
    R = EXT_R * MM
    plate(ax, f"(a) Steps: centre → edge → centre\n4 × (3 s out + 3 s back), r = {R:.0f} mm")
    for deg in (0, 90, 180, 270):
        t = np.radians(deg)
        p = (R * np.cos(t), R * np.sin(t))
        arrow(ax, (0, 0), (0.9 * p[0], 0.9 * p[1]), BLUE)
        ax.plot(*p, "o", color=BLUE, ms=7, mec="white", mew=1.5)
    ax.text(0, -96, "order shuffled per block", ha="center", fontsize=8, color=INK2)
    ax.set_ylabel("y [mm]", color=INK)

    # (b) circles
    ax = fig.add_subplot(gs[0, 1])
    plate(ax, "(b) Circles: r 63 mm at 4 s/turn,\nr 42 mm at 2 and 6 s/turn")
    th = np.linspace(0, 2 * np.pi, 300)
    for Rp, T, turns, col, ls in CIRCLES[:2]:
        Rc = Rp * MM
        ax.plot(Rc * np.cos(th), Rc * np.sin(th), color=col, lw=1.8, ls="-")
        a0 = np.radians(40)
        arrow(ax, (Rc * np.cos(a0), Rc * np.sin(a0)), (Rc * np.cos(a0 + 0.3), Rc * np.sin(a0 + 0.3)), col)
        ax.plot(Rc, 0, "o", color=col, ms=6, mec="white", mew=1.5)
    ax.set_yticklabels([])

    # (c) pentagram
    ax = fig.add_subplot(gs[0, 2])
    Rs = STAR_R * MM
    plate(ax, f"(c) Pentagram\ncorner r = {Rs:.0f} mm, {STAR_V*MM:.0f} mm/s, 1 loop")
    pts = [star_vertex(k, Rs) for k in range(6)]
    for k in range(5):
        p, q = np.array(pts[k]), np.array(pts[k + 1])
        ax.plot(*zip(p, q), color=AQUA, lw=1.8)
        arrow(ax, tuple(p + 0.45 * (q - p)), tuple(p + 0.55 * (q - p)), AQUA)
    for k in range(5):
        x, y = pts[k]
        ax.plot(x, y, "o", color=AQUA, ms=6 if k else 7, mec="white", mew=1.5)
        ax.text(x * 1.22, y * 1.22, str(k + 1), ha="center", va="center", fontsize=9, color=INK)
    ax.set_yticklabels([])

    # (d) timeline + (e) disturbance
    ax = fig.add_subplot(gsb[0])
    axd = fig.add_subplot(gsb[1], sharex=ax)
    t, spx, spy = a[:, 0] / 1000, a[:, 2] * MM, a[:, 3] * MM
    t = t - t[0]
    t0 = 0.0
    for s in segs:
        s["t0"], t0 = t0, t0 + s["dur"]
        s["t1"] = t0
        n = s["name"]
        col = (BLUE if n.startswith(("out_", "back_")) else ORANGE if n.startswith("circle") and not n.endswith("_in")
               else AQUA if n == "star" else RED if n.startswith(("kick", "push", "release")) else None)
        for axx in (ax, axd):
            if col:
                axx.axvspan(s["t0"], s["t1"], color=col, alpha=0.10, lw=0)
            axx.axvline(s["t0"], color=GRID, lw=0.8)
    ax.plot(t, spx, color=INK, lw=1.4, label="set point x")
    ax.plot(t, spy, color=INK2, lw=1.4, ls="--", label="set point y")

    def mid(*names):
        ss = [s for s in segs if s["name"].startswith(names)]
        return (ss[0]["t0"] + ss[-1]["t1"]) / 2
    for txt, x in [("hold", mid("hold")), ("steps", mid("out_", "back_")), ("circles", mid("circle")),
                   ("pentagram", mid("star")), ("disturbance", mid("kick", "push", "release"))]:
        ax.text(x, 84, txt, ha="center", fontsize=8, color=INK2)
    ax.set_ylim(-80, 95)
    ax.set_ylabel("Set point [mm]", color=INK)
    ax.set_title(f"(d) Set point and added disturbance over one run ({t0:.1f} s per controller)",
                 loc="left", fontsize=10, color=INK)
    ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=8, loc="lower left", ncol=2)
    plt.setp(ax.get_xticklabels(), visible=False)

    if a.shape[1] >= 17:
        axd.plot(t, a[:, 15], color=RED, lw=1.3, label="disturbance x")
        axd.plot(t, a[:, 16], color=RED, lw=1.3, ls="--", label="disturbance y")
        axd.legend(frameon=False, fontsize=8, loc="upper left", ncol=2)
    axd.set_ylim(-3.8, 3.8); axd.set_yticks([-3, 0, 3])
    axd.set_ylabel("Tilt [°]", color=INK)
    axd.set_xlabel("Time [s]", color=INK)
    axd.set_xlim(0, t0)
    for axx in (ax, axd):
        for s in ("top", "right"):
            axx.spines[s].set_visible(False)
        axx.tick_params(colors=INK2, labelsize=8)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig_test_patterns.png")
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(out, t0)


if __name__ == "__main__":
    main(sys.argv[1])
