#!/usr/bin/env python3
"""Figure of the scripted test sequence (Ball_Balance_Compare.ino buildSequence()).

Top: the three set-point patterns drawn on the plate (steps, circle, pentagram).
Bottom: the set point x/y against time for the whole ~52 s test, taken from a log.

Usage: python3 test_patterns.py [LOG.csv]   (log is optional, only for the timeline)
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

MM = 1.045               # mm per px
EXT_R, CIRC_R, STAR_R = 65, 60, 60   # px
CIRC_T, CIRC_TURNS = 4.0, 2
STAR_V = 60.0            # px/s
PLATE = 205.0            # mm, side
NUT = 85.0               # mm, corner nuts on a 170 mm square
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"


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


def arrow(ax, p, q, color, lw=1.6, style="-|>", ms=12):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=ms, color=color, lw=lw,
                                 shrinkA=0, shrinkB=0))


def star_vertex(k, R):
    order = [0, 2, 4, 1, 3]
    a = np.radians(90 + 72 * order[k % 5])
    return R * np.cos(a), R * np.sin(a)


def main(log):
    fig = plt.figure(figsize=(10, 7))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 0.85], hspace=0.35, wspace=0.12)

    # (a) steps
    ax = fig.add_subplot(gs[0, 0])
    plate(ax, f"(a) Steps: centre → edge → centre\n4 × (3 s out + 3 s back), r = {EXT_R*MM:.0f} mm")
    R = EXT_R * MM
    for k, deg in enumerate([0, 90, 180, 270]):
        a = np.radians(deg)
        p = (R * np.cos(a), R * np.sin(a))
        arrow(ax, (0, 0), (0.93 * p[0], 0.93 * p[1]), BLUE)
        ax.plot(*p, "o", color=BLUE, ms=7, mec="white", mew=1.5)
        off = (12 * np.cos(a) + (8 if deg in (90, 270) else 0), 12 * np.sin(a))
        ax.text(p[0] + off[0], p[1] + off[1], str(k + 1), ha="center", va="center", fontsize=9, color=INK)
    ax.set_ylabel("y [mm]", color=INK)

    # (b) circle
    ax = fig.add_subplot(gs[0, 1])
    plate(ax, f"(b) Fast circle\nr = {CIRC_R*MM:.0f} mm, {CIRC_T:.0f} s/turn ({2*np.pi*CIRC_R*MM/CIRC_T:.0f} mm/s), {CIRC_TURNS} turns")
    Rc = CIRC_R * MM
    th = np.linspace(0, 2 * np.pi, 300)
    ax.plot(Rc * np.cos(th), Rc * np.sin(th), color=ORANGE, lw=1.8)
    for a0 in (np.radians(40), np.radians(220)):
        arrow(ax, (Rc * np.cos(a0), Rc * np.sin(a0)), (Rc * np.cos(a0 + 0.25), Rc * np.sin(a0 + 0.25)), ORANGE)
    ax.plot(Rc, 0, "o", color=ORANGE, ms=7, mec="white", mew=1.5)
    ax.text(Rc + 6, -9, "start", fontsize=8, color=INK2)
    v = 2 * np.pi * Rc / CIRC_T
    ax.set_yticklabels([])

    # (c) pentagram
    ax = fig.add_subplot(gs[0, 2])
    plate(ax, f"(c) Pentagram\ncorner r = {STAR_R*MM:.0f} mm, {STAR_V*MM:.0f} mm/s, 1 loop")
    Rs = STAR_R * MM
    pts = [star_vertex(k, Rs) for k in range(6)]
    for k in range(5):
        p, q = np.array(pts[k]), np.array(pts[k + 1])
        ax.plot(*zip(p, q), color=AQUA, lw=1.8)
        m = p + 0.55 * (q - p)
        arrow(ax, tuple(p + 0.45 * (q - p)), tuple(m), AQUA)
    for k in range(5):
        x, y = pts[k]
        ax.plot(x, y, "o", color=AQUA, ms=6 if k else 7, mec="white", mew=1.5)
        ax.text(x * 1.22, y * 1.22, str(k + 1), ha="center", va="center", fontsize=9, color=INK)
    ax.set_yticklabels([])
    for a in fig.axes:
        a.set_xlabel("x [mm]", color=INK)

    # (d) timeline
    ax = fig.add_subplot(gs[1, :])
    segs, rows = [], []
    for line in open(log):
        if line.startswith("# SEG"):
            p = line.split()
            d = {"name": p[3]}
            d.update({k: float(v) for k, v in (kv.split("=") for kv in p[4:])})
            segs.append(d)
        elif line[:1].isdigit():
            rows.append([float(v) for v in line.split(",")[:4]])
    a = np.array(rows)
    t, spx, spy = a[:, 0] / 1000, a[:, 2] * MM, a[:, 3] * MM
    t0 = 0
    phases = [("settle", "hold", None), ("out_", "steps", BLUE), ("back_", "steps", BLUE),
              ("circle_in", "entry", None), ("circle", "circle", ORANGE), ("star_in", "entry", None),
              ("star", "pentagram", AQUA), ("end", "centre hold", None)]
    bounds = []
    for s in segs:
        s["t0"], t0 = t0, t0 + s["dur"]
        s["t1"] = t0
        col = next((c for pre, _, c in phases if s["name"].startswith(pre)), None)
        if s["name"] in ("circle", "star"):
            col = ORANGE if s["name"] == "circle" else AQUA
        elif s["name"].startswith(("out_", "back_")):
            col = BLUE
        else:
            col = None
        if col:
            ax.axvspan(s["t0"], s["t1"], color=col, alpha=0.10, lw=0)
        ax.axvline(s["t0"], color=GRID, lw=0.8)
    ax.plot(t, spx, color=INK, lw=1.5, label="set point x")
    ax.plot(t, spy, color=INK2, lw=1.5, ls="--", label="set point y")
    labels = [("settle", 1.0), ("steps", 13.0), ("circle", 30.0), ("pentagram", 41.8), ("centre hold", 49.5)]
    for txt, x in labels:
        ax.text(x, 84, txt, ha="center", fontsize=8, color=INK2)
    ax.set_xlim(0, t0); ax.set_ylim(-80, 95)
    ax.set_xlabel("Time [s]", color=INK); ax.set_ylabel("Set point [mm]", color=INK)
    ax.set_title(f"(d) Set point over the whole test ({t0:.1f} s per controller)", loc="left", fontsize=10, color=INK)
    ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.legend(frameon=False, fontsize=8, loc="lower left", ncol=2)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig_test_patterns.png")
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(out, t0)


if __name__ == "__main__":
    main(sys.argv[1])
