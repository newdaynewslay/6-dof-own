#!/usr/bin/env python3
"""Step-out figure for the report: normalised progress towards the edge point.

For every 'out_*' step of every run, progress = distance travelled along the step
direction / step size (0 = start, 1 = target). The figure shows the mean over all steps
and runs per controller, with the min-max range shaded, plus the 90 % line and each
controller's mean final value. It shows that the KF variants rise about as fast as
PID/LQR but stop short of the target.

Usage:  python3 step_figure.py LOGDIR [LOGDIR ...]     (one folder per test session)
Output: fig_step_out.png next to this script.
"""
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CTRLS = [("pid", "PID"), ("kf", "KF-PID"), ("lqr", "LQR"), ("kflqr", "KF-LQR")]
COLOR = {"PID": "#2a78d6", "KF-PID": "#eb6834", "LQR": "#1baf7a", "KF-LQR": "#c98500"}
LS = {"PID": "-", "KF-PID": "--", "LQR": "-", "KF-LQR": "--"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
SS_WINDOW = 1.5   # s, same as compare_controllers.py
TGRID = np.arange(0, 3.0, 0.016)


def steps_of(path):
    segs, rows = [], []
    for line in open(path, encoding="utf-8"):
        if line.startswith("# SEG"):
            p = line.split()
            d = {"name": p[3]}
            d.update({k: float(v) for k, v in (kv.split("=") for kv in p[4:])})
            segs.append(d)
        elif line[:1].isdigit():
            v = line.strip().split(",")
            if len(v) == 15:
                rows.append([float(x) for x in v])
    a = np.array(rows)
    t, seg, bx, by, found = a[:, 0] / 1000, a[:, 1], a[:, 4], a[:, 5], a[:, 13]
    out = []
    for i, s in enumerate(segs):
        if not s["name"].startswith("out_"):
            continue
        prev = np.array([segs[i - 1]["x"], segs[i - 1]["y"]])
        d = np.array([s["x"], s["y"]]) - prev
        A = np.hypot(*d)
        m = (seg == i) & (found == 1)
        tt = t[m] - t[m][0]
        prog = ((bx[m] - prev[0]) * d[0] + (by[m] - prev[1]) * d[1]) / A ** 2
        out.append(np.interp(TGRID, tt, prog, right=np.nan))
    return out


def main(dirs):
    P = {name: [] for _, name in CTRLS}
    nruns = {name: 0 for _, name in CTRLS}
    for d in dirs:
        for key, name in CTRLS:
            for f in glob.glob(os.path.join(d, f"*_{key}.csv")):
                if "FAILED" in f:
                    continue
                P[name] += steps_of(f)
                nruns[name] += 1

    fig, ax = plt.subplots(figsize=(8, 4.3))
    ax.axhline(100, color=INK2, lw=1, ls=(0, (1, 2)))
    ax.axhline(90, color=GRID, lw=1.2)
    ax.text(0.02, 90.5, "90 % of target", ha="left", va="bottom", fontsize=8, color=INK2)
    ax.text(0.02, 100.5, "target", ha="left", va="bottom", fontsize=8, color=INK2)
    ss = TGRID >= 3.0 - SS_WINDOW
    finals = {}
    for name in COLOR:
        arr = np.array(P[name]) * 100
        mean = np.nanmean(arr, axis=0)
        ax.fill_between(TGRID, np.nanmin(arr, axis=0), np.nanmax(arr, axis=0),
                        color=COLOR[name], alpha=0.10, lw=0)
        ax.plot(TGRID, mean, color=COLOR[name], ls=LS[name], lw=2, label=f"{name}")
        finals[name] = np.nanmean(mean[ss])
    # final-value labels on the right, spread out so they do not overlap
    ylab, last = {}, -1e9
    for name in sorted(finals, key=finals.get):
        last = max(finals[name], last + 6)
        ylab[name] = last
    for name, fin in finals.items():
        ax.annotate(f"{name} {fin:.0f} %", (3.0, fin), xytext=(3.06, ylab[name]),
                    textcoords="data", va="center", fontsize=8, color=COLOR[name],
                    annotation_clip=False)
    ax.set_xlim(0, 3.0)
    ax.set_ylim(-5, 115)
    ax.set_yticks([0, 25, 50, 75, 90, 100])
    ax.set_xlabel("Time after step [s]", color=INK)
    ax.set_ylabel("Progress towards edge point [%]", color=INK)
    n = len(P["PID"])
    ax.set_title(f"Step out from the centre: mean of {n} steps per controller (shaded: min–max)",
                 loc="left", fontsize=10, color=INK)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.legend(frameon=False, fontsize=9, loc="lower right", bbox_to_anchor=(0.97, 0.12))
    fig.tight_layout()
    outp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig_step_out.png")
    fig.savefig(outp, dpi=200)
    print(outp, {k: len(v) for k, v in P.items()}, nruns)


if __name__ == "__main__":
    main(sys.argv[1:])
