#!/usr/bin/env python3
"""Ball path in the plate plane for the steps and the pentagram (circles are in fig_circles.png).

Columns: PID, KF-PID, LQR, KF-LQR. Rows: steps, pentagram. All runs of a controller are drawn
as thin lines, the set point in black. fig_xy.png is left untouched.

Usage: python3 xy_steps_star.py LOGDIR
Output: fig_xy_steps_star.png next to this script.
"""
import glob, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from circle_figure import read_log, CTRL, MM, INK, INK2, GRID

STAR_R = 60 * MM


def star_pts(R):
    order = [0, 2, 4, 1, 3, 0]
    a = np.radians(90 + 72 * np.array(order))
    return R * np.cos(a), R * np.sin(a)


def main(logdir):
    fig, axs = plt.subplots(2, 4, figsize=(11, 6.0))
    for j, (key, label, col) in enumerate(CTRL):
        files = sorted(glob.glob(os.path.join(logdir, f"*_{key}.csv")))
        for f in files:
            segs, a = read_log(f)
            ok = a[:, 13] == 1
            step_ids = [i for n, (i, d) in segs.items() if n.startswith(("out_", "back_"))]
            for i in sorted(step_ids):
                s = a[(a[:, 1] == i) & ok]
                axs[0, j].plot(s[:, 4] * MM, s[:, 5] * MM, color=col, lw=0.7, alpha=0.8)
            s = a[(a[:, 1] == segs["star"][0]) & ok]
            axs[1, j].plot(s[:, 4] * MM, s[:, 5] * MM, color=col, lw=0.7, alpha=0.8)
        R = 50 * MM
        axs[0, j].plot([0, R, 0, -R, 0], [0, 0, R, 0, -R], "o", ms=7, mfc="white", mec=INK, mew=1.2, zorder=5)
        axs[1, j].plot(*star_pts(STAR_R), "--", color=INK, lw=1.2, zorder=5)
        axs[0, j].set_title(label, loc="left", fontsize=11, color=INK)
        for i in range(2):
            ax = axs[i, j]
            ax.set_xlim(-75, 75); ax.set_ylim(-75, 75); ax.set_aspect("equal")
            ax.set_xticks([-50, 0, 50]); ax.set_yticks([-50, 0, 50])
            ax.grid(color=GRID, lw=0.6); ax.tick_params(labelsize=8, colors=INK2)
            for s_ in ("top", "right"):
                ax.spines[s_].set_visible(False)
            if j == 0:
                ax.set_ylabel(("(a) steps" if i == 0 else "(b) pentagram") + "\ny [mm]")
            if i == 1:
                ax.set_xlabel("x [mm]")
    fig.subplots_adjust(left=0.08, right=0.99, top=0.94, bottom=0.09, wspace=0.18, hspace=0.18)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig_xy_steps_star.png")
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == "__main__":
    main(sys.argv[1])
