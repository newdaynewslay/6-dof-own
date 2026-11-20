#!/usr/bin/env python3
"""Compact drift figure for the report: how the results changed during one test session.

(a) Total IAE (reference part, disturbance excluded) against run number.
(b) Final position of the steps towards -x and -y (% of step) against run number.
Points = one run, lines = least-squares trend per controller, grey lines = new block.

Usage:  python3 drift_figure.py LOGDIR
Output: fig_drift_compact.png next to this script.
"""
import glob, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MM = 1.045
KEY = {"pid": "PID", "kf": "KF-PID", "lqr": "LQR", "kflqr": "KF-LQR"}
COLOR = {"PID": "#2a78d6", "KF-PID": "#eb6834", "LQR": "#1baf7a", "KF-LQR": "#c98500"}
MARK = {"PID": "o", "KF-PID": "s", "LQR": "^", "KF-LQR": "D"}
LS = {"PID": "-", "KF-PID": "--", "LQR": "-", "KF-LQR": "--"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def read(path):
    segs, rows = [], []
    for l in open(path, encoding="utf-8"):
        if l.startswith("# SEG"):
            p = l.split()
            d = {"name": p[3]}
            d.update({k: float(v) for k, v in (kv.split("=") for kv in p[4:])})
            segs.append(d)
        elif l[:1].isdigit():
            rows.append([float(x) for x in l.split(",")[:14]])
    return segs, np.array(rows)


def run_metrics(path):
    segs, a = read(path)
    t, seg, found = a[:, 0] / 1000, a[:, 1], a[:, 13] == 1
    t = t - t[0]
    names = [s["name"] for s in segs]
    # IAE up to the start of the disturbance part
    t0 = 0
    for s in segs:
        s["t0"], t0 = t0, t0 + s["dur"]
    td = next(s["t0"] for s in segs if s["name"].startswith("kick"))
    e = np.hypot(a[:, 4] - a[:, 2], a[:, 5] - a[:, 3]) * MM
    k = (t < td) & found
    iae = float(np.sum(e[1:][k[1:]] * np.diff(t)[k[1:]]))
    # final position of the -x / -y steps (last 1.5 s of the hold)
    fin = []
    for nm in ("out_180", "out_270"):
        i = names.index(nm); s = segs[i]
        m = (seg == i) & found
        kk = t[m] >= t[m][-1] - 1.5
        px, py = a[m, 4][kk].mean(), a[m, 5][kk].mean()
        fin.append(100 * (px * s["x"] + py * s["y"]) / (s["x"] ** 2 + s["y"] ** 2))
    return iae, float(np.mean(fin))


def main(d):
    data = {c: [] for c in COLOR}
    for f in sorted(glob.glob(os.path.join(d, "*_n??_b?_*.csv"))):
        if "FAILED" in f:
            continue
        m = re.search(r"_n(\d+)_b(\d)_(\w+)\.csv$", f)
        n, ctrl = int(m.group(1)), KEY[m.group(3)]
        data[ctrl].append((n, *run_metrics(f)))
    nmax = max(r[0] for v in data.values() for r in v)

    fig, axs = plt.subplots(1, 2, figsize=(9, 3.4))
    panels = [(1, "Total IAE [mm·s]", "(a) Tracking error grows during the session"),
              (2, "Final position [% of step]", "(b) Steps towards −x/−y stop shorter")]
    for ax, (j, ylab, title) in zip(axs, panels):
        for b in range(1, 4):
            ax.axvline(4 * b + 0.5, color=GRID, lw=1, zorder=0)
        for c in COLOR:
            r = np.array(sorted(data[c]))
            x, y = r[:, 0], r[:, j]
            ax.scatter(x, y, s=28, marker=MARK[c], color=COLOR[c], edgecolor="white", lw=0.8, zorder=3)
            p = np.polyfit(x, y, 1)
            xx = np.array([1, nmax])
            ax.plot(xx, np.polyval(p, xx), color=COLOR[c], ls=LS[c], lw=1.8, label=c, zorder=2)
        ax.set_xlim(0.5, nmax + 0.5)
        ax.set_xticks(range(1, nmax + 1, 2 if nmax > 10 else 1))
        ax.set_xlabel("Run number (order of execution)", color=INK)
        ax.set_ylabel(ylab, color=INK)
        ax.set_title(title, loc="left", fontsize=10, color=INK)
        ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.tick_params(colors=INK2, labelsize=8)
    axs[1].axhline(100, color=INK2, lw=0.8, ls=":")
    axs[1].set_ylim(0, 110)
    for b in range(4):
        axs[0].text(4 * b + 2.5, axs[0].get_ylim()[1], f"block {b+1}", ha="center", va="top", fontsize=7, color=INK2)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=4, frameon=False, fontsize=8, bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig_drift_compact.png")
    fig.savefig(out, dpi=200)
    for c in COLOR:
        r = np.array(sorted(data[c]))
        print(c, "IAE slope/run %.1f" % np.polyfit(r[:, 0], r[:, 1], 1)[0],
              "final%% slope/run %.2f" % np.polyfit(r[:, 0], r[:, 2], 1)[0])
    print(out)


if __name__ == "__main__":
    main(sys.argv[1])
