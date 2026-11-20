#!/usr/bin/env python3
"""Servo figures for a randomized 4-block session (e.g. logs/20261119_150440_full).

Wraps servo_figures.py: each block is treated as one "run" with all four controllers,
and each log is cut at the start of the disturbance part, so the servo numbers only
cover the tracking test (the kicks are added tilt, not controller action).

Usage: python3 servo_figures_blocks.py LOGDIR
Output: fig_servo_*.png and servo_metrics.md next to this script (overwrites the old ones).
"""
import glob, os, re, sys, tempfile
import numpy as np
import servo_figures as sf

sf.COLS = sf.COLS + ["d_x", "d_y"]
_load = sf.load


def load_cut(path):
    r = _load(path)
    td = next(s["t0"] for s in r.segs if s["name"].startswith("kick"))
    k = (r.t - r.t[0]) < td
    for a in ("t", "e", "sx", "sy", "upd"):
        setattr(r, a, getattr(r, a)[k])
    r.theta = r.theta[k]
    r.d = {c: v[k] for c, v in r.d.items()}
    return r


sf.load = load_cut

if __name__ == "__main__":
    logdir = os.path.abspath(sys.argv[1])
    tmp = tempfile.mkdtemp()
    blocks = {}
    for f in sorted(glob.glob(os.path.join(logdir, "*_n??_b?_*.csv"))):
        b = re.search(r"_b(\d)_", f).group(1)
        blocks.setdefault(b, []).append(f)
    dirs = []
    for b in sorted(blocks):
        d = os.path.join(tmp, f"block{b}")
        os.makedirs(d)
        for f in blocks[b]:
            os.symlink(f, os.path.join(d, os.path.basename(f)))
        dirs.append(d)
    sf.main(dirs, os.path.dirname(os.path.abspath(__file__)))
