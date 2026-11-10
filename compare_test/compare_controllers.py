#!/usr/bin/env python3
"""
PID vs LQR comparison for the 6-DOF ball balancing robot.

Works with compare_test/Ball_Balance_Compare/Ball_Balance_Compare.ino, which runs
the same scripted set point sequence for both controllers (centre, steps to the
edge points and back, fast circle, pentagram) and prints one CSV row per control
step.

Usage
-----
1) Record (close the Arduino Serial Monitor / pio monitor first):
       python3 compare_controllers.py run /dev/cu.usbmodem12345
   Servos are switched on only for each run and off again after it.
   Runs PID, then LQR, and saves logs/<time>_pid.csv and logs/<time>_lqr.csv.
   Options: --ctrl pid|lqr|both  --repeat 3  --dirs 4  --out logs  --tag kp009
   Find the port with:  ls /dev/cu.usbmodem*

2) Analyse / compare (any number of logs, grouped by controller):
       python3 compare_controllers.py analyze logs/*.csv
   Writes metrics.csv, metrics.md and fig_*.png to a new folder report_<time>
   (or --out <folder>; an existing folder is overwritten).
   --mm-per-px 1.09 sets the pixel scale used for mm values (default 1.09,
   measure it: ball moved a known distance / pixel change).

Requires: pip install pyserial numpy matplotlib
"""
import argparse
import csv
import glob
import math
import os
import re
import sys
import time
from datetime import datetime

COLS = ["t_ms", "seg", "sp_x", "sp_y", "ball_x", "ball_y", "err_x", "err_y",
        "derr_x", "derr_y", "u_x", "u_y", "sat", "found", "ik"]
CTRL_ID = {"pid": 0, "lqr": 1}

# colours: PID, LQR (fixed per controller), reference
COLORS = {"PID": "#2a78d6", "LQR": "#eb6834"}
REF = "#555555"

STEP_MIN = 5.0     # px, smaller set point jumps are not counted as steps
BAND_REL = 0.05    # settling band: 5 % of the step ...
BAND_MIN = 3.0     # ... but at least this many px (Pixy noise)
SS_WINDOW = 1.5    # s, steady-state error = mean |e| over the last part of a hold


# ============================================================================ run
def read_line(ser):
    return ser.readline().decode(errors="ignore").strip()


def send(ser, text):
    ser.write((text + "\n").encode())
    ser.flush()


def record_one(ser, ctrl, dirs, path):
    """Runs one test on the board and saves all lines to path. Returns True if it ended normally."""
    send(ser, f"m {CTRL_ID[ctrl]}")
    time.sleep(0.3)
    input(f"\n[{ctrl.upper()}] Press Enter to switch the servos ON ... ")
    send(ser, "e")
    time.sleep(0.3)
    ser.reset_input_buffer()
    input(f"[{ctrl.upper()}] Put the ball in the centre (within 20 s), let it settle, then press Enter ... ")
    send(ser, f"r {dirs}")
    n = 0
    status = None
    t_last = time.time()
    with open(path, "w") as f:
        while True:
            try:
                line = read_line(ser)
            except Exception as ex:  # USB unplugged / Teensy reset
                print(f"  serial connection lost: {ex}")
                status = "DISCONNECTED"
                break
            if not line:
                if time.time() - t_last > 5:
                    print("  no data for 5 s - is the board running?")
                    status = "TIMEOUT"
                    break
                continue
            t_last = time.time()
            f.write(line + "\n")
            if line.startswith("#"):
                if line.startswith("# ERROR") or line.startswith("# ABORT") or line.startswith("# BUSY"):
                    print("  [robot]", line[2:])
                    status = line[2:]
                    break
                if line.startswith("# END"):
                    status = "END"
                    break
                if not line.startswith("# SEG"):
                    print("  [robot]", line[2:])
            elif line[0].isdigit():
                n += 1
                if n % 500 == 0:
                    seg = line.split(",")[1]
                    print(f"  {n} samples (segment {seg})")
    print(f"  {status}: {n} samples -> {path}")
    return status == "END"


def cmd_run(args):
    import serial  # pyserial

    os.makedirs(args.out, exist_ok=True)
    ctrls = ["pid", "lqr"] if args.ctrl == "both" else [args.ctrl]
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(1.0)
    ser.reset_input_buffer()
    send(ser, "?")
    time.sleep(0.3)
    while ser.in_waiting:
        print("  [robot]", read_line(ser))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"_{args.tag}" if args.tag else ""
    saved = []
    try:
        for rep in range(args.repeat):
            for ctrl in ctrls:
                while True:
                    rp = f"_r{rep + 1}" if args.repeat > 1 else ""
                    path = os.path.join(args.out, f"{stamp}{tag}_{ctrl}{rp}.csv")
                    if record_one(ser, ctrl, args.dirs, path):
                        saved.append(path)
                        break
                    if not ser.is_open or not os.path.exists(args.port):
                        print("  The board is gone from USB - stopping. Partial log kept: " + path)
                        raise SystemExit(1)
                    if input("  Run failed. Retry? [Y/n] ").strip().lower() == "n":
                        os.rename(path, path.replace(".csv", "_FAILED.csv"))
                        break
    except KeyboardInterrupt:
        send(ser, "x")
        time.sleep(0.1)
        send(ser, "o")
        print("\nAborted.")
    ser.close()
    if saved:
        print("\nAnalyse with:\n  python3 compare_controllers.py analyze " + " ".join(saved))


# ============================================================================ parse
class Run:
    pass


def load_run(path):
    r = Run()
    r.path = path
    r.name = os.path.splitext(os.path.basename(path))[0]
    r.ctrl = None
    r.settings = {}
    r.segs = []      # dicts: i, name, kind, x, y, R, p, dur
    r.status = "INCOMPLETE"
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if line.startswith("# CTRL"):
                    r.ctrl = line.split()[2]
                elif line.startswith("# SEG"):
                    p = line.split()
                    d = {"i": int(p[2]), "name": p[3]}
                    for kv in p[4:]:
                        k, v = kv.split("=")
                        d[k] = float(v)
                    d["kind"] = int(d["kind"])
                    r.segs.append(d)
                elif line.startswith("# END"):
                    r.status = "END"
                elif line.startswith("# ABORT"):
                    r.status = line[2:]
                elif line.startswith(("# PID", "# LQR", "# COMMON")):
                    for kv in re.findall(r"(\w+)=([-\d.]+)", line):
                        r.settings[line.split()[1] + "_" + kv[0]] = float(kv[1])
                continue
            if not line[0].isdigit():
                continue
            parts = line.split(",")
            if len(parts) != len(COLS):
                continue
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                pass
    import numpy as np
    a = np.array(rows) if rows else np.zeros((0, len(COLS)))
    r.d = {c: a[:, i] for i, c in enumerate(COLS)}
    r.t = r.d["t_ms"] / 1000.0
    ok = r.d["found"] == 1
    r.ok = ok
    # ball position, NaN where the ball was not seen
    r.bx = np.where(ok, r.d["ball_x"], np.nan)
    r.by = np.where(ok, r.d["ball_y"], np.nan)
    r.ex = r.bx - r.d["sp_x"]
    r.ey = r.by - r.d["sp_y"]
    r.e = np.hypot(r.ex, r.ey)
    r.u = np.where(ok, np.hypot(r.d["u_x"], r.d["u_y"]), np.nan)
    # segment start times (from the durations printed by the sketch)
    t0 = 0.0
    for s in r.segs:
        s["t0"] = t0
        t0 += s["dur"]
        s["t1"] = t0
    if r.ctrl is None:
        r.ctrl = "LQR" if "lqr" in r.name.lower() else "PID"
    return r


# ============================================================================ metrics
def seg_mask(r, s):
    return r.d["seg"] == s["i"]


def trapz(yv, xv):
    import numpy as np
    m = ~np.isnan(yv)
    if m.sum() < 2:
        return float("nan")
    return float(np.trapz(yv[m], xv[m]))


def step_metrics(r, s, prev_sp, k):
    """Metrics of one step segment. prev_sp = set point before the step."""
    import numpy as np
    m = seg_mask(r, s)
    t = r.t[m] - s["t0"]
    bx, by = r.bx[m], r.by[m]
    sp = np.array([s["x"], s["y"]])
    d = sp - prev_sp
    A = float(np.hypot(*d))
    dn = d / A
    rel_x, rel_y = bx - prev_sp[0], by - prev_sp[1]
    prog = (rel_x * dn[0] + rel_y * dn[1]) / A         # 0 -> 1
    cross = rel_x * (-dn[1]) + rel_y * dn[0]          # px off the straight line
    e = np.hypot(bx - sp[0], by - sp[1])
    u = r.u[m]

    def first_cross(level):
        idx = np.where(prog >= level)[0]
        return t[idx[0]] if len(idx) else float("nan")

    band = max(BAND_REL * A, BAND_MIN)
    out = np.where((e > band) & ~np.isnan(e))[0]
    if len(t) == 0:
        settle = float("nan")
    elif len(out) == 0:
        settle = 0.0
    elif out[-1] == len(t) - 1:
        settle = float("nan")                          # never settled in this hold
    else:
        settle = float(t[out[-1] + 1])
    ss = t >= (s["dur"] - SS_WINDOW)
    return {
        "run": r.name, "ctrl": r.ctrl, "seg": s["name"], "angle": k,
        "amp_px": A,
        "rise_s": first_cross(0.9) - first_cross(0.1),
        "delay_s": first_cross(0.5),
        "overshoot_pct": max(0.0, float(np.nanmax(prog)) - 1.0) * 100 if len(prog) else float("nan"),
        "settle_s": settle,
        "ss_err_px": float(np.nanmean(e[ss])) if ss.any() else float("nan"),
        "cross_max_px": float(np.nanmax(np.abs(cross))) if len(cross) else float("nan"),
        "iae_pxs": trapz(e, t),
        "u_peak_deg": float(np.nanmax(u)) if len(u) else float("nan"),
        "t": t, "prog": prog, "bx": bx, "by": by,
    }


def track_metrics(r, s):
    """Metrics of a circle / star segment."""
    import numpy as np
    m = seg_mask(r, s)
    t = r.t[m] - s["t0"]
    e = r.e[m]
    u = r.u[m]
    dt = np.diff(t)
    du = np.abs(np.diff(np.hypot(r.d["u_x"][m], r.d["u_y"][m])))
    res = {
        "run": r.name, "ctrl": r.ctrl, "seg": s["name"],
        "rms_err_px": float(np.sqrt(np.nanmean(e ** 2))),
        "max_err_px": float(np.nanmax(e)),
        "iae_pxs": trapz(e, t),
        "u_rms_deg": float(np.sqrt(np.nanmean(u ** 2))),
        "u_peak_deg": float(np.nanmax(u)),
        "u_rate_degps": float(np.nansum(du) / max(t[-1] - t[0], 1e-6)) if len(t) > 1 else float("nan"),
        "sat_pct": float(np.mean(r.d["sat"][m]) * 100),
        "lost": int(np.sum(r.d["found"][m] != 1)),
    }
    if s["kind"] == 1:  # circle: radius error and phase lag
        bx, by = r.bx[m], r.by[m]
        spx, spy = r.d["sp_x"][m], r.d["sp_y"][m]
        res["radius_err_px"] = float(np.nanmean(np.hypot(bx, by)) - s["R"])
        lag = np.angle(np.exp(1j * (np.arctan2(spy, spx) - np.arctan2(by, bx))))
        lag = np.degrees(lag) * (1 if s["p"] > 0 else -1)
        res["phase_lag_deg"] = float(np.nanmean(lag))
        res["lag_s"] = res["phase_lag_deg"] / 360.0 * abs(s["p"])
    return res


def hold_metrics(r, s, window=3.0):
    """Regulation at the centre: noise / limit cycle over the last part of a hold."""
    import numpy as np
    m = seg_mask(r, s) & (r.t >= s["t1"] - window)
    e = r.e[m]
    return {"run": r.name, "ctrl": r.ctrl, "seg": s["name"],
            "rms_err_px": float(np.sqrt(np.nanmean(e ** 2))) if m.any() else float("nan"),
            "max_err_px": float(np.nanmax(e)) if m.any() else float("nan"),
            "u_rms_deg": float(np.sqrt(np.nanmean(r.u[m] ** 2))) if m.any() else float("nan")}


def analyse_run(r):
    import numpy as np
    steps, tracks, holds = [], [], []
    prev = np.array([0.0, 0.0])
    for s in r.segs:
        sp = np.array([s["x"], s["y"]]) if s["kind"] == 0 else None
        if s["kind"] == 0:
            if s["name"] in ("settle", "end"):
                holds.append(hold_metrics(r, s))
            if s["name"] != "settle" and np.hypot(*(sp - prev)) > STEP_MIN and seg_mask(r, s).sum() > 5:
                m = re.search(r"_(\d+)$", s["name"])
                k = int(m.group(1)) if m else -1
                steps.append(step_metrics(r, s, prev, k))
            prev = sp
        else:
            if seg_mask(r, s).sum() > 5:
                tracks.append(track_metrics(r, s))
            # set point at the end of the pattern (full turns / loops -> back at the start)
            i = np.where(seg_mask(r, s))[0]
            if len(i):
                prev = np.array([r.d["sp_x"][i[-1]], r.d["sp_y"][i[-1]]])
    tot = {"run": r.name, "ctrl": r.ctrl, "status": r.status,
           "duration_s": float(r.t[-1]) if len(r.t) else 0.0,
           "samples": len(r.t),
           "rate_hz": float(len(r.t) / r.t[-1]) if len(r.t) and r.t[-1] > 0 else float("nan"),
           "lost": int(np.sum(r.d["found"] != 1)),
           "ik_fail": int(np.sum(r.d["ik"] == 0)),
           "sat_pct": float(np.mean(r.d["sat"]) * 100) if len(r.t) else float("nan"),
           "iae_total_pxs": trapz(r.e, r.t),
           "u_rms_deg": float(np.sqrt(np.nanmean(r.u ** 2))) if len(r.t) else float("nan")}
    return steps, tracks, holds, tot


# ============================================================================ report
def mean_std(vals):
    import numpy as np
    v = np.array([x for x in vals if x is not None and not (isinstance(x, float) and math.isnan(x))])
    if len(v) == 0:
        return float("nan"), float("nan"), 0
    return float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0, len(v)


def build_table(all_steps, all_tracks, all_holds, all_tot, ctrls, mm):
    """Rows: (group, metric, unit, scale, list of values per controller)."""
    rows = []

    def add(group, label, unit, items, key, scale=1.0, filt=None):
        vals = {}
        for c in ctrls:
            v = [it[key] * scale for it in items if it["ctrl"] == c and key in it and (filt is None or filt(it))]
            vals[c] = v
        rows.append((group, label, unit, vals))

    for direction, f in (("Step out (centre -> edge)", lambda it: it["seg"].startswith("out_")),
                         ("Step back (edge -> centre)", lambda it: it["seg"].startswith("back_"))):
        add(direction, "Rise time 10-90 %", "s", all_steps, "rise_s", filt=f)
        add(direction, "Overshoot", "%", all_steps, "overshoot_pct", filt=f)
        add(direction, f"Settling time ({BAND_REL * 100:.0f} % / {BAND_MIN:.0f} px band)", "s", all_steps, "settle_s", filt=f)
        add(direction, "Steady-state error", "mm", all_steps, "ss_err_px", mm, filt=f)
        add(direction, "Max deviation from straight path", "mm", all_steps, "cross_max_px", mm, filt=f)
        add(direction, "IAE", "mm*s", all_steps, "iae_pxs", mm, filt=f)
        add(direction, "Peak tilt", "deg", all_steps, "u_peak_deg", filt=f)
    for seg, title in (("circle", "Fast circle"), ("star", "Pentagram")):
        f = (lambda sg: (lambda it: it["seg"] == sg))(seg)
        add(title, "RMS tracking error", "mm", all_tracks, "rms_err_px", mm, filt=f)
        add(title, "Max tracking error", "mm", all_tracks, "max_err_px", mm, filt=f)
        add(title, "IAE", "mm*s", all_tracks, "iae_pxs", mm, filt=f)
        if seg == "circle":
            add(title, "Mean radius error (ball - ref)", "mm", all_tracks, "radius_err_px", mm, filt=f)
            add(title, "Phase lag", "deg", all_tracks, "phase_lag_deg", filt=f)
            add(title, "Time lag", "s", all_tracks, "lag_s", filt=f)
        add(title, "RMS tilt", "deg", all_tracks, "u_rms_deg", filt=f)
        add(title, "Tilt rate (servo activity)", "deg/s", all_tracks, "u_rate_degps", filt=f)
        add(title, "Time at tilt limit", "%", all_tracks, "sat_pct", filt=f)
    add("Centre hold (last 3 s of 'end')", "RMS error", "mm", all_holds, "rms_err_px", mm, filt=lambda it: it["seg"] == "end")
    add("Centre hold (last 3 s of 'end')", "RMS tilt", "deg", all_holds, "u_rms_deg", filt=lambda it: it["seg"] == "end")
    add("Whole test", "Total IAE", "mm*s", all_tot, "iae_total_pxs", mm)
    add("Whole test", "RMS tilt", "deg", all_tot, "u_rms_deg")
    add("Whole test", "Time at tilt limit", "%", all_tot, "sat_pct")
    add("Whole test", "Control rate", "Hz", all_tot, "rate_hz")
    add("Whole test", "Ball-lost samples", "-", all_tot, "lost")
    return rows


def write_tables(rows, ctrls, outdir, runs, mm):
    with open(os.path.join(outdir, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        hdr = ["group", "metric", "unit"]
        for c in ctrls:
            hdr += [f"{c}_mean", f"{c}_std", f"{c}_n"]
        w.writerow(hdr)
        for g, lab, unit, vals in rows:
            line = [g, lab, unit]
            for c in ctrls:
                mu, sd, n = mean_std(vals[c])
                line += [f"{mu:.4g}", f"{sd:.3g}", n]
            w.writerow(line)

    with open(os.path.join(outdir, "metrics.md"), "w") as f:
        f.write("# PID vs LQR - scripted test metrics\n\n")
        f.write(f"Scale: {mm} mm/px. Values are mean ± std over all steps / runs (n in brackets).\n\n")
        f.write("Runs: " + ", ".join(f"`{r.name}` ({r.ctrl}, {r.status})" for r in runs) + "\n\n")
        group = None
        for g, lab, unit, vals in rows:
            if g != group:
                group = g
                f.write(f"\n### {g}\n\n| Metric | Unit | " + " | ".join(ctrls) + " |\n")
                f.write("|---|---|" + "---|" * len(ctrls) + "\n")
            cells = []
            for c in ctrls:
                mu, sd, n = mean_std(vals[c])
                cells.append("-" if n == 0 else (f"{mu:.3g} ± {sd:.2g} ({n})" if n > 1 else f"{mu:.3g}"))
            f.write(f"| {lab} | {unit} | " + " | ".join(cells) + " |\n")


# ============================================================================ plots
def style(ax):
    ax.grid(True, color="#e6e6e6", linewidth=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color("#999999")
    ax.tick_params(colors="#555555")


def shade_segments(ax, segs):
    for s in segs:
        if s["kind"] != 0:
            ax.axvspan(s["t0"], s["t1"], color="#f1f1f1", zorder=0)
            ax.text((s["t0"] + s["t1"]) / 2, 1.0, s["name"], transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=8, color="#555555")


def plot_overview(runs, mm, outdir, plt):
    fig, axs = plt.subplots(3, 1, figsize=(13, 8.5), sharex=True)
    first = runs[0]
    for i, (key, lab) in enumerate((("x", "x [mm]"), ("y", "y [mm]"))):
        ax = axs[i]
        ax.plot(first.t, first.d["sp_" + key] * mm, color=REF, lw=1.2, ls="--", label="set point")
        for r in runs:
            ax.plot(r.t, (r.bx if key == "x" else r.by) * mm, color=COLORS.get(r.ctrl), lw=1.2, alpha=0.9, label=r.ctrl)
        ax.set_ylabel(lab)
    for r in runs:
        axs[2].plot(r.t, r.e * mm, color=COLORS.get(r.ctrl), lw=1.0, alpha=0.9, label=r.ctrl)
    axs[2].set_ylabel("|error| [mm]")
    axs[2].set_xlabel("time [s]")
    for ax in axs:
        style(ax)
        shade_segments(ax, first.segs)
    dedup_legend(axs[0], loc="upper right")
    fig.suptitle("Scripted test - ball position vs set point", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_overview.png"), dpi=150)


def dedup_legend(ax, **kw):
    h, l = ax.get_legend_handles_labels()
    seen = {}
    for hh, ll in zip(h, l):
        seen.setdefault(ll, hh)
    ax.legend(seen.values(), seen.keys(), frameon=False, **kw)


def plot_steps(all_steps, ctrls, outdir, plt):
    import numpy as np
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
    tg = np.linspace(0, 5, 251)
    for ax, kind, title in ((axs[0], "out_", "Centre -> edge"), (axs[1], "back_", "Edge -> centre")):
        for c in ctrls:
            curves = []
            for st in all_steps:
                if st["ctrl"] != c or not st["seg"].startswith(kind):
                    continue
                ax.plot(st["t"], st["prog"], color=COLORS.get(c), lw=0.6, alpha=0.25)
                m = ~np.isnan(st["prog"])
                if m.sum() > 2:
                    curves.append(np.interp(tg, st["t"][m], st["prog"][m], right=np.nan))
            if curves:
                ax.plot(tg, np.nanmean(curves, axis=0), color=COLORS.get(c), lw=2.2, label=f"{c} (mean of {len(curves)})")
        ax.axhline(1, color=REF, lw=1, ls="--")
        ax.axhspan(1 - BAND_REL, 1 + BAND_REL, color="#eeeeee", zorder=0)
        ax.set_title(title, loc="left", fontsize=11)
        ax.set_xlabel("time since step [s]")
        style(ax)
        ax.legend(frameon=False, loc="lower right")
    axs[0].set_ylabel("normalised progress (0 = start, 1 = target)")
    fig.suptitle("Step responses, all directions (thin = single step, thick = mean)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_steps.png"), dpi=150)


def plot_step_directions(all_steps, ctrls, outdir, plt):
    import numpy as np
    out = [s for s in all_steps if s["seg"].startswith("out_")]
    angles = sorted({s["angle"] for s in out})
    if not angles:
        return
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.8))
    wbar = 0.8 / len(ctrls)
    for ax, key, lab in ((axs[0], "overshoot_pct", "overshoot [%]"),
                         (axs[1], "settle_s", "settling time [s]"),
                         (axs[2], "ss_err_px", "steady-state error [px]")):
        for j, c in enumerate(ctrls):
            means = [mean_std([s[key] for s in out if s["ctrl"] == c and s["angle"] == a])[0] for a in angles]
            ax.bar(np.arange(len(angles)) + (j - (len(ctrls) - 1) / 2) * wbar, means, wbar * 0.92,
                   color=COLORS.get(c), label=c)
        ax.set_xticks(np.arange(len(angles)))
        ax.set_xticklabels([f"{a}°" for a in angles])
        ax.set_xlabel("step direction (centre -> edge)")
        ax.set_title(lab, loc="left", fontsize=11)
        style(ax)
    axs[0].legend(frameon=False)
    fig.suptitle("Step metrics per direction (shows platform asymmetry / levelling offsets)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_step_directions.png"), dpi=150)


def plot_xy(runs, ctrls, mm, outdir, plt):
    import numpy as np
    names = ["steps", "circle", "star"]
    fig, axs = plt.subplots(len(ctrls), 3, figsize=(12, 4.1 * len(ctrls)), squeeze=False)
    R = runs[0].settings.get("COMMON_sp_max", 90)
    for i, c in enumerate(ctrls):
        rs = [r for r in runs if r.ctrl == c]
        for j, nm in enumerate(names):
            ax = axs[i][j]
            for r in rs:
                for s in r.segs:
                    m = seg_mask(r, s)
                    if nm == "steps":
                        if not (s["name"].startswith("out_") or s["name"].startswith("back_")):
                            continue
                    elif s["name"] != nm:
                        continue
                    ax.plot(r.d["sp_x"][m] * mm, r.d["sp_y"][m] * mm, color=REF, ls="--", lw=1.2)
                    ax.plot(r.bx[m] * mm, r.by[m] * mm, color=COLORS.get(c), lw=0.9, alpha=0.8)
                    if nm == "steps":
                        ax.plot(s["x"] * mm, s["y"] * mm, "o", ms=8, mfc="white", mec=REF)
            th = np.linspace(0, 2 * np.pi, 200)
            ax.plot(R * mm * np.cos(th), R * mm * np.sin(th), color="#cccccc", lw=0.8)
            ax.set_aspect("equal")
            ax.set_title(f"{c} - {nm}", loc="left", fontsize=11)
            ax.set_xlabel("x [mm]")
            ax.set_ylabel("y [mm]")
            style(ax)
    fig.suptitle("Ball path (colour) vs set point (dashed); grey circle = set point limit", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_xy.png"), dpi=150)


def plot_tracking_error(runs, mm, outdir, plt):
    fig, axs = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    for ax, nm in zip(axs, ("circle", "star")):
        for r in runs:
            for s in r.segs:
                if s["name"] == nm:
                    m = seg_mask(r, s)
                    ax.plot(r.t[m] - s["t0"], r.e[m] * mm, color=COLORS.get(r.ctrl), lw=1.1, label=r.ctrl)
        ax.set_title(f"{nm}: tracking error", loc="left", fontsize=11)
        ax.set_xlabel("time in segment [s]")
        style(ax)
        dedup_legend(ax, loc="upper right")
    axs[0].set_ylabel("|error| [mm]")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_tracking_error.png"), dpi=150)


def plot_tilt(runs, outdir, plt):
    fig, ax = plt.subplots(figsize=(13, 3.6))
    for r in runs:
        ax.plot(r.t, r.u, color=COLORS.get(r.ctrl), lw=0.9, alpha=0.9, label=r.ctrl)
    rmax = runs[0].settings.get("COMMON_r_max")
    if rmax:
        ax.axhline(rmax, color=REF, ls="--", lw=1)
        ax.text(0, rmax, " tilt limit", va="bottom", fontsize=8, color="#555555")
    shade_segments(ax, runs[0].segs)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|tilt command| [deg]")
    style(ax)
    dedup_legend(ax, loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_tilt.png"), dpi=150)


def plot_bars(rows, ctrls, outdir, plt):
    import numpy as np
    pick = [("Step out (centre -> edge)", "Rise time 10-90 %"),
            ("Step out (centre -> edge)", "Overshoot"),
            ("Step out (centre -> edge)", f"Settling time ({BAND_REL * 100:.0f} % / {BAND_MIN:.0f} px band)"),
            ("Step out (centre -> edge)", "Steady-state error"),
            ("Fast circle", "RMS tracking error"),
            ("Fast circle", "Phase lag"),
            ("Pentagram", "RMS tracking error"),
            ("Pentagram", "Max tracking error"),
            ("Whole test", "RMS tilt")]
    sel = [r for r in rows if (r[0], r[1]) in pick]
    n = len(sel)
    cols = 3
    fig, axs = plt.subplots(math.ceil(n / cols), cols, figsize=(12, 3.0 * math.ceil(n / cols)), squeeze=False)
    for k, (g, lab, unit, vals) in enumerate(sel):
        ax = axs[k // cols][k % cols]
        for j, c in enumerate(ctrls):
            mu, sd, cnt = mean_std(vals[c])
            ax.bar(j, mu, 0.6, yerr=sd if cnt > 1 else None, color=COLORS.get(c), capsize=4,
                   error_kw={"ecolor": "#555555", "lw": 1})
            if not math.isnan(mu):
                ax.text(j, mu, f"{mu:.3g}", ha="center", va="bottom", fontsize=9, color="#333333")
        ax.set_xticks(range(len(ctrls)))
        ax.set_xticklabels(ctrls)
        short = g.split(" (")[0]
        ax.set_title(f"{short}: {lab} [{unit}]", loc="left", fontsize=9.5)
        style(ax)
    for k in range(n, axs.size):
        axs[k // cols][k % cols].axis("off")
    fig.suptitle("Key metrics (bar = mean, whisker = std over steps / runs)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_metrics.png"), dpi=150)


def cmd_analyze(args):
    import warnings
    warnings.filterwarnings("ignore", category=RuntimeWarning)  # NaN means of empty / lost-ball slices
    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    files = []
    for p in args.files:
        files += sorted(glob.glob(p))
    runs = [load_run(p) for p in files]
    runs = [r for r in runs if len(r.t) > 0]
    runs.sort(key=lambda r: (r.ctrl != "PID", r.name))
    if not runs:
        sys.exit("no data in the given files")
    ctrls = [c for c in ("PID", "LQR") if any(r.ctrl == c for r in runs)]
    if args.out is None:
        args.out = "report_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(args.out, exist_ok=True)
    mm = args.mm_per_px

    all_steps, all_tracks, all_holds, all_tot = [], [], [], []
    for r in runs:
        st, tr, ho, tot = analyse_run(r)
        all_steps += st
        all_tracks += tr
        all_holds += ho
        all_tot.append(tot)
        print(f"{r.name}: {r.ctrl}, {tot['status']}, {tot['duration_s']:.1f} s, "
              f"{tot['rate_hz']:.0f} Hz, lost {tot['lost']}, IK fail {tot['ik_fail']}")
        if r.status != "END":
            print("   WARNING: run did not finish, metrics of the missing segments are left out")

    rows = build_table(all_steps, all_tracks, all_holds, all_tot, ctrls, mm)
    write_tables(rows, ctrls, args.out, runs, mm)

    plot_overview(runs, mm, args.out, plt)
    plot_steps(all_steps, ctrls, args.out, plt)
    plot_step_directions(all_steps, ctrls, args.out, plt)
    plot_xy(runs, ctrls, mm, args.out, plt)
    plot_tracking_error(runs, mm, args.out, plt)
    plot_tilt(runs, args.out, plt)
    plot_bars(rows, ctrls, args.out, plt)
    print(f"\nWrote {args.out}/metrics.md, metrics.csv and fig_*.png")
    with open(os.path.join(args.out, "metrics.md")) as f:
        print(f.read())
    if not args.no_show:
        plt.show()


# ============================================================================ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("run", help="run the scripted test on the robot and save the logs")
    p.add_argument("port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--ctrl", choices=["pid", "lqr", "both"], default="both")
    p.add_argument("--repeat", type=int, default=1, help="runs per controller")
    p.add_argument("--dirs", type=int, default=8, help="number of edge points (1..12)")
    p.add_argument("--out", default="logs")
    p.add_argument("--tag", default="", help="text added to the file names, e.g. kp009")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("analyze", help="compute metrics and plots from logs")
    p.add_argument("files", nargs="+")
    p.add_argument("--out", default=None, help="folder for the report (default: report_<time>, never overwrites)")
    p.add_argument("--mm-per-px", type=float, default=1.09)
    p.add_argument("--no-show", action="store_true", help="only save the figures")
    p.set_defaults(func=cmd_analyze)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
