#!/usr/bin/env python3
"""
PID tuning helper for the 6-DOF ball balancing robot.

Works with Ball_Balance_PID_log.ino, which prints one CSV line per control step:
    D,t_ms,ball_x,ball_y,set_x,set_y,err_x,err_y,deriv_x,deriv_y,int_x,int_y,out_x,out_y,r_out
and one gains header at start-up:
    G,kp,kd,ki,T,r_max

Usage
-----
1) Record a run (close the Arduino Serial Monitor first, only one program can hold the port):
       python3 pid_plot.py log /dev/cu.usbmodem12345 run_kp005_kd005.csv
   Ctrl+C to stop. Find the port with:  ls /dev/cu.*
   Optional: --seconds 20 to stop automatically, --baud 115200.

2) Plot it:
       python3 pid_plot.py plot run_kp005_kd005.csv
   Saves run_kp005_kd005.png next to the CSV and opens a window.
   Compare runs:  python3 pid_plot.py plot run1.csv run2.csv run3.csv

   If the G line was missed (Teensy USB opens after setup), pass the gains:
       python3 pid_plot.py plot run.csv --kp -0.05 --kd -0.05 --ki 0 --rmax 15

Requires: pip install pyserial matplotlib numpy
"""
import argparse
import os
import sys
import time

COLS = ["t_ms", "ball_x", "ball_y", "set_x", "set_y", "err_x", "err_y",
        "deriv_x", "deriv_y", "int_x", "int_y", "out_x", "out_y", "r_out"]


# ----------------------------------------------------------------------------- logging
def cmd_log(args):
    import serial  # pyserial

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2.0)  # board may reset when the port opens
    n = 0
    t_end = time.time() + args.seconds if args.seconds else None
    with open(args.out, "w", newline="") as f:
        f.write("# " + ",".join(COLS) + "\n")
        print(f"Logging {args.port} @ {args.baud} -> {args.out}  (Ctrl+C to stop)")
        try:
            while t_end is None or time.time() < t_end:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("D,") or line.startswith("G,"):
                    f.write(line + "\n")
                    n += line.startswith("D,")
                    if n % 200 == 0 and line.startswith("D,"):
                        print(f"  {n} samples")
                else:
                    # error messages from the sketch (NaN, out of range, no response...)
                    print("  [robot]", line)
                    f.write("# " + line + "\n")
        except KeyboardInterrupt:
            pass
    ser.close()
    print(f"Saved {n} samples to {args.out}")


# ----------------------------------------------------------------------------- loading
def load(path):
    import numpy as np

    gains, rows, events = {}, [], []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("G,"):
                p = line.split(",")[1:]
                try:
                    gains = dict(kp=float(p[0]), kd=float(p[1]), ki=float(p[2]),
                                 T=float(p[3]), rmax=float(p[4]))
                except (IndexError, ValueError):
                    pass
            elif line.startswith("D,"):
                p = line.split(",")[1:]
                if len(p) != len(COLS):
                    continue
                try:
                    rows.append([float(v) for v in p])
                except ValueError:
                    continue
            elif line.startswith("# ") and not line.startswith("# t_ms"):
                events.append((len(rows), line[2:]))
    if not rows:
        sys.exit(f"{path}: no data lines (expected lines starting with 'D,')")
    a = np.array(rows)
    d = {c: a[:, i] for i, c in enumerate(COLS)}
    d["t"] = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    return d, gains, events


def metrics(t, e, band):
    """RMS, max |e|, settling time into +-band (stays there), overshoot past zero."""
    import numpy as np

    rms = float(np.sqrt(np.mean(e ** 2)))
    emax = float(np.max(np.abs(e)))
    outside = np.where(np.abs(e) > band)[0]
    if len(outside) == 0:
        ts = 0.0
    elif outside[-1] == len(e) - 1:
        ts = float("nan")  # never settled
    else:
        ts = float(t[outside[-1] + 1])
    # overshoot: how far it goes past the setpoint on the other side of where it started
    e0 = e[0]
    os_ = float(np.max(-np.sign(e0) * e)) if e0 != 0 else 0.0
    os_pct = 100.0 * max(os_, 0.0) / abs(e0) if abs(e0) > 1e-6 else float("nan")
    return dict(rms=rms, emax=emax, ts=ts, os_px=max(os_, 0.0), os_pct=os_pct)


# ----------------------------------------------------------------------------- plotting
def cmd_plot(args):
    import numpy as np
    import matplotlib.pyplot as plt

    multi = len(args.csv) > 1
    fig, ax = plt.subplots(4, 2, figsize=(13, 11), sharex=True)
    summary = []

    for path in args.csv:
        d, g, events = load(path)
        kp = args.kp if args.kp is not None else g.get("kp")
        kd = args.kd if args.kd is not None else g.get("kd")
        ki = args.ki if args.ki is not None else g.get("ki", 0.0)
        rmax = args.rmax if args.rmax is not None else g.get("rmax")
        name = os.path.splitext(os.path.basename(path))[0]
        label = name if multi else None
        t = d["t"]
        dt = np.diff(t)

        for col, ax_name in enumerate(["x", "y"]):
            ball = d[f"ball_{ax_name}"]
            sp = d[f"set_{ax_name}"]
            e = d[f"err_{ax_name}"]

            a0 = ax[0, col]
            a0.plot(t, ball, label=f"ball {ax_name}" if not multi else label)
            if not multi or path == args.csv[0]:
                a0.plot(t, sp, "k--", lw=1, label="setpoint (platform centre)")
            a0.set_ylabel(f"{ax_name} position [px, camera]")
            a0.set_title(f"{ax_name.upper()} axis")

            a1 = ax[1, col]
            a1.plot(t, e, label=label)
            a1.axhline(0, color="k", lw=0.8)
            a1.axhspan(-args.band, args.band, color="g", alpha=0.08)
            a1.set_ylabel("error = ball − setpoint [px]")

            a2 = ax[2, col]
            if kp is not None and kd is not None:
                a2.plot(t, kp * e, label="P term  kp·e" if not multi else f"{name} P")
                a2.plot(t, kd * d[f"deriv_{ax_name}"], label="D term  kd·de/dt" if not multi else f"{name} D")
                if ki:
                    a2.plot(t, ki * d[f"int_{ax_name}"], label="I term  ki·∫e" if not multi else f"{name} I")
                a2.set_ylabel("PID terms [deg]")
            else:
                a2.plot(t, d[f"deriv_{ax_name}"], label=label)
                a2.set_ylabel("filtered de/dt [px/s]\n(gains unknown)")
            a2.axhline(0, color="k", lw=0.8)

            a3 = ax[3, col]
            a3.plot(t, d[f"out_{ax_name}"], label=label)
            if rmax:
                a3.axhline(rmax, color="r", ls=":", lw=1)
                a3.axhline(-rmax, color="r", ls=":", lw=1)
            a3.set_ylabel("controller out [deg]\n(" + ("pitch" if ax_name == "x" else "−roll") + ")")
            a3.set_xlabel("time [s]")

            m = metrics(t, e, args.band)
            summary.append((name, ax_name, m))

        for idx, txt in events:
            if idx < len(t):
                for c in range(2):
                    for r in range(4):
                        ax[r, c].axvline(t[idx], color="r", alpha=0.4, lw=1)
                print(f"  {name}: event at t={t[idx]:.2f}s: {txt}")

        sat = float(np.mean(d["r_out"] > rmax)) * 100 if rmax else float("nan")
        print(f"\n{name}: {len(t)} samples, {t[-1]:.1f}s, loop dt median {np.median(dt)*1000:.1f} ms, "
              f"max {np.max(dt)*1000:.0f} ms" + (f", saturated {sat:.0f}% of the time" if rmax else ""))
        if kp is not None:
            print(f"  gains: kp={kp}, kd={kd}, ki={ki}, r_max={rmax}")

    for r in range(4):
        for c in range(2):
            ax[r, c].grid(alpha=0.3)
            ax[r, c].legend(fontsize=8, loc="upper right")

    print(f"\n{'run':<28}{'axis':<6}{'RMS err':>9}{'max|e|':>9}{'settle':>9}{'overshoot':>11}")
    for name, a, m in summary:
        ts = "never" if m["ts"] != m["ts"] else f"{m['ts']:.2f}s"
        os_ = "-" if m["os_pct"] != m["os_pct"] else f"{m['os_pct']:.0f}%"
        print(f"{name:<28}{a:<6}{m['rms']:>8.1f}p{m['emax']:>8.1f}p{ts:>9}{os_:>11}")
    print(f"(settle = last time |error| leaves ±{args.band:g} px; overshoot relative to the starting error)")

    title = "PID tuning: " + ", ".join(os.path.basename(p) for p in args.csv)
    fig.suptitle(title)
    fig.tight_layout()
    out_png = os.path.splitext(args.csv[0])[0] + ("_compare" if multi else "") + ".png"
    fig.savefig(out_png, dpi=130)
    print(f"\nSaved plot to {out_png}")
    if not args.no_show:
        plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    pl = sub.add_parser("log", help="record serial data to CSV")
    pl.add_argument("port")
    pl.add_argument("out")
    pl.add_argument("--baud", type=int, default=115200)
    pl.add_argument("--seconds", type=float, default=None)
    pl.set_defaults(func=cmd_log)

    pp = sub.add_parser("plot", help="plot one or more CSV logs")
    pp.add_argument("csv", nargs="+")
    pp.add_argument("--kp", type=float)
    pp.add_argument("--kd", type=float)
    pp.add_argument("--ki", type=float)
    pp.add_argument("--rmax", type=float)
    pp.add_argument("--band", type=float, default=10.0, help="settling band in px (default 10)")
    pp.add_argument("--no-show", action="store_true", help="only save the PNG")
    pp.set_defaults(func=cmd_plot)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
