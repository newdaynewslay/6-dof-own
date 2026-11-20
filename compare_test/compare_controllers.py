#!/usr/bin/env python3
"""
PID vs Kalman+PID vs LQR comparison for the 6-DOF ball balancing robot.

Works with compare_test/Ball_Balance_Compare/Ball_Balance_Compare.ino, which runs
the same scripted set point sequence for every controller (centre, steps to the
edge points and back, fast circle, pentagram) and prints one CSV row per control
step. Controllers: PID ("m 0"), LQR ("m 1"), KF-PID ("m 2") and KF-LQR ("m 3").
KF-PID / KF-LQR use the same gains as PID / LQR, but position and velocity come
from a Kalman filter.

The reference part takes about 70 s per controller (4 edge directions), plus ~20 s for the
disturbance part (tilt kicks + constant tilt, leave out with --no-dist). Between
runs the servos stay on and the ball is held at the centre, so the next controller
starts by itself.
FULL TEST (2026-11-19): ~90 s per run = settle 2 + centre hold 5 + 4 edge points
24 + 3 circles (R60 4 s, R40 2 s, R40 6 s) 32 + pentagram 11.5 + end 2 +
disturbance 20. Default 4 blocks x 4 controllers = 16 runs = one full Williams
square, about 29 minutes.

Usage
-----
1) Everything in one go (close the Arduino Serial Monitor / pio monitor first,
   put the ball on the platform):
       python3 compare_controllers.py        (macOS / Linux)
       py compare_controllers.py             (Windows)
   Finds the Teensy port itself (by its USB id, so COMx on Windows,
   /dev/cu.usbmodem* on macOS), switches the servos on, runs PID, KF-PID,
   LQR and KF-LQR back to back, switches the servos off, then writes the logs AND the
   report (metrics.md, metrics.csv, fig_*.png) into logs/<time>/.
   No key presses needed. If the ball is not on the platform it waits for it
   (60 s); if a run aborts (ball lost) it waits for the ball and retries once.
   Options (after "run", port optional):
       python3 compare_controllers.py run /dev/cu.usbmodem123 --ctrl pid,kf
       py compare_controllers.py run COM5 --ctrl pid,kf
       python3 compare_controllers.py ports      (list the serial ports)
       --ctrl all|pid|kf|lqr|both or a list   --repeat 3   --dirs 8 (old long test)
       --pause 3   --wait-ball 60   --retries 1   --no-analyze   --tag kp009
       --kf "150 1 0"   Kalman settings (qa r bu) for this test, bu 0 = tilt input off

1b) Live / manual (no logging): python3 compare_controllers.py live --ctrl lqr
   Servos on, the controller balances at the centre, then type s 50 50 / 50 50 / c /
   o 60 3 (circle) / p 60 60 (star) / m 0..3 / exit.

2) Analyse old logs again (any number, grouped by controller):
       python3 compare_controllers.py analyze logs/<time>/*.csv
   Writes metrics.csv, metrics.md and fig_*.png to a new folder report_<time>
   (or --out <folder>; an existing folder is overwritten).
   --mm-per-px sets the pixel scale used for mm values (default 1.045 from the
   corner nuts: 170 mm square, 120.2 mm from the centre, ~115 px in the camera).

Random order (since 2026-11-19)
-------------------------------
"run" now does BLOCK RANDOMISATION: --repeat N blocks, each block runs every
controller once. The orders come from a randomised Williams Latin square: in 4
blocks every controller runs once in each position (1st..4th) and follows each
other controller once; a 5th block starts a new random square. No controller
runs twice in a row across a block boundary. Each block also gets its own seed, sent to the sketch
("r dirs keep seed dist"), which shuffles the order of the edge directions and
of the disturbance kicks; all controllers in a block get the identical test.
The master seed (--seed, default from the clock) and the whole plan are saved
in logs/<time>/plan.csv, so the order can be reproduced and reported.
The disturbance part (tilt kicks + constant tilt, ~25 s) is appended to every
run unless --no-dist is given. Default: --repeat 4 -> 4 runs per controller.
fig_drift.png shows key metrics against the run order (time drift check).

Requires: pip install pyserial numpy matplotlib
(Windows: py -m pip install pyserial numpy matplotlib)
"""
import argparse
import csv
import glob
import math
import os
import random
import re
import sys
import time
from datetime import datetime

COLS = ["t_ms", "seg", "sp_x", "sp_y", "ball_x", "ball_y", "err_x", "err_y",
        "derr_x", "derr_y", "u_x", "u_y", "sat", "found", "ik"]
COLS_DIST = COLS + ["d_x", "d_y"]   # logs since 2026-11-19 (disturbance tilt, deg)
CTRL_ID = {"pid": 0, "lqr": 1, "kf": 2, "kflqr": 3, "pidzn": 4}
CTRL_ORDER = ("PID", "PID-ZN", "KF-PID", "LQR", "KF-LQR")   # names printed by the sketch ("# CTRL ...")

# colours: one fixed colour per controller, reference
COLORS = {"PID-ZN": "#8a4fd0", "PID": "#2a78d6", "KF-PID": "#eb6834", "LQR": "#1baf7a", "KF-LQR": "#c98500"}  # same as servo_action/*.py figures
REF = "#555555"

STEP_MIN = 5.0     # px, smaller set point jumps are not counted as steps
BAND_REL = 0.05    # settling band: 5 % of the step ...
BAND_MIN = 3.0     # ... but at least this many px (Pixy noise)
SS_WINDOW = 1.5    # s, steady-state error = mean |e| over the last part of a hold
DIST_BAND = 5.0    # px (~5 mm), "recovered" = error stays inside this band


# ============================================================================ run
def read_line(ser):
    return ser.readline().decode(errors="ignore").strip()


def send(ser, text):
    ser.write((text + "\n").encode())
    ser.flush()


TEENSY_VID = 0x16C0  # PJRC (Teensy) USB vendor id


def list_ports():
    """All serial ports as (device, description, is_teensy)."""
    from serial.tools import list_ports as lp
    out = []
    for p in lp.comports():
        desc = " ".join(x for x in (p.description, p.manufacturer, p.product) if x and x != "n/a")
        teensy = p.vid == TEENSY_VID or "teensy" in desc.lower()
        out.append((p.device, desc, teensy))
    return sorted(out)


def find_port():
    """The Teensy's USB serial port (COMx on Windows, /dev/cu.usbmodem* on macOS,
    /dev/ttyACM* on Linux), or exit with a hint."""
    ports = list_ports()
    teensy = [d for d, _, t in ports if t]
    if not teensy:  # no USB id info (old pyserial / odd driver): fall back to the names
        teensy = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not teensy:
        found = "\n".join(f"  {d}  {desc}" for d, desc, _ in ports) or "  (none)"
        example = "COM5" if os.name == "nt" else "/dev/cu.usbmodemXXXX"
        sys.exit("No Teensy found. Serial ports on this PC:\n" + found +
                 f"\nPlug it in, or give the port: compare_controllers.py run {example}")
    if len(teensy) > 1:
        print(f"Several Teensy ports found {teensy}, using {teensy[0]}")
    return teensy[0]


def port_present(port):
    """True while the port is still there (works for COMx and /dev/... names)."""
    try:
        names = [d for d, _, _ in list_ports()]
    except Exception:
        return os.path.exists(port)
    return port in names or os.path.exists(port)


def safe_name(text):
    """Folder / file name part that is valid on Windows too (no : * ? " < > | / \\)."""
    return re.sub(r'[<>:"/\\|?*\s]+', "-", text).strip("-.")


def start_when_ready(ser, dirs, keep, wait_ball, seed=0, dist=False):
    """Sends "r" until the test starts. Switches the servos on when they are off
    and waits for the ball (up to wait_ball s). Returns the first lines it read
    (they belong in the log), or None if it gave up."""
    t0 = time.time()
    said_wait = False
    while time.time() - t0 < wait_ball:
        ser.reset_input_buffer()
        send(ser, f"r {dirs} {1 if keep else 0} {seed} {1 if dist else 0}")
        t1 = time.time()
        lines = []
        while time.time() - t1 < 1.0:
            line = read_line(ser)
            if not line:
                continue
            lines.append(line)
            if line.startswith("# START"):
                return lines
            if line.startswith("# ERROR servos off"):
                send(ser, "e")          # servos on, platform goes home
                time.sleep(1.5)
                break
            if line.startswith("# ERROR no ball"):
                if not said_wait:
                    print(f"  waiting for the ball - put it in the centre (max {wait_ball:.0f} s)")
                    said_wait = True
                time.sleep(0.5)
                break
    return None


def record_one(ser, ctrl, dirs, path, keep=False, pause=3.0, wait_ball=60.0, seed=0, dist=False):
    """Runs one test on the board without any user input and saves all lines to
    path. keep = leave the servos on after this run so the next one can start
    with the ball still balanced. Returns True if it ended normally."""
    send(ser, f"m {CTRL_ID[ctrl]}")
    print(f"\n[{ctrl.upper()}] controller set, starting in {pause:.0f} s ...")
    time.sleep(pause)  # the new controller settles the ball at the centre
    first = start_when_ready(ser, dirs, keep, wait_ball, seed, dist)
    if first is None:
        print("  no ball found - run skipped")
        return False
    n = 0
    status = None
    t_last = time.time()
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for line in first:
            f.write(line + "\n")
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
                if not line.startswith(("# SEG", "# SERVOS")):
                    print("  [robot]", line[2:])
            elif line[0].isdigit():
                n += 1
                if n % 500 == 0:
                    seg = line.split(",")[1]
                    print(f"  {n} samples (segment {seg})")
    print(f"  {status}: {n} samples -> {path}")
    return status == "END"


def williams_square(n):
    """Rows of a Williams Latin square: every item once per row and once per
    position, and (for even n) every item directly follows every other item
    exactly once, so carry-over / drift effects are balanced too."""
    first = [0]
    lo, hi = 1, n - 1
    for k in range(1, n):
        if k % 2:
            first.append(lo)
            lo += 1
        else:
            first.append(hi)
            hi -= 1
    rows = [[(v + i) % n for v in first] for i in range(n)]
    if n % 2:   # odd n needs the mirrored rows as well for full balance
        rows += [r[::-1] for r in rows]
    return rows


def make_plan(ctrls, blocks, master_seed):
    """Block randomisation: every block holds each controller once. The block
    orders come from a Williams Latin square (randomly relabelled and with the
    blocks in random order), so over a full square every controller runs equally
    often first / second / ... and after each other controller. Blocks beyond
    one square start a new random square. A controller never runs twice in a row
    across a block boundary. Each block gets its own test seed (1..99999) for
    the order of the directions inside the test."""
    rng = random.Random(master_seed)
    n = len(ctrls)
    orders = []
    while len(orders) < blocks:
        labels = list(ctrls)
        rng.shuffle(labels)
        rows = [[labels[i] for i in r] for r in williams_square(n)] if n > 1 else [labels]
        for _ in range(200):
            rng.shuffle(rows)
            prev = orders[-1][-1] if orders else None
            if n < 2 or (rows[0][0] != prev and
                         all(rows[i][-1] != rows[i + 1][0] for i in range(len(rows) - 1))):
                break
        orders += rows
    plan = []
    for b, order in enumerate(orders[:blocks]):
        block_seed = rng.randint(1, 99999)
        for pos, c in enumerate(order):
            plan.append({"block": b + 1, "pos": pos + 1, "ctrl": c, "seed": block_seed})
    return plan


def cmd_run(args):
    import serial  # pyserial

    if args.ctrl == "all":
        ctrls = ["pid", "kf", "lqr", "kflqr"]
    elif args.ctrl == "both":
        ctrls = ["pid", "lqr"]
    else:
        ctrls = [c.strip() for c in args.ctrl.split(",") if c.strip()]
        bad = [c for c in ctrls if c not in CTRL_ID]
        if bad or not ctrls:
            sys.exit(f"unknown controller(s) {bad}, use pid, pidzn, kf, lqr, kflqr, all or both")
    port = args.port or find_port()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")   # no ':' -> valid on Windows
    tag = f"_{safe_name(args.tag)}" if safe_name(args.tag) else ""
    folder = os.path.join(args.out, f"{stamp}{tag}")   # logs AND report go here
    os.makedirs(folder, exist_ok=True)
    print(f"Port {port}, controllers {', '.join(ctrls)}, results -> {folder}{os.sep}")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.5)
    except serial.SerialException as ex:
        sys.exit(f"Cannot open {port}: {ex}\n"
                 "Close the Arduino Serial Monitor / pio device monitor (only one program can "
                 "use the port), or check the port name with: compare_controllers.py ports")
    time.sleep(1.0)
    ser.reset_input_buffer()
    send(ser, "?")
    time.sleep(0.3)
    settings = []
    while ser.in_waiting:
        line = read_line(ser)
        settings.append(line)
        print("  [robot]", line)
    # gain log of this test (also in every CSV header): which gains produced these results
    with open(os.path.join(folder, "gains.txt"), "w", encoding="utf-8") as f:
        f.write(f"Test {stamp}{tag}, controllers {', '.join(ctrls)}\n")
        f.write("Gains reported by the robot before the test:\n")
        for line in settings:
            if line.startswith(("# PID", "# LQR", "# KF", "# COMMON")):
                f.write(line[2:] + "\n")
    if args.kf:  # Kalman settings for this test: "qa r bu"
        send(ser, "f " + args.kf)
        time.sleep(0.3)
        while ser.in_waiting:
            line = read_line(ser)
            if line.startswith("# KF"):
                print("  [robot]", line[2:])
    saved = []
    master = args.seed if args.seed is not None else int(time.time()) % 1000000
    plan = make_plan(ctrls, max(1, args.repeat), master)
    dist = not args.no_dist
    plan_path = os.path.join(folder, "plan.csv")
    print(f"Master seed {master} (repeat with --seed {master}), disturbance part {'on' if dist else 'off'}")
    for b in sorted(set(p["block"] for p in plan)):
        items = [p for p in plan if p["block"] == b]
        print(f"  block {b} (seed {items[0]['seed']}): " + ", ".join(p["ctrl"].upper() for p in items))

    def write_plan():
        with open(plan_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["run", "block", "pos", "ctrl", "block_seed", "master_seed", "dist",
                        "start", "status", "attempts", "file"])
            for k, p in enumerate(plan):
                w.writerow([k + 1, p["block"], p["pos"], p["ctrl"], p["seed"], master, int(dist),
                            p.get("start", ""), p.get("status", "not run"), p.get("attempts", 0),
                            p.get("file", "")])
    write_plan()
    t_start = time.time()
    try:
        for i, p in enumerate(plan):
            ctrl = p["ctrl"]
            last = i == len(plan) - 1
            print(f"\n=== run {i + 1}/{len(plan)}: block {p['block']}, #{p['pos']}, {ctrl.upper()}")
            p["start"] = datetime.now().strftime("%H:%M:%S")
            for attempt in range(1 + args.retries):
                p["attempts"] = attempt + 1
                name = f"{stamp}{tag}_n{i + 1:02d}_b{p['block']}_{ctrl}.csv"
                path = os.path.join(folder, name)
                ok = record_one(ser, ctrl, args.dirs, path, keep=not last, pause=args.pause,
                                wait_ball=args.wait_ball, seed=p["seed"], dist=dist)
                p["status"] = "ok" if ok else "failed"
                p["file"] = name if ok else ""
                write_plan()
                if ok:
                    saved.append(path)
                    break
                if not port_present(port):
                    print("  The board is gone from USB - stopping.")
                    raise KeyboardInterrupt
                failed = path.replace(".csv", f"_FAILED{attempt + 1}.csv")
                if os.path.exists(path):
                    os.replace(path, failed)
                if attempt < args.retries:
                    print(f"  retrying {ctrl.upper()} ({attempt + 2}/{1 + args.retries}) - "
                          "put the ball back in the centre if it fell off")
    except KeyboardInterrupt:
        print("\nStopped.")
        try:
            send(ser, "x")
            time.sleep(0.1)
        except Exception:
            pass
    try:
        send(ser, "q")  # make sure the servos are off at the end ("q" since 2026-11-20)
        ser.close()
    except Exception:
        pass
    print(f"\nTest time {time.time() - t_start:.0f} s, {len(saved)} of {len(plan)} runs saved")

    if not saved:
        print("Nothing to analyse.")
        return
    if args.no_analyze:
        print("Analyse with:\n  python3 compare_controllers.py analyze " + os.path.join(folder, "*.csv"))
        return
    # analyse right away, report into the same timestamped folder
    print("\nAnalysing ...")
    cmd_analyze(argparse.Namespace(files=saved, out=folder, mm_per_px=args.mm_per_px, no_show=True))


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
    cols = COLS
    with open(path, encoding="utf-8", errors="replace") as f:
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
                elif line.startswith("# TEST"):
                    for kv in re.findall(r"(\w+)=([-\d.]+)", line):
                        r.settings["TEST_" + kv[0]] = float(kv[1])
                elif line.startswith("# END"):
                    r.status = "END"
                elif line.startswith("# ABORT"):
                    r.status = line[2:]
                elif line.startswith(("# PID", "# LQR", "# KF", "# COMMON")):
                    for kv in re.findall(r"(\w+)=([-\d.]+)", line):
                        r.settings[line.split()[1] + "_" + kv[0]] = float(kv[1])
                continue
            if line.startswith("t_ms"):
                cols = COLS_DIST if line.split(",") == COLS_DIST else COLS
                continue
            if not line[0].isdigit():
                continue
            parts = line.split(",")
            if len(parts) != len(cols):
                continue
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                pass
    import numpy as np
    a = np.array(rows) if rows else np.zeros((0, len(cols)))
    r.d = {c: a[:, i] for i, c in enumerate(cols)}
    for c in ("d_x", "d_y"):   # old logs: no disturbance
        r.d.setdefault(c, np.zeros(len(a)))
    for s in r.segs:
        for k in ("dx", "dy", "dd"):
            s.setdefault(k, 0.0)
    mo = re.search(r"_n(\d+)_b(\d+)_", r.name)   # run order / block from the file name
    r.order = int(mo.group(1)) if mo else None
    r.block = int(mo.group(2)) if mo else None
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
        low = r.name.lower()
        if "kflqr" in low:
            r.ctrl = "KF-LQR"
        elif "lqr" in low:
            r.ctrl = "LQR"
        else:
            r.ctrl = "KF-PID" if "_kf" in low else "PID"
    return r


# ============================================================================ metrics
def seg_mask(r, s):
    return r.d["seg"] == s["i"]


def trapz(yv, xv):
    import numpy as np
    m = ~np.isnan(yv)
    if m.sum() < 2:
        return float("nan")
    integ = getattr(np, "trapezoid", None) or np.trapz  # numpy 2 renamed trapz
    return float(integ(yv[m], xv[m]))


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

    # Rise time is measured relative to the FINAL value (textbook definition: 10-90 % of
    # where the ball ended up, mean progress over the last SS_WINDOW s), not relative to the
    # target. Before 2026-11-17 it used the target, so steps that stopped short of 90 % got NaN
    # and were silently left out of the mean, which made slow controllers look faster.
    # The shortfall is reported by "Steady-state error" and "Steps reaching 90 % of target".
    ss = t >= (s["dur"] - SS_WINDOW)
    final = float(np.nanmean(prog[ss])) if ss.any() and np.any(~np.isnan(prog[ss])) else float("nan")
    if not math.isnan(final) and final > 0.2:
        rise = first_cross(0.9 * final) - first_cross(0.1 * final)
    else:
        rise = float("nan")                            # ball hardly moved: no rise time
    reached = 100.0 if len(prog) and np.nanmax(prog) >= 0.9 else 0.0

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
    return {
        "run": r.name, "ctrl": r.ctrl, "seg": s["name"], "angle": k,
        "amp_px": A,
        "rise_s": rise,
        "final_pct": final * 100 if not math.isnan(final) else float("nan"),
        "reached_pct": reached,
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
    if s["kind"] == 1 and s["dur"] > 1.5 * abs(s["p"]):
        # circle: drop the first turn (transient from the entry / previous speed)
        m = m & (r.t >= s["t0"] + abs(s["p"]))
    t = r.t[m] - s["t0"]
    e = r.e[m]
    u = r.u[m]
    du = np.abs(np.diff(np.hypot(r.d["u_x"][m], r.d["u_y"][m])))
    res = {
        "run": r.name, "ctrl": r.ctrl, "seg": s["name"], "R": s["R"], "p": s["p"],
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
        # frequency response at f = 1/|p|: least-squares fit of the ball path to the
        # reference circle, H = <z_ball * conj(z_ref)> / R^2 (z = x + iy). |H| = amplitude
        # ratio, angle(H) = phase (negative = ball lags). The circle is a sine on x and y.
        ok = ~np.isnan(bx)
        if ok.sum() > 10 and s["R"] > 0:
            zb = (bx[ok] - np.nanmean(bx[ok])) + 1j * (by[ok] - np.nanmean(by[ok]))
            zr = spx[ok] + 1j * spy[ok]
            H = np.mean(zb * np.conj(zr)) / np.mean(np.abs(zr) ** 2)
            if s["p"] < 0:
                H = np.conj(H)
            res["freq_hz"] = 1.0 / abs(s["p"])
            res["gain"] = float(np.abs(H))
            res["gain_db"] = float(20 * np.log10(max(np.abs(H), 1e-6)))
            res["phase_deg"] = float(np.degrees(np.angle(H)))
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


def dist_metrics(r, s):
    """Disturbance segment (kick_ / push_ / release): the set point is the centre."""
    import numpy as np
    m = seg_mask(r, s)
    t = r.t[m] - s["t0"]
    e = r.e[m]
    u = r.u[m]
    res = {"run": r.name, "ctrl": r.ctrl, "seg": s["name"], "dist_deg": float(np.hypot(s["dx"], s["dy"]))}
    if len(t) < 5 or np.all(np.isnan(e)):
        return None
    i_pk = int(np.nanargmax(e))
    res["peak_dev_px"] = float(e[i_pk])
    res["t_peak_s"] = float(t[i_pk])
    res["u_peak_deg"] = float(np.nanmax(u))
    res["iae_pxs"] = trapz(e, t)
    # recovery: from the end of the pulse (kick) / the start (push, release) until the error
    # stays inside DIST_BAND for the rest of the segment
    t_ref = s["dd"] if s["name"].startswith("kick_") else 0.0
    out = np.where((e > DIST_BAND) & (t >= t_ref))[0]
    if len(out) == 0:
        res["recover_s"] = 0.0
    elif out[-1] == len(t) - 1:
        res["recover_s"] = float("nan")       # never back in the band in this segment
    else:
        res["recover_s"] = float(t[out[-1] + 1] - t_ref)
    ss = t >= s["dur"] - SS_WINDOW
    res["ss_err_px"] = float(np.nanmean(e[ss])) if ss.any() else float("nan")
    return res


def analyse_run(r):
    import numpy as np
    steps, tracks, holds, dists = [], [], [], []
    prev = np.array([0.0, 0.0])
    t_dist = None   # start of the disturbance part (excluded from the old "total" metrics)
    for s in r.segs:
        if s["name"].startswith(("kick_", "push_")) or s["name"] == "release":
            if t_dist is None:
                t_dist = s["t0"]
            dm = dist_metrics(r, s)
            if dm:
                dists.append(dm)
            continue
        sp = np.array([s["x"], s["y"]]) if s["kind"] == 0 else None
        if s["kind"] == 0:
            if s["name"] in ("settle", "end"):
                holds.append(hold_metrics(r, s))
            elif s["name"] == "hold":
                holds.append(hold_metrics(r, s, window=s["dur"]))
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
    # "whole test" = the reference part only (same as before the disturbance part existed)
    w = r.t < t_dist if t_dist is not None else np.ones(len(r.t), bool)
    tw, ew, uw = r.t[w], r.e[w], r.u[w]
    tot = {"run": r.name, "ctrl": r.ctrl, "status": r.status,
           "order": r.order, "block": r.block,
           "duration_s": float(r.t[-1]) if len(r.t) else 0.0,
           "samples": len(r.t),
           "rate_hz": float(len(r.t) / r.t[-1]) if len(r.t) and r.t[-1] > 0 else float("nan"),
           "lost": int(np.sum(r.d["found"] != 1)),
           "ik_fail": int(np.sum(r.d["ik"] == 0)),
           "sat_pct": float(np.mean(r.d["sat"][w]) * 100) if len(tw) else float("nan"),
           "iae_total_pxs": trapz(ew, tw),
           "u_rms_deg": float(np.sqrt(np.nanmean(uw ** 2))) if len(tw) else float("nan")}
    so = [st["rise_s"] for st in steps if st["seg"].startswith("out_")]
    tot["rise_out_s"] = float(np.nanmean(so)) if so else float("nan")
    ci = [tr["rms_err_px"] for tr in tracks if tr["seg"] == "circle"]
    tot["circle_rms_px"] = ci[0] if ci else float("nan")
    kp_ = [d["peak_dev_px"] for d in dists if d["seg"].startswith("kick_")]
    tot["kick_peak_px"] = float(np.nanmean(kp_)) if kp_ else float("nan")
    return steps, tracks, holds, dists, tot


# ============================================================================ report
def mean_std(vals):
    import numpy as np
    v = np.array([x for x in vals if x is not None and not (isinstance(x, float) and math.isnan(x))])
    if len(v) == 0:
        return float("nan"), float("nan"), 0
    return float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0, len(v)


def circle_title(it):
    return f"Circle R{it['R'] * 1.045:.0f} mm, {abs(it['p']):g} s/turn ({1 / abs(it['p']):.2f} Hz)"


def build_table(all_steps, all_tracks, all_holds, all_tot, ctrls, mm, all_dists=()):
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
        add(direction, "Rise time 10-90 % (of final value)", "s", all_steps, "rise_s", filt=f)
        add(direction, "Steps reaching 90 % of target", "%", all_steps, "reached_pct", filt=f)
        add(direction, "Final position (of step size)", "%", all_steps, "final_pct", filt=f)
        add(direction, "Overshoot", "%", all_steps, "overshoot_pct", filt=f)
        add(direction, f"Settling time ({BAND_REL * 100:.0f} % / {BAND_MIN:.0f} px band)", "s", all_steps, "settle_s", filt=f)
        add(direction, "Steady-state error", "mm", all_steps, "ss_err_px", mm, filt=f)
        add(direction, "Max deviation from straight path", "mm", all_steps, "cross_max_px", mm, filt=f)
        add(direction, "IAE", "mm*s", all_steps, "iae_pxs", mm, filt=f)
        add(direction, "Peak tilt", "deg", all_steps, "u_peak_deg", filt=f)
    circ = []   # circle segment names in test order, e.g. circle, circle2, circle3
    for it in all_tracks:
        if it["seg"].startswith("circle") and it["seg"] not in circ:
            circ.append(it["seg"])
    titles = []
    for seg in circ:
        first = next(it for it in all_tracks if it["seg"] == seg)
        titles.append((seg, circle_title(first) + " - first turn dropped"))
    for seg, title in titles + [("star", "Pentagram")]:
        f = (lambda sg: (lambda it: it["seg"] == sg))(seg)
        add(title, "RMS tracking error", "mm", all_tracks, "rms_err_px", mm, filt=f)
        add(title, "Max tracking error", "mm", all_tracks, "max_err_px", mm, filt=f)
        add(title, "IAE", "mm*s", all_tracks, "iae_pxs", mm, filt=f)
        if seg.startswith("circle"):
            add(title, "Mean radius error (ball - ref)", "mm", all_tracks, "radius_err_px", mm, filt=f)
            add(title, "Amplitude ratio (ball / ref, sine fit)", "dB", all_tracks, "gain_db", filt=f)
            add(title, "Phase (sine fit, negative = lag)", "deg", all_tracks, "phase_deg", filt=f)
            add(title, "Phase lag", "deg", all_tracks, "phase_lag_deg", filt=f)
            add(title, "Time lag", "s", all_tracks, "lag_s", filt=f)
        add(title, "RMS tilt", "deg", all_tracks, "u_rms_deg", filt=f)
        add(title, "Tilt rate (servo activity)", "deg/s", all_tracks, "u_rate_degps", filt=f)
        add(title, "Time at tilt limit", "%", all_tracks, "sat_pct", filt=f)
    hold_seg = "hold" if any(it["seg"] == "hold" for it in all_holds) else "end"
    g = ("Centre hold ('hold', 5 s after settle)" if hold_seg == "hold"
         else "Centre hold (last 3 s of 'end')")
    hf = lambda it: it["seg"] == hold_seg
    add(g, "RMS error", "mm", all_holds, "rms_err_px", mm, filt=hf)
    add(g, "Max error", "mm", all_holds, "max_err_px", mm, filt=hf)
    add(g, "RMS tilt", "deg", all_holds, "u_rms_deg", filt=hf)
    if all_dists:
        g = f"Disturbance: tilt kick (pulse, recovery = back inside {DIST_BAND:.0f} px)"
        f = lambda it: it["seg"].startswith("kick_")
        add(g, "Kick size", "deg", all_dists, "dist_deg", filt=f)
        add(g, "Peak deviation", "mm", all_dists, "peak_dev_px", mm, filt=f)
        add(g, "Time to peak", "s", all_dists, "t_peak_s", filt=f)
        add(g, "Recovery time (after the pulse)", "s", all_dists, "recover_s", filt=f)
        add(g, "IAE", "mm*s", all_dists, "iae_pxs", mm, filt=f)
        add(g, "Peak tilt", "deg", all_dists, "u_peak_deg", filt=f)
        g = "Disturbance: constant tilt (push on, then release)"
        f = lambda it: it["seg"].startswith("push_")
        add(g, "Push size", "deg", all_dists, "dist_deg", filt=f)
        add(g, "Peak deviation (push)", "mm", all_dists, "peak_dev_px", mm, filt=f)
        add(g, "Recovery time (push)", "s", all_dists, "recover_s", filt=f)
        add(g, "Remaining offset (last 1.5 s of push)", "mm", all_dists, "ss_err_px", mm, filt=f)
        add(g, "IAE (push)", "mm*s", all_dists, "iae_pxs", mm, filt=f)
        f = lambda it: it["seg"] == "release"
        add(g, "Peak deviation (release)", "mm", all_dists, "peak_dev_px", mm, filt=f)
        add(g, "Recovery time (release)", "s", all_dists, "recover_s", filt=f)
    add("Whole test", "Total IAE", "mm*s", all_tot, "iae_total_pxs", mm)
    add("Whole test", "RMS tilt", "deg", all_tot, "u_rms_deg")
    add("Whole test", "Time at tilt limit", "%", all_tot, "sat_pct")
    add("Whole test", "Control rate", "Hz", all_tot, "rate_hz")
    add("Whole test", "Ball-lost samples", "-", all_tot, "lost")
    return rows


def write_tables(rows, ctrls, outdir, runs, mm):
    with open(os.path.join(outdir, "metrics.csv"), "w", newline="", encoding="utf-8") as f:
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

    with open(os.path.join(outdir, "metrics.md"), "w", encoding="utf-8") as f:
        f.write("# " + " vs ".join(ctrls) + " - scripted test metrics\n\n")
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
    # one block only: runs in other blocks have a different (shuffled) direction order
    blk = sorted(runs, key=lambda r: (r.order or 0))[0]
    seed = blk.settings.get("TEST_seed")
    runs = [r for r in runs if r.settings.get("TEST_seed") == seed]
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
    fig.suptitle("Scripted test - ball position vs set point"
                 + (f" (block {blk.block}, one run per controller)" if blk.block else ""), x=0.01, ha="left")
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
    t_hold = max([st["t"][-1] for st in all_steps if len(st["t"])] or [5.0])
    tg = np.linspace(0, t_hold, 251)
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
                    elif nm == "circle":
                        if not s["name"].startswith("circle") or s["name"].endswith("_in"):
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
    names = [n for n in ("circle", "circle2", "circle3", "star") if any(s["name"] == n for r in runs for s in r.segs)]
    fig, axs = plt.subplots(1, len(names), figsize=(4.2 * len(names), 4), sharey=True, squeeze=False)
    axs = axs[0]
    for ax, nm in zip(axs, names):
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
    pick = [("Step out (centre -> edge)", "Rise time 10-90 % (of final value)"),
            ("Step out (centre -> edge)", "Overshoot"),
            ("Step out (centre -> edge)", f"Settling time ({BAND_REL * 100:.0f} % / {BAND_MIN:.0f} px band)"),
            ("Step out (centre -> edge)", "Steady-state error"),
            ("Pentagram", "RMS tracking error"),
            ("Whole test", "RMS tilt")]
    circ = [g for g, *_ in rows if g.startswith("Circle")]
    first_circle = circ[0] if circ else "Fast circle"
    pick += [(first_circle, "RMS tracking error"), (first_circle, "Phase lag"),
             (first_circle, "Tilt rate (servo activity)")]
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
        short = g.split(" (")[0].split(" - ")[0]
        ax.set_title(f"{short}: {lab} [{unit}]", loc="left", fontsize=9.5)
        style(ax)
    for k in range(n, axs.size):
        axs[k // cols][k % cols].axis("off")
    fig.suptitle("Key metrics (bar = mean, whisker = std over steps / runs)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_metrics.png"), dpi=150)


def plot_freq(all_tracks, ctrls, outdir, plt):
    """Amplitude ratio and phase vs circle frequency (mean +- std over runs) per controller."""
    import numpy as np
    pts = [it for it in all_tracks if "freq_hz" in it]
    if len({round(it["freq_hz"], 3) for it in pts}) < 2:
        return
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    for j, c in enumerate(ctrls):
        fs = sorted({round(it["freq_hz"], 3) for it in pts if it["ctrl"] == c})
        if not fs:
            continue
        off = (j - (len(ctrls) - 1) / 2) * 0.006   # small x offset so the bars do not overlap
        for ax, key in ((a1, "gain_db"), (a2, "phase_deg")):
            mu = [mean_std([it[key] for it in pts if it["ctrl"] == c and round(it["freq_hz"], 3) == f]) for f in fs]
            ax.errorbar(np.array(fs) + off, [m[0] for m in mu], yerr=[m[1] for m in mu], fmt="o-",
                        color=COLORS.get(c), label=c, capsize=3, lw=1.2, ms=5)
    a1.axhline(0, color=REF, lw=0.8, ls="--")
    a1.set_ylabel("amplitude ratio [dB]")
    a2.set_ylabel("phase [deg]")
    a2.set_xlabel("circle frequency [Hz]")
    for ax in (a1, a2):
        style(ax)
    a1.legend(fontsize=8, frameon=False)
    fig.suptitle("Frequency response from the circles (ball vs set point, sine fit)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_freq.png"), dpi=150)


def plot_drift(all_tot, ctrls, mm, outdir, plt):
    """Key metrics against the run order: a trend means something drifted over the session
    (battery, temperature, ball / plate), which the random order spreads over the controllers."""
    import numpy as np
    tot = [t for t in all_tot if t.get("order") is not None]
    if len(tot) < 2:
        return
    items = [("iae_total_pxs", "Total IAE (reference part)", "mm*s", mm),
             ("rise_out_s", "Step out rise time", "s", 1.0),
             ("circle_rms_px", "Circle RMS error", "mm", mm),
             ("kick_peak_px", "Kick peak deviation", "mm", mm)]
    items = [it for it in items if any(not math.isnan(t[it[0]]) for t in tot)]
    fig, axs = plt.subplots(len(items), 1, figsize=(9, 2.4 * len(items)), sharex=True, squeeze=False)
    for ax, (key, lab, unit, sc) in zip(axs[:, 0], items):
        for c in ctrls:
            pts = sorted((t["order"], t[key] * sc) for t in tot if t["ctrl"] == c and not math.isnan(t[key]))
            if pts:
                xs, ys = zip(*pts)
                ax.plot(xs, ys, "o-", color=COLORS.get(c), label=c, lw=1, ms=5)
        blocks = sorted(set(t["block"] for t in tot))
        for b in blocks[1:]:
            first = min(t["order"] for t in tot if t["block"] == b)
            ax.axvline(first - 0.5, color="#cccccc", lw=0.8, zorder=0)
        ax.set_ylabel(f"{lab}\n[{unit}]", fontsize=8.5)
        style(ax)
    axs[0, 0].legend(fontsize=8, ncol=len(ctrls), loc="upper left", frameon=False)
    axs[-1, 0].set_xlabel("run number (execution order, grey lines = new block)")
    fig.suptitle("Drift check: metrics vs run order", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_drift.png"), dpi=150)


def plot_dist(all_dists, ctrls, mm, outdir, plt):
    """Mean |error| after a tilt kick, per controller (all kicks pooled)."""
    if not all_dists:
        return
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.text(0.99, 0.97, "bars: peak deviation of each tilt kick", transform=ax.transAxes,
            ha="right", va="top", fontsize=8, color="#555555")
    for j, c in enumerate(ctrls):
        v = [d["peak_dev_px"] * mm for d in all_dists if d["ctrl"] == c and d["seg"].startswith("kick_")]
        if v:
            mu, sd, _ = mean_std(v)
            ax.bar(j, mu, 0.6, yerr=sd, color=COLORS.get(c), capsize=4, error_kw={"ecolor": "#555555"})
            ax.text(j, mu, f"{mu:.3g}", ha="center", va="bottom", fontsize=9)
    ax.set_xticks(range(len(ctrls)))
    ax.set_xticklabels(ctrls)
    ax.set_ylabel("peak deviation [mm]")
    style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_disturbance.png"), dpi=150)


def plot_dist_time(runs, ctrls, mm, outdir, plt):
    """Error vs time after each kick, all runs overlaid per controller."""
    import numpy as np
    have = [r for r in runs if any(s["name"].startswith("kick_") for s in r.segs)]
    if not have:
        return
    fig, axs = plt.subplots(1, len(ctrls), figsize=(3.2 * len(ctrls), 3.2), sharey=True, squeeze=False)
    for ax, c in zip(axs[0], ctrls):
        for r in have:
            if r.ctrl != c:
                continue
            for s in r.segs:
                if s["name"].startswith(("kick_", "push_")) or s["name"] == "release":
                    m = seg_mask(r, s)
                    ax.plot(r.t[m] - s["t0"], r.e[m] * mm, color=COLORS.get(c), lw=0.7,
                            alpha=0.5 if s["name"].startswith("kick_") else 0.25)
        ax.axhline(DIST_BAND * mm, color=REF, lw=0.8, ls="--")
        ax.set_title(c, loc="left", fontsize=10)
        ax.set_xlabel("time in segment [s]")
        style(ax)
    axs[0, 0].set_ylabel("|error| [mm]")
    fig.suptitle("Disturbance response (kicks strong, push / release faint; dashed = recovery band)",
                 x=0.01, ha="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_disturbance_time.png"), dpi=150)


def cmd_analyze(args):
    import warnings
    warnings.filterwarnings("ignore", category=RuntimeWarning)  # NaN means of empty / lost-ball slices
    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    files = []
    for p in args.files:   # the script expands *.csv itself (cmd / PowerShell do not)
        files += sorted(glob.glob(p))
    # aborted attempts (*_FAILED1.csv ...) are kept on disk but never mixed into a report,
    # so a whole logs/*/*.csv pattern can be used to pool many test sessions
    skipped = [f for f in files if "FAILED" in os.path.basename(f)]
    files = sorted(set(f for f in files if f not in skipped))
    if skipped:
        print(f"skipping {len(skipped)} aborted run(s): " + ", ".join(os.path.basename(f) for f in skipped))
    runs = [load_run(p) for p in files]
    runs = [r for r in runs if len(r.t) > 0]
    order = {c: i for i, c in enumerate(CTRL_ORDER)}
    runs.sort(key=lambda r: (order.get(r.ctrl, 99), r.name))
    if not runs:
        sys.exit("no data in the given files")
    ctrls = [c for c in CTRL_ORDER if any(r.ctrl == c for r in runs)]
    if args.out is None:
        args.out = "report_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(args.out, exist_ok=True)
    mm = args.mm_per_px

    all_steps, all_tracks, all_holds, all_tot, all_dists = [], [], [], [], []
    for r in runs:
        st, tr, ho, di, tot = analyse_run(r)
        all_dists += di
        all_steps += st
        all_tracks += tr
        all_holds += ho
        all_tot.append(tot)
        print(f"{r.name}: {r.ctrl}, {tot['status']}, {tot['duration_s']:.1f} s, "
              f"{tot['rate_hz']:.0f} Hz, lost {tot['lost']}, IK fail {tot['ik_fail']}")
        if r.status != "END":
            print("   WARNING: run did not finish, metrics of the missing segments are left out")

    rows = build_table(all_steps, all_tracks, all_holds, all_tot, ctrls, mm, all_dists)
    write_tables(rows, ctrls, args.out, runs, mm)

    plot_overview(runs, mm, args.out, plt)
    plot_steps(all_steps, ctrls, args.out, plt)
    plot_step_directions(all_steps, ctrls, args.out, plt)
    plot_xy(runs, ctrls, mm, args.out, plt)
    plot_tracking_error(runs, mm, args.out, plt)
    plot_tilt(runs, args.out, plt)
    plot_bars(rows, ctrls, args.out, plt)
    plot_drift(all_tot, ctrls, mm, args.out, plt)
    plot_freq(all_tracks, ctrls, args.out, plt)
    plot_dist(all_dists, ctrls, mm, args.out, plt)
    plot_dist_time(runs, ctrls, mm, args.out, plt)
    print(f"\nWrote {os.path.join(args.out, 'metrics.md')}, metrics.csv and fig_*.png")
    with open(os.path.join(args.out, "metrics.md"), encoding="utf-8") as f:
        print(f.read())
    if not args.no_show:
        plt.show()


# ============================================================================ main
def cmd_live(args):
    """Interactive: servos on, chosen controller balances at the centre, then every line you
    type goes to the robot (s x y / x y / c / o R T / p R V / m 0-3 / ? ...). Nothing is logged."""
    import threading
    import serial  # pyserial
    port = args.port or find_port()
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as ex:
        sys.exit(f"Cannot open {port}: {ex}\nClose the serial monitor / other scripts first.")
    time.sleep(1.0)
    ser.reset_input_buffer()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                line = read_line(ser)
            except Exception:
                print("\n[serial connection lost]")
                stop.set()
                return
            if line and line[0] == "#":
                print("  [robot]", line[2:])

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    send(ser, "x")                     # abort a test that may still be running
    time.sleep(0.2)
    send(ser, f"m {CTRL_ID[args.ctrl]}")
    time.sleep(0.2)
    send(ser, "e")                     # servos on, home, balance at the centre
    print(f"Port {port}: {args.ctrl.upper()} balancing at the centre. Type commands, e.g.\n"
          "  s 50 50  or  50 50   point (px, max 80 from the centre)\n"
          "  c                    centre (stops circle / star)\n"
          "  o 60 3   /  o        circle R=60 px, 3 s per turn / circle on-off\n"
          "  p 60 60  /  p        star R=60 px, 60 px/s / star on-off\n"
          "  m 0..3               switch controller (PID, LQR, KF-PID, KF-LQR)\n"
          "  e / q / ?            servos on / off / settings\n"
          "  exit (or Ctrl+C)     servos off and quit\n"
          "Keep the ball on the plate: the servos switch off after 20 s without a ball (send e).")
    try:
        while not stop.is_set():
            text = input().strip()
            if text.lower() in ("exit", "quit"):
                break
            if text:
                send(ser, text)
    except (KeyboardInterrupt, EOFError):
        pass
    stop.set()
    try:
        send(ser, "q")
        time.sleep(0.2)
        ser.close()
    except Exception:
        pass
    print("Servos off, bye.")


def cmd_ports(args):
    ports = list_ports()
    if not ports:
        print("No serial ports found.")
    for d, desc, t in ports:
        print(f"{d:24s} {desc}{'   <- Teensy' if t else ''}")


def main():
    # Windows consoles may not be UTF-8: never crash on a ± or ° in the output
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    p = sub.add_parser("run", help="run the scripted test on the robot, save the logs and the report")
    p.add_argument("port", nargs="?", default=None, help="COM5 on Windows, /dev/cu.usbmodemXXXX on macOS; default: found by the Teensy USB id")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--ctrl", default="all",
                   help="all (pid,kf,lqr,kflqr), both (pid,lqr), one of pid/pidzn/kf/lqr/kflqr, or a list like pid,pidzn "
                        "(pidzn = PID with the Ziegler-Nichols gains)")
    p.add_argument("--repeat", type=int, default=4,
                   help="blocks = runs per controller (each block: every controller once, random order)")
    p.add_argument("--seed", type=int, default=None, help="master seed of the random order (default: from the clock)")
    p.add_argument("--no-dist", action="store_true", help="leave out the disturbance part (~25 s shorter per run)")
    p.add_argument("--dirs", type=int, default=4, help="number of edge points (1..12, 8 = old long test)")
    p.add_argument("--pause", type=float, default=3.0, help="s before each run (ball settles)")
    p.add_argument("--wait-ball", type=float, default=60.0, help="s to wait for the ball before a run is skipped")
    p.add_argument("--retries", type=int, default=1, help="extra attempts when a run aborts (ball lost)")
    p.add_argument("--no-analyze", action="store_true", help="only record, no report")
    p.add_argument("--kf", default="", help='Kalman settings "qa r bu" sent before the test, e.g. "150 1 0" = tilt input off')
    p.add_argument("--mm-per-px", type=float, default=1.045,
                   help="mm per camera px (corner nuts: 170 mm square = ~115 px radius)")
    p.add_argument("--out", default="logs", help="parent folder; each test gets logs/<time>/")
    p.add_argument("--tag", default="", help="text added to the folder / file names, e.g. kp009")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("analyze", help="compute metrics and plots from logs")
    p.add_argument("files", nargs="+")
    p.add_argument("--out", default=None, help="folder for the report (default: report_<time>, never overwrites)")
    p.add_argument("--mm-per-px", type=float, default=1.045,
                   help="mm per camera px (corner nuts: 170 mm square = ~115 px radius)")
    p.add_argument("--no-show", action="store_true", help="only save the figures")
    p.set_defaults(func=cmd_analyze)

    p = sub.add_parser("live", help="balance with one controller and type set point commands (s/c/o/p), no logging")
    p.add_argument("port", nargs="?", default=None)
    p.add_argument("--ctrl", default="lqr", choices=sorted(CTRL_ID), help="controller to start with (default lqr)")
    p.add_argument("--baud", type=int, default=115200)
    p.set_defaults(func=cmd_live)

    p = sub.add_parser("ports", help="list the serial ports (find the Teensy's COM port)")
    p.set_defaults(func=cmd_ports)

    argv = sys.argv[1:]
    if not argv or argv[0] not in ("run", "analyze", "live", "ports", "-h", "--help"):
        argv = ["run"] + argv  # plain "python3 compare_controllers.py" = run everything
    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
