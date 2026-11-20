# Controller gain log (Ball_Balance_Compare)

Units: tilt in degrees, position in camera px (~1.045 mm/px), time in s.
Every test folder `logs/<time>_<tag>/` also has `gains.txt` with the gains the robot
reported at the start, and every CSV header repeats them (`# PID ...`, `# PIDZN ...`).

| Date | Controller | kp | kd | ki | Source / note |
|---|---|---|---|---|---|
| 2026-11-12 | PID (old) | 0.09 | 0.05 | 0 | first compare runs |
| 2026-11-12 | PID / KF-PID | 0.10 | 0.055 | 0.08 | retuned from the 10:28 logs (current "PID", `m 0`) |
| 2026-11-12 | LQR / KF-LQR | K = [0.123, 0.058] | | 0.08 | q 1 / 0.05, r 60, model c ~100 px/s^2/deg |
| 2026-11-19 | PID-ZN (`m 4`, `--ctrl pidzn`) | 0.0132 | 0.0045 | 0.0718 | Ziegler-Nichols, calculated by hand by the group |

Quick check of PID-ZN vs PID (2026-11-19): `py compare_controllers.py run --ctrl pid,pidzn --repeat 2 --no-dist --tag zn_quick`
-> results in `logs/<time>_zn_quick/`.
