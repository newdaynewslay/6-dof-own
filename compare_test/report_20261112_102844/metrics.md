# PID vs KF-PID vs LQR - scripted test metrics

Scale: 1.09 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261112_102844_pid` (PID, END), `20261112_102844_kf` (KF-PID, END), `20261112_102844_lqr` (LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| Rise time 10-90 % | s | 1.41 ± 0.52 (3) | - | - |
| Overshoot | % | 6.07 ± 12 (4) | 0 ± 0 (4) | 0 ± 0 (4) |
| Settling time (5 % / 3 px band) | s | - | - | - |
| Steady-state error | mm | 13.9 ± 4.8 (4) | 27 ± 5.7 (4) | 28.4 ± 3.1 (4) |
| Max deviation from straight path | mm | 12.3 ± 3.8 (4) | 11.4 ± 4.2 (4) | 12.3 ± 3.5 (4) |
| IAE | mm*s | 68 ± 13 (4) | 102 ± 12 (4) | 117 ± 12 (4) |
| Peak tilt | deg | 6.56 ± 0.73 (4) | 6.21 ± 0.2 (4) | 7.3 ± 1.9 (4) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| Rise time 10-90 % | s | 0.933 ± 0.33 (4) | 0.69 ± 0.11 (4) | 1.36 ± 0.31 (3) |
| Overshoot | % | 3.21 ± 3.8 (4) | 3.93 ± 6.9 (4) | 2.5 ± 5 (4) |
| Settling time (5 % / 3 px band) | s | 2.53 | - | - |
| Steady-state error | mm | 8.28 ± 3.5 (4) | 11.2 ± 4.5 (4) | 10.6 ± 5.1 (4) |
| Max deviation from straight path | mm | 12 ± 5.8 (4) | 9.81 ± 2.8 (4) | 12 ± 3.9 (4) |
| IAE | mm*s | 54.7 ± 17 (4) | 45.3 ± 9.8 (4) | 56.4 ± 12 (4) |
| Peak tilt | deg | 7.17 ± 0.88 (4) | 5.06 ± 0.42 (4) | 6.71 ± 0.58 (4) |

### Fast circle

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| RMS tracking error | mm | 11.3 | 12.4 | 15.5 |
| Max tracking error | mm | 25.2 | 31.1 | 39.4 |
| IAE | mm*s | 78.7 | 87.1 | 103 |
| Mean radius error (ball - ref) | mm | 2.9 | -4.12 | -0.548 |
| Phase lag | deg | 4.53 | -1.23 | 2.79 |
| Time lag | s | 0.0503 | -0.0137 | 0.0309 |
| RMS tilt | deg | 2.38 | 1.23 | 6.85 |
| Tilt rate (servo activity) | deg/s | 38.3 | 15.2 | 63.8 |
| Time at tilt limit | % | 0 | 0 | 5.49 |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| RMS tracking error | mm | 11.6 | 15.8 | 10.9 |
| Max tracking error | mm | 33.7 | 33.7 | 19.8 |
| IAE | mm*s | 96.8 | 135 | 95.2 |
| RMS tilt | deg | 2.55 | 1.89 | 4.53 |
| Tilt rate (servo activity) | deg/s | 26.9 | 10.3 | 41.9 |
| Time at tilt limit | % | 0 | 0 | 0 |

### Centre hold (last 3 s of 'end')

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| RMS error | mm | 11.4 | 11.1 | 10.9 |
| RMS tilt | deg | 1.14 | 0.83 | 2.75 |

### Whole test

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| Total IAE | mm*s | 859 | 1.04e+03 | 1.13e+03 |
| RMS tilt | deg | 2.48 | 1.65 | 4.22 |
| Time at tilt limit | % | 0 | 0 | 0.851 |
| Control rate | Hz | 61.6 | 61.6 | 61.6 |
| Ball-lost samples | - | 0 | 0 | 0 |
