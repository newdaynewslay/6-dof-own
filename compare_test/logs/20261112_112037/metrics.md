# PID vs KF-PID vs LQR vs KF-LQR - scripted test metrics

Scale: 1.045 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261112_112037_pid` (PID, END), `20261112_112037_kf` (KF-PID, END), `20261112_112037_lqr` (LQR, END), `20261112_112037_kflqr` (KF-LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % | s | 1.24 ± 0.21 (4) | 2.37 ± 0.092 (2) | 1.46 ± 0.38 (4) | 2.32 |
| Overshoot | % | 1.15 ± 0.77 (4) | 0 ± 0 (4) | 4.23 ± 4.8 (4) | 0 ± 0 (4) |
| Settling time (5 % / 3 px band) | s | 2.57 ± 0.49 (3) | - | 3 | - |
| Steady-state error | mm | 5.23 ± 1.9 (4) | 20.1 ± 11 (4) | 7.34 ± 1.5 (4) | 23.4 ± 12 (4) |
| Max deviation from straight path | mm | 9.93 ± 2.5 (4) | 10.7 ± 3.2 (4) | 8.1 ± 1 (4) | 7.84 ± 1.8 (4) |
| IAE | mm*s | 52.4 ± 5.4 (4) | 90.1 ± 22 (4) | 52.7 ± 3.7 (4) | 94.3 ± 26 (4) |
| Peak tilt | deg | 7.11 ± 1 (4) | 6.52 ± 0.68 (4) | 8.38 ± 0.72 (4) | 7.8 ± 0.13 (4) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % | s | 0.463 ± 0.021 (4) | 0.556 ± 0.068 (4) | 0.385 ± 0.054 (4) | 0.52 ± 0.078 (4) |
| Overshoot | % | 4.23 ± 2.6 (4) | 7.69 ± 7 (4) | 5.38 ± 2.7 (4) | 7.31 ± 4.9 (4) |
| Settling time (5 % / 3 px band) | s | 2.67 ± 0.52 (4) | - | 2.79 ± 0.19 (3) | 2.83 ± 0.13 (3) |
| Steady-state error | mm | 2.59 ± 0.6 (4) | 4.32 ± 1.8 (4) | 3.02 ± 0.54 (4) | 3.64 ± 0.86 (4) |
| Max deviation from straight path | mm | 6.53 ± 1.8 (4) | 8.88 ± 3.6 (4) | 7.05 ± 2.2 (4) | 8.88 ± 1.3 (4) |
| IAE | mm*s | 36.7 ± 1.5 (4) | 37.1 ± 2.2 (4) | 33.6 ± 1.1 (4) | 29.8 ± 7.1 (4) |
| Peak tilt | deg | 7.7 ± 0.38 (4) | 7.72 ± 0.56 (4) | 8.59 ± 2.6 (4) | 7.94 ± 1.2 (4) |

### Fast circle

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 9.76 | 12.5 | 8.26 | 21.7 |
| Max tracking error | mm | 24.4 | 20.6 | 17.6 | 36 |
| IAE | mm*s | 62.4 | 94.2 | 60.3 | 167 |
| Mean radius error (ball - ref) | mm | 0.683 | -8.8 | -4.73 | -19.5 |
| Phase lag | deg | 3.73 | 1.01 | 1.66 | -2.06 |
| Time lag | s | 0.0414 | 0.0112 | 0.0184 | -0.0229 |
| RMS tilt | deg | 1.83 | 1.23 | 2.11 | 1.6 |
| Tilt rate (servo activity) | deg/s | 28.2 | 13.5 | 29.8 | 11.6 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 10.8 | 17 | 11.5 | 19.3 |
| Max tracking error | mm | 22.7 | 36 | 26 | 34.1 |
| IAE | mm*s | 91.3 | 140 | 97.7 | 169 |
| RMS tilt | deg | 2.45 | 2.05 | 2.82 | 2.21 |
| Tilt rate (servo activity) | deg/s | 23.7 | 10.9 | 24.5 | 12 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Centre hold (last 3 s of 'end')

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS error | mm | 3.61 | 3.13 | 4.35 | 5.02 |
| RMS tilt | deg | 1.15 | 0.506 | 1.66 | 1.22 |

### Whole test

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Total IAE | mm*s | 653 | 898 | 632 | 975 |
| RMS tilt | deg | 2.45 | 1.98 | 2.99 | 2.29 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |
| Control rate | Hz | 61.6 | 61.6 | 61.6 | 61.6 |
| Ball-lost samples | - | 0 | 0 | 0 | 0 |
