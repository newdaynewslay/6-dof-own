# PID vs KF-PID vs LQR - scripted test metrics

Scale: 1.09 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261112_105005_pid` (PID, END), `20261112_105005_kf` (KF-PID, END), `20261112_105005_lqr` (LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| Rise time 10-90 % | s | 0.442 ± 0.051 (4) | 0.963 ± 0.26 (3) | 0.422 ± 0.11 (4) |
| Overshoot | % | 28.2 ± 5.9 (4) | 8.21 ± 12 (4) | 12.9 ± 5.6 (4) |
| Settling time (5 % / 3 px band) | s | 2.99 | - | 2.99 |
| Steady-state error | mm | 12.5 ± 2.3 (4) | 12.2 ± 4.8 (4) | 5.07 ± 0.64 (4) |
| Max deviation from straight path | mm | 14.7 ± 2.9 (4) | 16.1 ± 4.5 (4) | 13.4 ± 3.8 (4) |
| IAE | mm*s | 63.8 ± 7.4 (4) | 68.4 ± 11 (4) | 46.1 ± 3.7 (4) |
| Peak tilt | deg | 9 ± 1.7 (4) | 7.22 ± 0.47 (4) | 9.93 ± 0.81 (4) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| Rise time 10-90 % | s | 0.358 ± 0.033 (4) | 0.426 ± 0.11 (4) | 0.272 ± 0.02 (4) |
| Overshoot | % | 21.1 ± 11 (4) | 26.1 ± 8.4 (4) | 18.2 ± 4.7 (4) |
| Settling time (5 % / 3 px band) | s | - | - | 3 |
| Steady-state error | mm | 13.4 ± 6 (4) | 10.2 ± 4 (4) | 9.14 ± 1.5 (4) |
| Max deviation from straight path | mm | 11.7 ± 2.9 (4) | 7.63 ± 3.2 (4) | 11.2 ± 2.1 (4) |
| IAE | mm*s | 64.2 ± 14 (4) | 63 ± 20 (4) | 52.4 ± 6.5 (4) |
| Peak tilt | deg | 9.78 ± 2 (4) | 8.88 ± 1.1 (4) | 12.5 ± 1.3 (4) |

### Fast circle

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| RMS tracking error | mm | 14 | 12.8 | 8.75 |
| Max tracking error | mm | 34.8 | 25.2 | 20.6 |
| IAE | mm*s | 98.2 | 95.4 | 58.4 |
| Mean radius error (ball - ref) | mm | 8.01 | -8.55 | -3.16 |
| Phase lag | deg | 6.02 | 1.3 | 2 |
| Time lag | s | 0.0669 | 0.0144 | 0.0222 |
| RMS tilt | deg | 2.47 | 1.16 | 2.1 |
| Tilt rate (servo activity) | deg/s | 34.4 | 14 | 28.3 |
| Time at tilt limit | % | 0 | 0 | 0 |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| RMS tracking error | mm | 10.2 | 16.9 | 12.2 |
| Max tracking error | mm | 21.7 | 33.7 | 26.1 |
| IAE | mm*s | 86.4 | 149 | 104 |
| RMS tilt | deg | 2.21 | 2 | 2.85 |
| Tilt rate (servo activity) | deg/s | 26 | 12.7 | 25 |
| Time at tilt limit | % | 0 | 0 | 0 |

### Centre hold (last 3 s of 'end')

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| RMS error | mm | 5.66 | 4.54 | 4.41 |
| RMS tilt | deg | 1.05 | 0.724 | 1.68 |

### Whole test

| Metric | Unit | PID | KF-PID | LQR |
|---|---|---|---|---|
| Total IAE | mm*s | 858 | 911 | 683 |
| RMS tilt | deg | 2.84 | 2.08 | 3.47 |
| Time at tilt limit | % | 0 | 0 | 0 |
| Control rate | Hz | 61.6 | 61.6 | 61.6 |
| Ball-lost samples | - | 0 | 0 | 0 |
