# PID vs KF-PID vs LQR vs KF-LQR - scripted test metrics

Scale: 1.045 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261112_110918_pid` (PID, END), `20261112_110918_kf` (KF-PID, END), `20261112_110918_lqr` (LQR, END), `20261112_110918_kflqr` (KF-LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % | s | 1.45 ± 0.46 (4) | 1.99 ± 0.66 (3) | 0.788 ± 0.41 (4) | 2.03 ± 0.6 (2) |
| Overshoot | % | 5 ± 5.2 (4) | 0 ± 0 (4) | 5 ± 5.5 (4) | 3.08 ± 6.2 (4) |
| Settling time (5 % / 3 px band) | s | 2.97 | - | 2.93 ± 0.0049 (2) | - |
| Steady-state error | mm | 8.38 ± 1.5 (4) | 14.6 ± 8.7 (4) | 5.65 ± 1.5 (4) | 14.3 ± 4 (4) |
| Max deviation from straight path | mm | 10.7 ± 2.9 (4) | 10.2 ± 4.3 (4) | 10.2 ± 1.8 (4) | 10.4 ± 3.5 (4) |
| IAE | mm*s | 56.2 ± 8.2 (4) | 76.8 ± 22 (4) | 45.3 ± 4.7 (4) | 71.1 ± 9.5 (4) |
| Peak tilt | deg | 7 ± 0.73 (4) | 6.64 ± 0.85 (4) | 8.07 ± 0.52 (4) | 8.15 ± 0.16 (4) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % | s | 0.678 ± 0.3 (4) | 0.662 ± 0.043 (4) | 0.682 ± 0.31 (4) | 0.544 ± 0.088 (4) |
| Overshoot | % | 8.08 ± 5.5 (4) | 6.92 ± 3.9 (4) | 5.38 ± 2 (4) | 5 ± 4.4 (4) |
| Settling time (5 % / 3 px band) | s | 1.69 | 2.85 ± 0.1 (3) | 2.57 ± 0.85 (4) | 2.59 ± 0.3 (3) |
| Steady-state error | mm | 4.66 ± 2.8 (4) | 3.59 ± 1.2 (4) | 2.74 ± 1.2 (4) | 3.45 ± 1.7 (4) |
| Max deviation from straight path | mm | 10.4 ± 4.4 (4) | 7.58 ± 1.8 (4) | 9.14 ± 3.1 (4) | 9.93 ± 4.9 (4) |
| IAE | mm*s | 42.9 ± 4.6 (4) | 36.7 ± 5.3 (4) | 35.1 ± 4.4 (4) | 34.2 ± 3.7 (4) |
| Peak tilt | deg | 7.38 ± 0.88 (4) | 7.28 ± 0.085 (4) | 8.6 ± 0.85 (4) | 9.04 ± 0.96 (4) |

### Fast circle

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 7.62 | 10.4 | 8.02 | 15.8 |
| Max tracking error | mm | 15 | 24.3 | 22.1 | 27.1 |
| IAE | mm*s | 54.9 | 69.1 | 50.6 | 121 |
| Mean radius error (ball - ref) | mm | 3.52 | -4.69 | -1.04 | -13.2 |
| Phase lag | deg | 3.63 | 1.69 | 2.53 | -1.25 |
| Time lag | s | 0.0404 | 0.0188 | 0.0281 | -0.0139 |
| RMS tilt | deg | 1.47 | 1.13 | 2.32 | 1.38 |
| Tilt rate (servo activity) | deg/s | 13.1 | 11.1 | 33.9 | 14.7 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 10 | 14 | 10.1 | 15.5 |
| Max tracking error | mm | 18.5 | 29.5 | 20.1 | 30.8 |
| IAE | mm*s | 86.6 | 118 | 87.4 | 134 |
| RMS tilt | deg | 2.52 | 2.09 | 2.9 | 2.09 |
| Tilt rate (servo activity) | deg/s | 17 | 12.7 | 29.6 | 13.2 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Centre hold (last 3 s of 'end')

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS error | mm | 4.22 | 8.4 | 4.23 | 2.27 |
| RMS tilt | deg | 0.932 | 1.44 | 1.56 | 0.684 |

### Whole test

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Total IAE | mm*s | 681 | 816 | 585 | 823 |
| RMS tilt | deg | 2.47 | 1.9 | 2.93 | 2.15 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |
| Control rate | Hz | 61.6 | 61.6 | 61.6 | 61.6 |
| Ball-lost samples | - | 0 | 0 | 0 | 0 |
