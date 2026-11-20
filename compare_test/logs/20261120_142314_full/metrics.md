# PID vs KF-PID vs LQR vs KF-LQR - scripted test metrics

Scale: 1.045 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261120_142314_full_n02_b1_pid` (PID, END), `20261120_142314_full_n03_b1_kf` (KF-PID, END), `20261120_142314_full_n01_b1_lqr` (LQR, END), `20261120_142314_full_n04_b1_kflqr` (KF-LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % (of final value) | s | 1.25 ± 0.19 (4) | 1.19 ± 0.63 (4) | 0.897 ± 0.37 (4) | 1.3 ± 0.52 (4) |
| Steps reaching 90 % of target | % | 100 ± 0 (4) | 75 ± 50 (4) | 100 ± 0 (4) | 50 ± 58 (4) |
| Final position (of step size) | % | 97.6 ± 5.7 (4) | 79.1 ± 15 (4) | 96.9 ± 5 (4) | 82 ± 16 (4) |
| Overshoot | % | 2.5 ± 3 (4) | 0 ± 0 (4) | 4 ± 3.3 (4) | 2.5 ± 5 (4) |
| Settling time (5 % / 3 px band) | s | 2.4 ± 0.78 (2) | 3.02 | 2.48 ± 0.75 (2) | - |
| Steady-state error | mm | 3.93 ± 1.6 (4) | 11.7 ± 8.2 (4) | 3.96 ± 1.5 (4) | 10.9 ± 7.1 (4) |
| Max deviation from straight path | mm | 7.31 ± 2.8 (4) | 8.88 ± 3.2 (4) | 6.79 ± 0.6 (4) | 8.36 ± 1.9 (4) |
| IAE | mm*s | 41.4 ± 7.7 (4) | 61.2 ± 29 (4) | 35.7 ± 6.5 (4) | 55.9 ± 24 (4) |
| Peak tilt | deg | 5.45 ± 0.21 (4) | 4.78 ± 0.71 (4) | 6.91 ± 0.5 (4) | 6.25 ± 0.62 (4) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % (of final value) | s | 0.601 ± 0.3 (4) | 0.593 ± 0.14 (4) | 0.552 ± 0.33 (4) | 0.471 ± 0.076 (4) |
| Steps reaching 90 % of target | % | 100 ± 0 (4) | 100 ± 0 (4) | 100 ± 0 (4) | 75 ± 50 (4) |
| Final position (of step size) | % | 102 ± 7 (4) | 98.8 ± 12 (4) | 102 ± 7.8 (4) | 98.3 ± 13 (4) |
| Overshoot | % | 7 ± 6.2 (4) | 12 ± 14 (4) | 9 ± 8.2 (4) | 13.5 ± 16 (4) |
| Settling time (5 % / 3 px band) | s | - | 2.85 | 2.69 ± 0.14 (2) | 2.86 |
| Steady-state error | mm | 4.99 ± 0.71 (4) | 7.11 ± 1 (4) | 5.04 ± 1.2 (4) | 8.04 ± 1.4 (4) |
| Max deviation from straight path | mm | 7.84 ± 1.3 (4) | 10.7 ± 4.2 (4) | 8.62 ± 1.3 (4) | 8.88 ± 1 (4) |
| IAE | mm*s | 34.7 ± 4 (4) | 40.6 ± 3.5 (4) | 34.5 ± 4.2 (4) | 40.3 ± 6.9 (4) |
| Peak tilt | deg | 5.32 ± 0.18 (4) | 5.78 ± 0.52 (4) | 6.25 ± 0.24 (4) | 6.6 ± 0.4 (4) |

### Circle R63 mm, 4 s/turn (0.25 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 6.96 | 14 | 7.68 | 7.37 |
| Max tracking error | mm | 15.8 | 21.5 | 14.2 | 11.8 |
| IAE | mm*s | 25.7 | 53.9 | 28.9 | 27.7 |
| Mean radius error (ball - ref) | mm | 4.73 | -3.96 | 5.26 | -5.7 |
| Amplitude ratio (ball / ref, sine fit) | dB | 0.622 | -0.651 | 0.689 | -0.844 |
| Phase (sine fit, negative = lag) | deg | -2.33 | -0.22 | -2.54 | 0.373 |
| Phase lag | deg | 2.37 | 0.5 | 2.61 | -0.393 |
| Time lag | s | 0.0264 | 0.00556 | 0.029 | -0.00437 |
| RMS tilt | deg | 1.76 | 1.59 | 1.89 | 1.3 |
| Tilt rate (servo activity) | deg/s | 13.9 | 12.3 | 19.6 | 13.6 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Circle R42 mm, 2 s/turn (0.50 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 20.4 | 16.3 | 19.7 | 13.2 |
| Max tracking error | mm | 24.5 | 25.5 | 24 | 20.3 |
| IAE | mm*s | 81.2 | 63.5 | 78.6 | 51.8 |
| Mean radius error (ball - ref) | mm | 13.1 | 8.59 | 13.6 | 7.97 |
| Amplitude ratio (ball / ref, sine fit) | dB | 2.35 | 1.6 | 2.45 | 1.5 |
| Phase (sine fit, negative = lag) | deg | -18.4 | -16.1 | -16.6 | -12.3 |
| Phase lag | deg | 18.4 | 16.1 | 16.6 | 12.4 |
| Time lag | s | 0.102 | 0.0893 | 0.0923 | 0.0687 |
| RMS tilt | deg | 3.94 | 3.52 | 4.27 | 3.33 |
| Tilt rate (servo activity) | deg/s | 13 | 10.1 | 16.1 | 7.9 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Circle R42 mm, 6 s/turn (0.17 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 8.01 | 16 | 6.06 | 14.9 |
| Max tracking error | mm | 13.8 | 28.5 | 14.6 | 23.9 |
| IAE | mm*s | 44.7 | 89.7 | 29.7 | 87.3 |
| Mean radius error (ball - ref) | mm | -5.93 | -12.9 | -2.12 | -13.8 |
| Amplitude ratio (ball / ref, sine fit) | dB | -1.35 | -3.39 | -0.484 | -3.54 |
| Phase (sine fit, negative = lag) | deg | -4.14 | -5.5 | -4.12 | -5.28 |
| Phase lag | deg | 4.29 | 6.07 | 4.35 | 5.42 |
| Time lag | s | 0.0716 | 0.101 | 0.0725 | 0.0903 |
| RMS tilt | deg | 1.74 | 1.61 | 1.69 | 1.51 |
| Tilt rate (servo activity) | deg/s | 13.9 | 6.34 | 11.7 | 4.08 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 10.1 | 13.9 | 10 | 15.9 |
| Max tracking error | mm | 19.3 | 27.6 | 25.4 | 30.8 |
| IAE | mm*s | 86.1 | 119 | 80.4 | 138 |
| RMS tilt | deg | 2.82 | 2.21 | 3.08 | 2.26 |
| Tilt rate (servo activity) | deg/s | 13.1 | 10.6 | 18 | 10.5 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |

### Centre hold ('hold', 5 s after settle)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS error | mm | 4.03 | 5.65 | 6.53 | 6.26 |
| Max error | mm | 7.54 | 8.87 | 13.1 | 11.1 |
| RMS tilt | deg | 1.15 | 1.02 | 1.17 | 1.25 |

### Disturbance: tilt kick (pulse, recovery = back inside 5 px)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Kick size | deg | 3 ± 0 (4) | 3 ± 0 (4) | 3 ± 0 (4) | 3 ± 0 (4) |
| Peak deviation | mm | 11.7 ± 3.9 (4) | 19.8 ± 5.4 (4) | 12.9 ± 3.9 (4) | 15.6 ± 5.8 (4) |
| Time to peak | s | 0.622 ± 0.21 (4) | 1.02 ± 0.67 (4) | 0.524 ± 0.13 (4) | 0.581 ± 0.18 (4) |
| Recovery time (after the pulse) | s | 1.29 ± 0.48 (4) | 2.25 ± 0.34 (4) | 1.76 ± 0.7 (4) | 2.35 ± 0.46 (3) |
| IAE | mm*s | 13.6 ± 1.6 (4) | 27.2 ± 1.7 (4) | 17.2 ± 3.2 (4) | 23.7 ± 5 (4) |
| Peak tilt | deg | 3.78 ± 0.69 (4) | 3.53 ± 0.64 (4) | 4.23 ± 1 (4) | 3.16 ± 0.32 (4) |

### Disturbance: constant tilt (push on, then release)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Push size | deg | 1 | 1 | 1 | 1 |
| Peak deviation (push) | mm | 5.22 | 6.61 | 5.33 | 8.43 |
| Recovery time (push) | s | 0 | 2.55 | 0.686 | 1.42 |
| Remaining offset (last 1.5 s of push) | mm | 1.71 | 2.56 | 2.05 | 2.66 |
| IAE (push) | mm*s | 13.9 | 19.8 | 15.1 | 19.6 |
| Peak deviation (release) | mm | 6.69 | 15.1 | 8.16 | 11.3 |
| Recovery time (release) | s | 1.91 | 2.69 | 1.83 | - |

### Whole test

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Total IAE | mm*s | 834 | 1.09e+03 | 767 | 1.02e+03 |
| RMS tilt | deg | 2.47 | 2 | 2.81 | 2.19 |
| Time at tilt limit | % | 0 | 0 | 0 | 0 |
| Control rate | Hz | 61.6 | 61.6 | 61.6 | 61.6 |
| Ball-lost samples | - | 0 | 0 | 0 | 0 |
