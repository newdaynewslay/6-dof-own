# PID vs KF-PID vs LQR vs KF-LQR - scripted test metrics

Scale: 1.045 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261119_150440_full_n04_b1_pid` (PID, END), `20261119_150440_full_n07_b2_pid` (PID, END), `20261119_150440_full_n09_b3_pid` (PID, END), `20261119_150440_full_n14_b4_pid` (PID, END), `20261119_150440_full_n03_b1_kf` (KF-PID, END), `20261119_150440_full_n05_b2_kf` (KF-PID, END), `20261119_150440_full_n10_b3_kf` (KF-PID, END), `20261119_150440_full_n16_b4_kf` (KF-PID, END), `20261119_150440_full_n01_b1_lqr` (LQR, END), `20261119_150440_full_n06_b2_lqr` (LQR, END), `20261119_150440_full_n12_b3_lqr` (LQR, END), `20261119_150440_full_n15_b4_lqr` (LQR, END), `20261119_150440_full_n02_b1_kflqr` (KF-LQR, END), `20261119_150440_full_n08_b2_kflqr` (KF-LQR, END), `20261119_150440_full_n11_b3_kflqr` (KF-LQR, END), `20261119_150440_full_n13_b4_kflqr` (KF-LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % (of final value) | s | 1.32 ± 0.19 (16) | 1.21 ± 0.57 (14) | 1.13 ± 0.36 (16) | 1.06 ± 0.48 (16) |
| Steps reaching 90 % of target | % | 81.2 ± 40 (16) | 31.2 ± 48 (16) | 81.2 ± 40 (16) | 31.2 ± 48 (16) |
| Final position (of step size) | % | 86.7 ± 8.6 (16) | 59.8 ± 29 (16) | 90.1 ± 11 (16) | 67.2 ± 21 (16) |
| Overshoot | % | 1.5 ± 2.9 (16) | 1 ± 4 (16) | 2.37 ± 5.4 (16) | 0 ± 0 (16) |
| Settling time (5 % / 3 px band) | s | 2.84 ± 0.18 (9) | 2.87 ± 0.15 (2) | 2.56 ± 0.62 (6) | - |
| Steady-state error | mm | 8.22 ± 4 (16) | 22.1 ± 14 (16) | 7.17 ± 3.9 (16) | 17.8 ± 11 (16) |
| Max deviation from straight path | mm | 7.71 ± 2.2 (16) | 9.67 ± 2.5 (16) | 6.07 ± 1.7 (16) | 8.88 ± 1.5 (16) |
| IAE | mm*s | 53.3 ± 13 (16) | 84.4 ± 34 (16) | 44.9 ± 14 (16) | 72.4 ± 27 (16) |
| Peak tilt | deg | 5.23 ± 0.5 (16) | 4.99 ± 0.64 (16) | 6.42 ± 0.53 (16) | 6.16 ± 0.36 (16) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % (of final value) | s | 0.464 ± 0.24 (16) | 0.439 ± 0.13 (16) | 0.419 ± 0.15 (16) | 0.425 ± 0.086 (16) |
| Steps reaching 90 % of target | % | 100 ± 0 (16) | 100 ± 0 (16) | 100 ± 0 (16) | 93.8 ± 25 (16) |
| Final position (of step size) | % | 100 ± 7.4 (16) | 98.8 ± 12 (16) | 101 ± 5.3 (16) | 98.3 ± 9.8 (16) |
| Overshoot | % | 6.63 ± 6.3 (16) | 9.75 ± 11 (16) | 6.38 ± 5.3 (16) | 9.87 ± 9.7 (16) |
| Settling time (5 % / 3 px band) | s | 2.82 ± 0.24 (5) | - | 2.72 ± 0.42 (9) | 2.97 |
| Steady-state error | mm | 5.22 ± 1.1 (16) | 7.83 ± 1.2 (16) | 3.76 ± 1.1 (16) | 6.25 ± 1.9 (16) |
| Max deviation from straight path | mm | 7.71 ± 1.9 (16) | 10.1 ± 1.9 (16) | 7.12 ± 2.4 (16) | 7.58 ± 2 (16) |
| IAE | mm*s | 32.7 ± 4.3 (16) | 34.9 ± 9.6 (16) | 28.9 ± 2.7 (16) | 30.3 ± 6.7 (16) |
| Peak tilt | deg | 5.58 ± 0.63 (16) | 5.07 ± 1.5 (16) | 7.07 ± 0.73 (16) | 6.34 ± 1.3 (16) |

### Circle R63 mm, 4 s/turn (0.25 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 9.49 ± 3.9 (4) | 23 ± 11 (4) | 8.65 ± 4.1 (4) | 19 ± 8.2 (4) |
| Max tracking error | mm | 14.6 ± 5.4 (4) | 32.3 ± 13 (4) | 13.2 ± 4.5 (4) | 26 ± 10 (4) |
| IAE | mm*s | 35.6 ± 15 (4) | 87.5 ± 45 (4) | 32.8 ± 17 (4) | 73.6 ± 33 (4) |
| Mean radius error (ball - ref) | mm | -6.58 ± 5.8 (4) | -19 ± 12 (4) | -4.86 ± 7.5 (4) | -16.8 ± 7.6 (4) |
| Amplitude ratio (ball / ref, sine fit) | dB | -1.02 ± 0.88 (4) | -3.53 ± 2.6 (4) | -0.769 ± 1.1 (4) | -2.88 ± 1.4 (4) |
| Phase (sine fit, negative = lag) | deg | 0.608 ± 1.9 (4) | 2.34 ± 2.2 (4) | 0.0509 ± 1.6 (4) | 2.97 ± 2.4 (4) |
| Phase lag | deg | -0.602 ± 1.9 (4) | -2.68 ± 2.5 (4) | -0.0208 ± 1.6 (4) | -2.8 ± 2.4 (4) |
| Time lag | s | -0.00669 ± 0.021 (4) | -0.0298 ± 0.027 (4) | -0.000231 ± 0.018 (4) | -0.0311 ± 0.026 (4) |
| RMS tilt | deg | 1.74 ± 0.23 (4) | 1.81 ± 0.43 (4) | 1.78 ± 0.34 (4) | 1.69 ± 0.29 (4) |
| Tilt rate (servo activity) | deg/s | 10.8 ± 1.1 (4) | 10.3 ± 1.5 (4) | 12.8 ± 0.99 (4) | 10 ± 1.4 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) |

### Circle R42 mm, 2 s/turn (0.50 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 12.9 ± 3.2 (4) | 12.4 ± 2.3 (4) | 12.4 ± 5.6 (4) | 7.36 ± 3.9 (4) |
| Max tracking error | mm | 18.1 ± 2.6 (4) | 20.3 ± 2.5 (4) | 18.4 ± 4.5 (4) | 12.7 ± 5.1 (4) |
| IAE | mm*s | 50.9 ± 13 (4) | 47.2 ± 9.5 (4) | 48.3 ± 24 (4) | 27.6 ± 16 (4) |
| Mean radius error (ball - ref) | mm | 8.28 ± 2.2 (4) | 1.73 ± 7.3 (4) | 8.37 ± 4.5 (4) | 1.58 ± 4.3 (4) |
| Amplitude ratio (ball / ref, sine fit) | dB | 1.55 ± 0.38 (4) | 0.199 ± 1.6 (4) | 1.54 ± 0.78 (4) | 0.253 ± 0.85 (4) |
| Phase (sine fit, negative = lag) | deg | -11.8 ± 3 (4) | -2.4 ± 13 (4) | -10.1 ± 4.8 (4) | -1.18 ± 7.2 (4) |
| Phase lag | deg | 11.8 ± 3 (4) | 2.31 ± 13 (4) | 10.1 ± 4.8 (4) | 1.15 ± 7.3 (4) |
| Time lag | s | 0.0653 ± 0.016 (4) | 0.0128 ± 0.073 (4) | 0.0559 ± 0.027 (4) | 0.00639 ± 0.04 (4) |
| RMS tilt | deg | 2.61 ± 0.64 (4) | 2.37 ± 0.94 (4) | 2.89 ± 1 (4) | 2.12 ± 0.83 (4) |
| Tilt rate (servo activity) | deg/s | 11.5 ± 0.62 (4) | 12.8 ± 6 (4) | 13.3 ± 0.73 (4) | 13.8 ± 4.4 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) |

### Circle R42 mm, 6 s/turn (0.17 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 12.3 ± 3.3 (4) | 20.9 ± 7.4 (4) | 9.91 ± 4.8 (4) | 17.5 ± 5.7 (4) |
| Max tracking error | mm | 17 ± 2.7 (4) | 26.2 ± 8.3 (4) | 14.2 ± 5.6 (4) | 23.5 ± 5.7 (4) |
| IAE | mm*s | 71.7 ± 20 (4) | 124 ± 45 (4) | 57.8 ± 29 (4) | 103 ± 36 (4) |
| Mean radius error (ball - ref) | mm | -10.7 ± 3.6 (4) | -19.7 ± 7.6 (4) | -8.62 ± 5.4 (4) | -16.2 ± 6.5 (4) |
| Amplitude ratio (ball / ref, sine fit) | dB | -2.67 ± 1 (4) | -6.06 ± 3.1 (4) | -2.11 ± 1.4 (4) | -4.53 ± 2.1 (4) |
| Phase (sine fit, negative = lag) | deg | -6.03 ± 1.1 (4) | -7.09 ± 0.96 (4) | -4.2 ± 1.9 (4) | -4.31 ± 0.97 (4) |
| Phase lag | deg | 6.12 ± 1.1 (4) | 7.22 ± 1.1 (4) | 4.29 ± 2 (4) | 4.35 ± 1.1 (4) |
| Time lag | s | 0.102 ± 0.019 (4) | 0.12 ± 0.019 (4) | 0.0716 ± 0.033 (4) | 0.0726 ± 0.018 (4) |
| RMS tilt | deg | 1.91 ± 0.28 (4) | 1.79 ± 0.54 (4) | 1.87 ± 0.42 (4) | 1.77 ± 0.26 (4) |
| Tilt rate (servo activity) | deg/s | 10.1 ± 0.68 (4) | 4.32 ± 0.22 (4) | 10.6 ± 0.51 (4) | 4.42 ± 0.77 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 14 ± 2.1 (4) | 20.7 ± 5.8 (4) | 12.2 ± 2.4 (4) | 17.5 ± 4.1 (4) |
| Max tracking error | mm | 29.1 ± 4.2 (4) | 39 ± 9.5 (4) | 25 ± 6.1 (4) | 34.2 ± 7.9 (4) |
| IAE | mm*s | 121 ± 17 (4) | 180 ± 53 (4) | 105 ± 20 (4) | 153 ± 37 (4) |
| RMS tilt | deg | 2.83 ± 0.19 (4) | 2.22 ± 0.26 (4) | 3.06 ± 0.18 (4) | 2.23 ± 0.21 (4) |
| Tilt rate (servo activity) | deg/s | 15.3 ± 0.9 (4) | 10.5 ± 0.36 (4) | 17.2 ± 0.8 (4) | 11 ± 0.21 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) |

### Centre hold ('hold', 5 s after settle)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS error | mm | 4.9 ± 0.82 (4) | 6.88 ± 0.64 (4) | 3.91 ± 0.6 (4) | 6.1 ± 1 (4) |
| Max error | mm | 8.79 ± 1.9 (4) | 10.4 ± 1.2 (4) | 7 ± 1.5 (4) | 9.23 ± 2 (4) |
| RMS tilt | deg | 1.63 ± 0.3 (4) | 1.29 ± 0.089 (4) | 1.45 ± 0.17 (4) | 1.2 ± 0.19 (4) |

### Disturbance: tilt kick (pulse, recovery = back inside 5 px)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Kick size | deg | 3 ± 0 (16) | 3 ± 0 (16) | 3 ± 0 (16) | 3 ± 0 (16) |
| Peak deviation | mm | 12.7 ± 3 (16) | 16.1 ± 4.3 (16) | 12 ± 3.4 (16) | 17.5 ± 4 (16) |
| Time to peak | s | 0.61 ± 0.31 (16) | 0.77 ± 0.41 (16) | 0.588 ± 0.38 (16) | 0.717 ± 0.38 (16) |
| Recovery time (after the pulse) | s | 1.76 ± 0.47 (15) | 2.58 ± 0.24 (3) | 2.07 ± 0.55 (16) | 2.74 ± 0.04 (4) |
| IAE | mm*s | 16.2 ± 1.9 (16) | 25.2 ± 2.4 (16) | 14.7 ± 1.6 (16) | 26.6 ± 3.8 (16) |
| Peak tilt | deg | 3.78 ± 0.74 (16) | 3.32 ± 0.39 (16) | 4.13 ± 0.93 (16) | 3.58 ± 0.5 (16) |

### Disturbance: constant tilt (push on, then release)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Push size | deg | 1 ± 0 (4) | 1 ± 0 (4) | 1 ± 0 (4) | 1 ± 0 (4) |
| Peak deviation (push) | mm | 6.07 ± 0.87 (4) | 9.73 ± 1.9 (4) | 6.78 ± 0.56 (4) | 10.4 ± 2.4 (4) |
| Recovery time (push) | s | 2.41 ± 1.1 (4) | 2.55 ± 0.68 (4) | 2.48 ± 1.7 (4) | 3.94 ± 1.3 (4) |
| Remaining offset (last 1.5 s of push) | mm | 2.06 ± 0.28 (4) | 2.72 ± 0.42 (4) | 1.84 ± 0.68 (4) | 2.88 ± 1.3 (4) |
| IAE (push) | mm*s | 14.1 ± 2.3 (4) | 23.3 ± 3.8 (4) | 14.5 ± 2.2 (4) | 25.6 ± 5.4 (4) |
| Peak deviation (release) | mm | 7.6 ± 0.94 (4) | 11.9 ± 0.6 (4) | 7.23 ± 1 (4) | 12.3 ± 2.4 (4) |
| Recovery time (release) | s | 2.55 ± 0.44 (4) | 2.81 ± 0.29 (2) | 1.78 ± 1.1 (3) | 2.82 ± 0.15 (2) |

### Whole test

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Total IAE | mm*s | 945 ± 87 (4) | 1.38e+03 ± 3.1e+02 (4) | 834 ± 1.2e+02 (4) | 1.17e+03 ± 2.2e+02 (4) |
| RMS tilt | deg | 2.51 ± 0.13 (4) | 2.09 ± 0.14 (4) | 2.78 ± 0.17 (4) | 2.14 ± 0.14 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) |
| Control rate | Hz | 61.6 ± 0.00038 (4) | 61.6 ± 0.00033 (4) | 61.6 ± 0.00038 (4) | 61.6 ± 0.00033 (4) |
| Ball-lost samples | - | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (4) |
