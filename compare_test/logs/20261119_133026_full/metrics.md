# PID vs KF-PID vs LQR vs KF-LQR - scripted test metrics

Scale: 1.045 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261119_133026_full_n02_b1_pid` (PID, END), `20261119_133026_full_n08_b2_pid` (PID, END), `20261119_133026_full_n11_b3_pid` (PID, END), `20261119_133026_full_n13_b4_pid` (PID, END), `20261119_133026_full_n01_b1_kf` (KF-PID, END), `20261119_133026_full_n06_b2_kf` (KF-PID, END), `20261119_133026_full_n12_b3_kf` (KF-PID, END), `20261119_133026_full_n15_b4_kf` (KF-PID, END), `20261119_133026_full_n04_b1_lqr` (LQR, END), `20261119_133026_full_n07_b2_lqr` (LQR, END), `20261119_133026_full_n09_b3_lqr` (LQR, END), `20261119_133026_full_n03_b1_kflqr` (KF-LQR, END), `20261119_133026_full_n05_b2_kflqr` (KF-LQR, END), `20261119_133026_full_n10_b3_kflqr` (KF-LQR, END), `20261119_133026_full_n16_b4_kflqr` (KF-LQR, END)


### Step out (centre -> edge)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % (of final value) | s | 0.846 ± 0.61 (16) | 1.01 ± 0.55 (13) | 1 ± 0.6 (12) | 1.19 ± 0.54 (14) |
| Steps reaching 90 % of target | % | 43.8 ± 51 (16) | 50 ± 52 (16) | 83.3 ± 39 (12) | 37.5 ± 50 (16) |
| Final position (of step size) | % | 69 ± 24 (16) | 66.2 ± 38 (16) | 83.4 ± 12 (12) | 64.9 ± 26 (16) |
| Overshoot | % | 2.5 ± 5.5 (16) | 3.88 ± 7.6 (16) | 0.833 ± 2.3 (12) | 1 ± 3.5 (16) |
| Settling time (5 % / 3 px band) | s | 2.97 ± 0.065 (2) | - | 2.99 ± 0.023 (3) | 2.98 |
| Steady-state error | mm | 18.1 ± 11 (16) | 22.6 ± 16 (16) | 10.1 ± 5.2 (12) | 19.8 ± 12 (16) |
| Max deviation from straight path | mm | 13.4 ± 4.3 (16) | 12.5 ± 5.7 (16) | 10.6 ± 3.7 (12) | 10.2 ± 2.2 (16) |
| IAE | mm*s | 74.6 ± 27 (16) | 85.5 ± 36 (16) | 53.9 ± 16 (12) | 77.9 ± 31 (16) |
| Peak tilt | deg | 7.52 ± 2.4 (16) | 5.35 ± 0.89 (16) | 7.83 ± 1.6 (12) | 6.36 ± 0.65 (16) |

### Step back (edge -> centre)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Rise time 10-90 % (of final value) | s | 0.55 ± 0.29 (16) | 0.594 ± 0.42 (16) | 0.361 ± 0.069 (12) | 0.412 ± 0.13 (16) |
| Steps reaching 90 % of target | % | 100 ± 0 (16) | 100 ± 0 (16) | 100 ± 0 (12) | 100 ± 0 (16) |
| Final position (of step size) | % | 102 ± 8.7 (16) | 102 ± 8.9 (16) | 99.1 ± 6.4 (12) | 100 ± 9.8 (16) |
| Overshoot | % | 15.6 ± 11 (16) | 11.7 ± 11 (16) | 11.5 ± 7.7 (12) | 12.5 ± 12 (16) |
| Settling time (5 % / 3 px band) | s | 2.44 ± 0.61 (4) | 2.71 ± 0.32 (4) | 2.91 ± 0.066 (4) | - |
| Steady-state error | mm | 7.29 ± 2.8 (16) | 6.72 ± 3.2 (16) | 5.75 ± 2 (12) | 6.86 ± 1.2 (16) |
| Max deviation from straight path | mm | 10.3 ± 2.9 (16) | 11.5 ± 5.9 (16) | 9.4 ± 1.9 (12) | 9.47 ± 2.8 (16) |
| IAE | mm*s | 35 ± 6.8 (16) | 36.8 ± 17 (16) | 33.1 ± 4.3 (12) | 32.7 ± 7.9 (16) |
| Peak tilt | deg | 6.67 ± 1 (16) | 4.99 ± 1.3 (16) | 7.46 ± 0.76 (12) | 6.25 ± 1.2 (16) |

### Circle R63 mm, 4 s/turn (0.25 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 26.8 ± 10 (4) | 26.6 ± 16 (4) | 13.5 ± 10 (3) | 23.5 ± 18 (4) |
| Max tracking error | mm | 37.2 ± 12 (4) | 40.9 ± 17 (4) | 18.9 ± 11 (3) | 32.7 ± 24 (4) |
| IAE | mm*s | 103 ± 39 (4) | 100 ± 67 (4) | 52.6 ± 41 (3) | 90 ± 71 (4) |
| Mean radius error (ball - ref) | mm | -16.6 ± 21 (4) | -16.4 ± 20 (4) | -10.9 ± 13 (3) | -19.2 ± 15 (4) |
| Amplitude ratio (ball / ref, sine fit) | dB | -3.47 ± 3.7 (4) | -3.97 ± 5.3 (4) | -1.86 ± 2.2 (3) | -4.04 ± 3.9 (4) |
| Phase (sine fit, negative = lag) | deg | 2.32 ± 7.5 (4) | 6.35 ± 14 (4) | 0.829 ± 1.9 (3) | 7.45 ± 8.5 (4) |
| Phase lag | deg | -2.56 ± 7.8 (4) | -6.53 ± 15 (4) | -0.783 ± 2 (3) | -7.79 ± 9.1 (4) |
| Time lag | s | -0.0284 ± 0.086 (4) | -0.0726 ± 0.16 (4) | -0.00871 ± 0.022 (3) | -0.0865 ± 0.1 (4) |
| RMS tilt | deg | 3.48 ± 0.82 (4) | 2.23 ± 0.86 (4) | 2.39 ± 1.1 (3) | 2.13 ± 0.96 (4) |
| Tilt rate (servo activity) | deg/s | 16.4 ± 2.9 (4) | 11.5 ± 1.5 (4) | 14.8 ± 1.4 (3) | 11.4 ± 2.2 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (3) | 0 ± 0 (4) |

### Circle R42 mm, 2 s/turn (0.50 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 15.1 ± 6 (4) | 16.5 ± 12 (4) | 11.1 ± 4 (3) | 13.8 ± 8.7 (4) |
| Max tracking error | mm | 23.5 ± 3.5 (4) | 26 ± 14 (4) | 17.8 ± 2.1 (3) | 21.3 ± 10 (4) |
| IAE | mm*s | 57.6 ± 26 (4) | 63.3 ± 49 (4) | 42.8 ± 17 (3) | 52.7 ± 35 (4) |
| Mean radius error (ball - ref) | mm | -1.76 ± 13 (4) | -1.84 ± 15 (4) | 5.31 ± 6 (3) | -2.31 ± 13 (4) |
| Amplitude ratio (ball / ref, sine fit) | dB | -0.767 ± 2.8 (4) | -1.3 ± 4.7 (4) | 0.954 ± 1.2 (3) | -1.04 ± 3.5 (4) |
| Phase (sine fit, negative = lag) | deg | -1.09 ± 15 (4) | 1.59 ± 24 (4) | -7.26 ± 6.2 (3) | 2.72 ± 16 (4) |
| Phase lag | deg | 1.13 ± 15 (4) | -1.78 ± 24 (4) | 7.34 ± 6 (3) | -3.24 ± 17 (4) |
| Time lag | s | 0.00628 ± 0.081 (4) | -0.00992 ± 0.14 (4) | 0.0408 ± 0.034 (3) | -0.018 ± 0.096 (4) |
| RMS tilt | deg | 3.05 ± 1.1 (4) | 2.94 ± 0.86 (4) | 2.5 ± 0.76 (3) | 2.68 ± 0.82 (4) |
| Tilt rate (servo activity) | deg/s | 15.4 ± 2.5 (4) | 13 ± 4.3 (4) | 16 ± 2.5 (3) | 14.6 ± 5.2 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (3) | 0 ± 0 (4) |

### Circle R42 mm, 6 s/turn (0.17 Hz) - first turn dropped

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 20.3 ± 11 (4) | 22.6 ± 9.5 (4) | 16.6 ± 6.2 (3) | 21.7 ± 8.7 (4) |
| Max tracking error | mm | 24.9 ± 10 (4) | 31.1 ± 10 (4) | 22.4 ± 6.1 (3) | 28.9 ± 8.7 (4) |
| IAE | mm*s | 121 ± 65 (4) | 132 ± 58 (4) | 97 ± 39 (3) | 127 ± 54 (4) |
| Mean radius error (ball - ref) | mm | -19.2 ± 12 (4) | -20 ± 11 (4) | -15.1 ± 7.1 (3) | -20.2 ± 9.1 (4) |
| Amplitude ratio (ball / ref, sine fit) | dB | -6.11 ± 3.9 (4) | -6.84 ± 4.7 (4) | -4.15 ± 2.2 (3) | -6.55 ± 4 (4) |
| Phase (sine fit, negative = lag) | deg | -6.83 ± 3.1 (4) | -6.56 ± 4.1 (4) | -5.94 ± 2.4 (3) | -5.29 ± 3 (4) |
| Phase lag | deg | 6.86 ± 3.1 (4) | 7.44 ± 4.3 (4) | 6.01 ± 2.7 (3) | 6.1 ± 3.2 (4) |
| Time lag | s | 0.114 ± 0.052 (4) | 0.124 ± 0.071 (4) | 0.1 ± 0.045 (3) | 0.102 ± 0.053 (4) |
| RMS tilt | deg | 2.85 ± 0.95 (4) | 2.13 ± 0.66 (4) | 2.58 ± 0.79 (3) | 2 ± 0.63 (4) |
| Tilt rate (servo activity) | deg/s | 11.2 ± 0.15 (4) | 5.98 ± 2.4 (4) | 12.3 ± 0.71 (3) | 4.83 ± 0.45 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (3) | 0 ± 0 (4) |

### Pentagram

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS tracking error | mm | 20.4 ± 6.4 (4) | 21.8 ± 8.6 (4) | 15.2 ± 4.8 (3) | 19.9 ± 6.7 (4) |
| Max tracking error | mm | 39.3 ± 9.1 (4) | 40.8 ± 15 (4) | 32.3 ± 5.1 (3) | 37.1 ± 11 (4) |
| IAE | mm*s | 178 ± 58 (4) | 191 ± 75 (4) | 129 ± 42 (3) | 174 ± 60 (4) |
| RMS tilt | deg | 3.79 ± 0.68 (4) | 2.45 ± 0.43 (4) | 3.68 ± 0.73 (3) | 2.53 ± 0.41 (4) |
| Tilt rate (servo activity) | deg/s | 19.5 ± 3.3 (4) | 12.2 ± 2 (4) | 20.6 ± 2.2 (3) | 13.3 ± 1.6 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0 ± 0 (4) | 0 ± 0 (3) | 0 ± 0 (4) |

### Centre hold ('hold', 5 s after settle)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| RMS error | mm | 8.09 ± 4.4 (4) | 6.79 ± 1 (4) | 5.38 ± 2.3 (3) | 6.64 ± 1.3 (4) |
| Max error | mm | 13.1 ± 5.2 (4) | 11.3 ± 2.6 (4) | 9.41 ± 3.4 (3) | 11.3 ± 2.9 (4) |
| RMS tilt | deg | 3.56 ± 2.3 (4) | 1.46 ± 0.53 (4) | 2.22 ± 1.4 (3) | 1.48 ± 0.47 (4) |

### Disturbance: tilt kick (pulse, recovery = back inside 5 px)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Kick size | deg | 3 ± 0 (16) | 3 ± 0 (16) | 3 ± 0 (12) | 3 ± 0 (16) |
| Peak deviation | mm | 18.9 ± 6 (16) | 18.4 ± 4.8 (16) | 16.7 ± 4 (12) | 15.3 ± 3.8 (16) |
| Time to peak | s | 0.542 ± 0.16 (16) | 0.646 ± 0.3 (16) | 0.505 ± 0.15 (12) | 0.547 ± 0.18 (16) |
| Recovery time (after the pulse) | s | 2.27 ± 0.56 (7) | 2.48 ± 0.68 (6) | 2.66 ± 0.11 (5) | 2.52 ± 0.24 (7) |
| IAE | mm*s | 31.7 ± 15 (16) | 26.4 ± 4 (16) | 25.4 ± 10 (12) | 23 ± 4.7 (16) |
| Peak tilt | deg | 7.05 ± 3.2 (16) | 3.96 ± 0.67 (16) | 6.49 ± 2.1 (12) | 3.52 ± 0.61 (16) |

### Disturbance: constant tilt (push on, then release)

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Push size | deg | 1 ± 0 (4) | 1 ± 0 (4) | 1 ± 0 (3) | 1 ± 0 (4) |
| Peak deviation (push) | mm | 12.8 ± 8.5 (4) | 11.2 ± 4.3 (4) | 11 ± 4.8 (3) | 9.37 ± 2.4 (4) |
| Recovery time (push) | s | 1.85 ± 1 (2) | 2.74 ± 1.8 (4) | 3.77 ± 1.1 (3) | 3.88 ± 1.3 (4) |
| Remaining offset (last 1.5 s of push) | mm | 7.42 ± 6.7 (4) | 3.15 ± 1.5 (4) | 3.9 ± 3.6 (3) | 3.6 ± 0.72 (4) |
| IAE (push) | mm*s | 41.6 ± 32 (4) | 28.2 ± 9.7 (4) | 26.3 ± 16 (3) | 23.8 ± 5.3 (4) |
| Peak deviation (release) | mm | 13.9 ± 6.6 (4) | 14.1 ± 0.68 (4) | 10.7 ± 5.5 (3) | 10.1 ± 1.8 (4) |
| Recovery time (release) | s | 2.07 ± 1.2 (2) | 2.98 | 2.93 | 2.77 ± 0.24 (3) |

### Whole test

| Metric | Unit | PID | KF-PID | LQR | KF-LQR |
|---|---|---|---|---|---|
| Total IAE | mm*s | 1.4e+03 ± 3.5e+02 (4) | 1.49e+03 ± 4.3e+02 (4) | 1.02e+03 ± 2.6e+02 (3) | 1.34e+03 ± 4.5e+02 (4) |
| RMS tilt | deg | 3.7 ± 0.96 (4) | 2.36 ± 0.45 (4) | 3.38 ± 0.75 (3) | 2.41 ± 0.41 (4) |
| Time at tilt limit | % | 0 ± 0 (4) | 0.104 ± 0.18 (4) | 0 ± 0 (3) | 0 ± 0 (4) |
| Control rate | Hz | 61.6 ± 0.00053 (4) | 61.6 ± 0.00062 (4) | 61.5 ± 0.049 (3) | 61.6 ± 0 (4) |
| Ball-lost samples | - | 0 ± 0 (4) | 0 ± 0 (4) | 2.67 ± 4.6 (3) | 0 ± 0 (4) |
