# PID vs LQR - scripted test metrics

Scale: 1.09 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261110_122212_pid` (PID, END)


### Step out (centre -> edge)

| Metric | Unit | PID |
|---|---|---|
| Rise time 10-90 % | s | 1.88 ± 1.2 (4) |
| Overshoot | % | 17.7 ± 28 (8) |
| Settling time (5 % / 3 px band) | s | 5 |
| Steady-state error | mm | 17.9 ± 3.7 (8) |
| Max deviation from straight path | mm | 15.1 ± 5.9 (8) |
| IAE | mm*s | 132 ± 18 (8) |
| Peak tilt | deg | 11.4 ± 3.1 (8) |

### Step back (edge -> centre)

| Metric | Unit | PID |
|---|---|---|
| Rise time 10-90 % | s | 1.05 ± 0.63 (8) |
| Overshoot | % | 8.93 ± 5.5 (8) |
| Settling time (5 % / 3 px band) | s | 4.97 ± 0.017 (3) |
| Steady-state error | mm | 5.49 ± 1.1 (8) |
| Max deviation from straight path | mm | 13.5 ± 4 (8) |
| IAE | mm*s | 61.5 ± 11 (8) |
| Peak tilt | deg | 8.78 ± 1.8 (8) |

### Fast circle

| Metric | Unit | PID |
|---|---|---|
| RMS tracking error | mm | 11 |
| Max tracking error | mm | 26.8 |
| IAE | mm*s | 123 |
| Mean radius error (ball - ref) | mm | -8.95 |
| Phase lag | deg | -2.54 |
| Time lag | s | -0.0282 |
| RMS tilt | deg | 2.09 |
| Tilt rate (servo activity) | deg/s | 54.6 |
| Time at tilt limit | % | 0 |

### Pentagram

| Metric | Unit | PID |
|---|---|---|
| RMS tracking error | mm | 15.1 |
| Max tracking error | mm | 34.6 |
| IAE | mm*s | 258 |
| RMS tilt | deg | 2.58 |
| Tilt rate (servo activity) | deg/s | 39.7 |
| Time at tilt limit | % | 0 |

### Centre hold (last 3 s of 'end')

| Metric | Unit | PID |
|---|---|---|
| RMS error | mm | 4.97 |
| RMS tilt | deg | 1.07 |

### Whole test

| Metric | Unit | PID |
|---|---|---|
| Total IAE | mm*s | 2.22e+03 |
| RMS tilt | deg | 3.1 |
| Time at tilt limit | % | 0.152 |
| Control rate | Hz | 61.5 |
| Ball-lost samples | - | 48 |
