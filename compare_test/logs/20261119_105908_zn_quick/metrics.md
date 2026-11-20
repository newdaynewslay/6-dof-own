# PID - scripted test metrics

Scale: 1.045 mm/px. Values are mean ± std over all steps / runs (n in brackets).

Runs: `20261119_105908_zn_quick_n01_b1_pid` (PID, END)


### Step out (centre -> edge)

| Metric | Unit | PID |
|---|---|---|
| Rise time 10-90 % (of final value) | s | 1.01 ± 0.14 (4) |
| Steps reaching 90 % of target | % | 100 ± 0 (4) |
| Final position (of step size) | % | 106 ± 3.8 (4) |
| Overshoot | % | 16.2 ± 8.4 (4) |
| Settling time (5 % / 3 px band) | s | 2.89 ± 0.075 (2) |
| Steady-state error | mm | 6.41 ± 3.1 (4) |
| Max deviation from straight path | mm | 15.7 ± 13 (4) |
| IAE | mm*s | 47.2 ± 6.4 (4) |
| Peak tilt | deg | 8.09 ± 1.5 (4) |

### Step back (edge -> centre)

| Metric | Unit | PID |
|---|---|---|
| Rise time 10-90 % (of final value) | s | 0.921 ± 0.14 (4) |
| Steps reaching 90 % of target | % | 100 ± 0 (4) |
| Final position (of step size) | % | 106 ± 3 (4) |
| Overshoot | % | 14.6 ± 2.7 (4) |
| Settling time (5 % / 3 px band) | s | 2.94 |
| Steady-state error | mm | 5.95 ± 1.9 (4) |
| Max deviation from straight path | mm | 11.8 ± 1.6 (4) |
| IAE | mm*s | 49.7 ± 5.2 (4) |
| Peak tilt | deg | 7.43 ± 0.58 (4) |

### Fast circle

| Metric | Unit | PID |
|---|---|---|
| RMS tracking error | mm | 20.6 |
| Max tracking error | mm | 41.5 |
| IAE | mm*s | 152 |
| Mean radius error (ball - ref) | mm | 14.6 |
| Phase lag | deg | 7.96 |
| Time lag | s | 0.0885 |
| RMS tilt | deg | 4.22 |
| Tilt rate (servo activity) | deg/s | 57.3 |
| Time at tilt limit | % | 0 |

### Pentagram

| Metric | Unit | PID |
|---|---|---|
| RMS tracking error | mm | 16.1 |
| Max tracking error | mm | 45.7 |
| IAE | mm*s | 128 |
| RMS tilt | deg | 4.61 |
| Tilt rate (servo activity) | deg/s | 62.6 |
| Time at tilt limit | % | 0.683 |

### Centre hold (last 3 s of 'end')

| Metric | Unit | PID |
|---|---|---|
| RMS error | mm | 10.5 |
| RMS tilt | deg | 3.28 |

### Whole test

| Metric | Unit | PID |
|---|---|---|
| Total IAE | mm*s | 860 |
| RMS tilt | deg | 3.81 |
| Time at tilt limit | % | 0.221 |
| Control rate | Hz | 61.6 |
| Ball-lost samples | - | 0 |
