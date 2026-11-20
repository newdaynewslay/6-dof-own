# Controller compare test

Compares PID, LQR, KF-PID and KF-LQR on the ball balancing robot.
The sketch runs on the Teensy, and `compare_controllers.py` runs on the computer.

## 1. Flash the sketch

`platformio.ini` already points to `compare_test/Ball_Balance_Compare`.

```
cd ~/"Documents/NTNU/MSMECAUT - Master/AIS4001/6-dof-own"
pio run -t upload
```

Close the serial monitor before running the script.

Python packages (once): `python3 -m pip install pyserial numpy matplotlib`

## 2. Full test

Put the ball on the plate, then:

```
cd compare_test
python3 compare_controllers.py run --tag full
```

- 16 runs (4 blocks x 4 controllers), random order, about 29 minutes.
- Each run: centre hold, 4 edge points, 3 circles, pentagram, disturbance.
- If the ball falls off, put it back. The script waits 60 s and retries once.
- Logs, metrics and figures end up in `logs/<time>_full/`.

Useful options:

| Option | What it does |
|---|---|
| `--ctrl pid,lqr` | only these controllers (`pid`, `lqr`, `kf`, `kflqr`, `pidzn`) |
| `--repeat 2` | number of blocks (default 4) |
| `--no-dist` | skip the disturbance part |
| `--tag name` | text added to the folder name |

Analyse old logs again:

```
python3 compare_controllers.py analyze logs/<folder>/*.csv
```

## 3. Live mode

Balances at the centre with one controller and waits for your commands. Nothing is logged.

```
python3 compare_controllers.py live --ctrl lqr
```

Type a command and press Enter:

| Command | What it does |
|---|---|
| `50 50` or `s 50 50` | move the set point to x=50, y=50 px |
| `c` | back to the centre (also stops a circle or star) |
| `o` | start/stop a circle with default size |
| `o 60 4` | circle, radius 60 px, 4 s per turn |
| `p` / `p 60 40` | pentagram (radius px, speed px/s) |
| `m 0` ... `m 4` | change controller: 0 PID, 1 LQR, 2 KF-PID, 3 KF-LQR, 4 PID-ZN |
| `?` | print the current settings and controller |
| `exit` or Ctrl+C | servos off and quit |

The set point is limited to 80 px from the centre. PID-ZN (`m 4`) is unstable and drops the ball.

## 4. Serial commands (pio device monitor)

The same commands work in `pio device monitor` after `e` (servos on). `q` turns the servos off.
`z i deg` trims servo offset i for a quick test (not saved, reset the Teensy to undo).
