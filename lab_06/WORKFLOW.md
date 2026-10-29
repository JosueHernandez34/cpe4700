# Lab 06 workflow checklist

Repo root is `~/dev/cpe4700`. Robot on a book, wheels off the ground, until step 3 passes.

## 1. Get the code on the Jetson
```
git fetch origin claude/sleepy-cerf-3tbv5d
git checkout -b lab_06 origin/claude/sleepy-cerf-3tbv5d
cd lab_06 && make lab06 gamepad_test
```
## 2. Confirm the controller codes (no motors)
```
grep -B2 -A8 "Pro Controller" /proc/bus/input/devices    # note the eventX in "Handlers="
./gamepad_test /dev/input/eventX
```
Press each d-pad direction, then X, B and + (START). Confirmed 10/28/26
(USB, auto-detected, so the eventX argument can be left out):

| Control | Code |
|---|---|
| D-pad | `ABS_HAT0X` (left -1, right +1) / `ABS_HAT0Y` (up -1, down +1) |
| X | `BTN_WEST` |
| B | `BTN_SOUTH` |
| + | `BTN_START` |

If a button prints a different name, change `BTN_MODE_TOGGLE`, `BTN_STOP` or
`BTN_EXIT` in `main.c`, fix the table, and `make lab06` again.

## 3. Direction check (supported, slow mode, short presses)
```
sudo ./lab06 /dev/input/eventX
```
- D-pad up: both wheels turn forward. If the encoder wheel runs away or
  oscillates hard, press B and check `FEEDBACK_MOTOR` (encoder 0) is the
  wheel on PWM channel 0.
- D-pad left: the left wheel slows and the right wheel speeds up. If it is the
  other way, swap `LEFT_CH` / `RIGHT_CH` (wrong wheel) or flip `STEER_SIGN`.
- B stops both wheels and holding up does not restart them; a new press does.
- START exits and prints the log name.

## 4. Data runs (each writes `lab06_YYYYMMDD_HHMMSS.csv`)
```
sudo ./lab06 /dev/input/eventX
```
- (a) Stationary baseline: no input for about 10 s, then START.
- (b) Slow: up 3 s, down 3 s, left 2 s, right 2 s, then START.
- (c) Fast: press X, then the same as (b).

## 5. Plots and numbers
```
python3 plot_lab06.py lab06_*.csv
```
Writes one PNG per CSV plus `baseline_vs_moving.png`, and prints row spacing,
tracking error, stationary speed std and PWM min/max.

## 6. Commit and push (no CSVs; `*.csv` is already in `.gitignore`)
```
git add main.c Makefile plot_lab06.py WORKFLOW.md *.png
git commit -m "Lab 06: plots"
git push -u origin lab_06     # then merge into main when finished
```
