# Lab 07 workflow checklist

Repo root is `~/dev/cpe4700`. Keep the robot on a book, with the wheels off the ground, until step 3 passes.
Have a hand ready on B (disarm) every time the motors are armed.

## 1. Get the code on the Jetson
```
git fetch origin claude/zen-turing-g3q7ym
git checkout -b lab_07 origin/claude/zen-turing-g3q7ym
cd lab_07 && make lab07
```

## 2. Verify the sensors, no motors (procedure 10.1)
```
sudo ./lab07 --monitor
```
- Tip the robot forward: `pitch` must go **up**. Tip it back: `pitch` must go down.
- Roll the robot forward by hand: `cnt0`, `cnt1` and `v` must all go **up**. If `cnt1`
  goes down, set `ENC1_SIGN` to `-1.0` in `main.c` and rebuild. Do the same with `ENC0_SIGN`
  if `cnt0` goes down.
- Roll it faster and slower: `v` must rise and fall with the wheel speed.
- **theta0:** hold the robot still at the point where it balances by itself, and read
  `mean 2s`. If it differs from 3.845 (the Lab 05 zero), pass it with `--theta0 X` on every
  run, or change `THETA0_DEFAULT_DEG` in `main.c`.

## 3. Verify the motor response, supported (procedure 10.2)
```
sudo ./lab07 --ki 0 --kd 0
```
- Hold the robot upright on the book. Press **X**. It starts when the pitch is within 2 deg
  of theta0 and prints `Balancing`.
- Tip it forward: both wheels must spin **forward** (toward the lean). Tip it back: both must
  spin backward. If they spin the other way, set `DRIVE_SIGN` to `+1.0` in `main.c`.
- D-pad left: the left wheel slows and the right speeds up (Lab 06 sign).
- Hold it past 6 deg for 1 s: it prints `REST` and the motors stop. Past 15 deg: `CUTOFF`.
- B stops the motors at once. START exits and prints the log name.

## 4. Tune on the floor (procedure 10.3)
Each run writes `lab07_YYYYMMDD_HHMMSS.csv` and a `_info.txt` with the gains. Hold the robot
upright, press X, let go once it says `Balancing`.
```
sudo ./lab07 --kv 0 --ki 0 --kd 0 --kp 0.015       # P only. Raise Kp until it catches a fall but oscillates
sudo ./lab07 --kv 0 --ki 0 --kd 0.005              # PD. Raise Kd until the oscillation dies out
sudo ./lab07 --kv 0                                # PID. Ki removes the steady lean
```
- Use `--kv 0` while tuning the balance loop, so the speed loop does not move the target.
- Then add the speed loop: `--kv 0.001`, then `0.002`, `0.003`. Too low: the robot creeps
  away. Too high: it surges back and forth.
- If small corrections do nothing (the Lab 04 deadband is 0.06), try `--deadband 0.03`.
- Write down the final Kp, Ki, Kd and Kv for the report.

## 5. Data runs (procedure 10.5 and section 11)
With the final gains, one run each:
- (a) Stationary: no input, about 15 s.
- (b) Forward: hold up about 3 s, release, let it stop.
- (c) Reverse: hold down about 3 s, release.
- (d) Turns: left about 2 s, then right about 2 s.
- (e) Disturbance: balancing, still, then a light push on the top. **Note the time** shown
  in the log (or count seconds from the start).
- Then START. Check that it prints `Log closed`.

## 6. Plots and numbers
```
python3 plot_lab07.py lab07_*.csv
python3 plot_lab07.py lab07_<push run>.csv --disturb <seconds>
```
Writes one PNG per CSV (five plots each) and prints stationary pitch error and std,
oscillation frequency, speed tracking, and disturbance peak, overshoot and recovery time.

## 7. Commit and push (CSVs are ignored by `.gitignore`, so add them with -f)
```
git add main.c Makefile plot_lab07.py WORKFLOW.md REPORT.md *.png *_info.txt
git add -f lab07_*.csv
git commit -m "Lab 07: data runs and plots"
git push -u origin lab_07
```
