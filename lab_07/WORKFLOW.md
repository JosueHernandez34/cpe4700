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
sudo ./lab07 --kv 0 --kvi 0 --ki 0 --kd 0 --kp 0.015   # P only. Raise Kp until it catches a fall but oscillates
sudo ./lab07 --kv 0 --kvi 0 --ki 0 --kd 0.005          # PD. Raise Kd until the oscillation dies out
sudo ./lab07 --kv 0 --kvi 0                            # PID. Ki removes the steady lean
sudo ./lab07 --kvi 0 --kv 0.002                        # + speed loop, P only
sudo ./lab07 --kv 0.002 --kvi 0.0005                   # + speed loop integral
```
- Use `--kv 0 --kvi 0` while tuning the balance loop, so the speed loop does not move the target.
- Then add the speed loop P: `--kv 0.001`, then `0.002`, `0.003`. Too low: the robot creeps
  away. Too high: it surges back and forth.
- Then the speed loop I: start at `--kvi 0.0005` (the default). Raise it if the robot settles
  below the commanded speed, or creeps when it should stand still. Lower it if the robot
  overshoots the speed and rocks slowly (about once every 2 to 5 s).
- If small corrections do nothing (the Lab 04 deadband is 0.06), try `--deadband 0.03`.
- Write down the final Kp, Ki, Kd, Kv and Kvi for the report.

## 5. Data runs (procedure 10.5 and section 11)
With the final gains, one run each:
- (a) Stationary: no input, about 15 s.
- (b) Forward: hold up about 3 s, release, let it stop.
- (c) Reverse: hold down about 3 s, release.
- (d) Turns: left about 2 s, then right about 2 s.
- (e) Disturbance: balancing, still, then a light push on the top. **Note the time** shown
  in the log (or count seconds from the start).
- Then START. Check that it prints `Log closed`.

## Expected results (what "on the right track" looks like)

There is no reference data from a working robot, so this lists what each check and plot
should show, and what each common fault looks like.

### Sensor and supported checks (steps 2-3)

| Check | Working | Fault and fix |
|---|---|---|
| `--monitor`, robot still | pitch std under about 0.1 deg; `v` reads 0 | `v` jumps with the robot still: encoder wiring or noise |
| `--monitor`, tip forward | pitch goes up | goes down: the IMU frame changed; stop and check before running motors |
| `--monitor`, roll forward | `cnt0`, `cnt1`, `v` all go up | one count goes down: flip that `ENC*_SIGN` |
| `--monitor`, held at balance | `mean 2s` within about 1 deg of 3.845 | far off: use `--theta0 <mean 2s>` |
| supported, `Balancing`, tip forward | both wheels spin forward, harder the further you tip | wheels spin backward: flip `DRIVE_SIGN` |
| supported, D-pad left | left wheel slower, right faster | other way round: flip `STEER_SIGN` |
| exit message | `Loop: ... worst` under about 2000 us, few late ticks | many late ticks: something else is loading the CPU |

### Tuning (step 4)

| What you see | Likely cause |
|---|---|
| Robot falls slowly in one direction, wheels too weak to catch it | Kp too low |
| Fast oscillation (several Hz) growing until it falls | Kp too high, or Kd too low |
| Buzzing or chattering wheels, PID output jumping between its limits | Kd too high (the pitch derivative is noisy) |
| Balances but leans, with the PID output holding steady to one side | theta0 off, or Ki too low; the I term should take it out within a few seconds |
| Slow back-and-forth that grows, PID output stuck at ±maxduty | Ki too high (windup), or maxduty too low |
| Balances, but rolls away slowly in one direction | speed loop off or Kv too low |
| Surges back and forth around the start point (about 0.5-1 Hz) | Kv too high |
| Rocks slowly (every 2-5 s) and overshoots the commanded speed | Kvi too high |

### Plots (step 6)

| Run | Plot | Working |
|---|---|---|
| Stationary | 2 (pitch) | pitch stays within about ±1-2 deg of the target; target stays near theta0 |
| Stationary | 1 (speed) | measured speed wanders around 0, no steady drift |
| Stationary | 3, 4 (PID, PWM) | small, centred on 0; left and right PWM lines on top of each other; rarely at ±maxduty |
| Forward / reverse | 1 (speed) | measured speed follows the ramp with some lag, then settles within about 10 % of the command (the speed I term closes the rest) |
| Forward / reverse | 2 (pitch) | at the start of a press the target leans in the direction of travel, then returns toward theta0 as the speed catches up; on release it leans the other way briefly to stop |
| Forward / reverse | dashed I lean line | builds slowly while driving and stays well inside ±2 deg |
| Turns | 4 (PWM) | left and right PWM split by about 2 × u_steer (0.16) around the balance command |
| Turns | 1 (speed) | measured speed stays near 0 (the two wheels cancel) |
| Disturbance | 5 | one large error peak, at most one or two overshoot swings, back inside ±1 deg in about 1-2 s |

Numbers printed by `plot_lab07.py` that point to a good tune: stationary pitch error std
under about 1 deg; left PWM at its max under a few percent of the time; disturbance
overshoot under about 50 % and recovery under about 2 s. These are guide values for a
first tune, not measured on this robot.

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
