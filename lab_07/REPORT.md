# Lab 07 Report -- Self-Balancing Robot Using PID Control

> **Draft.** The design sections are complete. Sections marked **TODO** need the robot runs
> from `WORKFLOW.md` steps 4 to 6.

The program (`main.c`) balances the robot on IMU pitch with a PID controller, and drives and
steers it from the Switch gamepad. It reuses the shared modules from earlier labs:
`imu`/`bno055` (Lab 05), `pwm` and `encoder` (Lab 04), and `gamepad` (Lab 06). The control loop
runs every 1 ms (`clock_nanosleep` on an absolute `CLOCK_MONOTONIC` tick, as in Labs 04 to 06),
and the log gets one row every 20 ms. Plots are made by `python3 plot_lab07.py`.

## Requirements map

| # | Requirement | Where |
|---|---|---|
| 1 | Balanced while stationary | PID on pitch, target theta0 when v_target = 0 |
| 2 | IMU pitch is the primary feedback | `read_pitch()`: theta = wrap(180 - raw BNO055 pitch) |
| 3 | Encoders estimate vehicle speed | `wheel_count()` + 50 ms count difference |
| 4 | PID with explicit P, I, D terms | `p_term`, `i_term`, `d_term` in the loop (also logged) |
| 5, 6 | Gamepad forward/reverse and steering | D-pad, section "Control mapping" |
| 7 | Balanced while moving | the speed loop only moves the pitch target, limited to theta0 ± 3 deg |
| 8 | Speed command → pitch target | theta_target = theta0 + Kv ev + Kvi ∫ev dt, ev = v_target - v_meas |
| 9 | CSV log | `lab07_YYYYMMDD_HHMMSS.csv`, gains in `_info.txt` |
| 10 | Clean init and shutdown | gamepad and IMU before motors; on every exit motors 0, log closed, all modules cleaned up |

## Control mapping

The codes are the Lab 06 ones, confirmed with `gamepad_test` on the Jetson.

| Control | evdev code | Action |
|---|---|---|
| D-pad up | `ABS_HAT0Y` -1 | ramps the forward speed command up while held (600 edges/s², up to vmax) |
| D-pad down | `ABS_HAT0Y` +1 | ramps the reverse speed command up while held |
| D-pad left / right | `ABS_HAT0X` -1 / +1 | steer left / right while held (u_steer ramps at 0.5 duty/s) |
| X | `BTN_WEST` | arm: the motors start once the robot is within 2 deg of theta0 |
| B | `BTN_SOUTH` | disarm: motors off at once |
| + (START) | `BTN_START` | exit |

When the d-pad is released, v_target ramps back to 0 at 1200 edges/s², so the robot comes
back to balancing in place. Ramping the command, instead of stepping it, keeps the pitch
target from jumping, which would kick the derivative term.

## Control law

Each 1 ms iteration follows the order in handout section 9:

    v_meas       = d/dt [ (count0 + count1) / 2 ]           (50 ms difference)
    ev           = v_target - v_meas
    theta_target = theta0 + clamp( Kv ev + Kvi ∫ev dt, ±3 deg )
    e            = theta_target - theta
    u            = DRIVE_SIGN ( Kp e + Ki ∫e dt + Kd de/dt ),  clamped to ±maxduty
    uL = u - u_steer,   uR = u + u_steer,                    each clamped to ±maxduty

- **Sign.** With e = theta_target - theta, a forward lean gives e < 0. The wheels have to drive
  forward (positive duty) to get back under the robot, so the PID sum is multiplied by
  DRIVE_SIGN = -1. With positive gains this is the same law as Lab 05.
- **Integral.** Integrated only while balancing and reset on every arm and disarm. It is
  clamped so that |Ki ∫e| stays at or below 0.30 duty (anti-windup).
- **Derivative.** The BNO055 fusion output only changes about every 10 ms. A derivative between
  1 ms samples would be mostly 0, with spikes. So de/dt is taken over a 10 ms window and
  low-pass filtered (weight 0.3, the Lab 05 values).
- **Clamp.** The final commands are clamped to ±maxduty (default 0.60, the Lab 05 tuning
  limit), which is inside the PWM module's ±1 range.
- **Pitch frame.** theta = wrap(180 - raw) is the Lab 05 convention: forward lean positive.
  **theta0 = 3.845 deg** is the Lab 05 zero, the midpoint of the measured block stops
  (11.25 and -3.56 deg). `--monitor` prints a 2 s mean of the pitch, so it can be checked
  with the robot held at its balance point. **TODO: value confirmed on the robot: ____ deg.**

## How the encoders generate the pitch-angle target

The vehicle speed is the average of the two wheel counts, so a turn in place (one wheel
forward, one back) reads as zero forward speed. The average count is stored every 1 ms in a
51-entry ring buffer. The speed is the change over the last 50 ms:

    v_meas = (c_now - c_50ms_ago) / 0.05 s          (edges/s)

One count on one wheel changes the average by 0.5, which is 10 edges/s. The 50 ms window
gives an estimate with no single-edge spikes, unlike the edge-timing speed used in Lab 06.
It lags by about 25 ms, which is slow next to the pitch loop and fast next to driving speed.

The speed error becomes a lean, through a PI speed loop:

    ev           = v_target - v_meas
    theta_target = theta0 + Kv ev + Kvi ∫ev dt,   limited to theta0 ± 3 deg

- **Robot too slow (or v_target > 0 from rest).** The target leans forward. The pitch loop
  drives the wheels to hold that lean, so the robot accelerates forward.
- **Robot drifting forward while v_target = 0.** v_meas > 0, so the target leans back. Holding a
  backward lean means slowing down. This also removes the slow creep that a theta0 error of
  a few tenths of a degree would cause.
- **v_target = 0 and stopped.** theta_target = theta0.

With the default Kv = 0.003 deg per edge/s, a 600 edges/s error asks for a 1.8 deg lean. The
±3 deg limit keeps any speed command from asking for a lean close to the blocks (about
±7.4 deg from theta0).

**Why the speed loop has an integral.** With P only, the speed settles where Kv ev equals
the lean needed to keep moving against rolling friction, so it always runs a little below
the command. A theta0 error does the same thing at standstill: the robot creeps. The pitch
PID's own integral cannot fix either one, because it drives the pitch to the target, not
the speed to the command. Kvi ∫ev slowly adds the missing lean until the speed error is 0.

- Default Kvi = 0.0005 deg per edge: a 600 edges/s error held for 1 s adds 0.3 deg of lean.
  That is an integral time Kv / Kvi = 6 s, slow next to the pitch loop, so it does not fight
  the balance.
- Anti-windup: the integral runs only while balancing and is reset on every arm and disarm.
  |Kvi ∫ev| is clamped to 2 deg. The integral is frozen while the total target sits at its
  ±3 deg limit and the error would push it further, so a long press cannot wind it up.
  (Checked in the PC test: with a large Kvi the target stopped at exactly θ0 − 3 deg and the
  integral held at about −1.2 deg instead of running to the 2 deg clamp.)
- `--kvi 0` turns it off, which gives the handout's suggested P-only form.

## Safety

| Event | Response |
|---|---|
| Start-up | motors disarmed; X arms; motors engage only within 2 deg of theta0 |
| \|theta - theta0\| > 15 deg | disarm (fell or picked up), press X to re-arm |
| beyond 6 deg for 1 s | disarm (lying on a block) |
| IMU read error | motors 0, exit (`get_imu_pitch_checked()`, added to `modules/imu.c`) |
| Gamepad disconnect or read error | motors 0, exit |
| PWM write error, Ctrl-C, START | motors 0, exit |

At exit the program prints the loop timing: ticks, the number more than 500 µs late, and
the worst one. **TODO: worst late tick in the data runs: ____ µs.**

## Logging

Header:
`time_ms,target_speed,measured_speed,pitch_deg,pitch_target_deg,error_deg,pid_output,left_pwm,right_pwm,u_steer,p_term,i_term,d_term,state,speed_p_deg,speed_i_deg`

The first nine columns are the handout's. The rest are the steering offset, the three PID
terms (each with DRIVE_SIGN applied, so p + i + d = pid_output before the clamp), and the state
(0 disarmed, 1 armed waiting, 2 balancing), and the speed loop's P and I lean in deg
(before the ±3 deg clamp). `next_log_ms` starts at the elapsed time + 20 and
advances by 20 after each row, as in the handout. If the loop ever stalls past a whole slot,
that slot is skipped instead of writing duplicate rows. The gains and limits for each run go
in `lab07_..._info.txt`, which `plot_lab07.py` reads.

## Controller tuning — TODO

Procedure 10.3: P only, then PD, then PID, first with `--kv 0 --kvi 0`.

| Step | Kp | Ki | Kd | Kv | Kvi | Result |
|---|---|---|---|---|---|---|
| P only | | 0 | 0 | 0 | 0 | |
| PD | | 0 | | 0 | 0 | |
| PID | | | | 0 | 0 | |
| + speed loop P | | | | | 0 | |
| + speed loop I | | | | | | |
| **Final** | | | | | | |

Start values are Lab 05 setting C (Kp 0.015, Ki 0.005, Kd 0.005), the only one that gave
long holds (21.9 s) there.

## Results — TODO (from `plot_lab07.py`)

| Run | Pitch error std (deg) | Oscillation (Hz) | Speed error (edges/s) | Notes |
|---|---|---|---|---|
| Stationary | | | — | |
| Forward | | | | |
| Reverse | | | | |
| Turns | | | — | |
| Disturbance | peak: | overshoot: | recovery: | |

## Discussion — TODO

- Stability:
- Oscillation:
- Overshoot:
- Recovery time:
- Effects of changing Kp, Ki and Kd:

## Files

- `main.c`: the Lab 07 program. The constants and the mapping are at the top. `--monitor` is
  the sensor check.
- `Makefile`: `make lab07`.
- `plot_lab07.py`: plots (five per run) and the numbers above.
- `WORKFLOW.md`: the run checklist for the Jetson.
- Shared modules in `../modules/`: `pwm`, `encoder`, `imu` (adds `get_imu_pitch_checked()`),
  `bno055` and `gamepad`.

The program was compiled and run on a PC against stub modules: a simulated pendulum and
a scripted gamepad. That test checked the signs (a forward lean gives forward duty, and
forward speed moves the target back), the arm, engage and cutoff sequence, the 20 ms rows
(mean 20.0 ms) and the shutdown path. The simulated plant was invented, so the test says
nothing about the gains.
