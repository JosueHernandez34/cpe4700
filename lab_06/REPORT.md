# Lab 06 Report -- Gamepad-Controlled Speed and Steering

The program (`main.c`) drives the robot from the Switch gamepad. It uses the Lab 04
PI speed controller on one encoder to set the forward speed, then adds a steering offset
to the two PWM channels. Speed is the encoder module's signed speed in A-edges (counts)
per second, the Lab 04 unit. The control loop runs every 1 ms (`clock_nanosleep` on
`CLOCK_MONOTONIC`, same as Lab 04), and the log gets one row every 20 ms. Plots are in
this folder and are made by `python3 plot_lab06.py`.

**Hardware assignment.** PWM channel 0 is the left motor and channel 1 is the right
motor. Encoder 0 is on the left motor and is the only feedback source. `pwm.c` already
inverts channel 0, so +duty drives both wheels forward. The direction check confirmed
the mapping: D-pad left slows the left wheel and speeds up the right one, so the
steering sign did not need to be flipped.

## Control mapping

The gamepad is read by `modules/gamepad.c`. A separate thread waits on the event device
with `poll()` and stores the latest values in atomics, so the 1 ms loop never blocks on
input. The device is found automatically, or a path such as `/dev/input/event13` can be
passed. The codes below were recorded with `gamepad_test` on the Jetson (USB).

| Control | evdev code | Values | Action |
|---|---|---|---|
| D-pad up/down | `ABS_HAT0Y` (0x11) | -1 up, 0 released, +1 down | forward/reverse target while held |
| D-pad left/right | `ABS_HAT0X` (0x10) | -1 left, 0 released, +1 right | steering while held |
| X | `BTN_WEST` (0x134) | 1 press, 0 release | toggle slow/fast on press |
| B | `BTN_SOUTH` (0x130) | 1 press, 0 release | stop both motors, latched |
| + (START) | `BTN_START` (0x13b) | 1 press, 0 release | exit |

- Releasing the D-pad returns the target and the steering to zero. Pressing opposite
  directions together gives zero for that control. Both start at zero.
- After B, the motors stay off until a new D-pad press. A direction that was already
  held when B was pressed does not restart them.
- The X toggle and the B stop act on the press edge, so holding a button does not repeat
  it.

| Mode | Target speed (edges/s) | Steering offset u_steer (duty) |
|---|---|---|
| Slow (0) | ±800 | ±0.08 |
| Fast (1) | ±2000 | ±0.15 |

## Control law

Each 1 ms iteration follows the handout's order: read the pad, read encoder 0 and run
the speed controller, then compute, clamp and write both PWMs. It logs every 20 ms and
then waits for the next tick.

    u_base = PI(target - measured)            (Lab 04 gains, deadband compensation)
    u_L    = clamp(u_base - u_steer, ±0.60)
    u_R    = clamp(u_base + u_steer, ±0.60)

Lab 04's `speed_control()` writes the PWM itself, so `speed_control_compute()` was added
to `modules/speed_control.c`. It returns u_base without writing anything, and
`speed_control()` now calls it, so Lab 04 is unchanged. When the target is zero, the
controller is reset and u_base is 0. This keeps a leftover integral from creeping the
wheels after a release. The logged `left_pwm` and `right_pwm` are the exact values
passed to `pwm_set_duty()`.

On START, Ctrl-C, a gamepad disconnect or read error, or a PWM write error, the program
sets both PWMs to 0, closes the log, and cleans up the speed-control, PWM, encoder and
gamepad modules. Then it reports the reason. Unplugging the controller mid-run stopped
both motors and ended the program, as required.

**Logging.** The header is `time_ms,target_speed,measured_speed,left_pwm,right_pwm,mode`.
`next_log_ms` starts at the current elapsed time + 20 and advances by 20 after each row,
as in the handout. `measured_speed` is the left (encoder) wheel only. In every run the
average row spacing is 20.0 ms. Single gaps range from 17 to 23 ms because the loop
occasionally runs a millisecond or two late. The next row then comes early, so the
20 ms schedule does not drift.

## Runs

| File | Robot | Mode | Motion | Length |
|---|---|---|---|---|
| `lab06_20261028_231010` | supported | slow | no input (stationary baseline) | 15.8 s |
| `lab06_20261028_231044` | supported | slow | forward, reverse, left, right | 17.5 s |
| `lab06_20261028_231107` | supported | fast | forward, reverse, left, right | 16.9 s |
| `lab06_20261028_230610` | floor | slow, then fast | short forward runs | 25.4 s |
| `lab06_20261028_230737` | floor | slow and fast | short forward runs, then fast left and right turns | 38.4 s |
| `lab06_20261028_232747` | floor | slow | forward, reverse, left, right | 17.6 s |

`lab06_20261028_230541` (floor, no input) was not used. The supported runs are the
stationary baseline and the floor runs are the moving data. A fast-mode reverse run on
the floor was not recorded.

## Results

Numbers are from `plot_lab06.py`. "Held" means rows where the target is non-zero and
the measured speed is within 20% of it, so it leaves out the rise after each press.
Mean |error| covers all rows with a non-zero target, including the rise. The `232747`
summary was not saved, so only its PWM values (read from the plot) are listed.

| Run | Robot | Mode | Mean \|error\| (edges/s) | Held speed std (edges/s) | Held mean \|PWM\| | Max \|PWM\| |
|---|---|---|---|---|---|---|
| 231044 | supported | slow | 31.4 | 25.7 | 0.164 | 0.170 |
| 231107 | supported | fast | 91.8 | 75.7 | 0.373 | 0.395 |
| 230610 | floor | slow | 96.3 | 47.2 | 0.212 | |
| 230737 | floor | slow | 171.8 | 59.6 | 0.224 | |
| 232747 | floor | slow | not printed | not printed | about 0.21 (plot) | about 0.24 (plot) |
| 230610 | floor | fast | 376.8 | 174.1 | 0.427 | 0.477 |
| 230737 | floor | fast | 324.6 | 148.0 | 0.426 | 0.463 |

The largest command in any run was 0.477, so the ±0.60 clamp was never reached.

**Tracking error.** With the robot supported, the encoder wheel reaches the target in
about 0.1-0.2 s and then holds it. In slow mode it stays within about 3% (std 26 of
800). In fast mode it stays within about 4% (std 76 of 2000). On the floor the presses in
230610 and 230737 were short (often under 1 s), so the rise takes up a large share of
each one. That is why their mean error is high even though the held speed is close to
the target. In the longer floor run (232747) the speed sags and recovers by about ±100
edges/s while it holds 800. Floor fast runs overshoot to about 2300-2700 edges/s when a
press starts. One sample at the end of a 230737 burst reads 4016 edges/s.

**Encoder noise.** At rest the encoder reads exactly 0 (231010: 791 rows, std 0). Speed
is computed from the time between A edges and reads 0 when no edge arrives, so there is
no noise floor at standstill. Noise appears only while moving. It grows with speed (std
26 slow vs 76 fast, supported) and with the floor load (47-60 slow, 148-174 fast). The
supported fast reverse run has two single-sample spikes to about -2700 edges/s (7.9 s and
9.3 s), and the PWM shows a matching blip each time. These are the same kind of
single-interval spikes seen in Lab 04, and the PI controller reacts to them.

**Steering response.** A turn with no forward target is open loop. The speed controller
is off (u_base = 0), so the wheels get -u_steer and +u_steer and the robot turns in
place. The signs are correct: D-pad left drives the left (encoder) wheel backward, and
D-pad right drives it forward. The encoder wheel's speed during turns was:

| Run | Robot | u_steer | Left turn | Right turn |
|---|---|---|---|---|
| 231044 | supported | 0.08 | about -330 | about +300 |
| 231107 | supported | 0.15 | about -740 | about +700 |
| 232747 | floor | 0.08 | about -200 | about +150 to +260 |
| 230737 | floor | 0.15 | about -570 | about +530 |

On the floor the same offset turns the wheel about 20-40% slower, because the tires
scrub sideways when the robot turns in place. 0.08 is only about 0.02 above the 0.06
deadband, so slow turns are weak and less even than fast ones.

**Slow vs fast.** The fast target is 2.5 times the slow one. The held duty is about 2.3
times higher with the robot supported (0.373 vs 0.164) and about 2.0 times on the floor
(0.43 vs 0.21-0.22). Fast mode has about 3 times the tracking error and speed noise. It
also overshoots more on the floor: about 2300-2700 edges/s on a 2000 target, vs about
900 on an 800 target in slow mode. The floor adds about 0.05 duty in both modes for the
same speed (0.21-0.22 vs 0.164 slow, 0.43 vs 0.373 fast), which is the extra load from
the robot's weight and rolling friction.

## Limits of single-encoder feedback

The one encoder measures only the left wheel's speed and direction. Forward speed is
regulated by making the left wheel follow the target, and the right wheel gets the same
u_base open loop. It is useful for:

- Regulating the left wheel's speed.
- Checking the sign of each command on that wheel.
- Measuring the left wheel's noise and its response to load.

It cannot show:

- **The right wheel's speed.** Lab 04 measured the two motors within about 3%, with a
  different reverse deadband (0.06 left, 0.04 right). A difference like that makes the
  robot curve slowly when driving "straight", and nothing corrects it.
- **Slip or a stall on the right wheel.** A blocked or slipping right wheel looks
  normal in the log.
- **The robot's heading or turn rate.** The steering plots show only the left wheel's
  half of each turn. The actual turn radius and yaw rate are not measured.
- **Slip on the left wheel itself.** Slip there shows up as speed error, and the
  controller cannot tell it apart from load.

There is also a side effect of using one encoder. If the robot steers while driving
forward, the PI loop keeps the left wheel at the target and so cancels part of -u_steer
on that side, while the right wheel keeps all of +u_steer. Steering is therefore not
symmetric between left and right turns while moving. All recorded turns were in place,
so this did not appear in the data. A second speed controller on encoder 1 (the
handout's alternative) would close the loop on both wheels and fix this. That optional
extension was not done, and no second-encoder data was logged.

## Files

- `main.c`: the Lab 06 program. The mapping table and event codes are at the top.
- `Makefile`: `make lab06`, plus `make gamepad_test` (prints every button and axis)
  and `make gamepad_drive` (an earlier open-loop test, not used for this lab).
- `plot_lab06.py`: plots and the numbers above. Run it as
  `python3 plot_lab06.py --baseline <supported runs> --moving <floor runs>`.
- Shared modules in `../modules/`: `pwm`, `encoder`, `speed_control` (adds
  `speed_control_compute()`) and `gamepad`.
- The CSV logs are on the Jetson and are not committed (`*.csv` is in `.gitignore`).
