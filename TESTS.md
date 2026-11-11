# Test log for the robot (Labs 1 to 7)

One place for every test that has been done on this robot, what it showed, where the data is, and how to
run it again. Built 11/11/26 from the lab reports, the Claude Code channel and the session reports in
`~/cc_reports/`. Numbers are copied from those sources. Where a result was not recorded, it says so.

Shared settings: `modules/robot_params.h` (C) and `modules/robot_params.py` (Python). If a test below gives a
better value, change it there and add a line here.

How to use this file when something breaks: read "Quick troubleshooting" first, then the test for that part.

CSV and .mat data files are not in git (`*.csv`, `*.mat` are ignored). They are on the Jetson in the lab folders.
Run every lab program with `sudo` (sysfs PWM, GPIO, I2C).

---

## Quick troubleshooting

| Symptom | Likely cause | Where it was seen | Check |
|---|---|---|---|
| Pitch reads 0.0 and looks "upright" | `get_imu_pitch()` returns 0.0 when the I2C read fails | Lab 05 Step 1 | Lab 07 treats a pitch that stays exactly unchanged for 200 ms as a failure |
| Pitch is all zero right after boot | BNO055 is in CONFIG mode until `imu_init()` runs | Lab 05 Step 1 (10/21) | Run any lab program once; `imu_init()` sets NDOF |
| Zero or balance angle looks wrong | Wrong saved zero, or IMU remounted | Lab 05, Lab 07 | Redo `sudo ./lab07 --cal`; file is `lab_07/pitch_zero.txt` (zero, forward stop raw, back stop raw) |
| Encoder speed spikes or one wrong-sign sample | Speed comes from the time between single A edges | Lab 04, Lab 06 | Expected, about 11 to 14 % noise at steady speed; PI reacts to spikes |
| Wheel 1 reads the wrong direction sometimes | Same cause as above | Lab 04 | Use encoder 0 for feedback (Labs 06, 07) |
| Robot will not move for small commands | Deadband about 0.06 duty | Lab 04 | Deadband compensation is in `speed_control.c` and Lab 07 `balance_ctrl.c` |
| Wheels fight or the robot turns while balancing | Wheels loaded unequally (one ran 300 RPM, the other 100 on the same duty) | Lab 05 Fig. 2 | Compare left and right PWM and wheel speeds |
| Controller not found | Not connected, wrong event device, or not in pairing mode | Lab 06, 10/26 and 11/11 | `cat /proc/bus/input/devices`, `bluetoothctl devices`, see "Gamepad and Bluetooth" |
| Gamepad works wired only | Wireless not paired yet | 11/11 | Session "Jetson wireless gamepad bring-up" in the channel |
| `enc_init` prints "Failed to set RT priority" | Not run with sudo | Lab 05 Step 1 | Use sudo |
| Program cannot open PWM or GPIO | Not run with sudo | all labs | Use sudo |
| Plots and fits only use some of the log | System-ID steps and block rests are excluded on purpose | Lab 05 | See `lab_05/REPORT.md` |

---

## Hardware and system checks

### 10/21 Lab 05 Step 1 read-only check (Jetson jetson-emb-ctrl-005)
Report: `~/cc_reports/lab05_step1_report.md`.
- IMU: i2c bus 7, address 0x28, CHIP_ID 0xA0 OK, SYS_STATUS 0, SYS_ERR 0. OPR_MODE was CONFIG (no fusion) until `imu_init()`. Calibration status 0/0/0/0 at that moment.
- Encoders: initialized on gpiochip0 lines 124/53 and 52/51. Counting not verified at that time (needs a wheel turned by hand).
- Jetson: Orin Nano, L4T R36.4.7, kernel 5.15.148-tegra, 25 W mode, 43 to 46 C. No brownout, OOM, throttling, USB reset or I2C errors in the journal.
- Bluetooth: built-in Realtek RTL8822CE, adapter hci0 up, BlueZ 5.64.
- GTK4: runtime installed, dev package was missing at that time (Lab 05 GUI needed it).
- Known API gaps noted: `get_imu_pitch()` returns 0.0 on I2C failure; IMU read every 1 ms but BNO055 fusion updates at 100 Hz.

### 11/11 Jetson Bluetooth and gamepad diagnosis (read-only)
Channel session "11/11 - Jetson wireless gamepad bring-up". Nothing was changed on the system.
- Kernel: `BT_HIDP=m`, `UHID=m`, `HID_GENERIC=y`, `INPUT_EVDEV=y`, `INPUT_JOYDEV=m`. No `hid_nintendo`, so a Switch-style mode will not work without extra setup.
- Adapter hci0 (Realtek, USB 13d3:3549, Bluetooth 5.1, BLE capable): up, not blocked, `Pairable: no`, nothing paired.
- Input devices present on 11/11 (before the pad was plugged in): Compx 2.4G Receiver (25a7:fa61, keyboard and mouse dongle, not the gamepad) and WaveShare WS170120 touchscreen (0eef:0005). The gamepad was not plugged in.
- Not done: wired baseline, wireless pairing modes. Status: waiting on Josue (controller manual and wired baseline).
- Earlier (10/26): adapter up, nothing paired, a 15 s scan saw no gamepad (controller probably not in pairing mode).

---

## Lab 04: motor characterization and speed control
Reports: `lab_04/REPORT.md`, `lab_04/REPORT_short.md`. Code: `lab_04/main.c`, `lab_04/analyze.py`, `lab_04/lab04_analysis.m`.
Data (Jetson, not in git): `lab_04/deadband.csv`, `lab_04/step_response.csv`, `lab_04/speed_control_1000.csv`, `lab_04/lab04_data.mat`.
Re-run: `cd lab_04 && make && sudo ./lab04 <mode>` (modes include `deadband`); see `lab_04/WORKFLOW.md`.

| Test | Setup | Result |
|---|---|---|
| Deadband | Duty raised from 0.00 in 0.02 steps, 1 s per level; deadband = first duty that moved the wheel 5 or more counts | W0 +0.06 / -0.06, W1 +0.06 / -0.04 |
| Step response | Steps of +/-0.60 duty, both wheels | First order. Gain about 6100 edges/s per unit duty (W0 6100 fwd / 5904 rev, W1 6139 / 6147). Time constant about 40 ms; 10 to 90 % rise 53 to 67 ms. Plateau about 3600 to 3700 edges/s. Wheels agree within about 3 %; net travel 3295 vs 3327 counts |
| Encoder noise | Steady speed | Std 420 (W0) and 507 (W1) edges/s, 11 to 14 % of speed; spikes up to about 11000 (W0) and 17000 (W1); W1 sometimes reads the wrong direction. No settling time could be measured because noise exceeds the 2, 5, 10 and 20 % bands |
| Speed control, first attempt | Kp 0.005 (loop gain about 30) | Duty switched between its limits, no tracking |
| Speed control (PI) | Triangle +/-1000 edges/s, 3 cycles, Kp 0.0001, Ki 0.0025, deadband compensation | Correlation 0.98 both wheels. RMS error 119 (W0) and 123 (W1) edges/s, about 12 %. Lag 47 to 49 ms. Peaks reach 960 to 975 instead of 1000 |
| Speed control at the handout's +/-100 edges/s | Same gains | Only about 0.02 duty above the deadband, wheels barely turned, correlation 0.56 to 0.58 (noise dominates); +/-1000 used instead |

Gain rule used: Kp chosen so loop gain Km*Kp is about 0.6; Ki = Kp / tau.

---

## Lab 05: self-balancing (PID on pitch) with GTK4 display
Reports: `lab_05/REPORT.md`, `~/cc_reports/lab05_step1_report.md`, `lab_05/runid_readiness_1026.md`. Plots: `lab_05/plots/` (`python3 plot_gains.py`). Fit: `lab_05/identify.py`, `lab_05/identify_v2.py`.
Data (Jetson, not in git): `lab_05/balance_log_*.csv` and `*_model.json`.
Re-run: `cd lab_05 && sudo ./robotest` (v1) or `sudo ./robotest_v2` (v2). A first-time build needs the GTK4 dev package.

### Balance runs (PID, gains recovered by least squares, R^2 = 1.000 on unsaturated windows)
| Setting | Kp | Ki | Kd | Runs | Result |
|---|---|---|---|---|---|
| A | 0.010 | 0.010 | 0.015 | 1 (10/21, limit 0.30) | Pitch std 2.4 deg, 5 Hz chatter, duty at the limit 72 % of the time, longest hold within +/-5 deg 4.2 s, stopped by hand at 4.5 s |
| B | 0.010 | 0.010 | 0.005 | 1 (10/21, limit 0.30) | Pitch std 4.2 deg, 71 % at the limit, hold 5.3 s, fell to -47 deg at 7.5 s and the cutoff latched |
| C (default) | 0.015 | 0.005 | 0.005 | 13 (10/21 to 10/26) | Pitch std 4.9 deg (median), 18 % at the limit (median), best hold 21.9 s, 10 of 13 runs ended by the cutoff |

| Log | Date | Span | Gains and limit | Used in |
|---|---|---|---|---|
| balance_log_141733 | 10/21 14:17 | 51.6 s | A, B, C at 0.30 | Fig. 1, 3 |
| balance_log_000508 | 10/22 00:05 | 290.9 s | C at 0.30 | Fig. 3 |
| balance_log_161309 | 10/26 16:13 | 4.3 s | Ki 0.04, wheels never moved | not used |
| balance_log_161658 | 10/26 16:16 | 17.9 s | C at 0.60 | Fig. 3 |
| balance_log_162227 | 10/26 16:22 | 120.0 s | C at 0.60 | Fig. 2, 3 |
| balance_log_163831 | 10/26 16:38 | 36.9 s | C at 0.60 | none (no usable segment) |
| balance_log_165627 | 10/26 16:56 | 71.6 s | C at 0.60 | Fig. 3 |
| balance_log_170442 | 10/26 17:04 | 142.1 s | C at 0.60 | Fig. 3, 4 (best hold 21.9 s) |

What was seen:
- Only Kp 0.015, Ki 0.005, Kd 0.005 at the 0.60 limit gave long holds (21.9 s and 12.6 s, 1 to 5 % of the time at the limit). Kd 0.015 chattered; Kd 0.005 removed the chatter but could not stop slow falls; the pitch rate is noisy, so Kd much above 0.005 drives the motors on noise.
- 10/26 16:22 run: held between about -15 and +15 deg for about 45 s, then tipped. Between 35 s and 50 s motor 1 ran at about 300 RPM while motor 0 ran at about 100 RPM on the same duty (unequal wheel load).
- Best run (17:04): stayed between 0 and +8 deg for 21.9 s, then dropped to -7.5 deg and the cutoff latched.
- Not measured: no P-only, no PI-only run, no Kp/Kd sweep, no A or B run at the 0.60 limit.

### System identification (Run ID, pseudo-random duty steps)
- 10/22 00:05 fit (`balance_log_000508_model.json`): one-step R^2 0.997 / 0.85 / 0.88, 1 s free-run pitch RMS 1.0 deg, but a1 = -0.41. A negative a1 does not look like a free inverted pendulum (robot probably supported). Used the 25 deg interim stop. Never copied into `ss_params.py`.
- 10/26 fit (`balance_log_170442_model.json`): marked `usable: false`. The robot rested on the block for 28 % of the run and only 17 of 40 segments stayed inside the stop zone. Coefficients (not trusted): a1 79.27, a2 30.76, a3 1.31, a33 -36.02, b2 -9533, b3 261420, c 0.0365, Km 7257, tau 0.0278.
- Result: the model coefficients a1, a2, c are NOT SET in `ss_params.py`, so the robot runs on PID, not state feedback.
- Run ID readiness (10/26): signal is a 9-bit pseudo-random sequence, 150 ms per level, 20 s per run; one Run ID per launch recommended.

### Mechanical stops
- 10/22 Josue's reading with the blocks: pitch -8 to +10 deg. The earlier "about 25 deg" claim was wrong or measured from the ground. Not measured with `pitch_probe` at that time.
- 10/26 ground reading (Josue): forward +11.25, back -3.56 in the IMU frame; zero was set to their midpoint (3.845), stops at +/-7.405.
- 11/11 `lab07 --cal`: forward +12.81 (moved 0.25 deg), back -2.81 (moved 0.00 deg). Zero = midpoint +4.997. Stops at +/-7.810. Saved in `lab_07/pitch_zero.txt`.

---

## Lab 06: gamepad speed and steering
Report: `lab_06/REPORT.md`. Plots: `lab_06/*.png` (`python3 plot_lab06.py`). Data: `lab_06/lab06_20261028_*.csv` (not in git).
Re-run: `cd lab_06 && make && sudo ./lab06` (add an event path such as `/dev/input/event13` if auto-detect fails). `./gamepad_test` prints live axes and buttons.

Gamepad codes recorded on USB: D-pad up/down `ABS_HAT0Y` (-1 up, +1 down); left/right `ABS_HAT0X`; X `BTN_WEST` 0x134; B `BTN_SOUTH` 0x130; START `BTN_START` 0x13b.

| Run | Robot | Mode | Mean abs error (edges/s) | Held speed std | Held mean abs PWM |
|---|---|---|---|---|---|
| 231010 | supported | slow | stationary baseline, encoder exactly 0 over 791 rows | | |
| 231044 | supported | slow | 31.4 | 25.7 | 0.164 |
| 231107 | supported | fast | 91.8 | 75.7 | 0.373 |
| 230610 | floor | slow / fast | 96.3 / 376.8 | 47.2 / 174.1 | 0.212 / 0.427 |
| 230737 | floor | slow / fast | 171.8 / 324.6 | 59.6 / 148.0 | 0.224 / 0.426 |
| 232747 | floor | slow | not printed | not printed | about 0.21 (from the plot) |

Not used: `lab06_20261028_230541` (floor, no input). A fast-mode reverse run on the floor was not recorded.
- Targets: slow 800 and fast 2000 edges/s; steering offset 0.08 and 0.15 duty. Largest command in any run was 0.477, so the 0.60 clamp was never reached.
- Steering: turns in place are open loop; signs correct without a flip (left turn about -330 and right about +300 edges/s on the encoder wheel, supported, 0.08; -740 and +700 at 0.15). On the floor the wheel is 20 to 40 % slower. 0.08 is only 0.02 above the deadband, so slow turns are weak.
- Floor fast runs overshoot to about 2300 to 2700 edges/s when a press starts (target 2000); slow runs overshoot about 900 on 800.
- Floor adds about 0.05 duty for the same speed (weight and rolling friction).
- Encoder noise grows with speed; two single-sample spikes to about -2700 edges/s on the supported fast reverse run (7.9 s and 9.3 s).
- Safety: unplugging the controller mid-run stopped both motors and ended the program, as required.
- Log timing: average row spacing 20.0 ms in every run, single gaps 17 to 23 ms.
- Limits of single-encoder feedback: only the left wheel is measured; the right wheel gets the same command. See the end of `lab_06/REPORT.md`.

---

## Lab 07: balance, speed to pitch, gamepad (in progress)
Code: `lab_07/` (branch `lab_07`). Orientation and checks: `~/cc_reports/lab07_step1_orientation.md`, `~/cc_reports/lab07_gain_and_cutoff_check.md`. Plot script: `lab_07/plot_lab07.py`.

### Off-robot checks (no hardware) 11/11
Run: `cd lab_07 && make check`. Result: 31 of 31 pass, build clean with `-Wall -Wextra`.
They cover: motors off before the 0.5 s arming hold and live after; forward lean drives forward and backward lean drives back; P output = Kp * error; D output = Kd * pitch rate; left/right never exceed 0.60; steering splits the wheels by 2 * 0.08; speed command steps and clamps at 2000; pitch target clamps to theta0 + 3 and to half the cutoff; deadband compensation lifts a small output and gives zero at zero error; integral limited to 0.15 duty; block-rest trip after 1 s near either stop but not while moving or at theta0; no block-rest trip when no stops are saved; cutoff trips and latches (forward, back, NaN); the shared values give zero +4.9975, stops +/-7.81, cutoff +/-15, theta0 5.81 deg inside the forward stop.

### Gain and cutoff consistency check 11/11
- Kp 0.015 and Kd 0.005 in motor terms (Km 6073): 1 deg asks for 91 edges/s, 1 deg/s asks for 30 edges/s; derivative time Kd/Kp 0.33 s, about 8 times the 40 ms motor lag.
- Deadband: P alone is below 0.06 duty until the error passes 4 deg, D alone until 12 deg/s. Deadband compensation (0.06, ramped over 0.02) added to the wheel commands.
- D path lag (estimated, not measured): about 70 to 80 ms (10 ms difference + 0.3 filter, 40 ms motor, 10 ms IMU update). Lab 05 oscillated at 5 to 8 Hz (125 to 200 ms), so this lag is a large part of a period.
- A pitch-model stability check was not possible (no usable model). A simulation using the unusable Lab 05 fit made every gain fall, so it was discarded.

### Robot checks reported by Josue (11/11)
- `sudo ./lab07 --cal` ran on the robot: forward stop +12.81, back stop -2.81, zero +4.997 (see "Mechanical stops").
- Josue reported the sensor and motor-direction checks (handout steps 1 and 2) done. The results (pitch sign, encoder sign, motor direction) were not recorded in the channel or on the Jetson at the time this file was written. **Add them here.**
- No `lab07_*.csv` run files existed on the Jetson at that time, so no balance run has been logged yet.

### 11/11 first robot runs of `sudo ./lab07`: nothing happened
- Four runs (13:50 to 13:52) left header-only CSVs (`lab07_20261111_1350*.csv`, `..._1351*.csv`, `..._135212.csv`, 103 bytes each, root-owned), meaning the program ended before the first 20 ms row.
- Cause (my code): before arming, a pitch outside +/-15 deg was treated as a tip-over and ended the run; a "frozen IMU" check could also fire on a robot at rest (the IMU repeats exact values, the back stop read "moved 0.00 deg" for 1 s). Both removed: out-of-range pitch before arming now just keeps the motors off, and a failed IMU read (pitch near 175 deg) trips the cutoff once armed. A status line now prints every second (pitch, speed, waiting / armed / LIVE).
- The gamepad seen on 11/11 is a "ZhiXu Gamepad" (USB 0079:181c, `/dev/input/event11`, js0), keys 0x130 to 0x13e, axes X Y Z RZ GAS BRAKE and HAT0X/HAT0Y. Josue reports X and Y are swapped compared with the Lab 06 codes (X = BTN_WEST). Calibration now accepts either BTN_WEST or BTN_NORTH as X.

### 11/11 first logged runs (new controller, wheels in the air)
Josue replaced the gamepad because the earlier one was not working (the ZhiXu pad was seen on 11/11 before that; check `/proc/bus/input/devices` for what is plugged in now). Three runs were logged and plotted
(`lab_07/lab07_20261111_135442.csv`, `..._135454.csv`, `..._135511.csv` with matching .png; CSVs are not in git). The motors went live, so the arming step and the loop work.

| Run | Length | Live | Pitch while live (deg) | Mean abs speed error | At the 0.60 clamp | Ended |
|---|---|---|---|---|---|---|
| 135442 | 5.0 s | 4.5 s (from 0.54 s) | -13.1 to +1.8 | 628 edges/s | 0.4 % | pitch -13.1, PWM -0.60 |
| 135454 | 11.6 s | 3.2 s (from 8.4 s) | -14.6 to +2.2 | 528 edges/s | 2.5 % | pitch -14.6, PWM -0.58 |
| 135511 | 24.2 s | 8.5 s (from 15.7 s) | -13.9 to +5.4 | 1300 edges/s | 10.3 % | pitch -13.9, PWM -0.60 |

- Signs: PWM follows pitch (correlation +0.72 to +0.91, positive pitch gives positive duty, as designed) and the measured encoder speed follows the PWM with the same sign (correlation +0.85 to +0.91 at 0 to 60 ms lag in the first two runs). So the encoder sign matches the duty sign. Whether positive pitch really is a forward lean, and whether the wheel really turns forward, still needs a visual confirmation from Josue.
- Every run ended with the pitch near -13 to -15 deg and the PWM at about -0.60: the cutoff at -15 from the zero cuts the motors there. With the wheels in the air the motors cannot change the pitch, so this is expected for this setup.
- Run 135511 shows a fast ripple (dominant pitch period 0.18 s, about 5.5 Hz, amplitude about 4 deg) and a PWM-to-speed correlation that turns negative at a 100 ms lag (-0.71) and positive again at 200 ms (+0.90). That is the lag Lab 05 also showed (5 to 8 Hz); the estimated 70 to 80 ms loop lag fits it. The robot was being moved by hand, so this is not a balance result yet.
- Not yet recorded: a run with the robot on the floor, speed-command steps, steering, the P / P+D / P+D+I tuning steps and the disturbance run.

### Joystick control (replaces the D-pad) 11/11
- Left stick up/down = forward/reverse speed command (proportional, up to 2000 edges/s, ramps at 3000 edges/s per second); right stick left/right = steering (full deflection = 0.08 duty difference); B = stop until both sticks are centred; START = exit.
- Off-robot checks: `cd lab_07 && make check` (37 checks pass, includes the stick mapping, ramp and clamps).
- Gamepad mapping test (no motors, no IMU): `cd lab_07 && make joystick_test && ./joystick_test` (guided PASS/FAIL for rest, each stick direction, B and START) or `./joystick_test live`.
- Controller seen on 11/11 afternoon: Bluetooth "Gamepad" (Bus 0005, 1949:0402, `/dev/input/event12`, js0).
- First run of the guided test with nobody touching the sticks: rest check PASSED (all four axes at 0.00), the push steps timed out as expected (no input). The real result with the sticks pushed is not recorded yet. **Add it here.**

### Balance in place first, steering that veers (11/11)
- `sudo ./lab07` now ignores the sticks by default: it only tries to hold the balance angle (speed and steering stay zero). `--drive` turns the sticks on. `--kv 0` removes the speed-to-pitch hold (pure pitch PID); `--kp/--ki/--kd` set the gains.
- Steering strength now scales with the speed command (full at 1000 edges/s), so the robot veers while moving and does not pivot on the spot; at a standstill the right stick does nothing.
- Off-robot checks: 42 pass (standstill gives equal wheel commands, half speed gives half strength, reverse works, Kv 0 keeps the pitch target at theta0).
- Robot results (14:14 to 14:15, Kp 0.015, Ki 0, Kd 0.005, balance in place, sticks ignored, so target speed and steering stayed 0 as designed):

| Run | Live | Pitch error (deg) | Longest time within +/-5 deg of the target | Ripple | Ended |
|---|---|---|---|---|---|
| 141458 | 5.5 s | std 3.04, max 12.9 | 4.7 s | 0.16 s period, 4.0 deg | pitch +14.9, PWM +0.46 (fell forward) |
| 141521 | 28.3 s | std 1.57, max 10.2 | 28.3 s (the whole live time) | 0.17 s period, 2.4 deg | pitch +12.2, PWM +0.60 (fell forward) |

  The 28 s run is the first sustained balance on this robot in Lab 07 (Lab 05's best hold was 21.9 s). Both runs ended falling forward with the PWM at or near the clamp. The fast ripple (about 6 Hz) is the same one seen in Lab 05 and fits the estimated 70 to 80 ms loop lag.
- Earlier runs at 14:08 to 14:12 (Kp 0.015, Ki 0, Kd 0 and 0.005, sticks on, older code) logged target speeds of +/-2000 edges/s and left/right PWM differences up to 0.16, so the sticks worked then.

### Tuning window (`sudo -E ./lab07 --gui`), 11/11
- Sliders and number boxes for balance angle, Kp, Ki, Kd and Kv change them while the robot runs; status readout (waiting / armed / LIVE / TRIPPED, pitch, speed, duty); Re-arm after a trip (the run continues); Reset to defaults; every change is written to `lab07_<time>_gui_changes.csv`. Run it on the Jetson's own screen (`sudo -E` keeps the display settings). Off-robot checks: 47 pass. Not yet run on the robot.

### Angle and gains from the command line
`sudo ./lab07 --theta0 3 --kp 0.02 --ki 0 --kd 0.002 --kv 0` (add `--drive` for the sticks, `--gui` for the window). The log name now contains the angle and the gains, for example `lab07_<time>_th3_kp0.02_ki0_kd0.002.csv`. An angle within 2.5 deg of a block stop, or beyond half the cutoff, is refused before anything starts (with the saved stops +/-7.81 deg the allowed range is -5.31 to +5.31 deg).

### Angle sweep runs (14:26 to 14:34, `--theta0` from -4 to +3, Kp 0.02 to 0.03, Ki 0) and the "is the polarity off?" question
- Josue reported: at different angles the robot barely moves, and with a negative angle it keeps moving forward even when it sits at a positive angle.
- Evidence the control polarity is right: in every run pitch and PWM correlate +0.73 to +0.96 (forward lean gives forward duty), wheel speed follows the PWM (+0.42 to +0.98), and the same law held the robot for 45 s at theta0 = 2 and 51 s at theta0 = 0.
- In every run the mean pitch while live equals the target (target +3 gave +2.9 and +3.2; -2 gave -1.8 to -3.4; -4 gave -3.0 to -3.5) with mean PWM near 0. That is partly by construction: the motors only go live after the robot is held within +/-2 deg of the target for 0.5 s.
- The best no-drift run is theta0 = 0 (51 s, mean PWM 0.000, mean wheel speed -1 edges/s, mean pitch -0.2): the saved zero (midpoint of the stops) is already close to the real balance point.
- A tilt other than the real balance point cannot be held still; the wheels must keep accelerating toward the lean, so large angle offsets make the robot drive off or fall (speed at the end of the runs was +/-1000 to 1800 edges/s).
- Still unconfirmed by eye: that positive pitch is a lean toward the robot's front and positive duty drives it forward.

### Gain comparison from the logged runs (11/11, wheels on the floor, balance in place, theta0 2)
Every log so far has Ki = 0: **no run with all three of P, I and D exists yet.**

| Gains (Kp / Ki / Kd) | Runs | Live time | Longest hold within +/-5 deg of target | Pitch std | Time at 0.60 clamp | Ripple | How it ended |
|---|---|---|---|---|---|---|---|
| 0.015 / 0 / 0 | 140831, 140858, 141203 | 10.6, 45.8, 23.7 s | 10.5, 44.9, 23.4 s | 1.1 to 1.15 deg | 0 % | slow wobble, period 1.9 s (140858), speed std about 390 edges/s | fell backward to -12.6 to -14.9 deg at duty only 0.30 to 0.33 |
| 0.015 / 0 / 0.005 | 141458, 141521 | 5.5, 28.3 s | 4.7, 28.3 s | 1.6 to 3.0 deg | 0.1 to 0.7 % | 0.17 s period (about 6 Hz), speed std 830 | fell forward to +12 to +15 deg, duty 0.46 to 0.60 |
| 0.015 / 0 / 0.005, sticks on (`_drive`) | 141824, 141903 | 10.5, 37.4 s | 3.7, 8.2 s | 2.7 to 2.95 deg | 10 to 11 % | 0.19 s period | cutoff at +14.5 (duty 0.56) and -14.3 (duty 0.60) |

- P only held the robot best: the 44.9 s hold is the longest in any lab, with low spread and no clamping. Adding Kd 0.005 made the pitch noisier (6 Hz ripple, speed noise doubled) and shorter holds. This matches Lab 05, where Kd 0.015 chattered; the estimated 70 to 80 ms loop lag (10 ms rate window, 0.3 filter, 40 ms motor) is the likely cause. Next test: Kd 0.001 to 0.002.
- The mean pitch while live was +1.4 to +2.2 deg in the balance-in-place runs, close to the 2 deg target.
- Falls were sudden (0.1 to 0.9 s from inside +/-3 deg to the cutoff), so they were not slow drifts; the cause is not known. The P-only falls were backward at only 0.3 duty, so a higher Kp (0.02) is worth a try.
- Stick runs: commands of -2000 to 0 and 0 to +2000 edges/s; steering was used very little (left/right PWM differed by at most 0.036). Holds were shorter than in place (3.7 and 8.2 s), but these are two runs only.

### Still to do
Motor polarity and sensor results, tuning (P only, then P+D, then P+D+I, with Kp, Ki, Kd recorded for each), required runs (stationary balance, forward, reverse, left turn, right turn, small-disturbance recovery), plots (`python3 plot_lab07.py lab07_*.csv`), wireless gamepad.

---

## Open issues across labs
1. Gamepad works wired only; wireless not paired.
2. No usable pitch model (a1, a2, c NOT SET); state feedback is not available.
3. `get_imu_pitch()` cannot report a read failure (returns 0.0).
4. IMU pitch-rate axis and sign (`IMU_PITCH_RATE_AXIS`, `_SIGN`) are NOT SET; the code estimates the rate from a 10 ms pitch difference instead.
5. Encoder speed spikes and occasional wrong-sign samples.
6. Right wheel has no encoder feedback in Lab 06 and Lab 07 (encoder 0 only).
7. Lab 07 results of the robot checks need to be added above.
