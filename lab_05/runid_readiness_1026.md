# Run ID readiness check (10/26, read only)

Nothing was run, built, committed or pushed. No motors. This is the only file written.

## 1. Ready? Yes, on the v1 files
- `robotest` (built 10/22 00:17) is newer than every source it uses (balance_loop.c, gui.c, state_space.c, modules/imu.c, bno055.c, pwm.c, encoder.c), so no rebuild is needed.
- GTK 4.6.9, numpy, pandas and scipy are all present.
- Command (start it from lab_05 or the live fit cannot find identify.py):
  `cd ~/dev/cpe4700/lab_05 && sudo ./robotest`
- In the GUI: gains at the launch defaults (Kp 0.015, Ki 0.005, Kd 0.005), Max duty 0.30, ID step 0.08 (range 0.02 to 0.20). Press **Run ID (20 s)**. **That button turns the motors ON by itself** if they are off (gui.c:201).

## 2. Cutoff and band in force
- **Correction:** the cutoff is **+6 / -5 deg**, not +6 / -4. balance_loop.c:205-206 enforces a 5 deg minimum, so the backward side becomes max(5, 8 - 4) = 5. That is the "+6/-5" Josue remembered.
- v1 has no standing band. The -8 / +9 band only exists in the planned v2. In v1, identify.py drops any 0.5 s segment that goes beyond stop - margin = **+6 / -4** (taken from ss_params.py).
- Injection runs only while pitch is between **+3.6 and -3.0 deg** (60 % of +6 / -5). Outside that range the steps pause, but the motors keep running.
- Assessment: these limits are tight. The 10/22 run had a pitch standard deviation of 3.8 deg, so with these limits the steps would pause often, the latch could trip, and many segments would be dropped. A first run will still show whether the robot stays inside the limits. Widening them means editing the stop values in ss_params.py, rerunning ss_design.py to regenerate ss_model.h, and rebuilding. That is a code change, so it is listed as a decision and nothing was changed.

## 3. Signal, duration, runs, duty, setup
- Signal: a 9-bit pseudo-random sequence (shift register, taps 9 and 5, same pattern every run). Each level is held 150 ms. The four levels are +/-A and +/-0.6A, with A = ID step (0.08 by default). The steps are added on top of the PID output.
- One press gives one 20 s run. The total duty is clamped to Max duty (0.30).
- Runs per launch: the log holds 120 s of enabled time (1 ms rows) and is written only at Quit or Ctrl-C. identify.py takes the span from the first nonzero injection to the last, so two Run IDs in one log merge into one window. Recommendation: **one Run ID per launch**, quit, then relaunch. Three runs in total.
- Setup: **the code was designed for the robot balancing on the floor under its own gains** (balance_loop.c:110). The unknown coefficients a1 (gravity) and c (wheel-to-pitch coupling) only show up when the body is free to tip. With the wheels in the air, only the motor terms (Km, tau) can be fitted. If someone's hand holds the robot, the fit models the hand. The block sits under the chassis and catches the robot at about +10 / -8 deg.

## 4. Files written and how to fit
- `lab_05/balance_log_HHMMSS.csv` is written at Quit. It is owned by root because of sudo. Columns: time_ms, pitch_deg, rate_dps, duty, speed_0, speed_1, inj_duty, count_0, count_1.
- During the run, the GUI writes a temporary snapshot and runs `identify.py --live` to show the live fit. The temporary files are named by process id.
- Fit command: `python3 identify.py balance_log_HHMMSS.csv --kp 0.015 --kd 0.005`. It prints the fit and K and writes `balance_log_HHMMSS_model.json`.
- Copying a1, a2 and c into ss_params.py and running ss_design.py is a separate step that needs approval.

## 5. Missing or caveats
- pitch_probe has not been run. The stops +10 / -8 are a rough reading.
- The model is NOT SET (a1, a2, c), so the robot runs on PID. That is expected for a Run ID.
- The gyro pitch-rate functions are untested on hardware. balance_loop.c does not use them, so they have no effect on this run.
- **There is already a fit from 10/22 00:05** (balance_log_000508_model.json). One-step R^2 is 0.997 / 0.85 / 0.88 and the 1 s free-run pitch RMS is 1.0 deg. But it gives a1 = -0.41, and a negative a1 means the data does not look like a free inverted pendulum (the robot was probably supported). It also used the 25 deg interim stop, and it was never copied into ss_params.py.
- Unreported file: modules/gamepad_drive.c and its binary (10/26 15:45) drive the motors straight from the left stick. They are not in any channel post. They were not run in this check.
