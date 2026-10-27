# Lab 05 Report -- Self-Balancing Module with GTK4 Display

The balance loop runs at 1 ms in a worker thread (`robotest.c`, `clock_nanosleep` on
`CLOCK_MONOTONIC`) and the GTK4 GUI runs in the main thread. The control law in
`balance_loop.c` is PID on IMU pitch:

    duty = DRIVE_SIGN * (Kp * e + Kd * pitch_rate + Ki * integral(e)),   e = pitch - target

The integral is clamped so `Ki * integral` alone can never exceed the duty limit
(anti-windup). The duty is clamped to the tuning limit, which is never above the
handout's 0.75, before `pwm_set_duty()`. A pitch cutoff latches the motors off if the
robot tips too far. On 10/21 the tuning limit was 0.30 (the launch default). On 10/26
(the v2 files) it was 0.60.

Plots are in `plots/` and are made by `python3 plot_gains.py`. Wheel speed is the encoder
module's A-edges/s converted to RPM with the handout's 700 counts/rev, after a 51 ms
median filter that removes single-sample encoder spikes. Each wheel's speed is shown
separately.

## How the gain settings were found

The logs do not store the gains. In PID mode every other term of the control law is
logged (duty, ID test step, pitch, pitch rate), so the gains were recovered by a
least-squares fit over 3 s windows. Windows with no saturation fit exactly (R^2 = 1.000),
so the values below are the gains the controller actually used, not estimates. Every
logged run used one of three settings:

| Setting | Kp (duty/deg) | Ki (duty/deg*s) | Kd (duty/(deg/s)) | Runs | Motors-on time used |
|---|---|---|---|---|---|
| A | 0.010 | 0.010 | 0.015 | 1 (10/21) | 4.5 s |
| B | 0.010 | 0.010 | 0.005 | 1 (10/21) | 7.8 s |
| C (launch default) | 0.015 | 0.005 | 0.005 | 13 (10/21 to 10/26) | 202 s |

The metrics below use only time with the motors on and no system-ID test steps. A
4-second 10/26 16:13 log (Ki 0.04) did not move the wheels and is not used.

## Results

| Setting | Pitch std (deg) | Time at duty limit | Longest hold within ±5 deg | Runs ended by cutoff |
|---|---|---|---|---|
| A | 2.4 | 72 % | 4.2 s | 0 of 1 (stopped by hand at 4.5 s) |
| B | 4.2 | 71 % | 5.3 s | 1 of 1 |
| C | 4.9 (median) | 18 % (median) | 21.9 s | 10 of 13 |

"Longest hold" is the longest continuous stretch with pitch within ±5 deg of that run's
median pitch.

**Fig. 1 (`fig1_gain_settings.png`): the three settings in the same 10/21 session.**
- **A (high Kd):** pitch held within a few degrees but oscillated at about 5 Hz. The
  wheels reversed every cycle between about +100 and -120 RPM, and the duty was at the
  0.30 limit 72 % of the time. The derivative term reacts strongly to the noisy pitch
  rate, so the command chatters between the limits. The run was too short to show
  whether it would have stayed up.
- **B (Kd cut to 0.005, same Kp and Ki):** the fast chatter died out after about 2 s, but
  the wheels then ran at a steady -145 RPM, held at the duty limit, while the robot
  leaned at about -7 deg. At 7.5 s it fell to -47 deg and the cutoff latched the motors
  off. With less damping and the output saturated, the controller had no reserve left
  to catch the fall.
- **C (higher Kp, lower Ki):** about 8 Hz oscillation, one jump from -8 to +10 deg, then
  a fall to about -25 deg with both wheels saturated at about -150 RPM. With the 0.30
  limit, all three settings spent much of the time at the limit.

**Fig. 2 (`fig2_default_gains_120s.png`): setting C, 120 s run on 10/26 with the 0.60
limit.** The first 20 s include the system-ID test steps (gray). After them the robot
held between about -15 and +15 deg for about 45 s, with slow drifts and a sudden
20 deg drop at 50 s. At 64 s it tipped past the cutoff and the motors stayed off for the rest of the
log. Between 35 s and 50 s, motor 1 ran at about 300 RPM while motor 0 ran at about
100 RPM on the same duty. The wheels were not equally loaded, which turns the robot
instead of correcting pitch.

**Fig. 3 (`fig3_summary.png`)** compares the three settings using the table above.

**Fig. 4 (`fig4_best_hold.png`): the best run, setting C on 10/26 17:04.** After the
system-ID steps the robot stayed between about 0 and +8 deg for 21.9 s, with a small
fast ripple and about 1 % of the time at the duty limit. At 44 s it dropped to -7.5 deg,
came to rest, and the cutoff latched.

## Effect of P, I and D

Only PID settings were logged. There are no P-only or PI-only runs, so the points below
about P and I alone are expected behavior, not measurements.

- **P (Kp)** sets how hard the wheels drive toward upright. Too low, and the robot leans
  further before the wheels catch it. In the 10/21 session, going from B to C (Kp 0.010
  to 0.015, Ki 0.010 to 0.005) did not hold the robot up longer (3.7 s vs 5.3 s) and
  added a faster ripple (about 8 Hz). C's long holds only came on 10/26, with the duty
  limit raised from 0.30 to 0.60, so the duty limit made a bigger difference than this
  Kp change. P alone is expected to oscillate with growing amplitude, because nothing
  damps the pendulum.
- **I (Ki)** removes a steady lean, such as an IMU zero offset (the 10/21 runs sit near
  -7 deg instead of 0). B (Ki 0.010) spent 71 % of the time at the duty limit and C
  (Ki 0.005) 49 % in the same session, but Kp changed too, so this cannot be put on Ki
  alone. Too much Ki with a saturated output winds up and tips the robot, which is why
  the integral is clamped.
- **D (Kd)** damps the fall. Measured: Kd 0.015 (A) gave the smallest pitch spread but a
  chattering 5 Hz output pinned at the limit. Kd 0.005 (B, C) removed the chatter but
  could not stop the slow falls. The pitch rate is noisy, so Kd much above 0.005 drives
  the motors hard on noise.

## Near-stable gain range

The only setting that produced long near-stable stretches is **Kp 0.015, Ki 0.005,
Kd 0.005 with a 0.60 duty limit** (21.9 s and 12.6 s holds, 1 to 5 % of the time at the
limit). Kp 0.010 with Ki 0.010 saturated the 0.30 limit most of the time, whatever Kd
was. With three gain settings logged, a range cannot be drawn more tightly than "around
the default". A sweep of Kp (0.010 to 0.025) and Kd (0.002 to 0.010) at the 0.60 limit,
plus a P-only and a PI run, would turn this into a measured range.
