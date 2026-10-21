# Lab 04 Report -- Motor Step Response Characterization

Both wheels were tested together. Speed is the encoder module's signed speed in
A-edges (counts) per second. Wheel 0's direction line is inverted in `pwm.c`
because the motors are mounted as mirror images, so +duty is the same direction
on both wheels.

## Deadband
| | Wheel 0 | Wheel 1 |
|---|---|---|
| positive_deadband | +0.06 | +0.06 |
| negative_deadband | -0.06 | -0.04 |

(From `sudo ./lab04 deadband`; step size 0.02, 1 s per level, "moving" = 5 counts of travel.)

## Step response (plot: step_response.png)
Metrics from the logged run, +/-0.60 duty:

| Metric | W0 Forward (+0.60) | W0 Reverse (-0.60) | W1 Forward (+0.60) | W1 Reverse (-0.60) |
|---|---|---|---|---|
| Rise time (10-90%) | 0.067 s | 0.053 s | 0.061 s | 0.057 s |
| Settling time (2%) | not reached | not reached | not reached | not reached |
| Steady-state speed | 3660 | -3542 | 3684 | -3688 |
| Motor gain (speed/duty) | 6100 | 5904 | 6139 | 6147 |

Time constant (63.2% of final): about 41 ms on both wheels.
Settling time: the speed noise (below) is larger than the 2% band (about 70 edges/s), so the
speed never stays inside it. Band used: 2% of steady-state speed. Looser bands of 5%, 10% and
20% were also checked, and the speed still left each band within the last 0.05 s of every 1 s step,
so no settling time is reported.

Forward vs reverse comparison: Wheel 1 is symmetric (6139 forward, 6147 reverse). Wheel 0
is about 3% lower in reverse (5904 vs 6100) and on the repeat forward step (5942).
Wheel 0 vs wheel 1 comparison: gains agree within about 3%; rise times are within 15 ms;
net position after the sequence was 3295 counts (W0) and 3327 counts (W1).

## Encoder noise
Std dev at steady state (1.5-2.0 s of the forward step): W0 420, W1 507 edges/s (11-14% of the
steady-state speed). At rest the speed reads exactly 0. In the steady window the speed ranges
2846 to 6108 (W0) and -2446 to 8373 (W1). Speed comes from the time between single A edges, so
one irregular interval gives a large spike, and W1 occasionally reads the wrong direction.

## Speed control (plot: speed_control.png)
Gains used: KP=0.0001 KI=0.0025. Deadband compensation used (W0 / W1): +0.06/-0.06 / +0.06/-0.04.
First run (KP=0.005, KI=0.020): did not track. Loop gain K*KP was about 30 with the measured
K = 6100, so the duty switched between the limits.
Tracking quality, lag, 3 cycles confirmed (rerun 10/21 13:08 with the new gains and deadband
compensation): 6000 samples = 3 cycles of 2 s, confirmed.

| | Wheel 0 | Wheel 1 |
|---|---|---|
| RMS tracking error (edges/s) | 82.2 | 78.8 |
| Correlation, target vs measured | 0.56 | 0.58 |
| Lag (peak of cross-correlation) | 29 ms | 27 ms |
| Measured speed range (edges/s) | -220 to 180 | -171 to 196 |

For reference, the RMS of the +/-100 triangle target itself is 57.7 edges/s.
