# Lab 04 Report -- Motor Characterization and Speed Control

Both wheels were tested together. Speed is the encoder module's signed speed in
A-edges (counts) per second. Wheel 0's direction line is inverted in `pwm.c` because
the motors are mounted as mirror images, so +duty turns both wheels the same way.
Plots are attached separately.

## Deadband
| | Wheel 0 | Wheel 1 |
|---|---|---|
| positive_deadband | +0.06 | +0.06 |
| negative_deadband | -0.06 | -0.04 |

Duty was raised from 0.00 in steps of 0.02 (1 s per level). The deadband is the first
duty that moved the wheel 5 or more encoder counts. These values are used as deadband
compensation in `modules/speed_control.c`.

## Analysis

**Motor model (step response, +/-0.60 duty).** Both wheels behave as first-order
systems. The gain is about 6100 edges/s per unit duty (W0 6100 forward / 5904 reverse,
W1 6139 / 6147). The time constant is about 40 ms, and the 10-90% rise time is 53-67 ms.
Wheel 1 is symmetric. Wheel 0 is about 3% weaker in reverse. The two wheels agree within
about 3%, and their net travel over the whole sequence differs by 1% (3295 vs 3327 counts).

**Encoder noise.** At steady speed the standard deviation is 420 (W0) and 507 (W1)
edges/s, which is 11-14% of the speed. Speed is computed from the time between single
A edges, so one irregular interval produces a spike, and W1 sometimes reads the wrong
direction. Because the noise is larger than a 2% band (and also 5%, 10% and 20% bands),
no settling time could be measured.

**Speed control (PI, triangle +/-1000 edges/s, 3 cycles).** Gains KP = 0.0001 and
KI = 0.0025, with the deadband compensation above. KP was chosen so the loop gain
K*KP is about 0.6 with K = 6100, and KI = KP/tau matches the integral to the 40 ms motor
time constant. A first attempt with KP = 0.005 had a loop gain of about 30, and the duty
switched between its limits without tracking.

| | Wheel 0 | Wheel 1 |
|---|---|---|
| Correlation, target vs measured | 0.98 | 0.98 |
| RMS tracking error (edges/s) | 119 (11.9% of amplitude) | 123 (12.3%) |
| Lag behind the target | 47 ms | 49 ms |

Both wheels follow the triangle closely, with all 3 cycles completed (6000 samples). The
lag is close to the motor time constant, as expected for a PI controller with this KP.
The error is largest at the peaks, where the target reverses direction (RMS about 170 for
|target| > 800 vs about 90 for |target| < 200). Most of the remaining error is the
11-14% encoder noise, not controller error.

At the handout's +/-100 edges/s, the controller needed only about 0.02 duty above the
deadband, so the wheels barely turned and the noise dominated (correlation 0.56-0.58).
The +/-1000 amplitude was used so that the tracking could be judged.
