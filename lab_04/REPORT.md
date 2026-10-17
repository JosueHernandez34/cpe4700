# Lab 04 Report -- Motor Step Response Characterization

## Deadband
| | Wheel 0 | Wheel 1 |
|---|---|---|
| positive_deadband | ___ | ___ |
| negative_deadband | ___ | ___ |

## Step response (plot: step_response.png)
| Metric | W0 Forward (+0.60) | W0 Reverse (-0.60) | W1 Forward (+0.60) | W1 Reverse (-0.60) |
|---|---|---|---|---|
| Rise time (10-90%) | | | | |
| Settling time (2%) | | | | |
| Steady-state speed | | | | |
| Motor gain (speed/duty) | | | | |

Forward vs reverse comparison: ___
Wheel 0 vs wheel 1 comparison: ___

## Encoder noise
Std dev at steady state (W0 / W1): ___ / ___ ; observations (quantization at low speed, spikes): ___

## Speed control (plot: speed_control.png)
Gains used: KP=___ KI=___. Deadband compensation used (W0 / W1): ___ / ___.
Tracking quality, lag, RMS error (W0 / W1), 3 cycles confirmed: ___
