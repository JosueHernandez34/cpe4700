#!/usr/bin/env python3
"""Plot one speed-control run, e.g. a higher-amplitude rerun:
    python3 plot_speed.py speed_control_1000.csv
Writes <csv name>.png next to the CSV (speed_control_1000.png) and prints
RMS error, correlation and lag per wheel. analyze.py is left as is, so the
handout run's speed_control.png is not overwritten."""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

WHEELS = (0, 1)

csv = sys.argv[1] if len(sys.argv) > 1 else "speed_control.csv"
sc = pd.read_csv(csv)
t = sc.time_ms / 1000.0
target = sc.target_speed.values
amp = np.abs(target).max()

fig, axes = plt.subplots(len(WHEELS), 1, sharex=True, figsize=(10, 7))
for w, a in zip(WHEELS, axes):
    meas = sc[f"measured_speed_{w}"].values
    a.plot(t, target, label="target")
    a.plot(t, meas, label="measured")
    a.set_ylabel(f"wheel {w} speed (edges/s)"); a.legend(); a.grid(True)

    rms = np.sqrt(np.mean((target - meas) ** 2))
    corr = np.corrcoef(target, meas)[0, 1]
    # Lag = shift (0-300 ms) where measured best lines up with the target.
    cc = [np.corrcoef(target[:len(target) - L], meas[L:])[0, 1] for L in range(301)]
    lag = int(np.argmax(cc))
    print(f"wheel {w}: RMS error {rms:.1f} edges/s ({100 * rms / amp:.1f}% of amplitude), "
          f"corr {corr:.2f}, lag {lag} ms")

axes[0].set_title(f"Triangular speed reference +/-{amp:.0f} edges/s, {len(sc) / 2000:.0f} cycles")
axes[-1].set_xlabel("time (s)")
out = csv.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=150)
print("wrote", out)
