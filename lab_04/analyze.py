#!/usr/bin/env python3
"""Post-lab analysis. Run from lab_04/ after collecting the CSVs:
    python3 analyze.py
Produces step_response.png and speed_control.png (do NOT submit the CSVs).
Every metric is computed for both wheels (0 and 1) so they can be compared."""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.io import savemat

WHEELS = (0, 1)

# ---------------- Step response ----------------
df = pd.read_csv("step_response.csv")
t = df.time_ms / 1000.0

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
axes[0].plot(t, df.duty_cycle); axes[0].set_ylabel("duty cycle")
axes[0].set_title("Step response, both wheels")
for w in WHEELS:
    axes[1].plot(t, df[f"speed_{w}"], label=f"wheel {w}")
    axes[2].plot(t, df[f"position_{w}"], label=f"wheel {w}")
axes[1].set_ylabel("speed (encoder edges/s)")
axes[2].set_ylabel("position (counts)"); axes[2].set_xlabel("time (s)")
for a in axes:
    a.grid(True)
axes[1].legend(); axes[2].legend()
fig.savefig("step_response.png", dpi=150)

def step_metrics(w, t0, t1, duty):
    """Rise (10-90%), settling (within 2% of final), steady-state, gain.
    t0/t1: window of the step in seconds. TODO: adjust if your timing differs.
    Speed is flipped to the direction of travel first, so forward and
    reverse (and a wheel that is mirrored) use the same formulas."""
    m = (t >= t0) & (t < t1)
    ts, s = t[m].values, df[f"speed_{w}"][m].values
    ss = np.mean(s[-100:])                      # last 100 ms = steady state
    sn = s * np.sign(ss)
    mag = abs(ss)
    rise = ts[np.argmax(sn >= 0.9*mag)] - ts[np.argmax(sn >= 0.1*mag)]
    out = np.where(np.abs(sn - mag) > 0.02*mag)[0]
    settle = ts[out[-1]] - t0 if len(out) else 0.0
    return dict(steady_state=ss, rise_s=rise, settle_s=settle,
                gain_speed_per_duty=ss/duty)

# Windows from the Part 6 sequence (steps start at 1 s, 4 s, 5 s)
for w in WHEELS:
    print(f"--- wheel {w} ---")
    print("forward :", step_metrics(w, 1.0, 2.0, +0.60))
    print("reverse :", step_metrics(w, 4.0, 5.0, -0.60))
    print("forward2:", step_metrics(w, 5.0, 6.0, +0.60))
    # Encoder noise: std dev of speed during a steady window, and at rest
    m = (t >= 1.5) & (t < 2.0)
    r = (t >= 0.0) & (t < 1.0)
    print("noise std (steady fwd):", df[f"speed_{w}"][m].std(),
          "| at rest:", df[f"speed_{w}"][r].std())

# ---------------- Triangular speed control ----------------
sc = pd.read_csv("speed_control.csv")
fig, axes = plt.subplots(len(WHEELS), 1, sharex=True, figsize=(10, 7))
for w, a in zip(WHEELS, axes):
    a.plot(sc.time_ms/1000, sc.target_speed, label="target")
    a.plot(sc.time_ms/1000, sc[f"measured_speed_{w}"], label="measured")
    a.set_ylabel(f"wheel {w} speed"); a.legend(); a.grid(True)
    rms = np.sqrt(np.mean((sc.target_speed - sc[f"measured_speed_{w}"])**2))
    print(f"wheel {w} RMS tracking error: {rms:.2f}")
axes[0].set_title("Triangular speed reference, 3 cycles")
axes[-1].set_xlabel("time (s)")
fig.savefig("speed_control.png", dpi=150)
print("cycles:", len(sc) / 2000)

# ---------------- Export for MATLAB ----------------
# load('lab04_data.mat') gives step, speed_control and deadband structs
# (one array per CSV column; deadband rows are [motor, duty], NaN = not found).
mat = {"step": {c: df[c].values for c in df.columns},
       "speed_control": {c: sc[c].values for c in sc.columns}}
if os.path.exists("deadband.csv"):
    db = pd.read_csv("deadband.csv")
    mat["deadband"] = {
        "positive": [db[(db.motor == w) & (db.direction == "positive")].deadband_duty.iloc[0] for w in WHEELS],
        "negative": [abs(db[(db.motor == w) & (db.direction == "negative")].deadband_duty.iloc[0]) for w in WHEELS],
    }
savemat("lab04_data.mat", mat)
print("wrote lab04_data.mat")
