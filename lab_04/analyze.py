#!/usr/bin/env python3
"""Post-lab analysis. Run from lab_04/ after collecting the CSVs:
    python3 analyze.py
Produces step_response.png and speed_control.png (do NOT submit the CSVs)."""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------- Step response ----------------
df = pd.read_csv("step_response.csv")
t = df.time_ms / 1000.0

fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
a1.plot(t, df.duty_cycle); a1.set_ylabel("duty cycle")
a2.plot(t, df.speed);      a2.set_ylabel("speed (encoder units)")
a2.set_xlabel("time (s)"); a1.set_title("Step response")
for a in (a1, a2): a.grid(True)
fig.savefig("step_response.png", dpi=150)

def step_metrics(t0, t1, duty):
    """Rise (10-90%), settling (within 2% of final), steady-state, gain.
    t0/t1: window of the step in seconds. TODO: adjust if your timing differs."""
    m = (t >= t0) & (t < t1)
    ts, s = t[m].values, df.speed[m].values
    ss = np.mean(s[-100:])                      # last 100 ms = steady state
    rise = ts[np.argmax(s >= 0.9*ss)] - ts[np.argmax(s >= 0.1*ss)]
    out = np.where(np.abs(s - ss) > 0.02*abs(ss))[0]
    settle = ts[out[-1]] - t0 if len(out) else 0.0
    return dict(steady_state=ss, rise_s=rise, settle_s=settle,
                gain_speed_per_duty=ss/duty)

# Windows from the Part 6 sequence (starts at 1 s, 4 s, 5 s)
print("forward:", step_metrics(1.0, 2.0, +0.60))
# TODO: reverse window uses negative speeds only if you sign the speed
print("reverse:", step_metrics(4.0, 5.0, -0.60))

# Encoder noise: std dev of speed during a steady window
m = (t >= 1.5) & (t < 2.0)
print("noise std (steady fwd):", df.speed[m].std())

# ---------------- Triangular speed control ----------------
sc = pd.read_csv("speed_control.csv")
plt.figure(figsize=(10, 4))
plt.plot(sc.time_ms/1000, sc.target_speed,   label="target")
plt.plot(sc.time_ms/1000, sc.measured_speed, label="measured")
plt.xlabel("time (s)"); plt.ylabel("speed"); plt.legend(); plt.grid(True)
plt.title("Triangular speed reference, 3 cycles")
plt.savefig("speed_control.png", dpi=150)
print("cycles:", sc.time_ms.max()/2000)
