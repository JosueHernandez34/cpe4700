#!/usr/bin/env python3
"""Plot Lab 07 balance runs and print plain numeric summaries (no discussion):
    python3 plot_lab07.py lab07_*.csv
    python3 plot_lab07.py run.csv --disturb run.csv=12.5      # disturbance applied at 12.5 s
Writes <csv name>.png with four panels (target and measured speed, pitch and pitch target,
PID output, left and right PWM) and prints, for the part of the run where the motors were live:
duration, pitch error (mean, std, max), dominant oscillation period and amplitude of the pitch,
speed error (mean absolute, max), time at the PWM clamp, and overshoot (largest pitch swing past
the target after a speed step). With --disturb, also the recovery time: how long after the
disturbance until |pitch error| stays under 1 degree for 0.5 s.
Speed is the feedback encoder only, in A-edges/s. Pitch is degrees from the calibrated zero."""
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modules"))
import robot_params as rp          # shared values (modules/robot_params.h)

MAX_DUTY = rp.ROBOT_MAX_DUTY
DT = rp.ROBOT_LOG_PERIOD_MS / 1000.0   # log period, s
REC_BAND = 1.0      # deg
REC_HOLD = 0.5      # s

files, disturb = [], {}
args = sys.argv[1:]
i = 0
while i < len(args):
    if args[i] == "--disturb":
        i += 1
        name, _, t = args[i].partition("=")
        disturb[name] = float(t)
    else:
        files.append(args[i])
    i += 1
if not files:
    sys.exit("usage: plot_lab07.py run.csv ... [--disturb run.csv=seconds]")


def dominant_period(x):
    x = x - x.mean()
    if len(x) < 64:
        return float("nan"), float("nan")
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    freqs = np.fft.rfftfreq(len(x), DT)
    spec[freqs < 0.3] = 0                      # ignore slow drift
    k = int(np.argmax(spec))
    amp = 0.5 * (np.percentile(x, 95) - np.percentile(x, 5))
    return (1.0 / freqs[k] if freqs[k] > 0 else float("nan")), amp


for path in files:
    d = pd.read_csv(path)
    t = (d.time_ms - d.time_ms.iloc[0]) / 1000.0
    live = (d.left_pwm != 0) | (d.right_pwm != 0)
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    ax[0].plot(t, d.target_speed, label="target speed")
    ax[0].plot(t, d.measured_speed, label="measured speed", alpha=0.7)
    ax[0].set_ylabel("speed (edges/s)")
    ax[1].plot(t, d.pitch_deg, label="pitch")
    ax[1].plot(t, d.pitch_target_deg, label="pitch target", linestyle="--")
    ax[1].set_ylabel("pitch (deg from zero)")
    ax[2].plot(t, d.pid_output, color="tab:green", label="PID output")
    ax[2].set_ylabel("PID output (duty)")
    ax[3].plot(t, d.left_pwm, label="left PWM")
    ax[3].plot(t, d.right_pwm, label="right PWM", alpha=0.7)
    ax[3].set_ylabel("PWM duty")
    ax[3].set_xlabel("time (s)")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper right")
        if path in disturb:
            a.axvline(disturb[path], color="gray", linestyle=":")
    ax[0].set_title(path)
    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=110)
    plt.close(fig)

    print(f"\n{path}  ->  {out}")
    if not live.any():
        print("  motors never live (all PWM zero): no balance summary")
        continue
    L = d[live]
    tl = t[live]
    err = L.error_deg
    period, amp = dominant_period(L.pitch_deg.to_numpy())
    serr = (L.target_speed - L.measured_speed).abs()
    clamp = ((L.left_pwm.abs() >= MAX_DUTY - 1e-6) | (L.right_pwm.abs() >= MAX_DUTY - 1e-6)).mean() * 100
    print(f"  live time {tl.iloc[-1] - tl.iloc[0]:.1f} s of {t.iloc[-1]:.1f} s")
    print(f"  pitch error (deg): mean {err.mean():+.2f}, std {err.std():.2f}, max abs {err.abs().max():.2f}")
    print(f"  pitch oscillation: period {period:.2f} s, amplitude {amp:.2f} deg")
    print(f"  speed error (edges/s): mean abs {serr.mean():.0f}, max {serr.max():.0f}")
    print(f"  time at the {MAX_DUTY:.2f} PWM clamp: {clamp:.1f} %")
    steps = np.flatnonzero(np.diff(d.target_speed.to_numpy()) != 0)
    for s in steps[:3]:
        seg = d.iloc[s + 1:s + 1 + int(2.0 / DT)]
        if len(seg):
            dev = (seg.pitch_deg - seg.pitch_target_deg)
            print(f"  speed step at {t.iloc[s + 1]:.1f} s: largest pitch deviation from target {dev.abs().max():.2f} deg within 2 s")
    if path in disturb:
        td = disturb[path]
        sub = d[t >= td]
        ok = (sub.error_deg.abs() < REC_BAND).to_numpy()
        need = int(REC_HOLD / DT)
        rec = None
        for k in range(len(ok) - need):
            if ok[k:k + need].all():
                rec = t[sub.index[k]] - td
                break
        peak = sub.error_deg.abs().iloc[:int(2.0 / DT)].max()
        print(f"  disturbance at {td:.1f} s: peak pitch error {peak:.2f} deg, recovery time "
              + (f"{rec:.2f} s" if rec is not None else "not reached"))
