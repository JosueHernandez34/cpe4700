#!/usr/bin/env python3
"""Plot Lab 06 gamepad runs:
    python3 plot_lab06.py lab06_*.csv
For each CSV writes <csv name>.png (target vs measured speed, left and right
PWM) and prints tracking error and PWM range. A run where the target and both
PWMs stay at 0 is treated as a stationary baseline. If there is at least one
baseline and one moving run, also writes baseline_vs_moving.png.
Speed is the feedback (encoder) wheel only, in A-edges/s."""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

MODES = {0: "slow", 1: "fast"}

if len(sys.argv) < 2:
    sys.exit("usage: plot_lab06.py run1.csv [run2.csv ...]")

runs = []
for csv in sys.argv[1:]:
    d = pd.read_csv(csv)
    t = (d.time_ms - d.time_ms.iloc[0]) / 1000.0
    err = d.target_speed - d.measured_speed
    stationary = (d.target_speed == 0).all() and (d.left_pwm == 0).all() and (d.right_pwm == 0).all()
    runs.append((csv, d, t, stationary))

    dt = np.diff(d.time_ms)
    print(f"{csv}: {len(d)} rows, {t.iloc[-1]:.2f} s, row spacing {dt.mean():.1f} ms "
          f"(min {dt.min()}, max {dt.max()}), {'stationary baseline' if stationary else 'moving run'}")
    if stationary:
        print(f"  measured speed: mean {d.measured_speed.mean():.1f}, "
              f"std {d.measured_speed.std():.1f} edges/s")
    else:
        moving = d.target_speed != 0
        print(f"  tracking error, all rows: mean {err.mean():.1f}, mean |e| {err.abs().mean():.1f}, "
              f"max |e| {err.abs().max():.1f} edges/s")
        if moving.any():
            e = err[moving]
            print(f"  tracking error, target != 0: mean {e.mean():.1f}, mean |e| {e.abs().mean():.1f}, "
                  f"max |e| {e.abs().max():.1f} edges/s")
        for m, name in MODES.items():
            sel = moving & (d["mode"] == m)
            if sel.any():
                print(f"  {name} mode (target != 0): mean |e| {err[sel].abs().mean():.1f}, "
                      f"max |e| {err[sel].abs().max():.1f} edges/s, {sel.sum()} rows")
    print(f"  left_pwm  min {d.left_pwm.min():+.3f} max {d.left_pwm.max():+.3f}")
    print(f"  right_pwm min {d.right_pwm.min():+.3f} max {d.right_pwm.max():+.3f}")

    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    a1.plot(t, d.target_speed, label="target")
    a1.plot(t, d.measured_speed, label="measured (encoder wheel)")
    a1.set_ylabel("speed (edges/s)"); a1.legend(); a1.grid(True)
    a1.set_title(csv)
    a2.plot(t, d.left_pwm, label="left PWM")
    a2.plot(t, d.right_pwm, label="right PWM")
    a2.axhline(0.60, color="gray", ls="--", lw=0.8)
    a2.axhline(-0.60, color="gray", ls="--", lw=0.8)
    a2.set_ylabel("duty"); a2.set_xlabel("time (s)"); a2.legend(); a2.grid(True)
    out = csv.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("  wrote", out)

base = [r for r in runs if r[3]]
moving = [r for r in runs if not r[3]]
if base and moving:
    fig, ax = plt.subplots(figsize=(10, 4))
    for csv, d, t, _ in base:
        ax.plot(t, d.measured_speed, label=f"baseline: {csv}")
    for csv, d, t, _ in moving:
        ax.plot(t, d.measured_speed, label=f"moving: {csv}", alpha=0.8)
    ax.set_xlabel("time (s)"); ax.set_ylabel("measured speed (edges/s)")
    ax.set_title("Stationary baseline vs moving runs")
    ax.legend(fontsize=8); ax.grid(True)
    fig.savefig("baseline_vs_moving.png", dpi=150)
    print("wrote baseline_vs_moving.png")
