#!/usr/bin/env python3
"""Plot a Lab 07 balancing run and print the numbers for the report.

    python3 plot_lab07.py lab07_*.csv
    python3 plot_lab07.py run.csv --disturb 12.0          # push at t = 12.0 s
    python3 plot_lab07.py run.csv --disturb 12.0 --window 4

For each CSV writes <csv name>.png with the five plots the handout asks for:
  1. target speed and measured speed vs time
  2. pitch angle and pitch-angle target vs time
  3. PID controller output vs time
  4. left and right PWM commands vs time
  5. response to a disturbance (zoomed pitch error and PID output)
Shaded bands mark when the robot was balancing (state 2).

Disturbance: give its time with --disturb (seconds from the start of the log).
Without it, the largest |error| while balancing with no speed command is used.
Recovery time = time from the peak until |error| stays inside the --band
(default 1 deg) for 0.5 s. Overshoot = largest error of the opposite sign
after the peak, as a percent of the peak.

The gains are read from the matching <csv name>_info.txt if it is there.
Speeds are encoder A-edges/s, angles deg, commands duty.
"""
import argparse
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("csv", nargs="+")
ap.add_argument("--disturb", type=float, help="disturbance time, s from log start")
ap.add_argument("--window", type=float, default=3.0, help="disturbance plot length, s")
ap.add_argument("--band", type=float, default=1.0, help="recovery band, deg")
args = ap.parse_args()


def read_info(csv):
    info = {}
    path = csv[:-4] + "_info.txt" if csv.endswith(".csv") else csv + "_info.txt"
    if os.path.exists(path):
        for line in open(path):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                info[k] = v
    return info


def segments(mask):
    """(start, end) index pairs where mask is True."""
    m = np.concatenate(([False], mask.values, [False])).astype(int)
    d = np.diff(m)
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1))


def shade(ax, t, active):
    for a, b in segments(active):
        ax.axvspan(t.iloc[a], t.iloc[b], color="tab:green", alpha=0.08, lw=0)


def dominant_freq(x, fs):
    x = np.asarray(x) - np.mean(x)
    if len(x) < 64:
        return float("nan")
    f = np.fft.rfftfreq(len(x), 1 / fs)
    p = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    p[f < 0.3] = 0                      # ignore slow drift
    return f[np.argmax(p)]


for csv in args.csv:
    d = pd.read_csv(csv)
    info = read_info(csv)
    t = (d.time_ms - d.time_ms.iloc[0]) / 1000.0
    active = d.state == 2
    dt = np.diff(d.time_ms)
    print(f"\n{csv}: {len(d)} rows, {t.iloc[-1]:.2f} s, row spacing mean {dt.mean():.2f} ms "
          f"(min {dt.min()}, max {dt.max()})")
    if info:
        print("  gains: Kp {kp}  Ki {ki}  Kd {kd}  Kv {kv}  Kvi {kvi}  theta0 {theta0_deg}  "
              "maxduty {maxduty}".format(**{k: info.get(k, "?") for k in
                                            ("kp", "ki", "kd", "kv", "kvi", "theta0_deg", "maxduty")}))

    # ---- summary numbers -------------------------------------------------
    bal = d[active]
    if len(bal):
        segs = segments(active)
        longest = max((t.iloc[b] - t.iloc[a]) for a, b in segs)
        still = bal[(bal.target_speed == 0) & (bal.u_steer == 0)]
        print(f"  balancing: {active.sum() * 0.02:.1f} s in {len(segs)} stretch(es), "
              f"longest {longest:.1f} s")
        if len(still) > 10:
            print(f"  stationary: pitch error mean {still.error_deg.mean():+.2f} deg, "
                  f"std {still.error_deg.std():.2f} deg, |error| max {still.error_deg.abs().max():.2f}; "
                  f"speed std {still.measured_speed.std():.0f} edges/s; "
                  f"dominant oscillation {dominant_freq(still.error_deg, 50):.2f} Hz")
        sat = (bal.left_pwm.abs() >= bal.left_pwm.abs().max() - 1e-3).mean() * 100
        print(f"  PID output: |u| mean {bal.pid_output.abs().mean():.3f}, max {bal.pid_output.abs().max():.3f}; "
              f"left PWM at its max {sat:.1f} % of the time")
        moving = bal[bal.target_speed.abs() > 0]
        if len(moving):
            for sgn, name in ((1, "forward"), (-1, "reverse")):
                m = moving[np.sign(moving.target_speed) == sgn]
                if len(m):
                    print(f"  {name}: target max {m.target_speed.abs().max():.0f}, "
                          f"speed error mean {(m.target_speed - m.measured_speed).mean():+.0f} edges/s, "
                          f"pitch target range {m.pitch_target_deg.min():+.2f}..{m.pitch_target_deg.max():+.2f} deg")
        if "speed_i_deg" in d and len(moving):
            # Steady part of each press: the last 1 s before release.
            held = moving.tail(min(len(moving), 50))
            print(f"  speed loop (last 1 s of driving): speed error mean "
                  f"{(held.target_speed - held.measured_speed).mean():+.0f} edges/s, "
                  f"P lean {held.speed_p_deg.mean():+.2f} deg, I lean {held.speed_i_deg.mean():+.2f} deg")
        for sgn, name in ((1, "left"), (-1, "right")):
            s = bal[np.sign(bal.u_steer) == sgn]
            if len(s):
                print(f"  {name} turn: {len(s) * 0.02:.1f} s, pitch error std {s.error_deg.std():.2f} deg")
    else:
        print("  never balancing (state 2) in this log")

    # ---- disturbance -------------------------------------------------------
    t_d = args.disturb
    if t_d is None and len(bal):
        cand = d[active & (d.target_speed == 0)]
        if len(cand):
            t_d = t[cand.error_deg.abs().idxmax()] - 0.3
    rec = None
    if t_d is not None:
        win = (t >= t_d) & (t <= t_d + args.window) & active   # balancing rows only
        w = d[win]
        if len(w) > 5:
            ipk = w.error_deg.abs().idxmax()
            pk = d.error_deg[ipk]
            after = d[(t > t[ipk]) & win]
            opp = after.error_deg[np.sign(after.error_deg) == -np.sign(pk)]
            over = (opp.abs().max() / abs(pk) * 100) if len(opp) else 0.0
            inside = (after.error_deg.abs() < args.band).astype(int)
            hold = inside.rolling(25, min_periods=25).min()    # 0.5 s inside the band
            ok = hold[hold == 1]
            rec = (t[ok.index[0]] - 0.5 - t[ipk]) if len(ok) else None
            print(f"  disturbance at {t_d:.2f} s: peak error {pk:+.2f} deg at {t[ipk]:.2f} s, "
                  f"overshoot {over:.0f} %, recovery to +/-{args.band} deg "
                  + (f"{rec:.2f} s" if rec is not None else "not reached in the window"))

    # ---- plots -------------------------------------------------------------
    fig, ax = plt.subplots(5, 1, figsize=(11, 15))
    ax[0].plot(t, d.target_speed, label="target speed")
    ax[0].plot(t, d.measured_speed, label="measured speed (encoders)", alpha=0.8)
    ax[0].set_ylabel("speed (edges/s)")
    ax[0].set_title("1. Target and measured speed")

    ax[1].plot(t, d.pitch_deg, label="pitch")
    ax[1].plot(t, d.pitch_target_deg, label="pitch target")
    if "theta0_deg" in info:
        ax[1].axhline(float(info["theta0_deg"]), color="gray", ls=":", label="theta0")
    if "speed_i_deg" in d:
        ax[1].plot(t, (float(info.get("theta0_deg", 0)) + d.speed_i_deg).where(active), lw=0.8,
                   ls="--", label="theta0 + speed-loop I lean")
    ax[1].set_ylabel("angle (deg)")
    ax[1].set_title("2. Pitch angle and pitch-angle target")
    if len(bal):
        lo, hi = bal.pitch_deg.min(), bal.pitch_deg.max()
        pad = max(1.0, 0.1 * (hi - lo))
        ax[1].set_ylim(lo - pad, hi + pad)

    ax[2].plot(t, d.pid_output, label="u (PID output)")
    for col, name in (("p_term", "P"), ("i_term", "I"), ("d_term", "D")):
        if col in d:
            ax[2].plot(t, d[col].where(active), lw=0.7, alpha=0.6, label=name)
    ax[2].set_ylabel("duty")
    ax[2].set_title("3. PID controller output")

    ax[3].plot(t, d.left_pwm, label="left PWM")
    ax[3].plot(t, d.right_pwm, label="right PWM", alpha=0.8)
    ax[3].set_ylabel("duty")
    ax[3].set_title("4. Left and right PWM commands")

    if t_d is not None:
        z = (t >= t_d - 0.5) & (t <= t_d + args.window)
        ax[4].plot(t[z], d.error_deg.where(active)[z], label="pitch error (balancing)")
        ax[4].axhspan(-args.band, args.band, color="gray", alpha=0.15, label=f"+/-{args.band} deg")
        a2 = ax[4].twinx()
        a2.plot(t[z], d.pid_output.where(active)[z], color="tab:red", alpha=0.7, label="PID output")
        a2.set_ylabel("duty", color="tab:red")
        title = f"5. Disturbance response (t = {t_d:.2f} s"
        title += f", recovery {rec:.2f} s)" if rec is not None else ")"
        ax[4].set_title(title)
    else:
        ax[4].set_title("5. Disturbance response (no balancing data)")
    ax[4].set_ylabel("error (deg)")

    for i, a in enumerate(ax):
        if i < 4:
            shade(a, t, active)
        a.set_xlabel("time (s)")
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    gains = ""
    if info:
        gains = f"  Kp {info.get('kp')}  Ki {info.get('ki')}  Kd {info.get('kd')}  Kv {info.get('kv')}  Kvi {info.get('kvi')}"
    fig.suptitle(os.path.basename(csv) + gains + "   (green = balancing)")
    fig.tight_layout()
    out = csv[:-4] + ".png" if csv.endswith(".csv") else csv + ".png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  wrote {out}")
