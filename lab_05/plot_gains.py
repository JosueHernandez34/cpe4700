"""Lab 05 post-lab plots: pitch and wheel speed for each PID gain setting.

The logs do not store the gains, so they were recovered from the data itself:
in PID mode duty - inj = Kp*pitch + Kd*rate + Ki*integral(pitch), and a least
squares fit over 3 s windows gives the gains exactly (R^2 = 1.000).

Run from lab_05:  python3 plot_gains.py
Writes plots/*.png and prints the summary table used in REPORT.md.
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import medfilt

COUNTS_PER_REV = 700          # handout: encoder resolution
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
IDSHADE, CUT = "#e4e3df", "#f6dcd0"

# Gain setting -> (log, enable segment). Segments are split where the log has a gap (motors off).
SETTINGS = {
    "A": dict(kp=0.010, ki=0.010, kd=0.015, color=BLUE,   runs=[("141733", 2)]),
    "B": dict(kp=0.010, ki=0.010, kd=0.005, color=ORANGE, runs=[("141733", 3)]),
    "C": dict(kp=0.015, ki=0.005, kd=0.005, color=AQUA,
              runs=[("000508", s) for s in (1, 2, 3, 4, 6, 7)] + [("141733", 4), ("161658", 2), ("162227", 1),
                    ("163831", 1), ("165627", 1), ("165627", 2)] + [("170442", s) for s in (2, 4, 5, 6, 7)]),
}

plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.4})


def load(log):
    d = pd.read_csv(f"balance_log_{log}.csv", comment="#")
    d["seg"] = np.cumsum(np.r_[True, np.diff(d.time_ms.values) > 5])
    if "inj_duty" not in d:
        d["inj_duty"] = 0.0
    # Encoder speed is A-edges/s with 1-sample spikes; a 51 ms median removes the spikes.
    for k in (0, 1):
        d[f"rpm_{k}"] = medfilt(d[f"speed_{k}"].values, 51) * 60.0 / COUNTS_PER_REV
    return d


def segment(log, seg):
    d = load(log)
    s = d[d.seg == seg].copy()
    s["t"] = (s.time_ms - s.time_ms.iloc[0]) / 1000.0
    # The cutoff latch sets duty to exactly 0 while logging continues: those rows are "motors cut".
    s["active"] = s.duty != 0
    return s


def shade(axes, s, mask, color, label=None):
    """Shade the time spans where mask is true on every axis given."""
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    for i, (x0, x1) in enumerate(zip(edges[::2], edges[1::2])):
        for a in axes:
            a.axvspan(s.t.iloc[x0], s.t.iloc[x1 - 1], color=color, lw=0,
                      label=label if (i == 0 and a is axes[0]) else None)


def longest_within(p, band=5.0):
    """Longest stretch (s) with pitch inside +/-band deg of its median."""
    inside = np.abs(p - np.median(p)) < band
    best = run = 0
    for v in inside:
        run = run + 1 if v else 0
        best = max(best, run)
    return best / 1000.0


# ---- Figure 1: the three settings, same session (10/21 14:17), no ID steps ----
fig, ax = plt.subplots(3, 2, figsize=(11, 8), sharex="col")
for row, (name, cfg) in enumerate(SETTINGS.items()):
    s = segment("141733", {"A": 2, "B": 3, "C": 4}[name])
    title = f"{name}: Kp {cfg['kp']:.3f}  Ki {cfg['ki']:.3f}  Kd {cfg['kd']:.3f}"
    a = ax[row, 0]
    shade([a, ax[row, 1]], s, ~s.active.values, CUT, "Motors cut (pitch cutoff)")
    a.axhline(0, color=INK2, lw=0.8)
    a.plot(s.t, s.pitch_deg, color=cfg["color"])
    a.set_ylabel("Pitch (deg)")
    a.set_title(title, loc="left", color=INK, fontsize=10)
    a.set_ylim(-50, 15)
    b = ax[row, 1]
    b.plot(s.t, s.rpm_0, color=BLUE, label="Motor 0")
    b.plot(s.t, s.rpm_1, color=ORANGE, label="Motor 1")
    b.set_ylabel("Wheel speed (RPM)")
    b.set_title("Wheel speed", loc="left", color=INK, fontsize=10)
    b.legend(loc="upper right", frameon=False)
    if (~s.active).any():
        a.legend(loc="lower left", frameon=False)
ax[2, 0].set_xlabel("Time since motors on (s)")
ax[2, 1].set_xlabel("Time since motors on (s)")
fig.suptitle("Pitch and wheel speed for three PID gain settings (10/21 session)", x=0.01, ha="left", color=INK)
fig.tight_layout()
fig.savefig("plots/fig1_gain_settings.png", dpi=150)

# ---- Figure 2: default gains over a 120 s run, ID steps shaded ----
s = segment("162227", 1)
fig, (a, b) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
shade([a, b], s, s.inj_duty.values != 0, IDSHADE, "System-ID test steps on")
shade([a, b], s, ~s.active.values, CUT, "Motors cut (pitch cutoff)")
a.axhline(0, color=INK2, lw=0.8)
a.plot(s.t, s.pitch_deg, color=AQUA)
a.set_ylabel("Pitch (deg)")
a.legend(loc="lower left", frameon=False)
a.set_title("Setting C (Kp 0.015, Ki 0.005, Kd 0.005), 10/26 16:22 run",
            loc="left", color=INK, fontsize=10)
b.plot(s.t, s.rpm_0, color=BLUE, label="Motor 0")
b.plot(s.t, s.rpm_1, color=ORANGE, label="Motor 1")
b.set_ylabel("Wheel speed (RPM)")
b.set_xlabel("Time since motors on (s)")
b.legend(loc="upper right", frameon=False)
fig.tight_layout()
fig.savefig("plots/fig2_default_gains_120s.png", dpi=150)

# ---- Summary table + Figure 3 ----
rows = []
for name, cfg in SETTINGS.items():
    for log, seg in cfg["runs"]:
        s = segment(log, seg)
        lim = 0.30 if log in ("000508", "141733") else 0.60
        noid = s[(s.inj_duty == 0) & s.active]   # controller driving, no test steps
        if len(noid) < 1000:
            continue
        rows.append(dict(setting=name, log=log, seg=seg, seconds=len(noid) / 1000.0,
                         cut=bool((~s.active).any()), pitch_std=noid.pitch_deg.std(), pitch_min=noid.pitch_deg.min(), pitch_max=noid.pitch_deg.max(),
                         saturated=(noid.duty.abs() >= 0.99 * lim).mean() * 100,
                         rpm_rms=np.sqrt(((noid.rpm_0 + noid.rpm_1) ** 2 / 4).mean()),
                         best_hold=longest_within(noid.pitch_deg.values)))
tab = pd.DataFrame(rows)
tab.to_csv("plots/segments_summary.csv", index=False)
summ = tab.groupby("setting").agg(segments=("seg", "size"), seconds=("seconds", "sum"),
                                  pitch_std=("pitch_std", "median"), saturated=("saturated", "median"),
                                  rpm_rms=("rpm_rms", "median"), best_hold=("best_hold", "max"),
                                  cut_runs=("cut", "sum"))
print(tab.round(2).to_string(index=False))
print()
print(summ.round(2).to_string())

fig, ax = plt.subplots(1, 3, figsize=(11, 3.2))
names = list(summ.index)
colors = [SETTINGS[n]["color"] for n in names]
for a, col, label in zip(ax, ["pitch_std", "saturated", "best_hold"],
                         ["Pitch std dev (deg, median)", "Time at duty limit (%, median)",
                          "Longest hold within ±5 deg (s)"]):
    vals = summ[col].values
    bars = a.bar(names, vals, color=colors, width=0.6, edgecolor="white", linewidth=2)
    for r, v in zip(bars, vals):
        a.text(r.get_x() + r.get_width() / 2, v, f"{v:.1f}", ha="center", va="bottom", color=INK)
    a.set_title(label, loc="left", color=INK, fontsize=10)
    a.set_ylim(0, vals.max() * 1.18)
    a.grid(axis="x", visible=False)
fig.suptitle("Gain settings compared (motors-on time only, system-ID steps excluded)", x=0.01, ha="left", color=INK)
fig.tight_layout()
fig.savefig("plots/fig3_summary.png", dpi=150)

# ---- Figure 4: best near-stable stretch (setting C, 10/26 17:04 run, last enable) ----
s = segment("170442", 7)
fig, (a, b) = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
shade([a, b], s, s.inj_duty.values != 0, IDSHADE, "System-ID test steps on")
shade([a, b], s, ~s.active.values, CUT, "Motors cut (pitch cutoff)")
a.axhline(0, color=INK2, lw=0.8)
a.plot(s.t, s.pitch_deg, color=AQUA)
a.set_ylabel("Pitch (deg)")
a.legend(loc="upper right", frameon=False)
a.set_title("Setting C, 10/26 17:04 run: longest stretch within ±5 deg of the run median (21.9 s)", loc="left", color=INK, fontsize=10)
b.plot(s.t, s.rpm_0, color=BLUE, label="Motor 0")
b.plot(s.t, s.rpm_1, color=ORANGE, label="Motor 1")
b.set_ylabel("Wheel speed (RPM)")
b.set_xlabel("Time since motors on (s)")
b.legend(loc="upper right", frameon=False)
fig.tight_layout()
fig.savefig("plots/fig4_best_hold.png", dpi=150)


def dominant_hz(p):
    """Strongest pitch oscillation frequency between 0.5 and 20 Hz."""
    p = p - p.mean()
    f = np.fft.rfftfreq(len(p), 0.001)
    m = (f > 0.5) & (f < 20)
    return f[m][np.argmax(np.abs(np.fft.rfft(p))[m])]


print()
for name, log, seg, t0, t1 in [("A", "141733", 2, 0, 4.5), ("B", "141733", 3, 0, 3), ("C", "141733", 4, 4, 8.5)]:
    s = segment(log, seg)
    w = s[(s.t >= t0) & (s.t < t1)]
    print(f"{name}: dominant pitch oscillation {dominant_hz(w.pitch_deg.values):.1f} Hz, "
          f"peak-to-peak {np.ptp(w.pitch_deg):.1f} deg over {t0}-{t1} s")
