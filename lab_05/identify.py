#!/usr/bin/env python3
"""identify.py -- fit a state-space model of the balancing robot from a log
made with "Run ID", then design a state-feedback gain K and check stability.

    python3 identify.py balance_log_HHMMSS.csv [--kp 0.015 --kd 0.005]

WHAT IT DOES
  1. Finds the identification window in the log (where inj_duty != 0).
  2. Resamples to 100 Hz (the BNO055 only updates its angle every ~10 ms).
  3. Builds the state  x = [ theta, theta_dot, omega ]
         theta      pitch, degrees (0 upright, forward lean positive)
         theta_dot  pitch rate, degrees/second (smoothed derivative of pitch)
         omega      average wheel speed, encoder counts/second (derivative of counts)
     and the input u = total duty sent to the motors (-1..1).
     Keeping omega in counts/s means no wheel radius or counts-per-rev is
     needed: the K printed here is already in the units balance_loop.c uses.
  4. Least-squares fit of   x[k+1] = Ad x[k] + Bd u[k]   (Ts = 0.01 s).
  5. Checks the fit (one-step R^2 and 1 s free-run prediction of pitch).
  6. Designs K with LQR (discrete Riccati) and prints the closed-loop poles.
     The poles must all be inside the unit circle (|pole| < 1) for stability.

SIGN CONVENTION OF THE PRINTED GAINS (same as balance_loop.c)
     duty = K_theta*theta + K_rate*theta_dot + K_omega*omega
     Leaning forward gives positive duty. (LQR itself returns u = -K x; the
     sign is flipped here so the numbers can be pasted into the C code.)

NOTE ON CLOSED-LOOP DATA
  The log was made while your gains were balancing the robot. That is fine
  because the injected steps are known and independent of the sensor noise,
  but the model is only trustworthy where the data moved: check the fit
  numbers before using K.
"""
import argparse
import datetime
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ss_params  # noqa: E402  (stop angles and margin live in the parameter block)
from scipy.linalg import expm, logm, solve_discrete_are
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

TS = 0.01            # s: model sample time (IMU update period)
SG_WINDOW = 7        # samples in the pitch-rate smoothing derivative (70 ms)
SG_ORDER = 2         # polynomial order of the smoothing derivative
SG_WINDOW_W = 5      # wheel speed uses a shorter window (50 ms): the motor lag is only ~40 ms,
                     # and a longer window smears it and biases the motor model


def load_window(path, pad_s=0.5):
    """Return the log rows around the identification run."""
    d = pd.read_csv(path)
    need = {"time_ms", "pitch_deg", "duty", "inj_duty", "count_0", "count_1"}
    if not need <= set(d.columns):
        sys.exit(f"{path}: missing columns {sorted(need - set(d.columns))} "
                 "(log made before the ID mode was added?)")
    idx = np.flatnonzero(d.inj_duty.values != 0.0)
    if len(idx) < 3000:
        sys.exit("The log has fewer than 3 s of injected steps. Press 'Run ID' while the "
                 "robot is balancing and let it finish (20 s).")
    a = max(0, idx[0] - int(pad_s * 1000))
    b = min(len(d), idx[-1] + int(pad_s * 1000))
    return d.iloc[a:b].reset_index(drop=True)


def build_states(w):
    """Block the 1 ms log into 10 ms samples and build x, u.

    Alignment matters: x[k] is the state at the START of block k, and u[k] is
    the duty averaged over block k, which is what drives x[k] -> x[k+1].
    (Sampling the state at the end of the block would pair each duty with a
    state it had already acted on, and badly bias the fit.)"""
    n = len(w) // 10
    blk = lambda col: w[col].values[: n * 10].reshape(n, 10)
    theta = blk("pitch_deg")[:, 0]                             # deg at block start
    u = blk("duty").mean(axis=1)                               # duty averaged over the block
    cnt = 0.5 * (blk("count_0")[:, 0] + blk("count_1")[:, 0])  # counts at block start, both wheels averaged
    theta_dot = savgol_filter(theta, SG_WINDOW, SG_ORDER, deriv=1, delta=TS)   # deg/s
    omega = savgol_filter(cnt, SG_WINDOW_W, SG_ORDER, deriv=1, delta=TS)       # counts/s
    return np.column_stack([theta, theta_dot, omega]), u


def fit(x, u):
    """Least squares for x[k+1] = Ad x[k] + Bd u[k]. Returns Ad, Bd, R2 per state."""
    X = np.column_stack([x[:-1], u[:-1]])          # N x 4 regressors
    Y = x[1:]                                      # N x 3 targets
    theta_hat, *_ = np.linalg.lstsq(X, Y, rcond=None)   # 4 x 3

    # The wheel-speed row is a motor: it can only depend on its own speed and
    # on the duty, not on pitch. Because the controller makes duty depend on
    # pitch, an unconstrained fit would pick up a false pitch term (closed-loop
    # bias). Refit that row using only [omega, u].
    Xm = np.column_stack([x[:-1, 2], u[:-1]])
    th_m, *_ = np.linalg.lstsq(Xm, Y[:, 2], rcond=None)
    theta_hat[:, 2] = [0.0, 0.0, th_m[0], th_m[1]]

    Ad = theta_hat[:3, :].T
    Bd = theta_hat[3, :].reshape(3, 1)
    resid = Y - X @ theta_hat
    r2 = 1.0 - resid.var(axis=0) / Y.var(axis=0)
    return Ad, Bd, r2


def discretize(A, B, ts=TS):
    """Zero-order-hold discretization of x' = A x + B u (exact, via the matrix exponential)."""
    n = A.shape[0]
    M = np.zeros((n + 1, n + 1))
    M[:n, :n] = A * ts
    M[:n, n:] = B.reshape(n, 1) * ts
    E = expm(M)
    return E[:n, :n], E[:n, n:]


def continuous_from_discrete(Ad, Bd, ts=TS):
    """Rough inverse of discretize(), used only as a starting guess."""
    try:
        Ac = np.real(logm(Ad)) / ts
        Bc = Ac @ np.linalg.solve(Ad - np.eye(3), Bd)
        if np.all(np.isfinite(Ac)) and np.all(np.isfinite(Bc)):
            return Ac, Bc
    except Exception:
        pass
    return None, None


def fit_output_error(w, x0_est, seg=50, structure="physics", limits=(90.0, 90.0)):
    """Fit the model by SIMULATION instead of by differentiating the data.

    Differentiating pitch and wheel counts (as fit() does) adds noise and lag
    to theta_dot and omega, and fitting on those biases the model. Here the
    model is
          theta'' = a1*theta + a2*theta' + a3*omega + b2*u
          omega'  = a33*omega + b3*u                      (motor: speed and duty only)
    and its parameters (a1,a2,a3,a33,b2,b3) are chosen so that, started from
    the measured pitch/counts, the simulated pitch and wheel counts match the
    measured ones over each 0.5 s segment, driven by the logged duty. Only the
    measured signals (pitch, counts, duty) are used; the unknown start values of
    theta_dot and omega are fitted per segment.
    structure="physics" ties the wheel-to-pitch coupling to the motor, because the
    wheels only push the body through their ACCELERATION:
          theta'' = a1*theta + a2*theta' - c*omega'     and   omega' = a33*omega + b3*u
       =>  a3 = -c*a33,  b2 = -c*b3        (5 unknowns: a1, a2, c, a33, b3)
    structure="free" fits a3 and b2 independently (6 unknowns). "physics" is
    better conditioned and is the default.
    limits=(fwd, back): segments that reach beyond +fwd or -back degrees are DROPPED, because
    contact with the mechanical stop is a one-sided nonlinearity that the linear model cannot
    represent (the caller passes stop angle - margin).
    Returns Ad, Bd (3x3, 3x1 at Ts), RMS errors (pitch deg, counts), a dict of the
    continuous-time coefficients (a1, a2, c, Km, tau, a3, b2, a33, b3) and (kept, total) segments."""
    n = len(w) // 10
    blk = lambda col: w[col].values[: n * 10].reshape(n, 10)
    theta = blk("pitch_deg")[:, 0]
    cnt = 0.5 * (blk("count_0")[:, 0] + blk("count_1")[:, 0])
    u = blk("duty").mean(axis=1)
    nseg_all = (n - 1) // seg
    starts = np.arange(nseg_all) * seg
    # Drop segments that touch the stop zone (any sample, including the start and end values).
    keep = np.array([theta[s0:s0 + seg + 1].max() <= limits[0] and theta[s0:s0 + seg + 1].min() >= -limits[1]
                     for s0 in starts])
    if keep.sum() < 5:
        sys.exit("Only %d of %d segments stay inside +%.1f/-%.1f deg (stop - margin). The run spent too long "
                 "near the stop to identify a linear model. Repeat Run ID with smaller steps." %
                 (keep.sum(), nseg_all, limits[0], limits[1]))
    starts = starts[keep]
    nseg = len(starts)
    U = np.stack([u[s:s + seg] for s in starts])                      # S x seg
    TH = np.stack([theta[s + 1:s + seg + 1] for s in starts])         # measured pitch after each step
    CT = np.stack([cnt[s + 1:s + seg + 1] for s in starts])           # measured counts after each step
    th0, ct0 = theta[starts], cnt[starts]
    W_TH, W_CT = 1 / 0.15, 1 / 3.0          # residual weights: ~0.15 deg and ~3 counts of measurement noise

    def coeffs(p):
        """Parameter vector -> (a1, a2, a3, a33, b2, b3)."""
        if structure == "physics":
            a1, a2, c, a33, b3 = p
            return a1, a2, -c * a33, a33, -c * b3, b3
        return tuple(p)

    NP = 5 if structure == "physics" else 6          # number of model parameters

    def model(p):
        a1, a2, a3, a33, b2, b3 = coeffs(p)
        A = np.array([[0, 1, 0, 0], [a1, a2, a3, 0], [0, 0, a33, 0], [0, 0, 1, 0]], float)   # 4th state = counts
        B = np.array([0, b2, b3, 0], float)
        return discretize(A, B)

    def resid(q):
        Ad4, Bd4 = model(q[:NP])
        X = np.column_stack([th0, q[NP:NP + nseg], q[NP + nseg:], ct0])  # S x 4 start states
        r = []
        for k in range(seg):
            X = X @ Ad4.T + U[:, k:k + 1] * Bd4.T
            r.append(np.column_stack([(X[:, 0] - TH[:, k]) * W_TH, (X[:, 3] - CT[:, k]) * W_CT]))
        return np.concatenate([a.ravel() for a in r])

    # Starting guess: the regression fit, converted to continuous time; else a generic one.
    Ac, Bc = continuous_from_discrete(*fit(x0_est, u_blocks(w))[:2])
    full0 = [20.0, 0.0, 25.0, -25.0, -100.0, 1.5e5]          # generic start: a1, a2, a3, a33, b2, b3
    if Ac is not None:
        full0 = [Ac[1, 0], Ac[1, 1], Ac[1, 2], Ac[2, 2], Bc[1, 0], Bc[2, 0]]
    if structure == "physics":
        c0 = -full0[2] / full0[3] if abs(full0[3]) > 1e-6 else 1.0
        p0 = [full0[0], full0[1], c0, full0[3], full0[5]]
    else:
        p0 = full0
    q0 = np.concatenate([p0, x0_est[starts, 1], x0_est[starts, 2]])
    sol = least_squares(resid, q0, x_scale="jac", max_nfev=300)
    Ad4, Bd4 = model(sol.x[:NP])
    a1, a2, a3, a33, b2, b3 = coeffs(sol.x[:NP])
    co = {"a1": a1, "a2": a2, "a3": a3, "a33": a33, "b2": b2, "b3": b3,
          "c": (-a3 / a33) if abs(a33) > 1e-9 else float("nan"),     # wheel accel -> pitch accel coupling
          "Km": (-b3 / a33) if abs(a33) > 1e-9 else float("nan"),    # motor gain, counts/s per duty
          "tau": (-1.0 / a33) if a33 < 0 else float("nan")}          # motor time constant, s
    r = sol.fun.reshape(seg, nseg, 2)
    rms_th = float(np.sqrt(np.mean((r[:, :, 0] / W_TH) ** 2)))
    rms_ct = float(np.sqrt(np.mean((r[:, :, 1] / W_CT) ** 2)))
    return Ad4[:3, :3], Bd4[:3, :], rms_th, rms_ct, co, (nseg, nseg_all)


def u_blocks(w):
    n = len(w) // 10
    return w["duty"].values[: n * 10].reshape(n, 10).mean(axis=1)


def free_run_rms(Ad, Bd, x, u, seconds=1.0):
    """Predict pitch `seconds` ahead from measured u, start every second; RMS error (deg)."""
    h = int(seconds / TS)
    errs = []
    for s in range(0, len(x) - h, h):
        xp = x[s].copy()
        for k in range(h):
            xp = Ad @ xp + (Bd[:, 0] * u[s + k])
        errs.append(xp[0] - x[s + h][0])
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")


def lqr(Ad, Bd, q, r):
    P = solve_discrete_are(Ad, Bd, np.diag(q), np.array([[r]]))
    K = np.linalg.solve(r + Bd.T @ P @ Bd, Bd.T @ P @ Ad)      # u = -K x
    return K


RESULT = {}     # filled by main(); live_main() writes it for the GUI


def model_check(co, kept, total, rms_th, pitch_std, w, stop_f, stop_b):
    """Reasons the fitted model cannot be trusted (empty list = usable).

    These checks exist because a fit can match the data and still be physically nonsense, for
    example when the robot spent the run resting against a block (a nonlinearity) instead of
    moving freely. Returns (problems, pinned_fraction)."""
    problems = []
    p = w["pitch_deg"].values
    rate = np.gradient(p, 0.001)                                    # deg/s at 1 ms
    near_block = (p >= stop_f - 2.5) | (p <= -(stop_b - 2.5))       # within 2.5 deg of a stop
    pinned = float(np.mean(near_block & (np.abs(rate) < 5.0)))      # near a block AND hardly moving
    if pinned > 0.2:
        problems.append("the robot sat against a block for %d%% of the run (near a stop and hardly moving), "
                        "so the data holds the block's behaviour, not free balancing" % round(100 * pinned))
    if not co["a1"] > 0:
        problems.append("gravity term a1 = %+.2f is not positive: this model would balance by itself, a real "
                        "inverted pendulum falls" % co["a1"])
    if not co["c"] > 0:
        problems.append("wheel coupling c = %+.3f is not positive (accelerating the wheels forward must pitch the "
                        "body backward)" % co["c"])
    if not (0.01 <= co["tau"] <= 0.15):
        problems.append("motor time constant %.3g s is far from Lab 04's 0.04 s" % co["tau"])
    if not (3000 <= co["Km"] <= 10000):
        problems.append("motor gain %.0f counts/s per duty is far from Lab 04's 6100" % co["Km"])
    if kept < 0.6 * total:
        problems.append("only %d of %d segments stayed inside the stop zone" % (kept, total))
    if rms_th > 0.5 * pitch_std:
        problems.append("pitch prediction error %.2f deg is more than half the pitch spread (%.2f deg)" % (rms_th, pitch_std))
    return problems, pinned


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--kp", type=float, default=0.015, help="current Kp, to check against the model")
    ap.add_argument("--kd", type=float, default=0.005, help="current Kd, to check against the model")
    ap.add_argument("--structure", choices=["physics", "free"], default="physics",
                    help="physics: tie the wheel->pitch coupling to the motor (5 unknowns, default); free: 6 unknowns")
    ap.add_argument("--live", help="quiet mode for the GUI: write a short key=value summary to this file")
    ap.add_argument("--stop-fwd", type=float, help="forward stop angle, deg (default: ss_params, else the unconfirmed 25)")
    ap.add_argument("--stop-back", type=float, help="backward stop angle magnitude, deg")
    ap.add_argument("--margin", type=float, help="degrees before the stop to exclude (default: ss_params cutoff_margin)")
    ap.add_argument("--q-theta", type=float, default=1 / 5.0 ** 2, help="LQR weight 1/(5 deg)^2")
    ap.add_argument("--q-rate", type=float, default=1 / 50.0 ** 2, help="LQR weight 1/(50 deg/s)^2")
    ap.add_argument("--q-omega", type=float, default=1 / 300.0 ** 2, help="LQR weight 1/(300 counts/s)^2 (wheel speed; smaller = stronger penalty, more robust)")
    ap.add_argument("--r", type=float, default=1 / 0.3 ** 2, help="LQR input weight 1/(0.3 duty)^2")
    a = ap.parse_args()

    w = load_window(a.log)
    x, u = build_states(w)
    print(f"{a.log}: {len(w) / 1000:.1f} s of data, {len(x)} samples at {1 / TS:.0f} Hz")
    print(f"  pitch range {x[:, 0].min():.1f}..{x[:, 0].max():.1f} deg, "
          f"duty range {u.min():.2f}..{u.max():.2f}, wheel speed range {x[:, 2].min():.0f}..{x[:, 2].max():.0f} counts/s")

    np.set_printoptions(precision=5, suppress=True, linewidth=110)
    # Quick regression fit (differentiated signals): only a starting guess and a cross-check.
    Ad_r, Bd_r, r2 = fit(x, u)
    # Main fit: simulate the model and match measured pitch and wheel counts.
    sf = a.stop_fwd if a.stop_fwd is not None else (ss_params.PARAMS["pitch_stop_fwd"]["value"] or ss_params.PITCH_STOP_CLAIMED_DEG)
    sb = a.stop_back if a.stop_back is not None else (ss_params.PARAMS["pitch_stop_back"]["value"] or ss_params.PITCH_STOP_CLAIMED_DEG)
    mg = a.margin if a.margin is not None else ss_params.PARAMS["cutoff_margin"]["value"]
    Ad, Bd, rms_th, rms_ct, co, (kept, total) = fit_output_error(w, x, structure=a.structure, limits=(sf - mg, sb - mg))
    print("stop zone: segments beyond +%.1f / -%.1f deg (stop %.1f/%.1f minus %.1f margin%s) are excluded: kept %d of %d"
          % (sf - mg, sb - mg, sf, sb, mg,
             "" if (ss_params.PARAMS["pitch_stop_fwd"]["value"] is not None) else "; stop UNCONFIRMED, interim 25",
             kept, total))
    if kept < 0.8 * total:
        print("WARNING: %d%% of the run was within the stop margin. Use smaller Run ID steps so pitch stays inside "
              "the stop minus the margin." % round(100 * (1 - kept / total)))
    print("\nIdentified model  x[k+1] = Ad x[k] + Bd u[k]   (x = [theta deg, theta_dot deg/s, omega counts/s])")
    print("Ad =\n", Ad)
    print("Bd =\n", Bd)
    print("(for reference, the quick regression fit gave Bd =", np.round(Bd_r.ravel(), 2), ")")
    print("Grey-box coefficients (%s structure):" % a.structure)
    print("  a1 = %+.3f 1/s^2     (gravity term: how fast the lean grows)" % co["a1"])
    print("  a2 = %+.3f 1/s       (pitch damping)" % co["a2"])
    print("  c  = %+.4f deg/s^2 per (count/s^2)   (wheel acceleration -> pitch acceleration)" % co["c"])
    print("  Km = %.0f counts/s per duty,  tau = %.4f s   (motor; Lab 04 measured about 6100 and 0.04)" % (co["Km"], co["tau"]))
    print("fit quality over 0.5 s segments: pitch error %.3f deg RMS, wheel-count error %.2f counts RMS "
          "(pitch std is %.2f deg)" % (rms_th, rms_ct, x[:, 0].std()))
    rms = rms_th
    problems, pinned = model_check(co, kept, total, rms_th, float(x[:, 0].std()), w, sf, sb)
    RESULT.update(seconds=len(w) / 1000.0, kept=kept, total=total, pitch_rms=rms_th, pitch_std=float(x[:, 0].std()),
                  a1=co["a1"], c=co["c"], Km=co["Km"], tau=co["tau"], pinned=pinned)
    print("\nMODEL CHECK: %s" % ("USABLE" if not problems else "NOT USABLE"))
    for pr in problems:
        print("  - " + pr)
    poles = np.linalg.eigvals(Ad)
    print("open-loop poles of Ad:", np.round(poles, 4), " |pole|:", np.round(np.abs(poles), 3))
    print("  (one pole outside the unit circle is expected: the robot falls over without control)")

    # --- Check the gains you are using now against this model ---
    # State feedback in the C sign convention: u = Kc x, so closed loop Ad + Bd Kc.
    Kcur = np.array([[a.kp, a.kd, 0.0]])
    pc = np.linalg.eigvals(Ad + Bd @ Kcur)
    print(f"\nYour current gains (Kp={a.kp}, Kd={a.kd}, no wheel-speed feedback) in this model:")
    print("  closed-loop poles:", np.round(pc, 4), " max |pole| =", round(float(np.abs(pc).max()), 4),
          "-> stable" if np.abs(pc).max() < 1 else "-> UNSTABLE in the model")

    # --- LQR design (only from a usable model) ---
    if problems:
        print("\nK WITHHELD: the model failed the checks above, so no gain is designed from it. Repeat Run ID "
              "with the robot moving freely between the blocks (not resting on them).")
        RESULT.update(status="NOT_USABLE", reason=" | ".join(problems))
        out = os.path.splitext(a.log)[0] + "_model.json"
        if not a.live and not os.path.exists(out):
            with open(out, "w") as f:
                json.dump({"usable": False, "problems": problems, "coefficients": co, "structure": a.structure,
                           "source_log": os.path.basename(a.log), "date": datetime.date.today().isoformat()}, f, indent=2)
            print("saved %s (marked unusable)" % out)
        return
    K = lqr(Ad, Bd, [a.q_theta, a.q_rate, a.q_omega], a.r)
    Kc = -K                                          # flip to the C convention (duty = Kc x)
    pcl = np.linalg.eigvals(Ad + Bd @ Kc)
    print("\nLQR design  (weights: theta %.3g, rate %.3g, omega %.3g, r %.3g)" %
          (a.q_theta, a.q_rate, a.q_omega, a.r))
    print("  K_theta (duty/deg)     = %+.5f" % Kc[0, 0])
    print("  K_rate  (duty/(deg/s)) = %+.5f" % Kc[0, 1])
    print("  K_omega (duty/(count/s)) = %+.7f" % Kc[0, 2])
    print("  closed-loop poles:", np.round(pcl, 4), " |pole|:", np.round(np.abs(pcl), 3))
    ok = np.abs(pcl).max() < 1.0
    print("  STABLE in the model (all |pole| < 1)" if ok else "  NOT stable in the model -- do not use")
    RESULT.update(status="OK" if ok else "NOT_USABLE", Ktheta=float(Kc[0, 0]), Krate=float(Kc[0, 1]),
                  Komega=float(Kc[0, 2]), maxpole=float(np.abs(pcl).max()),
                  reason="" if ok else "closed loop not stable in the model")

    # --- Save the result next to the log (never overwrites) ---
    out = os.path.splitext(a.log)[0] + "_model.json"
    if a.live:
        pass                                          # the GUI only needs the summary, never write files
    elif os.path.exists(out):
        print(f"\n{out} exists, not overwritten.")
    else:
        with open(out, "w") as f:
            json.dump({"usable": True, "Ts": TS, "Ad": Ad.tolist(), "Bd": Bd.tolist(), "K_c_convention": Kc.tolist(),
                       "r2": r2.tolist(), "freerun_rms_deg": rms,
                       # Grey-box coefficients for ss_design.py / ss_params.py, with provenance:
                       "structure": a.structure, "coefficients": co,
                       "source_log": os.path.basename(a.log),
                       "date": datetime.date.today().isoformat(),
                       "stop_zone": {"fwd": sf, "back": sb, "margin": mg, "segments_kept": int(kept), "segments_total": int(total)},
                       "fit": {"pitch_rms_deg": rms_th, "count_rms": rms_ct,
                               "pitch_std_deg": float(x[:, 0].std()), "seconds": len(w) / 1000.0}},
                      f, indent=2)
        print(f"\nsaved {out}")

    if rms_th > 0.5 * x[:, 0].std():
        print("\nWARNING: the model predicts pitch poorly (error is more than half the pitch spread). "
              "The log may not have enough movement. Repeat Run ID with a larger step size before trusting K.")


def live_main():
    """--live FILE: run quietly and write a key=value summary (atomically) for the GUI."""
    path = sys.argv[sys.argv.index("--live") + 1]
    sys.stdout = open(os.devnull, "w")
    try:
        main()
        if "status" not in RESULT:
            RESULT["status"] = "ERROR"
            RESULT["reason"] = "no result"
    except SystemExit as e:
        RESULT.update(status="ERROR", reason=str(e.code))
    except Exception as e:                              # never leave the GUI waiting without an answer
        RESULT.update(status="ERROR", reason="%s: %s" % (type(e).__name__, e))
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        for k, v in RESULT.items():
            f.write("%s=%s\n" % (k, ("%.6g" % v) if isinstance(v, float) else str(v).replace("\n", " ")))
    os.replace(tmp, path)


if __name__ == "__main__":
    if "--live" in sys.argv:
        live_main()
    else:
        main()
