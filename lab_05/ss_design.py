#!/usr/bin/env python3
"""ss_design.py -- build the state-space model from ss_params.py, design the
feedback gain K, check stability, and write the C header the robot uses.

    python3 ss_design.py                                  # parameters as typed in ss_params.py
    python3 ss_design.py --model balance_log_X_model.json # use identify.py's result
    python3 ss_design.py --model X_model.json --emit      # also write generated/ss_model_<time>.h
    python3 ss_design.py --model X_model.json --emit --activate   # ...and install it as ss_model.h

K choices (default is LQR):
    --k K_THETA K_RATE K_OMEGA     type K directly (duty/deg, duty/(deg/s), duty/(count/s))
    --poles P1 P2 P3               pole placement, discrete-time poles (e.g. 0.9 0.92 0.8)
    (neither)                      LQR with --q-theta/--q-rate/--q-omega/--r

WHY A SCRIPT AND NOT C ON THE ROBOT (recommendation)
  The design needs a matrix exponential and a Riccati solve. scipy does both
  reliably and the Jetson already has it, so the design runs here, once, off
  line, where you can look at the numbers. The robot's C code (state_space.c)
  does NOT trust the header blindly: at every startup it re-computes the
  closed-loop poles from the matrices itself, prints them, and refuses to use
  state feedback unless every parameter is SET and all poles are inside the
  unit circle. The real-time loop only ever does one 3-term multiply-add.
  Nothing heavy runs on the robot, and a stale or hand-edited header cannot
  silently put an unstable controller on the wheels.

SIGN CONVENTION (same as balance_loop.c and identify.py)
    duty = K_theta*(theta - theta_ref) + K_rate*(theta_dot - 0) + K_omega*(omega - omega_ref)
  Leaning forward (theta > 0) gives a positive duty. LQR itself returns
  u = -K x; the sign is flipped here so the numbers drop straight into C.

NEVER OVERWRITES: --emit writes a new time-stamped file under generated/.
--activate copies it to ss_model.h and first saves the old ss_model.h into
generated/ as ss_model_prev_<time>.h.
"""
import argparse
import copy
import datetime
import itertools
import json
import os
import shutil
import sys

import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy.signal import place_poles

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ss_params  # noqa: E402  (the parameter block lives next to this script)

TS = ss_params.SAMPLE_TIME_S
COEFFS = ("a1", "a2", "c", "Km", "tau")     # the five grey-box coefficients identify.py produces


# --------------------------------------------------------------------------- parameters
def load_params(model_json, force=False):
    """Copy of the parameter block, with identify.py's values filled in if a model file is given."""
    params = copy.deepcopy(ss_params.PARAMS)
    if model_json:
        with open(model_json) as f:
            m = json.load(f)
        if m.get("usable") is not True and not force:
            why = m.get("problems") or ["no usability record: made by an older identify.py that did not check the "
                                         "model. Re-run identify.py on the log."]
            sys.exit("%s is NOT marked usable by identify.py:\n  - %s\nRepeat Run ID with the robot moving freely "
                     "(not resting on a block). Use --force only if you know why the check is wrong." %
                     (model_json, "\n  - ".join(why)))
        co = m.get("coefficients")
        if not co:
            sys.exit(f"{model_json} has no 'coefficients' (made by an older identify.py). Re-run identify.py.")
        fit = m.get("fit", {})
        fit_txt = ("pitch error %.3f deg RMS vs %.2f deg spread, %.0f s of data"
                   % (fit.get("pitch_rms_deg", float("nan")), fit.get("pitch_std_deg", float("nan")),
                      fit.get("seconds", float("nan"))))
        for name in COEFFS:
            params[name]["value"] = float(co[name])
            params[name]["source"] = "identify.py on %s (%s structure)" % (m.get("source_log", "?"), m.get("structure", "?"))
            params[name]["date"] = m.get("date", "")
            params[name]["fit"] = fit_txt
    return params


def print_table(params):
    print("PARAMETER BLOCK (ss_params.py%s)" % (" + model file" if any(
        "identify.py" in p["source"] for p in params.values()) else ""))
    print("  %-14s %-10s %-34s %s" % ("name", "status", "value  [units]", "source / date / fit"))
    for n, p in params.items():
        st = "NOT SET" if p["value"] is None else "SET"
        val = "-" if p["value"] is None else (
            "%s" % (p["value"],) if isinstance(p["value"], tuple) else "%.6g" % p["value"])
        req = "" if p["required"] else "  (not used by the linear model)"
        src = "; ".join(x for x in (p["source"], p["date"], p["fit"]) if x)
        print("  %-14s %-10s %-34s %s%s" % (n, st, "%s  [%s]" % (val, p["units"]), src, req))


# --------------------------------------------------------------------------- model
def build_AB(a1, a2, c, Km, tau):
    """Continuous model x' = A x + B u, x = [theta deg, theta_dot deg/s, omega counts/s], u = duty."""
    A = np.array([[0.0, 1.0, 0.0],          # theta' = theta_dot
                  [a1, a2, c / tau],        # theta_dot' = a1*theta + a2*theta_dot + (c/tau)*omega - (c*Km/tau)*u
                  [0.0, 0.0, -1.0 / tau]])  # omega' = -omega/tau + (Km/tau)*u   (motor)
    B = np.array([[0.0], [-c * Km / tau], [Km / tau]])
    return A, B


def discretize(A, B, ts=TS):
    """Exact zero-order-hold discretization (the duty is held constant for one sample)."""
    M = np.zeros((4, 4))
    M[:3, :3] = A * ts
    M[:3, 3:] = B * ts
    E = expm(M)
    return E[:3, :3], E[:3, 3:]


def stable(Ad, Bd, Kc):
    """Closed loop with duty = Kc x:  x[k+1] = (Ad + Bd Kc) x[k]. Returns poles, max |pole|."""
    p = np.linalg.eigvals(Ad + Bd @ Kc)
    return p, float(np.abs(p).max())


def design(Ad, Bd, a):
    """Returns Kc (1x3, C sign convention) and a text description of how it was made."""
    if a.k is not None:
        return np.array([a.k], float), "typed in by hand (--k)"
    if a.poles is not None:
        K = place_poles(Ad, Bd, np.array(a.poles, float)).gain_matrix       # u = -K x
        return -K, "pole placement at %s" % (a.poles,)
    Q = np.diag([a.q_theta, a.q_rate, a.q_omega])
    R = np.array([[a.r]])
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)                    # u = -K x
    return -K, "LQR (q_theta %.3g, q_rate %.3g, q_omega %.3g, r %.3g)" % (a.q_theta, a.q_rate, a.q_omega, a.r)


# --------------------------------------------------------------------------- mechanical stop
def stop_values(params):
    """(fwd, back, margin, measured?) in degrees. Unmeasured stops fall back to the unconfirmed 25 deg claim."""
    f, b = params["pitch_stop_fwd"]["value"], params["pitch_stop_back"]["value"]
    measured = f is not None and b is not None
    f = ss_params.PITCH_STOP_CLAIMED_DEG if f is None else float(f)
    b = ss_params.PITCH_STOP_CLAIMED_DEG if b is None else float(b)
    return f, b, float(params["cutoff_margin"]["value"]), measured


def simulate_recovery(v, Kc, theta0, stop_f, stop_b, cap, seconds=6.0, dt=0.001, ts=TS):
    """Nonlinear recovery from rest at theta0 (deg) with the block as a ONE-SIDED limit.

    Plant (deg, counts/s, duty), the usual cart-pendulum form:
        theta'' = a1*(180/pi)*sin(theta) + a2*theta' - c*cos(theta)*omega'
        omega'  = (-omega + Km*u)/tau
    The block holds theta at +stop_f / -stop_b: while the plant is pushed further into the block,
    theta stays on the stop and theta' = 0. The controller sees the TRUE state (no estimator),
    is sampled every Ts and held, and duty is clipped to +/-cap.
    Returns (recovered, peak |duty|, peak |theta| on the far side after crossing upright,
             time to get within 2 deg, rested_on_block_after_start)."""
    a1, a2, c, Km, tau = (v[k] for k in ("a1", "a2", "c", "Km", "tau"))
    k = [float(x) for x in np.asarray(Kc).ravel()]
    th, thd, om, u = float(theta0), 0.0, 0.0, 0.0
    steps = int(seconds / dt)
    per = int(round(ts / dt))
    peak_u, far, t_in, hit_after = 0.0, 0.0, None, False
    crossed = False
    r2d = 180.0 / np.pi
    for n in range(steps):
        if n % per == 0:
            u = max(-cap, min(cap, k[0] * th + k[1] * thd + k[2] * om))
            peak_u = max(peak_u, abs(u))
        omd = (-om + Km * u) / tau
        thdd = a1 * r2d * np.sin(th / r2d) + a2 * thd - c * np.cos(th / r2d) * omd
        if (th >= stop_f and thdd > 0) or (th <= -stop_b and thdd < 0):
            thd, thdd = 0.0, 0.0                       # resting on the block: it carries the load
        om += omd * dt
        thd += thdd * dt
        th += thd * dt
        if th > stop_f:
            th, thd = stop_f, min(thd, 0.0)            # one-sided limit (forward block)
        if th < -stop_b:
            th, thd = -stop_b, max(thd, 0.0)           # one-sided limit (back block)
        if t_in is None and abs(th) < 2.0:
            t_in = n * dt
        if t_in is not None and not crossed and th * theta0 < 0:
            crossed = True
        if crossed:
            far = max(far, abs(th))
        if n * dt > 0.5 and (th >= stop_f - 1e-9 or th <= -stop_b + 1e-9):
            hit_after = True
    tail = steps // 6
    recovered = abs(th) < 1.0 and abs(thd) < 5.0 and t_in is not None and not hit_after
    return recovered, peak_u, far, t_in, hit_after


def recovery_report(v, Kc, stop_f, stop_b, margin, measured):
    """Print how well the design recovers from the stop, against the 0.30 and 0.75 duty caps."""
    cut_f, cut_b = min(45.0, stop_f - margin), min(45.0, stop_b - margin)
    print("\nRecovery check (nonlinear, block as a one-sided limit; true states, no estimator):")
    print("  stop angles: forward %.1f deg, back %.1f deg %s; cutoff %.1f / %.1f deg (stop - %.1f deg margin)" %
          (stop_f, stop_b, "(measured)" if measured else "(UNCONFIRMED, interim 25)", cut_f, cut_b, margin))
    # duty the controller asks for when it starts at rest ON the forward block
    r_free = simulate_recovery(v, Kc, stop_f, stop_f, stop_b, cap=1.0)
    print("  starting at rest ON the forward stop (%.1f deg): peak duty requested = %.3f, %s" %
          (stop_f, r_free[1], "recovers to upright" if r_free[0] else "does NOT recover"))
    for cap in (0.30, 0.75):
        r = simulate_recovery(v, Kc, stop_f, stop_f, stop_b, cap=cap)
        # largest start angle for which every smaller start also recovers
        best = 0.0
        for th0 in np.arange(1.0, stop_f + 0.01, 0.5):
            if simulate_recovery(v, Kc, th0, stop_f, stop_b, cap=cap)[0]:
                best = th0
            else:
                break
        print("  duty cap %.2f: from the stop %s (peak duty %.3f, overshoot to far side %.1f deg); "
              "largest start angle recovered: %.1f deg%s" %
              (cap, "RECOVERS" if r[0] else "FAILS", r[1], r[2], best,
               "  (>= the stop)" if best >= stop_f - 0.01 else ""))
    # smallest cap that still recovers from the stop
    lo, hi = 0.02, 1.0
    if simulate_recovery(v, Kc, stop_f, stop_f, stop_b, cap=hi)[0]:
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            if simulate_recovery(v, Kc, stop_f, stop_f, stop_b, cap=mid)[0]:
                hi = mid
            else:
                lo = mid
        print("  smallest duty cap that still lifts the robot from the stop: %.3f" % hi)
    print("  NOTE: with the cutoff at %.1f deg the motors are cut on the block (it counts as a fall), so a run "
          "that starts ON the block needs the cutoff held off at the start (Josue to decide how runs start)." % cut_f)


# --------------------------------------------------------------------------- C header
def c_matrix(name, M):
    rows = ",\n".join("    {%s}" % ", ".join("%.10g" % v for v in r) for r in M)
    return "static const double %s[3][3] = {\n%s\n};\n" % (name, rows)


def write_header(path, params, ok_all, missing, Ad, Bd, Kc, method, poles, maxabs, note):
    z3 = np.zeros((3, 3)); z1 = np.zeros(3)
    stop_f, stop_b, margin, stop_set = stop_values(params)
    if not ok_all:
        Ad, Bd, Kc = z3, z1.reshape(3, 1), np.zeros((1, 3))
    text = f'''/* ss_model.h -- GENERATED by ss_design.py on {datetime.datetime.now():%Y-%m-%d %H:%M:%S}. Do not edit by hand.
 *
 * Model   x[k+1] = Ad x[k] + Bd u[k]      Ts = {TS} s
 *   x = [ theta (deg), theta_dot (deg/s), omega (counts/s) ]   theta: 0 upright, forward lean positive
 *   u = duty (-1..1), held for one sample
 * Control duty = K . (x - xref)           K = [ duty/deg, duty/(deg/s), duty/(count/s) ]
 *   (forward lean -> positive duty). state_space.c re-checks the closed-loop poles at startup.
 * Status: {note}
 */
#ifndef SS_MODEL_H
#define SS_MODEL_H

#define SS_TS_S        {TS}
#define SS_ALL_SET     {1 if ok_all else 0}      /* 1 = every required coefficient is SET */
#define SS_NOT_SET_LIST "{', '.join(missing)}"   /* coefficients still NOT SET */
#define SS_K_METHOD    "{method}"
#define SS_EXPECT_MAX_POLE {maxabs if ok_all else 0.0:.6f}   /* max |closed-loop pole| from the script, for cross-checking */

/* Mechanical stop (block under the robot). Forward lean is positive pitch; both values are magnitudes.
 * SS_STOP_SET 0 = not measured yet: the 25 deg claim is used as an INTERIM value. The balance loop cuts
 * the motors at (stop - margin) on each side; resting on the block counts as a fall. */
#define SS_STOP_SET         {1 if stop_set else 0}
#define SS_STOP_FWD_DEG     {stop_f}
#define SS_STOP_BACK_DEG    {stop_b}
#define SS_CUTOFF_MARGIN_DEG {margin}

/* Ad: how the state carries over one sample with duty = 0. Rows/cols = theta, theta_dot, omega. */
{c_matrix("SS_AD", Ad)}
/* Bd: effect of one sample of duty on [theta, theta_dot, omega]. */
static const double SS_BD[3] = {{ {", ".join("%.10g" % v for v in np.asarray(Bd).ravel())} }};

/* K: state-feedback gain, duty = K . (x - xref). */
static const double SS_K[3] = {{ {", ".join("%.10g" % v for v in np.asarray(Kc).ravel())} }};

#endif
'''
    with open(path, "w") as f:
        f.write(text)


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", help="<log>_model.json written by identify.py")
    ap.add_argument("--k", type=float, nargs=3, metavar=("K_THETA", "K_RATE", "K_OMEGA"))
    ap.add_argument("--poles", type=float, nargs=3, metavar=("P1", "P2", "P3"))
    ap.add_argument("--q-theta", type=float, default=1 / 5.0 ** 2, help="LQR weight 1/(5 deg)^2")
    ap.add_argument("--q-rate", type=float, default=1 / 50.0 ** 2, help="LQR weight 1/(50 deg/s)^2")
    ap.add_argument("--q-omega", type=float, default=1 / 300.0 ** 2, help="LQR weight 1/(300 counts/s)^2 (wheel speed; smaller = stronger penalty, more robust)")
    ap.add_argument("--r", type=float, default=1 / 0.3 ** 2, help="LQR input weight 1/(0.3 duty)^2")
    ap.add_argument("--force", action="store_true", help="use a model file even if identify.py marked it unusable")
    ap.add_argument("--emit", action="store_true", help="write generated/ss_model_<time>.h")
    ap.add_argument("--out-dir", help="where --emit writes (default: generated/ next to this script)")
    ap.add_argument("--activate", action="store_true", help="with --emit: install it as ss_model.h (old one saved)")
    a = ap.parse_args()
    if a.k is not None and a.poles is not None:
        sys.exit("give either --k or --poles, not both")

    np.set_printoptions(precision=6, suppress=True, linewidth=110)
    params = load_params(a.model, a.force)
    print_table(params)
    missing = [n for n, p in params.items() if p["required"] and p["value"] is None]

    Ad = Bd = Kc = None
    poles = None
    maxabs = float("nan")
    method = "none (model incomplete)"
    if missing:
        note = "NOT READY: %s NOT SET. The robot will use the PID path." % ", ".join(missing)
        print("\n" + note)
    else:
        v = {n: params[n]["value"] for n in COEFFS}
        A, B = build_AB(**v)
        print("\nContinuous model  x' = A x + B u   (x = [theta deg, theta_dot deg/s, omega counts/s], u = duty)")
        print("A =\n", A)
        print("B =\n", B)
        Ad, Bd = discretize(A, B)
        ol = np.linalg.eigvals(Ad)
        print("\nDiscrete model at Ts = %g s   x[k+1] = Ad x[k] + Bd u[k]" % TS)
        print("Ad =\n", Ad)
        print("Bd =\n", Bd)
        print("open-loop poles:", np.round(ol, 4), " |pole|:", np.round(np.abs(ol), 4),
              "(one outside the unit circle = the robot falls over without control)")
        try:
            Kc, method = design(Ad, Bd, a)
        except Exception as e:                                 # not controllable, bad poles, ...
            sys.exit("K design failed: %s" % e)
        poles, maxabs = stable(Ad, Bd, Kc)
        print("\nK designed by %s" % method)
        print("  K_theta = %+.6f duty/deg          (pitch)" % Kc[0, 0])
        print("  K_rate  = %+.6f duty/(deg/s)      (pitch rate)" % Kc[0, 1])
        print("  K_omega = %+.8f duty/(count/s)    (wheel speed)" % Kc[0, 2])
        print("closed-loop poles:", np.round(poles, 4), " |pole|:", np.round(np.abs(poles), 4))
        # Robustness: is it still stable if the identified numbers are off by +/-20%?
        bad = 0
        for s1, s2, s3 in itertools.product((0.8, 1.2), repeat=3):
            A2, B2 = build_AB(v["a1"] * s1, v["a2"], v["c"] * s2, v["Km"] * s3, v["tau"])
            Ad2, Bd2 = discretize(A2, B2)
            if stable(Ad2, Bd2, Kc)[1] >= 1.0:
                bad += 1
        print("robustness: %d of 8 corners (a1, c, Km each +/-20%%) are unstable with this K" % bad)
        if maxabs < 1.0:
            sf, sb, mg, sm = stop_values(params)
            recovery_report(v, Kc, sf, sb, mg, sm)
        if maxabs < 1.0:
            note = "READY: all %d required coefficients SET, closed loop stable (max |pole| %.4f)" % (len(COEFFS), maxabs)
            print("\nSTABLE in the model (all |pole| < 1)  -> state feedback allowed")
        else:
            note = "NOT READY: closed loop UNSTABLE (max |pole| %.4f). The robot will use the PID path." % maxabs
            print("\nNOT STABLE in the model (max |pole| = %.4f) -> state feedback will be refused" % maxabs)

    if a.emit:
        gen_dir = a.out_dir or os.path.join(HERE, "generated")
        os.makedirs(gen_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.join(gen_dir, "ss_model_%s.h" % stamp)
        ready = (not missing) and maxabs < 1.0
        write_header(out, params, not missing, missing, Ad, Bd, Kc, method, poles, maxabs, note)
        print("\nwrote %s" % out)
        if a.activate:
            live = os.path.join(HERE, "ss_model.h")
            if os.path.exists(live):
                prev = os.path.join(gen_dir, "ss_model_prev_%s.h" % stamp)
                shutil.copyfile(live, prev)
                print("saved the old ss_model.h as %s" % prev)
            shutil.copyfile(out, live)
            print("installed as %s -- rebuild with make" % live)
            if not ready:
                print("(the robot will still use PID: %s)" % note)
        else:
            print("not installed yet; add --activate (or copy it over ss_model.h) to use it")
    sys.exit(0 if (not missing and maxabs < 1.0) else 2)


if __name__ == "__main__":
    main()
