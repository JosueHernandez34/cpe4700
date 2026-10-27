"""ss_params.py -- THE PARAMETER BLOCK for the state-space balance controller.

This is the one place that holds every number the model needs. The robot is
treated as a GREY BOX: no masses, heights or inertias are measured. Instead the
model is a physics-structured inverted pendulum whose few lumped coefficients
are identified from logged data (identify.py + the "Run ID" logs):

    states   x = [ theta, theta_dot, omega ]
               theta      pitch                 deg   (0 upright, forward lean +)
               theta_dot  pitch rate            deg/s
               omega      average wheel speed   counts/s (both encoders averaged)
    input    u = motor duty, -1..1  (+ = wheels forward)

    theta'' = a1*theta + a2*theta_dot - c*omega_dot          (pendulum + wheel push)
    omega'  = -(1/tau)*omega + (Km/tau)*u                     (Lab 04 motor model)

  which, with omega_dot substituted, gives the continuous model x' = A x + B u:

          [ 0    1    0        ]            [ 0         ]
      A = [ a1   a2   c/tau    ]        B = [ -c*Km/tau ]
          [ 0    0    -1/tau   ]            [ Km/tau    ]

  Why the wheel only enters through omega_dot: the wheels push the body by
  ACCELERATING, so one coefficient c links the motor to the pitch dynamics.
  Wheel speed stays in counts/s, so no wheel radius or counts-per-revolution
  is ever needed (those cannot be identified separately from logs and are not
  needed for control).

HOW TO FILL IT IN
  * Run "Run ID" on the floor, quit, then run   python3 identify.py <log>
    That writes <log>_model.json with a1, a2, c, Km, tau, the source log, the
    date and the fit quality.
  * python3 ss_design.py --model <log>_model.json   copies those values in
    (with their source) and designs the controller. You can also type values
    below by hand: set "value" and fill "source", "date", "fit".
  * "value": None means NOT SET. State feedback is only used when every
    REQUIRED entry is set AND the closed-loop poles are inside the unit circle.
    Otherwise the robot falls back to the PID path and the GUI says why.

HOW FAR THE LINEAR MODEL CAN BE TRUSTED (small-angle accuracy)
  The model is the small-angle linearisation of the usual cart-pendulum form
      theta'' = a1*sin(theta) - c*cos(theta)*omega_dot ...
  (assumed: this form is not verified on the real robot). Error of the small-angle terms:
      angle    gravity term sin(t)/t    wheel coupling cos(t)
        10 deg      0.5%                    1.5%
        14 deg      1.0%  <- 1% limit       3.0%
        18 deg      1.6%                    5.0%  <- 5% limit
        21 deg      2.2%                    6.6%
        25 deg      3.1%                    9.4%
        31.6 deg    5.0%  <- 5% limit      15%
  So inside a 25 deg stop the gravity term is within 3%, but the wheel coupling is overstated by up
  to 9%: the linear design is still sound there because ss_design.py checks +/-20% on a1, c and Km
  and requires all of them stable. Identification (identify.py) should only use samples well inside
  the stop, because the stop itself is a nonlinearity (see below).

THE STOP IS A NONLINEARITY
  Contact with the block is a one-sided limit that no linear fit can represent. identify.py
  therefore drops any 0.5 s segment that comes within the cutoff margin of the stop, and the Run ID
  steps stop being injected well before that (60% of the cutoff). ss_design.py also simulates
  recovery from the stop with the block modelled as a one-sided limit.

WHAT THE LOGS CAN AND CANNOT TELL YOU (grey-box identifiability)
  The numbers below come from tests on SIMULATED robots with sensor noise (the
  true values were known). They are a best case: the simulated robot is linear,
  with no deadband and no delay. Spread = how much the fitted value moved
  between 5 different noise seeds, at three Run ID step sizes.

  Identifiable from Run ID logs (spread at step 0.08 / 0.02 / 0.005 duty):
    a1   gravity term          0.1% / 0.7% / 2%     needs real pitch excursions: if the robot is held
                                                      too tightly to move, a1 is poorly known.
    c    wheel->pitch coupling 0.1% / 0.3% / 2%     same excitation as a1; tied to the motor by the
                                                      physics structure (identify.py --structure physics).
    Km   motor gain            0.1% / 0.2% / 1%     only needs wheel counts and duty; also known from Lab 04.
    tau  motor time constant   0.1% / 0.4% / 3%     needs steps held longer than about 3*tau (0.12 s);
                                                      the 150 ms holds in Run ID are just enough.
  Weakly identifiable:
    a2   pitch damping         std 0.009 / 0.03 / 0.22 1/s against a true value near 0. It is small and
                                                      hides in the noise. Setting a2 = 0.0 on purpose is fine.
                                                      Fix: more or longer Run ID logs, larger steps.
  NOT identifiable from the logs (and what would fix it):
    * Deadband: a nonlinearity, so a linear fit cannot see it. Use the Lab 04 values and keep the Run ID
      step size above them (0.08 vs 0.06). Fix for control: compensate it in the balance loop.
    * Time delay (PWM write, IMU age): not a coefficient in this model. It shows up as a worse fit. Fix:
      a Run ID with short holds (40-60 ms) so a 5-10 ms delay becomes visible, then add a delay term.
    * Individual masses, heights, wheel radius, inertia, counts per revolution: only their lumped products
      (a1, c, Km) appear in the data, so they cannot be separated, and control does not need them.
    * Pitch equilibrium offset (the true balance point may not be exactly 0 deg): assumed 0. Fix: hold the
      robot at a few target pitches and see where the average wheel speed is zero.
    * Anything outside the excited range: the steps cover roughly 0-3 Hz and about +/-0.1 duty.
  Practical limit: sample age. The BNO055 gives a new angle every ~10 ms, not synchronised with the
  1 ms loop. In simulation, data 9 ms old instead of 0 ms old turned +/-0.4 deg into +/-27 deg, so the
  state-feedback path updates when a NEW IMU value arrives (balance_loop.c).
"""

SAMPLE_TIME_S = 0.01      # s: model/controller sample time = BNO055 update period (100 Hz)

# Josue says a block under the robot stops it from falling below "about 25 degrees". He did not say
# whether that is measured from vertical or from the ground, so 25 is UNCONFIRMED. It is used only as
# an INTERIM value (for the cutoff) until pitch_stop_fwd / pitch_stop_back below are measured.
PITCH_STOP_CLAIMED_DEG = 25.0   # deg: unconfirmed claim, interim only

# status is derived: value is None -> "NOT SET", otherwise "SET".
# required=True entries must all be SET before state feedback is allowed.
PARAMS = {
    # ---------------- Pitch dynamics: identified from Run ID logs ----------------
    "a1": dict(
        value=None, units="1/s^2", required=True,
        meaning="gravity term: pitch acceleration per degree of pitch. Positive = a lean grows (falls over). "
                "Lumped g/l of the robot.",
        source="", date="", fit=""),
    "a2": dict(
        value=None, units="1/s", required=True,
        meaning="pitch damping: pitch acceleration per deg/s of pitch rate. Small; weakest to identify. "
                "May be set to 0.0 on purpose if the fit cannot resolve it.",
        source="", date="", fit=""),
    "c": dict(
        value=None, units="deg/s^2 per (count/s^2)", required=True,
        meaning="wheel acceleration -> pitch acceleration coupling (positive: accelerating the wheels "
                "forward pitches the body backward). Lumped wheel radius / body geometry.",
        source="", date="", fit=""),

    # ---------------- Motor: measured in Lab 04, refined by identify.py ----------------
    "Km": dict(
        value=6073.0, units="counts/s per unit duty", required=True,
        meaning="motor steady-state gain, encoder A-edges per second per unit duty.",
        source="Lab 04 step response, mean of W0/W1 fwd/rev gains 6100, 5904, 6139, 6147 (lab_04/REPORT.md)",
        date="2026-10-17", fit="spread +/-3% between wheels and directions"),
    "tau": dict(
        value=0.040, units="s", required=True,
        meaning="motor time constant (speed response to a duty step).",
        source="Lab 04 step response: about 41 ms by the 63.2% rule, 27 ms from the 10-90% rise (lab_04/REPORT.md)",
        date="2026-10-17", fit="uncertain, roughly 30-40 ms"),

    # ---------------- Mechanical stop (a block that holds the robot off the ground) ----------------
    # MEASURE (no motors): run pitch_probe, rest the robot on the block leaning forward, then
    # leaning back, and read the corrected pitch (0 upright, forward lean positive). The reading
    # also settles whether the "25 degrees" is from vertical (reads about 25) or from the ground
    # (reads about 65).
    "pitch_stop_fwd": dict(
        value=10.0, units="deg (forward lean, positive)", required=False,
        meaning="corrected pitch when the robot rests on the block leaning FORWARD.",
        source="Josue's reading, 10/22: pitch is -8 to +10 with the blocks. NOT yet measured with pitch_probe. "
               "(The earlier '25 deg' claim was wrong or measured from the ground.)",
        date="2026-10-22", fit="whole degrees, rough"),
    "pitch_stop_back": dict(
        value=8.0, units="deg (backward lean, magnitude)", required=False,
        meaning="magnitude of the corrected pitch when the robot rests on the block leaning BACK "
                "(the reading itself is negative).",
        source="Josue's reading, 10/22: pitch is -8 to +10 with the blocks. NOT yet measured with pitch_probe.",
        date="2026-10-22", fit="whole degrees, rough"),
    "cutoff_margin": dict(
        value=4.0, units="deg", required=False,
        meaning="motors are cut this many degrees BEFORE the stop angle. Resting on the block counts as a "
                "fall (motors to 0, latched). PROPOSED, not measured: 4 deg is well above the 0.0625 deg "
                "angle step and the ~1 deg scatter expected when resting, and still leaves "
                "stop - margin = 21 deg of usable range if the stop is 25 deg. Shrink it only if the "
                "resting readings scatter very little.",
        source="proposed by Claude Code, to be confirmed by Josue", date="2026-10-21", fit="not measured"),

    # ---------------- Not used by the linear model (kept so nothing is lost) ----------------
    "deadband_pos": dict(
        value=(0.06, 0.06), units="duty (wheel 0, wheel 1)", required=False,
        meaning="smallest +duty that moves each wheel. NOT in the linear model and NOT compensated in the "
                "balance loop yet. Keep the Run ID step size above this.",
        source="Lab 04 deadband run (lab_04/deadband.csv)", date="2026-10-21", fit="0.02 resolution"),
    "deadband_neg": dict(
        value=(0.06, 0.04), units="duty magnitude (wheel 0, wheel 1)", required=False,
        meaning="smallest -duty (magnitude) that moves each wheel. Same note as deadband_pos.",
        source="Lab 04 deadband run (lab_04/deadband.csv)", date="2026-10-21", fit="0.02 resolution"),
    "counts_per_rev": dict(
        value=None, units="counts per wheel revolution", required=False,
        meaning="NOT NEEDED: wheel speed stays in counts/s. Only for converting plots to RPM.",
        source="", date="", fit=""),
}


def status(name):
    """'SET' or 'NOT SET' for one entry."""
    return "NOT SET" if PARAMS[name]["value"] is None else "SET"


def required_not_set():
    """Names of required entries that are still NOT SET."""
    return [n for n, p in PARAMS.items() if p["required"] and p["value"] is None]
