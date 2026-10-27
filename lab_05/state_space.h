#ifndef STATE_SPACE_H
#define STATE_SPACE_H

/*
 * state_space -- state-feedback controller framework (Lab 05).
 *
 * The model (Ad, Bd) and gain K come from ss_model.h, which ss_design.py
 * generates off line from the parameter block in ss_params.py. Nothing heavy
 * runs on the robot: ss_init() re-computes the closed-loop poles from the
 * matrices (a 3x3 characteristic polynomial, no libraries) as an independent
 * check, and the real-time loop only does a 3-term multiply-add.
 *
 * State vector x = [ theta, theta_dot, omega ]
 *   theta      deg      pitch, 0 = upright, forward lean positive
 *   theta_dot  deg/s    pitch rate
 *   omega      counts/s average wheel speed (both encoders), + = forward
 * Control law  duty = K . (x - xref)          (forward lean -> positive duty)
 */

/* What to do when the model is incomplete or fails the stability check:
 *   1 = fall back to the PID path (the GUI says so)
 *   0 = refuse to enable the motors (the GUI says so) */
#define SS_FALLBACK_TO_PID 1

typedef enum {
    SS_MODE_STATE_FEEDBACK = 0,   /* every parameter SET and all closed-loop poles inside the unit circle */
    SS_MODE_PID            = 1,   /* not ready, falling back to PID */
    SS_MODE_REFUSED        = 2    /* not ready and SS_FALLBACK_TO_PID is 0: motors cannot be enabled */
} ss_mode_t;

/* Verify the generated model and decide the mode. Prints the parameters
 * status, the closed-loop poles and the stability check. Call once at
 * startup, before the balance thread starts. */
void ss_init(void);

ss_mode_t   ss_mode(void);
const char *ss_reason(void);       /* one line: why not state feedback, or the pole summary when it is */
double      ss_max_pole(void);     /* max |closed-loop pole| computed here (NaN if no model) */

/* Mechanical stop (block under the robot), from ss_model.h. Forward lean is positive pitch; both are
 * magnitudes in degrees. ss_stop_is_set() is 0 until the stop angles are measured (ss_params.py); until
 * then the unconfirmed 25 deg claim is returned as an INTERIM value. */
double ss_stop_fwd_deg(void);
double ss_stop_back_deg(void);
int    ss_stop_is_set(void);
double ss_cutoff_margin_deg(void);    /* motors are cut this far BEFORE the stop */

/* The designed gain K[i] from ss_model.h (i = 0 pitch, 1 pitch rate, 2 wheel speed), for display. */
double ss_gain(int i);

/* duty = sum_i K[i] * trim[i] * (x[i] - xref[i]). trim[] are the GUI gain
 * multipliers (1.0 = the designed gain). The caller clamps the result. */
double ss_control(const double x[3], const double xref[3], const double trim[3]);

#endif
