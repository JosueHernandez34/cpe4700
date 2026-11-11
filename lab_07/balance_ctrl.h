#ifndef BALANCE_CTRL_H
#define BALANCE_CTRL_H

/*
 * Lab 07 balance controller: the control math only, no hardware.
 * main.c feeds it measurements and writes its outputs to the PWM.
 * test_ctrl.c feeds it fake measurements, so the logic can be checked
 * with the robot switched off.
 *
 * Pitch convention (same as Lab 05): degrees, 0 = upright, forward lean positive.
 * Speed convention: encoder A-edges per second, forward positive.
 */

/* ---- TUNING BLOCK: every value to adjust lives here ---- */
#define THETA0_DEG      2.0      /* balance angle (deg from the calibrated zero); 2 because the robot is back-heavy (Josue 11/11) */

#define KP              0.015    /* duty per deg          (Lab 05 default) */
#define KI              0.005    /* duty per (deg * s)    (Lab 05 default) */
#define KD              0.005    /* duty per (deg / s)    (Lab 05 default) */
#define I_LIMIT         0.15     /* anti-windup: |Ki * integral| never above this (duty) */

/* Lab 05 drove +duty when leaning forward: u = DRIVE_SIGN * (Kp e + Ki int(e) + Kd de/dt)
 * with e = theta_target - theta, so DRIVE_SIGN is -1. Flip if the wheels run away from a lean. */
#define DRIVE_SIGN     -1.0

#define KV              0.0010   /* deg of pitch target per (edges/s) of speed error */
#define PITCH_TARGET_RANGE 3.0   /* pitch target stays within theta0 +/- this (deg) */

#define SPEED_STEP      200.0    /* edges/s per d-pad press */
#define SPEED_MAX       2000.0   /* edges/s, clamp on the speed command */
#define STEER_DUTY      0.08     /* duty difference between the wheels for a steering press */

#define MAX_DUTY        0.60     /* final clamp on both PWM commands (Labs 05 and 06) */

/* Pitch zero (Lab 05 v2 rule, Josue 10/26): the zero is the midpoint of the IMU readings at the
 * forward and back block stops, found by moving the robot to both ends and averaging. Every angle in
 * this program (theta0, cutoff, logged pitch) is measured from that zero, so the stops sit at
 * +/- half their span. main.c measures the stops with "lab07 --cal" and saves the zero; these are the
 * Lab 05 readings (10/26), used until a calibration file exists. */
#define STOP_FWD_RAW_DEG    11.25    /* IMU reading (0 upright raw frame) resting on the forward stop */
#define STOP_BACK_RAW_DEG   -3.56    /* ... and on the back stop */
#define PITCH_ZERO_DEFAULT_DEG  (0.5 * (STOP_FWD_RAW_DEG + STOP_BACK_RAW_DEG))   /* +3.845 */

/* Cutoff (Lab 05 v2 default, Josue 10/26): +/-15 deg from the zero. The stops are inside it, so it
 * only trips on a fall past the stops or a dead IMU (reads 180). Allowed range 2 to 30 deg. */
#define CUT_FWD_DEG     15.0
#define CUT_BACK_DEG    15.0
#define CUTOFF_MIN_DEG  2.0
#define CUTOFF_MAX_DEG  30.0
_Static_assert(CUT_FWD_DEG >= CUTOFF_MIN_DEG && CUT_FWD_DEG <= CUTOFF_MAX_DEG &&
               CUT_BACK_DEG >= CUTOFF_MIN_DEG && CUT_BACK_DEG <= CUTOFF_MAX_DEG, "cutoff outside 2..30 deg");
/* Lab 05 rule: the pitch target stays inside half of the smaller cutoff. */
_Static_assert(THETA0_DEG <= 0.5 * (CUT_FWD_DEG < CUT_BACK_DEG ? CUT_FWD_DEG : CUT_BACK_DEG),
               "theta0 must be inside half of the smaller cutoff");
/* Lab 04 deadband: wheels do not move until |duty| is about 0.06 (W0 +0.06/-0.06, W1 +0.06/-0.04).
 * Added to every nonzero output, ramped in over DEADBAND_RAMP so noise at zero does not chatter. */
#define DEADBAND_DUTY   0.06
#define DEADBAND_RAMP   0.02

/* Block-rest trip (Lab 05 v2): within NEAR_BLOCK_DEG of a stop and moving slower than
 * NEAR_BLOCK_RATE for BLOCK_REST_MS means the robot is resting on the block, so stop pushing into it.
 * (Lab 05's 5 s lift window is not needed: the arming step below already requires the robot to be
 * held off the blocks, near theta0.) */
#define NEAR_BLOCK_DEG  2.5
#define NEAR_BLOCK_RATE 5.0      /* deg/s */
#define BLOCK_REST_MS   1000
_Static_assert(THETA0_DEG < (STOP_FWD_RAW_DEG - PITCH_ZERO_DEFAULT_DEG) - NEAR_BLOCK_DEG &&
               THETA0_DEG > (STOP_BACK_RAW_DEG - PITCH_ZERO_DEFAULT_DEG) + NEAR_BLOCK_DEG,
               "theta0 must be outside the near-block zone");

#define ARM_BAND_DEG    2.0      /* motors stay off until |pitch - theta0| is within this ... */
#define ARM_HOLD_MS     500      /* ... for this long */

#define LOOP_MS_DT      0.001    /* s, control loop period */

typedef struct {
    double integral;      /* deg * s */
    double speed_cmd;     /* edges/s, signed */
    int    armed;
    int    tripped;       /* latched: tip-over seen, motors stay off */
    int    in_band_ms;
    int    block_rest_ms;
    double stop_fwd, stop_back;   /* block stops, deg from the zero (forward positive, back negative) */
} ctrl_t;

typedef struct {
    double pitch;         /* deg */
    double pitch_rate;    /* deg/s, forward positive */
    double speed;         /* edges/s measured, forward positive */
    int    steer;         /* -1, 0, +1 (+1 = d-pad left turn) */
} ctrl_in_t;

typedef struct {
    double pitch_target;  /* deg */
    double error;         /* deg, pitch_target - pitch */
    double pid_output;    /* duty, after DRIVE_SIGN, before steering and clamp */
    double left, right;   /* duty, exactly what is written to the PWM */
} ctrl_out_t;

void ctrl_init(ctrl_t *c);
void ctrl_set_stops(ctrl_t *c, double fwd, double back);   /* from the calibration */

/* D-pad press: +1 raises the forward command one step, -1 raises the reverse command one step. */
void ctrl_step_speed(ctrl_t *c, int dir);
void ctrl_zero_speed(ctrl_t *c);

/* One loop pass (call every LOOP_MS_DT). Returns 1 if the motors are live,
 * 0 if they are off (not armed yet, or tripped). out is always filled. */
int ctrl_update(ctrl_t *c, const ctrl_in_t *in, ctrl_out_t *out);

#endif
