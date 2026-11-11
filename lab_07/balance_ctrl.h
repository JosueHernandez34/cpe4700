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

#include "robot_params.h"

/* ---- TUNING BLOCK ----
 * Robot-wide measured values (gains, angles, stops, cutoff, deadband, limits) live in
 * ~/dev/cpe4700/modules/robot_params.h. Change them there. They are mapped to the short names this
 * lab uses below. Values that only Lab 07 needs are defined here. */
#define THETA0_DEG          ROBOT_BALANCE_ANGLE_DEG
#define KP                  ROBOT_KP
#define KI                  ROBOT_KI
#define KD                  ROBOT_KD
#define DRIVE_SIGN          ROBOT_DRIVE_SIGN
#define MAX_DUTY            ROBOT_MAX_DUTY
#define STOP_FWD_RAW_DEG    ROBOT_STOP_FWD_RAW_DEG
#define STOP_BACK_RAW_DEG   ROBOT_STOP_BACK_RAW_DEG
#define PITCH_ZERO_DEFAULT_DEG ROBOT_PITCH_ZERO_DEG
#define CUT_FWD_DEG         ROBOT_CUTOFF_FWD_DEG
#define CUT_BACK_DEG        ROBOT_CUTOFF_BACK_DEG
#define CUTOFF_MIN_DEG      ROBOT_CUTOFF_MIN_DEG
#define CUTOFF_MAX_DEG      ROBOT_CUTOFF_MAX_DEG
#define DEADBAND_DUTY       ROBOT_DEADBAND_DUTY
#define DEADBAND_RAMP       ROBOT_DEADBAND_RAMP
#define NEAR_BLOCK_DEG      ROBOT_NEAR_BLOCK_DEG
#define NEAR_BLOCK_RATE     ROBOT_NEAR_BLOCK_RATE
#define BLOCK_REST_MS       ROBOT_BLOCK_REST_MS

/* Lab 07 only. */
#define I_LIMIT         0.15     /* anti-windup: |Ki * integral| never above this (duty) */
#define KV              0.0010   /* deg of pitch target per (edges/s) of speed error */
#define PITCH_TARGET_RANGE 3.0   /* pitch target stays within theta0 +/- this (deg) */
#define SPEED_STEP      200.0    /* edges/s per d-pad press */
#define SPEED_MAX       2000.0   /* edges/s, clamp on the speed command */
#define STEER_DUTY      0.08     /* duty difference between the wheels for a steering press */

/* Pitch zero rule (Lab 05 v2): see robot_params.h. "lab07 --cal" measures the stops and saves the zero;
 * the defaults there are used until pitch_zero.txt exists. */
_Static_assert(CUT_FWD_DEG >= CUTOFF_MIN_DEG && CUT_FWD_DEG <= CUTOFF_MAX_DEG &&
               CUT_BACK_DEG >= CUTOFF_MIN_DEG && CUT_BACK_DEG <= CUTOFF_MAX_DEG, "cutoff outside the allowed range");
/* Lab 05 rule: the pitch target stays inside half of the smaller cutoff. */
_Static_assert(THETA0_DEG <= 0.5 * (CUT_FWD_DEG < CUT_BACK_DEG ? CUT_FWD_DEG : CUT_BACK_DEG),
               "theta0 must be inside half of the smaller cutoff");
/* The block-rest trip (Lab 05 v2) needs theta0 outside the near-block zone, or balancing would look like resting. */
_Static_assert(THETA0_DEG < (STOP_FWD_RAW_DEG - PITCH_ZERO_DEFAULT_DEG) - NEAR_BLOCK_DEG &&
               THETA0_DEG > (STOP_BACK_RAW_DEG - PITCH_ZERO_DEFAULT_DEG) + NEAR_BLOCK_DEG,
               "theta0 must be outside the near-block zone");

#define ARM_BAND_DEG    2.0      /* motors stay off until |pitch - theta0| is within this ... */
#define ARM_HOLD_MS     500      /* ... for this long */

#define LOOP_MS_DT      ROBOT_LOOP_PERIOD_S

typedef struct {
    double integral;      /* deg * s */
    double speed_cmd;     /* edges/s, signed */
    int    armed;
    int    tripped;       /* latched: tip-over seen, motors stay off */
    int    in_band_ms;
    int    block_rest_ms;
    int    stops_valid;   /* 0 = no block stops known (stopper removed): block-rest trip is off */
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
