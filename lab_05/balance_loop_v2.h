#ifndef BALANCE_LOOP_H
#define BALANCE_LOOP_H

/*
 * balance_loop_v2 -- pitch-based self-balancing controller (Lab 05), version 2.
 *
 * WHAT CHANGED FROM v1 (balance_loop.h, left untouched):
 *   - The motor cutoff is set from the GUI (two sliders, forward and back), default +/-15 deg,
 *     range 2..30 deg per side, applied live. v1 fixed it at (stop - margin) with a 5 deg floor.
 *   - A standing band (two more sliders, default -8 / +9 deg) is stored and written to the log so
 *     identify_v2.py knows which data was taken while the robot stood free. It does not move the motors.
 *   - Block-rest trip: with a cutoff wider than the block, resting on the block would never trip,
 *     so the loop now latches the motors off if the robot sits against a block for 1 s.
 *   - Lift window: for 5 s after Enable goes ON the block-rest trip is held off, so the controller
 *     can lift the robot off the block. If it never leaves the block in 5 s: LIFT FAILED, latched off.
 *   - Launch defaults per Josue 10/26: max duty 0.60, ID step 0.05.
 *   - Block stops are measured in the GUI with the motors off (tilt onto each block, press Measure)
 *     or typed, instead of the rough ss_model.h values; signed, any value; logged per row as
 *     stop_fwd, stop_back.
 * Same function names as v1, so robotest.c links against either version unchanged.
 *
 * balance_loop_step() runs in the 1 ms worker thread; the GUI runs in the
 * main thread. Gains and telemetry are shared through atomics.
 *
 * Units and sign conventions (used by balance_loop.c and the GUI):
 *   pitch      degrees, 0 = robot upright, FORWARD lean = POSITIVE
 *   pitch rate degrees per second, same sign as pitch
 *   duty       -1..1 sent to pwm_set_duty(), clamped to +/-BALANCE_DUTY_LIMIT
 *   Kp         duty per degree of pitch
 *   Kd         duty per (degree/second) of pitch rate
 *   Ki         duty per (degree*second) of accumulated pitch
 */

#include <stddef.h>
#include "state_space.h"      /* ss_mode_t */

#define BALANCE_DUTY_LIMIT   0.75    /* |duty| clamp before pwm_set_duty() (handout) */
#define BALANCE_PITCH_TARGET 0.0     /* degrees, upright */
/* Motor cutoff, per side, degrees (magnitudes). Set live from the GUI sliders. */
#define BALANCE_CUTOFF_DEFAULT_DEG 15.0  /* launch value, both sides (Josue 10/26) */
#define BALANCE_CUTOFF_MIN_DEG      2.0  /* slider bottom: the cutoff can never be switched off */
#define BALANCE_CUTOFF_MAX_DEG     30.0  /* slider top (Josue 10/26: 30, not 45) */

/* Standing band, degrees (signed: lo is the back edge, hi the forward edge). Logged only.
 * Calibrated from the stops (Josue 10/26): at launch and every time the stops change through
 * balance_loop_center_zero(), band = (back stop + margin, forward stop - margin). It can still be
 * typed; a typed band holds until the next calibration. */
#define BALANCE_BAND_STOP_MARGIN_DEG  1.0
#define BALANCE_BAND_LIMIT_DEG    30.0   /* slider range -30..+30; lo < hi is enforced */

/* Lift window and block-rest trip (decided 10/26: ADD the trip, 5 s window). */
#define BALANCE_LIFT_WINDOW_MS   5000    /* ms after Enable ON with the block-rest trip held off */
#define BALANCE_BLOCK_REST_MS    1000    /* ms resting against a block before the trip latches */

/* Why the motors are latched off (balance_loop_get_trip_reason). */
typedef enum { TRIP_NONE = 0, TRIP_CUTOFF, TRIP_BLOCK_REST, TRIP_LIFT_FAILED } trip_reason_t;

#define BALANCE_DEFAULT_DUTY_LIMIT 0.60  /* tuning limit at launch (Josue 10/26, was 0.30); the GUI
                                          * slider can change it, up to BALANCE_DUTY_LIMIT. */
#define BALANCE_DEFAULT_ID_STEP    0.05  /* duty: Run ID step size at launch (Josue 10/26, was 0.08) */

/* Pitch zero: the IMU reading halfway between the forward and back block stops (Josue 10/26: "use the
 * midpoint of the measured back and front values"). It is subtracted from the IMU pitch before anything
 * uses it, so every angle in this program (target, cutoff, band, stops, log) is measured from that
 * midpoint, and the two stops are always +/- half the span between them. */
/* Block stops on the ground as the IMU read them (Josue 10/26). At launch the zero is their midpoint,
 * (11.25 + -3.56) / 2 = +3.845 deg, so the stops start at +/-7.405 deg. */
#define BALANCE_STOP_FWD_RAW_DEG       11.25
#define BALANCE_STOP_BACK_RAW_DEG      -3.56
#define BALANCE_DEFAULT_TARGET     0.0   /* deg: target pitch at launch (positive = lean forward) */

/* PID launch defaults (approved by Josue 10/21: close to balancing, max duty 0.30 above). Only used in
 * PID mode; in state-feedback mode the sliders start at 1.0 (multipliers on the designed gains). */
#define BALANCE_DEFAULT_KP  0.015    /* duty per deg */
#define BALANCE_DEFAULT_KI  0.005    /* duty per (deg*s) */
#define BALANCE_DEFAULT_KD  0.005    /* duty per (deg/s) */

void balance_loop_init(void);                                   /* reset gains/state, motors disabled */
void balance_loop_step(unsigned int loop_count);                /* call once per 1 ms tick */
void balance_loop_set_gains(double kp, double ki, double kd);   /* thread-safe */
void balance_loop_cleanup(void);                                /* command 0 duty on both motors */

/* Control path chosen at startup (see state_space.h): state feedback, PID
 * fallback or refused. The text is one line for the GUI that says which and why. */
ss_mode_t balance_loop_control_mode(void);
void balance_loop_control_text(char *buf, size_t n);

/* Slider values at launch and after "Reset Gains", returned as (Kp, Kd, Ki):
 * PID mode: 0, 0, 0. State-feedback mode: 1, 1, 1 (multipliers on the designed
 * gains; Kp->K_theta, Kd->K_rate, Ki->K_omega, see balance_loop.c). */
void balance_loop_default_gains(double *kp, double *kd, double *ki);

/* Cutoff angles, degrees (thread-safe): motors are cut if pitch > +fwd or pitch < -back.
 * set_cutoff clamps each side into BALANCE_CUTOFF_MIN_DEG..BALANCE_CUTOFF_MAX_DEG and applies it live. */
void balance_loop_set_cutoff(double fwd, double back);
void balance_loop_get_cutoff(double *fwd, double *back);

/* Standing band, degrees (thread-safe). Clamped to +/-BALANCE_BAND_LIMIT_DEG; if lo >= hi the call
 * is ignored and the band keeps its previous values. Only logged, never acts on the motors. */
void balance_loop_set_band(double lo, double hi);
void balance_loop_get_band(double *lo, double *hi);

/* Block stop angles, SIGNED pitch in deg (forward stop normally positive, back stop normally negative),
 * used by the block-rest trip and the lift window. They start at +stop_fwd / -stop_back from ss_model.h;
 * the GUI replaces them by measuring the pitch with the robot tilted onto each block, or by typing.
 * Any values are accepted, at any time (Josue 10/26). The "near a block" zone is within NEAR_BLOCK_DEG
 * of the larger value or beyond it, and within NEAR_BLOCK_DEG of the smaller value or beyond it. */
void balance_loop_get_stops(double *fwd, double *back);
void balance_loop_set_stops(double fwd, double back);

/* Pitch zero in deg (thread-safe), the raw IMU reading at the midpoint of the two stops.
 * center_zero() moves the zero to the midpoint of the stops in force and shifts both stops with it, so
 * they keep pointing at the same physical blocks and end up symmetric, then re-calibrates the band to
 * the stops minus BALANCE_BAND_STOP_MARGIN_DEG. Call it after changing a stop.
 * set_pitch_zero() sets it directly (same stop shift); used by the host test. */
void   balance_loop_center_zero(void);
void   balance_loop_set_pitch_zero(double zero_deg);
double balance_loop_get_pitch_zero(void);

/* Why the motors are latched off (TRIP_NONE while running or OFF), and ms left in the
 * lift window (0 once it is over or the block-rest trip is armed). */
trip_reason_t balance_loop_get_trip_reason(void);
int balance_loop_get_lift_left_ms(void);

/* Target pitch in degrees (thread-safe). A small positive value makes the
 * controller hold a slight forward lean, so the robot creeps forward. */
void balance_loop_set_target(double target_deg);

/* Tuning duty limit, 0..BALANCE_DUTY_LIMIT (thread-safe). Values outside
 * that range are clamped into it. */
void balance_loop_set_duty_limit(double limit);

/* System identification run (thread-safe). While the robot is balancing, adds
 * a known pseudo-random duty sequence for 20 s so the log can be used to fit
 * a model (see identify.py). Ignored unless the motors are ON and not cut off.
 * amplitude: step size in duty, capped at 0.20. */
void balance_loop_set_id_amplitude(double amp);
void balance_loop_start_id(void);
int  balance_loop_get_id_left_ms(void);     /* 0 when no run is active */

#define BALANCE_ID_DURATION_MS 20000   /* ms: length of one Run ID (same as ID_DURATION_MS in balance_loop.c) */

/* Live view of a Run ID for the GUI (thread-safe): ms left (0 = not running), the duty step being injected
 * right now, the samples taken so far in this run, and how many of them were spent resting against a block. */
void balance_loop_get_id_stats(int *left_ms, double *inj, unsigned int *total, unsigned int *near_block);

/* Write the log since just before the current ID run to a CSV (for the live fit in identify.py).
 * Returns the number of rows, or -1 on failure. Called from the GUI thread. */
int balance_loop_write_snapshot(const char *path);

/* Motor enable switch (thread-safe). Starts OFF on every launch. While OFF,
 * or after the cutoff trips, the loop commands 0 duty. */
void balance_loop_set_enabled(int enabled);

/* Latest values for the GUI (thread-safe snapshot). pitch is the corrected
 * angle (0 upright, forward positive). */
void balance_loop_get_state(double *pitch, double *rpm0, double *rpm1);

/* Duty most recently sent to the motors (-1..1, after clamping), for the GUI. */
double balance_loop_get_duty(void);

/* enabled: switch position. tripped: 1 if the cutoff, the block-rest trip or a failed lift
 * stopped the motors (flip the switch OFF and ON again to re-arm). */
void balance_loop_get_status(int *enabled, int *tripped);

#endif
