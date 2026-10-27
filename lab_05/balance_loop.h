#ifndef BALANCE_LOOP_H
#define BALANCE_LOOP_H

/*
 * balance_loop -- pitch-based self-balancing controller (Lab 05).
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
#define BALANCE_CUTOFF_DEG   45.0    /* UPPER limit on the cutoff. The real cutoff is (stop angle - margin) on each
                                      * side when that is smaller (see balance_loop_get_cutoff), because a block under the
                                      * robot stops it falling past the stop, so a 45 deg cutoff could never trigger. */

#define BALANCE_DEFAULT_DUTY_LIMIT 0.30  /* tuning limit at launch; the GUI slider can raise it up to
                                          * BALANCE_DUTY_LIMIT. Low so a runaway stays gentle. */
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

/* Cutoff angles actually in use, degrees: motors are cut if pitch > +fwd or pitch < -back. These are
 * (stop angle - margin) per side, capped at BALANCE_CUTOFF_DEG. *stop_measured is 0 while the stop
 * angles are still the unconfirmed interim 25 deg. Resting on the block counts as a fall. */
void balance_loop_get_cutoff(double *fwd, double *back, int *stop_measured);

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

/* enabled: switch position. tripped: 1 if the |pitch| cutoff stopped the
 * motors (flip the switch OFF and ON again to re-arm). */
void balance_loop_get_status(int *enabled, int *tripped);

#endif
