#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

/*
 * speed_control -- closed-loop wheel speed controller (Lab 04, Part 7).
 *
 * SHARED MODULE: lives in ~/dev/cpe4700/modules/ (commit on main).
 */

int  speed_control_init(void);                            /* reset state; 0 = ok */
void speed_control(int motor, double target_speed);       /* call once per 1 ms tick */
void speed_control_cleanup(void);                         /* command 0 duty on all motors */

/* Lab 06: one PI step on a measured speed the caller already read. Returns
 * the duty (deadband added, clamped to +/-1) WITHOUT writing the PWM.
 * speed_control() is this plus enc_get_speed() and pwm_set_duty(). */
double speed_control_compute(int motor, double target_speed, double measured);

/* Optional: let main.c hand the controller a signed measured speed.
 * (enc_get_speed() is unsigned -- see TODO in speed_control.c.) */

#endif
