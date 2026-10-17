/*
 * speed_control.c -- PI speed controller with deadband compensation.
 *
 * Control law, per motor, evaluated every 1 ms:
 *
 *     error   = target - measured
 *     integ  += error * DT                (clamped: anti-windup)
 *     u       = KP*error + KI*integ       (+ deadband offset, see below)
 *     duty    = clamp(u, -1, +1)
 *     pwm_set_duty(motor, duty)
 *
 * The motor does not move below the deadband duty you measured in
 * Part 3, so we add a small offset in the direction of u to "jump" it.
 */

#include <math.h>
#include <stdio.h>

#include "speed_control.h"
#include "pwm.h"
#include "encoder.h"

#define NUM_MOTORS 2
#define DT         0.001      /* loop period, seconds (1 ms) */

/* ---- TODO: fill in from your Part 3 / Part 6 results ------------------ */
#define DEADBAND_POS   0.00   /* TODO: positive_deadband (duty, e.g. 0.12) */
#define DEADBAND_NEG   0.00   /* TODO: negative_deadband (magnitude)       */

/* ---- TODO: tune. Start with KI = 0, raise KP until it tracks, then add KI.
 * Units: duty per (speed unit). Your speed units are whatever
 * enc_get_speed() returns -- check them before choosing gains. ---------- */
#define KP   0.005
#define KI   0.020

#define INTEG_LIMIT 1.0       /* clamp on KI*integ contribution */

static double integ[NUM_MOTORS];

int speed_control_init(void)
{
    for (int m = 0; m < NUM_MOTORS; m++)
        integ[m] = 0.0;
    return 0;
}

void speed_control(int motor, double target_speed)
{
    if (motor < 0 || motor >= NUM_MOTORS)
        return;

    /* enc_get_speed() is signed (A-edges/s, negative in reverse) and reads
     * 0 when no edge has arrived for ~50 ms, so it can track +/- targets. */
    double measured = enc_get_speed(motor);

    double error = target_speed - measured;

    /* Integrate with anti-windup clamp. */
    integ[motor] += error * DT;
    if (KI > 0.0) {
        double lim = INTEG_LIMIT / KI;
        if (integ[motor] >  lim) integ[motor] =  lim;
        if (integ[motor] < -lim) integ[motor] = -lim;
    }

    double u = KP * error + KI * integ[motor];

    /* Deadband compensation: push past the dead zone when commanding motion. */
    if (u > 0.0)       u += DEADBAND_POS;
    else if (u < 0.0)  u -= DEADBAND_NEG;

    if (u >  1.0) u =  1.0;
    if (u < -1.0) u = -1.0;

    pwm_set_duty(motor, (float)u);
}

void speed_control_cleanup(void)
{
    for (int m = 0; m < NUM_MOTORS; m++) {
        pwm_set_duty(m, 0.0f);
        integ[m] = 0.0;
    }
}
