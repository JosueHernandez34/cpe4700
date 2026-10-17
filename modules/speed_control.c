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

/* ---- TODO: fill in from your Part 3 results, one entry per wheel -------- */
static const double DEADBAND_POS[NUM_MOTORS] = { 0.00, 0.00 };  /* positive_deadband */
static const double DEADBAND_NEG[NUM_MOTORS] = { 0.00, 0.00 };  /* negative_deadband (magnitude) */

/* ---- Gains: duty per (edge/s). Starting point from the Lab 04 step response:
 * plant gain K ~ 6100 edges/s per duty, time constant tau ~ 0.04 s.
 * KP = 1e-4 gives loop gain K*KP ~ 0.6; KI = KP/tau matches the integral
 * time to the motor. Adjust from the speed_control.csv results. -------- */
#define KP   0.0001
#define KI   0.0025

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
    if (u > 0.0)       u += DEADBAND_POS[motor];
    else if (u < 0.0)  u -= DEADBAND_NEG[motor];

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
