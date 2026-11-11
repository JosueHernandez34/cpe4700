#include <math.h>
#include "balance_ctrl.h"

static double clamp(double x, double lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

void ctrl_init(ctrl_t *c)
{
    c->kv = KV;
    c->kp = KP;
    c->ki = KI;
    c->kd = KD;
    c->integral = 0.0;
    c->speed_cmd = 0.0;
    c->armed = 0;
    c->tripped = 0;
    c->in_band_ms = 0;
    c->block_rest_ms = 0;
    c->stops_valid = 1;
    c->stop_fwd  = STOP_FWD_RAW_DEG  - PITCH_ZERO_DEFAULT_DEG;
    c->stop_back = STOP_BACK_RAW_DEG - PITCH_ZERO_DEFAULT_DEG;
}

void ctrl_set_stops(ctrl_t *c, double fwd, double back)
{
    if (isfinite(fwd) && isfinite(back)) {
        c->stop_fwd = fwd;
        c->stop_back = back;
        c->stops_valid = 1;
    } else {
        c->stops_valid = 0;
    }
}

double ctrl_stick_speed(double ly)
{
    return clamp(-ly * SPEED_MAX, SPEED_MAX);
}

double ctrl_stick_steer(double rx)
{
    return clamp(-rx * ROBOT_STEER_SIGN, 1.0);
}

void ctrl_set_speed(ctrl_t *c, double target, double dt)
{
    target = clamp(target, SPEED_MAX);
    double step = SPEED_SLEW * dt;
    double d = target - c->speed_cmd;
    c->speed_cmd += clamp(d, step);
}

double ctrl_steer_scale(const ctrl_t *c)
{
    double s = fabs(c->speed_cmd) / STEER_FULL_SPEED;
    return s > 1.0 ? 1.0 : s;
}

void ctrl_zero_speed(ctrl_t *c)
{
    c->speed_cmd = 0.0;
}

int ctrl_update(ctrl_t *c, const ctrl_in_t *in, ctrl_out_t *out)
{
    /* Speed error moves the pitch target, within a small range around theta0.
     * Forward speed wanted (positive error) leans the robot forward. */
    double speed_err = c->speed_cmd - in->speed;
    out->pitch_target = THETA0_DEG + clamp(c->kv * speed_err, PITCH_TARGET_RANGE);
    /* Lab 05 rule: the target stays inside half of the smaller cutoff. */
    double half_cut = 0.5 * (CUT_FWD_DEG < CUT_BACK_DEG ? CUT_FWD_DEG : CUT_BACK_DEG);
    out->pitch_target = clamp(out->pitch_target, half_cut);
    out->error = out->pitch_target - in->pitch;
    out->pid_output = 0.0;
    out->left = 0.0;
    out->right = 0.0;

    /* Safety: a cutoff beyond the limits (or a non-finite pitch) latches the motors off once they are
     * armed. Before arming the motors are already off, so a robot that is tilted or being carried
     * simply stays off until it is held near theta0. */
    int beyond = !isfinite(in->pitch) || in->pitch > CUT_FWD_DEG || in->pitch < -CUT_BACK_DEG;
    if (c->tripped || (c->armed && beyond)) {
        c->tripped = 1;
        c->armed = 0;
        c->integral = 0.0;
        return 0;
    }

    /* Arming: near the balance angle for ARM_HOLD_MS before the motors go live. */
    if (!c->armed) {
        if (fabs(in->pitch - THETA0_DEG) < ARM_BAND_DEG)
            c->in_band_ms++;
        else
            c->in_band_ms = 0;
        if (c->in_band_ms >= ARM_HOLD_MS)
            c->armed = 1;
        c->integral = 0.0;
        return 0;
    }

    /* Block-rest trip: resting against a stop (Lab 05 v2). */
    double hi = c->stop_fwd > c->stop_back ? c->stop_fwd : c->stop_back;
    double lo = c->stop_fwd > c->stop_back ? c->stop_back : c->stop_fwd;
    int near_block = c->stops_valid && (in->pitch >= hi - NEAR_BLOCK_DEG || in->pitch <= lo + NEAR_BLOCK_DEG);
    if (near_block && fabs(in->pitch_rate) < NEAR_BLOCK_RATE) {
        if (++c->block_rest_ms >= BLOCK_REST_MS) {
            c->tripped = 1;
            c->armed = 0;
            c->integral = 0.0;
            return 0;
        }
    } else {
        c->block_rest_ms = 0;
    }

    /* P, I and D written out. Derivative is on the measurement: de/dt = -pitch_rate
     * (the target moves slowly, so this avoids a kick when the speed command steps). */
    double p = c->kp * out->error;
    c->integral += out->error * LOOP_MS_DT;
    if (c->ki > 0.0)
        c->integral = clamp(c->integral, I_LIMIT / c->ki);   /* anti-windup */
    else
        c->integral = 0.0;
    double i = c->ki * c->integral;
    double d = c->kd * (-in->pitch_rate);
    out->pid_output = DRIVE_SIGN * (p + i + d);

    /* Lab 04 deadband compensation, ramped through zero. */
    double u = out->pid_output;
    u += (fabs(u) >= DEADBAND_RAMP ? (u > 0 ? 1.0 : -1.0) : u / DEADBAND_RAMP) * DEADBAND_DUTY;

    /* Steering strength grows with speed, so the robot veers while moving and does not pivot at a standstill. */
    double usteer = clamp(in->steer, 1.0) * STEER_DUTY * ctrl_steer_scale(c);
    out->left  = clamp(u - usteer, MAX_DUTY);
    out->right = clamp(u + usteer, MAX_DUTY);
    return 1;
}
