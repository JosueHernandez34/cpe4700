/* Off-robot check of the Lab 07 control logic with fake measurements. */
#include <math.h>
#include <stdio.h>
#include "balance_ctrl.h"

static int fails;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } \
                              else printf("ok:   %s\n", msg); } while (0)

static ctrl_t armed_ctrl(void)
{
    ctrl_t c; ctrl_in_t in = { THETA0_DEG, 0, 0, 0 }; ctrl_out_t o;
    ctrl_init(&c);
    for (int i = 0; i < ARM_HOLD_MS + 5; i++) ctrl_update(&c, &in, &o);
    return c;
}

int main(void)
{
    ctrl_out_t o;
    ctrl_in_t in = { THETA0_DEG, 0, 0, 0 };

    /* Arming: off until held in the band long enough. */
    ctrl_t c; ctrl_init(&c);
    int live = 0;
    for (int i = 0; i < ARM_HOLD_MS - 1; i++) live |= ctrl_update(&c, &in, &o);
    CHECK(!live && o.left == 0.0, "motors off before the arming hold time");
    c = armed_ctrl();
    CHECK(ctrl_update(&c, &in, &o) == 1, "motors live after the hold time");

    /* Forward lean drives forward (positive duty), backward lean drives back. */
    in.pitch = THETA0_DEG + 3; ctrl_update(&c, &in, &o);
    CHECK(o.left > 0 && o.right > 0 && o.left == o.right, "forward lean gives equal positive duty");
    c = armed_ctrl(); in.pitch = THETA0_DEG - 3; ctrl_update(&c, &in, &o);
    CHECK(o.left < 0 && o.right < 0, "backward lean gives negative duty");

    /* P term size: Kp * 3 deg, DRIVE_SIGN applied (first sample, I and D tiny). */
    c = armed_ctrl(); in.pitch = THETA0_DEG + 3; ctrl_update(&c, &in, &o);
    CHECK(fabs(o.pid_output - KP * 3.0) < 1e-3, "P output equals Kp * error");

    /* D term opposes a forward rotation of the pitch (extra forward duty when falling forward). */
    c = armed_ctrl(); in.pitch = THETA0_DEG; in.pitch_rate = 10; ctrl_update(&c, &in, &o);
    CHECK(fabs(o.pid_output - KD * 10.0) < 1e-3, "D output equals Kd * pitch rate");
    in.pitch_rate = 0;

    /* Clamp: big error stays within MAX_DUTY, and left/right clamp separately with steering. */
    c = armed_ctrl(); in.pitch = CUT_FWD_DEG - 0.1; in.pitch_rate = 400; in.steer = 1;
    ctrl_update(&c, &in, &o);
    CHECK(fabs(o.left) <= MAX_DUTY && fabs(o.right) <= MAX_DUTY, "left and right never exceed MAX_DUTY");
    in.pitch_rate = 0; in.steer = 0;

    /* Deadband: a tiny error still gives at least the Lab 04 deadband duty. */
    c = armed_ctrl(); in.pitch = THETA0_DEG + 2.0; ctrl_update(&c, &in, &o);
    CHECK(o.left >= DEADBAND_DUTY && o.pid_output < DEADBAND_DUTY, "deadband compensation lifts a small output");
    c = armed_ctrl(); in.pitch = THETA0_DEG; ctrl_update(&c, &in, &o);
    CHECK(fabs(o.left) < 1e-9, "zero error gives zero duty (no deadband chatter at zero)");

    /* Steering: uL = u - usteer, uR = u + usteer. */
    c = armed_ctrl(); in.pitch = THETA0_DEG + 1; in.steer = 1; ctrl_update(&c, &in, &o);
    CHECK(fabs((o.right - o.left) - 2 * STEER_DUTY) < 1e-9, "steering splits the wheels by 2 * STEER_DUTY");
    in.steer = 0;

    /* Joysticks: left stick up (LY negative) is forward, right stick left (RX negative) is a left turn. */
    CHECK(ctrl_stick_speed(-1.0) == SPEED_MAX && ctrl_stick_speed(1.0) == -SPEED_MAX, "left stick up = forward, down = reverse, full = SPEED_MAX");
    CHECK(ctrl_stick_speed(0.0) == 0.0 && fabs(ctrl_stick_speed(-0.5) - 0.5 * SPEED_MAX) < 1e-9, "stick speed is proportional and zero at centre");
    CHECK(ctrl_stick_steer(-1.0) == ROBOT_STEER_SIGN && ctrl_stick_steer(1.0) == -ROBOT_STEER_SIGN && ctrl_stick_steer(0.0) == 0.0,
          "right stick left = left turn, right = right turn, zero at centre");
    CHECK(fabs(ctrl_stick_steer(-3.0)) <= 1.0, "steering stays within +/-1");

    /* Speed command ramps at SPEED_SLEW and clamps. */
    c = armed_ctrl(); ctrl_set_speed(&c, SPEED_MAX, 0.001);
    CHECK(fabs(c.speed_cmd - SPEED_SLEW * 0.001) < 1e-9, "speed command moves at most SPEED_SLEW * dt per pass");
    for (int i = 0; i < 5000; i++) ctrl_set_speed(&c, 10 * SPEED_MAX, 0.001);
    CHECK(c.speed_cmd == SPEED_MAX, "speed command clamps at SPEED_MAX");
    for (int i = 0; i < 5000; i++) ctrl_set_speed(&c, -10 * SPEED_MAX, 0.001);
    CHECK(c.speed_cmd == -SPEED_MAX, "reverse command clamps at -SPEED_MAX");
    ctrl_zero_speed(&c);
    CHECK(c.speed_cmd == 0.0, "zero command resets the speed");

    /* Speed to pitch target: forward command leans forward, limited to the range. */
    c = armed_ctrl(); in.pitch = THETA0_DEG; c.speed_cmd = 200.0;
    ctrl_update(&c, &in, &o);
    CHECK(fabs(o.pitch_target - (THETA0_DEG + KV * 200.0)) < 1e-9, "forward speed command raises the pitch target");
    c.speed_cmd = SPEED_MAX; in.speed = -100000; ctrl_update(&c, &in, &o);
    double top = THETA0_DEG + PITCH_TARGET_RANGE;
    double half_cut = 0.5 * (CUT_FWD_DEG < CUT_BACK_DEG ? CUT_FWD_DEG : CUT_BACK_DEG);
    if (top > half_cut) top = half_cut;
    CHECK(fabs(o.pitch_target - top) < 1e-9, "pitch target clamps to theta0 + range and to half the cutoff");
    in.speed = 0;
    c = armed_ctrl(); ctrl_update(&c, &in, &o);
    CHECK(fabs(o.pitch_target - THETA0_DEG) < 1e-9, "zero command and zero speed keep target at theta0");

    /* Anti-windup: a long constant error cannot push the I term past I_LIMIT. */
    c = armed_ctrl(); in.pitch = THETA0_DEG + 4;
    for (int i = 0; i < 60000; i++) ctrl_update(&c, &in, &o);
    CHECK(fabs(KI * c.integral) <= I_LIMIT + 1e-9, "integral term stays within I_LIMIT");

    /* Gains can be overridden at run time (P only, then P+D, then P+D+I). */
    c = armed_ctrl(); c.kd = 0.0; c.ki = 0.0; in.pitch = THETA0_DEG + 3; in.pitch_rate = 50;
    ctrl_update(&c, &in, &o);
    CHECK(fabs(o.pid_output - KP * 3.0) < 1e-3 && c.integral == 0.0, "P only: no D or I contribution");
    c = armed_ctrl(); c.kd = 0.01; c.ki = 0.0; in.pitch = THETA0_DEG; in.pitch_rate = 10; ctrl_update(&c, &in, &o);
    CHECK(fabs(o.pid_output - 0.01 * 10.0) < 1e-3, "overridden Kd is used");
    in.pitch_rate = 0;

    /* Block rest: sitting near a stop, hardly moving, for 1 s cuts the motors; moving does not. */
    c = armed_ctrl(); in.pitch = c.stop_fwd - 1.0; in.pitch_rate = 0;
    for (int i = 0; i < BLOCK_REST_MS - 2; i++) ctrl_update(&c, &in, &o);
    CHECK(!c.tripped, "no block-rest trip before 1 s");
    for (int i = 0; i < 5; i++) ctrl_update(&c, &in, &o);
    CHECK(c.tripped && o.left == 0.0, "resting near the forward stop for 1 s trips");
    c = armed_ctrl(); in.pitch = c.stop_back + 1.0;
    for (int i = 0; i < BLOCK_REST_MS + 5; i++) ctrl_update(&c, &in, &o);
    CHECK(c.tripped, "resting near the back stop also trips");
    c = armed_ctrl(); in.pitch = c.stop_fwd - 1.0; in.pitch_rate = 20;
    for (int i = 0; i < 3 * BLOCK_REST_MS; i++) ctrl_update(&c, &in, &o);
    CHECK(!c.tripped, "moving near a stop does not trip");
    c = armed_ctrl(); in.pitch = THETA0_DEG; in.pitch_rate = 0;
    for (int i = 0; i < 3 * BLOCK_REST_MS; i++) ctrl_update(&c, &in, &o);
    CHECK(!c.tripped, "balancing still at theta0 does not trip");
    in.pitch_rate = 0;

    /* No stops saved (stopper removed): the block-rest trip is off, balancing near any angle is fine. */
    c = armed_ctrl(); ctrl_set_stops(&c, NAN, NAN); in.pitch = THETA0_DEG; in.pitch_rate = 0;
    for (int i = 0; i < 3 * BLOCK_REST_MS; i++) ctrl_update(&c, &in, &o);
    CHECK(!c.tripped && !c.stops_valid, "no stops: block-rest trip is off");

    /* Before arming, a tilted robot just stays off; it does not trip or end the run. */
    ctrl_init(&c); in.pitch = CUT_FWD_DEG + 20; in.pitch_rate = 0;
    live = ctrl_update(&c, &in, &o);
    CHECK(!live && !c.tripped && o.left == 0.0, "tilted before arming: motors off, no trip");

    /* Tip-over: cuts the motors and latches, even if the pitch returns. */
    c = armed_ctrl(); in.pitch = CUT_FWD_DEG + 1; ctrl_update(&c, &in, &o);
    CHECK(c.tripped && o.left == 0.0 && o.right == 0.0, "tip-over cuts both motors");
    in.pitch = THETA0_DEG; live = ctrl_update(&c, &in, &o);
    CHECK(!live && o.left == 0.0, "tip-over stays latched");
    c = armed_ctrl(); in.pitch = -CUT_BACK_DEG - 1; ctrl_update(&c, &in, &o);
    CHECK(c.tripped, "backward tip-over also trips");
    c = armed_ctrl(); in.pitch = NAN; ctrl_update(&c, &in, &o);
    CHECK(c.tripped, "NaN pitch trips the safety");

    printf("%s\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return fails != 0;
}
