/*
 * Joystick test for Lab 07. No motors, no IMU: it only reads the gamepad and shows what the
 * balance program would do with it. Uses the same mapping functions as lab07 (balance_ctrl.c).
 *
 *   make joystick_test
 *   sudo ./joystick_test            guided check: asks for each stick direction and button, prints PASS/FAIL
 *   sudo ./joystick_test live       live display of the mapped speed command and steering
 *   sudo ./joystick_test /dev/input/event12 [live]     pick the device yourself
 * Ctrl-C to quit.
 *
 * Mapping under test (balance_ctrl.h): left stick up = forward speed, down = reverse; right stick
 * left = left turn, right = right turn; B = stop; START = exit.
 */
#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/input.h>

#include "balance_ctrl.h"
#include "gamepad.h"

#define PASS_LEVEL  0.8     /* stick must pass this to count as pushed */
#define CENTRE_TOL  0.05    /* stick must be inside this when released */
#define CROSS_TOL   0.3     /* other axes should stay below this while one stick is pushed */
#define STEP_TIMEOUT_S 10

static volatile sig_atomic_t quit = 0;
static void on_sigint(int s) { (void)s; quit = 1; }

static const char *AXIS_NAME[GP_AXIS_COUNT] = { "LX", "LY", "RX", "RY", "L2", "R2", "HAT_X", "HAT_Y" };

static void sleep_ms(int ms)
{
    struct timespec d = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&d, NULL);
}

static int fails, checks;
static void result(int ok, const char *what, const char *hint)
{
    checks++;
    if (!ok) fails++;
    printf("  %s  %s%s%s\n", ok ? "PASS" : "FAIL", what, ok ? "" : "  -> ", ok ? "" : hint);
}

/* Which axis moved the most (by absolute value) right now. */
static int biggest_axis(double *val)
{
    int best = 0;
    double bv = 0.0;
    for (int i = 0; i < GP_AXIS_COUNT; i++) {
        double v = gp_get_axis(i);
        if (fabs(v) > fabs(bv)) { bv = v; best = i; }
    }
    *val = bv;
    return best;
}

/* Wait until axis a passes level in direction dir (+1 or -1). Tracks the other axes for cross-talk. */
static int wait_push(int a, int dir, double *worst_other, int *other_axis)
{
    *worst_other = 0.0;
    *other_axis = -1;
    for (int t = 0; t < STEP_TIMEOUT_S * 100 && !quit; t++) {
        double v = gp_get_axis(a);
        for (int i = 0; i < GP_AXIS_COUNT; i++) {
            if (i == a) continue;
            double o = fabs(gp_get_axis(i));
            if (o > *worst_other) { *worst_other = o; *other_axis = i; }
        }
        if (dir * v >= PASS_LEVEL) return 1;
        sleep_ms(10);
    }
    return 0;
}

static void wait_released(void)
{
    for (int t = 0; t < 500 && !quit; t++) {
        int ok = 1;
        for (int i = 0; i < 4; i++) if (fabs(gp_get_axis(i)) > CENTRE_TOL) ok = 0;
        if (ok) return;
        sleep_ms(10);
    }
}

static void stick_step(const char *prompt, int axis, int dir, const char *effect, int (*effect_ok)(double))
{
    printf("\n%s\n", prompt);
    fflush(stdout);
    double other; int oa;
    int got = wait_push(axis, dir, &other, &oa);
    if (!got) {
        double v; int ba = biggest_axis(&v);
        char hint[320];
        snprintf(hint, sizeof hint, "%s did not reach %.1f in %d s; the axis that moved most was %s (%+.2f). "
                 "If it is a different axis, the pad's mapping differs (see AXIS_MAP in modules/gamepad.c)",
                 AXIS_NAME[axis], dir * PASS_LEVEL, STEP_TIMEOUT_S, AXIS_NAME[ba], v);
        result(0, effect, hint);
        return;
    }
    double v = gp_get_axis(axis);
    char what[200];
    snprintf(what, sizeof what, "%s reads %+.2f -> %s", AXIS_NAME[axis], v, effect);
    result(effect_ok(v), what, "the sign is the other way round; flip it in balance_ctrl.c (ctrl_stick_speed / ctrl_stick_steer)");
    if (other > CROSS_TOL) {
        char w[120];
        snprintf(w, sizeof w, "other axes stayed quiet (%s reached %.2f)", AXIS_NAME[oa], other);
        result(0, w, "two axes move together; check the mapping or your thumb");
    }
    printf("  release the stick...\n");
    wait_released();
}

static int fwd_ok(double ly)   { return ctrl_stick_speed(ly) >=  PASS_LEVEL * SPEED_MAX; }
static int rev_ok(double ly)   { return ctrl_stick_speed(ly) <= -PASS_LEVEL * SPEED_MAX; }
static int left_ok(double rx)  { return ctrl_stick_steer(rx) * ROBOT_STEER_SIGN >=  PASS_LEVEL; }
static int right_ok(double rx) { return ctrl_stick_steer(rx) * ROBOT_STEER_SIGN <= -PASS_LEVEL; }

static void button_step(const char *prompt, int code, const char *name)
{
    printf("\n%s\n", prompt);
    fflush(stdout);
    int seen = 0;
    for (int t = 0; t < STEP_TIMEOUT_S * 100 && !quit && !seen; t++) {
        seen = gp_get_button(code);
        sleep_ms(10);
    }
    char what[80];
    snprintf(what, sizeof what, "%s (0x%x) pressed", name, code);
    result(seen, what, "not seen; run gamepad_test in lab_06 to find which code this button sends");
    for (int t = 0; t < 300 && gp_get_button(code); t++) sleep_ms(10);
}

static void guided(void)
{
    printf("\nGuided check. Follow each prompt within %d s. Nothing here drives the motors.\n", STEP_TIMEOUT_S);

    printf("\nLet go of both sticks and keep them centred for 2 s.\n");
    sleep_ms(2000);
    double worst = 0.0; int wa = 0;
    for (int i = 0; i < 4; i++) if (fabs(gp_get_axis(i)) > worst) { worst = fabs(gp_get_axis(i)); wa = i; }
    char w[120];
    snprintf(w, sizeof w, "all four stick axes read zero at rest (worst %s %.2f)", AXIS_NAME[wa], worst);
    result(worst <= CENTRE_TOL, w, "a stick does not centre; the deadzone is GP_DEADZONE in gamepad.h");

    stick_step("Push the LEFT stick UP and hold.",    GP_AXIS_LY, -1, "speed command is FORWARD", fwd_ok);
    stick_step("Push the LEFT stick DOWN and hold.",  GP_AXIS_LY, +1, "speed command is REVERSE", rev_ok);
    stick_step("Push the RIGHT stick LEFT and hold.", GP_AXIS_RX, -1, "steering is a LEFT turn",  left_ok);
    stick_step("Push the RIGHT stick RIGHT and hold.", GP_AXIS_RX, +1, "steering is a RIGHT turn", right_ok);
    button_step("Press B.", BTN_SOUTH, "B (stop)");
    button_step("Press START (+).", BTN_START, "START (exit)");

    printf("\n%d of %d checks passed.\n", checks - fails, checks);
    if (fails) printf("Fix the FAIL lines above, then run again.\n");
    else printf("Mapping is good. Next: wheels in the air, `sudo ./lab07`.\n");
}

static void live(void)
{
    ctrl_t c;
    ctrl_init(&c);
    printf("\nLive (no motors). Left stick = speed, right stick = steer, B = zero, START = quit.\n");
    while (!quit && gp_is_connected() && !gp_get_button(BTN_START)) {
        double ly = gp_get_axis(GP_AXIS_LY), rx = gp_get_axis(GP_AXIS_RX);
        if (gp_get_button(BTN_SOUTH)) ctrl_zero_speed(&c);
        ctrl_set_speed(&c, ctrl_stick_speed(ly), 0.05);   /* this loop runs at 20 Hz */
        double steer = ctrl_stick_steer(rx);
        printf("\rLX %+.2f LY %+.2f | RX %+.2f RY %+.2f | speed target %+6.0f  command %+6.0f edges/s | steer %+.2f x speed scale %.2f = %+.3f duty   ",
               gp_get_axis(GP_AXIS_LX), ly, rx, gp_get_axis(GP_AXIS_RY), ctrl_stick_speed(ly), c.speed_cmd, steer, ctrl_steer_scale(&c), steer * ctrl_steer_scale(&c) * STEER_DUTY);
        fflush(stdout);
        sleep_ms(50);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    const char *dev = NULL;
    int do_live = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "live") == 0) do_live = 1;
        else dev = argv[i];
    }
    signal(SIGINT, on_sigint);
    if (gp_init(dev) != 0) { fprintf(stderr, "gamepad init failed\n"); return 1; }
    printf("Waiting for the controller (up to 10 s)...\n");
    for (int ms = 0; ms < 10000 && !quit && !gp_is_connected(); ms += 10) sleep_ms(10);
    if (!gp_is_connected()) {
        fprintf(stderr, "no controller found. Is it on and connected? Try: cat /proc/bus/input/devices\n");
        gp_cleanup();
        return 1;
    }
    printf("Controller connected.\n");
    if (do_live) live(); else guided();
    gp_cleanup();
    return fails != 0;
}
