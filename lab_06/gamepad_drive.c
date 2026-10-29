// Spins the wheels from the left joystick. No balancing, no encoders: the
// stick goes straight to motor duty, so put the robot on blocks first.
//
//   left stick up/down    : both wheels forward/back, speed = how far pushed
//   left stick left/right : turn (one wheel faster, the other slower)
//
// Duty is capped at MAX_DUTY. Motors stop on Ctrl-C and whenever the
// controller is unplugged (gamepad.c zeroes the axes then).
//
// Build (from lab_06/):
//   make gamepad_drive
// Run (needs the same permissions as robotest for PWM and GPIO):
//   ./gamepad_drive

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "gamepad.h"
#include "pwm.h"

#define MAX_DUTY   0.30    // same cap as the balance loop default
#define LOOP_MS    20      // 50 Hz is plenty for a hand on a stick

static volatile sig_atomic_t stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    stop = 1;
}

static double clamp(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

int main(void)
{
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    if (gp_init(NULL) < 0) {
        fprintf(stderr, "gp_init failed\n");
        return 1;
    }
    if (pwm_init() < 0) {
        fprintf(stderr, "pwm_init failed (try running it the way you run robotest)\n");
        gp_cleanup();
        return 1;
    }

    printf("Left stick drives the wheels (max duty %.2f). Ctrl-C to stop.\n", MAX_DUTY);

    while (!stop) {
        // Stick up reads negative, so flip it: up = forward.
        double fwd  = -gp_get_axis(GP_AXIS_LY);
        double turn =  gp_get_axis(GP_AXIS_LX);

        // Arcade mix, then scale so neither wheel goes past +/-1 before the cap.
        double left  = fwd + turn;
        double right = fwd - turn;
        double big = clamp(left < 0 ? -left : left, 1.0, 2.0);
        double big_r = right < 0 ? -right : right;
        if (big_r > big)
            big = big_r;
        left  = left  / big * MAX_DUTY;
        right = right / big * MAX_DUTY;

        if (!gp_is_connected())
            left = right = 0.0;

        // Channel 0 / 1 to left / right is a guess; swap if turning is reversed.
        pwm_set_duty(0, (float)left);
        pwm_set_duty(1, (float)right);

        printf("\r%s  fwd %+.2f turn %+.2f -> duty0 %+.2f duty1 %+.2f   ",
               gp_is_connected() ? "pad OK  " : "NO PAD  ", fwd, turn, left, right);
        fflush(stdout);
        usleep(LOOP_MS * 1000);
    }

    pwm_set_duty(0, 0.0f);      // motors off first
    pwm_set_duty(1, 0.0f);
    pwm_cleanup();
    gp_cleanup();
    printf("\nmotors off\n");
    return 0;
}
