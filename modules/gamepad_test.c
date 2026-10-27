// Prints live gamepad values so you can see which control is which.
//
// Build (from modules/):
//   gcc -Wall -Wextra -O2 -I. -o gamepad_test gamepad_test.c gamepad.c -lpthread
// Run:
//   ./gamepad_test              (auto-detect the controller)
//   ./gamepad_test /dev/input/event11
// Ctrl-C to quit.

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <linux/input.h>
#include "gamepad.h"

static volatile sig_atomic_t stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    stop = 1;
}

static const struct {
    int code;
    const char *name;
} BUTTONS[] = {
    { BTN_SOUTH,    "BTN_SOUTH" },   { BTN_EAST,    "BTN_EAST" },
    { BTN_C,        "BTN_C" },       { BTN_NORTH,   "BTN_NORTH" },
    { BTN_WEST,     "BTN_WEST" },    { BTN_Z,       "BTN_Z" },
    { BTN_TL,       "BTN_TL" },      { BTN_TR,      "BTN_TR" },
    { BTN_TL2,      "BTN_TL2" },     { BTN_TR2,     "BTN_TR2" },
    { BTN_SELECT,   "BTN_SELECT" },  { BTN_START,   "BTN_START" },
    { BTN_MODE,     "BTN_MODE" },    { BTN_THUMBL,  "BTN_THUMBL" },
    { BTN_THUMBR,   "BTN_THUMBR" },  { KEY_HOMEPAGE, "KEY_HOMEPAGE" },
};
#define NUM_BUTTONS (int)(sizeof(BUTTONS) / sizeof(BUTTONS[0]))

int main(int argc, char **argv)
{
    int last[NUM_BUTTONS] = {0};
    int was_connected = -1;

    signal(SIGINT, on_sigint);

    if (gp_init(argc > 1 ? argv[1] : NULL) < 0) {
        fprintf(stderr, "gp_init failed\n");
        return 1;
    }

    while (!stop) {
        int now = gp_is_connected();
        if (now != was_connected) {
            printf("\n%s\n", now ? "controller connected"
                                 : "waiting for controller...");
            was_connected = now;
        }

        // One line per button press or release.
        for (int i = 0; i < NUM_BUTTONS; i++) {
            int v = gp_get_button(BUTTONS[i].code);
            if (v != last[i]) {
                printf("\n%s %s\n", BUTTONS[i].name, v ? "pressed" : "released");
                last[i] = v;
            }
        }

        // One status line that updates in place.
        printf("\rLX %+.2f LY %+.2f | RX %+.2f RY %+.2f | L2 %.2f R2 %.2f"
               " | hat %+d %+d   ",
               gp_get_axis(GP_AXIS_LX), gp_get_axis(GP_AXIS_LY),
               gp_get_axis(GP_AXIS_RX), gp_get_axis(GP_AXIS_RY),
               gp_get_axis(GP_AXIS_L2), gp_get_axis(GP_AXIS_R2),
               (int)gp_get_axis(GP_AXIS_HAT_X), (int)gp_get_axis(GP_AXIS_HAT_Y));
        fflush(stdout);
        usleep(50 * 1000);
    }

    printf("\n");
    gp_cleanup();
    return 0;
}
