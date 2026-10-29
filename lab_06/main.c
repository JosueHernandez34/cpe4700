/*
 * main.c -- Lab 06: gamepad-controlled speed and steering
 *
 * Usage:  sudo ./lab06 [event_device]
 *
 *   event_device : for example /dev/input/event11. Leave it out to use the
 *                  first device that looks like a gamepad. Find it with
 *                  grep -B2 -A8 "Pro Controller" /proc/bus/input/devices
 *
 * Writes lab06_YYYYMMDD_HHMMSS.csv in the current folder, so a new run never
 * overwrites an old one.
 *
 * Robot must be supported (wheels off the ground) until the steering sign and
 * left/right channels have been checked. START exits, B stops, Ctrl-C exits.
 *
 * ---------------------------------------------------------------------------
 * CONTROLLER MAPPING (Nintendo Switch Pro Controller, Linux evdev)
 *
 *   Control        evdev code                     value
 *   D-pad up/down  ABS_HAT0Y (0x11)               -1 up, 0 released, +1 down
 *                  or BTN_DPAD_UP/DOWN (0x220/1)  1 pressed, 0 released
 *   D-pad l/r      ABS_HAT0X (0x10)               -1 left, 0 released, +1 right
 *                  or BTN_DPAD_LEFT/RIGHT (0x222/3)
 *   X  (top)       BTN_NORTH (0x133)              slow/fast toggle on press
 *   B  (bottom)    BTN_SOUTH (0x130)              stop on press
 *   +  (START)     BTN_START (0x13b)              exit on press
 *
 * Depending on the kernel driver the d-pad arrives either as the hat axis or
 * as four buttons, so both are read. These are the codes expected for this
 * pad but NOT YET CONFIRMED on the robot: make gamepad_test, run it, press
 * each control, and correct this table and the BTN_* defines below if needed.
 * ---------------------------------------------------------------------------
 *
 * Control, every 1 ms (same period as Lab 04):
 *   1. read the pad state (gamepad.c reads events in its own thread, so the
 *      loop never waits on the controller)
 *   2. read the feedback encoder, ubase = Lab 04 PI speed controller
 *   3. uL = ubase - usteer, uR = ubase + usteer, clamp both to +/-MAX_DUTY,
 *      write both PWMs
 *   4. log one row every 20 ms (the values just written)
 *   5. wait for the next 1 ms tick
 *
 * Speeds are in encoder A-edges/s, the Lab 04 unit.
 */

#define _POSIX_C_SOURCE 200809L

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <linux/input.h>

#include "pwm.h"
#include "encoder.h"
#include "speed_control.h"
#include "gamepad.h"

#define LOOP_NS       1000000L    /* 1 ms loop period (Lab 04) */
#define LOG_MS        20          /* one CSV row every 20 ms */
#define CONNECT_WAIT_MS 10000     /* time to wait for the pad at start */

/* Wheels. pwm.c already inverts channel 0 so +duty drives both forward.
 * Swap LEFT_CH and RIGHT_CH if the robot turns the wrong way round. */
#define LEFT_CH        0
#define RIGHT_CH       1
#define FEEDBACK_MOTOR 0          /* encoder used by the speed controller */

/* +1: D-pad left turns left. Flip to -1 if the robot turns the wrong way. */
#define STEER_SIGN    1.0

#define MAX_DUTY      0.60        /* final clamp on both PWM commands */

/* Mode 0 = slow, 1 = fast. Fast target ~2000/6100 + 0.06 deadband = ~0.39
 * duty, so fast plus steering stays under MAX_DUTY. */
static const double TARGET_SPEED[2] = { 800.0, 2000.0 };   /* edges/s */
static const double STEER_DUTY[2]   = { 0.08, 0.15 };      /* duty */

#define BTN_MODE_TOGGLE BTN_NORTH   /* X */
#define BTN_STOP        BTN_SOUTH   /* B */
#define BTN_EXIT        BTN_START   /* + */

static volatile sig_atomic_t running = 1;
static void on_signal(int sig) { (void)sig; running = 0; }

/* Sleep until the next absolute 1 ms tick -- avoids cumulative drift. */
static void tick(struct timespec *next)
{
    next->tv_nsec += LOOP_NS;
    if (next->tv_nsec >= 1000000000L) {
        next->tv_nsec -= 1000000000L;
        next->tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
}

static long elapsed_ms(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L
         + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

static double clamp(double x, double lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

/* D-pad as one of -1, 0, +1 per axis. Pressing both opposing directions
 * gives 0. Hat and d-pad buttons are combined so either driver works. */
static void read_dpad(int *up, int *down, int *left, int *right)
{
    double hx = gp_get_axis(GP_AXIS_HAT_X);
    double hy = gp_get_axis(GP_AXIS_HAT_Y);

    *up    = hy < 0.0 || gp_get_button(BTN_DPAD_UP);
    *down  = hy > 0.0 || gp_get_button(BTN_DPAD_DOWN);
    *left  = hx < 0.0 || gp_get_button(BTN_DPAD_LEFT);
    *right = hx > 0.0 || gp_get_button(BTN_DPAD_RIGHT);
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : NULL;
    const char *err = NULL;           /* set when the loop ends on an error */

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* Controller first, so a missing pad never leaves the motors set up. */
    if (gp_init(dev) != 0) {
        fprintf(stderr, "gamepad init failed\n");
        return 1;
    }
    printf("Waiting for controller%s%s...\n", dev ? " on " : "", dev ? dev : "");
    for (int ms = 0; ms < CONNECT_WAIT_MS && running && !gp_is_connected(); ms += 10) {
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    if (!gp_is_connected()) {
        fprintf(stderr, "no controller found (is it paired, and is %s right?)\n",
                dev ? dev : "auto-detect");
        gp_cleanup();
        return 1;
    }

    char fname[64];
    time_t wall = time(NULL);
    strftime(fname, sizeof fname, "lab06_%Y%m%d_%H%M%S.csv", localtime(&wall));
    FILE *log = fopen(fname, "w");
    if (!log) {
        perror(fname);
        gp_cleanup();
        return 1;
    }
    fprintf(log, "time_ms,target_speed,measured_speed,left_pwm,right_pwm,mode\n");

    if (pwm_init() != 0 || enc_init() != 0) {
        fprintf(stderr, "init failed (need sudo for sysfs/GPIO?)\n");
        fclose(log);
        gp_cleanup();
        return 1;
    }
    speed_control_init();

    printf("Logging to %s\n"
           "D-pad drives, X slow/fast, B stop, START exit. Mode: slow\n", fname);

    int mode = 0;                     /* 0 slow, 1 fast */
    int stopped = 0;                  /* B latched: wait for a new d-pad press */
    int prev_dir = 0, prev_toggle = 0, prev_stop = 0;

    struct timespec start, next;
    clock_gettime(CLOCK_MONOTONIC, &start);
    next = start;
    long next_log_ms = elapsed_ms(&start) + LOG_MS;

    while (running) {
        /* 1. Pad state. A disconnect or read error in gamepad.c shows up here. */
        if (!gp_is_connected()) {
            err = "controller disconnected or read error";
            break;
        }
        if (gp_get_button(BTN_EXIT))
            break;

        int up, down, left, right;
        read_dpad(&up, &down, &left, &right);
        int fwd  = up - down;             /* +1 forward, 0 none or both */
        int turn = left - right;          /* +1 left,    0 none or both */

        int toggle_btn = gp_get_button(BTN_MODE_TOGGLE);
        if (toggle_btn && !prev_toggle) {
            mode = !mode;
            printf("Mode: %s\n", mode ? "fast" : "slow");
        }
        prev_toggle = toggle_btn;

        int stop_btn = gp_get_button(BTN_STOP);
        if (stop_btn && !prev_stop) {
            stopped = 1;
            printf("STOP (press a d-pad direction to resume)\n");
        }
        prev_stop = stop_btn;

        /* Resume only on a new press, not a direction still held from before B. */
        int dir_now = up | down | left | right;
        if (stopped && dir_now && !prev_dir && !stop_btn) {
            stopped = 0;
            printf("Resumed\n");
        }
        prev_dir = dir_now;

        double target_speed = stopped ? 0.0 : fwd * TARGET_SPEED[mode];
        double usteer = stopped ? 0.0 : STEER_SIGN * turn * STEER_DUTY[mode];

        /* 2. Feedback and Lab 04 speed controller. With no target the
         * controller is reset, so a leftover integral cannot creep the wheels. */
        double measured_speed = enc_get_speed(FEEDBACK_MOTOR);
        double ubase;
        if (target_speed == 0.0) {
            speed_control_init();
            ubase = 0.0;
        } else {
            ubase = speed_control_compute(FEEDBACK_MOTOR, target_speed, measured_speed);
        }

        /* 3. Steering, final clamp, write. Logged values are these floats. */
        float left_pwm  = (float)clamp(ubase - usteer, MAX_DUTY);
        float right_pwm = (float)clamp(ubase + usteer, MAX_DUTY);
        if (pwm_set_duty(LEFT_CH, left_pwm) < 0 || pwm_set_duty(RIGHT_CH, right_pwm) < 0) {
            err = "pwm_set_duty failed";
            break;
        }

        /* 4. One row every 20 ms, from this same iteration. */
        long t_ms = elapsed_ms(&start);
        if (t_ms >= next_log_ms) {
            fprintf(log, "%ld,%.3f,%.3f,%.3f,%.3f,%d\n", t_ms, target_speed,
                    measured_speed, left_pwm, right_pwm, mode);
            next_log_ms += LOG_MS;
            /* If the loop ever stalls past a whole slot, skip it rather than
             * writing catch-up rows with the same timestamp. */
            while (next_log_ms <= t_ms)
                next_log_ms += LOG_MS;
        }

        /* 5. Next 1 ms tick. */
        tick(&next);
    }

    /* Motors off first, then the log, then module cleanup. */
    pwm_set_duty(LEFT_CH, 0.0f);
    pwm_set_duty(RIGHT_CH, 0.0f);
    fclose(log);
    speed_control_cleanup();
    pwm_cleanup();
    enc_cleanup();
    gp_cleanup();

    if (err) {
        fprintf(stderr, "Stopped: %s. Log closed: %s\n", err, fname);
        return 1;
    }
    printf("Exit. Log closed: %s\n", fname);
    return 0;
}
