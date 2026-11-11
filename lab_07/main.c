/*
 * Lab 07: self-balancing robot, PID on pitch, speed command from the d-pad.
 *
 * Gamepad:  D-pad Up/Down  raise forward / reverse speed command one step
 *           D-pad Left/Right  steer (neutral when released)
 *           B  zero the speed command      START  exit
 * Gains, limits and safety values are in balance_ctrl.h (TUNING BLOCK).
 *
 * Pitch zero: the midpoint of the IMU readings at the two block stops (Lab 05 rule). Run
 *   sudo ./lab07 --cal    to measure it: hold the robot on the forward stop and press X, then
 *                         hold it on the back stop and press B. The zero is saved in pitch_zero.txt
 *                         and used by every later run. Without a file, balance_ctrl.h defaults are used.
 *
 * Robot must be supported above the floor for the sensor and polarity checks.
 * Run with sudo (sysfs PWM and GPIO). Writes lab07_YYYYmmdd_HHMMSS.csv here.
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
#include "encoder.h"
#include "gamepad.h"
#include "imu.h"
#include "pwm.h"

#define LOOP_NS        1000000L   /* 1 ms control loop */
#define LOG_MS         20         /* one CSV row every 20 ms */
#define CONNECT_WAIT_MS 10000
#define LEFT_CH        0
#define RIGHT_CH       1
#define FEEDBACK_ENC   0          /* encoder used for the speed estimate */
#define STEER_SIGN     1.0        /* +1: d-pad left turns left; flip if reversed */
#define PITCH_ZERO_DEG 180.0      /* raw BNO055 reading when upright (Lab 05) */
#define RATE_WINDOW_STEPS 10      /* pitch rate over 10 ms (the IMU updates about every 10 ms) */
#define RATE_FILTER    0.3
#define STALE_MS       200        /* IMU value exactly unchanged this long = read failure */

#define BTN_ZERO_SPEED BTN_SOUTH  /* B */
#define BTN_EXIT       BTN_START
#define BTN_CAL_FWD    BTN_WEST   /* X: record the forward stop (calibration only) */
#define BTN_CAL_BACK   BTN_SOUTH  /* B: record the back stop (calibration only) */
#define ZERO_FILE      "pitch_zero.txt"
#define CAL_SAMPLES    100        /* 1 s of 10 ms samples */
#define CAL_MAX_SPREAD 1.0        /* deg: robot must be held still or the average means nothing */

static volatile sig_atomic_t running = 1;
static void on_signal(int sig) { (void)sig; running = 0; }

static void tick(struct timespec *next)
{
    next->tv_nsec += LOOP_NS;
    if (next->tv_nsec >= 1000000000L) { next->tv_nsec -= 1000000000L; next->tv_sec++; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
}

static long elapsed_ms(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

static double wrap180(double a)
{
    if (!isfinite(a)) return 180.0;
    while (a > 180.0) a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

static double raw_pitch(void)
{
    return wrap180(PITCH_ZERO_DEG - get_imu_pitch());   /* deg, before the zero is subtracted */
}

/* Average the pitch for 1 s while the robot is held on a stop. Returns 0 and the mean, or -1 if it moved. */
static int sample_stop(double *mean)
{
    double sum = 0.0, lo = 1e9, hi = -1e9;
    for (int i = 0; i < CAL_SAMPLES && running; i++) {
        double p = raw_pitch();
        sum += p; if (p < lo) lo = p; if (p > hi) hi = p;
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    *mean = sum / CAL_SAMPLES;
    printf("  read %+.2f deg (moved %.2f deg)\n", *mean, hi - lo);
    return (hi - lo) <= CAL_MAX_SPREAD ? 0 : -1;
}

static int wait_button(int code)
{
    while (running && gp_is_connected() && !gp_get_button(code)) {
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    while (running && gp_get_button(code)) {          /* wait for release */
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    return running && gp_is_connected();
}

/* Move to both ends and take the average (Josue's Lab 05 method). Returns the zero, or NAN. */
static double calibrate_zero(void)
{
    double f, b;
    do {
        printf("Hold the robot on the FORWARD stop, keep it still, press X.\n");
        if (!wait_button(BTN_CAL_FWD)) return NAN;
    } while (sample_stop(&f) != 0 && running && printf("  moved too much, try again\n"));
    do {
        printf("Hold the robot on the BACK stop, keep it still, press B.\n");
        if (!wait_button(BTN_CAL_BACK)) return NAN;
    } while (sample_stop(&b) != 0 && running && printf("  moved too much, try again\n"));
    double zero = 0.5 * (f + b);
    printf("Zero = midpoint %+.3f deg. Stops are at %+.3f (forward) and %+.3f (back) from the zero.\n",
           zero, f - zero, b - zero);
    FILE *z = fopen(ZERO_FILE, "w");
    if (z) { fprintf(z, "%.4f %.4f %.4f\n", zero, f, b); fclose(z); printf("Saved to %s\n", ZERO_FILE); }
    else perror(ZERO_FILE);
    return zero;
}

/* File holds: zero, forward stop raw, back stop raw. stops[] gets the stops measured from the zero. */
static double load_zero(double stops[2])
{
    double zero, f, b;
    stops[0] = STOP_FWD_RAW_DEG  - PITCH_ZERO_DEFAULT_DEG;
    stops[1] = STOP_BACK_RAW_DEG - PITCH_ZERO_DEFAULT_DEG;
    FILE *z = fopen(ZERO_FILE, "r");
    if (z && fscanf(z, "%lf %lf %lf", &zero, &f, &b) == 3 && isfinite(zero) && isfinite(f) && isfinite(b)) {
        fclose(z);
        stops[0] = f - zero;
        stops[1] = b - zero;
        printf("Pitch zero %+.3f deg, stops %+.2f / %+.2f (from %s)\n", zero, stops[0], stops[1], ZERO_FILE);
        return zero;
    }
    if (z) fclose(z);
    printf("Pitch zero %+.3f deg (Lab 05 default; run with --cal to measure)\n", PITCH_ZERO_DEFAULT_DEG);
    return PITCH_ZERO_DEFAULT_DEG;
}

static void read_dpad(int *up, int *down, int *left, int *right)
{
    double hx = gp_get_axis(GP_AXIS_HAT_X), hy = gp_get_axis(GP_AXIS_HAT_Y);
    *up    = hy < 0.0 || gp_get_button(BTN_DPAD_UP);
    *down  = hy > 0.0 || gp_get_button(BTN_DPAD_DOWN);
    *left  = hx < 0.0 || gp_get_button(BTN_DPAD_LEFT);
    *right = hx > 0.0 || gp_get_button(BTN_DPAD_RIGHT);
}

static void motors_off(void)
{
    pwm_set_duty(LEFT_CH, 0.0f);
    pwm_set_duty(RIGHT_CH, 0.0f);
}

static void shutdown_hw(void)
{
    motors_off();
    pwm_cleanup();
    enc_cleanup();
    imu_cleanup();
    gp_cleanup();
}

int main(int argc, char **argv)
{
    const char *dev = NULL;
    int cal = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--cal") == 0) cal = 1;
        else dev = argv[i];
    }
    const char *err = NULL;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (gp_init(dev) != 0) { fprintf(stderr, "gamepad init failed\n"); return 1; }
    printf("Waiting for controller...\n");
    for (int ms = 0; ms < CONNECT_WAIT_MS && running && !gp_is_connected(); ms += 10) {
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    if (!gp_is_connected()) {
        fprintf(stderr, "no controller found\n");
        gp_cleanup();
        return 1;
    }

    if (pwm_init() != 0 || enc_init() != 0 || imu_init() != 0) {
        fprintf(stderr, "init failed (need sudo for sysfs/GPIO/I2C?)\n");
        gp_cleanup();
        return 1;
    }
    motors_off();
    double pitch_zero = PITCH_ZERO_DEFAULT_DEG;
    double stops[2] = { 0, 0 };
    if (cal) {
        pitch_zero = calibrate_zero();
        shutdown_hw();
        if (!isfinite(pitch_zero)) { fprintf(stderr, "calibration cancelled\n"); return 1; }
        return 0;                       /* calibration only: run again without --cal to balance */
    } else {
        pitch_zero = load_zero(stops);
    }

    char fname[64];
    time_t wall = time(NULL);
    strftime(fname, sizeof fname, "lab07_%Y%m%d_%H%M%S.csv", localtime(&wall));
    FILE *log = fopen(fname, "w");
    if (!log) { perror(fname); shutdown_hw(); return 1; }
    fprintf(log, "time_ms,target_speed,measured_speed,pitch_deg,pitch_target_deg,error_deg,pid_output,left_pwm,right_pwm\n");

    printf("Logging to %s\nHold the robot near %.1f deg; motors go live after %d ms inside +/-%.1f deg.\n"
           "D-pad Up/Down speed, Left/Right steer, B zero speed, START exit.\n",
           fname, THETA0_DEG, ARM_HOLD_MS, ARM_BAND_DEG);

    ctrl_t c;
    ctrl_init(&c);
    ctrl_set_stops(&c, stops[0], stops[1]);
    int prev_up = 0, prev_down = 0, prev_b = 0, was_live = 0;

    double pitch = raw_pitch() - pitch_zero;
    double pitch_at_window = pitch, rate = 0.0, last_raw = 1e9;
    int window_count = 0, same_ms = 0;

    struct timespec start, next;
    clock_gettime(CLOCK_MONOTONIC, &start);
    next = start;
    long next_log_ms = elapsed_ms(&start) + LOG_MS;

    while (running) {
        /* 1. Gamepad. */
        if (!gp_is_connected()) { err = "controller disconnected"; break; }
        if (gp_get_button(BTN_EXIT)) break;

        int up, down, left, right;
        read_dpad(&up, &down, &left, &right);
        if (up && !prev_up)     ctrl_step_speed(&c, +1);
        if (down && !prev_down) ctrl_step_speed(&c, -1);
        prev_up = up; prev_down = down;
        int b = gp_get_button(BTN_ZERO_SPEED);
        if (b && !prev_b) ctrl_zero_speed(&c);
        prev_b = b;

        /* 2. Measurements. get_imu_pitch() returns 0.0 on a failed read, so a value that
         * never changes is treated as a failure. */
        double raw = get_imu_pitch();
        if (raw == last_raw) same_ms++; else same_ms = 0;
        last_raw = raw;
        if (same_ms > STALE_MS) { err = "IMU value frozen (read failure)"; break; }
        pitch = wrap180(PITCH_ZERO_DEG - raw) - pitch_zero;   /* deg from the calibrated zero */
        if (++window_count >= RATE_WINDOW_STEPS) {
            double r = (pitch - pitch_at_window) / (RATE_WINDOW_STEPS * LOOP_MS_DT);
            rate += RATE_FILTER * (r - rate);
            pitch_at_window = pitch;
            window_count = 0;
        }
        double speed = enc_get_speed(FEEDBACK_ENC);

        /* 3. Controller, then write exactly what is logged. */
        ctrl_in_t in = { pitch, rate, speed, STEER_SIGN * (left - right) };
        ctrl_out_t out;
        int live = ctrl_update(&c, &in, &out);
        if (live && !was_live) printf("Motors LIVE\n");
        was_live = live;
        if (c.tripped) { err = c.block_rest_ms >= BLOCK_REST_MS ? "resting on a block stop" : "cutoff exceeded"; motors_off(); }
        float l = (float)out.left, r = (float)out.right;
        if (pwm_set_duty(LEFT_CH, l) < 0 || pwm_set_duty(RIGHT_CH, r) < 0) {
            err = "pwm_set_duty failed";
            break;
        }

        /* 4. One row every 20 ms. */
        long t_ms = elapsed_ms(&start);
        if (t_ms >= next_log_ms) {
            fprintf(log, "%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t_ms, c.speed_cmd,
                    speed, pitch, out.pitch_target, out.error, out.pid_output, (double)l, (double)r);
            next_log_ms += LOG_MS;
            while (next_log_ms <= t_ms) next_log_ms += LOG_MS;
        }
        if (err) break;

        tick(&next);
    }

    fclose(log);
    shutdown_hw();

    if (err) { fprintf(stderr, "Stopped: %s. Log closed: %s\n", err, fname); return 1; }
    printf("Exit. Log closed: %s\n", fname);
    return 0;
}
