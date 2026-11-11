/*
 * Lab 07: self-balancing robot, PID on pitch, speed command from the joysticks.
 *
 * Gamepad:  Left stick up/down     forward / reverse speed command (ramps to the stick value)
 *           Right stick left/right steer (neutral when released)
 *           B  stop: zero the speed and steering until both sticks are centred again
 *           START  exit
 * Gains: defaults are Kp/Ki/Kd in robot_params.h. For tuning, override them without rebuilding:
 *   sudo ./lab07 --kp 0.015 --ki 0 --kd 0       P only
 *   sudo ./lab07 --kp 0.015 --ki 0 --kd 0.005   P + D
 *   sudo ./lab07 --kp 0.015 --ki 0.005 --kd 0.005   P + D + I (the defaults)
 * The gains used are printed and put in the CSV file name.
 *           (check the mapping with no motors: make joystick_test && sudo ./joystick_test)
 * Gains, limits and safety values are in balance_ctrl.h (TUNING BLOCK).
 *
 * Pitch zero: the midpoint of the IMU readings at the two block stops (Lab 05 rule). Run
 *   sudo ./lab07 --cal    to measure it: hold the robot on the forward stop and press X, then
 *                         hold it on the back stop and press B. The zero is saved in pitch_zero.txt next to the lab07 program
 *                         and used by every later run. Without a file, balance_ctrl.h defaults are used.
 *
 *   sudo ./lab07 --cal-balance   with NO stopper: hold the robot balanced (or let it balance), press X; it
 *                         averages 3 s of pitch and saves that as the zero. No stops are saved, so the
 *                         block-rest trip is off.
 *   sudo ./lab07 --zero 4.2   type the IMU pitch (raw frame) to use as zero; also saved without stops.
 * The IMU pitch is measured against gravity (BNO055 NDOF fusion), so a saved zero stays valid between
 * runs; redo it only if the IMU is remounted or the weight moves.
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
#include <unistd.h>
#include <linux/input.h>

#include "balance_ctrl.h"
#include "encoder.h"
#include "gamepad.h"
#include "imu.h"
#include "pwm.h"

#define LOOP_NS        ((long)(ROBOT_LOOP_PERIOD_S * 1e9 + 0.5))
#define LOG_MS         ROBOT_LOG_PERIOD_MS
#define CONNECT_WAIT_MS 10000
#define LEFT_CH        ROBOT_LEFT_CH
#define RIGHT_CH       ROBOT_RIGHT_CH
#define FEEDBACK_ENC   ROBOT_FEEDBACK_ENC
#define PITCH_ZERO_DEG ROBOT_PITCH_RAW_UPRIGHT_DEG
#define RATE_WINDOW_STEPS ROBOT_RATE_WINDOW_STEPS
#define RATE_FILTER    ROBOT_RATE_FILTER

#define BTN_ZERO_SPEED BTN_SOUTH  /* B */
#define BTN_EXIT       BTN_START
#define BTN_CAL_FWD    BTN_WEST   /* X on the Lab 06 pad (calibration only) */
#define BTN_CAL_FWD_ALT BTN_NORTH /* X on pads that report X and Y the other way round (ZhiXu pad, 11/11) */
#define BTN_CAL_BACK   BTN_SOUTH  /* B: record the back stop (calibration only) */
#define ZERO_NAME      "pitch_zero.txt"   /* stored next to the lab07 program, wherever it is run from */
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

static char zero_path[512] = ZERO_NAME;

static void find_zero_path(void)
{
    char exe[400];
    ssize_t n = readlink("/proc/self/exe", exe, sizeof exe - 1);
    if (n <= 0) return;
    exe[n] = '\0';
    char *slash = strrchr(exe, '/');
    if (!slash) return;
    *slash = '\0';
    snprintf(zero_path, sizeof zero_path, "%s/%s", exe, ZERO_NAME);
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

static int btn_x(void) { return gp_get_button(BTN_CAL_FWD) || gp_get_button(BTN_CAL_FWD_ALT); }

static int wait_button(int code)
{
    while (running && gp_is_connected() && !(code == BTN_CAL_FWD ? btn_x() : gp_get_button(code))) {
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    while (running && (code == BTN_CAL_FWD ? btn_x() : gp_get_button(code))) {          /* wait for release */
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
    FILE *z = fopen(zero_path, "w");
    if (z) { fprintf(z, "%.4f %.4f %.4f\n", zero, f, b); fclose(z); printf("Saved to %s\n", zero_path); }
    else perror(zero_path);
    return zero;
}

/* File holds: zero, forward stop raw, back stop raw. stops[] gets the stops measured from the zero. */
/* Balance-point calibration (no stopper): average 3 s of raw pitch while the robot is held balanced. */
static double calibrate_balance(void)
{
    printf("Hold the robot balanced (or let it balance), keep it still, press X.\n");
    if (!wait_button(BTN_CAL_FWD)) return NAN;
    double sum = 0.0, lo = 1e9, hi = -1e9;
    for (int i = 0; i < 3 * CAL_SAMPLES && running; i++) {
        double p = raw_pitch();
        sum += p; if (p < lo) lo = p; if (p > hi) hi = p;
        struct timespec d = { 0, 10000000L };
        nanosleep(&d, NULL);
    }
    double zero = sum / (3 * CAL_SAMPLES);
    printf("  balance point %+.3f deg (moved %.2f deg)\n", zero, hi - lo);
    if (hi - lo > 2.0 * CAL_MAX_SPREAD) { printf("  moved too much, not saved. Try again.\n"); return NAN; }
    return zero;
}

static void save_zero_only(double zero)
{
    FILE *z = fopen(zero_path, "w");
    if (z) { fprintf(z, "%.4f nan nan\n", zero); fclose(z); printf("Saved zero %+.3f deg to %s (no stops)\n", zero, zero_path); }
    else perror(zero_path);
}

static double load_zero(double stops[2])
{
    double zero, f, b;
    stops[0] = STOP_FWD_RAW_DEG  - PITCH_ZERO_DEFAULT_DEG;
    stops[1] = STOP_BACK_RAW_DEG - PITCH_ZERO_DEFAULT_DEG;
    FILE *z = fopen(zero_path, "r");
    if (z && fscanf(z, "%lf %lf %lf", &zero, &f, &b) == 3 && isfinite(zero)) {
        fclose(z);
        stops[0] = f - zero;      /* NAN when the file has no stops */
        stops[1] = b - zero;
        if (isfinite(f) && isfinite(b))
            printf("Pitch zero %+.3f deg, stops %+.2f / %+.2f (from %s)\n", zero, stops[0], stops[1], zero_path);
        else
            printf("Pitch zero %+.3f deg, no stops saved: block-rest trip off (from %s)\n", zero, zero_path);
        return zero;
    }
    if (z) fclose(z);
    printf("Pitch zero %+.3f deg (Lab 05 default; run with --cal to measure)\n", PITCH_ZERO_DEFAULT_DEG);
    return PITCH_ZERO_DEFAULT_DEG;
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
    int cal = 0, cal_bal = 0, have_zero = 0;
    double kp = KP, ki = KI, kd = KD;
    double typed_zero = 0.0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--cal") == 0) cal = 1;
        else if (strcmp(argv[i], "--cal-balance") == 0) cal_bal = 1;
        else if (strcmp(argv[i], "--kp") == 0 && i + 1 < argc) kp = atof(argv[++i]);
        else if (strcmp(argv[i], "--ki") == 0 && i + 1 < argc) ki = atof(argv[++i]);
        else if (strcmp(argv[i], "--kd") == 0 && i + 1 < argc) kd = atof(argv[++i]);
        else if (strcmp(argv[i], "--zero") == 0 && i + 1 < argc) { typed_zero = atof(argv[++i]); have_zero = 1; }
        else dev = argv[i];
    }
    const char *err = NULL;
    find_zero_path();

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
    if (have_zero) {
        save_zero_only(typed_zero);
        shutdown_hw();
        return 0;
    }
    if (cal_bal) {
        pitch_zero = calibrate_balance();
        if (isfinite(pitch_zero)) save_zero_only(pitch_zero);
        shutdown_hw();
        return isfinite(pitch_zero) ? 0 : 1;
    }
    if (cal) {
        pitch_zero = calibrate_zero();
        shutdown_hw();
        if (!isfinite(pitch_zero)) { fprintf(stderr, "calibration cancelled\n"); return 1; }
        return 0;                       /* calibration only: run again without --cal to balance */
    } else {
        pitch_zero = load_zero(stops);
    }

    char fname[128];
    time_t wall = time(NULL);
    char stamp[32];
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", localtime(&wall));
    snprintf(fname, sizeof fname, "lab07_%s_kp%g_ki%g_kd%g.csv", stamp, kp, ki, kd);
    FILE *log = fopen(fname, "w");
    if (!log) { perror(fname); shutdown_hw(); return 1; }
    fprintf(log, "time_ms,target_speed,measured_speed,pitch_deg,pitch_target_deg,error_deg,pid_output,left_pwm,right_pwm\n");

    printf("Logging to %s\nHold the robot near %.1f deg; motors go live after %d ms inside +/-%.1f deg.\n"
           "Left stick speed, right stick steer, B stop, START exit.\n",
           fname, THETA0_DEG, ARM_HOLD_MS, ARM_BAND_DEG);

    ctrl_t c;
    ctrl_init(&c);
    ctrl_set_stops(&c, stops[0], stops[1]);
    c.kp = kp; c.ki = ki; c.kd = kd;
    printf("Gains: Kp %g, Ki %g, Kd %g\n", kp, ki, kd);
    int prev_b = 0, was_live = 0, stopped = 0;

    double pitch = raw_pitch() - pitch_zero;
    double pitch_at_window = pitch, rate = 0.0;
    int window_count = 0;
    long next_status_ms = 0;

    struct timespec start, next;
    clock_gettime(CLOCK_MONOTONIC, &start);
    next = start;
    long next_log_ms = elapsed_ms(&start) + LOG_MS;

    while (running) {
        /* 1. Gamepad. */
        if (!gp_is_connected()) { err = "controller disconnected"; break; }
        if (gp_get_button(BTN_EXIT)) break;

        double ly = gp_get_axis(GP_AXIS_LY);
        double rx = gp_get_axis(GP_AXIS_RX);
        int b = gp_get_button(BTN_ZERO_SPEED);
        if (b && !prev_b) { stopped = 1; ctrl_zero_speed(&c); printf("STOP (centre both sticks to resume)\n"); }
        prev_b = b;
        if (stopped && ly == 0.0 && rx == 0.0 && !b) { stopped = 0; printf("Resumed\n"); }
        ctrl_set_speed(&c, stopped ? 0.0 : ctrl_stick_speed(ly), LOOP_MS_DT);
        double steer = stopped ? 0.0 : ctrl_stick_steer(rx);

        /* 2. Measurements. get_imu_pitch() returns 0.0 on a failed read, which becomes a pitch of about
         * 175 deg here, so a failed read trips the cutoff and cuts the motors. (A pitch that repeats
         * exactly is normal at rest: the IMU resolution is 1/16 deg.) */
        double raw = get_imu_pitch();
        pitch = wrap180(PITCH_ZERO_DEG - raw) - pitch_zero;   /* deg from the calibrated zero */
        if (++window_count >= RATE_WINDOW_STEPS) {
            double r = (pitch - pitch_at_window) / (RATE_WINDOW_STEPS * LOOP_MS_DT);
            rate += RATE_FILTER * (r - rate);
            pitch_at_window = pitch;
            window_count = 0;
        }
        double speed = enc_get_speed(FEEDBACK_ENC);

        /* 3. Controller, then write exactly what is logged. */
        ctrl_in_t in = { pitch, rate, speed, steer };
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
        if (t_ms >= next_status_ms) {
            printf("t %4.0f s  pitch %+6.2f deg (target %+.1f)  speed %+7.0f edges/s  %s\n", t_ms / 1000.0, pitch,
                   out.pitch_target, speed, live ? "LIVE" : (c.armed ? "armed" : "waiting: hold the robot near the target"));
            next_status_ms += 1000;
        }
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
