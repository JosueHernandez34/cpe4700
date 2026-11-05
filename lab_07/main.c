/*
 * main.c -- Lab 07: self-balancing robot with PID control and gamepad driving
 *
 * Usage:  sudo ./lab07 [options] [event_device]
 *         sudo ./lab07 --monitor            sensor check, motors never touched
 *
 *   event_device   for example /dev/input/event11. Leave it out to use the
 *                  first device that looks like a gamepad (Lab 06).
 *   --kp X --ki X --kd X   PID gains (defaults: Lab 05 setting C)
 *   --kv X         speed-to-angle gain, deg per (edge/s)
 *   --theta0 X     balance angle theta0, deg (default: the Lab 05 pitch zero)
 *   --vmax X       largest speed command, edges/s
 *   --steer X      steering duty u_steer at full d-pad
 *   --maxduty X    final clamp on both PWM commands (<= 1.0, the PWM range)
 *   --deadband X   duty added in the direction of each motor command
 *   --monitor      print pitch and encoder speed only (procedure 10.1)
 *
 * The gains are options so the P -> PD -> PID tuning steps of procedure 10.3
 * need no rebuild:   --ki 0 --kd 0   then   --ki 0   then all three.
 *
 * Writes lab07_YYYYMMDD_HHMMSS.csv (one row every 20 ms) and
 * lab07_YYYYMMDD_HHMMSS_info.txt (the gains and limits used for that run).
 *
 * ---------------------------------------------------------------------------
 * CONTROLLER MAPPING (Nintendo Switch Pro Controller, codes confirmed in Lab 06)
 *
 *   D-pad up      ABS_HAT0Y -1   ramp the forward speed command up (held)
 *   D-pad down    ABS_HAT0Y +1   ramp the reverse speed command up (held)
 *   D-pad left    ABS_HAT0X -1   steer left (held)
 *   D-pad right   ABS_HAT0X +1   steer right (held)
 *   X             BTN_WEST       arm: the motors start once the robot is held
 *                                within ARM_WINDOW_DEG of theta0
 *   B             BTN_SOUTH      disarm: motors off at once
 *   + (START)     BTN_START      exit
 *
 * With the d-pad released the speed command ramps back to 0, so the robot
 * balances in place with theta_target ~= theta0.
 * ---------------------------------------------------------------------------
 *
 * CONTROL (every 1 ms, the order of handout section 9)
 *   1. read the gamepad              (gamepad.c thread, never blocks)
 *   2. v_target: ramp toward +/-vmax while the d-pad is held, toward 0 when not
 *   3. v_meas  = average of both wheel counts, differenced over 50 ms
 *   4. theta_target = theta0 + Kv*(v_target - v_meas),
 *                     clamped to theta0 +/- TARGET_LIMIT_DEG
 *   5. theta   = wrap(180 - raw BNO055 pitch)   (Lab 05: 0 = sensor level,
 *                                                forward lean positive)
 *   6. e       = theta_target - theta
 *   7. u       = DRIVE_SIGN * (Kp*e + Ki*integral(e) + Kd*de/dt)
 *   8. u_steer = ramped STEER_SIGN * turn * steer
 *   9. uL = u - u_steer, uR = u + u_steer, clamped to +/-maxduty
 *  10. pwm_set_duty() on both channels
 *  11. log one row when time_ms >= next_log_ms
 *  12. sleep to the next absolute 1 ms tick
 *
 * SIGN OF THE PID OUTPUT. With e = theta_target - theta, a forward lean
 * (theta above target) gives e < 0. To catch it, the wheels must drive
 * forward (positive duty), so the PID sum is multiplied by DRIVE_SIGN = -1.
 * With positive gains this is the same law as Lab 05 (duty = Kp*(pitch -
 * target) + ...). If the wheels run away from the lean, flip DRIVE_SIGN.
 *
 * SAFETY (handout section 4)
 *   - motors start disarmed and only engage near upright
 *   - |theta - theta0| > CUTOFF_DEG          -> disarm (fell or lifted)
 *   - beyond REST_DEG for REST_MS            -> disarm (lying on a block)
 *   - IMU read error                         -> motors 0, exit
 *   - gamepad disconnect                     -> motors 0, exit
 *   - PWM write error, Ctrl-C, START         -> motors 0, exit
 * On every exit: motors 0, log closed, then PWM, encoder, IMU and gamepad
 * cleanup.
 *
 * Speeds are in encoder A-edges/s (the Lab 04/06 unit), angles in degrees,
 * motor commands in duty (-1..1).
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/input.h>

#include "pwm.h"
#include "encoder.h"
#include "imu.h"
#include "gamepad.h"

/* ---------------- Timing ---------------- */
#define LOOP_NS          1000000L   /* 1 ms control period (Labs 04-06) */
#define DT_S             0.001      /* s, same period */
#define LOG_MS           20         /* one CSV row every 20 ms */
#define CONNECT_WAIT_MS  10000      /* time to wait for the pad at start */
#define LATE_US          500        /* a tick this late counts as an overrun */

/* ---------------- Wheels ---------------- */
/* pwm.c already inverts channel 0 so +duty drives both wheels forward.
 * Lab 06 confirmed channel 0 = left, channel 1 = right. */
#define LEFT_CH          0
#define RIGHT_CH         1
/* Encoder count direction, +1 when forward motion counts up. Encoder 0 was
 * confirmed in Lab 06. Encoder 1 is not confirmed: check it with --monitor
 * (push the robot forward, both counts must go up) and flip if needed. */
#define ENC0_SIGN        1.0
#define ENC1_SIGN        1.0
/* +1: d-pad left turns left (confirmed in Lab 06). */
#define STEER_SIGN       1.0

/* ---------------- Pitch ---------------- */
/* The BNO055 reads about +/-180 deg upright (Lab 05), so theta = wrap(180 - raw). */
#define PITCH_RAW_UPRIGHT 180.0
/* theta0: the Lab 05 balance angle, the midpoint of the measured block stops
 * (11.25 + -3.56) / 2 = +3.845 deg (lab_05/balance_loop_v2.h). Check it with
 * --monitor holding the robot at its balance point. */
#define THETA0_DEFAULT_DEG 3.845
#define CUTOFF_DEG       15.0       /* |theta - theta0| beyond this: disarm */
#define REST_DEG         6.0        /* the blocks stop the body at about +/-7.4 deg */
#define REST_MS          1000       /* this long beyond REST_DEG: disarm */
#define ARM_WINDOW_DEG   2.0        /* armed robot engages once inside this */

/* ---------------- PID ---------------- */
/* Defaults: Lab 05 setting C, the only one with long holds (Kp 0.015,
 * Ki 0.005, Kd 0.005 with a 0.60 duty limit). */
#define KP_DEFAULT       0.015      /* duty per deg */
#define KI_DEFAULT       0.005      /* duty per (deg*s) */
#define KD_DEFAULT       0.005      /* duty per (deg/s) */
#define DRIVE_SIGN       (-1.0)     /* see "SIGN OF THE PID OUTPUT" above */
#define I_LIMIT_DUTY     0.30       /* anti-windup: |Ki*integral| never above this */
/* The BNO055 fusion output changes about every 10 ms, so de/dt between 1 ms
 * samples would be mostly 0 with spikes. It is taken over a 10 ms window and
 * low-pass filtered (same values as Lab 05). */
#define D_WINDOW_STEPS   10         /* 1 ms steps per derivative update */
#define D_FILTER         0.3        /* weight of each new derivative sample */

/* ---------------- Speed command and estimate ---------------- */
#define SPEED_WINDOW_MS  50         /* counts differenced over 50 ms: one count
                                     * on one wheel = 10 edges/s after averaging */
#define VMAX_DEFAULT     600.0      /* edges/s (Lab 06 slow mode was 800) */
#define V_RAMP           600.0      /* edges/s per s while the d-pad is held */
#define V_RELEASE        1200.0     /* edges/s per s back toward 0 on release */
#define KV_DEFAULT       0.003      /* deg per (edge/s): 600 edges/s error = 1.8 deg */
#define TARGET_LIMIT_DEG 3.0        /* theta_target stays within theta0 +/- this */

/* ---------------- Steering and output ---------------- */
#define STEER_DEFAULT    0.08       /* duty (Lab 06 slow mode) */
#define STEER_RAMP       0.5        /* duty per s */
#define MAXDUTY_DEFAULT  0.60       /* Lab 05 tuning limit */

#define BTN_ARM          BTN_WEST   /* X */
#define BTN_DISARM       BTN_SOUTH  /* B */
#define BTN_EXIT         BTN_START  /* + */

typedef struct {
    double kp, ki, kd, kv, theta0, vmax, steer, maxduty, deadband;
} params_t;

typedef enum { ST_DISARMED = 0, ST_WAITING = 1, ST_ACTIVE = 2 } state_t;
static const char *STATE_NAME[] = { "DISARMED", "ARMED (waiting for upright)", "BALANCING" };

static volatile sig_atomic_t running = 1;
static void on_signal(int sig) { (void)sig; running = 0; }

static double clamp(double x, double lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

/* Move x toward target by at most step. */
static double ramp(double x, double target, double step)
{
    if (x < target) return fmin(x + step, target);
    if (x > target) return fmax(x - step, target);
    return x;
}

/* Wrap into -180..+180. A non-finite value becomes 180, which trips the cutoff. */
static double wrap180(double a)
{
    if (!isfinite(a))
        return 180.0;
    a = fmod(a, 360.0);
    if (a >  180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
}

static long elapsed_ms(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L
         + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

/* Sleep until the next absolute 1 ms tick (no cumulative drift). Returns how
 * late the wake-up was, in microseconds (0 if the tick was already past). */
static long tick(struct timespec *next)
{
    next->tv_nsec += LOOP_NS;
    if (next->tv_nsec >= 1000000000L) {
        next->tv_nsec -= 1000000000L;
        next->tv_sec++;
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long late_ns = (now.tv_sec - next->tv_sec) * 1000000000L + (now.tv_nsec - next->tv_nsec);
    if (late_ns < 0)
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
    return late_ns > 0 ? late_ns / 1000 : 0;
}

/* Robot-frame pitch in degrees (Lab 05 convention). Returns -1 on a failed read. */
static int read_pitch(double *theta)
{
    double raw;
    if (get_imu_pitch_checked(&raw) != 0)
        return -1;
    *theta = wrap180(PITCH_RAW_UPRIGHT - raw);
    return 0;
}

/* Average signed wheel count, edges (forward positive). */
static double wheel_count(void)
{
    return 0.5 * (ENC0_SIGN * enc_get_count(0) + ENC1_SIGN * enc_get_count(1));
}

/* D-pad as booleans; hat axis and d-pad buttons combined (Lab 06). */
static void read_dpad(int *up, int *down, int *left, int *right)
{
    double hx = gp_get_axis(GP_AXIS_HAT_X);
    double hy = gp_get_axis(GP_AXIS_HAT_Y);

    *up    = hy < 0.0 || gp_get_button(BTN_DPAD_UP);
    *down  = hy > 0.0 || gp_get_button(BTN_DPAD_DOWN);
    *left  = hx < 0.0 || gp_get_button(BTN_DPAD_LEFT);
    *right = hx > 0.0 || gp_get_button(BTN_DPAD_RIGHT);
}

/* ======================================================================
 * --monitor: procedure 10.1. Motors are never initialised.
 * Prints pitch, pitch - theta0, the mean and spread over the last 2 s (hold
 * the robot at its balance point to read theta0), both encoder counts and the
 * vehicle speed estimate used by the controller.
 * ====================================================================== */
#define MON_PERIOD_MS 10
#define MON_HIST      200           /* 2 s of 10 ms samples */

static int run_monitor(const params_t *p)
{
    if (imu_init() != 0) {
        fprintf(stderr, "IMU init failed (need sudo for I2C?)\n");
        return 1;
    }
    if (enc_init() != 0) {
        fprintf(stderr, "encoder init failed (need sudo for GPIO?)\n");
        imu_cleanup();
        return 1;
    }
    printf("Monitor: motors are NOT used. Ctrl-C to quit.\n"
           "Tip forward: pitch must go UP. Push forward: both counts and v must go UP.\n"
           "theta0 now %.3f deg. Hold the robot at its balance point and read 'mean 2s'.\n\n",
           p->theta0);

    double hist[MON_HIST];
    int n_hist = 0, i_hist = 0;
    double cnt_hist[SPEED_WINDOW_MS / MON_PERIOD_MS + 1];
    const int n_cnt = SPEED_WINDOW_MS / MON_PERIOD_MS + 1;
    for (int i = 0; i < n_cnt; i++)
        cnt_hist[i] = wheel_count();
    int i_cnt = 0, err = 0;

    struct timespec d = { 0, MON_PERIOD_MS * 1000000L };
    for (unsigned int n = 0; running; n++) {
        double theta;
        if (read_pitch(&theta) != 0) {
            fprintf(stderr, "\nIMU read failed\n");
            err = 1;
            break;
        }
        hist[i_hist] = theta;
        i_hist = (i_hist + 1) % MON_HIST;
        if (n_hist < MON_HIST) n_hist++;

        double c_now = wheel_count();
        double c_old = cnt_hist[i_cnt];          /* oldest entry = 50 ms ago */
        cnt_hist[i_cnt] = c_now;
        i_cnt = (i_cnt + 1) % n_cnt;
        double v = (c_now - c_old) / ((n_cnt - 1) * MON_PERIOD_MS * 1e-3);

        if (n % 20 == 0) {                        /* print every 200 ms */
            double m = 0.0, s = 0.0;
            for (int i = 0; i < n_hist; i++) m += hist[i];
            m /= n_hist;
            for (int i = 0; i < n_hist; i++) s += (hist[i] - m) * (hist[i] - m);
            s = sqrt(s / n_hist);
            printf("pitch %+7.2f  -theta0 %+6.2f  mean 2s %+7.3f (std %.2f)  "
                   "cnt0 %7d  cnt1 %7d  spd0 %+7.0f  spd1 %+7.0f  v %+7.0f edges/s\n",
                   theta, theta - p->theta0, m, s, enc_get_count(0), enc_get_count(1),
                   enc_get_speed(0), enc_get_speed(1), v);
            fflush(stdout);
        }
        nanosleep(&d, NULL);
    }
    enc_cleanup();
    imu_cleanup();
    return err;
}

/* ======================================================================
 * Options
 * ====================================================================== */
static int parse_args(int argc, char **argv, params_t *p, const char **dev, int *monitor)
{
    struct { const char *name; double *val; } opt[] = {
        { "--kp", &p->kp }, { "--ki", &p->ki }, { "--kd", &p->kd }, { "--kv", &p->kv },
        { "--theta0", &p->theta0 }, { "--vmax", &p->vmax }, { "--steer", &p->steer },
        { "--maxduty", &p->maxduty }, { "--deadband", &p->deadband },
    };
    const int n_opt = (int)(sizeof opt / sizeof opt[0]);

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--monitor") == 0) {
            *monitor = 1;
            continue;
        }
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
            return -1;
        int k;
        for (k = 0; k < n_opt; k++)
            if (strcmp(argv[i], opt[k].name) == 0)
                break;
        if (k < n_opt) {
            char *end;
            if (i + 1 >= argc) {
                fprintf(stderr, "%s needs a value\n", argv[i]);
                return -1;
            }
            *opt[k].val = strtod(argv[++i], &end);
            if (*end != '\0' || !isfinite(*opt[k].val)) {
                fprintf(stderr, "bad value for %s: %s\n", opt[k].name, argv[i]);
                return -1;
            }
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "unknown option %s\n", argv[i]);
            return -1;
        } else {
            *dev = argv[i];
        }
    }
    if (p->maxduty <= 0.0 || p->maxduty > 1.0) {
        fprintf(stderr, "--maxduty must be in (0, 1], the PWM module's range\n");
        return -1;
    }
    if (p->vmax < 0.0 || p->steer < 0.0 || p->deadband < 0.0 || p->deadband > 0.2) {
        fprintf(stderr, "--vmax and --steer must be >= 0, --deadband 0..0.2\n");
        return -1;
    }
    return 0;
}

static void write_info(const char *path, const char *csv, const params_t *p)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        perror(path);
        return;
    }
    fprintf(f, "log=%s\nkp=%.6f\nki=%.6f\nkd=%.6f\nkv=%.6f\ntheta0_deg=%.3f\n"
               "vmax=%.1f\nsteer=%.3f\nmaxduty=%.3f\ndeadband=%.3f\n"
               "drive_sign=%.0f\ntarget_limit_deg=%.1f\ncutoff_deg=%.1f\n"
               "rest_deg=%.1f\nrest_ms=%d\ni_limit_duty=%.2f\nspeed_window_ms=%d\n"
               "v_ramp=%.0f\nv_release=%.0f\nloop_ms=1\nlog_ms=%d\n",
            csv, p->kp, p->ki, p->kd, p->kv, p->theta0, p->vmax, p->steer, p->maxduty,
            p->deadband, DRIVE_SIGN, TARGET_LIMIT_DEG, CUTOFF_DEG, REST_DEG, REST_MS,
            I_LIMIT_DUTY, SPEED_WINDOW_MS, V_RAMP, V_RELEASE, LOG_MS);
    fclose(f);
}

/* ======================================================================
 * Balance and drive
 * ====================================================================== */
int main(int argc, char **argv)
{
    params_t p = { KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, KV_DEFAULT, THETA0_DEFAULT_DEG,
                   VMAX_DEFAULT, STEER_DEFAULT, MAXDUTY_DEFAULT, 0.0 };
    const char *dev = NULL;
    int monitor = 0;

    if (parse_args(argc, argv, &p, &dev, &monitor) != 0) {
        fprintf(stderr, "usage: %s [--monitor] [--kp X] [--ki X] [--kd X] [--kv X] "
                        "[--theta0 X] [--vmax X] [--steer X] [--maxduty X] [--deadband X] "
                        "[event_device]\n", argv[0]);
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (monitor)
        return run_monitor(&p);

    /* ---- Initialisation: controller and IMU before the motors, so a missing
     * device never leaves the motors set up. ---- */
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

    double theta;
    if (imu_init() != 0 || read_pitch(&theta) != 0) {
        fprintf(stderr, "IMU init or first read failed (need sudo for I2C?)\n");
        imu_cleanup();
        gp_cleanup();
        return 1;
    }

    char fname[64], iname[80];
    time_t wall = time(NULL);
    strftime(fname, sizeof fname, "lab07_%Y%m%d_%H%M%S.csv", localtime(&wall));
    strftime(iname, sizeof iname, "lab07_%Y%m%d_%H%M%S_info.txt", localtime(&wall));
    FILE *log = fopen(fname, "w");
    if (!log) {
        perror(fname);
        imu_cleanup();
        gp_cleanup();
        return 1;
    }
    fprintf(log, "time_ms,target_speed,measured_speed,pitch_deg,pitch_target_deg,error_deg,"
                 "pid_output,left_pwm,right_pwm,u_steer,p_term,i_term,d_term,state\n");
    write_info(iname, fname, &p);

    if (pwm_init() != 0 || enc_init() != 0) {
        fprintf(stderr, "PWM/encoder init failed (need sudo for sysfs/GPIO?)\n");
        pwm_cleanup();
        fclose(log);
        imu_cleanup();
        gp_cleanup();
        return 1;
    }

    printf("Logging to %s (settings in %s)\n"
           "Kp %.4f  Ki %.4f  Kd %.4f  Kv %.4f deg/(edge/s)  theta0 %.3f deg  "
           "vmax %.0f  steer %.3f  maxduty %.2f\n"
           "X arm, B disarm, D-pad drive/steer, START exit.  Pitch now %+.2f deg.\n",
           fname, iname, p.kp, p.ki, p.kd, p.kv, p.theta0, p.vmax, p.steer, p.maxduty, theta);

    /* ---- Controller state ---- */
    state_t state = ST_DISARMED;
    double v_target = 0.0, u_steer = 0.0;
    double integral = 0.0;                     /* deg*s */
    double e_win = 0.0, d_filt = 0.0;          /* derivative window start, filtered de/dt */
    int win_steps = 0, rest_ms = 0, prev_arm = 0, prev_disarm = 0;

    const int n_cnt = SPEED_WINDOW_MS + 1;     /* one count sample per 1 ms tick */
    double cnt_hist[SPEED_WINDOW_MS + 1];
    for (int i = 0; i < n_cnt; i++)
        cnt_hist[i] = wheel_count();
    int i_cnt = 0;

    const char *err = NULL;
    long late_max_us = 0, overruns = 0;
    unsigned long ticks = 0;

    struct timespec start, next;
    clock_gettime(CLOCK_MONOTONIC, &start);
    next = start;
    long next_log_ms = elapsed_ms(&start) + LOG_MS;
    long next_status_ms = 0;

    while (running) {
        /* 1. Gamepad. */
        if (!gp_is_connected()) {
            err = "controller disconnected or read error";
            break;
        }
        if (gp_get_button(BTN_EXIT))
            break;

        int up, down, left, right;
        read_dpad(&up, &down, &left, &right);
        int fwd  = up - down;                  /* +1 forward, 0 none or both */
        int turn = left - right;               /* +1 left,    0 none or both */

        int arm_btn = gp_get_button(BTN_ARM), disarm_btn = gp_get_button(BTN_DISARM);
        if (disarm_btn && !prev_disarm && state != ST_DISARMED) {
            state = ST_DISARMED;
            printf("B: disarmed, motors off\n");
        } else if (arm_btn && !prev_arm && state == ST_DISARMED && !disarm_btn) {
            state = ST_WAITING;
            printf("X: armed. Hold the robot within %.1f deg of theta0 to start.\n",
                   ARM_WINDOW_DEG);
        }
        prev_arm = arm_btn;
        prev_disarm = disarm_btn;

        /* 2. Speed command: ramp while held, back to 0 when released. Held at
         * 0 unless balancing, so nothing builds up while disarmed. */
        if (state != ST_ACTIVE)
            v_target = 0.0;
        else if (fwd != 0)
            v_target = ramp(v_target, fwd * p.vmax, V_RAMP * DT_S);
        else
            v_target = ramp(v_target, 0.0, V_RELEASE * DT_S);

        /* 3. Vehicle speed: both wheels' counts averaged, differenced over 50 ms. */
        double c_now = wheel_count();
        double c_old = cnt_hist[i_cnt];        /* oldest entry, SPEED_WINDOW_MS ago */
        cnt_hist[i_cnt] = c_now;
        i_cnt = (i_cnt + 1) % n_cnt;
        double v_meas = (c_now - c_old) / (SPEED_WINDOW_MS * DT_S);   /* edges/s */

        /* 4. Pitch target from the speed error, limited to a small lean. */
        double theta_target = p.theta0
            + clamp(p.kv * (v_target - v_meas), TARGET_LIMIT_DEG);

        /* 5. IMU. A failed read stops the motors at once. */
        if (read_pitch(&theta) != 0) {
            err = "IMU read failed";
            break;
        }

        /* Safety: cutoff, resting on a block, and engaging near upright. */
        double lean = theta - p.theta0;
        if (state == ST_WAITING && fabs(lean) < ARM_WINDOW_DEG) {
            state = ST_ACTIVE;
            integral = 0.0;
            rest_ms = 0;
            printf("Balancing (pitch %+.2f deg)\n", theta);
        } else if (state == ST_ACTIVE) {
            if (fabs(lean) > CUTOFF_DEG) {
                state = ST_DISARMED;
                printf("CUTOFF: pitch %+.2f deg is more than %.0f deg from theta0, motors off. "
                       "Press X to re-arm.\n", theta, CUTOFF_DEG);
            } else if (fabs(lean) > REST_DEG) {
                if (++rest_ms >= REST_MS) {
                    state = ST_DISARMED;
                    printf("REST: leaning over %.0f deg for %d ms (on a block), motors off. "
                           "Press X to re-arm.\n", REST_DEG, REST_MS);
                }
            } else {
                rest_ms = 0;
            }
        }
        int active = (state == ST_ACTIVE);

        /* 6. Error. */
        double e = theta_target - theta;

        /* 7. PID, each term explicit. The derivative is updated over a 10 ms
         * window (the IMU's update period) and low-pass filtered; it keeps
         * running while disarmed so it is settled when balancing starts. */
        if (++win_steps >= D_WINDOW_STEPS) {
            double de_dt = (e - e_win) / (D_WINDOW_STEPS * DT_S);    /* deg/s */
            d_filt += D_FILTER * (de_dt - d_filt);
            e_win = e;
            win_steps = 0;
        }
        if (active) {
            integral += e * DT_S;
            if (p.ki > 0.0)
                integral = clamp(integral, I_LIMIT_DUTY / p.ki);     /* anti-windup */
        } else {
            integral = 0.0;
        }
        double p_term = DRIVE_SIGN * p.kp * e;
        double i_term = DRIVE_SIGN * p.ki * integral;
        double d_term = DRIVE_SIGN * p.kd * d_filt;
        double u = active ? clamp(p_term + i_term + d_term, p.maxduty) : 0.0;

        /* 8. Steering correction, ramped so a press does not jolt the balance. */
        u_steer = ramp(u_steer, active ? STEER_SIGN * turn * p.steer : 0.0, STEER_RAMP * DT_S);
        if (!active)
            u_steer = 0.0;

        /* 9. Differential motor commands, optional deadband push, final clamp. */
        double uL = u - u_steer, uR = u + u_steer;
        if (active && p.deadband > 0.0) {
            if (fabs(uL) > 1e-3) uL += copysign(p.deadband, uL);
            if (fabs(uR) > 1e-3) uR += copysign(p.deadband, uR);
        }
        float left_pwm  = (float)clamp(uL, p.maxduty);
        float right_pwm = (float)clamp(uR, p.maxduty);

        /* 10. Outputs. */
        if (pwm_set_duty(LEFT_CH, left_pwm) < 0 || pwm_set_duty(RIGHT_CH, right_pwm) < 0) {
            err = "pwm_set_duty failed";
            break;
        }

        /* 11. One row every 20 ms, from this iteration's values. */
        long t_ms = elapsed_ms(&start);
        if (t_ms >= next_log_ms) {
            fprintf(log, "%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%d\n",
                    t_ms, v_target, v_meas, theta, theta_target, e, u,
                    left_pwm, right_pwm, u_steer, p_term, i_term, d_term, (int)state);
            next_log_ms += LOG_MS;
            while (next_log_ms <= t_ms)        /* skip a slot rather than duplicate it */
                next_log_ms += LOG_MS;
        }

        /* Waiting to engage: show how far from upright, twice a second. */
        if (state == ST_WAITING && t_ms >= next_status_ms) {
            printf("  pitch %+6.2f deg (theta0 %+.2f, engage within +/-%.1f)\n",
                   theta, p.theta0, ARM_WINDOW_DEG);
            fflush(stdout);
            next_status_ms = t_ms + 500;
        }

        /* 12. Next 1 ms tick. */
        long late = tick(&next);
        ticks++;
        if (late > late_max_us) late_max_us = late;
        if (late > LATE_US) overruns++;
    }

    /* ---- Shutdown: motors off first, then the log, then every module. ---- */
    pwm_set_duty(LEFT_CH, 0.0f);
    pwm_set_duty(RIGHT_CH, 0.0f);
    fclose(log);
    pwm_cleanup();
    enc_cleanup();
    imu_cleanup();
    gp_cleanup();

    printf("Loop: %lu ticks, %ld over %d us late, worst %ld us. State at exit: %s\n",
           ticks, overruns, LATE_US, late_max_us, STATE_NAME[state]);
    if (err) {
        fprintf(stderr, "Stopped: %s. Motors off. Log closed: %s\n", err, fname);
        return 1;
    }
    printf("Exit. Motors off. Log closed: %s\n", fname);
    return 0;
}
