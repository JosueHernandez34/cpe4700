/*
 * main.c -- Lab 04: Motor step-response characterization (both wheels)
 *
 * Usage:  ./lab04 deadband | step | speed [amplitude]
 *
 *   deadband : Part 3  -- ramp duty in 0.02 steps, one wheel at a time,
 *              reports each wheel's positive/negative deadband and
 *              writes deadband.csv
 *   step     : Parts 4-6 -- run the step sequence on both wheels at once,
 *              log step_response.csv
 *   speed    : Part 7  -- triangular reference on both wheels at once,
 *              log speed_control.csv
 *              Optional amplitude (edges/s, default 100 = the handout's
 *              +/-100). Any other amplitude logs to
 *              speed_control_<amplitude>.csv so the handout run is kept.
 *
 * Robot must be on a book with wheels off the ground. Ctrl-C stops safely.
 */

#define _POSIX_C_SOURCE 200809L

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pwm.h"
#include "encoder.h"
#include "speed_control.h"

#define NUM_MOTORS   2
#define LOOP_NS      1000000L     /* 1 ms loop period */
#define STEP_DUTY    0.60         /* Part 4 */
#define DB_STEP      0.02         /* Part 3 increment */
#define DB_DWELL_MS  1000         /* time at each duty level */
#define DB_MOVE_COUNTS 5          /* encoder counts of travel that count as "moving" */

#define TRI_CYCLES   3
#define TRI_PERIOD_S 2.0          /* 1 s down, 1 s up */
#define TRI_AMP_DEFAULT 100.0     /* handout target: +100 to -100 edges/s */
#define TRI_AMP_MAX  3000.0       /* ~3000/6100 + 0.06 deadband = ~0.55 duty, stays
                                   * under the 0.75 limit used in Lab 05 */

static volatile sig_atomic_t running = 1;
static void on_sigint(int sig) { (void)sig; running = 0; }

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

static void stop_all(void)
{
    for (int m = 0; m < NUM_MOTORS; m++)
        pwm_set_duty(m, 0.0f);
}

/* ------------------------------------------------------------------ */
/* Part 3: deadband. Each wheel is swept alone (the other stays at 0)  */
/* so one wheel's load can't affect the other's threshold.             */
/* ------------------------------------------------------------------ */
static void run_deadband(void)
{
    FILE *db = fopen("deadband.csv", "w");
    if (!db) { perror("deadband.csv"); return; }
    fprintf(db, "motor,direction,deadband_duty\n");

    for (int motor = 0; motor < NUM_MOTORS && running; motor++) {
        for (int dir = 1; dir >= -1 && running; dir -= 2) {      /* +, then - */
            printf("Motor %d, %s sweep:\n", motor, dir > 0 ? "positive" : "negative");
            int    found = 0;
            double found_duty = 0.0;

            for (int i = 1; i <= 50 && running; i++) {
                double duty = dir * i * DB_STEP;
                pwm_set_duty(motor, (float)duty);

                /* Movement is judged from encoder count travel, not enc_get_speed(),
                 * so a stopped wheel can't look "moving" from a stale speed. */
                int start = enc_get_count(motor);
                struct timespec next;
                clock_gettime(CLOCK_MONOTONIC, &next);
                for (int ms = 0; ms < DB_DWELL_MS && running; ms++)
                    tick(&next);
                int travel = abs(enc_get_count(motor) - start);
                printf("  duty=%+.2f  count_travel=%d\n", duty, travel);

                if (travel >= DB_MOVE_COUNTS) {
                    found = 1;
                    found_duty = duty;
                    break;                   /* stop at first movement */
                }
            }
            pwm_set_duty(motor, 0.0f);
            if (found) {
                printf("motor%d_%s_deadband = %.2f\n", motor,
                       dir > 0 ? "positive" : "negative", found_duty);
                fprintf(db, "%d,%s,%.2f\n", motor,
                        dir > 0 ? "positive" : "negative", found_duty);
            } else {
                printf("motor%d_%s_deadband = not found\n", motor,
                       dir > 0 ? "positive" : "negative");
                fprintf(db, "%d,%s,NaN\n", motor,
                        dir > 0 ? "positive" : "negative");
            }
            fflush(db);

            /* Let the wheel coast to a stop before the next sweep. */
            struct timespec settle = {1, 0};
            nanosleep(&settle, NULL);
        }
    }
    fclose(db);
}

/* ------------------------------------------------------------------ */
/* Parts 4-6: step response sequence, both wheels, logged every 1 ms.  */
/* ------------------------------------------------------------------ */
typedef struct { double duty; int ms; } step_t;

static void run_step(void)
{
    /* Sequence from Part 6. NOTE: the PDF's step 5 says "0.60" with no
     * sign -- almost certainly +0.60. Confirm with your instructor. */
    static const step_t seq[] = {
        { 0.0,        1000 },
        { +STEP_DUTY, 1000 },
        { 0.0,        2000 },
        { -STEP_DUTY, 1000 },
        { +STEP_DUTY, 1000 },
        { 0.0,        1000 },   /* trailing coast time (PDF gives none) */
    };

    FILE *fp = fopen("step_response.csv", "w");
    if (!fp) { perror("step_response.csv"); return; }
    fprintf(fp, "time_ms,duty_cycle,speed_0,position_0,speed_1,position_1\n");

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int t_ms = 0;

    for (size_t s = 0; s < sizeof seq / sizeof seq[0] && running; s++) {
        double duty = seq[s].duty;
        for (int m = 0; m < NUM_MOTORS; m++)
            pwm_set_duty(m, (float)duty);

        for (int i = 0; i < seq[s].ms && running; i++, t_ms++) {
            fprintf(fp, "%d,%f,%f,%d,%f,%d\n", t_ms, duty,
                    enc_get_speed(0), enc_get_count(0),    /* lab calls this enc_get_position */
                    enc_get_speed(1), enc_get_count(1));
            tick(&next);
        }
    }
    stop_all();
    fclose(fp);
    printf("Wrote step_response.csv (%d rows)\n", t_ms);
}

/* ------------------------------------------------------------------ */
/* Part 7: triangular speed reference, 3 cycles, closed loop on both   */
/* wheels (same target, independent controller state per wheel).       */
/* ------------------------------------------------------------------ */
static void run_speed(double amp)
{
    speed_control_init();

    /* Handout amplitude keeps the original file name; any other amplitude
     * gets its own file so the submitted run is never overwritten. */
    char fname[64];
    if (amp == TRI_AMP_DEFAULT)
        snprintf(fname, sizeof fname, "speed_control.csv");
    else
        snprintf(fname, sizeof fname, "speed_control_%.0f.csv", amp);
    printf("Triangle +/-%.0f edges/s, 3 cycles -> %s\n", amp, fname);

    FILE *speed_log = fopen(fname, "w");
    if (!speed_log) { perror(fname); return; }
    fprintf(speed_log, "time_ms,target_speed,measured_speed_0,measured_speed_1\n");

    const int total_ms = (int)(TRI_CYCLES * TRI_PERIOD_S * 1000.0);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (int t_ms = 0; t_ms < total_ms && running; t_ms++) {
        /* Time within the current 2 s cycle, in seconds. Integer math on
         * t_ms avoids float drift across cycles. */
        double cycle_time = (t_ms % (int)(TRI_PERIOD_S * 1000)) / 1000.0;

        double target_speed;
        /* +amp down to -amp over 1 s, then back up over 1 s. */
        if (cycle_time < 1.0)
            target_speed = amp - 2.0 * amp * cycle_time;
        else
            target_speed = -amp + 2.0 * amp * (cycle_time - 1.0);

        double measured_speed[NUM_MOTORS];
        for (int m = 0; m < NUM_MOTORS; m++) {
            measured_speed[m] = enc_get_speed(m);
            speed_control(m, target_speed);
        }
        fprintf(speed_log, "%d,%f,%f,%f\n", t_ms, target_speed,
                measured_speed[0], measured_speed[1]);
        tick(&next);
    }

    for (int m = 0; m < NUM_MOTORS; m++)
        speed_control(m, 0.0);        /* command target 0, then clean up */
    fclose(speed_log);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s deadband|step|speed [amplitude]\n", argv[0]);
        return 1;
    }

    /* Optional triangle amplitude for "speed"; checked before any motor
     * is initialized so a bad value never spins the wheels. */
    double amp = TRI_AMP_DEFAULT;
    if (argc >= 3) {
        amp = atof(argv[2]);
        if (amp <= 0.0 || amp > TRI_AMP_MAX) {
            fprintf(stderr, "amplitude must be > 0 and <= %.0f edges/s\n", TRI_AMP_MAX);
            return 1;
        }
    }

    signal(SIGINT, on_sigint);

    if (pwm_init() != 0 || enc_init() != 0) {
        fprintf(stderr, "init failed (need sudo for sysfs/GPIO?)\n");
        return 1;
    }

    if      (!strcmp(argv[1], "deadband")) run_deadband();
    else if (!strcmp(argv[1], "step"))     run_step();
    else if (!strcmp(argv[1], "speed"))    run_speed(amp);
    else fprintf(stderr, "unknown mode '%s'\n", argv[1]);

    /* Part 8: cleanup */
    stop_all();
    pwm_cleanup();
    enc_cleanup();
    speed_control_cleanup();   /* note: PWM already torn down; harmless if it errors */
    return 0;
}
