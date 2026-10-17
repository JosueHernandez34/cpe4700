/*
 * main.c -- Lab 04: Motor step-response characterization
 *
 * Usage:  ./lab04 deadband | step | speed     [motor]   (motor defaults to 0)
 *
 *   deadband : Part 3  -- ramp duty in 0.02 steps, you note when wheel moves
 *   step     : Parts 4-6 -- run the step sequence, log step_response.csv
 *   speed    : Part 7  -- triangular reference, log speed_control.csv
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

#define LOOP_NS      1000000L     /* 1 ms loop period */
#define STEP_DUTY    0.60         /* Part 4 */
#define DB_STEP      0.02         /* Part 3 increment */
#define DB_DWELL_MS  1000         /* time at each duty level */
#define DB_MOVE_COUNTS 5          /* encoder counts of travel that count as "moving" */

#define TRI_CYCLES   3
#define TRI_PERIOD_S 2.0          /* 1 s down, 1 s up */

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

/* ------------------------------------------------------------------ */
/* Part 3: deadband. Prints speed at each duty so you can read the     */
/* smallest duty where the wheel starts to move.                       */
/* ------------------------------------------------------------------ */
static void run_deadband(int motor)
{
    for (int dir = 1; dir >= -1 && running; dir -= 2) {      /* +, then - */
        printf("%s sweep:\n", dir > 0 ? "Positive" : "Negative");
        double found = 0.0;

        for (int i = 0; i <= 50 && running; i++) {
            double duty = dir * i * DB_STEP;
            pwm_set_duty(motor, (float)duty);

            /* Movement is judged from encoder count travel, not enc_get_speed():
             * the module's speed is unsigned and holds the last edge's value
             * after the wheel stops, so it would look "moving" at duty 0. */
            int start = enc_get_count(motor);
            struct timespec next;
            clock_gettime(CLOCK_MONOTONIC, &next);
            for (int ms = 0; ms < DB_DWELL_MS && running; ms++)
                tick(&next);
            int travel = abs(enc_get_count(motor) - start);
            printf("  duty=%+.2f  count_travel=%d\n", duty, travel);

            if (i > 0 && travel >= DB_MOVE_COUNTS) {
                found = duty;
                break;                       /* stop at first movement */
            }
        }
        pwm_set_duty(motor, 0.0f);
        printf(dir > 0 ? "positive_deadband = %.2f\n"
                       : "negative_deadband = %.2f\n", found);

        /* Let the wheel coast to a stop before the reverse sweep. */
        struct timespec settle = {1, 0};
        nanosleep(&settle, NULL);
    }
}

/* ------------------------------------------------------------------ */
/* Parts 4-6: step response sequence, logged every 1 ms.               */
/* ------------------------------------------------------------------ */
typedef struct { double duty; int ms; } step_t;

static void run_step(int motor)
{
    /* Sequence from Part 6. NOTE: the PDF's step 5 says "0.60" with no
     * sign -- almost certainly +0.60. Confirm with your instructor. */
    static const step_t seq[] = {
        { 0.0,        1000 },
        { +STEP_DUTY, 1000 },
        { 0.0,        2000 },
        { -STEP_DUTY, 1000 },
        { +STEP_DUTY, 1000 },
        { 0.0,        1000 },   /* TODO: trailing coast time (PDF gives none) */
    };

    FILE *fp = fopen("step_response.csv", "w");
    if (!fp) { perror("step_response.csv"); return; }
    fprintf(fp, "time_ms,duty_cycle,speed,position\n");

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int t_ms = 0;

    for (size_t s = 0; s < sizeof seq / sizeof seq[0] && running; s++) {
        double duty = seq[s].duty;
        pwm_set_duty(motor, (float)duty);

        for (int i = 0; i < seq[s].ms && running; i++, t_ms++) {
            double speed = enc_get_speed(motor);
            int    pos   = enc_get_count(motor);   /* lab calls this enc_get_position */
            fprintf(fp, "%d,%f,%f,%d\n", t_ms, duty, speed, pos);
            tick(&next);
        }
    }
    pwm_set_duty(motor, 0.0f);
    fclose(fp);
    printf("Wrote step_response.csv (%d rows)\n", t_ms);
}

/* ------------------------------------------------------------------ */
/* Part 7: triangular speed reference, 3 cycles, closed loop.          */
/* ------------------------------------------------------------------ */
static void run_speed(int motor)
{
    speed_control_init();
    FILE *speed_log = fopen("speed_control.csv", "w");
    if (!speed_log) { perror("speed_control.csv"); return; }
    fprintf(speed_log, "time_ms,target_speed,measured_speed\n");

    const int total_ms = (int)(TRI_CYCLES * TRI_PERIOD_S * 1000.0);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (int t_ms = 0; t_ms < total_ms && running; t_ms++) {
        /* Time within the current 2 s cycle, in seconds. Integer math on
         * t_ms avoids float drift across cycles. */
        double cycle_time = (t_ms % (int)(TRI_PERIOD_S * 1000)) / 1000.0;

        double target_speed;
        if (cycle_time < 1.0)
            target_speed = 100.0 - 200.0 * cycle_time;
        else
            target_speed = -100.0 + 200.0 * (cycle_time - 1.0);

        double measured_speed = enc_get_speed(motor);
        speed_control(motor, target_speed);
        fprintf(speed_log, "%d,%f,%f\n", t_ms, target_speed, measured_speed);
        tick(&next);
    }

    speed_control(motor, 0.0);        /* command target 0, then clean up */
    fclose(speed_log);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s deadband|step|speed [motor]\n", argv[0]);
        return 1;
    }
    int motor = (argc > 2) ? atoi(argv[2]) : 0;

    signal(SIGINT, on_sigint);

    if (pwm_init() != 0 || enc_init() != 0) {
        fprintf(stderr, "init failed (need sudo for sysfs/GPIO?)\n");
        return 1;
    }

    if      (!strcmp(argv[1], "deadband")) run_deadband(motor);
    else if (!strcmp(argv[1], "step"))     run_step(motor);
    else if (!strcmp(argv[1], "speed"))    run_speed(motor);
    else fprintf(stderr, "unknown mode '%s'\n", argv[1]);

    /* Part 8: cleanup */
    pwm_cleanup();
    enc_cleanup();
    speed_control_cleanup();   /* note: PWM already torn down; harmless if it errors */
    return 0;
}
