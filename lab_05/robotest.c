/*
 * robotest.c -- Lab 05: self-balancing with GTK4 display
 *
 * Usage:  sudo ./robotest
 *
 * The 1 ms balance loop runs in a worker thread; GTK4 runs in the main
 * thread. Robot must be on a book with wheels off the ground for the first
 * tests. Ctrl-C or the GUI "Quit" button stops the loop and zeroes the PWM.
 */

#define _POSIX_C_SOURCE 200809L

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <time.h>

#include "pwm.h"
#include "encoder.h"
#include "imu.h"
#include "balance_loop.h"
#include "gui.h"

#define LOOP_NS 1000000L             /* 1 ms loop period */

static atomic_int running = 1;

static void request_stop(void) { atomic_store(&running, 0); }
static void on_sigint(int sig) { (void)sig; request_stop(); }

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

static void *balance_thread(void *arg)
{
    (void)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (unsigned int n = 0; atomic_load(&running); n++) {
        balance_loop_step(n);
        tick(&next);
    }
    balance_loop_cleanup();          /* zero duty before the thread exits */
    return NULL;
}

int main(int argc, char **argv)
{
    signal(SIGINT, on_sigint);

    if (pwm_init() != 0 || enc_init() != 0 || imu_init() != 0) {
        fprintf(stderr, "init failed (need sudo for sysfs/GPIO/I2C?)\n");
        return 1;
    }
    balance_loop_init();

    pthread_t tid;
    if (pthread_create(&tid, NULL, balance_thread, NULL) != 0) {
        fprintf(stderr, "failed to start balance thread\n");
        return 1;
    }

    int status = gui_run(argc, argv, request_stop);

    request_stop();                  /* window closed without Quit, etc. */
    pthread_join(tid, NULL);

    imu_cleanup();
    enc_cleanup();
    pwm_cleanup();
    return status;
}
