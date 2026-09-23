#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>
#include <sched.h>
#include <gpiod.h>
#include <time.h>
#include "encoder.h"

// Adjust these for the correct Jetson GPIO lines
static const char *CHIPNAME = "gpiochip0";
static const int ENC0_A = 124;
static const int ENC0_B = 53;
static const int ENC1_A = 52;
static const int ENC1_B = 51;

static struct gpiod_chip *chip;
static struct gpiod_line *line_a[2];
static struct gpiod_line *line_b[2];
static pthread_t thread_id[2];

// Thread-safe atomic variables -- read and write these
// from your decode logic
typedef _Atomic(double) atomic_double;
static atomic_long encoder_count[2] = {0};
static atomic_double encoder_speed[2] = {0};
static atomic_bool running = true;

static void *encoder_worker(void *arg);
static int enc_init_inst(int encoder_id, int line_a_offset,
           int line_b_offset);

int enc_init(void)
{
    if (enc_init_inst(0, ENC0_A, ENC0_B) < 0)
    {
        fprintf(stderr, "[ERROR] Failed to initialize encoder 0\n");
        return -1;
    }
    if (enc_init_inst(1, ENC1_A, ENC1_B) < 0)
    {
        fprintf(stderr, "[ERROR] Failed to initialize encoder 1\n");
        return -1;
    }
    return 0;
}

// Provided: opens the GPIO chip, requests lines, and spawns
// the polling thread
static int enc_init_inst(int encoder_id, int line_a_offset, int line_b_offset)
{
    chip = gpiod_chip_open_by_name(CHIPNAME);
    if (!chip)
    {
        fprintf(stderr, "[ERROR] Failed to open chip: %s\n", CHIPNAME);
        return -1;
    }

    line_a[encoder_id] = gpiod_chip_get_line(chip, line_a_offset);
    line_b[encoder_id] = gpiod_chip_get_line(chip, line_b_offset);
    if (!line_a[encoder_id] || !line_b[encoder_id])
    {
        fprintf(stderr, "[ERROR] Failed to fetch hardware lines\n");
        gpiod_chip_close(chip);
        return -1;
    }

    // Pin A is watched for edges; Pin B is sampled as a plain
    // digital input
    if (gpiod_line_request_both_edges_events(line_a[encoder_id],
        "Encoder_A") < 0)
    {
        fprintf(stderr, "[ERROR] Failed to request events on Pin A\n");
        gpiod_chip_close(chip);
        return -1;
    }

    if (gpiod_line_request_input(line_b[encoder_id], "Encoder_B") < 0)
    {
        fprintf(stderr, "[ERROR] Failed to request input on Pin B\n");
        gpiod_line_release(line_a[encoder_id]);
        gpiod_chip_close(chip);
        return -1;
    }

    if (pthread_create(&thread_id[encoder_id], NULL, encoder_worker,
                       (void *)(long)encoder_id) != 0)
    {
        fprintf(stderr, 
            "[ERROR] Failed to create background thread\n");
        return -1;
    }

    return 0;
}

// Provided: raises the calling thread to real-time FIFO priority
static void set_realtime_priority(void)
{
    struct sched_param param;
    int policy = SCHED_FIFO;
    param.sched_priority = sched_get_priority_max(policy);

    int result=pthread_setschedparam(pthread_self(),policy,&param);
    if (result != 0)
    {
        fprintf(stderr, "Failed to set RT priority: %s\n",
             strerror(result));
        // Thread will fall back to normal scheduling
    }
}

// Background POSIX thread: polls Pin A edges and hands them off to your decode logic
static void *encoder_worker(void *arg)
{
    set_realtime_priority();

    int encoder_id = (int)(long)arg;
    struct gpiod_line_event event;
    struct timespec timeout = {0, 100000000}; // 100 ms poll timeout

    static struct timespec last_edge_ts[2] = {{0, 0}, {0, 0}};

    while (atomic_load(&running))
    {
        int event_status = gpiod_line_event_wait(line_a[encoder_id], &timeout);

        if (event_status > 0)
        {
            if (gpiod_line_event_read(line_a[encoder_id], &event) < 0)
                continue;

            int state_a = (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
            int state_b = gpiod_line_get_value(line_b[encoder_id]);

            if (state_a == 1 && state_b == 1) {
                atomic_fetch_add(&encoder_count[encoder_id], 1);
            }
            else if (state_a == 1 && state_b == 0) {
                atomic_fetch_sub(&encoder_count[encoder_id], 1);
            }
            else if (state_a == 0 && state_b == 1) {
                atomic_fetch_sub(&encoder_count[encoder_id], 1);
            }
            else {
                atomic_fetch_add(&encoder_count[encoder_id], 1);
            }

            struct timespec now = event.ts;
            if (last_edge_ts[encoder_id].tv_sec != 0 || last_edge_ts[encoder_id].tv_nsec != 0) {
                long seconds = now.tv_sec - last_edge_ts[encoder_id].tv_sec;
                long nanoseconds = now.tv_nsec - last_edge_ts[encoder_id].tv_nsec;
                double delta_s = (double)seconds + ((double)nanoseconds / 1000000000.0);

                if (delta_s > 0.0) {
                    double revolutions_per_second = 1.0 / delta_s;
                    atomic_store(&encoder_speed[encoder_id], revolutions_per_second);
                }
            }

            last_edge_ts[encoder_id] = now;
        }
    }

    gpiod_line_release(line_a[encoder_id]);
    gpiod_line_release(line_b[encoder_id]);
    return NULL;
}

int enc_get_count(int encoder)
{
    if (encoder != 0 && encoder != 1) return -1;
    return (int)atomic_load(&encoder_count[encoder]);
}

double enc_get_speed(int encoder)
{
    if (encoder != 0 && encoder != 1) return -1;
    return atomic_load(&encoder_speed[encoder]);
}

int enc_cleanup(void)
{
    atomic_store(&running, false);
    pthread_join(thread_id[0], NULL);
    pthread_join(thread_id[1], NULL);
    gpiod_chip_close(chip);
    return 0;
}
