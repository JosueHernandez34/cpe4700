#ifndef TUNE_SHARED_H
#define TUNE_SHARED_H

/* Values shared between the control loop thread (main.c) and the tuning window (tune_gui.c).
 * Everything is an atomic, so neither side ever waits for the other. */
#include <stdatomic.h>

typedef struct {
    /* set by the window, read by the control loop every pass */
    _Atomic double theta0, kp, ki, kd, kv;
    _Atomic int    rearm_req;           /* 1 = clear a trip and wait to arm again */
    _Atomic int    quit;                /* 1 = stop the run */

    /* slider ranges and defaults, written once before the window opens */
    double theta0_min, theta0_max;
    double d_theta0, d_kp, d_ki, d_kd, d_kv;

    /* written by the control loop, read by the window */
    _Atomic double pitch, pitch_target, speed, speed_cmd, left, right;
    _Atomic int    state;               /* 0 waiting to arm, 1 armed, 2 live, 3 tripped */
    _Atomic int    trip_reason;         /* 1 cutoff, 2 resting on a block stop */
    _Atomic int    done;                /* 1 = the control loop has ended */
    const char    *err;                 /* why it ended, valid once done is 1 */
    const char    *log_name;
} tune_t;

/* Opens the tuning window and returns when it is closed (or when t->done is set). 0 = ok. */
int tune_gui_run(tune_t *t);
int tune_gui_available(void);           /* 1 if a display can be opened */

#endif
