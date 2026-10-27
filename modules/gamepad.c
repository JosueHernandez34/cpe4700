#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include "gamepad.h"

#define MAX_EVENT_DEVICES 32
#define RECONNECT_MS      500
#define POLL_TIMEOUT_MS   100

typedef _Atomic(double) atomic_double;

// How each gp_axis maps to an evdev code, and how its raw value is scaled.
enum axis_kind { KIND_STICK, KIND_TRIGGER, KIND_HAT };
static const struct {
    int code;
    enum axis_kind kind;
} AXIS_MAP[GP_AXIS_COUNT] = {
    [GP_AXIS_LX]    = { ABS_X,     KIND_STICK },
    [GP_AXIS_LY]    = { ABS_Y,     KIND_STICK },
    [GP_AXIS_RX]    = { ABS_Z,     KIND_STICK },
    [GP_AXIS_RY]    = { ABS_RZ,    KIND_STICK },
    [GP_AXIS_L2]    = { ABS_BRAKE, KIND_TRIGGER },
    [GP_AXIS_R2]    = { ABS_GAS,   KIND_TRIGGER },
    [GP_AXIS_HAT_X] = { ABS_HAT0X, KIND_HAT },
    [GP_AXIS_HAT_Y] = { ABS_HAT0Y, KIND_HAT },
};

// Shared with the caller's threads: only atomics, so reads never block.
static atomic_double axis_value[GP_AXIS_COUNT];
static atomic_char   button_state[KEY_MAX + 1];
static atomic_bool   connected = false;
static atomic_bool   running = false;
static pthread_t     thread_id;

// Used only by the worker thread (and by gp_init before it starts).
static char fixed_path[64];              // empty string = auto-detect
static int  fd = -1;
static struct input_absinfo absinfo[GP_AXIS_COUNT];

static void *gamepad_worker(void *arg);

static int test_bit(int bit, const unsigned long *bits)
{
    return (bits[bit / (8 * sizeof(long))] >> (bit % (8 * sizeof(long)))) & 1;
}

// True if the open device looks like a gamepad: has BTN_SOUTH and ABS_X.
static int is_gamepad(int dev_fd)
{
    unsigned long key_bits[KEY_MAX / (8 * sizeof(long)) + 1] = {0};
    unsigned long abs_bits[ABS_MAX / (8 * sizeof(long)) + 1] = {0};

    if (ioctl(dev_fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0)
        return 0;
    if (ioctl(dev_fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0)
        return 0;
    return test_bit(BTN_SOUTH, key_bits) && test_bit(ABS_X, abs_bits);
}

// Turns a raw hardware value into the scaled value described in gamepad.h.
static double normalize(int axis, int raw)
{
    const struct input_absinfo *a = &absinfo[axis];
    double range = (double)a->maximum - a->minimum;

    if (range <= 0.0)
        return 0.0;

    switch (AXIS_MAP[axis].kind) {
    case KIND_HAT:
        return raw < 0 ? -1.0 : (raw > 0 ? 1.0 : 0.0);
    case KIND_TRIGGER: {
        double t = (raw - a->minimum) / range;
        return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    }
    default: {
        double x = (raw - (a->minimum + a->maximum) / 2.0) / (range / 2.0);
        double mag = x < 0.0 ? -x : x;
        if (mag > 1.0)
            mag = 1.0;
        if (mag < GP_DEADZONE)
            return 0.0;
        mag = (mag - GP_DEADZONE) / (1.0 - GP_DEADZONE);
        return x < 0.0 ? -mag : mag;
    }
    }
}

// Reads the current state straight from the driver. Used right after opening
// the device and after the kernel reports SYN_DROPPED (we missed events).
static void resync(void)
{
    unsigned long keys[KEY_MAX / (8 * sizeof(long)) + 1] = {0};

    for (int i = 0; i < GP_AXIS_COUNT; i++) {
        if (ioctl(fd, EVIOCGABS(AXIS_MAP[i].code), &absinfo[i]) == 0)
            atomic_store(&axis_value[i], normalize(i, absinfo[i].value));
        else
            atomic_store(&axis_value[i], 0.0);
    }
    if (ioctl(fd, EVIOCGKEY(sizeof(keys)), keys) == 0) {
        for (int c = 0; c <= KEY_MAX; c++)
            atomic_store(&button_state[c], (char)test_bit(c, keys));
    }
}

// Opens the fixed path, or scans /dev/input/event* for a gamepad.
static int open_device(void)
{
    char path[64];

    if (fixed_path[0]) {
        fd = open(fixed_path, O_RDONLY | O_NONBLOCK);
    } else {
        for (int n = 0; n < MAX_EVENT_DEVICES && fd < 0; n++) {
            snprintf(path, sizeof(path), "/dev/input/event%d", n);
            int t = open(path, O_RDONLY | O_NONBLOCK);
            if (t < 0)
                continue;
            if (is_gamepad(t))
                fd = t;
            else
                close(t);
        }
    }
    if (fd < 0)
        return -1;

    resync();
    atomic_store(&connected, true);
    return 0;
}

// Controller gone or unreadable: close it and zero everything so a stuck
// stick value can never keep driving something after an unplug.
static void drop_device(void)
{
    if (fd >= 0)
        close(fd);
    fd = -1;
    atomic_store(&connected, false);
    for (int i = 0; i < GP_AXIS_COUNT; i++)
        atomic_store(&axis_value[i], 0.0);
    for (int c = 0; c <= KEY_MAX; c++)
        atomic_store(&button_state[c], 0);
}

static void handle_event(const struct input_event *ev)
{
    if (ev->type == EV_KEY && ev->code <= KEY_MAX) {
        atomic_store(&button_state[ev->code], (char)(ev->value != 0));
    } else if (ev->type == EV_ABS) {
        for (int i = 0; i < GP_AXIS_COUNT; i++) {
            if (AXIS_MAP[i].code == ev->code) {
                atomic_store(&axis_value[i], normalize(i, ev->value));
                break;
            }
        }
    } else if (ev->type == EV_SYN && ev->code == SYN_DROPPED) {
        resync();
    }
}

static void *gamepad_worker(void *arg)
{
    (void)arg;

    while (atomic_load(&running)) {
        if (fd < 0 && open_device() < 0) {
            usleep(RECONNECT_MS * 1000);
            continue;
        }

        struct pollfd p = { .fd = fd, .events = POLLIN };
        int r = poll(&p, 1, POLL_TIMEOUT_MS);
        if (r < 0 && errno != EINTR) {
            drop_device();
            continue;
        }
        if (r <= 0)
            continue;
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            drop_device();
            continue;
        }

        struct input_event ev[32];
        ssize_t n = read(fd, ev, sizeof(ev));
        if (n < 0) {
            if (errno != EAGAIN && errno != EINTR)
                drop_device();      // ENODEV when unplugged
            continue;
        }
        for (size_t i = 0; i < (size_t)n / sizeof(ev[0]); i++)
            handle_event(&ev[i]);
    }

    drop_device();
    return NULL;
}

int gp_init(const char *path)
{
    if (atomic_exchange(&running, true))
        return -1;

    fixed_path[0] = '\0';
    if (path)
        snprintf(fixed_path, sizeof(fixed_path), "%s", path);

    if (pthread_create(&thread_id, NULL, gamepad_worker, NULL) != 0) {
        fprintf(stderr, "[ERROR] gamepad: could not start thread\n");
        atomic_store(&running, false);
        return -1;
    }
    return 0;
}

int gp_is_connected(void)
{
    return atomic_load(&connected);
}

double gp_get_axis(int axis)
{
    if (axis < 0 || axis >= GP_AXIS_COUNT)
        return 0.0;
    return atomic_load(&axis_value[axis]);
}

int gp_get_button(int code)
{
    if (code < 0 || code > KEY_MAX)
        return 0;
    return atomic_load(&button_state[code]);
}

int gp_cleanup(void)
{
    if (!atomic_exchange(&running, false))
        return -1;
    pthread_join(thread_id, NULL);
    return 0;
}
