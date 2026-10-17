#define _XOPEN_SOURCE 700

#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "pwm.h"

static struct gpiod_chip *chip = NULL;
static struct gpiod_line *dir_line[PWM_CHANNEL_COUNT] = {NULL, NULL};

static int write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        return -1;
    }

    size_t len = strlen(value);
    if (write(fd, value, len) != (ssize_t)len) {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static const char *pwm_chip_path(int channel)
{
    if (channel == 0) {
        return PWM_CHIP_PATH2;
    }
    if (channel == 1) {
        return PWM_CHIP_PATH3;
    }
    return NULL;
}

static const char *pwm_export_path(int channel)
{
    if (channel == 0) {
        return PWM_CHIP_PATH2 "/export";
    }
    if (channel == 1) {
        return PWM_CHIP_PATH3 "/export";
    }
    return NULL;
}

static const char *pwm_unexport_path(int channel)
{
    if (channel == 0) {
        return PWM_CHIP_PATH2 "/unexport";
    }
    if (channel == 1) {
        return PWM_CHIP_PATH3 "/unexport";
    }
    return NULL;
}

static int pwm_dir_line_offset(int channel)
{
    if (channel == 0) {
        return DIR_LINE_OFFSET;
    }
    if (channel == 1) {
        return DIR_LINE_OFFSET2;
    }
    return -1;
}

int pwm_init(void)
{
    char path[128];
    int channel;

    for (channel = 0; channel < PWM_CHANNEL_COUNT; channel++) {
        const char *chip_path = pwm_chip_path(channel);
        if (chip_path == NULL) {
            return -1;
        }

        snprintf(path, sizeof(path), "%s/pwm0", chip_path);
        if (access(path, F_OK) != 0) {
            if (write_sysfs(pwm_export_path(channel), PWM_CHANNEL) != 0) {
                return -1;
            }
            struct timespec ts = {0, 100000000L};
            nanosleep(&ts, NULL);
        }

        snprintf(path, sizeof(path), "%s/pwm0/period", chip_path);
        if (write_sysfs(path, "100000") != 0) {
            return -1;
        }

        snprintf(path, sizeof(path), "%s/pwm0/duty_cycle", chip_path);
        if (write_sysfs(path, "0") != 0) {
            return -1;
        }

        snprintf(path, sizeof(path), "%s/pwm0/enable", chip_path);
        if (write_sysfs(path, "1") != 0) {
            return -1;
        }
    }

    chip = gpiod_chip_open(GPIO_CHIP);
    if (chip == NULL) {
        return -1;
    }

    for (channel = 0; channel < PWM_CHANNEL_COUNT; channel++) {
        int offset = pwm_dir_line_offset(channel);
        if (offset < 0) {
            return -1;
        }

        dir_line[channel] = gpiod_chip_get_line(chip, offset);
        if (dir_line[channel] == NULL) {
            return -1;
        }

        if (gpiod_line_request_output(dir_line[channel], "motor_dir", 0) != 0) {
            return -1;
        }
    }

    return 0;
}

int pwm_set_duty(int channel, float duty_cycle)
{
    char duty_cycle_str[32];
    char path[128];
    float magnitude;
    int direction;

    if (channel < 0 || channel >= PWM_CHANNEL_COUNT) {
        return -1;
    }
    if (dir_line[channel] == NULL) {
        return -1;
    }

    if (duty_cycle > 1.0f) {
        duty_cycle = 1.0f;
    } else if (duty_cycle < -1.0f) {
        duty_cycle = -1.0f;
    }

    direction = (duty_cycle >= 0.0f) ? 1 : 0;
    if (gpiod_line_set_value(dir_line[channel], direction) != 0) {
        return -1;
    }

    magnitude = fabsf(duty_cycle);
    snprintf(duty_cycle_str, sizeof(duty_cycle_str), "%d", (int)(magnitude * (float)PWM_PERIOD_NS));

    snprintf(path, sizeof(path), "%s/pwm0/duty_cycle", pwm_chip_path(channel));
    return write_sysfs(path, duty_cycle_str);
}

int pwm_cleanup(void)
{
    char path[128];
    int channel;

    for (channel = 0; channel < PWM_CHANNEL_COUNT; channel++) {
        snprintf(path, sizeof(path), "%s/pwm0/enable", pwm_chip_path(channel));
        write_sysfs(path, "0");

        snprintf(path, sizeof(path), "%s/pwm0/duty_cycle", pwm_chip_path(channel));
        write_sysfs(path, "0");

        if (dir_line[channel] != NULL) {
            gpiod_line_release(dir_line[channel]);
            dir_line[channel] = NULL;
        }

        if (write_sysfs(pwm_unexport_path(channel), PWM_CHANNEL) != 0) {
            if (errno == ENOENT) {
                continue;
            }
        }
    }

    if (chip != NULL) {
        gpiod_chip_close(chip);
        chip = NULL;
    }

    return 0;
}