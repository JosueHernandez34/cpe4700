/*
 * Bob Turney
 * compile command
 *          $ gcc -o imu_test1 imu_test1.c bno055.c
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "bno055.h"
#include "imu.h"      /* Lab 05: for the new gyro prototypes and IMU_PITCH_RATE_* settings */

static int fd;

int imu_init(void)
{

    fd = bno055_open("/dev/i2c-7");

    if (fd < 0)
    {
        perror("BNO055 open");
        return 1;
    }

    if (bno055_init(fd))
    {
        fprintf(stderr,
                "Failed to initialize BNO055\n");
        return 1;
    }

    printf("BNO055 initialized\n");
    return 0;
}

double get_imu_roll()
{
    bno055_euler_t e;

    if(!bno055_read_euler(fd, &e))
    {
        return e.roll;
    }
    fprintf(stderr, "Failed to read IMU data\n");
    return 0.0; // Return a default value or handle the error as needed
}

double get_imu_pitch()
{
    bno055_euler_t e;

    if(!bno055_read_euler(fd, &e))
    {
        return e.pitch;
    }
    fprintf(stderr, "Failed to read IMU data\n");
    return 0.0; // Return a default value or handle the error as needed
}

double get_imu_heading()
{
    bno055_euler_t e;

    if(!bno055_read_euler(fd, &e))
    {
        return e.heading;
    }
    fprintf(stderr, "Failed to read IMU data\n");
    return 0.0; // Return a default value or handle the error as needed
}

void imu_cleanup(void)
{
    bno055_close(fd);
}

/* ---- Added for Lab 05 (new functions only) ---- */

int get_imu_gyro(double *gx, double *gy, double *gz)
{
    float x, y, z;

    if (bno055_read_gyro(fd, &x, &y, &z))
        return -1;                      /* read failed: leave the outputs untouched */

    *gx = x;
    *gy = y;
    *gz = z;
    return 0;
}

double get_imu_pitch_rate(void)
{
    double g[3];

    /* Axis and sign are measured by pitch_probe; until they are set, refuse to guess. */
    if (IMU_PITCH_RATE_AXIS < 0 || IMU_PITCH_RATE_SIGN == 0)
        return NAN;
    if (get_imu_gyro(&g[0], &g[1], &g[2]))
        return NAN;                     /* failed read, caller must handle it */
    return IMU_PITCH_RATE_SIGN * g[IMU_PITCH_RATE_AXIS];
}
