#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>

#define BNO055_I2C_ADDR 0x28

/* Registers */
#define BNO055_CHIP_ID          0x00
#define BNO055_CALIB_STAT       0x35
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E

#define BNO055_EULER_H_LSB      0x1A
#define BNO055_QUATERNION_W_LSB 0x20

#define BNO055_CONFIG_MODE      0x00
#define BNO055_NDOF_MODE        0x0C

typedef struct
{
    float heading;
    float roll;
    float pitch;
} bno055_euler_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} bno055_quat_t;

int bno055_open(const char *device);
void bno055_close(int fd);

int bno055_init(int fd);

int bno055_read_euler(
    int fd,
    bno055_euler_t *euler);

int bno055_read_quaternion(
    int fd,
    bno055_quat_t *q);

int bno055_read_calibration(
    int fd,
    uint8_t *sys,
    uint8_t *gyro,
    uint8_t *accel,
    uint8_t *mag);

#endif