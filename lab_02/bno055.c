#include "bno055.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

static int write_reg(
    int fd,
    uint8_t reg,
    uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg;
    tx[1] = value;

    return (write(fd, tx, 2) == 2) ? 0 : -1;
}

static int read_reg(
    int fd,
    uint8_t reg,
    uint8_t *data,
    size_t len)
{
    if (write(fd, &reg, 1) != 1)
        return -1;

    if (read(fd, data, len) != (ssize_t)len)
        return -1;

    return 0;
}

static int16_t s16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)((msb << 8) | lsb);
}

int bno055_open(const char *device)
{
    int fd = open(device, O_RDWR);

    if (fd < 0)
        return -1;

    if (ioctl(fd, I2C_SLAVE, BNO055_I2C_ADDR) < 0)
    {
        close(fd);
        return -1;
    }

    return fd;
}

void bno055_close(int fd)
{
    close(fd);
}

int bno055_init(int fd)
{
    uint8_t chip_id;

    if (read_reg(fd,
                 BNO055_CHIP_ID,
                 &chip_id,
                 1))
        return -1;

    if (chip_id != 0xA0)
    {
        fprintf(stderr,
                "Unexpected CHIP_ID 0x%02X\n",
                chip_id);
        return -1;
    }

    write_reg(
        fd,
        BNO055_OPR_MODE,
        BNO055_CONFIG_MODE);

    usleep(30000);

    write_reg(fd,
              BNO055_PWR_MODE,
              0x00);

    usleep(10000);

    write_reg(
        fd,
        BNO055_OPR_MODE,
        BNO055_NDOF_MODE);

    usleep(50000);

    return 0;
}

int bno055_read_euler(
    int fd,
    bno055_euler_t *e)
{
    uint8_t data[6];

    if (read_reg(fd,
                 BNO055_EULER_H_LSB,
                 data,
                 6))
        return -1;

    e->heading =
        s16(data[0], data[1]) / 16.0f;

    e->roll =
        s16(data[2], data[3]) / 16.0f;

    e->pitch =
        s16(data[4], data[5]) / 16.0f;

    return 0;
}

int bno055_read_quaternion(
    int fd,
    bno055_quat_t *q)
{
    uint8_t d[8];

    if (read_reg(fd,
                 BNO055_QUATERNION_W_LSB,
                 d,
                 8))
        return -1;

    q->w = s16(d[0], d[1]) / 16384.0f;
    q->x = s16(d[2], d[3]) / 16384.0f;
    q->y = s16(d[4], d[5]) / 16384.0f;
    q->z = s16(d[6], d[7]) / 16384.0f;

    return 0;
}

int bno055_read_calibration(
    int fd,
    uint8_t *sys,
    uint8_t *gyro,
    uint8_t *accel,
    uint8_t *mag)
{
    uint8_t status;

    if (read_reg(fd,
                 BNO055_CALIB_STAT,
                 &status,
                 1))
        return -1;

    *sys   = (status >> 6) & 0x03;
    *gyro  = (status >> 4) & 0x03;
    *accel = (status >> 2) & 0x03;
    *mag   = status & 0x03;

    return 0;
}