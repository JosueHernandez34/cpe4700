#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <gpiod.h>

/* ---------- PWM DEFINITIONS ---------- */
#define PWM_CHIP_PATH   "/sys/class/pwm/pwmchip2"
#define PWM_CHANNEL     "0"
#define PWM_PATH        PWM_CHIP_PATH "/pwm" PWM_CHANNEL

#define PWM_CHIP_PATH2  "/sys/class/pwm/pwmchip3"
#define PWM_PATH2       PWM_CHIP_PATH2 "/pwm" PWM_CHANNEL

#define PWM_PERIOD_NS   100000    // 100 kHz
#define PWM_DUTY_50     50000    // 50%

/* ---------- GPIO (Direction: PY.04) ---------- */
#define GPIO_CHIP       "/dev/gpiochip0"
#define DIR_LINE_OFFSET 106
#define DIR_LINE_OFFSET2 113

/* ---------- Helper: write to sysfs ---------- */
void write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror(path);
        exit(EXIT_FAILURE);
    }
    write(fd, value, strlen(value));
    close(fd);
}

int main()
{
    /* ---------- GPIO SETUP (Direction) ---------- */
    struct gpiod_chip *chip;
    struct gpiod_line *dir_line, *dir_line2;

    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("gpiod_chip_open");
        return 1;
    }

    dir_line = gpiod_chip_get_line(chip, DIR_LINE_OFFSET);
    if (!dir_line) {
        perror("gpiod_chip_get_line");
        return 1;
    }

    dir_line2 = gpiod_chip_get_line(chip, DIR_LINE_OFFSET2);
    if (!dir_line2) {
        perror("gpiod_chip_get_line2");
        return 1;
    } 

    if (gpiod_line_request_output(dir_line, "mdd10-dir", 1) < 0) {
        perror("gpiod_line_request_output");
        return 1;
    }


    if (gpiod_line_request_output(dir_line2, "mdd10-dir2", 1) < 0) {
        perror("gpiod_line_request_output");
        return 1;
    }

    /* ---------- PWM SETUP ---------- */
    if (access(PWM_PATH, F_OK) != 0) {
        write_sysfs(PWM_CHIP_PATH "/export", PWM_CHANNEL);
        usleep(100000);
    }

    if (access(PWM_PATH2, F_OK) != 0) {
        write_sysfs(PWM_CHIP_PATH2 "/export", PWM_CHANNEL);
        usleep(100000);
    } 
	    
	    
    write_sysfs(PWM_PATH "/period", "100000");
    write_sysfs(PWM_PATH2 "/period", "100000");
    write_sysfs(PWM_PATH "/duty_cycle", "0");
    write_sysfs(PWM_PATH2 "/duty_cycle", "0");
    write_sysfs(PWM_PATH "/enable", "1");
    write_sysfs(PWM_PATH2 "/enable", "1");

    for(int i=0;i<1;i++)
    {
        /* ---------- MOTOR CONTROL ---------- */
        printf("Motor forward (100%% speed)\n");
        gpiod_line_set_value(dir_line, 0);
        gpiod_line_set_value(dir_line2, 1);
        write_sysfs(PWM_PATH "/duty_cycle", "50000");
        write_sysfs(PWM_PATH2 "/duty_cycle", "50000");
        sleep(5);
        write_sysfs(PWM_PATH "/duty_cycle", "50000");
        write_sysfs(PWM_PATH2 "/duty_cycle", "20000");
	    sleep(3);
        printf("Motor reverse (100%% speed)\n");
        gpiod_line_set_value(dir_line, 1);
        gpiod_line_set_value(dir_line2, 0);
        write_sysfs(PWM_PATH "/duty_cycle", "50000");
        write_sysfs(PWM_PATH2 "/duty_cycle", "50000");
        sleep(2);
        write_sysfs(PWM_PATH "/duty_cycle", "30000");
        write_sysfs(PWM_PATH2 "/duty_cycle", "30000");
	    sleep(2);
    }
    printf("Motor stop\n");
    write_sysfs(PWM_PATH "/duty_cycle", "10000");
    write_sysfs(PWM_PATH2 "/duty_cycle", "10000");
    sleep(1);
    write_sysfs(PWM_PATH "/duty_cycle", "0");
    write_sysfs(PWM_PATH2 "/duty_cycle", "0");
    sleep(3);

    /* ---------- CLEANUP ---------- */
    write_sysfs(PWM_PATH "/enable", "0");
    write_sysfs(PWM_PATH2 "/enable", "0");
    gpiod_line_release(dir_line);
    gpiod_chip_close(chip);

    return 0;
}
