#ifndef PWM_H
#define PWM_H

// GPIO direction line definitions on gpiochip0
#define GPIO_CHIP0 "/dev/gpiochip0"
#define GPIO_CHIP2 "/dev/gpiochip2"
#define GPIO_CHIP3 "/dev/gpiochip3"
#define GPIO_CHIP GPIO_CHIP0

#define DIR_LINE_OFFSET 106   // channel 0 direction
#define DIR_LINE_OFFSET2 113  // channel 1 direction

// PWM constants
#define PWM_CHANNEL "0"
#define PWM_ZERO 0
#define PWM_CHIP2 2
#define PWM_CHIP3 3
#define PWM0 0
#define PWM_CHANNEL_COUNT 2

#define PWM_CHIP_PATH2 "/sys/class/pwm/pwmchip2"
#define PWM_CHIP_PATH3 "/sys/class/pwm/pwmchip3"
#define PWM_CHIP_PATH PWM_CHIP_PATH2
#define PWM_PATH2 PWM_CHIP_PATH2 "/pwm" PWM_CHANNEL
#define PWM_PATH3 PWM_CHIP_PATH3 "/pwm" PWM_CHANNEL
#define PWM_PATH PWM_PATH2

#define PWM_PERIOD_NS 100000 // 10 kHz

int pwm_init(void);
int pwm_set_duty(int channel, float duty_cycle);
int pwm_cleanup(void);

#endif