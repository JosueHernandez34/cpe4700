# Lab 07 build. Shared modules live in ~/dev/cpe4700/modules/
MOD_DIR ?= ../modules

CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -I$(MOD_DIR)
LDLIBS  = -lgpiod -lpthread -lm

SRCS = main.c $(MOD_DIR)/pwm.c $(MOD_DIR)/encoder.c $(MOD_DIR)/imu.c \
       $(MOD_DIR)/bno055.c $(MOD_DIR)/gamepad.c

# sudo ./lab07 [--monitor] [--kp X --ki X --kd X --kv X --theta0 X ...]
lab07: $(SRCS) $(MOD_DIR)/imu.h $(MOD_DIR)/gamepad.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

clean:
	rm -f lab07
.PHONY: clean
