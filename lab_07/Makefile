# Lab 07 build. Shared modules live in ~/dev/cpe4700/modules/
MOD_DIR ?= ../modules

CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -I$(MOD_DIR) -I.
LDLIBS  = -lgpiod -lpthread -lm

SRCS = main.c balance_ctrl.c $(MOD_DIR)/pwm.c $(MOD_DIR)/encoder.c $(MOD_DIR)/imu.c \
       $(MOD_DIR)/bno055.c $(MOD_DIR)/gamepad.c

lab07: $(SRCS) balance_ctrl.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

# Gamepad mapping check. No motors, no IMU. Run: sudo ./joystick_test   (or: sudo ./joystick_test live)
joystick_test: joystick_test.c balance_ctrl.c balance_ctrl.h $(MOD_DIR)/gamepad.c
	$(CC) $(CFLAGS) -o $@ joystick_test.c balance_ctrl.c $(MOD_DIR)/gamepad.c -lpthread -lm

# Control-logic check with fake measurements. No hardware, no motors.

test_ctrl: test_ctrl.c balance_ctrl.c balance_ctrl.h
	$(CC) $(CFLAGS) -o $@ test_ctrl.c balance_ctrl.c -lm

# Cutoff rule with the real default stop angles.
test_cutoff: test_cutoff.c balance_ctrl.h
	$(CC) $(CFLAGS) -o $@ test_cutoff.c -lm

check: test_ctrl test_cutoff
	./test_ctrl
	./test_cutoff

clean:
	rm -f lab07 test_ctrl test_cutoff joystick_test
.PHONY: check clean
