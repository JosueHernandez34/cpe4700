# Lab 06 build. Shared modules live in ~/dev/cpe4700/modules/
MOD_DIR ?= ../modules

CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -I$(MOD_DIR)
LDLIBS  = -lgpiod -lpthread -lm

SRCS = main.c $(MOD_DIR)/pwm.c $(MOD_DIR)/encoder.c $(MOD_DIR)/speed_control.c \
       $(MOD_DIR)/gamepad.c

lab06: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

# Prints every button press/release and axis value (no motors).
gamepad_test: gamepad_test.c $(MOD_DIR)/gamepad.c
	$(CC) $(CFLAGS) -o $@ gamepad_test.c $(MOD_DIR)/gamepad.c -lpthread

# Earlier open-loop stick drive (no encoder, no logging). Not the Lab 06 program.
gamepad_drive: gamepad_drive.c $(MOD_DIR)/gamepad.c $(MOD_DIR)/pwm.c
	$(CC) $(CFLAGS) -o $@ gamepad_drive.c $(MOD_DIR)/gamepad.c $(MOD_DIR)/pwm.c $(LDLIBS)

clean:
	rm -f lab06 gamepad_test gamepad_drive
.PHONY: clean
