# Lab 04 build. Shared modules live in ~/dev/cpe4700/modules/
MOD_DIR ?= ../modules

CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -I$(MOD_DIR)
LDLIBS  = -lgpiod -lpthread -lm

SRCS = main.c $(MOD_DIR)/pwm.c $(MOD_DIR)/encoder.c $(MOD_DIR)/speed_control.c

lab04: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

clean:
	rm -f lab04
.PHONY: clean
