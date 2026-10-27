# Lab 05 build. Shared modules live in ~/dev/cpe4700/modules/
# Needs libgtk-4-dev:  sudo apt install libgtk-4-dev
MOD_DIR ?= ../modules

GTK4_CFLAGS = $(shell pkg-config --cflags gtk4)
GTK4_LIBS   = $(shell pkg-config --libs gtk4)

CC     = gcc
CFLAGS = -Wall -Wextra -O3 -I$(MOD_DIR) $(GTK4_CFLAGS)
LIBS   = -lgpiod -lpthread -lm $(GTK4_LIBS)

SRCS = robotest.c balance_loop.c gui.c state_space.c \
       $(MOD_DIR)/pwm.c $(MOD_DIR)/encoder.c $(MOD_DIR)/imu.c $(MOD_DIR)/bno055.c

robotest: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LIBS)

# Measures the IMU on the real robot (no motors): stop angles, resting scatter, gyro pitch axis.
pitch_probe: pitch_probe.c $(MOD_DIR)/imu.c $(MOD_DIR)/bno055.c
	$(CC) -Wall -Wextra -O2 -I$(MOD_DIR) -o $@ pitch_probe.c $(MOD_DIR)/imu.c $(MOD_DIR)/bno055.c -lm

# v2: GUI cutoff and band sliders, block-rest trip, lift window (balance_loop_v2.c, gui_v2.c).
# robotest.c is shared: it only calls init/step/cleanup and gui_run, which v2 keeps unchanged.
SRCS_V2 = robotest.c balance_loop_v2.c gui_v2.c state_space.c \
          $(MOD_DIR)/pwm.c $(MOD_DIR)/encoder.c $(MOD_DIR)/imu.c $(MOD_DIR)/bno055.c

robotest_v2: $(SRCS_V2) balance_loop_v2.h
	$(CC) $(CFLAGS) -o $@ $(SRCS_V2) $(LIBS)

clean:
	rm -f robotest robotest_v2 pitch_probe
.PHONY: clean
