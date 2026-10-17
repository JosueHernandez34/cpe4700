# Shared module build. Compiles each module to an object file so you can
# confirm they build from modules/ (Lab 04 and Lab 05 compile the sources
# directly via their own Makefiles).
CC     = gcc
CFLAGS = -Wall -Wextra -O2 -I.

OBJS = pwm.o encoder.o speed_control.o

all: $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o
.PHONY: all clean
