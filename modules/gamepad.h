#ifndef GAMEPAD_H
#define GAMEPAD_H

// USB / Bluetooth game controller reader (Linux evdev, input_event API only).
//
// gp_init() starts one background thread that waits on the controller with
// poll() and updates atomic variables. The calling code (for example the 1 ms
// balance loop) only reads those atomics, so it never blocks and never waits
// on the controller. If the controller is unplugged the thread keeps trying
// to find it again every 0.5 s and the values drop back to zero.

// Axes after the deadzone and scaling are applied.
//   sticks   : -1.0 .. +1.0, 0.0 inside the deadzone. Stick up reads negative
//              (that is how the hardware reports it; negate in your code if
//              you want up = positive).
//   triggers :  0.0 .. +1.0
//   hat      : -1, 0 or +1 (d-pad)
// Which physical control is which is the usual evdev layout; confirm it with
// gamepad_test and adjust the names if this pad differs.
enum gp_axis {
    GP_AXIS_LX,     // ABS_X      left stick left/right
    GP_AXIS_LY,     // ABS_Y      left stick up/down
    GP_AXIS_RX,     // ABS_Z      right stick left/right
    GP_AXIS_RY,     // ABS_RZ     right stick up/down
    GP_AXIS_L2,     // ABS_BRAKE  left trigger
    GP_AXIS_R2,     // ABS_GAS    right trigger
    GP_AXIS_HAT_X,  // ABS_HAT0X  d-pad left/right
    GP_AXIS_HAT_Y,  // ABS_HAT0Y  d-pad up/down
    GP_AXIS_COUNT
};

// Fraction of full stick travel that is treated as zero (0.0 .. 1.0).
// Outside it the value is rescaled so it still reaches +/-1.0.
#define GP_DEADZONE 0.12

// Starts the reader thread. path is an event device such as
// "/dev/input/event11", or NULL to find the first device that has a gamepad
// button (BTN_SOUTH) and an X axis. Returns 0 on success, -1 if it was already
// started or the thread could not be created. A missing controller is not an
// error: gp_is_connected() stays 0 until one shows up.
int gp_init(const char *path);

// 1 while a controller is open and being read, 0 otherwise.
int gp_is_connected(void);

// Latest value of one axis (see enum gp_axis). Returns 0.0 for a bad index.
double gp_get_axis(int axis);

// 1 if the button is held down, 0 if not. code is the evdev key code from
// <linux/input.h>, for example BTN_SOUTH or BTN_START.
int gp_get_button(int code);

// Stops the thread and closes the device.
int gp_cleanup(void);

#endif
