#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

/*
 * robot_params.h -- ONE place for the measured values and settings of this robot.
 *
 * SHARED: lives in ~/dev/cpe4700/modules/ (next to pwm, encoder, imu, gamepad).
 * C code includes it. Python reads the same file through modules/robot_params.py, so no value is
 * typed twice. Every value says where it came from. If a test gives a better number, change it
 * HERE and add a line to ~/dev/cpe4700/TESTS.md.
 *
 * Lab 07 uses it now. The Lab 04 to 06 programs keep their own copies of these numbers (they are
 * submitted work and were not edited); the values below were checked against them, see TESTS.md.
 * New code should include this file instead of retyping a number.
 *
 * Names start with ROBOT_ so they cannot clash with a lab's own #defines. Keep each value on one
 * line as "#define ROBOT_NAME value": robot_params.py reads them that way.
 */

/* ======================= Motors and encoders (Lab 04 step response) ======================= */
#define ROBOT_MOTOR_GAIN_KM        6073.0   /* edges/s per unit duty; mean of W0/W1 fwd/rev 6100, 5904, 6139, 6147 */
#define ROBOT_MOTOR_TAU_S          0.040    /* s, first-order time constant (41 ms by 63.2 %, 27 ms by 10-90 % rise) */
#define ROBOT_DEADBAND_W0_POS      0.06     /* duty at which wheel 0 first moves, forward (Lab 04 deadband run) */
#define ROBOT_DEADBAND_W0_NEG      0.06     /* wheel 0, reverse */
#define ROBOT_DEADBAND_W1_POS      0.06     /* wheel 1, forward */
#define ROBOT_DEADBAND_W1_NEG      0.04     /* wheel 1, reverse */
#define ROBOT_DEADBAND_DUTY        0.06     /* value used for compensation (the larger of the four) */
#define ROBOT_DEADBAND_RAMP        0.02     /* duty: compensation ramps in over this so noise at zero does not chatter */
#define ROBOT_ENCODER_COUNTS_REV   700.0    /* counts per wheel revolution (handout; Lab 05 plots convert to RPM with it) */
#define ROBOT_SPEED_PI_KP          0.0001   /* Lab 04 speed controller: loop gain Km*Kp = 0.6 */
#define ROBOT_SPEED_PI_KI          0.0025   /* = Kp / tau */

/* ===================================== Wheels and PWM ===================================== */
#define ROBOT_LEFT_CH              0        /* PWM channel of the left wheel (channel 0 is inverted in pwm.c so +duty is forward on both) */
#define ROBOT_RIGHT_CH             1
#define ROBOT_FEEDBACK_ENC         0        /* encoder 0 = left wheel, the speed feedback in Labs 06 and 07 */
#define ROBOT_STEER_SIGN           1.0      /* +1: d-pad left turns left (confirmed Lab 06) */
#define ROBOT_MAX_DUTY             0.60     /* final clamp on both PWM commands (Labs 05 v2, 06, 07) */
#define ROBOT_HANDOUT_MAX_DUTY     0.75     /* handout's absolute limit */

/* ====================================== Pitch (IMU) ======================================= */
/* The BNO055 runs in NDOF fusion, so pitch is measured against gravity. pitch = wrap180(180 - raw).
 * Zero = the angle the robot balances at, NOT vertical: the robot is back-heavy. It is the midpoint
 * of the readings on the two block stops (Lab 05 v2 method), measured with "lab07 --cal" and saved
 * in lab_07/pitch_zero.txt. The values below are the defaults used when that file does not exist. */
#define ROBOT_PITCH_RAW_UPRIGHT_DEG 180.0   /* raw BNO055 pitch when upright */
#define ROBOT_STOP_FWD_RAW_DEG     12.8075  /* reading resting on the forward stop, 11/11 lab07 --cal (10/26: 11.25) */
#define ROBOT_STOP_BACK_RAW_DEG    -2.8125  /* reading resting on the back stop, 11/11 (10/26: -3.56) */
#define ROBOT_PITCH_ZERO_DEG       (0.5 * (ROBOT_STOP_FWD_RAW_DEG + ROBOT_STOP_BACK_RAW_DEG))   /* +4.9975 */
#define ROBOT_BALANCE_ANGLE_DEG    2.0      /* where to balance, measured from that zero; back-heavy (Josue 11/11) */
#define ROBOT_CUTOFF_FWD_DEG       15.0     /* cut the motors past this, from the zero (Lab 05 v2 default) */
#define ROBOT_CUTOFF_BACK_DEG      15.0
#define ROBOT_CUTOFF_MIN_DEG       2.0      /* allowed cutoff range (Lab 05 v2 sliders) */
#define ROBOT_CUTOFF_MAX_DEG       30.0
#define ROBOT_NEAR_BLOCK_DEG       2.5      /* "resting on a stop" zone (Lab 05 v2) */
#define ROBOT_NEAR_BLOCK_RATE      5.0      /* deg/s: slower than this near a stop counts as resting */
#define ROBOT_BLOCK_REST_MS        1000     /* ms of resting before the trip */
#define ROBOT_RATE_WINDOW_STEPS    10       /* pitch rate = change over 10 ms (the IMU updates at 100 Hz) */
#define ROBOT_RATE_FILTER          0.3      /* weight of each new rate sample (Lab 05) */
#define ROBOT_IMU_STALE_MS         200      /* a pitch exactly unchanged this long = IMU read failure (get_imu_pitch returns 0.0 on failure) */

/* ============================== Balance gains (Lab 05, PID on pitch) ============================== */
/* Only these held the robot in Lab 05 (21.9 s and 12.6 s holds at the 0.60 limit). */
#define ROBOT_KP                   0.015    /* duty per deg */
#define ROBOT_KI                   0.005    /* duty per (deg * s) */
#define ROBOT_KD                   0.005    /* duty per (deg / s) */
#define ROBOT_DRIVE_SIGN           -1.0     /* with e = target - pitch; forward lean must drive forward */

/* ================================== Timing (Labs 04 to 07) ================================== */
#define ROBOT_LOOP_PERIOD_S        0.001    /* control loop, clock_nanosleep on CLOCK_MONOTONIC */
#define ROBOT_LOG_PERIOD_MS        20       /* one CSV row every 20 ms (handouts) */

/* ================================ Gamepad (YUYIU Windchaser Plus) ================================ */
/* Codes recorded with gamepad_test on USB in Lab 06. Wireless is still being brought up. */
#define ROBOT_GP_DEADZONE          0.12     /* fraction of stick travel treated as zero */
#define ROBOT_GP_EXIT_BUTTON       0x13b    /* START, BTN_START */
#define ROBOT_GP_B_BUTTON          0x130    /* B, BTN_SOUTH */
#define ROBOT_GP_X_BUTTON          0x134    /* X, BTN_WEST */
/* d-pad is the hat axes ABS_HAT0X (0x10) and ABS_HAT0Y (0x11): -1 left/up, +1 right/down */

/* ============= Hardware map (REFERENCE ONLY: the real values are in the modules) ============= */
/* pwm.h:      gpiochip0, direction lines 106 (ch 0) and 113 (ch 1); PWM chips 2 and 3; 10 kHz (100000 ns)
 * encoder.c:  gpiochip0, enc 0 A=124 B=53, enc 1 A=52 B=51
 * imu.c:      BNO055 on /dev/i2c-7, address 0x28, NDOF mode
 * Jetson:     Orin Nano (jetson-emb-ctrl-005), kernel 5.15.148-tegra; Bluetooth hci0 F0:68:E3:48:C7:8C
 * Run the lab programs with sudo (sysfs PWM, GPIO, I2C). */

#endif
