/*
 * balance_loop_v2.c -- Lab 05 balance controller, version 2 (copy of balance_loop.c, which is untouched).
 *
 * v2 CHANGES (see balance_loop_v2.h for the list): cutoff and standing band set live from the GUI,
 * block-rest trip with a 5 s lift window, four extra log columns plus a header comment line.
 * Everything else (control laws, Run ID sequence, timing) is the same as v1.
 *
 * PURPOSE
 *   Every 1 ms: read pitch from the IMU, compute a motor duty with a
 *   state-feedback law, and send it to both motors.
 *
 * STATE-FEEDBACK FORM (first version, no robot parameters needed yet)
 *   State vector      x = [ theta, theta_dot, integral(theta) ]
 *                         pitch (deg), pitch rate (deg/s), pitch integral (deg*s)
 *   Reference         x_ref = [ target, 0, 0 ]   (stay upright and still)
 *   Gain row          K = [ Kp, Kd, Ki ]         (the three GUI gains)
 *   Control law       u = DRIVE_SIGN * K * (x - x_ref)
 *   Output            duty = clamp(u, +/-BALANCE_DUTY_LIMIT)
 *   Leaning forward (positive pitch) gives positive u, so the wheels drive
 *   forward to get back under the robot. If the wheels run AWAY from the
 *   lean on the real robot, flip DRIVE_SIGN.
 *   Later this becomes the full model-based design: x gets the wheel speed,
 *   and K comes from pole placement / LQR instead of hand tuning.
 *
 * UNITS / SIGNS: see balance_loop.h (forward lean = positive pitch, 0 = upright).
 *
 * TWO CONTROL PATHS (decided once at startup by state_space.c)
 *   STATE FEEDBACK  only when every model coefficient is SET (ss_params.py) and
 *                   the closed-loop poles are inside the unit circle:
 *                       duty = K . (x - xref),   x = [pitch, pitch rate, wheel speed]
 *                   K comes from ss_model.h (made by ss_design.py).
 *   PID             otherwise (or REFUSED, see SS_FALLBACK_TO_PID): the PID law below.
 *   The GUI shows which one is active and why.
 *
 * HOW THE THREE GUI SLIDERS MAP ONTO THE STATE-FEEDBACK GAINS
 *   In state-feedback mode the sliders are MULTIPLIERS on the designed gains
 *   (1.00 = exactly the designed K, so you can trim +/-):
 *       Kp slider  ->  multiplies K_theta  (pitch gain,        duty/deg)
 *       Kd slider  ->  multiplies K_rate   (pitch-rate gain,   duty/(deg/s))
 *       Ki slider  ->  multiplies K_omega  (wheel-speed gain,  duty/(count/s))
 *   i.e.  duty = (Kp*K_theta)*(theta - ref) + (Kd*K_rate)*theta_dot + (Ki*K_omega)*(omega - 0).
 *   Kp and Kd keep their PID meaning (angle and its rate). Ki is the slow term:
 *   in the PID it integrated pitch, and an integral of pitch is a wheel velocity,
 *   so here it scales the wheel-speed feedback, the slow state of the model.
 *   Because K_omega is about 1e-4 duty per count/s, a direct slider in those
 *   units would be unusable; a multiplier keeps all three sliders at 0..2.
 *   Change the mapping by editing the "trim[]" lines in balance_loop_step().
 *
 * ESTIMATOR NOTE: the design assumes the true theta_dot and omega. The robot
 * only has them from a 10 ms difference + low-pass (RATE_FILTER, OMEGA_FILTER),
 * which adds a little lag the model does not include. If the real robot is
 * worse than the model predicts, that lag is the first suspect; an observer
 * built from Ad/Bd would remove it.
 *
 * HOW IT CONNECTS
 *   imu.c  -> get_imu_pitch()        raw BNO055 pitch (degrees)
 *   encoder.c -> enc_get_speed()     wheel speeds, only reported to the GUI
 *   pwm.c  -> pwm_set_duty()         motor command
 *   gui.c  -> set_gains / set_enabled / get_state / get_status
 *   robotest.c calls balance_loop_step() once per 1 ms from its worker thread.
 */

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <time.h>

#include "balance_loop_v2.h"
#include "imu.h"
#include "encoder.h"
#include "pwm.h"
#include "state_space.h"      /* ss_init(), ss_mode(), ss_control() */

/* ---------------- Constants (all in one block) ---------------- */

#define DT_S 0.001                   /* s: loop period, matches the 1 ms tick */

/* Motor 1 is usually mounted mirrored; flip to -1.0 if the wheels fight.
 * (pwm.c already inverts wheel 0 so +duty is the same direction on both.) */
#define MOTOR0_SIGN  1.0
#define MOTOR1_SIGN  1.0

/* +1: forward lean -> positive duty. Flip to -1.0 if the wheels drive away
 * from the lean when you test it. */
#define DRIVE_SIGN   1.0

/* The BNO055 reads about +/-180 deg when the robot is upright (it is mounted
 * so "upright" sits at the wrap-around point of its pitch range). Corrected
 * pitch = wrap(180 - raw): raw +179 -> +1 deg (forward), raw -179 -> -1 deg
 * (back). A failed IMU read returns raw 0.0, which becomes 180 deg and
 * trips the cutoff, so a dead IMU stops the motors. */
#define PITCH_ZERO_DEG 180.0         /* deg: raw reading when upright */

/* The BNO055 updates its fusion output about every 10 ms, so a rate computed
 * between 1 ms samples would be mostly 0. Measure over 10 ms instead. */
#define RATE_WINDOW_STEPS 10         /* 1 ms steps between rate updates (= 10 ms) */
#define RATE_FILTER       0.3        /* 0..1: weight of each new rate sample (smaller = smoother, slower) */
#define OMEGA_FILTER      0.3        /* 0..1: same for the wheel speed; counts arrive 1 count = 100 counts/s per
                                      * 10 ms, so a little smoothing is needed */

/* State-feedback path only (the PID path above is unchanged). The BNO055 puts
 * out a new angle every ~10 ms, but its timing is not tied to this 1 ms loop,
 * so on a fixed 10 ms clock the controller can be working with an angle that is
 * up to 9 ms old. In a simulation of the unstable robot that alone turned
 * +/-0.4 deg into +/-27 deg. So the state-feedback update runs when a NEW IMU
 * value appears (the angle changed), and the rate and wheel speed are
 * differences over the real time since the previous new value. */
#define SS_RATE_FILTER    1.0        /* 0..1: 1.0 = no smoothing of the pitch rate (smoothing is lag, and
                                      * the unstable pole leaves little room for lag) */
#define SS_OMEGA_FILTER   0.5        /* 0..1: wheel speed is a slow state, so a little smoothing is cheap */
#define SS_MAX_SAMPLE_MS  15         /* ms: if the angle has not changed for this long, update anyway */

/* ---- System identification ("Run ID") -----------------------------------
 * While the robot is balancing on the floor with your gains, this adds a
 * pseudo-random sequence of duty steps ON TOP of the controller output. The
 * extra duty is known and independent of the sensor noise, so the logged
 * response (pitch, wheel counts) lets identify.py fit a model of the robot
 * while it is under control. The sequence comes from a 9-bit shift register
 * (same pattern every run) and steps between four levels: +/-A and +/-0.6*A. */
#define ID_DURATION_MS  20000        /* ms: length of one identification run */
#define ID_BIT_MS       150          /* ms: how long each level is held (about 3 Hz bandwidth) */
#define ID_LIMIT_FRACTION  0.6       /* stop injecting once pitch passes 60% of the cutoff on either side
                                      * (about 12.6 deg with a 25 deg stop), so the steps never push the robot
                                      * toward the block and the controller can recover on its own */
#define ID_LOW_LEVEL    0.6          /* the smaller step is 0.6 * amplitude */
#define ID_LFSR_SEED    0x1FF        /* nonzero start value of the shift register */

/* Run log: one row per 1 ms step while the motors are enabled, kept in memory
 * during the run and written to a CSV when the loop ends (file I/O inside the
 * 1 ms loop could make it miss its deadline). 120000 rows = 2 minutes. */
#define LOG_MAX 120000
typedef struct {
    float t_ms, pitch, rate, duty, rpm0, rpm1;   /* ms, deg, deg/s, -1..1, edges/s, edges/s */
    float inj, count0, count1;                   /* injected duty (ID), wheel encoder counts */
    float cut_fwd, cut_back, band_lo, band_hi;   /* v2: limits in force on this row, deg, so a change
                                                  * in the middle of a run is recorded */
    float stop_fwd, stop_back;                   /* v2: block stop angles in force (signed pitch), deg */
    float pitch_zero;                            /* v2: IMU reading at balance, already taken off pitch */
} log_row_t;
static log_row_t log_buf[LOG_MAX];
static atomic_uint log_n;                        /* rows stored so far (atomic: the GUI thread reads it to
                                                  * write a snapshot while the balance thread keeps logging) */

/* Live ID statistics for the GUI. "Near a block" = within NEAR_BLOCK_DEG of a stop angle and moving slower than
 * NEAR_BLOCK_RATE: the robot is resting against the block, where the model does not apply (same test as
 * identify.py, so the GUI warning and the fit's verdict agree). */
#define NEAR_BLOCK_DEG   2.5         /* deg */
#define NEAR_BLOCK_RATE  5.0         /* deg/s */
static atomic_uint g_id_log_start;               /* log row where the current ID run began */
static atomic_uint g_id_total, g_id_near_block;  /* samples during the ID run, and how many were near a block */
static _Atomic double g_id_inj;                  /* duty step being injected right now */

/* Shared with the GUI thread. */
static _Atomic double g_target;                  /* deg: target pitch, forward positive */
static _Atomic double g_duty_limit;              /* tuning duty limit, <= BALANCE_DUTY_LIMIT */
static _Atomic double g_kp, g_ki, g_kd;
static _Atomic double g_pitch, g_rpm0, g_rpm1;
static _Atomic double g_duty;                    /* last duty sent to the motors, for the GUI */
static _Atomic double g_id_amp = BALANCE_DEFAULT_ID_STEP;  /* duty: identification step size (GUI) */
static atomic_int g_id_request;                  /* 1 = GUI pressed "Run ID" */
static atomic_int g_id_left_ms;                  /* ms of identification left (0 = not running) */
static atomic_int g_enabled;         /* 1 when the GUI switch is ON */
static atomic_int g_tripped;         /* 1 when the cutoff has stopped the motors */

/* Controller state -- only touched by the balance thread. */
static double integral;              /* deg*s: running sum of pitch * DT_S */
static double pitch_at_window;       /* deg: pitch at the start of this rate window */
static double rate_filtered;         /* deg/s: low-pass filtered pitch rate */
static unsigned int window_count;    /* steps since the rate was last updated */
static double cnt_at_window;         /* counts: average wheel count at the start of this window */
static double omega_filtered;        /* counts/s: low-pass filtered average wheel speed */
/* v2: cutoff and band are written by the GUI thread and read every step, so they are atomic. */
static _Atomic double g_cut_fwd = BALANCE_CUTOFF_DEFAULT_DEG;   /* deg: cut the motors if pitch > this */
static _Atomic double g_cut_back = BALANCE_CUTOFF_DEFAULT_DEG;  /* deg: cut the motors if pitch < -this */
static _Atomic double g_band_lo, g_band_hi;     /* deg: standing band, logged only; set from the stops */
static _Atomic double g_pitch_zero;              /* deg: raw IMU pitch at the stops' midpoint, subtracted from pitch */
static _Atomic double g_stop_fwd, g_stop_back;  /* deg (signed pitch): block stops, from ss_model.h until
                                                 * measured or typed in the GUI */
static atomic_int g_trip_reason;     /* trip_reason_t: why the motors are latched off */
static atomic_int g_lift_left_ms;    /* ms left in the lift window, for the GUI */
static int prev_enabled;             /* switch position at the previous step (catches OFF -> ON) */
static int block_armed;              /* 1 once the robot has left the block: block-rest trip active */
static int block_rest_ms;            /* ms in a row spent resting against a block */
static double duty_hold;             /* duty: state-feedback output, held until the next new IMU value */
static double ss_last_raw;           /* deg: raw IMU angle at the previous step (detects a new IMU value) */
static unsigned int ss_ms;           /* ms since the last new IMU value */
static double ss_pitch_prev;         /* deg: pitch at the last new IMU value */
static double ss_cnt_prev;           /* counts: average wheel count at the last new IMU value */
static double ss_rate, ss_omega;     /* deg/s, counts/s: state-feedback estimates of theta_dot and omega */
static unsigned int id_lfsr;         /* shift register that makes the step pattern */
static int id_hold;                  /* ms left before the next level is picked */
static double id_level;              /* duty: level being injected right now */

static double clamp(double x, double lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

/* Wrap an angle into -180..+180 degrees. fmod() is used instead of a loop so
 * a garbage sensor value (huge, infinite or NaN) can never stall the 1 ms
 * loop; a non-finite angle is reported as 180 deg, which trips the cutoff. */
static double wrap180(double a)
{
    if (!isfinite(a))
        return 180.0;
    a = fmod(a, 360.0);                    /* now in -360..+360 */
    if (a >  180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
}

/* Calibrate the standing band from the stops: 1 deg inside each block (stops may be in any order).
 * If the stops are closer than 2 margins the band would be empty, so the band is left as it is. */
static void band_from_stops(void)
{
    double f = atomic_load(&g_stop_fwd), b = atomic_load(&g_stop_back);
    double lo = fmin(f, b) + BALANCE_BAND_STOP_MARGIN_DEG;
    double hi = fmax(f, b) - BALANCE_BAND_STOP_MARGIN_DEG;
    if (lo < hi) {
        atomic_store(&g_band_lo, lo);
        atomic_store(&g_band_hi, hi);
    }
}

void balance_loop_init(void)
{
    ss_init();                       /* verify the model, print poles, decide STATE FEEDBACK / PID / REFUSED */

    /* v2: the cutoff comes from the GUI sliders (default +/-15 deg). A cutoff wider than the block
     * can never trip while the robot rests on it, so the block-rest trip in balance_loop_step()
     * covers that case instead. */
    atomic_store(&g_cut_fwd, BALANCE_CUTOFF_DEFAULT_DEG);
    atomic_store(&g_cut_back, BALANCE_CUTOFF_DEFAULT_DEG);
    /* Stops measured on the ground (raw IMU frame); the zero is their midpoint, so both are +/- half the span. */
    double zero0 = 0.5 * (BALANCE_STOP_FWD_RAW_DEG + BALANCE_STOP_BACK_RAW_DEG);
    atomic_store(&g_pitch_zero, zero0);
    atomic_store(&g_stop_fwd,  BALANCE_STOP_FWD_RAW_DEG  - zero0);
    atomic_store(&g_stop_back, BALANCE_STOP_BACK_RAW_DEG - zero0);
    band_from_stops();                                 /* band = stops minus 1 deg: +/-6.405 at launch */
    atomic_store(&g_trip_reason, TRIP_NONE);
    atomic_store(&g_lift_left_ms, 0);
    atomic_store(&g_id_amp, BALANCE_DEFAULT_ID_STEP);
    prev_enabled = 0;
    block_armed = 0;
    block_rest_ms = 0;
    printf("balance_loop v2: cutoff +%.1f / -%.1f deg (GUI), band %+.1f / %+.1f deg, "
           "block stops %+.1f / %+.1f deg (%s, measure or type them in the GUI), lift window %d ms\n",
           BALANCE_CUTOFF_DEFAULT_DEG, BALANCE_CUTOFF_DEFAULT_DEG, atomic_load(&g_band_lo), atomic_load(&g_band_hi),
           atomic_load(&g_stop_fwd), atomic_load(&g_stop_back), "10/26 ground reading, zero at their midpoint",
           BALANCE_LIFT_WINDOW_MS);
    double g0, g1, g2;
    balance_loop_default_gains(&g0, &g1, &g2);
    balance_loop_set_gains(g0, g2, g1);          /* (kp, ki, kd) = (pitch, wheel-speed, rate) */
    atomic_store(&g_target, BALANCE_DEFAULT_TARGET);
    atomic_store(&g_duty_limit, BALANCE_DEFAULT_DUTY_LIMIT);
    atomic_store(&g_enabled, 0);     /* motors always start OFF */
    atomic_store(&g_tripped, 0);
    atomic_store(&log_n, 0);
    atomic_store(&g_id_total, 0);
    atomic_store(&g_id_near_block, 0);
    atomic_store(&g_id_inj, 0.0);
    atomic_store(&g_id_request, 0);
    atomic_store(&g_id_left_ms, 0);
    id_hold = 0;
    id_level = 0.0;
    integral = 0.0;
    pitch_at_window = 0.0;
    rate_filtered = 0.0;
    window_count = 0;
    cnt_at_window = 0.5 * (enc_get_count(0) + enc_get_count(1));
    omega_filtered = 0.0;
    duty_hold = 0.0;
    ss_last_raw = PITCH_ZERO_DEG;
    ss_ms = 0;
    ss_pitch_prev = 0.0;
    ss_cnt_prev = cnt_at_window;
    ss_rate = ss_omega = 0.0;
}

/* Slider values at launch and after "Reset Gains". PID: all 0 (nothing moves
 * until you raise one). State feedback: 1.0 = the designed gain. Order returned
 * is (Kp, Kd, Ki) = (pitch, rate, wheel-speed / integral). */
void balance_loop_default_gains(double *kp, double *kd, double *ki)
{
    if (ss_mode() == SS_MODE_STATE_FEEDBACK) {
        *kp = *kd = *ki = 1.0;
    } else {
        *kp = BALANCE_DEFAULT_KP;
        *kd = BALANCE_DEFAULT_KD;
        *ki = BALANCE_DEFAULT_KI;
    }
}

ss_mode_t balance_loop_control_mode(void)
{
    return ss_mode();
}

void balance_loop_control_text(char *buf, size_t n)
{
    switch (ss_mode()) {
    case SS_MODE_STATE_FEEDBACK:
        snprintf(buf, n, "Control: STATE FEEDBACK (%s). Sliders multiply the designed gains (1.00 = design).", ss_reason());
        break;
    case SS_MODE_PID:
        snprintf(buf, n, "Control: PID FALLBACK - %s", ss_reason());
        break;
    default:
        snprintf(buf, n, "Control: REFUSED, motors cannot be enabled - %s", ss_reason());
        break;
    }
}

void balance_loop_set_gains(double kp, double ki, double kd)
{
    atomic_store(&g_kp, kp);
    atomic_store(&g_ki, ki);
    atomic_store(&g_kd, kd);
}

void balance_loop_set_target(double target_deg)
{
    /* Keep the target inside half of the smaller cutoff, well inside the mechanical stop range. */
    atomic_store(&g_target, clamp(target_deg, 0.5 * fmin(atomic_load(&g_cut_fwd), atomic_load(&g_cut_back))));
}

void balance_loop_set_cutoff(double fwd, double back)
{
    /* Clamp each side into the slider range; never 0, so the cutoff cannot be switched off. */
    fwd  = fmin(BALANCE_CUTOFF_MAX_DEG, fmax(BALANCE_CUTOFF_MIN_DEG, fwd));
    back = fmin(BALANCE_CUTOFF_MAX_DEG, fmax(BALANCE_CUTOFF_MIN_DEG, back));
    atomic_store(&g_cut_fwd, fwd);
    atomic_store(&g_cut_back, back);
}

void balance_loop_get_cutoff(double *fwd, double *back)
{
    *fwd  = atomic_load(&g_cut_fwd);
    *back = atomic_load(&g_cut_back);
}

void balance_loop_set_band(double lo, double hi)
{
    lo = clamp(lo, BALANCE_BAND_LIMIT_DEG);
    hi = clamp(hi, BALANCE_BAND_LIMIT_DEG);
    if (lo >= hi)
        return;                       /* low must stay below high: keep the previous band */
    atomic_store(&g_band_lo, lo);
    atomic_store(&g_band_hi, hi);
}

void balance_loop_get_band(double *lo, double *hi)
{
    *lo = atomic_load(&g_band_lo);
    *hi = atomic_load(&g_band_hi);
}

void balance_loop_get_stops(double *fwd, double *back)
{
    *fwd  = atomic_load(&g_stop_fwd);
    *back = atomic_load(&g_stop_back);
}

void balance_loop_set_pitch_zero(double zero_deg)
{
    if (!isfinite(zero_deg))
        return;
    double delta = zero_deg - atomic_load(&g_pitch_zero);
    atomic_store(&g_pitch_zero, zero_deg);
    /* The blocks did not move, so in the new frame their angles change by -delta. */
    atomic_store(&g_stop_fwd,  atomic_load(&g_stop_fwd)  - delta);
    atomic_store(&g_stop_back, atomic_load(&g_stop_back) - delta);
}

void balance_loop_center_zero(void)
{
    /* Raw IMU readings of the two stops, then put the zero halfway between them. */
    double z = atomic_load(&g_pitch_zero);
    double raw_f = atomic_load(&g_stop_fwd) + z;
    double raw_b = atomic_load(&g_stop_back) + z;
    balance_loop_set_pitch_zero(0.5 * (raw_f + raw_b));   /* shifts both stops to +/- half the span */
    band_from_stops();                                      /* band follows the new stops */
}

double balance_loop_get_pitch_zero(void)
{
    return atomic_load(&g_pitch_zero);
}

void balance_loop_set_stops(double fwd, double back)
{
    /* Any values, any time (Josue 10/26). A non-finite value is ignored so the zone test stays valid. */
    if (!isfinite(fwd) || !isfinite(back))
        return;
    atomic_store(&g_stop_fwd, fwd);
    atomic_store(&g_stop_back, back);
}

trip_reason_t balance_loop_get_trip_reason(void)
{
    return (trip_reason_t)atomic_load(&g_trip_reason);
}

int balance_loop_get_lift_left_ms(void)
{
    return atomic_load(&g_lift_left_ms);
}

void balance_loop_set_duty_limit(double limit)
{
    if (limit < 0.0) limit = 0.0;
    if (limit > BALANCE_DUTY_LIMIT) limit = BALANCE_DUTY_LIMIT;   /* handout limit */
    atomic_store(&g_duty_limit, limit);
}

void balance_loop_set_id_amplitude(double amp)
{
    if (amp < 0.0) amp = 0.0;
    if (amp > 0.20) amp = 0.20;           /* hard cap on the step size */
    atomic_store(&g_id_amp, amp);
}

void balance_loop_start_id(void)
{
    atomic_store(&g_id_request, 1);       /* picked up by the next step, only if the motors are running */
}

int balance_loop_get_id_left_ms(void)
{
    return atomic_load(&g_id_left_ms);
}

void balance_loop_set_enabled(int enabled)
{
    if (ss_mode() == SS_MODE_REFUSED)
        enabled = 0;                 /* model not ready and fallback is off: motors stay off */
    atomic_store(&g_enabled, enabled ? 1 : 0);
}

void balance_loop_step(unsigned int loop_count)
{
    /* --- Measurements --- */
    double raw_pitch = get_imu_pitch();                  /* deg, raw BNO055 */
    double pitch_zero = atomic_load(&g_pitch_zero);      /* deg: IMU reading at the balance point */
    double pitch = wrap180(PITCH_ZERO_DEG - raw_pitch) - pitch_zero;  /* deg, 0 = balanced, forward + */
    double rpm0  = enc_get_speed(0);   /* edges/s, signed (GUI label says RPM; unit fix pending) */
    double rpm1  = enc_get_speed(1);

    /* --- Pitch rate: change in pitch over a 10 ms window, then smoothed --- */
    int tick = 0;                    /* 1 on steps where the state-feedback estimates were just refreshed */
    window_count++;
    if (window_count >= RATE_WINDOW_STEPS) {
        double raw_rate = (pitch - pitch_at_window)
                          / (RATE_WINDOW_STEPS * DT_S);  /* deg/s */
        rate_filtered += RATE_FILTER * (raw_rate - rate_filtered);
        pitch_at_window = pitch;

        /* Wheel speed: change in the average encoder count over the window. */
        double cnt_avg = 0.5 * (enc_get_count(0) + enc_get_count(1));   /* counts, both wheels */
        double raw_omega = (cnt_avg - cnt_at_window)
                           / (RATE_WINDOW_STEPS * DT_S);               /* counts/s */
        omega_filtered += OMEGA_FILTER * (raw_omega - omega_filtered);
        cnt_at_window = cnt_avg;

        window_count = 0;
    }

    /* --- State-feedback estimates: refreshed whenever a NEW IMU value arrives --- */
    tick = 0;
    ss_ms++;
    if (raw_pitch != ss_last_raw || ss_ms >= SS_MAX_SAMPLE_MS) {
        double dt = ss_ms * DT_S;                                       /* s since the previous new value */
        double cnt_now = 0.5 * (enc_get_count(0) + enc_get_count(1));   /* counts, both wheels averaged */
        ss_rate  += SS_RATE_FILTER  * ((pitch - ss_pitch_prev) / dt - ss_rate);     /* deg/s */
        ss_omega += SS_OMEGA_FILTER * ((cnt_now - ss_cnt_prev) / dt - ss_omega);    /* counts/s */
        ss_pitch_prev = pitch;
        ss_cnt_prev = cnt_now;
        ss_last_raw = raw_pitch;
        ss_ms = 0;
        tick = 1;                                                       /* time for a new duty */
    }

    /* --- Safety: cutoff latch, lift window and block-rest trip (v2) --- */
    double cut_fwd  = atomic_load(&g_cut_fwd);       /* deg: this step's cutoff, from the GUI sliders */
    double cut_back = atomic_load(&g_cut_back);
    /* "Near a block": within NEAR_BLOCK_DEG of a stop angle (same zone as the Run ID statistics). */
    double stop_fwd  = atomic_load(&g_stop_fwd);     /* deg, signed: block stops (measured, typed or ss_model.h) */
    double stop_back = atomic_load(&g_stop_back);
    double stop_hi = fmax(stop_fwd, stop_back);      /* any values are allowed, so sort them */
    double stop_lo = fmin(stop_fwd, stop_back);
    int near_block = (pitch >= stop_hi - NEAR_BLOCK_DEG || pitch <= stop_lo + NEAR_BLOCK_DEG);
    int enabled = atomic_load(&g_enabled);
    if (!enabled) {
        atomic_store(&g_tripped, 0);   /* switching OFF re-arms everything */
        atomic_store(&g_trip_reason, TRIP_NONE);
        block_armed = 0;
        block_rest_ms = 0;
        atomic_store(&g_lift_left_ms, 0);
    } else {
        if (!prev_enabled) {           /* Enable just went ON: open the lift window */
            atomic_store(&g_lift_left_ms, BALANCE_LIFT_WINDOW_MS);
            block_armed = 0;
            block_rest_ms = 0;
        }
        if (!atomic_load(&g_tripped)) {
            trip_reason_t why = TRIP_NONE;
            if (pitch > cut_fwd || pitch < -cut_back) {
                why = TRIP_CUTOFF;     /* too far over, or IMU dead (reads 180 deg): latch off */
            } else if (!block_armed) {
                /* Lift window: the block-rest trip is held off so the controller can lift the robot.
                 * It arms the first time pitch leaves the near-block zone. */
                int lift_left = atomic_load(&g_lift_left_ms);
                if (!near_block) {
                    block_armed = 1;
                    atomic_store(&g_lift_left_ms, 0);
                } else if (lift_left > 0) {
                    atomic_store(&g_lift_left_ms, lift_left - 1);
                } else {
                    why = TRIP_LIFT_FAILED;   /* still on the block after 5 s */
                }
            } else if (near_block && fabs(rate_filtered) < NEAR_BLOCK_RATE) {
                if (++block_rest_ms >= BALANCE_BLOCK_REST_MS)
                    why = TRIP_BLOCK_REST;    /* resting against a block for 1 s: stop pushing into it */
            } else {
                block_rest_ms = 0;
            }
            if (why != TRIP_NONE) {
                atomic_store(&g_trip_reason, why);
                atomic_store(&g_tripped, 1);
                atomic_store(&g_lift_left_ms, 0);
            }
        }
    }
    prev_enabled = enabled;
    int active = enabled && !atomic_load(&g_tripped);

    double duty_limit = atomic_load(&g_duty_limit);   /* tuning limit, <= 0.75 */
    double duty = 0.0;
    if (active && ss_mode() == SS_MODE_STATE_FEEDBACK) {
        /* STATE FEEDBACK: recomputed once per NEW IMU value (about every 10 ms, the
         * model's Ts) and held until the next one, as the model assumes. */
        if (tick) {
            double x[3]    = { pitch, ss_rate, ss_omega };              /* deg, deg/s, counts/s */
            double xref[3] = { atomic_load(&g_target), 0.0, 0.0 };      /* hold the target lean, zero rate, zero wheel speed */
            double trim[3] = { atomic_load(&g_kp),      /* Kp slider multiplies K_theta (pitch) */
                               atomic_load(&g_kd),      /* Kd slider multiplies K_rate  (pitch rate) */
                               atomic_load(&g_ki) };    /* Ki slider multiplies K_omega (wheel speed) */
            duty_hold = ss_control(x, xref, trim);      /* duty = K . (x - xref); no DRIVE_SIGN: K is in real duty */
        }
        duty = duty_hold;
        integral = 0.0;                                 /* the PID integral is not used here */
    } else if (active) {
        double kp = atomic_load(&g_kp);   /* duty per deg */
        double ki = atomic_load(&g_ki);   /* duty per (deg*s) */
        double kd = atomic_load(&g_kd);   /* duty per (deg/s) */

        /* State error x - x_ref. Target for the rate and integral is 0. */
        double e_theta = pitch - atomic_load(&g_target); /* deg (target 0 = upright) */
        double e_rate  = rate_filtered;                  /* deg/s */

        /* Integrate the pitch error, clamped so Ki * integral can never
         * exceed the duty limit on its own (anti-windup). */
        integral += e_theta * DT_S;
        if (ki > 0.0)
            integral = clamp(integral, duty_limit / ki);

        /* u = DRIVE_SIGN * K * (x - x_ref), with K = [Kp, Kd, Ki]. */
        duty = DRIVE_SIGN * (kp * e_theta + kd * e_rate + ki * integral);
    } else {
        integral = 0.0;                /* no stored error while stopped */
        duty_hold = 0.0;
    }

    /* --- Identification: add the known test steps on top of the controller --- */
    double inj = 0.0;                                  /* duty added by the ID run */
    int id_left = atomic_load(&g_id_left_ms);
    if (atomic_exchange(&g_id_request, 0) && active) {  /* new run requested and motors running */
        id_left = ID_DURATION_MS;
        atomic_store(&g_id_log_start, atomic_load(&log_n));   /* the snapshot for the live fit starts here */
        atomic_store(&g_id_total, 0);
        atomic_store(&g_id_near_block, 0);
        id_lfsr = ID_LFSR_SEED;
        id_hold = 0;                                    /* pick the first level right away */
    }
    if (id_left > 0) {
        if (!active) {
            id_left = 0;                                /* motors switched off or cutoff: abort */
        } else {
            if (pitch <= ID_LIMIT_FRACTION * cut_fwd && pitch >= -ID_LIMIT_FRACTION * cut_back) {  /* only inject well inside the stop */
                if (id_hold <= 0) {
                    /* Advance the shift register (taps 9 and 5, period 511 bits). */
                    unsigned int bit = ((id_lfsr >> 8) ^ (id_lfsr >> 4)) & 1u;
                    id_lfsr = ((id_lfsr << 1) | bit) & 0x1FFu;
                    double mag = ((id_lfsr >> 3) & 1u) ? 1.0 : ID_LOW_LEVEL;  /* big or small step */
                    id_level = ((id_lfsr & 1u) ? 1.0 : -1.0) * mag * atomic_load(&g_id_amp);
                    id_hold = ID_BIT_MS;
                }
                id_hold--;
                inj = id_level;
            }
            id_left--;
            atomic_fetch_add(&g_id_total, 1);
            if (near_block && fabs(rate_filtered) < NEAR_BLOCK_RATE)
                atomic_fetch_add(&g_id_near_block, 1);      /* resting against a block */
        }
        atomic_store(&g_id_left_ms, id_left);
    }
    atomic_store(&g_id_inj, inj);
    duty += inj;                                       /* total command = controller + test steps */

    /* Tuning limit (never above the handout's 0.75) before pwm_set_duty(). */
    duty = clamp(duty, duty_limit);

    pwm_set_duty(0, (float)(MOTOR0_SIGN * duty));
    pwm_set_duty(1, (float)(MOTOR1_SIGN * duty));

    /* Log this step (only while enabled, until the buffer is full). */
    unsigned int n_log = atomic_load(&log_n);          /* only this thread writes log_n */
    if (enabled && n_log < LOG_MAX) {
        log_buf[n_log] = (log_row_t){ (float)loop_count, (float)pitch,
            (float)rate_filtered, (float)duty, (float)rpm0, (float)rpm1,
            (float)inj, (float)enc_get_count(0), (float)enc_get_count(1),
            (float)cut_fwd, (float)cut_back, (float)atomic_load(&g_band_lo), (float)atomic_load(&g_band_hi),
            (float)stop_fwd, (float)stop_back, (float)pitch_zero };
        atomic_store(&log_n, n_log + 1);               /* publish the row only after it is complete */
    }

    /* --- Telemetry for the GUI --- */
    atomic_store(&g_duty, duty);
    atomic_store(&g_pitch, pitch);
    atomic_store(&g_rpm0, rpm0);
    atomic_store(&g_rpm1, rpm1);
}

void balance_loop_get_state(double *pitch, double *rpm0, double *rpm1)
{
    *pitch = atomic_load(&g_pitch);
    *rpm0  = atomic_load(&g_rpm0);
    *rpm1  = atomic_load(&g_rpm1);
}

double balance_loop_get_duty(void)
{
    return atomic_load(&g_duty);
}

/* Write log rows [start, end) as CSV. v2: the first line is a comment with the limits in force at
 * the first row (identify_v2.py skips lines starting with '#'), and every row ends with the four
 * limit columns, so a slider change in the middle of a run is still recorded. */
static void write_log_rows(FILE *f, unsigned int start, unsigned int end)
{
    const log_row_t *r0 = &log_buf[start];
    fprintf(f, "# balance_loop v2: cut_fwd=%.1f cut_back=%.1f band_lo=%.1f band_hi=%.1f stop_fwd=%+.2f stop_back=%+.2f (signed) "
               "pitch_zero=%.2f lift_window_ms=%d block_rest_ms=%d max_duty=%.2f id_step=%.3f\n",
            r0->cut_fwd, r0->cut_back, r0->band_lo, r0->band_hi, r0->stop_fwd, r0->stop_back,
            r0->pitch_zero, BALANCE_LIFT_WINDOW_MS, BALANCE_BLOCK_REST_MS, atomic_load(&g_duty_limit), atomic_load(&g_id_amp));
    fprintf(f, "time_ms,pitch_deg,rate_dps,duty,speed_0,speed_1,inj_duty,count_0,count_1,"
               "cut_fwd,cut_back,band_lo,band_hi,stop_fwd,stop_back,pitch_zero\n");
    for (unsigned int i = start; i < end; i++)
        fprintf(f, "%.0f,%.3f,%.2f,%.4f,%.1f,%.1f,%.4f,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n", log_buf[i].t_ms,
                log_buf[i].pitch, log_buf[i].rate, log_buf[i].duty,
                log_buf[i].rpm0, log_buf[i].rpm1,
                log_buf[i].inj, log_buf[i].count0, log_buf[i].count1,
                log_buf[i].cut_fwd, log_buf[i].cut_back, log_buf[i].band_lo, log_buf[i].band_hi,
                log_buf[i].stop_fwd, log_buf[i].stop_back, log_buf[i].pitch_zero);
}

void balance_loop_get_id_stats(int *left_ms, double *inj, unsigned int *total, unsigned int *near_block)
{
    *left_ms = atomic_load(&g_id_left_ms);
    *inj = atomic_load(&g_id_inj);
    *total = atomic_load(&g_id_total);
    *near_block = atomic_load(&g_id_near_block);
}

/* Copy the log from just before the current ID run to now, for the live fit. Runs in the GUI thread:
 * rows below log_n are complete, so it never reads a half-written row. Returns the rows written, or -1. */
int balance_loop_write_snapshot(const char *path)
{
    unsigned int end = atomic_load(&log_n);
    unsigned int start = atomic_load(&g_id_log_start);
    start = start > 500 ? start - 500 : 0;             /* half a second of context before the first step */
    if (end <= start)
        return -1;
    FILE *f = fopen(path, "w");
    if (!f)
        return -1;
    write_log_rows(f, start, end);
    fclose(f);
    return (int)(end - start);
}

void balance_loop_get_status(int *enabled, int *tripped)
{
    *enabled = atomic_load(&g_enabled);
    *tripped = atomic_load(&g_tripped);
}

void balance_loop_cleanup(void)
{
    pwm_set_duty(0, 0.0f);           /* motors off first */
    pwm_set_duty(1, 0.0f);

    /* Write the run log. The timestamp in the name means no earlier log is
     * ever overwritten. Skipped if the motors were never enabled. */
    unsigned int n_rows = atomic_load(&log_n);
    if (n_rows > 0) {
        char name[64];
        time_t now = time(NULL);
        strftime(name, sizeof name, "balance_log_%H%M%S.csv", localtime(&now));
        FILE *f = fopen(name, "w");
        if (f) {
            write_log_rows(f, 0, n_rows);
            fclose(f);
            fprintf(stderr, "balance log: %s (%u rows)\n", name, n_rows);
        }
        atomic_store(&log_n, 0);
    }
}
