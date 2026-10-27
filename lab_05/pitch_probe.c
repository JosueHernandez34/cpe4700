/*
 * pitch_probe.c -- measure the IMU on the real robot. NO MOTORS are touched.
 *
 * PURPOSE
 *   Answers four questions that the controller needs and that nobody has measured yet:
 *     1. What does the corrected pitch read when the robot is held upright (zero check)?
 *     2. What does it read when the robot rests on the mechanical block leaning FORWARD,
 *        and leaning BACK?  These are pitch_stop_fwd / pitch_stop_back for ss_params.py.
 *        They also settle whether the "about 25 degrees" is from vertical (reads about 25)
 *        or from the ground (reads about 65).
 *     3. How much does the reading scatter at rest? (guides the cutoff margin)
 *     4. Which BNO055 gyro axis is the pitch rate, and with what sign? (for
 *        IMU_PITCH_RATE_AXIS / IMU_PITCH_RATE_SIGN in imu.h)
 *
 * HOW TO RUN (from lab_05/, robot powered, wheels free of the table edge, motors not running):
 *     make pitch_probe
 *     ./pitch_probe
 *   It runs four phases. Each phase gives a 5 s countdown, then samples for 10 s at about
 *   100 Hz. Do what the screen says during each phase:
 *     1 HOLD UPRIGHT      hold the robot still at the balance point
 *     2 REST FORWARD      let it rest on the block leaning forward (do not touch it)
 *     3 REST BACK         let it rest on the block leaning back (do not touch it)
 *     4 ROCK BY HAND      slowly rock it forward and back about 10-20 degrees, a few times
 *   Options:  --countdown SECONDS   --seconds SECONDS   (shorter phases, for testing)
 *   Every sample is also saved to pitch_probe_HHMMSS.csv (a new file each run).
 *
 * UNITS / SIGNS
 *   pitch   degrees, corrected like balance_loop.c: 0 upright, FORWARD lean POSITIVE.
 *           corrected = wrap(180 - raw BNO055 pitch); the raw reading is about +/-180 upright.
 *   gyro    degrees per second about the sensor X, Y, Z axes.
 *
 * HOW IT FINDS THE PITCH-RATE AXIS
 *   In phase 4 the pitch rate is computed from the pitch itself (change over 50 ms). Each gyro
 *   axis is correlated with that rate. The axis with the largest |correlation| is the pitch
 *   axis; the sign of the correlation is the sign to use; the regression slope should be close
 *   to +/-1 (deg/s of gyro per deg/s of pitch), which checks the units.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "imu.h"

#define SAMPLE_MS      10          /* ms between samples: about the BNO055 update rate (100 Hz) */
#define MAX_SAMPLES    4000        /* per phase: enough for 40 s at 100 Hz */
#define PITCH_ZERO_DEG 180.0       /* raw reading when upright; same constant as balance_loop.c */

typedef struct { double pitch, gx, gy, gz; } sample_t;

static sample_t buf[4][MAX_SAMPLES];
static int      n_buf[4];

static const char *phase_name[4] = { "HOLD UPRIGHT", "REST FORWARD on the block",
                                     "REST BACK on the block", "ROCK BY HAND" };
static const char *phase_help[4] = {
    "Hold the robot still, upright at the balance point.",
    "Let the robot rest on the block leaning FORWARD. Do not touch it.",
    "Let the robot rest on the block leaning BACK. Do not touch it.",
    "Slowly rock the robot forward and back by about 10-20 degrees, a few times." };

/* Wrap an angle into -180..+180 degrees. */
static double wrap180(double a)
{
    if (!isfinite(a)) return 180.0;
    a = fmod(a, 360.0);
    if (a >  180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
}

static void sleep_ms(long ms)
{
    struct timespec t = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

static void stats(const sample_t *s, int n, double *mean, double *sd, double *mn, double *mx)
{
    double sum = 0, sq = 0;
    *mn = 1e9; *mx = -1e9;
    for (int i = 0; i < n; i++) {
        sum += s[i].pitch;
        if (s[i].pitch < *mn) *mn = s[i].pitch;
        if (s[i].pitch > *mx) *mx = s[i].pitch;
    }
    *mean = n ? sum / n : 0;
    for (int i = 0; i < n; i++) sq += (s[i].pitch - *mean) * (s[i].pitch - *mean);
    *sd = n > 1 ? sqrt(sq / (n - 1)) : 0;
}

/* Correlation and regression slope of y on x. */
static void corr_slope(const double *x, const double *y, int n, double *r, double *slope)
{
    double mx = 0, my = 0, sxx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
    mx /= n; my /= n;
    for (int i = 0; i < n; i++) {
        sxx += (x[i] - mx) * (x[i] - mx);
        syy += (y[i] - my) * (y[i] - my);
        sxy += (x[i] - mx) * (y[i] - my);
    }
    *r = (sxx > 0 && syy > 0) ? sxy / sqrt(sxx * syy) : 0.0;
    *slope = sxx > 0 ? sxy / sxx : 0.0;
}

int main(int argc, char **argv)
{
    int countdown_s = 5, seconds = 10;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!strcmp(argv[i], "--countdown")) countdown_s = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--seconds")) seconds = atoi(argv[i + 1]);
    }
    if (seconds * 1000 / SAMPLE_MS > MAX_SAMPLES) seconds = MAX_SAMPLES * SAMPLE_MS / 1000;

    if (imu_init() != 0) {          /* writes the BNO055 into NDOF mode, like every lab run */
        fprintf(stderr, "IMU init failed\n");
        return 1;
    }

    char csv_name[64];
    time_t now = time(NULL);
    strftime(csv_name, sizeof csv_name, "pitch_probe_%H%M%S.csv", localtime(&now));
    FILE *csv = fopen(csv_name, "w");        /* a new, time-stamped file every run */
    if (csv) fprintf(csv, "phase,time_ms,pitch_deg,gyro_x_dps,gyro_y_dps,gyro_z_dps\n");

    for (int ph = 0; ph < 4; ph++) {
        printf("\n=== Phase %d of 4: %s ===\n%s\n", ph + 1, phase_name[ph], phase_help[ph]);
        for (int c = countdown_s; c > 0; c--) { printf("  starting in %d ...\r", c); fflush(stdout); sleep_ms(1000); }
        printf("  SAMPLING for %d s ...            \n", seconds);
        fflush(stdout);

        int n = 0, total = seconds * 1000 / SAMPLE_MS;
        for (int i = 0; i < total && n < MAX_SAMPLES; i++) {
            double raw = get_imu_pitch();
            double gx = NAN, gy = NAN, gz = NAN;
            if (get_imu_gyro(&gx, &gy, &gz) != 0) { gx = gy = gz = NAN; }
            buf[ph][n].pitch = wrap180(PITCH_ZERO_DEG - raw);
            buf[ph][n].gx = gx; buf[ph][n].gy = gy; buf[ph][n].gz = gz;
            if (csv) fprintf(csv, "%d,%d,%.3f,%.3f,%.3f,%.3f\n", ph + 1, i * SAMPLE_MS,
                             buf[ph][n].pitch, gx, gy, gz);
            n++;
            sleep_ms(SAMPLE_MS);
        }
        n_buf[ph] = n;
    }
    if (csv) fclose(csv);
    imu_cleanup();

    /* ------------------------------ report ------------------------------ */
    printf("\n================ RESULTS ================\n");
    printf("Corrected pitch (0 upright, forward lean positive), degrees:\n");
    printf("  %-28s %9s %8s %8s %8s\n", "phase", "mean", "std", "min", "max");
    double mean[4], sd[4], mn[4], mx[4];
    for (int ph = 0; ph < 4; ph++) {
        stats(buf[ph], n_buf[ph], &mean[ph], &sd[ph], &mn[ph], &mx[ph]);
        printf("  %-28s %9.2f %8.3f %8.2f %8.2f\n", phase_name[ph], mean[ph], sd[ph], mn[ph], mx[ph]);
    }

    printf("\nGyro bias while held still (phase 1, deg/s):");
    double bx = 0, by = 0, bz = 0; int nb = 0;
    for (int i = 0; i < n_buf[0]; i++)
        if (isfinite(buf[0][i].gx)) { bx += buf[0][i].gx; by += buf[0][i].gy; bz += buf[0][i].gz; nb++; }
    if (nb) printf("  X %+.2f   Y %+.2f   Z %+.2f\n", bx / nb, by / nb, bz / nb);
    else    printf("  gyro reads failed\n");

    printf("\nMechanical stop (for ss_params.py):\n");
    printf("  pitch_stop_fwd  = %+.2f deg   (resting on the block leaning forward)\n", mean[1]);
    printf("  pitch_stop_back = %.2f deg    (magnitude; resting back reads %+.2f)\n", fabs(mean[2]), mean[2]);
    printf("  resting scatter: std %.3f deg forward, %.3f deg back (min-max span %.2f / %.2f deg)\n",
           sd[1], sd[2], mx[1] - mn[1], mx[2] - mn[2]);
    printf("  upright check: held-still mean %+.2f deg (should be about 0)\n", mean[0]);
    double span = fmax(mx[1] - mn[1], mx[2] - mn[2]);
    printf("  suggested cutoff margin: at least %.1f deg (5x the larger resting span, minimum 3)\n",
           fmax(3.0, 5.0 * span));
    if (fabs(mean[1]) > 45 || fabs(mean[2]) > 45)
        printf("  NOTE: a stop reading beyond 45 deg means the \"25 degrees\" was measured from the ground, or the "
               "robot was not on the block.\n");

    /* Pitch-rate axis from the rocking phase. */
    int n4 = n_buf[3];
    const int lag = 5;                               /* 50 ms difference for the pitch rate */
    int m = n4 - 2 * lag;
    if (m < 50) { printf("\nPhase 4 too short to find the gyro axis.\n"); return 0; }
    static double rate[MAX_SAMPLES], g[3][MAX_SAMPLES];
    for (int i = 0; i < m; i++) {
        int c = i + lag;                             /* centre sample */
        rate[i] = (buf[3][c + lag].pitch - buf[3][c - lag].pitch) / (2.0 * lag * SAMPLE_MS / 1000.0);
        g[0][i] = buf[3][c].gx; g[1][i] = buf[3][c].gy; g[2][i] = buf[3][c].gz;
    }
    printf("\nPitch-rate axis (phase 4, rocking): gyro axis vs the pitch's own rate\n");
    const char *an[3] = { "X", "Y", "Z" };
    int best = -1; double best_r = 0, best_slope = 0;
    for (int a = 0; a < 3; a++) {
        double r, slope;
        corr_slope(g[a], rate, m, &r, &slope);       /* pitch rate = slope * gyro */
        printf("  gyro %s: correlation %+.3f, pitch rate = %+.3f * gyro\n", an[a], r, slope);
        if (fabs(r) > fabs(best_r)) { best = a; best_r = r; best_slope = slope; }
    }
    double span4 = 0;
    for (int i = 0; i < m; i++) if (fabs(rate[i]) > span4) span4 = fabs(rate[i]);
    if (span4 < 15.0)
        printf("  NOT ENOUGH MOTION in phase 4 (peak pitch rate %.1f deg/s): rock it faster/further and repeat.\n", span4);
    else if (fabs(best_r) < 0.8)
        printf("  No axis correlates well (best |r| = %.2f). Rock about the pitch axis only and repeat.\n", fabs(best_r));
    else
        printf("  RESULT: pitch rate = %s%s gyro axis   ->  IMU_PITCH_RATE_AXIS %d, IMU_PITCH_RATE_SIGN %+d"
               "   (slope %+.2f; about +/-1 means the units agree)\n",
               best_r > 0 ? "+" : "-", an[best], best, best_r > 0 ? 1 : -1, best_slope);
    printf("\nSamples saved to %s\n", csv_name);
    return 0;
}
