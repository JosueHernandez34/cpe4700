/*
 * state_space.c -- startup verification and the state-feedback law.
 *
 * PURPOSE
 *   Decide, once at startup, whether the generated model may be used:
 *     1. every required coefficient is SET (SS_ALL_SET, from ss_model.h);
 *     2. the matrices and K are finite numbers;
 *     3. the closed-loop poles of  x[k+1] = (Ad + Bd*K) x[k]  all have
 *        |pole| < 1 (stable), computed HERE, not trusted from the script;
 *     4. the pole size computed here agrees with the script's own value.
 *   If any check fails the mode is PID (or REFUSED, see SS_FALLBACK_TO_PID)
 *   and ss_reason() says which check failed, for the GUI.
 *
 * UNITS / SIGNS: see state_space.h. K is in the C convention: duty = K . (x - xref).
 *
 * HOW IT CONNECTS
 *   ss_model.h  <- written by ss_design.py (never edit by hand)
 *   balance_loop.c calls ss_init() from balance_loop_init() and ss_control()
 *   every 10 ms (the model's sample time) while in state-feedback mode.
 */

#include <complex.h>
#include <math.h>
#include <stdio.h>

#include "state_space.h"
#include "ss_model.h"                /* SS_AD, SS_BD, SS_K, SS_ALL_SET, SS_NOT_SET_LIST, ... */

static ss_mode_t g_mode = SS_MODE_PID;
static char      g_reason[200] = "model not initialised";
static double    g_max_pole = NAN;

/* Closed-loop matrix Acl = Ad + Bd * K  (Bd is 3x1, K is 1x3, so the product is 3x3).
 * With duty = K x, one sample later x[k+1] = Ad x + Bd (K x) = Acl x. */
static void closed_loop(double Acl[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Acl[i][j] = SS_AD[i][j] + SS_BD[i] * SS_K[j];
}

/* Roots of the 3x3 matrix's characteristic polynomial
 *     lambda^3 + c2*lambda^2 + c1*lambda + c0 = 0
 *   c2 = -trace,  c1 = sum of the three 2x2 principal minors,  c0 = -determinant.
 * Found with the Durand-Kerner iteration (simple, no libraries; 3 roots only). */
static void poles3(const double M[3][3], double complex root[3])
{
    double tr  = M[0][0] + M[1][1] + M[2][2];
    double m01 = M[0][0] * M[1][1] - M[0][1] * M[1][0];       /* principal minors */
    double m02 = M[0][0] * M[2][2] - M[0][2] * M[2][0];
    double m12 = M[1][1] * M[2][2] - M[1][2] * M[2][1];
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
               - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
               + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    double c2 = -tr, c1 = m01 + m02 + m12, c0 = -det;

    /* Start the roots on a spiral so they are all different, then iterate. */
    root[0] = 0.4 + 0.9 * I;
    root[1] = root[0] * root[0];
    root[2] = root[1] * root[0];
    for (int it = 0; it < 500; it++) {
        double change = 0.0;
        for (int i = 0; i < 3; i++) {
            double complex r = root[i];
            double complex num = ((r + c2) * r + c1) * r + c0;           /* polynomial value */
            double complex den = 1.0;                                   /* product of (r_i - r_j) */
            for (int j = 0; j < 3; j++)
                if (j != i) den *= (r - root[j]);
            double complex step = num / den;
            root[i] = r - step;
            change += cabs(step);
        }
        if (change < 1e-14) break;                                      /* converged */
    }
}

void ss_init(void)
{
    printf("state_space: model from ss_model.h, Ts = %g s, K = [%+.6g duty/deg, %+.6g duty/(deg/s), %+.6g duty/(count/s)]\n",
           (double)SS_TS_S, SS_K[0], SS_K[1], SS_K[2]);
    printf("state_space: K made by %s\n", SS_K_METHOD);

    int finite = 1;
    for (int i = 0; i < 3; i++) {
        finite &= isfinite(SS_BD[i]) && isfinite(SS_K[i]);
        for (int j = 0; j < 3; j++) finite &= isfinite(SS_AD[i][j]);
    }

    double complex p[3] = {0, 0, 0};
    g_max_pole = NAN;
    if (SS_ALL_SET && finite) {
        double Acl[3][3];
        closed_loop(Acl);
        poles3(Acl, p);
        g_max_pole = 0.0;
        for (int i = 0; i < 3; i++) {
            printf("state_space: closed-loop pole %d = %+.4f %+.4fi   |pole| = %.4f\n",
                   i, creal(p[i]), cimag(p[i]), cabs(p[i]));
            if (cabs(p[i]) > g_max_pole) g_max_pole = cabs(p[i]);
        }
    }

    /* Decide the mode. The first failing check is the reason shown on the GUI. */
    int ok = 0;
    if (!SS_ALL_SET)
        snprintf(g_reason, sizeof g_reason, "NOT SET: %s", SS_NOT_SET_LIST);
    else if (!finite)
        snprintf(g_reason, sizeof g_reason, "model contains non-finite numbers");
    else if (!(g_max_pole < 1.0))
        snprintf(g_reason, sizeof g_reason, "closed loop unstable (max |pole| = %.4f, must be < 1)", g_max_pole);
    else if (fabs(g_max_pole - SS_EXPECT_MAX_POLE) > 1e-3)
        snprintf(g_reason, sizeof g_reason, "poles here (%.4f) disagree with ss_design.py (%.4f): regenerate ss_model.h",
                 g_max_pole, (double)SS_EXPECT_MAX_POLE);
    else {
        ok = 1;
        snprintf(g_reason, sizeof g_reason, "stable, max |pole| = %.4f", g_max_pole);
    }

    g_mode = ok ? SS_MODE_STATE_FEEDBACK : (SS_FALLBACK_TO_PID ? SS_MODE_PID : SS_MODE_REFUSED);
    printf("state_space: stability check: %s\n", ok ? "PASSED" : "FAILED");
    printf("state_space: control mode = %s (%s)\n",
           g_mode == SS_MODE_STATE_FEEDBACK ? "STATE FEEDBACK"
           : g_mode == SS_MODE_PID ? "PID FALLBACK" : "REFUSED (motors cannot be enabled)", g_reason);
}

double ss_gain(int i)              { return (i >= 0 && i < 3) ? SS_K[i] : 0.0; }
double ss_stop_fwd_deg(void)       { return SS_STOP_FWD_DEG; }
double ss_stop_back_deg(void)      { return SS_STOP_BACK_DEG; }
int    ss_stop_is_set(void)        { return SS_STOP_SET; }
double ss_cutoff_margin_deg(void)  { return SS_CUTOFF_MARGIN_DEG; }

ss_mode_t   ss_mode(void)    { return g_mode; }
const char *ss_reason(void)  { return g_reason; }
double      ss_max_pole(void){ return g_max_pole; }

double ss_control(const double x[3], const double xref[3], const double trim[3])
{
    double u = 0.0;
    for (int i = 0; i < 3; i++)
        u += SS_K[i] * trim[i] * (x[i] - xref[i]);   /* K_i * (GUI multiplier) * state error */
    return u;
}
