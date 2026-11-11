/* Checks the Lab 05 v2 angle rules: calibrated zero and cutoff. */
#include <math.h>
#include <stdio.h>
#include "balance_ctrl.h"
static int fails;
#define CHECK(c, m) do { if (!(c)) { printf("FAIL: %s\n", m); fails++; } else printf("ok:   %s\n", m); } while (0)
int main(void)
{
    double zero = PITCH_ZERO_DEFAULT_DEG;
    printf("zero %.3f raw, stops %+.3f / %+.3f from zero, cutoff +%.1f / -%.1f, theta0 %.1f\n", zero,
           STOP_FWD_RAW_DEG - zero, STOP_BACK_RAW_DEG - zero, (double)CUT_FWD_DEG, (double)CUT_BACK_DEG, THETA0_DEG);
    CHECK(fabs(zero - 3.845) < 1e-9, "default zero is the midpoint of the Lab 05 stop readings (3.845)");
    CHECK(fabs((STOP_FWD_RAW_DEG - zero) - 7.405) < 1e-9 && fabs((STOP_BACK_RAW_DEG - zero) + 7.405) < 1e-9,
          "stops sit at +/-7.405 from the zero");
    CHECK(CUT_FWD_DEG == 15.0 && CUT_BACK_DEG == 15.0, "cutoff is +/-15 from the zero");
    CHECK(THETA0_DEG < (STOP_FWD_RAW_DEG - zero) - NEAR_BLOCK_DEG, "theta0 is outside the near-block zone of the forward stop");
    printf("theta0 is %.2f deg from the forward stop\n", (STOP_FWD_RAW_DEG - zero) - THETA0_DEG);
    return fails != 0;
}
