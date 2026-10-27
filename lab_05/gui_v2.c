/*
 * gui_v2.c -- Lab 05 GTK4 GUI, version 2 (copy of gui.c, which is untouched).
 *
 * v2 CHANGES: every setting is a typed number box (no sliders), two per row, so the window stays
 * small: gains, target, max duty, ID step, motor cutoff (forward, back; default +/-15 deg) and the
 * standing band (low, high; default -8 / +9 deg). Run ID, Reset Gains and Quit sit next to the Enable
 * switch. "Measure forward / back stop" records the block angles with the motors OFF (tilt the robot
 * onto each block, press, hold still 0.5 s). A status line shows the values in force; the motor status
 * says WHY the motors stopped (cutoff, block rest, lift failed) and counts down the 5 s lift window;
 * launch defaults max duty 0.60 and ID step 0.05; the live fit runs identify_v2.py. Talks to
 * balance_loop_v2.c.
 */

#include <math.h>                   /* fabs() for the target-pitch note */
#include <stdio.h>                  /* snprintf() */
#include <string.h>
#include <unistd.h>                 /* getpid() for the temp file names */
#include <glib/gstdio.h>            /* g_remove() */
#include <gtk/gtk.h>

#include "gui.h"
#include "balance_loop_v2.h"
#include "state_space.h"      /* ss_gain(): the designed gains, for the live display */

#define GUI_PERIOD_MS 50             /* 20 Hz refresh */
#define GAIN_MAX      1.0            /* top of the slider; gains are duty per degree, so
                                      * Kp = 0.02 already gives full duty at ~37 deg */
#define GAIN_STEP     0.005          /* one click on the spin button / arrow key */

static void (*quit_cb)(void);

static GtkWidget *pitch_label;                    /* v2: pitch and both motor speeds on one line */
static GtkWidget *status_label;                   /* motors OFF / ON / CUTOFF */
static GtkWidget *id_label;                       /* live Run ID panel: progress, gains in use, live fit */

/* ---- Live fit during a Run ID ------------------------------------------------------------------
 * While Run ID is running, the log so far is saved to a temp file and "python3 identify.py --live"
 * fits it in the background (nice 19, one thread, so it cannot disturb the 1 ms balance loop for
 * long). The K values it finds are shown below and refreshed at 8, 13 and 18 s and at the end.
 * An early fit has little data, so its K values move around; they settle as the run goes on. */
#define FIT_AT_S_1 8
#define FIT_AT_S_2 13
#define FIT_AT_S_3 18
static const int fit_schedule_s[3] = { FIT_AT_S_1, FIT_AT_S_2, FIT_AT_S_3 };
static char   snap_path[256], fit_path[256];      /* temp files: the log snapshot, the fit result */
static char   fit_text[700] = "Live fit: starts 8 s into a Run ID (needs enough data).";
static int    fit_running;                        /* 1 while identify.py is running */
static int    fit_pending_final;                  /* the run ended while a fit was still running */
static int    fit_next;                           /* index of the next scheduled fit */
static int    fit_for_s;                          /* seconds of the run the running fit covers */
static int    prev_left_ms;                       /* previous ID time left, to catch the start and the end */
static unsigned int last_near, last_total;        /* near-block statistics at the end of the last ID run */
static GtkWidget *enable_switch;                  /* motor enable, OFF at launch */
static GtkWidget *target_note;                    /* reminder: target applies only on OFF -> ON */
static GtkAdjustment *adj_target;                 /* target pitch slider/spin (what you type) */
static double active_target = BALANCE_DEFAULT_TARGET;  /* deg: target the controller is actually using */
static GtkAdjustment *adj_kp, *adj_ki, *adj_kd;   /* shared by scale + spin */
static GtkWidget *cutoff_label;                   /* v2: limits in force (cutoff, band, block stops) */
static GtkAdjustment *adj_cut_fwd, *adj_cut_back;  /* v2: cutoff boxes, deg (magnitudes) */
static GtkAdjustment *adj_band_lo, *adj_band_hi;   /* v2: standing band boxes, deg (signed) */

/* v2: block stop measurement. The GUI averages the pitch it already reads every 50 ms over
 * MEAS_SAMPLES ticks (0.5 s) and rejects the result if the robot moved more than MEAS_MAX_SPREAD. */
#define MEAS_SAMPLES     10
#define MEAS_MAX_SPREAD  1.0          /* deg: max - min allowed while measuring */
static int    meas_side;              /* +1 measuring forward, -1 measuring back, 0 idle */
static int    meas_n;                 /* samples taken so far */
static double meas_sum, meas_min, meas_max;
static GtkAdjustment *adj_stop_fwd, *adj_stop_back;  /* v2: block stop boxes, deg (signed, any value) */
static const char *stop_src_fwd = "ground 10/26", *stop_src_back = "ground 10/26";   /* default / measured / typed */
static int    stop_set_by_measure;    /* 1 while measure_tick writes a box, so it is not marked "typed" */
static char   meas_text[200] = "";               /* v2: result of the last stop measurement */
static GtkWidget *meas_label;
/* Launch / "Reset Gains" slider values and slider top. PID: 0 and GAIN_MAX.
 * State feedback: the sliders are multipliers on the designed gains, so
 * 1.0 and 2.0 (see balance_loop.c for the mapping). Set in activate(). */
static double gain_def_kp = BALANCE_DEFAULT_KP, gain_def_kd = BALANCE_DEFAULT_KD, gain_def_ki = BALANCE_DEFAULT_KI;
static double gain_max = GAIN_MAX;
static GtkApplication *app;

/* Any gain widget changed -> push all three gains to the controller. */
static void on_gain_changed(GtkAdjustment *adj, gpointer data)
{
    (void)adj; (void)data;
    balance_loop_set_gains(gtk_adjustment_get_value(adj_kp),
                           gtk_adjustment_get_value(adj_ki),
                           gtk_adjustment_get_value(adj_kd));
}

/* Update the note under the sliders: always reminds you to cycle Enable, and
 * says so loudly when the slider differs from the target the robot is using. */
static void refresh_target_note(void)
{
    if (!target_note || !adj_target)
        return;
    double wanted = gtk_adjustment_get_value(adj_target);
    char buf[160];
    if (fabs(wanted - active_target) > 1e-6)
        snprintf(buf, sizeof buf,
                 "NOTE: target %+.1f deg NOT applied yet (robot uses %+.1f deg). "
                 "Switch Enable OFF, then ON to apply.", wanted, active_target);
    else
        buf[0] = '\0';                                 /* v2: nothing to say, keep the window short */
    gtk_label_set_text(GTK_LABEL(target_note), buf);
}

/* Target-pitch slider changed (degrees, positive = lean forward). The
 * controller is NOT told yet: the value is applied when Enable goes OFF -> ON,
 * so the target can't jump while the robot is balancing. */
static void on_target_changed(GtkAdjustment *adj, gpointer data)
{
    (void)adj; (void)data;
    refresh_target_note();
}

/* Max-duty slider changed (0..0.75, the tuning limit on motor power). */
static void on_duty_limit_changed(GtkAdjustment *adj, gpointer data)
{
    (void)data;
    balance_loop_set_duty_limit(gtk_adjustment_get_value(adj));
}

/* ID step-size slider changed (duty added on top of the controller). */
static void on_id_amp_changed(GtkAdjustment *adj, gpointer data)
{
    (void)data;
    balance_loop_set_id_amplitude(gtk_adjustment_get_value(adj));
}

/* v2: either cutoff slider changed -> send both sides (applied live, next 1 ms step).
 * Changing the cutoff never touches the band, and the other way round. */
static void on_cutoff_changed(GtkAdjustment *adj, gpointer data)
{
    (void)adj; (void)data;
    if (!adj_cut_fwd || !adj_cut_back)
        return;                                        /* still building the window */
    balance_loop_set_cutoff(gtk_adjustment_get_value(adj_cut_fwd), gtk_adjustment_get_value(adj_cut_back));
}

/* v2: either band slider changed -> send both edges. If low >= high the loop keeps the old band;
 * the status line shows the band actually in force. */
static void on_band_changed(GtkAdjustment *adj, gpointer data)
{
    (void)adj; (void)data;
    if (!adj_band_lo || !adj_band_hi)
        return;
    balance_loop_set_band(gtk_adjustment_get_value(adj_band_lo), gtk_adjustment_get_value(adj_band_hi));
}

/* Pull "key=value" out of the fit result text. Returns 1 and fills out[] if found. */
static int fit_value(const char *txt, const char *key, char *out, size_t n)
{
    char pat[48];
    snprintf(pat, sizeof pat, "%s=", key);
    const char *p = txt;
    while ((p = strstr(p, pat)) != NULL) {
        if (p == txt || p[-1] == '\n') {              /* only at the start of a line */
            p += strlen(pat);
            size_t i = 0;
            while (p[i] && p[i] != '\n' && i + 1 < n) { out[i] = p[i]; i++; }
            out[i] = '\0';
            return 1;
        }
        p++;
    }
    return 0;
}

static void start_fit(int at_s);

/* identify.py finished: turn its answer into the live-fit line. */
static void on_fit_done(GPid pid, gint status, gpointer data)
{
    (void)status; (void)data;
    g_spawn_close_pid(pid);
    fit_running = 0;

    gchar *txt = NULL;
    char st[32] = "", reason[500] = "", a[40], b[40], c[40], d[40], e[40];
    if (!g_file_get_contents(fit_path, &txt, NULL, NULL)) {
        snprintf(fit_text, sizeof fit_text, "Live fit (data to %d s): no answer from identify_v2.py", fit_for_s);
    } else {
        fit_value(txt, "status", st, sizeof st);
        fit_value(txt, "reason", reason, sizeof reason);
        if (strcmp(st, "OK") == 0 && fit_value(txt, "Ktheta", a, sizeof a) && fit_value(txt, "Krate", b, sizeof b)
            && fit_value(txt, "Komega", c, sizeof c) && fit_value(txt, "maxpole", d, sizeof d)
            && fit_value(txt, "pitch_rms", e, sizeof e))
            snprintf(fit_text, sizeof fit_text,
                     "Live fit (data to %d s): K_pitch %+.5f | K_rate %+.5f | K_wheel %+.7f | max pole %.3f | fit error %.2f deg",
                     fit_for_s, atof(a), atof(b), atof(c), atof(d), atof(e));
        else if (strcmp(st, "NOT_USABLE") == 0)
            snprintf(fit_text, sizeof fit_text, "Live fit (data to %d s): NOT USABLE, no K given. %.420s", fit_for_s, reason);
        else
            snprintf(fit_text, sizeof fit_text, "Live fit (data to %d s): not enough data yet. %.420s", fit_for_s, reason);
        g_free(txt);
    }
    if (fit_pending_final) {                           /* the run ended while this fit was busy: fit the full log */
        fit_pending_final = 0;
        start_fit(BALANCE_ID_DURATION_MS / 1000);
    }
}

/* Save the log so far and fit it in the background. Only one fit runs at a time. */
static void start_fit(int at_s)
{
    if (fit_running)
        return;
    if (!g_file_test("identify_v2.py", G_FILE_TEST_EXISTS)) {
        snprintf(fit_text, sizeof fit_text, "Live fit: identify_v2.py not found. Start robotest_v2 from the lab_05 folder.");
        return;
    }
    if (balance_loop_write_snapshot(snap_path) < 0) {
        snprintf(fit_text, sizeof fit_text, "Live fit: no log data yet");
        return;
    }
    g_remove(fit_path);                                /* so a stale answer is never shown */
    char *argv[] = { "nice", "-n", "19", "python3", "identify_v2.py", snap_path, "--live", fit_path, NULL };
    char **envp = g_get_environ();                     /* one thread for the numerics, so the fit stays light */
    envp = g_environ_setenv(envp, "OMP_NUM_THREADS", "1", TRUE);
    envp = g_environ_setenv(envp, "OPENBLAS_NUM_THREADS", "1", TRUE);
    envp = g_environ_setenv(envp, "MKL_NUM_THREADS", "1", TRUE);
    GPid pid;
    GError *err = NULL;
    gboolean ok = g_spawn_async(NULL, argv, envp,
                                G_SPAWN_SEARCH_PATH | G_SPAWN_DO_NOT_REAP_CHILD |
                                G_SPAWN_STDOUT_TO_DEV_NULL | G_SPAWN_STDERR_TO_DEV_NULL,
                                NULL, NULL, &pid, &err);
    g_strfreev(envp);
    if (!ok) {
        snprintf(fit_text, sizeof fit_text, "Live fit: could not start identify_v2.py (%s)", err ? err->message : "?");
        g_clear_error(&err);
        return;
    }
    fit_running = 1;
    fit_for_s = at_s;
    snprintf(fit_text, sizeof fit_text, "Live fit: fitting the first %d s of data ...", at_s);
    g_child_watch_add(pid, on_fit_done, NULL);
}

/* "Run ID" pressed. If the motors are OFF this turns them ON first (the same way the Enable switch does:
 * it latches the target pitch, and a refused or cut-off state still wins), then starts the 20 s run. */
static void on_run_id(GtkButton *btn, gpointer data)
{
    (void)btn; (void)data;
    int enabled, tripped;
    balance_loop_get_status(&enabled, &tripped);
    if (!enabled)
        gtk_switch_set_active(GTK_SWITCH(enable_switch), TRUE);   /* runs on_enable_toggled() */
    balance_loop_start_id();
}

static void recenter_zero(void);

/* v2: a stop box changed (data = +1 forward, -1 back): send both stops, any value. */
static void on_stop_changed(GtkAdjustment *adj, gpointer data)
{
    (void)adj;
    if (!adj_stop_fwd || !adj_stop_back)
        return;                                        /* still building the window */
    balance_loop_set_stops(gtk_adjustment_get_value(adj_stop_fwd), gtk_adjustment_get_value(adj_stop_back));
    if (!stop_set_by_measure) {
        if (GPOINTER_TO_INT(data) > 0) stop_src_fwd = "typed"; else stop_src_back = "typed";
        recenter_zero();                               /* zero = midpoint of the stops (Josue 10/26) */
    }
}

/* v2: after a stop changes, move the pitch zero to the midpoint of the two stops (the loop shifts
 * both stops with it) and show the new, symmetric stop values without marking them "typed". */
static void recenter_zero(void)
{
    balance_loop_center_zero();
    double f, b;
    balance_loop_get_stops(&f, &b);
    stop_set_by_measure = 1;
    gtk_adjustment_set_value(adj_stop_fwd, f);
    gtk_adjustment_set_value(adj_stop_back, b);
    stop_set_by_measure = 0;
    double lo, hi;                                     /* the loop re-calibrated the band: show it */
    balance_loop_get_band(&lo, &hi);
    gtk_adjustment_set_value(adj_band_lo, lo);
    gtk_adjustment_set_value(adj_band_hi, hi);
}

/* v2: "Measure fwd stop" (data = +1) or "Measure back stop" (data = -1). Starts a 0.5 s
 * average in update_display; the result goes into the matching stop box. */
static void on_measure_stop(GtkButton *btn, gpointer data)
{
    (void)btn;
    meas_side = GPOINTER_TO_INT(data);
    meas_n = 0;
    meas_sum = 0.0;
    meas_min = 1e9;
    meas_max = -1e9;
    snprintf(meas_text, sizeof meas_text, "Measuring the %s stop: hold the robot still on the block ...",
             meas_side > 0 ? "FORWARD" : "BACK");
}

/* v2: one measurement tick (called from update_display every 50 ms while measuring). */
static void measure_tick(double pitch)
{
    meas_sum += pitch;
    if (pitch < meas_min) meas_min = pitch;
    if (pitch > meas_max) meas_max = pitch;
    if (++meas_n < MEAS_SAMPLES)
        return;
    double mean = meas_sum / meas_n;              /* deg, signed */
    double spread = meas_max - meas_min;          /* deg: how much it moved during the 0.5 s */
    const char *side = meas_side > 0 ? "Fwd" : "Back";
    if (spread > MEAS_MAX_SPREAD) {
        /* the only check left: the robot must be held still, or the average means nothing */
        snprintf(meas_text, sizeof meas_text, "%s stop NOT set: moved %.1f deg. Hold still, press again.", side, spread);
    } else {
        /* Any value is accepted. Writing the box runs on_stop_changed, which sends it to the loop. */
        stop_set_by_measure = 1;
        gtk_adjustment_set_value(meas_side > 0 ? adj_stop_fwd : adj_stop_back, mean);
        stop_set_by_measure = 0;
        if (meas_side > 0) stop_src_fwd = "measured"; else stop_src_back = "measured";
        recenter_zero();                               /* zero = midpoint of the stops (Josue 10/26) */
        snprintf(meas_text, sizeof meas_text, "%s stop read %+.1f deg (moved %.1f); zero re-centred to %+.2f.",
                 side, mean, spread, balance_loop_get_pitch_zero());
    }
    meas_side = 0;
}

static void on_reset(GtkButton *btn, gpointer data)
{
    (void)btn; (void)data;
    gtk_adjustment_set_value(adj_kp, gain_def_kp);
    gtk_adjustment_set_value(adj_ki, gain_def_ki);
    gtk_adjustment_set_value(adj_kd, gain_def_kd);
}

/* Enable switch flipped -> tell the balance loop (it also stops on cutoff). */
static void on_enable_toggled(GObject *sw, GParamSpec *pspec, gpointer data)
{
    (void)pspec; (void)data;
    int on = gtk_switch_get_active(GTK_SWITCH(sw));
    if (on && balance_loop_control_mode() == SS_MODE_REFUSED) {
        gtk_switch_set_active(GTK_SWITCH(sw), FALSE);   /* model not ready and fallback is off: stay OFF */
        return;                                         /* (the control line says why) */
    }
    if (on) {
        /* Latch the slider's target into the controller BEFORE enabling, so
         * the motors start with the value you see on the slider. */
        active_target = gtk_adjustment_get_value(adj_target);
        balance_loop_set_target(active_target);
    }
    balance_loop_set_enabled(on);
    refresh_target_note();
}

static void on_quit(GtkButton *btn, gpointer data)
{
    (void)btn; (void)data;
    balance_loop_set_enabled(0);     /* motors off first, then stop the loop */
    if (quit_cb) quit_cb();
    g_application_quit(G_APPLICATION(app));
}

static gboolean update_display(gpointer data)
{
    (void)data;
    double pitch, rpm0, rpm1;
    balance_loop_get_state(&pitch, &rpm0, &rpm1);
    if (meas_side != 0)
        measure_tick(pitch);                           /* v2: block stop measurement in progress */

    char buf[200];                   /* long enough for the CUTOFF status message */
    snprintf(buf, sizeof buf, "Pitch: %+7.2f deg     Motor 0: %7.1f RPM     Motor 1: %7.1f RPM", pitch, rpm0, rpm1);
    gtk_label_set_text(GTK_LABEL(pitch_label), buf);   /* v2: one line for all three readouts */

    /* Motor status line: OFF (switch off), ON (running) or CUTOFF (|pitch|
     * went past the limit and the loop latched the motors off). */
    int enabled, tripped;
    balance_loop_get_status(&enabled, &tripped);
    double cf, cb, blo, bhi, sf, sb;
    balance_loop_get_cutoff(&cf, &cb);
    balance_loop_get_band(&blo, &bhi);
    balance_loop_get_stops(&sf, &sb);
    if (tripped) {
        /* v2: say which limit stopped the motors. */
        switch (balance_loop_get_trip_reason()) {
        case TRIP_BLOCK_REST:
            snprintf(buf, sizeof buf, "Motors: STOPPED - resting on the block for 1 s - switch OFF, then ON to re-arm");
            break;
        case TRIP_LIFT_FAILED:
            snprintf(buf, sizeof buf, "Motors: LIFT FAILED - still on the block after %d s - switch OFF, then ON to re-arm",
                     BALANCE_LIFT_WINDOW_MS / 1000);
            break;
        default:
            snprintf(buf, sizeof buf, "Motors: CUTOFF (pitch beyond +%.1f / -%.1f deg) - switch OFF, then ON to re-arm",
                     cf, cb);
            break;
        }
    }
    else {  /* also show the duty being sent: 0.000 means the controller is commanding nothing */
        int id_ms = balance_loop_get_id_left_ms();     /* >0 while an ID run is active */
        int lift_ms = balance_loop_get_lift_left_ms(); /* >0 while the lift window is open */
        char id_txt[40] = "", lift_txt[40] = "";
        if (id_ms > 0)
            snprintf(id_txt, sizeof id_txt, "   ID RUNNING: %.1f s left", id_ms / 1000.0);
        if (enabled && lift_ms > 0)
            snprintf(lift_txt, sizeof lift_txt, "   LIFTING: %.1f s left", lift_ms / 1000.0);
        snprintf(buf, sizeof buf, "Motors: %s   Duty sent: %+.3f%s%s",
                 enabled ? "ON" : "OFF", balance_loop_get_duty(), lift_txt, id_txt);
    }
    gtk_label_set_text(GTK_LABEL(status_label), buf);

    /* v2 status line: the limits in force right now, plus a warning when a cutoff sits inside the block. */
    char lim[400];
    snprintf(lim, sizeof lim,
             "In force: cutoff +%.1f / -%.1f deg | standing band %+.1f / %+.1f deg | Run ID injects within +%.1f / -%.1f deg"
             " | block stops %+.1f (%s) / %+.1f (%s) deg | pitch zero %+.2f (midpoint of the stops)%s",
             cf, cb, blo, bhi, 0.6 * cf, 0.6 * cb, sf, stop_src_fwd, sb, stop_src_back, balance_loop_get_pitch_zero(),
             (cf < fmax(sf, sb) || -cb > fmin(sf, sb))   /* stops are signed and may be in any order */
                 ? "\nWARNING: a cutoff is inside the block range, so it trips at once while the robot rests on the block." : "");
    gtk_label_set_text(GTK_LABEL(cutoff_label), lim);
    gtk_label_set_text(GTK_LABEL(meas_label), meas_text);

    /* ---- Live Run ID panel ---- */
    int left; double inj; unsigned int tot, near;
    balance_loop_get_id_stats(&left, &inj, &tot, &near);

    if (left > 0 && prev_left_ms == 0) {              /* a run just started */
        fit_next = 0;
        snprintf(fit_text, sizeof fit_text, "Live fit: first fit at %d s ...", fit_schedule_s[0]);
    }
    if (left > 0) {                                    /* scheduled fits while the run is going */
        double elapsed = (BALANCE_ID_DURATION_MS - left) / 1000.0;
        if (fit_next < 3 && elapsed >= fit_schedule_s[fit_next] && !fit_running) {
            start_fit(fit_schedule_s[fit_next]);
            fit_next++;
        }
    }
    if (left == 0 && prev_left_ms > 0) {               /* the run just ended: fit everything */
        last_near = near; last_total = tot;
        if (fit_running) fit_pending_final = 1;
        else start_fit(BALANCE_ID_DURATION_MS / 1000);
    }
    prev_left_ms = left;

    /* Gains actually in use. State feedback: the designed K times the slider. PID: the sliders. */
    char gains[220];
    if (balance_loop_control_mode() == SS_MODE_STATE_FEEDBACK)
        snprintf(gains, sizeof gains, "Gains in use (state feedback): K_pitch %+.5f | K_rate %+.5f | K_wheel %+.7f",
                 ss_gain(0) * gtk_adjustment_get_value(adj_kp), ss_gain(1) * gtk_adjustment_get_value(adj_kd),
                 ss_gain(2) * gtk_adjustment_get_value(adj_ki));
    else
        snprintf(gains, sizeof gains, "Gains in use (PID): Kp %.4f | Ki %.4f | Kd %.4f",
                 gtk_adjustment_get_value(adj_kp), gtk_adjustment_get_value(adj_ki), gtk_adjustment_get_value(adj_kd));

    char idline[300];
    if (left > 0) {
        double pct = tot ? 100.0 * near / tot : 0.0;
        snprintf(idline, sizeof idline, "ID RUNNING: %.1f s left | step now %+.3f duty | against a block %.0f%% of the run so far%s",
                 left / 1000.0, inj, pct,
                 pct > 20.0 ? "  <-- too much: the fit will be refused. Keep the robot off the blocks." : "");
    } else if (last_total > 0) {
        snprintf(idline, sizeof idline, "Last ID run: against a block %.0f%% of the time (above 20%% the fit is refused)",
                 100.0 * last_near / last_total);
    } else {
        idline[0] = '\0';                            /* v2: no instructions, keep the window short */
    }

    char panel[1300];
    if (left > 0 || last_total > 0)
        snprintf(panel, sizeof panel, "%s\n%s\n%s", idline, gains, fit_text);
    else
        panel[0] = '\0';                               /* v2: empty until the first Run ID */
    gtk_label_set_text(GTK_LABEL(id_label), panel);
    return G_SOURCE_CONTINUE;
}

/* v2: one typed number box with its name, placed at column pair col (0, 1 or 2) of a row.
 * The value is applied when you press Enter or leave the box (Up/Down arrow keys step it).
 * Returns the adjustment, which holds the range and the step. */
static GtkAdjustment *add_field(GtkGrid *grid, int col, int row, const char *name,
                                double init, double min, double max, double step, int digits,
                                void (*changed)(GtkAdjustment *, gpointer))
{
    GtkAdjustment *adj = gtk_adjustment_new(init, min, max, step, step * 10, 0.0);
    GtkWidget *spin = gtk_spin_button_new(adj, step, digits);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(spin), TRUE);   /* digits, sign and point only */
    gtk_editable_set_width_chars(GTK_EDITABLE(spin), 6);
    /* v2: hide the +/- buttons so the box is only as wide as the number (Up/Down arrow keys still step). */
    for (GtkWidget *c = gtk_widget_get_first_child(spin); c; c = gtk_widget_get_next_sibling(c))
        if (GTK_IS_BUTTON(c))
            gtk_widget_set_visible(c, FALSE);

    GtkWidget *label = gtk_label_new(name);
    gtk_label_set_xalign(GTK_LABEL(label), 1.0f);               /* right-align next to its box */
    gtk_grid_attach(grid, label, 2 * col, row, 1, 1);
    gtk_grid_attach(grid, spin, 2 * col + 1, row, 1, 1);

    g_signal_connect(adj, "value-changed", G_CALLBACK(changed), NULL);
    return adj;
}

static void activate(GtkApplication *a, gpointer data)
{
    (void)data;
    GtkWidget *win = gtk_application_window_new(a);
    gtk_window_set_title(GTK_WINDOW(win), "Balance Control v2");
    gtk_window_set_default_size(GTK_WINDOW(win), 760, 520);   /* fits the screen; the content scrolls if needed */

    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_start(box, 12);
    gtk_widget_set_margin_end(box, 12);
    gtk_widget_set_margin_top(box, 8);
    gtk_widget_set_margin_bottom(box, 8);
    /* v2: the content is taller than the screen, so it sits in a scrolled window
     * (vertical scrollbar only); every box and button stays reachable. */
    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), box);
    gtk_window_set_child(GTK_WINDOW(win), scroll);

    /* v2 layout, top to bottom: readouts | Enable, Run ID, Reset, Quit | number boxes (3 columns) |
     * stop measurement | motor status | limits in force | Run ID panel (only during and after a run). */
    pitch_label = gtk_label_new("Pitch: --");
    gtk_label_set_xalign(GTK_LABEL(pitch_label), 0.0f);
    gtk_box_append(GTK_BOX(box), pitch_label);

    /* Enable switch: starts OFF every launch. Motors only run while it is ON. */
    GtkWidget *enable_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    enable_switch = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(enable_switch), FALSE);
    g_signal_connect(enable_switch, "notify::active",
                     G_CALLBACK(on_enable_toggled), NULL);
    gtk_box_append(GTK_BOX(enable_row), gtk_label_new("Enable motors"));
    gtk_box_append(GTK_BOX(enable_row), enable_switch);
    GtkWidget *run_id = gtk_button_new_with_label("Run ID (20 s)");
    GtkWidget *reset = gtk_button_new_with_label("Reset Gains");
    GtkWidget *quit  = gtk_button_new_with_label("Quit");
    g_signal_connect(run_id, "clicked", G_CALLBACK(on_run_id), NULL);
    g_signal_connect(reset, "clicked", G_CALLBACK(on_reset), NULL);
    g_signal_connect(quit,  "clicked", G_CALLBACK(on_quit),  NULL);
    gtk_box_append(GTK_BOX(enable_row), run_id);
    gtk_box_append(GTK_BOX(enable_row), reset);
    gtk_box_append(GTK_BOX(enable_row), quit);
    gtk_box_append(GTK_BOX(box), enable_row);

    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 4);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
    gtk_box_append(GTK_BOX(box), grid);

    /* Which control path is running (decided at startup by state_space.c; the terminal says why). In
     * state-feedback mode the gains are multipliers (0..2, 1.00 = designed gain); in PID mode they are
     * the PID gains (0..GAIN_MAX). */
    int ss_on = (balance_loop_control_mode() == SS_MODE_STATE_FEEDBACK);
    balance_loop_default_gains(&gain_def_kp, &gain_def_kd, &gain_def_ki);
    gain_max = ss_on ? 2.0 : GAIN_MAX;

    /* v2: typed number boxes, three per row. Gains in 0.005 steps (3 decimals). */
    adj_kp = add_field(GTK_GRID(grid), 0, 0, ss_on ? "Kp x K_pitch" : "Kp", gain_def_kp, 0.0, gain_max, GAIN_STEP, 3, on_gain_changed);
    adj_ki = add_field(GTK_GRID(grid), 1, 0, ss_on ? "Ki x K_wheel" : "Ki", gain_def_ki, 0.0, gain_max, GAIN_STEP, 3, on_gain_changed);
    adj_kd = add_field(GTK_GRID(grid), 2, 0, ss_on ? "Kd x K_rate" : "Kd", gain_def_kd, 0.0, gain_max, GAIN_STEP, 3, on_gain_changed);
    /* Target pitch: +/-5 deg (positive = slight forward lean); applied on Enable OFF -> ON. */
    adj_target = add_field(GTK_GRID(grid), 0, 1, "Target (deg)", BALANCE_DEFAULT_TARGET, -5.0, 5.0, 0.1, 1,
                           on_target_changed);
    /* Max duty: motor power limit while tuning, up to the handout's 0.75. */
    add_field(GTK_GRID(grid), 1, 1, "Max duty", BALANCE_DEFAULT_DUTY_LIMIT, 0.0, BALANCE_DUTY_LIMIT, 0.05, 2,
              on_duty_limit_changed);
    /* ID step size: duty added by "Run ID" (v2 default 0.05); the small step is 0.6 x this. */
    add_field(GTK_GRID(grid), 2, 1, "ID step", BALANCE_DEFAULT_ID_STEP, 0.02, 0.20, 0.01, 3,
              on_id_amp_changed);
    /* Motor cutoff per side (magnitudes, 2..30 deg) and the standing band (signed, -30..+30 deg). */
    adj_cut_fwd  = add_field(GTK_GRID(grid), 0, 2, "Cutoff fwd (deg)", BALANCE_CUTOFF_DEFAULT_DEG,
                             BALANCE_CUTOFF_MIN_DEG, BALANCE_CUTOFF_MAX_DEG, 0.5, 1, on_cutoff_changed);
    adj_cut_back = add_field(GTK_GRID(grid), 1, 2, "Cutoff back (deg)", BALANCE_CUTOFF_DEFAULT_DEG,
                             BALANCE_CUTOFF_MIN_DEG, BALANCE_CUTOFF_MAX_DEG, 0.5, 1, on_cutoff_changed);
    /* Band: calibrated from the stops (stops minus 1 deg) at launch and on every stop change; typable. */
    double blo0, bhi0;
    balance_loop_get_band(&blo0, &bhi0);
    adj_band_lo  = add_field(GTK_GRID(grid), 0, 3, "Band low (deg)", blo0,
                             -BALANCE_BAND_LIMIT_DEG, BALANCE_BAND_LIMIT_DEG, 0.5, 1, on_band_changed);
    adj_band_hi  = add_field(GTK_GRID(grid), 1, 3, "Band high (deg)", bhi0,
                             -BALANCE_BAND_LIMIT_DEG, BALANCE_BAND_LIMIT_DEG, 0.5, 1, on_band_changed);
    /* Block stops, signed pitch, any value (Josue 10/26); filled in by the Measure buttons or typed. */
    double sf0, sb0;
    balance_loop_get_stops(&sf0, &sb0);
    adj_stop_fwd  = add_field(GTK_GRID(grid), 2, 2, "Fwd stop (deg)", sf0, -180.0, 180.0, 0.1, 1, on_stop_changed);
    adj_stop_back = add_field(GTK_GRID(grid), 2, 3, "Back stop (deg)", sb0, -180.0, 180.0, 0.1, 1, on_stop_changed);
    g_signal_handlers_disconnect_by_func(adj_stop_fwd, G_CALLBACK(on_stop_changed), NULL);   /* reconnect with */
    g_signal_handlers_disconnect_by_func(adj_stop_back, G_CALLBACK(on_stop_changed), NULL);  /* the side as data */
    g_signal_connect(adj_stop_fwd, "value-changed", G_CALLBACK(on_stop_changed), GINT_TO_POINTER(+1));
    g_signal_connect(adj_stop_back, "value-changed", G_CALLBACK(on_stop_changed), GINT_TO_POINTER(-1));

    /* Shown only when the typed target differs from the one in use (see refresh_target_note). */
    target_note = gtk_label_new("");
    gtk_label_set_wrap(GTK_LABEL(target_note), TRUE);
    gtk_label_set_xalign(GTK_LABEL(target_note), 0.0f);
    gtk_box_append(GTK_BOX(box), target_note);
    refresh_target_note();

    /* v2: block stop measurement, motors OFF: tilt onto the block, press, hold still 0.5 s.
     * The result (or why it was refused) appears next to the buttons. */
    GtkWidget *meas_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    GtkWidget *meas_f = gtk_button_new_with_label("Measure fwd stop");
    GtkWidget *meas_b = gtk_button_new_with_label("Measure back stop");
    g_signal_connect(meas_f, "clicked", G_CALLBACK(on_measure_stop), GINT_TO_POINTER(+1));
    g_signal_connect(meas_b, "clicked", G_CALLBACK(on_measure_stop), GINT_TO_POINTER(-1));
    gtk_box_append(GTK_BOX(meas_row), meas_f);
    gtk_box_append(GTK_BOX(meas_row), meas_b);
    meas_label = gtk_label_new(meas_text);
    gtk_label_set_wrap(GTK_LABEL(meas_label), TRUE);
    gtk_label_set_xalign(GTK_LABEL(meas_label), 0.0f);
    gtk_box_append(GTK_BOX(meas_row), meas_label);
    gtk_box_append(GTK_BOX(box), meas_row);

    status_label = gtk_label_new("Motors: OFF");
    gtk_label_set_xalign(GTK_LABEL(status_label), 0.0f);
    gtk_box_append(GTK_BOX(box), status_label);

    /* v2: limits in force (filled in by update_display every 50 ms). */
    cutoff_label = gtk_label_new("");
    gtk_label_set_wrap(GTK_LABEL(cutoff_label), TRUE);
    gtk_label_set_xalign(GTK_LABEL(cutoff_label), 0.0f);
    gtk_box_append(GTK_BOX(box), cutoff_label);

    /* Live Run ID panel (see update_display). Temp files are named by process id so runs never collide. */
    g_snprintf(snap_path, sizeof snap_path, "%s/cpe4700_live_%d_log.csv", g_get_tmp_dir(), (int)getpid());
    g_snprintf(fit_path, sizeof fit_path, "%s/cpe4700_live_%d_fit.txt", g_get_tmp_dir(), (int)getpid());
    id_label = gtk_label_new("");
    gtk_label_set_wrap(GTK_LABEL(id_label), TRUE);
    gtk_label_set_xalign(GTK_LABEL(id_label), 0.0f);
    gtk_label_set_selectable(GTK_LABEL(id_label), TRUE);
    gtk_box_append(GTK_BOX(box), id_label);

    g_timeout_add(GUI_PERIOD_MS, update_display, NULL);
    gtk_window_present(GTK_WINDOW(win));
}

int gui_run(int argc, char **argv, void (*on_quit_cb)(void))
{
    quit_cb = on_quit_cb;
    /* G_APPLICATION_FLAGS_NONE = "no special flags". The newer name
     * G_APPLICATION_DEFAULT_FLAGS needs GLib 2.74; this Jetson has 2.72. */
    app = gtk_application_new("edu.cpe4700.balance", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    int status = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);
    return status;
}
