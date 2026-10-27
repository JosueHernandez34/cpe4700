/*
 * gui.c -- Lab 05 GTK4 GUI: pitch/RPM readouts, gain sliders + spin buttons.
 */

#include <math.h>                   /* fabs() for the target-pitch note */
#include <stdio.h>                  /* snprintf() */
#include <string.h>
#include <unistd.h>                 /* getpid() for the temp file names */
#include <glib/gstdio.h>            /* g_remove() */
#include <gtk/gtk.h>

#include "gui.h"
#include "balance_loop.h"
#include "state_space.h"      /* ss_gain(): the designed gains, for the live display */

#define GUI_PERIOD_MS 50             /* 20 Hz refresh */
#define GAIN_MAX      1.0            /* top of the slider; gains are duty per degree, so
                                      * Kp = 0.02 already gives full duty at ~37 deg */
#define GAIN_STEP     0.005          /* one click on the spin button / arrow key */

static void (*quit_cb)(void);

static GtkWidget *pitch_label, *rpm0_label, *rpm1_label;
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
static GtkWidget *control_label;                  /* which control path is active and why */
static GtkWidget *cutoff_label;                   /* cutoff angles and whether the stop angle is measured */
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
        snprintf(buf, sizeof buf,
                 "NOTE: target pitch changes only apply when you switch Enable "
                 "OFF, then ON. Active target: %+.1f deg.", active_target);
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

/* "Run ID" pressed: start the 20 s identification sequence. The balance loop
 * ignores it unless the motors are ON and not cut off. */
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
        snprintf(fit_text, sizeof fit_text, "Live fit (data to %d s): no answer from identify.py", fit_for_s);
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
    if (!g_file_test("identify.py", G_FILE_TEST_EXISTS)) {
        snprintf(fit_text, sizeof fit_text, "Live fit: identify.py not found. Start robotest from the lab_05 folder.");
        return;
    }
    if (balance_loop_write_snapshot(snap_path) < 0) {
        snprintf(fit_text, sizeof fit_text, "Live fit: no log data yet");
        return;
    }
    g_remove(fit_path);                                /* so a stale answer is never shown */
    char *argv[] = { "nice", "-n", "19", "python3", "identify.py", snap_path, "--live", fit_path, NULL };
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
        snprintf(fit_text, sizeof fit_text, "Live fit: could not start identify.py (%s)", err ? err->message : "?");
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

    char buf[200];                   /* long enough for the CUTOFF status message */
    snprintf(buf, sizeof buf, "Pitch: %+7.2f deg", pitch);
    gtk_label_set_text(GTK_LABEL(pitch_label), buf);
    snprintf(buf, sizeof buf, "Motor 0: %7.1f RPM", rpm0);
    gtk_label_set_text(GTK_LABEL(rpm0_label), buf);
    snprintf(buf, sizeof buf, "Motor 1: %7.1f RPM", rpm1);
    gtk_label_set_text(GTK_LABEL(rpm1_label), buf);

    /* Motor status line: OFF (switch off), ON (running) or CUTOFF (|pitch|
     * went past the limit and the loop latched the motors off). */
    int enabled, tripped;
    balance_loop_get_status(&enabled, &tripped);
    if (tripped) {
        double cf, cb; int sm;
        balance_loop_get_cutoff(&cf, &cb, &sm);
        snprintf(buf, sizeof buf,
                 "Motors: CUTOFF (pitch beyond +%.1f / -%.1f deg, or on the block) - switch OFF, then ON to re-arm",
                 cf, cb);
    }
    else {  /* also show the duty being sent: 0.000 means the controller is commanding nothing */
        int id_ms = balance_loop_get_id_left_ms();     /* >0 while an ID run is active */
        char id_txt[40] = "";
        if (id_ms > 0)
            snprintf(id_txt, sizeof id_txt, "   ID RUNNING: %.1f s left", id_ms / 1000.0);
        snprintf(buf, sizeof buf, "Motors: %s   Duty sent: %+.3f%s",
                 enabled ? "ON" : "OFF", balance_loop_get_duty(), id_txt);
    }
    gtk_label_set_text(GTK_LABEL(status_label), buf);

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
        snprintf(idline, sizeof idline, "Run ID: press the button; it turns the motors ON and starts a 20 s run.");
    }

    char panel[1300];
    snprintf(panel, sizeof panel, "%s\n%s\n%s", idline, gains, fit_text);
    gtk_label_set_text(GTK_LABEL(id_label), panel);
    return G_SOURCE_CONTINUE;
}

/* One row: name | slider | spin button. Returns the shared adjustment. */
static GtkAdjustment *add_gain_row(GtkGrid *grid, int row, const char *name,
                                   double init)
{
    GtkAdjustment *adj = gtk_adjustment_new(init, 0.0, gain_max, GAIN_STEP,
                                            0.1, 0.0);
    GtkWidget *scale = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, adj);
    GtkWidget *spin  = gtk_spin_button_new(adj, GAIN_STEP, 3);

    gtk_widget_set_hexpand(scale, TRUE);
    gtk_scale_set_draw_value(GTK_SCALE(scale), FALSE);

    gtk_grid_attach(grid, gtk_label_new(name), 0, row, 1, 1);
    gtk_grid_attach(grid, scale, 1, row, 1, 1);
    gtk_grid_attach(grid, spin,  2, row, 1, 1);

    g_signal_connect(adj, "value-changed", G_CALLBACK(on_gain_changed), NULL);
    return adj;
}

/* Same layout as a gain row (name | slider | spin) with its own range and
 * callback. Used for the target pitch and the max duty. */
static GtkAdjustment *add_param_row(GtkGrid *grid, int row, const char *name,
                                    double init, double min, double max,
                                    double step,
                                    void (*changed)(GtkAdjustment *, gpointer))
{
    GtkAdjustment *adj = gtk_adjustment_new(init, min, max, step, step * 10, 0.0);
    GtkWidget *scale = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, adj);
    GtkWidget *spin  = gtk_spin_button_new(adj, step, 3);

    gtk_widget_set_hexpand(scale, TRUE);
    gtk_scale_set_draw_value(GTK_SCALE(scale), FALSE);

    gtk_grid_attach(grid, gtk_label_new(name), 0, row, 1, 1);
    gtk_grid_attach(grid, scale, 1, row, 1, 1);
    gtk_grid_attach(grid, spin,  2, row, 1, 1);

    g_signal_connect(adj, "value-changed", G_CALLBACK(changed), NULL);
    return adj;
}

static void activate(GtkApplication *a, gpointer data)
{
    (void)data;
    GtkWidget *win = gtk_application_window_new(a);
    gtk_window_set_title(GTK_WINDOW(win), "Balance Control");
    gtk_window_set_default_size(GTK_WINDOW(win), 760, 640);   /* taller: extra rows */

    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_start(box, 12);
    gtk_widget_set_margin_end(box, 12);
    gtk_widget_set_margin_top(box, 12);
    gtk_widget_set_margin_bottom(box, 12);
    gtk_window_set_child(GTK_WINDOW(win), box);

    pitch_label = gtk_label_new("Pitch: --");
    rpm0_label  = gtk_label_new("Motor 0: --");
    rpm1_label  = gtk_label_new("Motor 1: --");
    gtk_box_append(GTK_BOX(box), pitch_label);
    gtk_box_append(GTK_BOX(box), rpm0_label);
    gtk_box_append(GTK_BOX(box), rpm1_label);

    /* Enable switch: starts OFF every launch. Motors only run while it is ON. */
    GtkWidget *enable_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    enable_switch = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(enable_switch), FALSE);
    g_signal_connect(enable_switch, "notify::active",
                     G_CALLBACK(on_enable_toggled), NULL);
    status_label = gtk_label_new("Motors: OFF");
    gtk_box_append(GTK_BOX(enable_row), gtk_label_new("Enable motors"));
    gtk_box_append(GTK_BOX(enable_row), enable_switch);
    gtk_box_append(GTK_BOX(enable_row), status_label);
    gtk_box_append(GTK_BOX(box), enable_row);

    /* Which control path is active and why (state feedback / PID fallback / refused). */
    char ctl[260];
    balance_loop_control_text(ctl, sizeof ctl);
    control_label = gtk_label_new(ctl);
    gtk_label_set_wrap(GTK_LABEL(control_label), TRUE);
    gtk_label_set_xalign(GTK_LABEL(control_label), 0.0f);
    gtk_box_append(GTK_BOX(box), control_label);

    /* Cutoff angles; says so when they rest on the unconfirmed 25 deg stop claim. */
    double cf, cb; int sm;
    balance_loop_get_cutoff(&cf, &cb, &sm);
    char cut[200];
    snprintf(cut, sizeof cut, "Cutoff: +%.1f / -%.1f deg%s", cf, cb,
             sm ? " (from the measured stop angles)" : " (INTERIM: stop angle not measured yet, using the unconfirmed 25 deg)");
    cutoff_label = gtk_label_new(cut);
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

    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 10);
    gtk_box_append(GTK_BOX(box), grid);

    /* Which control path is running (decided at startup by state_space.c). In
     * state-feedback mode the sliders are multipliers (0..2, 1.00 = designed gain);
     * in PID mode they are the PID gains (0..GAIN_MAX, start at 0). */
    int ss_on = (balance_loop_control_mode() == SS_MODE_STATE_FEEDBACK);
    balance_loop_default_gains(&gain_def_kp, &gain_def_kd, &gain_def_ki);
    gain_max = ss_on ? 2.0 : GAIN_MAX;

    adj_kp = add_gain_row(GTK_GRID(grid), 0, ss_on ? "Kp x K_pitch" : "Kp", gain_def_kp);
    adj_ki = add_gain_row(GTK_GRID(grid), 1, ss_on ? "Ki x K_wheel speed" : "Ki", gain_def_ki);
    adj_kd = add_gain_row(GTK_GRID(grid), 2, ss_on ? "Kd x K_pitch rate" : "Kd", gain_def_kd);

    /* Target pitch: +/-5 deg in 0.1 deg steps. A small positive value makes
     * the robot hold a slight forward lean, so it creeps forward. */
    adj_target = add_param_row(GTK_GRID(grid), 3, "Target pitch (deg)",
                  BALANCE_DEFAULT_TARGET, -5.0, 5.0, 0.1, on_target_changed);
    /* Max duty: motor power limit while tuning, 0.05 steps up to the handout's 0.75. */
    add_param_row(GTK_GRID(grid), 4, "Max duty", BALANCE_DEFAULT_DUTY_LIMIT,
                  0.0, BALANCE_DUTY_LIMIT, 0.05, on_duty_limit_changed);
    /* ID step size: duty added by "Run ID". 0.08 clears the ~0.06 wheel deadband. */
    add_param_row(GTK_GRID(grid), 5, "ID step size (duty)", 0.08,
                  0.02, 0.20, 0.01, on_id_amp_changed);

    /* Reminder under the sliders (see refresh_target_note). */
    target_note = gtk_label_new("");
    gtk_label_set_wrap(GTK_LABEL(target_note), TRUE);
    gtk_label_set_xalign(GTK_LABEL(target_note), 0.0f);
    gtk_box_append(GTK_BOX(box), target_note);
    refresh_target_note();

    GtkWidget *btns = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    GtkWidget *reset = gtk_button_new_with_label("Reset Gains");
    GtkWidget *quit  = gtk_button_new_with_label("Quit");
    GtkWidget *run_id = gtk_button_new_with_label("Run ID (20 s)");
    g_signal_connect(run_id, "clicked", G_CALLBACK(on_run_id), NULL);
    gtk_box_append(GTK_BOX(btns), run_id);
    g_signal_connect(reset, "clicked", G_CALLBACK(on_reset), NULL);
    g_signal_connect(quit,  "clicked", G_CALLBACK(on_quit),  NULL);
    gtk_box_append(GTK_BOX(btns), reset);
    gtk_box_append(GTK_BOX(btns), quit);
    gtk_box_append(GTK_BOX(box), btns);

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
