/*
 * Tuning window for Lab 07 (GTK4). Runs in the main thread while the control loop runs in its own
 * thread. Sliders and number boxes change the balance angle and the gains live. The only link to
 * the control loop is the atomics in tune_shared.h.
 */
#include <gtk/gtk.h>
#include <stdio.h>
#include "tune_shared.h"
#include "balance_ctrl.h"

typedef struct {
    const char *label;
    const char *tip;
    _Atomic double *target;
    double lo, hi, step;
    int digits;
    double def;
    GtkAdjustment *adj;
} param_t;

static tune_t *T;
static param_t P[5];
static GtkWidget *lbl_state, *lbl_pitch, *lbl_speed, *lbl_duty, *lbl_file, *win;

static void on_value(GtkAdjustment *a, gpointer data)
{
    param_t *p = data;
    atomic_store(p->target, gtk_adjustment_get_value(a));
}

static void on_rearm(GtkButton *b, gpointer d)  { (void)b; (void)d; atomic_store(&T->rearm_req, 1); }

static void on_reset(GtkButton *b, gpointer d)
{
    (void)b; (void)d;
    for (int i = 0; i < 5; i++) gtk_adjustment_set_value(P[i].adj, P[i].def);
}

static void on_quit(GtkButton *b, gpointer d)
{
    (void)b; (void)d;
    atomic_store(&T->quit, 1);
    gtk_window_close(GTK_WINDOW(win));
}

static gboolean on_tick(gpointer d)
{
    (void)d;
    static const char *STATE[] = { "WAITING: hold the robot near the balance angle", "ARMED", "LIVE", "TRIPPED: press Re-arm" };
    int s = atomic_load(&T->state);
    char buf[200];
    if (s == 3) snprintf(buf, sizeof buf, "TRIPPED (%s): press Re-arm, then hold the robot near the angle",
                         atomic_load(&T->trip_reason) == 2 ? "resting on a stop" : "past the cutoff");
    else snprintf(buf, sizeof buf, "%s", STATE[s < 0 || s > 3 ? 0 : s]);
    gtk_label_set_text(GTK_LABEL(lbl_state), buf);
    snprintf(buf, sizeof buf, "Pitch %+.2f deg   target %+.2f deg", atomic_load(&T->pitch), atomic_load(&T->pitch_target));
    gtk_label_set_text(GTK_LABEL(lbl_pitch), buf);
    snprintf(buf, sizeof buf, "Wheel speed %+.0f edges/s   speed command %+.0f", atomic_load(&T->speed), atomic_load(&T->speed_cmd));
    gtk_label_set_text(GTK_LABEL(lbl_speed), buf);
    snprintf(buf, sizeof buf, "Left %+.3f   Right %+.3f", atomic_load(&T->left), atomic_load(&T->right));
    gtk_label_set_text(GTK_LABEL(lbl_duty), buf);
    if (atomic_load(&T->done)) {
        if (T->err) {
            snprintf(buf, sizeof buf, "STOPPED: %s", T->err);
            gtk_label_set_text(GTK_LABEL(lbl_state), buf);
        }
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

static void activate(GtkApplication *app, gpointer d)
{
    (void)d;
    win = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(win), "Lab 07 balance tuning");
    gtk_window_set_default_size(GTK_WINDOW(win), 560, 420);

    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_start(box, 12); gtk_widget_set_margin_end(box, 12);
    gtk_widget_set_margin_top(box, 12);   gtk_widget_set_margin_bottom(box, 12);
    gtk_window_set_child(GTK_WINDOW(win), box);

    lbl_state = gtk_label_new("");
    lbl_pitch = gtk_label_new("");
    lbl_speed = gtk_label_new("");
    lbl_duty  = gtk_label_new("");
    char fbuf[200];
    snprintf(fbuf, sizeof fbuf, "Log: %s", T->log_name ? T->log_name : "");
    lbl_file = gtk_label_new(fbuf);
    for (GtkWidget **w = (GtkWidget *[]){ lbl_state, lbl_pitch, lbl_speed, lbl_duty, lbl_file, NULL }; *w; w++) {
        gtk_widget_set_halign(*w, GTK_ALIGN_START);
        gtk_box_append(GTK_BOX(box), *w);
    }
    PangoAttrList *al = pango_attr_list_new();
    pango_attr_list_insert(al, pango_attr_weight_new(PANGO_WEIGHT_BOLD));
    gtk_label_set_attributes(GTK_LABEL(lbl_state), al);
    pango_attr_list_unref(al);

    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 10);
    gtk_box_append(GTK_BOX(box), grid);
    for (int i = 0; i < 5; i++) {
        param_t *p = &P[i];
        double now = atomic_load(p->target);
        p->adj = gtk_adjustment_new(now, p->lo, p->hi, p->step, p->step * 10, 0);
        GtkWidget *name = gtk_label_new(p->label);
        gtk_widget_set_halign(name, GTK_ALIGN_START);
        gtk_widget_set_tooltip_text(name, p->tip);
        GtkWidget *scale = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, p->adj);
        gtk_scale_set_draw_value(GTK_SCALE(scale), FALSE);
        gtk_widget_set_hexpand(scale, TRUE);
        GtkWidget *spin = gtk_spin_button_new(p->adj, p->step, p->digits);
        gtk_grid_attach(GTK_GRID(grid), name, 0, i, 1, 1);
        gtk_grid_attach(GTK_GRID(grid), scale, 1, i, 1, 1);
        gtk_grid_attach(GTK_GRID(grid), spin, 2, i, 1, 1);
        g_signal_connect(p->adj, "value-changed", G_CALLBACK(on_value), p);
    }

    GtkWidget *row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_box_append(GTK_BOX(box), row);
    GtkWidget *b1 = gtk_button_new_with_label("Re-arm after a trip");
    GtkWidget *b2 = gtk_button_new_with_label("Reset to defaults");
    GtkWidget *b3 = gtk_button_new_with_label("Stop and quit");
    g_signal_connect(b1, "clicked", G_CALLBACK(on_rearm), NULL);
    g_signal_connect(b2, "clicked", G_CALLBACK(on_reset), NULL);
    g_signal_connect(b3, "clicked", G_CALLBACK(on_quit), NULL);
    gtk_box_append(GTK_BOX(row), b1);
    gtk_box_append(GTK_BOX(row), b2);
    gtk_box_append(GTK_BOX(row), b3);

    GtkWidget *hint = gtk_label_new("Changes apply immediately. Every change is written to the *_changes.csv next to the log.");
    gtk_widget_set_halign(hint, GTK_ALIGN_START);
    gtk_box_append(GTK_BOX(box), hint);

    g_timeout_add(100, on_tick, NULL);
    gtk_window_present(GTK_WINDOW(win));
}

int tune_gui_available(void)
{
    return gtk_init_check();
}

int tune_gui_run(tune_t *t)
{
    T = t;
    P[0] = (param_t){ "Balance angle (deg)", "Angle from the saved zero that the robot balances at", &t->theta0, t->theta0_min, t->theta0_max, 0.1, 2, t->d_theta0, NULL };
    P[1] = (param_t){ "Kp (duty per deg)", "Proportional gain", &t->kp, 0.0, 0.06, 0.001, 4, t->d_kp, NULL };
    P[2] = (param_t){ "Ki (duty per deg*s)", "Integral gain", &t->ki, 0.0, 0.03, 0.0005, 4, t->d_ki, NULL };
    P[3] = (param_t){ "Kd (duty per deg/s)", "Derivative gain", &t->kd, 0.0, 0.03, 0.0005, 4, t->d_kd, NULL };
    P[4] = (param_t){ "Kv (deg per edges/s)", "Speed-to-pitch gain: 0 turns the hold-in-place off", &t->kv, 0.0, 0.005, 0.0001, 4, t->d_kv, NULL };

    GtkApplication *app = gtk_application_new("edu.cpe4700.lab07", G_APPLICATION_NON_UNIQUE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    int rc = g_application_run(G_APPLICATION(app), 0, NULL);
    g_object_unref(app);
    return rc;
}
