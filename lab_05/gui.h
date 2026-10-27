#ifndef GUI_H
#define GUI_H

/*
 * gui -- GTK4 display and gain tuning (Lab 05). Runs in the main thread.
 *
 * on_quit is called when the user presses "Quit" so the caller can stop
 * the balance thread. gui_run() blocks until the window closes and returns
 * the GApplication exit status.
 */
int gui_run(int argc, char **argv, void (*on_quit)(void));

#endif
