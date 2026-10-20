#ifndef UI_API_H
#define UI_API_H

/*
 * The UI's public interface: facts in, actions out.
 *
 * Both the watch's src/main.c and the simulator call these. Everything in ui/
 * uses LVGL only (no Zephyr, no hardware), so the same code runs in both places.
 */

/* Build the first screen. Call once, after LVGL and the display are set up. */
void ui_init(void);

#endif /* UI_API_H */
