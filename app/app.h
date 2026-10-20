#ifndef APP_H
#define APP_H

/*
 * The watch's logic: what happens when a notification arrives, when to buzz, when
 * the battery is low, and so on. No Zephyr and no LVGL in app/: it talks to the
 * hardware through hal/ and to the screens through ui/ui_api.h, so the same code runs
 * on the watch and in the simulator.
 *
 * Both mains (the watch's src/main.c and the simulator's) call these, from the main
 * loop only.
 */

/* Once at boot, after ui_init(). */
void app_init(void);

/* Every pass of the main loop, before LVGL redraws. Must not block. */
void app_tick(void);

#endif /* APP_H */
