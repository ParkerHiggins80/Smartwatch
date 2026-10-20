#ifndef HAL_BUTTONS_H
#define HAL_BUTTONS_H

/*
 * The two side buttons. Watch: PB1 on P1.10 and PB2 on P0.24, active low with
 * pull-ups. Simulator: the PB1 / PB2 buttons in the dev tools.
 *
 * All functions return 0 on success or a negative error code.
 */

#include <stdbool.h>

enum button {
    BUTTON_PB1,
    BUTTON_PB2,
};

int buttons_init(void);

/* *pressed is true while the button is held down. */
int buttons_read(enum button button, bool *pressed);

#endif /* HAL_BUTTONS_H */
