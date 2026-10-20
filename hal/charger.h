#ifndef HAL_CHARGER_H
#define HAL_CHARGER_H

/*
 * Battery charger: BQ25185, STAT1 on P1.04 and STAT2 on P1.06. The charger reports
 * its state only through these two pins; turning the levels into "charging",
 * "done" or "fault" is up to the app (see the BQ25185 datasheet).
 * Simulator: the USB toggle and charger state picker in the dev tools.
 *
 * All functions return 0 on success or a negative error code.
 */

#include <stdbool.h>

int charger_init(void);

/* Pin levels as read: 0 = low, 1 = high. */
int charger_read_stat_pins(int *stat1, int *stat2);

/* *present is true while USB power is plugged in (nRF52840 VBUS detection). */
int charger_read_usb_power(bool *present);

#endif /* HAL_CHARGER_H */
