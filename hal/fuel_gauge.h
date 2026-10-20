#ifndef HAL_FUEL_GAUGE_H
#define HAL_FUEL_GAUGE_H

/*
 * Battery fuel gauge: MAX17048 at I2C 0x36. Simulator: the voltage and percent
 * sliders and the discharge simulation in the dev tools.
 *
 * All functions return 0 on success or a negative error code (-EIO when the chip
 * does not answer, which is what the current board does).
 */

#include <stdint.h>

int fuel_gauge_init(void);

/* Battery voltage in millivolts, for example 3960. */
int fuel_gauge_read_voltage_mv(uint16_t *mv);

/* State of charge, 0-100 %. */
int fuel_gauge_read_percent(uint8_t *percent);

#endif /* HAL_FUEL_GAUGE_H */
