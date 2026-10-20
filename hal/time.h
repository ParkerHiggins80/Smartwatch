#ifndef HAL_TIME_H
#define HAL_TIME_H

/*
 * Time since boot. Watch: k_uptime_get_32(). Simulator: time since the simulated
 * watch started (it restarts at 0 after a recompile or reset).
 *
 * Include it as "hal/time.h". Never put the hal/ folder itself on the include path:
 * this file would then hide the C library's <time.h>.
 */

#include <stdint.h>

/* Milliseconds since boot. Wraps around after about 49 days. */
uint32_t time_ms(void);

#endif /* HAL_TIME_H */
