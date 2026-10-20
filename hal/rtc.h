#ifndef HAL_RTC_H
#define HAL_RTC_H

/*
 * Real-time clock: RV-3028 (I2C). Keeps time while the rest of the watch is off.
 * Simulator: set, run, stop and drift it in the dev tools.
 *
 * All functions return 0 on success or a negative error code (-EIO when the chip
 * does not answer, which is what the current board does).
 */

#include <stdint.h>

struct rtc_time {
    uint16_t year;     /* 2000-2099 */
    uint8_t month;     /* 1-12 */
    uint8_t day;       /* 1-31 */
    uint8_t hour;      /* 0-23 */
    uint8_t minute;    /* 0-59 */
    uint8_t second;    /* 0-59 */
    uint8_t weekday;   /* 0 = Sunday ... 6 = Saturday */
};

int rtc_init(void);

int rtc_get_time(struct rtc_time *time);

/* weekday is ignored; the clock works it out. -EINVAL if a field is out of range. */
int rtc_set_time(const struct rtc_time *time);

#endif /* HAL_RTC_H */
