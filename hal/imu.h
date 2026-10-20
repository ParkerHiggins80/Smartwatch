#ifndef HAL_IMU_H
#define HAL_IMU_H

/*
 * Accelerometer: LSM6DSO32 at I2C 0x6A, interrupts on ACCEL_INT2 (P1.09).
 * Simulator: the accelerometer sliders and event buttons in the dev tools.
 *
 * All functions return 0 on success or a negative error code (-EIO when the chip
 * does not answer).
 */

#include <stdint.h>

/* Acceleration in milli-g (1000 = 1 g). Flat on a table, face up: z is about +1000. */
struct imu_accel {
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
};

/* Interrupt events, as bits in the mask imu_get_events() returns */
#define IMU_EVENT_WAKE (1u << 0)   /* movement above the wake-up threshold */
#define IMU_EVENT_TILT (1u << 1)   /* the watch was tilted */

int imu_init(void);

int imu_read_accel(struct imu_accel *accel);

/* Which interrupts fired since the last call (0 if none). Reading clears them. */
int imu_get_events(uint32_t *events);

#endif /* HAL_IMU_H */
