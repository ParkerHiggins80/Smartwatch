#ifndef HAL_HAPTIC_H
#define HAL_HAPTIC_H

/*
 * Vibration motor driver: DRV2605L (enable pin P1.02). It has a library of built-in
 * effects numbered 1 to 123, for example 1 = strong click, 47 = buzz.
 * Simulator: an on-screen pulse and the action log.
 *
 * All functions return 0 on success or a negative error code (-EIO when the chip
 * does not answer, -EINVAL for an effect number that does not exist).
 */

#include <stdint.h>

int haptic_init(void);

/* Play one built-in effect (1-123). Returns straight away; the motor runs on its own. */
int haptic_play_effect(uint8_t effect);

/* Play up to 8 steps back to back, like the chip's sequencer. A step of 1-123 is an
 * effect; 0x81-0xFF is a pause of (step - 0x80) x 10 ms. */
int haptic_play_sequence(const uint8_t *steps, int count);

/* Stop whatever is playing. */
int haptic_stop(void);

#endif /* HAL_HAPTIC_H */
