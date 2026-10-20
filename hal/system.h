#ifndef HAL_SYSTEM_H
#define HAL_SYSTEM_H

/*
 * Whole-watch actions. Watch: sys_reboot() and bootmode_set(). Simulator: restarts
 * the simulated watch, or shows a "USB mode" screen.
 */

/* Restart the watch. Never returns. */
void system_reboot(void);

/* Reboot into the bootloader's USB mode (for flashing). Only returns if it failed,
 * with a negative error code. */
int system_enter_usb_mode(void);

#endif /* HAL_SYSTEM_H */
