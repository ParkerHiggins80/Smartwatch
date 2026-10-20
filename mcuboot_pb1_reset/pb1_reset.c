/* Built into MCUboot only. Runs once at startup, before MCUboot's main code:
 *  - Freeze rescue: two watchdog resets in a row -> force USB mode.
 *  - Normal boot: arm the watchdog (the app must feed it), then exit.
 *  - USB mode coming (boot mode flag set, or PB2 held): watch PB1
 *    (press = reset back to the app) and keep the watchdog fed. */
#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>
#include <stdbool.h>

#define PB1_PIN    (32 + 10)   // P1.10, pulls to GND when pressed
#define PB2_PIN    (24)        // P0.24, pulls to GND when pressed
#define DB_LED_PIN (32 + 11)   // P1.11, active high
#define BOOT_MODE_BOOTLOADER 0x01

#define WDT_TIMEOUT_S      8   // app must feed at least this often
#define RESCUE_AFTER_DOGS  2   // watchdog resets in a row before forcing USB mode

static void wdt_feed(void)
{
    if (NRF_WDT->RUNSTATUS) {
        for (int i = 0; i < 8; i++) {
            if (NRF_WDT->RREN & (1UL << i)) {
                NRF_WDT->RR[i] = WDT_RR_RR_Reload;
            }
        }
    }
}

static void wdt_arm(void)
{
    // Still running from before a soft reset: it can't be changed, and doesn't need to be
    if (NRF_WDT->RUNSTATUS) {
        return;
    }
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |   // keeps counting while CPU sleeps
                      (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);    // pauses while J-Link halts the CPU
    NRF_WDT->CRV = WDT_TIMEOUT_S * 32768 - 1;
    NRF_WDT->RREN = WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
}

static void pb1_reset_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    // ---- Freeze rescue: count watchdog resets ----
    bool rescue = false;
    if (NRF_POWER->RESETREAS & POWER_RESETREAS_DOG_Msk) {
        NRF_POWER->RESETREAS = POWER_RESETREAS_DOG_Msk;   // clear it so each one counts once
        uint32_t dogs = NRF_POWER->GPREGRET2 + 1;
        if (dogs >= RESCUE_AFTER_DOGS) {
            NRF_POWER->GPREGRET2 = 0;                     // fresh tries for the next app
            NRF_POWER->GPREGRET = BOOT_MODE_BOOTLOADER;   // MCUboot reads this next -> USB mode
            rescue = true;
        } else {
            NRF_POWER->GPREGRET2 = dogs;
        }
    }

    // Check PB2 with its pull-up; wait 10 ms so its 0.1 uF cap can charge on power-up
    nrf_gpio_cfg_input(PB2_PIN, NRF_GPIO_PIN_PULLUP);
    k_busy_wait(10000);

    bool flag_set = (NRF_POWER->GPREGRET == BOOT_MODE_BOOTLOADER);
    bool pb2_held = !nrf_gpio_pin_read(PB2_PIN);

    if (!flag_set && !pb2_held) {
        wdt_arm();   // normal boot: from here on the app has to feed it
        return;      // leave everything else to MCUboot
    }

    // ---- USB mode is coming: watch PB1, keep the watchdog fed ----
    nrf_gpio_cfg_input(PB1_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(DB_LED_PIN);
    nrf_gpio_pin_set(DB_LED_PIN);

    bool armed = false;
    int low_ms = 0;
    int tick = 0;

    while (1) {
        wdt_feed();

        // PB1: must be seen released first, then pressed for 60 ms
        if (nrf_gpio_pin_read(PB1_PIN)) {
            armed = true;
            low_ms = 0;
        } else if (armed) {
            low_ms += 20;
            if (low_ms >= 60) {
                NVIC_SystemReset();
            }
        }

        // LED: normal USB mode = on with a short off-blink every second
        //      freeze rescue   = slow on/off blink
        tick++;
        if (rescue) {
            nrf_gpio_pin_write(DB_LED_PIN, (tick % 50) < 25);
        } else {
            nrf_gpio_pin_write(DB_LED_PIN, (tick % 50) >= 3);
        }

        k_msleep(20);
    }
}

// Priority -1 runs before MCUboot's main thread (0), so the checks above
// happen before MCUboot reads and clears the boot mode flag.
// No start delay: it decides immediately.
K_THREAD_DEFINE(pb1_reset_tid, 512, pb1_reset_thread, NULL, NULL, NULL,
                -1, 0, 0);