#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <SEGGER_RTT.h>

#define I2C_DEV DT_NODELABEL(i2c0)

int main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    char buf[64];

    k_sleep(K_SECONDS(2));

    if (!device_is_ready(i2c_dev)) {
        SEGGER_RTT_WriteString(0, "I2C device not ready!\n");
        return -1;
    }

    SEGGER_RTT_WriteString(0, "Scanning I2C bus...\n");

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy;
        int ret = i2c_read(i2c_dev, &dummy, 0, addr);
        if (ret == 0) {
            snprintf(buf, sizeof(buf), "Found device at 0x%02X\n", addr);
            SEGGER_RTT_WriteString(0, buf);
        }
    }

    SEGGER_RTT_WriteString(0, "Scan complete.\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}