/*
 * CO5300 AMOLED display driver (single-line SPI on SPIM3)
 * Board-specific pins for Rev 1.1:
 *   VCI_EN = P1.15, RESET = P0.28, CS = P0.17 (driven here, not by SPI)
 * Rules for this panel:
 *   - drawing windows must start on even x/y and be even in width/height
 *   - 120 ms minimum between sleep-in and sleep-out
 */
#define DT_DRV_COMPAT watch_co5300

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <hal/nrf_gpio.h>
#include <string.h>
#include <errno.h>

#define VCI_EN_PIN    (32 + 15)
#define DISP_RST_PIN  (28)
#define CS_PIN        (17)
#define X_OFFSET      6            // panel's first visible column

#define CMD_SLPIN     0x10
#define CMD_SLPOUT    0x11
#define CMD_DISPOFF   0x28
#define CMD_DISPON    0x29
#define CMD_CASET     0x2A
#define CMD_RASET     0x2B
#define CMD_RAMWR     0x2C
#define CMD_TEON      0x35
#define CMD_COLMOD    0x3A
#define CMD_WRDISBV   0x51
#define CMD_WRCTRLD   0x53
#define CMD_SPIMODE   0xC4
#define CMD_PAGESW    0xFE

struct co5300_config {
	struct spi_dt_spec bus;
	uint16_t width;
	uint16_t height;
};

struct co5300_data {
	int64_t slept_at;
	bool asleep;
	uint8_t brightness;
};

static int send(const struct device *dev, const void *buf, size_t len)
{
	const struct co5300_config *cfg = dev->config;
	struct spi_buf b = { .buf = (void *)buf, .len = len };
	struct spi_buf_set s = { .buffers = &b, .count = 1 };
	return spi_write_dt(&cfg->bus, &s);
}

static int cmd(const struct device *dev, uint8_t c, const uint8_t *p, size_t n)
{
	uint8_t buf[4 + 16];
	if (n > 16) return -EINVAL;
	buf[0] = 0x02; buf[1] = 0x00; buf[2] = c; buf[3] = 0x00;   // single-line write
	if (p && n) memcpy(&buf[4], p, n);

	nrf_gpio_pin_clear(CS_PIN);
	int ret = send(dev, buf, 4 + n);
	nrf_gpio_pin_set(CS_PIN);
	return ret;
}

static int cmd1(const struct device *dev, uint8_t c, uint8_t v)
{
	return cmd(dev, c, &v, 1);
}

static void set_window(const struct device *dev, int x0, int y0, int w, int h)
{
	int x1 = x0 + w - 1;
	int y1 = y0 + h - 1;
	uint8_t caset[4] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
	uint8_t raset[4] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
	cmd(dev, CMD_CASET, caset, 4);
	cmd(dev, CMD_RASET, raset, 4);
}

// Configuration registers; re-sent on every wake in case a write was corrupted
static void config_regs(const struct device *dev)
{
	cmd1(dev, CMD_PAGESW, 0x00);
	cmd1(dev, CMD_SPIMODE, 0x80);
	cmd1(dev, CMD_COLMOD, 0x55);    // RGB565
	cmd1(dev, CMD_TEON, 0x00);
	cmd1(dev, CMD_WRCTRLD, 0x20);   // brightness control on
}

static int co5300_write(const struct device *dev, const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc, const void *buf)
{
	static uint8_t ramwr[4] = { 0x02, 0x00, CMD_RAMWR, 0x00 };   // RAM, not flash (DMA)
	const uint8_t *p = buf;
	size_t row_bytes = (size_t)desc->width * 2;
	int ret;

	set_window(dev, x + X_OFFSET, y, desc->width, desc->height);

	nrf_gpio_pin_clear(CS_PIN);
	ret = send(dev, ramwr, sizeof(ramwr));
	if (desc->pitch == desc->width) {
		if (!ret) ret = send(dev, p, row_bytes * desc->height);
	} else {
		for (uint16_t r = 0; r < desc->height && !ret; r++) {
			ret = send(dev, p + (size_t)r * desc->pitch * 2, row_bytes);
		}
	}
	nrf_gpio_pin_set(CS_PIN);
	return ret;
}

// "Blanking on" = panel sleep (image is kept in panel RAM)
static int co5300_blanking_on(const struct device *dev)
{
	struct co5300_data *d = dev->data;
	if (d->asleep) return 0;

	cmd(dev, CMD_DISPOFF, NULL, 0);
	k_msleep(20);
	cmd(dev, CMD_SLPIN, NULL, 0);
	d->slept_at = k_uptime_get();
	d->asleep = true;
	return 0;
}

static int co5300_blanking_off(const struct device *dev)
{
	struct co5300_data *d = dev->data;
	if (!d->asleep) return 0;

	int64_t since = k_uptime_get() - d->slept_at;
	if (since < 120) k_msleep(120 - since);

	config_regs(dev);
	cmd1(dev, CMD_WRDISBV, d->brightness);
	cmd(dev, CMD_SLPOUT, NULL, 0);
	k_msleep(60);
	cmd(dev, CMD_DISPON, NULL, 0);
	d->asleep = false;
	return 0;
}

static int co5300_set_brightness(const struct device *dev, const uint8_t brightness)
{
	struct co5300_data *d = dev->data;
	d->brightness = brightness;
	return cmd1(dev, CMD_WRDISBV, brightness);
}

static void co5300_get_capabilities(const struct device *dev,
				    struct display_capabilities *caps)
{
	const struct co5300_config *cfg = dev->config;
	memset(caps, 0, sizeof(*caps));
	caps->x_resolution = cfg->width;
	caps->y_resolution = cfg->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
	caps->current_pixel_format = PIXEL_FORMAT_RGB_565;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int co5300_set_pixel_format(const struct device *dev,
				   const enum display_pixel_format pf)
{
	return (pf == PIXEL_FORMAT_RGB_565) ? 0 : -ENOTSUP;
}

static int co5300_init(const struct device *dev)
{
	const struct co5300_config *cfg = dev->config;
	struct co5300_data *d = dev->data;

	if (!spi_is_ready_dt(&cfg->bus)) return -ENODEV;

	nrf_gpio_cfg_output(VCI_EN_PIN);
	nrf_gpio_cfg_output(DISP_RST_PIN);
	nrf_gpio_cfg_output(CS_PIN);

	nrf_gpio_pin_clear(VCI_EN_PIN);
	nrf_gpio_pin_clear(DISP_RST_PIN);
	nrf_gpio_pin_set(CS_PIN);

	nrf_gpio_pin_set(VCI_EN_PIN);
	k_msleep(15);
	nrf_gpio_pin_clear(DISP_RST_PIN);
	k_msleep(10);
	nrf_gpio_pin_set(DISP_RST_PIN);
	k_msleep(150);

	config_regs(dev);
	d->brightness = 0;                      // app fades in after first draw
	cmd1(dev, CMD_WRDISBV, 0);
	cmd(dev, CMD_SLPOUT, NULL, 0);
	k_msleep(60);
	cmd(dev, CMD_DISPON, NULL, 0);
	k_msleep(20);

	d->asleep = false;
	return 0;
}

static const struct display_driver_api co5300_api = {
	.blanking_on = co5300_blanking_on,
	.blanking_off = co5300_blanking_off,
	.write = co5300_write,
	.set_brightness = co5300_set_brightness,
	.get_capabilities = co5300_get_capabilities,
	.set_pixel_format = co5300_set_pixel_format,
};

#define CO5300_DEFINE(n)                                                         \
	static const struct co5300_config co5300_cfg_##n = {                     \
		.bus = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
		.width = DT_INST_PROP(n, width),                                 \
		.height = DT_INST_PROP(n, height),                               \
	};                                                                       \
	static struct co5300_data co5300_data_##n;                               \
	DEVICE_DT_INST_DEFINE(n, co5300_init, NULL, &co5300_data_##n,            \
			      &co5300_cfg_##n, POST_KERNEL,                      \
			      CONFIG_DISPLAY_INIT_PRIORITY, &co5300_api);

DT_INST_FOREACH_STATUS_OKAY(CO5300_DEFINE)