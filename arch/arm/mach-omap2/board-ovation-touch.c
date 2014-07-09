/*
 * arch/arm/mach-omap2/board-44xx-ovation2-touch.c
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <plat/i2c.h>
#include "board-ovation.h"
#include "mux.h"
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE)
#include <linux/input/ft5x06.h>
#endif /* defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE) */

#define TOUCHPANEL_GPIO_IRQ     37
#define TOUCHPANEL_GPIO_RESET   39

#define OVATION_TOUCH_X_RES 1280
#define OVATION_TOUCH_Y_RES 1920

extern int Vdd_LCD_CT_PEN_request_supply(struct device *dev, const char *supply_name);
extern int Vdd_LCD_CT_PEN_enable(struct device *dev, const char *supply_name);
extern int Vdd_LCD_CT_PEN_disable(struct device *dev, const char *supply_name);
extern int Vdd_LCD_CT_PEN_release_supply(struct device *dev, const char *supply_name);

static struct gpio ovation_touch_gpios[] = {
	{ TOUCHPANEL_GPIO_IRQ,		GPIOF_IN,		"touch_irq"   },
	{ TOUCHPANEL_GPIO_RESET,	GPIOF_OUT_INIT_LOW,	"touch_reset" },
};

static int ovation_touch_request_resources(struct device  *dev)
{
	int ret;

	/* Request GPIO lines */
	ret = gpio_request_array(ovation_touch_gpios, ARRAY_SIZE(ovation_touch_gpios));
	if (ret)
	{
		dev_err(dev, "%s: Could not get touch gpios\n", __func__);
		ret = -EBUSY;
		goto err_gpio_request;
	}
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);

	/* Request the required power supplies */
	ret = Vdd_LCD_CT_PEN_request_supply(NULL, "vtp");
	if(ret != 0)
	{
		dev_err(dev, "%s: Could not get touch supplies\n", __func__);
		goto err_regulator_get;
	}

	return 0;

err_regulator_get:
	gpio_free_array(ovation_touch_gpios, ARRAY_SIZE(ovation_touch_gpios));
err_gpio_request:
	return ret;
}

static int ovation_touch_release_resources(struct device  *dev)
{
	int ret;

	gpio_free_array(ovation_touch_gpios, ARRAY_SIZE(ovation_touch_gpios));

	/* Release the touch power supplies */
	ret = Vdd_LCD_CT_PEN_release_supply(NULL, "vtp");
	if(ret != 0)
	{
		dev_err(dev, "%s: Could not release touch supplies\n", __func__);
	}

	return 0;
}

static int ovation_touch_power_on(struct device  *dev)
{
	int ret;

	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);

	ret = Vdd_LCD_CT_PEN_enable(NULL, "vtp");

	msleep(10);

	/* Pull the nRESET line high after the power stabilises */
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 1);

	msleep(220);

	return 0;
}

static int ovation_touch_power_off(struct device  *dev)
{
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);
	msleep(2);

	Vdd_LCD_CT_PEN_disable(NULL, "vtp");

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE)
static struct ft5x06_platform_data ft5x06_platform_data = {
	.max_tx_lines = 38,
	.max_rx_lines = 26,
	.maxx = OVATION_TOUCH_X_RES,
	.maxy = OVATION_TOUCH_Y_RES,
	.flags = FLIP_DATA_FLAG | REVERSE_Y_FLAG,
	.reset_gpio = TOUCHPANEL_GPIO_RESET,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = FT_USE_TRACKING_ID,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 1,
	.request_resources = ovation_touch_request_resources,
	.release_resources = ovation_touch_release_resources,
	.power_on          = ovation_touch_power_on,
	.power_off         = ovation_touch_power_off,
};
#endif /* defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE) */

struct i2c_board_info __initdata ovation_i2c_3_boardinfo[] = {
#if defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE)
        {
		I2C_BOARD_INFO(FT_DEVICE_5x06_NAME, FT5x06_I2C_SLAVEADDRESS),
                .platform_data = &ft5x06_platform_data,
                .irq = OMAP_GPIO_IRQ(TOUCHPANEL_GPIO_IRQ),
        },
#endif /* defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE) */
};

int __init ovation_touch_init(void)
{
	printk(KERN_INFO "%s: Registering touch controller device\n", __func__);

	i2c_register_board_info(3, ovation_i2c_3_boardinfo, ARRAY_SIZE(ovation_i2c_3_boardinfo));

	return 0;
}
