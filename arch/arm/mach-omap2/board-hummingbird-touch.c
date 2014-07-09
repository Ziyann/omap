/*
 * arch/arm/mach-omap2/board-hummingbird-touch.c
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
#include "board-hummingbird.h"
#include "mux.h"
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE)
#include <linux/input/ft5x06.h>
#endif /* defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE) */

#define TOUCHPANEL_GPIO_IRQ     37
#define TOUCHPANEL_GPIO_RESET   39

#define HUMMINGBIRD_TOUCH_X_RES 900
#define HUMMINGBIRD_TOUCH_Y_RES 1440

static struct gpio hummingbird_touch_gpios[] = {
	{ TOUCHPANEL_GPIO_IRQ,		GPIOF_IN,		"touch_irq"   },
	{ TOUCHPANEL_GPIO_RESET,	GPIOF_OUT_INIT_LOW,	"touch_reset" },
};
static struct regulator *hummingbird_touch_vdd;
static struct regulator *hummingbird_touch_power;

static int hummingbird_touch_request_resources(struct device  *dev)
{
	int ret;

	/* Request GPIO lines */
	ret = gpio_request_array(hummingbird_touch_gpios, ARRAY_SIZE(hummingbird_touch_gpios));
	if (ret)
	{
		dev_err(dev, "%s: Could not get touch gpios\n", __func__);
		ret = -EBUSY;
		goto err_gpio_request;
	}
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);

	/* Request the required power supplies */
	hummingbird_touch_vdd = regulator_get(NULL, "touch_vdd");
	if(IS_ERR(hummingbird_touch_vdd))
	{
		dev_err(dev, "%s: Could not get touch io regulator\n", __func__);
		ret = -EBUSY;
		goto err_regulator_get;
	}
	hummingbird_touch_power = regulator_get(NULL, "vtp");
	if (IS_ERR(hummingbird_touch_power)) {
		dev_err(dev, "%s: Could not get touch power regulator\n", __func__);
		ret = -EBUSY;
		goto err_regulator_get_vtp;
	}

	return 0;

err_regulator_get_vtp:
	regulator_put(hummingbird_touch_vdd);
err_regulator_get:
	gpio_free_array(hummingbird_touch_gpios, ARRAY_SIZE(hummingbird_touch_gpios));
err_gpio_request:
	return ret;

}

static int hummingbird_touch_release_resources(struct device  *dev)
{

	regulator_put(hummingbird_touch_vdd);
	regulator_put(hummingbird_touch_power);
	gpio_free_array(hummingbird_touch_gpios, ARRAY_SIZE(hummingbird_touch_gpios));

	return 0;
}

static int hummingbird_touch_power_on(struct device  *dev)
{
	int ret;

	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);

	if (!IS_ERR(hummingbird_touch_power))
	{
		ret = regulator_enable(hummingbird_touch_power);
		if (ret)
		{
			dev_err(dev, "%s:Could not enable touch power regulator\n", __func__);
			return -EBUSY;
		}
	}
	else
	{
		dev_err(dev, "%s: Touch power regulator is not valid\n", __func__);
		return -ENODEV;

	}

	if(!IS_ERR(hummingbird_touch_vdd))
	{
		ret = regulator_enable(hummingbird_touch_vdd);
		if (ret)
		{
			regulator_disable(hummingbird_touch_power);
			dev_err(dev, "%s: Could not enable touch vdd regulator\n", __func__);
			return -EBUSY;
		}
	}
	else
	{
		regulator_disable(hummingbird_touch_power);
		dev_err(dev, "%s: Touch io regulator is not valid\n", __func__);
		return -ENODEV;
	}

	msleep(10);

	/* Pull the nRESET line high after the power stabilises */
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 1);

	msleep(220);

	return 0;
}

static int hummingbird_touch_power_off(struct device  *dev)
{

	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);
	msleep(2);

	if(!IS_ERR(hummingbird_touch_vdd))
	{
		regulator_disable(hummingbird_touch_vdd);
	}
	else
	{
		dev_err(dev, "%s: Touch io regulator is not valid\n", __func__);
		return -ENODEV;
	}


	if (!IS_ERR(hummingbird_touch_power))
	{
		regulator_disable(hummingbird_touch_power);
	}
	else
	{
		dev_err(dev, "%s: Touch power regulator is not valid\n", __func__);
		return -ENODEV;
	}

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE)
static struct ft5x06_platform_data ft5x06_platform_data = {
	.max_tx_lines = 32,
	.max_rx_lines = 20,
	.maxx = HUMMINGBIRD_TOUCH_X_RES,
	.maxy = HUMMINGBIRD_TOUCH_Y_RES,
	.flags =  REVERSE_Y_FLAG | REVERSE_X_FLAG,
	.reset_gpio = TOUCHPANEL_GPIO_RESET,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = FT_USE_TRACKING_ID,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 1,
	.request_resources = hummingbird_touch_request_resources,
	.release_resources = hummingbird_touch_release_resources,
	.power_on          = hummingbird_touch_power_on,
	.power_off         = hummingbird_touch_power_off,
};
#endif /* defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE) */

static struct i2c_board_info __initdata hummingbird_i2c_3_boardinfo[] = {
#if defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE)
        {
                I2C_BOARD_INFO(FT_DEVICE_5x06_NAME, FT5x06_I2C_SLAVEADDRESS),
                .platform_data = &ft5x06_platform_data,
                .irq = OMAP_GPIO_IRQ(TOUCHPANEL_GPIO_IRQ),
        },
#endif /* defined(CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_FT5X06_MODULE) */
};

int __init hummingbird_touch_init(void)
{
	printk(KERN_INFO "%s: Registering touch controller device\n", __func__);

	i2c_register_board_info(3, hummingbird_i2c_3_boardinfo, ARRAY_SIZE(hummingbird_i2c_3_boardinfo));

	return 0;
}
