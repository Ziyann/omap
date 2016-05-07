/* Power support for Samsung Gerry Board.
 *
 * Copyright (C) 2011 SAMSUNG, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/twl.h>
#include <linux/power/max17042_battery.h>
#include <linux/power/smb347-charger.h>
#include <linux/battery.h>
#include <linux/irq.h>

#include <asm/system_info.h>

#include "board-espresso.h"

#define GPIO_TA_NCONNECTED	32
#define GPIO_TA_NCHG		142
#define GPIO_TA_EN		13
#define GPIO_FUEL_ALERT	44

#define GPIO_CHG_SDA		98
#define GPIO_CHG_SCL		99
#define GPIO_CHG_SDA_BBY	113
#define GPIO_CHG_SCL_BBY	110
#define GPIO_FUEL_SDA		62
#define GPIO_FUEL_SCL		61

#define IRQ_FUEL_ALERT		204

#define CHARGER_STATUS_FULL	0x1

#define HIGH_BLOCK_TEMP	500
#define HIGH_RECOVER_TEMP	420
#define LOW_BLOCK_TEMP		(-50)
#define LOW_RECOVER_TEMP	0

static struct i2c_gpio_platform_data espresso_gpio_i2c5_pdata = {
	.udelay = 10,
	.timeout = 0,
	.sda_pin = GPIO_CHG_SDA,
	.scl_pin = GPIO_CHG_SCL,
};

static struct platform_device espresso_gpio_i2c5_device = {
	.name = "i2c-gpio",
	.id = 5,
	.dev = {
		.platform_data = &espresso_gpio_i2c5_pdata,
	}
};

static struct i2c_gpio_platform_data espresso_gpio_i2c7_pdata = {
	.udelay = 3,
	.timeout = 0,
	.sda_pin = GPIO_FUEL_SDA,
	.scl_pin = GPIO_FUEL_SCL,
};

static struct platform_device espresso_gpio_i2c7_device = {
	.name = "i2c-gpio",
	.id = 7,
	.dev = {
		.platform_data = &espresso_gpio_i2c7_pdata,
	},
};

static char *espresso_battery[] = {
	"max17042_battery",
};

static struct smb347_charger_platform_data smb347_charger_pdata = {
	.battery_info = {
		.technology		= POWER_SUPPLY_TECHNOLOGY_LION,
		.voltage_max_design	= 4220000,
		.voltage_min_design	= 2500000,
	},
	.max_charge_current	= 2200000,
	.max_charge_voltage	= 4200000,
	.pre_charge_current	= 250000,
	.termination_current	= 250000,
	.mains_current_limit	= 1800000,
	.usb_hc_current_limit	= 1800000,
	.soft_cold_temp_limit	= SMB347_TEMP_USE_DEFAULT,
	.soft_hot_temp_limit	= 42,
	.hard_cold_temp_limit	= SMB347_TEMP_USE_DEFAULT,
	.hard_hot_temp_limit	= 50,
	.suspend_on_hard_temp_limit = true,
	.use_mains		= true,
	.use_usb		= true,
	.use_usb_otg		= false,
	.disable_automatic_recharge = false,
	.irq_gpio		= GPIO_TA_NCHG,
	.disable_stat_interrupts = true,
	.enable_control		= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.usb_mode_pin_ctrl	= false,
	.supplied_to		= espresso_battery,
	.num_supplicants	= ARRAY_SIZE(espresso_battery),
	.en_gpio		= GPIO_TA_EN,
};

static const __initdata struct i2c_board_info smb136_i2c[] = {
	{
		I2C_BOARD_INFO("smb136", 0x4D), /* 9A >> 1 */
	},
};

static const __initdata struct i2c_board_info smb347_i2c[] = {
	{
		I2C_BOARD_INFO("smb347", 0x0C >> 1),
		.platform_data = &smb347_charger_pdata,
	},
};

static struct max17042_platform_data max17042_pdata = {
	.enable_current_sense = true,
	.r_sns = 10000,
	.enable_por_init = false,
	.vmin = 2500000,
	.vmax = 4300000,
	.temp_min = -50,
	.temp_max = 500,
};

static const __initdata struct i2c_board_info max17042_i2c[] = {
	{
		I2C_BOARD_INFO("max17042", 0x36),
		.platform_data = &max17042_pdata,
		.irq = IRQ_FUEL_ALERT,
	},
};

int check_charger_type(void)
{
	int cable_type;
	short adc;

	if (gpio_is_valid(GPIO_TA_NCONNECTED) && gpio_get_value(GPIO_TA_NCONNECTED))
		return CABLE_TYPE_NONE;

	adc = omap4_espresso_get_adc(ADC_CHECK_1);
	cable_type = adc > CABLE_DETECT_VALUE ?
			CABLE_TYPE_AC :
			CABLE_TYPE_USB;

	pr_info("%s: Charger type is [%s], adc = %d\n",
		__func__,
		cable_type == CABLE_TYPE_AC ? "AC" : "USB",
		adc);

	return cable_type;
}

void __init omap4_espresso_charger_init(void)
{
	int ret;

	if (!gpio_is_valid(GPIO_TA_NCONNECTED))
		gpio_request(GPIO_TA_NCONNECTED, "TA_nCONNECTED");

	if (board_is_espresso10() && board_is_bestbuy_variant()) {
		espresso_gpio_i2c5_pdata.sda_pin = GPIO_CHG_SDA_BBY;
		espresso_gpio_i2c5_pdata.scl_pin = GPIO_CHG_SCL_BBY;
	}
	ret = platform_device_register(&espresso_gpio_i2c5_device);
	if (ret < 0)
		pr_err("%s: gpio_i2c5 device register fail\n", __func__);

	ret = platform_device_register(&espresso_gpio_i2c7_device);
	if (ret < 0)
		pr_err("%s: gpio_i2c7 device register fail\n", __func__);

	if (board_is_espresso10())
		i2c_register_board_info(5, smb347_i2c, ARRAY_SIZE(smb347_i2c));
	else
		i2c_register_board_info(5, smb136_i2c, ARRAY_SIZE(smb136_i2c));

	i2c_register_board_info(7, max17042_i2c, ARRAY_SIZE(max17042_i2c));
}
