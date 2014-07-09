/*
 * arch/arm/mach-omap2/board-44xx-hummingbird-sensors.c
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

#include <linux/i2c/bma180.h>
#include <linux/i2c/tsl2771.h>
#include <linux/i2c/mpu3050.h>

#include <plat/i2c.h>

#include "board-hummingbird.h"
#include "mux.h"

#define OMAP4_BMA180ACCL_GPIO		178

/* BMA180 Accelerometer Begin */
static struct bma180accel_platform_data bma180accel_platform_data = {
	.ctrl_reg0	= 0x11,
	.g_range	= BMA_GRANGE_2G,
	.bandwidth	= BMA_BW_10HZ,
	.mode		= BMA_MODE_LOW_NOISE,
	.bit_mode	= BMA_BITMODE_14BITS,
	.smp_skip	= 1,
	.def_poll_rate	= 200,
	.fuzz_x		= 25,
	.fuzz_y		= 25,
	.fuzz_z		= 25,
};

static void blaze_hummingbird_bma180accl_init(void)
{
	if (gpio_request(OMAP4_BMA180ACCL_GPIO, "Accelerometer") < 0) {
		pr_err("Accelerometer GPIO request failed\n");
		return;
	}
	gpio_direction_input(OMAP4_BMA180ACCL_GPIO);
}
/* BMA180 Accelerometer End */


static struct i2c_board_info __initdata blaze_hummingbird_i2c_bus4_sensor_info[] = {
	{
		I2C_BOARD_INFO("bma180_accel", 0x40),
		.platform_data = &bma180accel_platform_data,
	},
};

int __init hummingbird_sensor_init(void)
{
//	blaze_hummingbird_bma180accl_init();

//	i2c_register_board_info(4, blaze_hummingbird_i2c_bus4_sensor_info,
//		ARRAY_SIZE(blaze_hummingbird_i2c_bus4_sensor_info));

	return 0;
}

