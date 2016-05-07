/* arch/arm/mach-omap2/board-espresso-sdio.c
 *
 * Copyright (C) 2011 Samsung Electronics Co, Ltd.
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

#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <plat/irqs.h>
#include <plat/mmc.h>

#include "common.h"
#include "hsmmc.h"
#include "board-espresso.h"

static struct omap2_hsmmc_info espresso_mmc_info[] = {
	{
		.mmc		= 2,
		.nonremovable	= true,
		.caps		= MMC_CAP_8_BIT_DATA
				| MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50,
		.ocr_mask	= MMC_VDD_165_195,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
		.nonremovable	= false,
		.mmc_data	= &espresso_wifi_data,
	},
	{}	/* Terminator */
};

void __init omap4_espresso_sdio_init(void)
{
	omap4_twl6030_hsmmc_init(espresso_mmc_info);
}
