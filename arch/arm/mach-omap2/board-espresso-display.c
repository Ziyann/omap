/* arch/arm/mach-omap2/board-espresso-display.c
 *
 * Copyright (C) 2011 Samsung Electronics Co, Ltd.
 *
 * Based on mach-omap2/board-tuna-display.c
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/omapfb.h>
#include <linux/platform_data/panel-ltn.h>

#include <plat/omap_hwmod.h>
#include <plat/android-display.h>

#include <video/omapdss.h>

#include "board-espresso.h"

#define GPIO_LED_BACKLIGHT_RESET	95
#define GPIO_LCD_EN			135
#define GPIO_LVDS_NSHDN		136

static int espresso_lcd_enable(struct omap_dss_device *dssdev)
{
	pr_debug("%s\n", __func__);
	gpio_set_value(GPIO_LCD_EN, 1);

	return 0;
}

static void espresso_lcd_disable(struct omap_dss_device *dssdev)
{
	pr_debug("%s\n", __func__);
	gpio_set_value(GPIO_LCD_EN, 0);
}

static struct ltn_panel_data espresso_panel_data = {
	.lvds_nshdn_gpio		= GPIO_LVDS_NSHDN,
	.led_backlight_reset_gpio	= GPIO_LED_BACKLIGHT_RESET,
	.backlight_gptimer_num		= 10,
	.brightness_table = {
		.platform_value	= {BRIGHTNESS_OFF, BRIGHTNESS_DIM, BRIGHTNESS_MIN,
				BRIGHTNESS_25, BRIGHTNESS_DEFAULT, BRIGHTNESS_MAX},
		.kernel_value	= { 0, 2, 3, 8, 40, 85 },
	},
};

static struct omap_dss_device espresso_lcd_device = {
	.name			= "lcd",
	.driver_name		= "ltn_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.data			= &espresso_panel_data,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.platform_enable	= espresso_lcd_enable,
	.platform_disable	= espresso_lcd_disable,
	.panel = {
		.timings	= {
			.x_res		= 1024,
			.y_res		= 600,
			.pixel_clock	= 56888, /* closest to 56000 */
			.hfp		= 186,
			.hsw		= 50,
			.hbp		= 210,
			.vfp		= 24,
			.vsw		= 10,
			.vbp		= 11,
		},
		.width_in_um	= 153600,
		.height_in_um	= 90000,
	},
};

static struct omap_dss_device espresso10_lcd_config = {
	.panel = {
		.timings	= {
			.x_res		= 1280,
			.y_res		= 800,
			.pixel_clock	= 69818, /* closest to 69000 */
			.hfp		= 16,
			.hsw		= 48,
			.hbp		= 64,
			.vfp		= 16,
			.vsw		= 3,
			.vbp		= 11,
		},
		.width_in_um	= 216960,
		.height_in_um	= 135600,
	},
};

static struct omap_dss_device *espresso_dss_devices[] = {
	&espresso_lcd_device,
};

static struct omap_dss_board_info espresso_dss_data = {
	.num_devices	= ARRAY_SIZE(espresso_dss_devices),
	.devices	= espresso_dss_devices,
	.default_device	= &espresso_lcd_device,
};

static struct sgx_omaplfb_config espresso_omaplfb_config[] = {
	{
		.tiler2d_buffers = 2,
		.swap_chain_length = 2,
	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	},
};

static struct sgx_omaplfb_platform_data espresso_omaplfb_plat_data = {
	.num_configs = ARRAY_SIZE(espresso_omaplfb_config),
	.configs = espresso_omaplfb_config,
};

static struct omapfb_platform_data espresso_fb_pdata = {
	.mem_desc = {
		.region_cnt = ARRAY_SIZE(espresso_omaplfb_config),
	},
};

static struct dsscomp_platform_data dsscomp_config_x7k = {
	.tiler1d_slotsz = SZ_16M,
};

void __init omap4_espresso_memory_display_init(void)
{
	if (board_is_espresso10())
		espresso_dss_data.devices[0]->panel = espresso10_lcd_config.panel;

	omap_android_display_setup(&espresso_dss_data,
				   &dsscomp_config_x7k,
				   &espresso_omaplfb_plat_data,
				   &espresso_fb_pdata);
}

void __init omap4_espresso_display_init(void)
{
	int ret;

	if (board_is_espresso10())
		espresso_panel_data.pwm_duty_max = 1600; /* 25kHz */
	else
		espresso_panel_data.pwm_duty_max = 1200; /* 32kHz */

	ret = gpio_request(GPIO_LCD_EN, "lcd_en");
	if (ret < 0)
		pr_err("%s: gpio_request %d failed!\n", __func__, GPIO_LCD_EN);

	gpio_direction_output(GPIO_LCD_EN, 1);

	omapfb_set_platform_data(&espresso_fb_pdata);

	omap_display_init(&espresso_dss_data);
}
