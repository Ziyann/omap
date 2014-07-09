/*
 * arch/arm/mach-omap2/board-44xx-tablet-panel.c
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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <video/omapdss.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <plat/vram.h>
#include <plat/omap_apps_brd_id.h>
#include <mach/board-4430-acclaim.h>

#include "board-acclaim.h"
#include "control.h"
#include "mux.h"


#define DEFAULT_BACKLIGHT_BRIGHTNESS    105
#define LCD_BL_PWR_EN_GPIO	38
#define LCD_CABC0_GPIO		44
#define LCD_CABC1_GPIO		45

static void acclaim4430_disp_backlight_init(void)
{
	int ret;

	ret = gpio_request(LCD_BL_PWR_EN_GPIO, "BL-PWR-EN");
	if (ret)
		pr_err("Cannot request backlight power enable gpio");
	gpio_direction_output(LCD_BL_PWR_EN_GPIO, 0);
}

static void acclaim4430_disp_backlight_setpower(struct omap_pwm_led_platform_data *pdata, int state)
{
        if (state)
                gpio_direction_output(LCD_BL_PWR_EN_GPIO,  1);
        else
                gpio_direction_output(LCD_BL_PWR_EN_GPIO,  0);
	gpio_direction_output(LCD_CABC0_GPIO, 0);
	gpio_direction_output(LCD_CABC1_GPIO, 0);
	printk("[BL set power] %d\n", state);
}

static struct omap_pwm_led_platform_data acclaim4430_disp_backlight_data = {
        .name            = "lcd-backlight",
        .pwm_module_id   = 11,
        .def_on          = 0,
        .set_power       = acclaim4430_disp_backlight_setpower,
        .def_brightness  = DEFAULT_BACKLIGHT_BRIGHTNESS,
};

static struct platform_device sdp4430_disp_led = {
        .name   =       "omap_pwm_led",
        .id     =       -1,
        .dev    = {
                .platform_data = &acclaim4430_disp_backlight_data,
        },
};

static struct regulator_consumer_supply acclaim_lcd_tp_supply[] = {
        { .supply = "vtp" },
        { .supply = "vlcd" },
};

static struct regulator_init_data acclaim_lcd_tp_vinit = {
        .constraints = {
                .min_uV = 3300000,
                .max_uV = 3300000,
                .valid_modes_mask = REGULATOR_MODE_NORMAL,
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
        },
        .num_consumer_supplies = 2,
        .consumer_supplies = acclaim_lcd_tp_supply,
};

static struct fixed_voltage_config acclaim_lcd_touch_reg_data = {
        .supply_name = "vdd_lcdtp",
        .microvolts = 3300000,
        .gpio = 36,
        .enable_high = 1,
        .enabled_at_boot = 1,
        .init_data = &acclaim_lcd_tp_vinit,
};

static struct platform_device acclaim_lcd_touch_regulator_device = {
        .name   = "reg-fixed-voltage",
        .id     = -1,
        .dev    = {
                .platform_data = &acclaim_lcd_touch_reg_data,
        },
};


static struct platform_device *sdp4430_devices[] __initdata = {
        &sdp4430_disp_led,
	&acclaim_lcd_touch_regulator_device,
};

static struct boxer_panel_data boxer_panel;

static struct omap_dss_device tablet_lcd_device = {
        .name                   = "boxerLCD",
        .driver_name            = "boxer_panel_drv",
        .type                   = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     = 24,
        .channel                = OMAP_DSS_CHANNEL_LCD2,
        .data                   = &boxer_panel,
};

static struct omap_dss_device tablet_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *tablet_dss_devices[] = {
	&tablet_lcd_device,
	&tablet_hdmi_device,
};

static struct omap_dss_board_info tablet_dss_data = {
	.num_devices	= ARRAY_SIZE(tablet_dss_devices),
	.devices	= tablet_dss_devices,
	.default_device	= &tablet_lcd_device,
};

static void tablet_lcd_init(void)
{
	u32 reg;

	/* Enable 3 lanes in DSI1 module, disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x1f << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
}

#define TABLET_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */
static struct omapfb_platform_data tablet_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = TABLET_FB_RAM_SIZE,
			},
		},
	},
};

static void sdp4430_panel_get_resource(void)
{
        int ret_val = 0;
//        ret_val = gpio_request(36, "BOXER LCD PWR EN");
//        if ( ret_val ){
//                printk("%s : Could not request lcd pwr enable\n",__FUNCTION__);
//        }

        ret_val = gpio_request(LCD_CABC0_GPIO, "BOXER CABC0");
        if ( ret_val ){
                printk( "%s : could not request CABC0\n",__FUNCTION__);
        }

        ret_val = gpio_request(LCD_CABC1_GPIO, "BOXER CABC1");
        if ( ret_val ) {
                printk("%s: could not request CABC1\n",__FUNCTION__);
        }
}

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias               = "boxer_disp_spi",
		.bus_num                = 4,    /* 4: McSPI4 */
		.chip_select            = 0,
		.max_speed_hz           = 375000,
		.platform_data          = &tablet_lcd_device,
	},
};



int __init acclaim_panel_init(void)
{
	sdp4430_panel_get_resource();
	acclaim4430_disp_backlight_init();
	tablet_lcd_init();
	/* disabling HDMI since Acclaim does not provide such support */
	omap_vram_set_sdram_vram(TABLET_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tablet_fb_pdata);

	omap_display_init(&tablet_dss_data);
	omap_mux_init_signal("abe_dmic_din2.dmtimer11_pwm_evt", OMAP_MUX_MODE5);
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	spi_register_board_info(sdp4430_spi_board_info,
			ARRAY_SIZE(sdp4430_spi_board_info));

	return 0;
}
