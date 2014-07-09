/*
 * arch/arm/mach-omap2/board-44xx-hummingbird-panel.c
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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/lp855x.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <video/omapdss.h>
#include <video/omap-panel-hydis.h>
#include <linux/regulator/consumer.h>
#include <video/omap-panel-dsi.h>
#include <linux/spi/spi.h>
#include <linux/memblock.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/maxim9606.h>

#include <plat/vram.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/dsscomp.h>
#include <plat/sgx_omaplfb.h>
#include <plat/android-display.h>

#include "board-hummingbird.h"
#include "control.h"
#include "mux.h"

#define LCD_BL_PWR_EN_GPIO	38
#define LCD_DCR_1V8_GPIO_EVT1B	27

#define INITIAL_BRT		0x3F
#define MAX_BRT			0xFF

#define HDMI_GPIO_CT_CP_HPD	60
#define HDMI_GPIO_HPD		63  /* Hot plug pin for HDMI */
#define HDMI_GPIO_LS_OE		81  /* Level shifter for HDMI */
#define GPIO_UART_DDC_SWITCH    182

#define HDMI_DDC_SCL_PULLUPRESX		24
#define HDMI_DDC_SDA_PULLUPRESX		28
#define HDMI_DDC_DISABLE_PULLUP		1
#define HDMI_DDC_ENABLE_PULLUP		0

#define TWL6030_TOGGLE3        	0x92
#define LED_PWM1ON             	0x00
#define LED_PWM1OFF            	0x01

static char boot_fb[51];
static __init int get_boot_fb(char *str)
{
	strncpy(boot_fb, str, sizeof(boot_fb));
	boot_fb[sizeof(boot_fb) - 1] = '\0';
	return 0;
}
early_param("boot.fb", get_boot_fb);

static char display[51];
static __init int get_display_vendor(char *str)
{
	strncpy(display, str, sizeof(display));
	display[sizeof(display) - 1] = '\0';
	return 0;
}
early_param("display.vendor", get_display_vendor);

static struct regulator *hummingbird_lcd_power;
static struct regulator *hummingbird_bl_i2c_pullup_power;
static bool first_boot = true;

struct omap_tablet_panel_data {
	struct omap_dss_board_info *board_info;
	struct dsscomp_platform_data *dsscomp_data;
	struct sgx_omaplfb_platform_data *omaplfb_data;
};

static struct dsscomp_platform_data dsscomp_config_hummingbird = {
	.tiler1d_slotsz = (SZ_16M + SZ_2M + SZ_8M + SZ_1M),
};


#ifdef CONFIG_FB_OMAP2_NUM_FBS
#define OMAPLFB_NUM_DEV CONFIG_FB_OMAP2_NUM_FBS
#else
#define OMAPLFB_NUM_DEV 1
#endif

struct lp855x_rom_data bl_rom_data[] = {
	{
		.addr = 0xA9,
		.val  = 0x60
	},
};

static int _request_resources(void)
{
	if (!hummingbird_lcd_power) {
		hummingbird_lcd_power = regulator_get(NULL, "vlcd");
	}

	if (IS_ERR(hummingbird_lcd_power)) {
		pr_err("%s: failed to get regulator vlcd", __func__);
		return -EBUSY;
	}

	if (!hummingbird_bl_i2c_pullup_power) {
		hummingbird_bl_i2c_pullup_power = regulator_get(NULL, "bl_i2c_pup");
	} 	

	if (IS_ERR(hummingbird_bl_i2c_pullup_power)) {
		pr_err("%s: failed to get regulator bl_i2c_pup", __func__);
		return -EBUSY;
	}

	if (strncmp(display, "AUO", 3) == 0) {
		regulator_enable(hummingbird_lcd_power);
	}

	return 0;
}

static int hummingbird_bl_request_resources(struct device *dev)
{
	int ret;

	ret = gpio_request(LCD_BL_PWR_EN_GPIO, "BL-PWR-EN");

	if (ret) {
		pr_err("Cannot request backlight power enable gpio");
		return -EBUSY;
	}

	_request_resources();

	return 0;
}

static int hummingbird_bl_release_resources(struct device *dev)
{
	gpio_free(LCD_BL_PWR_EN_GPIO);
	return 0;
}

static int hummingbird_bl_power_on(struct device *dev)
{
	int ret = 0;

	regulator_enable(hummingbird_bl_i2c_pullup_power);
	gpio_set_value(LCD_BL_PWR_EN_GPIO, 1);
	msleep(20);
	
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM1ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0XFF, LED_PWM1OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x06, TWL6030_TOGGLE3);

	return ret;
}

static int hummingbird_bl_power_off(struct device *dev)
{
	gpio_set_value(LCD_BL_PWR_EN_GPIO, 0);

	if (regulator_is_enabled(hummingbird_bl_i2c_pullup_power))
		regulator_disable(hummingbird_bl_i2c_pullup_power);

	msleep(200);

	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x01, TWL6030_TOGGLE3);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x07, TWL6030_TOGGLE3);
	
	return 0;
}

static int lg_maxim9606_request_resources(struct device *dev)
{
	return _request_resources();
}

static int _release_resources(void)
{
	regulator_put(hummingbird_lcd_power);
	regulator_put(hummingbird_bl_i2c_pullup_power);

	omap_mux_init_signal("i2c3_scl.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c3_sda.safe_mode", OMAP_PIN_INPUT);

	return 0;
}

static int lg_maxim9606_release_resources(struct device *dev)
{
	return _release_resources();
}

static void _enable_supplies(ulong delay)
{
	bool safemode = false;

	if (!regulator_is_enabled(hummingbird_bl_i2c_pullup_power)) {
		omap_mux_init_signal("i2c3_scl.safe_mode", OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c3_sda.safe_mode", OMAP_PIN_INPUT);
		safemode = true;
	}

	// enable i2c level shifter so tcon can talk to eeprom
	regulator_enable(hummingbird_bl_i2c_pullup_power);
	msleep(6);

	regulator_enable(hummingbird_lcd_power);

	msleep(delay);

	if (safemode) {
		omap_mux_init_signal("i2c3_scl.i2c3_scl", OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c3_sda.i2c3_sda", OMAP_PIN_INPUT);
	}
}

static void _disable_supplies(void)
{
	if (regulator_is_enabled(hummingbird_bl_i2c_pullup_power)) {
		regulator_disable(hummingbird_bl_i2c_pullup_power);
		msleep(10);
	}

	if (regulator_is_enabled(hummingbird_lcd_power)) {
		regulator_disable(hummingbird_lcd_power);
	}
}

static int lg_maxim9606_power_on(struct device *dev)
{
	_enable_supplies(74);
	return 0;
}

static int lg_maxim9606_power_off(struct device *dev)
{
	_disable_supplies();
	return 0;
}

static struct lp855x_platform_data lp8556_pdata = {
	.name = "lcd-backlight",
	.mode = REGISTER_BASED,
	.device_control = LP8556_COMB1_CONFIG | LP8556_FAST,
	.initial_brightness = INITIAL_BRT,
	.max_brightness = MAX_BRT,
	.led_setting = PS_MODE_4P4D | PWM_FREQ6916HZ,
	.boost_freq = BOOST_FREQ625KHZ,
	.nonlinearity_factor = 30,
	.load_new_rom_data = 1,
	.size_program = 1,
	.rom_data = bl_rom_data,
	.request_resources = hummingbird_bl_request_resources,
	.release_resources = hummingbird_bl_release_resources,
	.power_on = hummingbird_bl_power_on,
	.power_off = hummingbird_bl_power_off,
};

static struct maxim9606_platform_data maxim9606_pdata = {
	.power_on 	= lg_maxim9606_power_on,
	.power_off 	= lg_maxim9606_power_off,
	.request_resources	= lg_maxim9606_request_resources,
	.release_resources	= lg_maxim9606_release_resources,	
};

static struct i2c_board_info __initdata bl_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("maxim9606", 0x74),
		.platform_data = &maxim9606_pdata,
	},

	{
		I2C_BOARD_INFO("lp8556", 0x2C),
		.platform_data = &lp8556_pdata,
	},
};

static struct gpio hummingbird_hdmi_gpios[] = {
	{HDMI_GPIO_CT_CP_HPD,  GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_hpd"   },
	{HDMI_GPIO_LS_OE,      GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_ls_oe" },
};

static void __init hummingbird_lcd_mux_init(void)
{
	omap_mux_init_signal("sdmmc5_dat2.mcspi2_cs1",
				OMAP_PIN_OUTPUT);
//	omap_mux_init_signal("sdmmc5_dat0.mcspi2_somi",
//				OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_cmd.mcspi2_simo",
				OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdmmc5_clk.mcspi2_clk",
				OMAP_PIN_OUTPUT);

	/* reset mipi to lvds bridge */
	omap_mux_init_signal("sdmmc5_dat0.gpio_147",
				OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dpm_emu16.gpio_27",
			OMAP_PIN_OUTPUT);
}

static void __init hummingbird_hdmi_mux_init(void)
{
	u32 r;
	int status;
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	if (system_rev > HUMMINGBIRD_EVT0) {
		omap_mux_init_signal("hdmi_ddc_scl.safe_mode",
			OMAP_PIN_INPUT);
		omap_mux_init_signal("hdmi_ddc_sda.safe_mode",
			OMAP_PIN_INPUT);
	} else {
		omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT);
		omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT);
	}

	/* Disable strong pullup on DDC lines using unpublished register */
	r = ((HDMI_DDC_DISABLE_PULLUP << HDMI_DDC_SCL_PULLUPRESX) |
		(HDMI_DDC_DISABLE_PULLUP << HDMI_DDC_SDA_PULLUPRESX));
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(HDMI_GPIO_HPD);

	status = gpio_request_array(hummingbird_hdmi_gpios,
			ARRAY_SIZE(hummingbird_hdmi_gpios));
	if (status)
		pr_err("%s: Cannot request HDMI GPIOs %x \n", __func__, status);
}

static int lg_enable_dsi(struct omap_dss_device *dssdev)
{
	_enable_supplies(14);
	
	gpio_direction_output(LCD_DCR_1V8_GPIO_EVT1B, 1);
	return 0;
}

static void lg_disable_dsi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(LCD_DCR_1V8_GPIO_EVT1B, 0);	
	msleep(100);
	_disable_supplies();
}

static int auo_enable_dsi(struct omap_dss_device *dssdev)
{
	bool safemode = false;

	if (!regulator_is_enabled(hummingbird_lcd_power)) {
		omap_mux_init_signal("i2c3_scl.safe_mode", OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c3_sda.safe_mode", OMAP_PIN_INPUT);
		safemode = true;
	}

	// enable i2c level shifter so tcon can talk to eeprom
	regulator_enable(hummingbird_bl_i2c_pullup_power);
	regulator_enable(hummingbird_lcd_power);

	msleep(200);

	if (safemode) {
		omap_mux_init_signal("i2c3_scl.i2c3_scl", OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c3_sda.i2c3_sda", OMAP_PIN_INPUT);
	}

	gpio_direction_output(LCD_DCR_1V8_GPIO_EVT1B, 0);

	return 0;
}

static void auo_disable_dsi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(LCD_DCR_1V8_GPIO_EVT1B, 0);	
	msleep(100);

	if (first_boot) {
		regulator_disable(hummingbird_lcd_power);
		first_boot = false;
	}

	_disable_supplies();
}

static int hummingbird_enable_hdmi(struct omap_dss_device *dssdev)
{
	if (system_rev > HUMMINGBIRD_EVT0) {
		omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT);
		omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT);

		omap_mux_init_signal("i2c2_scl.safe_mode",
			OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c2_sda.safe_mode",
			OMAP_PIN_INPUT);
	} else
		gpio_direction_output(GPIO_UART_DDC_SWITCH, 0);

	return 0;
}

static void hummingbird_disable_hdmi(struct omap_dss_device *dssdev)
{
	if (system_rev > HUMMINGBIRD_EVT0) {
		omap_mux_init_signal("hdmi_ddc_scl.safe_mode",
			OMAP_PIN_INPUT);
		omap_mux_init_signal("hdmi_ddc_sda.safe_mode",
			OMAP_PIN_INPUT);

		omap_mux_init_signal("i2c2_sda.uart1_tx",
			OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c2_scl.uart1_rx",
			OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE |
			OMAP_PIN_OFF_INPUT_PULLUP);
	} else
		gpio_direction_output(GPIO_UART_DDC_SWITCH, 1);
}

static DSI_FPS_DATA(30_novatek, 30, 125, 5, 27, 7, 18, 9, 11, 15, 11);
static DSI_FPS_DATA(48_novatek, 48, 200, 8, 43, 11, 12, 14, 16, 24, 15);
static DSI_FPS_DATA(50_novatek, 50, 210, 9, 45, 11, 13, 14, 18, 25, 16);
static DSI_FPS_DATA(60_novatek, 60, 250, 10, 53, 13, 14, 16, 21, 29, 17);

static struct panel_dsi_fps_data *dsi_fps_data_novatek[] = {
	&dsi_fps_data_60_novatek,
	&dsi_fps_data_50_novatek,
	&dsi_fps_data_48_novatek,
	&dsi_fps_data_30_novatek,
	NULL,
};

static struct omap_dss_device hummingbird_lcd_device_novatek = {
	.name                   = "lcd",
	.driver_name            = "novatek-panel",
	.type                   = OMAP_DISPLAY_TYPE_DSI,
	.data                   = dsi_fps_data_novatek,
	.phy.dsi                = {
		.clk_lane       = 1,
		.clk_pol        = 0,
		.data1_lane     = 2,
		.data1_pol      = 0,
		.data2_lane     = 3,
		.data2_pol      = 0,
		.data3_lane     = 4,
		.data3_pol      = 0,
		.data4_lane     = 5,
		.data4_pol      = 0,
		.type 		= OMAP_DSS_DSI_TYPE_VIDEO_MODE,
		.line_bufs	= 2,
	},

	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 1,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
		},

		.dsi = {
			.regn           = 24,
			.regm           = 250,
			.regm_dispc     = 9,
			.regm_dsi       = 5,
			.lp_clk_div     = 8,
			.offset_ddr_clk = 0,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
			.tlpx	= 10,
			.tclk = {
				.zero	 = 53,
				.prepare = 13,
				.trail	 = 14,
			},
			.ths = {
				.zero	 = 21,
				.prepare = 16,
				.exit	 = 29,
				.trail	 = 17,
			},

		},
	},

	.ctrl = {
		.pixel_size 	= 18,
		.dither		= true,
	},

	.panel = {
		.timings = {
			.x_res		= 900,
			.y_res		= 1440,
			.pixel_clock 	= 88888,
			.hfp		= 32,
			.hsw		= 28,
			.hbp		= 48,
			.vfp		= 14,
			.vsw		= 18,
			.vbp		= 3,
		},

		.acbi 		= 40,
		.acb		= 0,
		.width_in_um	= 94230,
		.height_in_um	= 150770,
	},

	.channel = OMAP_DSS_CHANNEL_LCD,
	.skip_init = true,

	.platform_enable = lg_enable_dsi,
	.platform_disable = lg_disable_dsi,
};

static DSI_FPS_DATA(30_orise, 30, 130, 6, 28, 7, 9, 10, 11, 16, 12);
static DSI_FPS_DATA(48_orise, 48, 208, 9, 45, 11, 12, 14, 18, 25, 15);
static DSI_FPS_DATA(50_orise, 50, 217, 9, 47, 12, 13, 15, 18, 26, 16);
static DSI_FPS_DATA(60_orise, 60, 260, 11, 56, 14, 15, 17, 22, 31, 18);

static struct panel_dsi_fps_data *dsi_fps_data_orise[] = {
	&dsi_fps_data_60_orise,
	&dsi_fps_data_50_orise,
	&dsi_fps_data_48_orise,
	&dsi_fps_data_30_orise,
	NULL,
};

static struct omap_dss_device hummingbird_lcd_device_orise = {
	.name                   = "lcd",
	.driver_name            = "orise-panel",
	.type                   = OMAP_DISPLAY_TYPE_DSI,
	.data                   = dsi_fps_data_orise,
	.phy.dsi                = {
		.clk_lane       = 1,
		.clk_pol        = 0,
		.data1_lane     = 2,
		.data1_pol      = 0,
		.data2_lane     = 3,
		.data2_pol      = 0,
		.data3_lane     = 4,
		.data3_pol      = 0,
		.data4_lane     = 5,
		.data4_pol      = 0,

		.type 		= OMAP_DSS_DSI_TYPE_VIDEO_MODE,
		.line_bufs	= 2,
	},

	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 1,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
		},

		.dsi = {
			.regn           = 24,
			.regm           = 260,
			.regm_dispc     = 9,
			.regm_dsi       = 5,
			.lp_clk_div     = 9,
			.offset_ddr_clk = 0,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
			.tlpx	= 11,
			.tclk = {
				.zero	 = 55,
				.prepare = 14,
				.trail	 = 15,
			},
			.ths = {
				.zero	 = 22,
				.prepare = 17,
				.exit	 = 31,
				.trail	 = 17,
			},

		},
	},

	.ctrl = {
		.pixel_size 	= 18,
		.dither		= true,
	},

	.panel = {
		.timings = {
			.x_res		= 900,
			.y_res		= 1440,
			.pixel_clock 	= 92444,
			.hfp		= 76,
			.hsw		= 40,
			.hbp		= 40,
			.vfp		= 10,
			.vsw		= 9,
			.vbp		= 1,
		},

		.acbi 		= 40,
		.acb		= 0,
		.width_in_um	= 94230,
		.height_in_um	= 150770,
	},


	.channel = OMAP_DSS_CHANNEL_LCD,
	.skip_init = true,

	.platform_enable = auo_enable_dsi,
	.platform_disable = auo_disable_dsi,
};

static struct omap_dss_device hummingbird_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
	.platform_enable	= hummingbird_enable_hdmi,
	.platform_disable	= hummingbird_disable_hdmi,
};

static struct omap_dss_device *hummingbird_dss_devices[] = {
	&hummingbird_lcd_device_novatek,
	&hummingbird_hdmi_device,
};

static struct omap_dss_board_info hummingbird_dss_data = {
	.num_devices	= ARRAY_SIZE(hummingbird_dss_devices),
	.devices	= hummingbird_dss_devices,
	.default_device	= &hummingbird_lcd_device_novatek,
};

static struct sgx_omaplfb_config omaplfb_config_hummingbird[OMAPLFB_NUM_DEV] = {
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	}
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_hummingbird = {
	.num_configs = OMAPLFB_NUM_DEV,
	.configs = omaplfb_config_hummingbird,
};

static struct omap_tablet_panel_data panel_data_hummingbird = {
	.board_info = &hummingbird_dss_data,
	.dsscomp_data = &dsscomp_config_hummingbird,
	.omaplfb_data = &omaplfb_plat_data_hummingbird,
};

static void hummingbird_lcd_init(void)
{
	u32 reg;

	gpio_request(LCD_DCR_1V8_GPIO_EVT1B, "lcd_dcr");

	/* Enable 5 lanes in DSI1 module, disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x1f << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
}

static struct omapfb_platform_data hummingbird_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
	},
	.boot_fb_addr = 0,
	.boot_fb_size = 0,
};

void hummingbird_android_display_setup(struct omap_ion_platform_data *ion)
{
	u32 boot_fb_addr = simple_strtol(boot_fb, NULL, 16);
	if (boot_fb_addr) {
		if (memblock_remove(boot_fb_addr, 900*1440*4) < 0) {
			pr_err("%s: failed to reserve VRAM - no memory\n", __FUNCTION__);
		} else {
			hummingbird_fb_pdata.boot_fb_addr = boot_fb_addr;
			hummingbird_fb_pdata.boot_fb_size = 900*1440*4;
		}
	}

	if (system_rev >= HUMMINGBIRD_EVT0B) {
		if (strncmp(display, "LG", 2) == 0) {
			pr_info("selecting Novatek panel@18-bit");
			hummingbird_dss_devices[0] = &hummingbird_lcd_device_novatek;
			hummingbird_dss_data.default_device = &hummingbird_lcd_device_novatek;
		} else if (strncmp(display, "AUO", 3) == 0) {
			pr_info("selecting Orise panel@18-bit");
			hummingbird_dss_devices[0] = &hummingbird_lcd_device_orise;
			hummingbird_dss_data.default_device = &hummingbird_lcd_device_orise;
		} else {
			pr_info("unknown display vendor, selecting LG panel@18-bit");
			hummingbird_dss_devices[0] = &hummingbird_lcd_device_novatek;
			hummingbird_dss_data.default_device = &hummingbird_lcd_device_novatek;
		}
	}

	omap_android_display_setup(panel_data_hummingbird.board_info,
				NULL,
				NULL,
				&hummingbird_fb_pdata, ion);
}

int __init hummingbird_panel_init(void)
{
	hummingbird_lcd_init();
	hummingbird_hdmi_mux_init();
	hummingbird_lcd_mux_init();

	omapfb_set_platform_data(&hummingbird_fb_pdata);
		
	omap_display_init(&hummingbird_dss_data);

	if (strncmp(display, "AUO", 3) == 0) {
		i2c_register_board_info(3, &bl_i2c_boardinfo[1], 
					ARRAY_SIZE(bl_i2c_boardinfo) - 1);
	} else {
		i2c_register_board_info(3, bl_i2c_boardinfo,
					ARRAY_SIZE(bl_i2c_boardinfo));
	}
	return 0;
}
