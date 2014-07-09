/*
 * arch/arm/mach-omap2/board-44xx-ovation-panel.c
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
#include <video/omap-panel-dsi.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/memblock.h>
#include <linux/i2c/twl.h>

#include <plat/vram.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/android-display.h>

#include "board-ovation.h"
#include "control.h"
#include "mux.h"

#define LCD_BL_PWR_EN_GPIO	38
#define LCD_DCR_1V8_GPIO	153
#define LCD_DCR_1V8_GPIO_EVT1B	27

/* For board version >= preevt1b */
#define LCD_CM_EN		145

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

int Vdd_LCD_CT_PEN_request_supply(struct device *dev, const char *supply_name);
int Vdd_LCD_CT_PEN_enable(struct device *dev, const char *supply_name);
int Vdd_LCD_CT_PEN_disable(struct device *dev, const char *supply_name);
int Vdd_LCD_CT_PEN_release_supply(struct device *dev, const char *supply_name);
static int ovation_request_lcd_resources(void);

static char boot_fb[51];
static __init int get_boot_fb(char *str)
{
	strncpy(boot_fb, str, sizeof(boot_fb));
	boot_fb[sizeof(boot_fb) - 1] = '\0';
	return 0;
}
early_param("boot.fb", get_boot_fb);

static struct regulator *ovation_bl_i2c_pullup_power;
static int lcd_supply_requested;
static bool first_boot = true;

static struct lp855x_rom_data ovation_bl_rom_data[] = {
	/*reg 0xA7 bits [1..0] set boost current limit to 1,2A */
	{
		.addr = 0xA7,
		.val  = 0xFD,
	},
	{
		.addr = 0xA9,
		.val  = 0x80
	},
	{
		.addr = 0xA4,
		.val  = 0x02
	},
};

static int ovation_bl_request_resources(struct device *dev)
{
	int ret;

	ret = gpio_request(LCD_BL_PWR_EN_GPIO, "BL-PWR-EN");

	if (ret) {
		pr_err("Cannot request backlight power enable gpio");
		return -EBUSY;
	}

	gpio_direction_output(LCD_BL_PWR_EN_GPIO, 1);

	//ovation_bl_i2c_pullup_power = regulator_get(dev, "touch");
	ovation_bl_i2c_pullup_power = regulator_get(NULL, "bl_i2c_pup");

	if (IS_ERR(ovation_bl_i2c_pullup_power)) {
		dev_err(dev, "%s: Could not get backlight "
		"i2c-3 pullup power regulator\n", __func__);
		gpio_free(LCD_BL_PWR_EN_GPIO);
		return -EBUSY;
	}

	// enable the LCD regulator early on first boot to maintain boot splash image
	if (first_boot == true) {
		if(lcd_supply_requested == 0)
		{
			ovation_request_lcd_resources(); //dssdev);
			lcd_supply_requested = 1;
		}

		ret = Vdd_LCD_CT_PEN_enable(NULL, "vlcd");

		msleep(15);
	}

	return ret;
}

static int ovation_bl_release_resources(struct device *dev)
{
	gpio_free(LCD_BL_PWR_EN_GPIO);
	regulator_put(ovation_bl_i2c_pullup_power);

	return 0;
}

static int ovation_bl_power_on(struct device *dev)
{
	int ret = -EBUSY;

	msleep(200);
	if (!IS_ERR(ovation_bl_i2c_pullup_power)) {
		ret = regulator_enable(ovation_bl_i2c_pullup_power);
		if (ret) {
			pr_err("%s:Could not enable bl i2c-3 pullup power regulator\n",
				__func__);
		}
	} else {
		pr_err("%s: touch power regulator is not valid\n", __func__);
	}

	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM1ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0XFF, LED_PWM1OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x06, TWL6030_TOGGLE3);

	msleep(20); // T14 timing

	gpio_set_value(LCD_BL_PWR_EN_GPIO, 1);

	msleep(100);

	return ret;
}

static int ovation_bl_power_off(struct device *dev)
{
	gpio_set_value(LCD_BL_PWR_EN_GPIO, 0);

	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x01, TWL6030_TOGGLE3);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x07, TWL6030_TOGGLE3);

	if(!IS_ERR(ovation_bl_i2c_pullup_power)) {
		regulator_disable(ovation_bl_i2c_pullup_power);
	} else {
		dev_err(dev,
			"%s: touch vdd regulator is not valid\n", __func__);
			return -ENODEV;
	}

	msleep(200); // T6 timing
	return 0;
}

static struct lp855x_platform_data lp8556_pdata = {
	.name = "lcd-backlight",
	.mode = REGISTER_BASED,
	.device_control = LP8556_COMB1_CONFIG,
	.initial_brightness = INITIAL_BRT,
	.max_brightness = MAX_BRT,
	.led_setting = PS_MODE_5P5D|PWM_FREQ6916HZ,
	.boost_freq = BOOST_FREQ625KHZ,
	.nonlinearity_factor = 30,
	.load_new_rom_data = 1,
	.size_program = 2,
	.rom_data = ovation_bl_rom_data,
	.request_resources = ovation_bl_request_resources,
	.release_resources = ovation_bl_release_resources,
	.power_on = ovation_bl_power_on,
	.power_off = ovation_bl_power_off,
};

static struct i2c_board_info __initdata bl_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("lp8556", 0x2C),
		.platform_data = &lp8556_pdata,
	},
};

static struct gpio ovation_hdmi_gpios[] = {
	{HDMI_GPIO_CT_CP_HPD,  GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_hpd"   },
	{HDMI_GPIO_LS_OE,      GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_ls_oe" },
};

struct omap_tablet_panel_data {
	struct omap_dss_board_info *board_info;
	struct dsscomp_platform_data *dsscomp_data;
	struct sgx_omaplfb_platform_data *omaplfb_data;
};

static struct dsscomp_platform_data dsscomp_config_ovation_wuxga = {
	.tiler1d_slotsz = (SZ_16M + SZ_2M + SZ_8M + SZ_1M),
};

#ifdef CONFIG_FB_OMAP2_NUM_FBS
#define OMAPLFB_NUM_DEV CONFIG_FB_OMAP2_NUM_FBS
#else
#define OMAPLFB_NUM_DEV 1
#endif

static struct sgx_omaplfb_config omaplfb_config_ovation_wuxga[OMAPLFB_NUM_DEV] = {
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	}
};


static void ovation_hdmi_mux_init(void)
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
	if (system_rev > OVATION_EVT1A) {
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

	if (system_rev >= OVATION_EVT1B) {
		omap_mux_init_signal("sdmmc5_clk.gpio_145",
			OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dpm_emu16.gpio_27",
			OMAP_PIN_OUTPUT);
	} else {
		/* EVT1a compatibility */
		omap_mux_init_signal("mcspi4_somi.gpio_153",
			OMAP_PIN_OUTPUT);
		omap_mux_init_signal("usbb2_ulpitll_stp.dispc2_data23",
			OMAP_PIN_INPUT);
	}

	/* Disable strong pullup on DDC lines using unpublished register */
	r = ((HDMI_DDC_DISABLE_PULLUP << HDMI_DDC_SCL_PULLUPRESX) |
		(HDMI_DDC_DISABLE_PULLUP << HDMI_DDC_SDA_PULLUPRESX));
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(HDMI_GPIO_HPD);

	status = gpio_request_array(ovation_hdmi_gpios,
			ARRAY_SIZE(ovation_hdmi_gpios));
	if (status)
		pr_err("%s: Cannot request HDMI GPIOs %x \n", __func__, status);
}

static void __init ovation_panel_init_dpi(void)
{
	if (system_rev >= OVATION_EVT1B) {
		u32 reg;

		gpio_request(LCD_DCR_1V8_GPIO_EVT1B, "lcd_dcr");
		gpio_request(LCD_CM_EN, "lcd_cm_en");

		/* Enable 5 lanes in DSI1 module, disable pull down */
		reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
		reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
		reg |= 0x1f << OMAP4_DSI1_LANEENABLE_SHIFT;
		reg &= ~OMAP4_DSI1_PIPD_MASK;
		reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
		omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	} else {
		gpio_request(LCD_DCR_1V8_GPIO, "lcd_dcr");
		gpio_direction_output(LCD_DCR_1V8_GPIO, 1);
	}
}

static int ovation_request_lcd_resources(void) 
{
	int ret;

	/* Request the display power supply */
	ret = Vdd_LCD_CT_PEN_request_supply(NULL, "vlcd");
	if(ret != 0)
	{
		pr_err("%s: Could not get lcd supply\n", __func__);
	}
	return ret;
}

static int ovation_release_lcd_resources(struct omap_dss_device *dssdev)
{
	int ret;

	/* Release the display power supply */
	ret = Vdd_LCD_CT_PEN_release_supply(NULL, "vlcd");
	if(ret != 0)
	{
		pr_err("%s: Could not release lcd supply\n", __func__);
	}
	return ret;
}

static int ovation_enable_lcd(struct omap_dss_device *dssdev)
{
	int ret;
	// on the first boot the LCD regulator was enabled earlier
	if (first_boot == false) {
		if(lcd_supply_requested == 0)
		{
			ovation_request_lcd_resources();
			lcd_supply_requested = 1;
		}

		ret = Vdd_LCD_CT_PEN_enable(NULL, "vlcd");

		msleep(100); // T2 timing
	} else {
		first_boot = false;
	}

	gpio_direction_output(LCD_DCR_1V8_GPIO_EVT1B, 1);	
	gpio_direction_output(LCD_CM_EN, 1);
	return 0;
}

static void ovation_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(LCD_CM_EN, 0);
	gpio_direction_output(LCD_DCR_1V8_GPIO_EVT1B, 0);	

	Vdd_LCD_CT_PEN_disable(NULL, "vlcd");
}

static int ovation_enable_hdmi(struct omap_dss_device *dssdev)
{
	if (system_rev > OVATION_EVT1A) {
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

static void ovation_disable_hdmi(struct omap_dss_device *dssdev)
{
	if (system_rev > OVATION_EVT1A) {
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

static struct panel_dsi_data dsi_data = {
	.x_res		= 1920,
	.y_res		= 1280,

	.pixel_clock 	= 93600,

	.hfp		= 32,
	.hsw		= 4,
	.hbp		= 32,

	.vfp		= 3,
	.vsw		= 1,
	.vbp		= 5,

	.width_in_um    = 190080,
	.height_in_um   = 126720,
};

static int pwm_level;

static int ovation_set_pwm_bl(struct omap_dss_device *dssdev, int level)
{
	u8 brightness = 0;

	if (level) {
		if (level >= 255) {
			brightness = 0x7f;
		} else {
			brightness = (~(level/2)) & 0x7f;
		}

		twl_i2c_write_u8(TWL_MODULE_PWM, brightness, LED_PWM1ON);
		twl_i2c_write_u8(TWL_MODULE_PWM, 0XFF, LED_PWM1OFF);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x06, TWL6030_TOGGLE3);
	} else {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x01, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x07, TWL6030_TOGGLE3);
	}

	pwm_level = level;
	return 0;
}

static int ovation_get_pwm_bl(struct omap_dss_device *dssdev)
{
	return pwm_level;
}

static DSI_FPS_DATA(48, 48, 288, 14, 74, 18, 19, 22, 29, 41, 22);
static DSI_FPS_DATA(50, 50, 300, 15, 77, 19, 20, 23, 30, 42, 23);
static DSI_FPS_DATA(60, 60, 348, 17, 89, 22, 23, 26, 35, 49, 26);

static struct panel_dsi_fps_data *dsi_fps_data[] = {
	&dsi_fps_data_60,
	&dsi_fps_data_50,
	&dsi_fps_data_48,
	NULL,
};

static struct omap_dsi_timings dsi_timings_samsung = {
	.hbp		= 0,
	.hfp		= 24,
	.hsa		= 0,
	.vbp		= 9,
	.vfp		= 10,
	.vsa		= 1,
	.vact		= 1280,
	.tl		= 1107,
	.hsa_hs_int	= 0,
	.hfp_hs_int	= 0,
	.hbp_hs_int	= 0,
	.hsa_lp_int	= 130,
	.hfp_lp_int	= 223,
	.hbp_lp_int	= 59,
	.bl_lp_int	= 1038,
	.bl_hs_int	= 1038,
	.exit_lat	= 21,
	.enter_lat	= 23,
};

static struct omap_video_timings dispc_timings_samsung = {
	.x_res		= 1920,
	.y_res		= 1280,
	.hfp		= 4,
	.hsw		= 5,
	.hbp		= 39,
	.vfp		= 9,
	.vsw		= 1,
	.vbp		= 10,
 };

static struct omap_dss_device ovation_evt1b_lcd_device = {
	.name                   = "lcd",
	.driver_name            = "novatek-panel",
	.type                   = OMAP_DISPLAY_TYPE_DSI,
	.data                   = dsi_fps_data,
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
		.line_bufs	= 0,
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
			.regn           = 20,
			.regm           = 348,
			.regm_dispc     = 9,
			.regm_dsi       = 8,
			.lp_clk_div     = 16,
			.offset_ddr_clk = 0,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
			.tlpx	= 17,
			.tclk = {
				.zero	 = 89,
				.prepare = 22,
				.trail	 = 23,
			},
			.ths = {
				.zero	 = 35,
				.prepare = 26,
				.exit	 = 49,
				.trail	 = 26,
			},
		},
	},

	.panel = {
		.timings = {
			.x_res		= 1920,
			.y_res		= 1280,
			.pixel_clock	= 148480,
			.hfp		= 4,
			.hsw		= 5,
			.hbp		= 39,
			.vfp		= 9,
			.vsw		= 1,
			.vbp		= 10,
		},
		.width_in_um    = 190080,
		.height_in_um   = 126720,

		.acbi		= 40,
		.acb		= 0,
	},

	.ctrl = {
		.pixel_size = 18,
		.dither	= 1,
	},

	.channel = OMAP_DSS_CHANNEL_LCD,
	.skip_init = true,

	.platform_enable = ovation_enable_lcd,
	.platform_disable = ovation_disable_lcd,

	.set_backlight = ovation_set_pwm_bl,
	.get_backlight = ovation_get_pwm_bl,

	.dispc_timings = &dispc_timings_samsung,
	.dsi_timings = &dsi_timings_samsung,
};

static struct omap_dss_device ovation_lcd_device = {
	.name                   = "auo_lcd",
	.driver_name            = "auo",
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.data                   = NULL,
	.phy.dpi.data_lines	= 24,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.platform_enable	= ovation_enable_lcd,
	.platform_disable	= ovation_disable_lcd,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
			.max_pixclk_khz = 148500,
		},
	},
};

static struct omap_dss_device ovation_hdmi_device = {
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
			.max_pixclk_khz = 148500,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
	.platform_enable	= ovation_enable_hdmi,
	.platform_disable	= ovation_disable_hdmi,
};

static struct omap_dss_device *ovation_dss_devices[] = {
	&ovation_lcd_device,
	&ovation_hdmi_device,
};

static struct omap_dss_board_info ovation_dss_data = {
	.num_devices	= ARRAY_SIZE(ovation_dss_devices),
	.devices	= ovation_dss_devices,
	.default_device	= &ovation_lcd_device,
};


static struct sgx_omaplfb_platform_data omaplfb_plat_data_ovation_wuxga = {
	.num_configs = OMAPLFB_NUM_DEV,
	.configs = omaplfb_config_ovation_wuxga,
};

static struct omap_tablet_panel_data panel_data_ovation_wuxga = {
	.board_info = &ovation_dss_data,
	.dsscomp_data = &dsscomp_config_ovation_wuxga,
	.omaplfb_data = &omaplfb_plat_data_ovation_wuxga,
};

static struct omap_tablet_panel_data panel_data_ovation = {
	.board_info = &ovation_dss_data,
	.dsscomp_data = NULL,
	.omaplfb_data = NULL,
};

static struct omapfb_platform_data ovation_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
	},
	.boot_fb_addr = 0,
	.boot_fb_size = 0,
};

void ovation_android_display_setup(struct omap_ion_platform_data *ion)
{
	u32 boot_fb_addr = simple_strtol(boot_fb, NULL, 16);
	if (boot_fb_addr) {
		if (memblock_remove(boot_fb_addr, 1920*1280*4) < 0) {
			pr_err("%s: failed to reserve VRAM - no memory\n", __FUNCTION__);
		} else {
			ovation_fb_pdata.boot_fb_addr = boot_fb_addr;
			ovation_fb_pdata.boot_fb_size = 1920*1280*4;
		}
	}

	ovation_evt1b_lcd_device.panel.timings.x_res  = 1920;
	ovation_evt1b_lcd_device.panel.timings.y_res  = 1280;

	if (system_rev >= OVATION_EVT1B) {
		pr_info("Ovation EVT1B detected - selecting Novatek panel");
		ovation_dss_devices[0] = &ovation_evt1b_lcd_device;
		ovation_dss_data.default_device = &ovation_evt1b_lcd_device;
	}

	omap_android_display_setup(panel_data_ovation_wuxga.board_info,
				panel_data_ovation_wuxga.dsscomp_data,
				panel_data_ovation_wuxga.omaplfb_data,
				&ovation_fb_pdata, ion);
}

int __init ovation_panel_init(void)
{
	ovation_panel_init_dpi();
	ovation_hdmi_mux_init();
	omapfb_set_platform_data(&ovation_fb_pdata);
	omap_display_init(&ovation_dss_data);

	if (system_rev >= OVATION_EVT1B) {
		i2c_register_board_info(3, bl_i2c_boardinfo,
				ARRAY_SIZE(bl_i2c_boardinfo));
	} else {
		i2c_register_board_info(1, bl_i2c_boardinfo,
 				ARRAY_SIZE(bl_i2c_boardinfo));
	}
	return 0;
}

