/*
 * Board support file for OMAP44xx hummingbird.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/memblock.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wakelock.h>

#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_info.h>

#include <mach/hardware.h>
#include <mach/omap-secure.h>
#include <mach/omap4_ion.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <plat/omap-pm.h>
#include <plat/drm.h>

#include "mux.h"
#include "hsmmc.h"
#include "common.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "board-hummingbird.h"
#include "omap_ram_console.h"

#ifdef CONFIG_INPUT_KXTF9
#include <linux/input/kxtf9.h>
#endif
#ifdef CONFIG_CHARGER_BQ2419x
#include <linux/i2c/bq2419x.h>
#endif
#ifdef CONFIG_BATTERY_BQ27x00
#include <linux/power/bq27x00_battery-bn.h>
#endif

#define KXTJ9_GPIO_IRQ 152

#define CHG_LEVEL		84
#define GG_CE			113

#define PN544_IRQ_GPIO		157
#define PN544_VEN_GPIO		53
#define PN544_FIRM_GPIO		55
#define PN544_PWR_EN		172

#define GPIO_UART_DDC_SWITCH	182
#define GPIO_LS_DCDC_EN		60
#define GPIO_LS_OE		81
#define BQ27500_BAT_LOW_GPIO	42
volatile unsigned int KERNEL_SV = 0x0;


static struct omap_board_config_kernel hummingbird_config[] __initdata = {
};

static void __init omap_hummingbird_init_early(void)
{
	struct omap_hwmod *oh;
	int i;
	char const * const hwmods[] = {
		[0] = "gpio1",
		[1] = "gpio3",
		[2] = "gpio5"
	};

	omap4430_init_early();

	for (i = 0; i < sizeof(hwmods)/sizeof(char*); i++) {
		oh = omap_hwmod_lookup(hwmods[i]);
		if (oh) {
			if (omap_hwmod_no_setup_reset(oh))
				printk("Failed to disable reset for %s\n", hwmods[i]);
		} else
			printk("%s hwmod not found\n", hwmods[i]);
	}
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#else
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 200,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.built_in	= 1,
		.no_off_init	= true,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= 0,
	},
	{
                .mmc            = 3,
                .caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
                .gpio_cd        = -EINVAL,
                .gpio_wp        = -EINVAL,
                .ocr_mask       = MMC_VDD_165_195,
		.built_in	= 1,
		.nonremovable   = true,
        },
	{}	/* Terminator */
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int irq = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		irq = twl6030_mmc_card_detect_config();
		if (irq) {
			pr_err("Failed configuring MMC1 card detect\n");
			return irq;
		}
		pdata->slots[0].card_detect_irq = irq;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return 0;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(&c->pdev->dev);

	return 0;
}
static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);

	omap_mux_init_signal("usbb1_ulpitll_dat4.abe_mcbsp3_dr",\
		OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLDOWN);

	omap_mux_init_signal("usbb1_ulpitll_dat5.abe_mcbsp3_dx",\
		OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW);

	omap_mux_init_signal("usbb1_ulpitll_dat6.abe_mcbsp3_clkx",\
		OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);

	omap_mux_init_signal("usbb1_ulpitll_dat7.abe_mcbsp3_fsx",\
		OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
}

#ifdef CONFIG_BATTERY_BQ27x00
static struct bq27x00_platform_data __initdata hummingbird_bq27520_platform_data = {
	.gpio_ce = 113,
	.gpio_soc_int = 176,
	.gpio_bat_low = 42,
};
#endif

#ifdef CONFIG_CHARGER_BQ2419x
static struct bq2419x_platform_data __initdata hummingbird_bqdata = {
	.max_charger_voltagemV = 4200,
	.max_charger_currentmA = 1550,
	.gpio_int = 142,
	.gpio_ce = 158,
	.gpio_psel = 85,
	.stimer_sdp = CHGTIMER_12h,
	.stimer_dcp = CHGTIMER_8h,
};
#endif

#ifdef CONFIG_INPUT_KXTF9
static int __init kxtj9_dev_init(void)
{
	int ret;
	ret = gpio_request(KXTJ9_GPIO_IRQ, "kxtj9_irq");

	if (ret)
		return ret;

	gpio_direction_input(KXTJ9_GPIO_IRQ);
	return 0;
}

struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,	   //1
	.poll_interval  = 1000,	

	.g_range	= KXTF9_G_8G,
	.shift_adj	= SHIFT_ADJ_2G,

	/* Map the axes from the sensor to the device */
	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,
	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
	.data_odr_init		= ODR12_5F,
	.ctrl_reg1_init		= KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init		= KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init		= KXTF9_IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x16,
	.wuf_thresh_init	= 0x28,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xFF,
	.tdt_l_thresh_init	= 0x14,
	.tdt_tap_timer_init	= 0x53,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x10,
	.tdt_window_timer_init	= 0xA0,

	.gpio			= KXTJ9_GPIO_IRQ,
};
#endif

static struct i2c_board_info __initdata hummingbird_i2c_boardinfo[] = {
#ifdef CONFIG_BATTERY_BQ27x00
	{
		I2C_BOARD_INFO("bq27500", 0x55),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &hummingbird_bq27520_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_KXTF9
	{
		I2C_BOARD_INFO("kxtf9", 0xe),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTJ9_GPIO_IRQ),
	},
#endif
#ifdef CONFIG_CHARGER_BQ2419x
	{
		I2C_BOARD_INFO("bq24196", 0x6b),
		.platform_data = &hummingbird_bqdata,
	},
#endif
};

static void __init hummingbird_pmic_mux_init(void)
{
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request(USE_MUTEX_LOCK);
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id, USE_MUTEX_LOCK);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata hummingbird_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata hummingbird_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata hummingbird_i2c_3_bus_pdata;
//static struct omap_i2c_bus_board_data __initdata hummingbird_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &hummingbird_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &hummingbird_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &hummingbird_i2c_3_bus_pdata);

	omap_register_i2c_bus_board_data(1, &hummingbird_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &hummingbird_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &hummingbird_i2c_3_bus_pdata);

	bn_power_init();

	i2c_register_board_info(1, hummingbird_i2c_boardinfo,
				ARRAY_SIZE(hummingbird_i2c_boardinfo));

	omap_register_i2c_bus(3, 400, NULL, 0);

	// Disable the strong pull-ups on I2C3 and I2C4
	omap2_i2c_pullups_en_dis(3, 0);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	*/
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

#define DEFAULT_UART_AUTOSUSPEND_DELAY	3000	/* Runtime autosuspend (msecs)*/

static struct omap_device_pad hummingbird_uart1_pads[] __initdata = {
	{
		.name	= "i2c2_sda.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "i2c2_scl.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE1,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad hummingbird_uart4_pads[] __initdata = {
	{
		.name   = "abe_dmic_clk1.uart4_cts",
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE5,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
			  OMAP_MUX_MODE5,
	},
	{
		.name   = "abe_dmic_din1.uart4_rts",
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE5,
		.idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_board_data hummingbird_uart1_board_data __initdata = {
	.id = 0,
	.pads = hummingbird_uart1_pads,
	.pads_cnt = ARRAY_SIZE(hummingbird_uart1_pads),
};

static struct omap_board_data hummingbird_uart4_board_data __initdata = {
	.id = 3,
	.pads = hummingbird_uart4_pads,
	.pads_cnt = ARRAY_SIZE(hummingbird_uart4_pads),
};

static struct omap_uart_port_info hummingbird_uart_info __initdata = {
	.dma_enabled = 0,
	.autosuspend_timeout = DEFAULT_UART_AUTOSUSPEND_DELAY,
//      .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static inline void __init board_serial_init(void)
{
	if (system_rev == HUMMINGBIRD_EVT0) {
		/* default switch to  UART1 output */
		gpio_request(GPIO_UART_DDC_SWITCH, "UART1_DDC_SWITCH");
		gpio_direction_output(GPIO_UART_DDC_SWITCH, 1);

	}
	/* NOTE: Didn't control the LS_OE and CT_CP_HPD gpio pins,
	 * because it will be controlled from hummingbird_hdmi_mux_init()
	 */
	omap_serial_init_port(&hummingbird_uart1_board_data, &hummingbird_uart_info);
	omap_serial_init_port(&hummingbird_uart4_board_data, &hummingbird_uart_info);
}

static phys_addr_t hummingbird_get_ram_size(void)
{
	phys_addr_t ram_size = SZ_1G;
	return ram_size;
}


static void __init hummingbird_mem_init(void)
{
	bool supported = 0;

	switch(omap_sdram_vendor()) {
		case SAMSUNG_SDRAM:
			printk(KERN_INFO "Samsung DDR Memory\n");
			omap_emif_set_device_details(1, &lpddr2_samsung_4G_S4_info,
					lpddr2_samsung_4G_S4_timings,
					ARRAY_SIZE(lpddr2_samsung_4G_S4_timings),
					&lpddr2_samsung_S4_min_tck, NULL);
			omap_emif_set_device_details(2, &lpddr2_samsung_4G_S4_info,
					lpddr2_samsung_4G_S4_timings,
					ARRAY_SIZE(lpddr2_samsung_4G_S4_timings),
					&lpddr2_samsung_S4_min_tck, NULL);
			supported = 1;
		break;
		case ELPIDA_SDRAM:
			printk(KERN_INFO "Elpida DDR Memory\n");
			omap_emif_set_device_details(1, &lpddr2_elpida_4G_S4_info,
					lpddr2_elpida_4G_S4_timings,
					ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
					&lpddr2_elpida_S4_min_tck, NULL);
			omap_emif_set_device_details(2, &lpddr2_elpida_4G_S4_info,
					lpddr2_elpida_4G_S4_timings,
					ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
					&lpddr2_elpida_S4_min_tck, NULL);
			supported = 1;
		break;
		case HYNIX_SDRAM:
			printk(KERN_INFO "Hynix DDR Memory\n");
			omap_emif_set_device_details(1, &lpddr2_hynix_4G_S4_info,
					lpddr2_hynix_4G_S4_timings,
					ARRAY_SIZE(lpddr2_hynix_4G_S4_timings),
					&lpddr2_hynix_S4_min_tck, NULL);
			omap_emif_set_device_details(2, &lpddr2_hynix_4G_S4_info,
					lpddr2_hynix_4G_S4_timings,
					ARRAY_SIZE(lpddr2_hynix_4G_S4_timings),
					&lpddr2_hynix_S4_min_tck, NULL);
			supported = 1;
		break;
		default:
			 pr_err("Memory type does not exist\n");
	}

	if (!supported)
		pr_err("Memory type not supported [VID: 0x%X, SYSTEM_REV: 0x%X]\n", omap_sdram_vendor(), system_rev);
}

static struct regulator_consumer_supply hummingbird_lcd_tp_supply[] = {
	{ .supply = "vtp" },
	{ .supply = "vlcd" },
};

static struct regulator_init_data hummingbird_lcd_tp_vinit = {
        .constraints = {
                .min_uV = 3300000,
                .max_uV = 3300000,
                .valid_modes_mask = REGULATOR_MODE_NORMAL,
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
        },
        .num_consumer_supplies = 2,
        .consumer_supplies = hummingbird_lcd_tp_supply,
};

static struct fixed_voltage_config hummingbird_lcd_touch_reg_data = {
        .supply_name = "vdd_lcdtp",
        .microvolts = 3300000,
        .gpio = 36,
        .enable_high = 1,
        .enabled_at_boot = 1,
        .init_data = &hummingbird_lcd_tp_vinit,
};

static struct platform_device hummingbird_lcd_touch_regulator_device = {
        .name   = "reg-fixed-voltage",
        .id     = -1,
        .dev    = {
                .platform_data = &hummingbird_lcd_touch_reg_data,
        },
};

static struct platform_device *hummingbird_lcd_touch_devices[] __initdata = {
        &hummingbird_lcd_touch_regulator_device,
};

static void __init hummingbird_lcd_touch_init(void)
{
        platform_add_devices(hummingbird_lcd_touch_devices,
                                ARRAY_SIZE(hummingbird_lcd_touch_devices));
}

static void __init omap_hummingbird_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	// For EVT1 Bring-up
	gpio_request(CHG_LEVEL,"CHG-LEVEL");
	gpio_direction_output(CHG_LEVEL, 0);

	// For EVT1 Bring-up
	gpio_request(GG_CE,"GG-CE");
	gpio_direction_output(GG_CE, 0);

	hummingbird_mem_init();
	omap_board_config = hummingbird_config;
	omap_board_config_size = ARRAY_SIZE(hummingbird_config);
	omap_create_board_props();
	omap4_audio_conf();
	omap4_i2c_init();
	hummingbird_lcd_touch_init();
	hummingbird_touch_init();
	hummingbird_panel_init();
	hummingbird_pmic_mux_init();
	hummingbird_button_init();
#ifdef CONFIG_INPUT_KXTF9
	kxtj9_dev_init();
#endif
	omap_init_dmm_tiler();
#ifdef CONFIG_ION_OMAP
	omap4_register_ion();
#endif
	board_serial_init();
	bn_wilink_init();

	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);
	usb_musb_init(&musb_board_data);

	omap_enable_smartreflex_on_init();
        if (enable_suspend_off)
                omap_pm_enable_off_mode();

	/* Enable GG bat_low irq to wake up device to inform framework to shutdown. */
	omap_mux_init_gpio(BQ27500_BAT_LOW_GPIO, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
}

static void __init omap_hummingbird_reserve(void)
{
	omap_set_ram_size(hummingbird_get_ram_size());

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT, OMAP_RAM_CONSOLE_SIZE_DEFAULT);
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);

	hummingbird_android_display_setup();
#ifdef CONFIG_ION_OMAP
	omap4_ion_init();
#endif

	omap4_secure_workspace_addr_default();

	omap_reserve();
}

MACHINE_START(OMAP_HUMMINGBIRD, "OMAP4 Hummingbird board")
	.atag_offset	= 0x100,
	.reserve	= omap_hummingbird_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap_hummingbird_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= omap_hummingbird_init,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
