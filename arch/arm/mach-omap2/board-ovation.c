/*
 * Board support file for OMAP44xx ovation.
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
#include <linux/i2c/bq2419x.h>
#include <linux/power/bq27x00_battery.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <plat/omap-serial.h>
#include <linux/input/kxtf9.h>
#include <mach/dmm.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/lpddr2-hynix.h>
#include <mach/lpddr2-samsung.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <plat/omap-pm.h>
#include <linux/wakelock.h>
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "pm.h"
#include "board-ovation.h"
#include <mach/omap4_ion.h>
#include "omap_ram_console.h"

#define KXTJ9_GPIO_IRQ 152

#define EXT_FET_EN              4

#define  SAMSUNG_SDRAM 0x01
#define  ELPIDA_SDRAM  0x03
#define  HYNIX_SDRAM   0x06

#define GPIO_UART_DDC_SWITCH	182
#define GPIO_LS_DCDC_EN		60
#define GPIO_LS_OE		81
#define BQ27500_BAT_LOW_GPIO	42
volatile unsigned int KERNEL_SV = 0x0;

static struct omap_board_config_kernel ovation_config[] __initdata = {
};

static void __init omap_ovation_init_early(void)
{
	struct omap_hwmod *oh;
	int i;
	char const * const hwmods[] = {
		[0] = "gpio1",
		[1] = "gpio3",
		[2] = "gpio5"
	};

	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
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
		.no_off_init	= true,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= 0,
	},
	{
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}

	/* Setting MMC3 SDIO card .built-in variable
	 * This is to make sure that if WiFi driver is not loaded
	 * at all, then the MMC/SD/SDIO driver does not keep
	 * turning on/off the voltage to the SDIO card
	 */
	if (pdev->id == 2) {
		ret = 0;
		pdata->slots[0].mmc_data.built_in = 1;
	}

	return ret;
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

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}

static struct bq27x00_platform_data __initdata ovation_bq27520_platform_data = {
	.gpio_ce = 113,
	.gpio_soc_int = 176,
	.gpio_bat_low = 42,
};

static struct bq2419x_platform_data __initdata ovation_bqdata = {
	.max_charger_voltagemV = 4200,
	.max_charger_currentmA = 1550,
	.gpio_int = 142,
	.gpio_ce = 158,
	.gpio_psel = 85,
	.stimer_sdp = CHGTIMER_12h,
	.stimer_dcp = CHGTIMER_12h,
};

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
	.min_interval   = 1,
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

static struct i2c_board_info __initdata ovation_i2c_bq27520_boardinfo = {
	I2C_BOARD_INFO("bq27500", 0x55),
	.flags = I2C_CLIENT_WAKE,
	.platform_data = &ovation_bq27520_platform_data,
};

static struct i2c_board_info __initdata ovation_i2c_kxtf9_boardinfo = {
	I2C_BOARD_INFO("kxtf9", 0xf),
	.platform_data = &kxtf9_platform_data_here,
	.irq = OMAP_GPIO_IRQ(KXTJ9_GPIO_IRQ),
};

static struct i2c_board_info __initdata ovation_i2c_bq24196_boardinfo = {
	I2C_BOARD_INFO("bq24196", 0x6b),
	.platform_data = &ovation_bqdata,
};

static void __init ovation_pmic_mux_init(void)
{
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata ovation_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata ovation_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata ovation_i2c_3_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &ovation_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &ovation_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &ovation_i2c_3_bus_pdata);

	omap_register_i2c_bus_board_data(1, &ovation_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &ovation_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &ovation_i2c_3_bus_pdata);

	bn_power_init();

	//set kxtf9 address
	if (system_rev >= OVATION_EVT1A) {
		printk(KERN_INFO "kxtf9 i2c address = 0xe \n");
		ovation_i2c_kxtf9_boardinfo.addr = 0xe;
	}
	i2c_register_board_info(1, &ovation_i2c_kxtf9_boardinfo, 1);

	if (system_rev >= OVATION_EVT1B) {
		i2c_register_board_info(1, &ovation_i2c_bq27520_boardinfo, 1);
		i2c_register_board_info(1, &ovation_i2c_bq24196_boardinfo, 1);
	}

	/* Disable i2c2, same pins for uart1 console */
	//omap_register_i2c_bus(2, 400, NULL, 0);

	//i2c_register_board_info(3, ovation_i2c_3_boardinfo,
	//			ARRAY_SIZE(ovation_i2c_3_boardinfo));
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
#define board_wkup_mux	NULL
#endif


/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */

static __initdata struct emif_device_details emif_devices_elpida_2x2G_S4 = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = &lpddr2_elpida_2G_S4_dev,
};

static __initdata struct emif_device_details emif_devices_elpida_4G_S4 = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
};

static __initdata struct emif_device_details emif_devices_elpida_2x4G_S4 = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
	.cs1_device = &lpddr2_elpida_4G_S4_dev,
};

static __initdata struct emif_device_details emif_devices_hynix_4G_S4 = {
	.cs0_device = &lpddr2_hynix_4G_S4_dev,
};

static __initdata struct emif_device_details emif_devices_samsung_4G_S4 = {
	.cs0_device = &lpddr2_samsung_4G_S4_dev,
};

static __initdata struct emif_device_details emif_devices_samsung_2x4G_S4 = {
	.cs0_device = &lpddr2_samsung_4G_S4_dev,
	.cs1_device = &lpddr2_samsung_4G_S4_dev,
};


static struct omap_device_pad ovation_uart1_pads[] __initdata = {
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

static struct omap_device_pad ovation_uart4_pads[] __initdata = {
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

static struct omap_uart_port_info ovation_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = 0,
};

static inline void __init board_serial_init(void)
{
	/* default switch to  UART1 output */
	gpio_request(GPIO_UART_DDC_SWITCH, "UART1_DDC_SWITCH");
	gpio_direction_output(GPIO_UART_DDC_SWITCH, 1);

	/* DC/DC Enable for the level Shifter */
	gpio_request(GPIO_LS_DCDC_EN , "LS_DCDC_EN");
	gpio_direction_output(GPIO_LS_DCDC_EN, 1);

	gpio_request(GPIO_LS_OE, "LS_OE");
	gpio_direction_output(GPIO_LS_OE, 1);

	omap_serial_init_port_pads(0, ovation_uart1_pads,
		ARRAY_SIZE(ovation_uart1_pads), &ovation_uart_info_uncon);
	omap_serial_init_port_pads(3, ovation_uart4_pads,
		ARRAY_SIZE(ovation_uart4_pads), &ovation_uart_info_uncon);
}

static unsigned long __init get_installed_mem_size()
{
	int i;
	unsigned long mem_size = 0;
	const struct meminfo* mi = &meminfo;

	for_each_bank(i, mi)
		mem_size += mi->bank[i].size;

	return mem_size;
}

static void ovation_mem_init(void)
{
	struct emif_device_details *emif_dev = NULL;

	switch(sdram_vendor()) {
		case SAMSUNG_SDRAM:
			printk(KERN_INFO "Samsung DDR Memory\n");
			switch(system_rev) {
				case OVATION_EVT1A:
					emif_dev = &emif_devices_samsung_4G_S4;
				break;
				case OVATION_EVT1B:
				case OVATION_EVT2:
					switch (get_installed_mem_size()) {
						case SZ_1G:
							pr_info("\twith size: 1GB\n");
							emif_dev = &emif_devices_samsung_4G_S4;
							break;
						case SZ_2G - SZ_4K:
						case SZ_2G:
							pr_info("\twith size: 2GB\n");
						default:
							emif_dev = &emif_devices_samsung_2x4G_S4;

					}
				break;
				default:
					emif_dev = &emif_devices_samsung_4G_S4;
			}
		break;
		case ELPIDA_SDRAM:
			printk(KERN_INFO "Elpida DDR Memory\n");
			switch(system_rev) {
				case OVATION_EVT0:
				case OVATION_EVT0B:
					emif_dev = &emif_devices_elpida_2x2G_S4;
				break;
				case OVATION_EVT0C:
				case OVATION_EVT1A:
					emif_dev = &emif_devices_elpida_4G_S4;
				break;
				case OVATION_EVT1B:
				case OVATION_EVT2:
                                       switch (get_installed_mem_size()) {
                                               case SZ_1G:
							pr_info("\twith size 1G\n");
							emif_dev = &emif_devices_elpida_4G_S4;
                                               break;
					       case SZ_2G - SZ_4K:
                                               case SZ_2G:
							pr_info("\twith size 2G\n");
                                               default:
							emif_dev = &emif_devices_elpida_2x4G_S4;
                                       }
				break;
				default:
					emif_dev = &emif_devices_elpida_4G_S4;
			}
		break;
		case HYNIX_SDRAM:
			printk(KERN_INFO "Hynix DDR Memory\n");
			if (system_rev >= OVATION_EVT1A)
				emif_dev = &emif_devices_hynix_4G_S4;
		break;
		default:
			 pr_err("Memory type does not exist\n");
	}

	if (emif_dev)
		omap_emif_setup_device_details(emif_dev, emif_dev);
	else
		pr_err("Memory type not supported [VID: 0x%X, SYSTEM_REV: 0x%X]\n", sdram_vendor(), system_rev);
}


static struct regulator_consumer_supply ovation_lcd_tp_supply[] = {
	{ .supply = "vtp" },
	{ .supply = "vlcd" },
};

static struct regulator_init_data ovation_lcd_tp_vinit = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 2,
	.consumer_supplies = ovation_lcd_tp_supply,
};

static struct fixed_voltage_config ovation_lcd_touch_reg_data = {
	.supply_name = "vdd_lcdtp",
	.microvolts = 3300000,
	.gpio = 36,
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &ovation_lcd_tp_vinit,
};

static struct platform_device ovation_lcd_touch_regulator_device = {
	.name   = "reg-fixed-voltage",
	.id     = -1,
	.dev    = {
		.platform_data = &ovation_lcd_touch_reg_data,
	},
};

static struct platform_device *ovation_lcd_touch_devices[] __initdata = {
	&ovation_lcd_touch_regulator_device,
};

static void __init ovation_lcd_touch_init(void)
{
	platform_add_devices(ovation_lcd_touch_devices,
				ARRAY_SIZE(ovation_lcd_touch_devices));
}

static struct regulator *ovation_tp_vdd;
static int ovation_tp_vdd_enabled;
static struct regulator *ovation_lcd_tp_pwr[ARRAY_SIZE(ovation_lcd_tp_supply)];
static int ovation_lcd_tp_pwr_enabled[ARRAY_SIZE(ovation_lcd_tp_supply)];

int Vdd_LCD_CT_PEN_request_supply(struct device *dev, const char *supply_name)
{
	int ret;
	int iLoop;
	int index;
	int n_users;

	for(iLoop=0, index=-1; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
	{
		if(!strcmp(supply_name, ovation_lcd_tp_supply[iLoop].supply))
		{
			index = iLoop;
			break;
		}
	}
	if(index >= 0)
	{
		for(iLoop=0, n_users=0; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
		{
			if(ovation_lcd_tp_pwr[iLoop] != NULL)
			{
				n_users++;
				break; /* We just need to check if there is at least one user for the lcd_tp supply */
			}
		}
		if(!n_users)
		{
			if(ovation_tp_vdd == NULL)
			{
				ovation_tp_vdd = regulator_get(dev, "touch_vdd");
				if(IS_ERR(ovation_tp_vdd))
				{
					ovation_tp_vdd = NULL;
					pr_err("%s: touch vdd regulator not valid\n", __func__);
					ret = -ENODEV;
					goto err_regulator_tp_vdd_get;
				}
			}
		}

		if(ovation_lcd_tp_pwr[index] == NULL)
		{
			ovation_lcd_tp_pwr[index] = regulator_get(dev, ovation_lcd_tp_supply[index].supply);
			if(IS_ERR(ovation_lcd_tp_pwr[index]))
			{
				ovation_lcd_tp_pwr[index] = NULL;
				pr_err("%s: Could not get \"%s\" regulator\n", __func__, supply_name);
				ret = -ENODEV;
				goto err_regulator_lcd_tp_pwr_get;
			}
			ret = 0;
			n_users++;
			goto err_return;
		}
		else
		{
			pr_debug("%s: \"%s\" already in use\n", __func__, supply_name);
			ret = 0;
			goto err_return;
		}
	}
	else
	{
		pr_err("%s: no regulator with name \"%s\" found\n", __func__, supply_name);
		ret = -ENODEV;
		goto err_return;
	}

err_regulator_lcd_tp_pwr_get:
	if(n_users == 0) /* No devices using the regulator */
	{
		if(ovation_tp_vdd != NULL)
		{
			regulator_put(ovation_tp_vdd);
			ovation_tp_vdd = NULL;
		}
	}
err_regulator_tp_vdd_get:
err_return:
	return ret;
}
EXPORT_SYMBOL(Vdd_LCD_CT_PEN_request_supply);

int Vdd_LCD_CT_PEN_release_supply(struct device *dev, const char *supply_name)
{
	int ret;
	int iLoop;
	int index;
	int n_users;

	for(iLoop=0, index=-1; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
	{
		if(!strcmp(supply_name, ovation_lcd_tp_supply[iLoop].supply))
		{
			index = iLoop;
			break;
		}
	}
	if(index >= 0)
	{
			if(ovation_lcd_tp_pwr[index] != NULL)
			{
				regulator_put(ovation_lcd_tp_pwr[index]);
				ovation_lcd_tp_pwr[index] = NULL;
			}
			else
			{
				pr_debug("%s: \"%s\" already released\n", __func__, supply_name);
			}
	}

	for(iLoop=0, n_users=0; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
	{
		if(ovation_lcd_tp_pwr[iLoop] != NULL)
		{
			n_users++;
			break; /* We just need to check if there is at least one user for the lcd_tp supply */
		}
	}
	if(n_users == 0)
	{
		if(ovation_tp_vdd != NULL)
		{
			regulator_put(ovation_tp_vdd);
			ovation_tp_vdd = NULL;
		}
	}

	return 0;
}
EXPORT_SYMBOL(Vdd_LCD_CT_PEN_release_supply);

int Vdd_LCD_CT_PEN_enable(struct device *dev, const char *supply_name)
{
	int ret;
	int iLoop;
	int index;
	int n_users;

	for(iLoop=0, index=-1; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
	{
		if(!strcmp(supply_name, ovation_lcd_tp_supply[iLoop].supply))
		{
			index = iLoop;
			break;
		}
	}
	if(index >= 0)
	{
		if(ovation_tp_vdd_enabled == 0)
		{
			if(ovation_tp_vdd != NULL)
			{
				ret = regulator_enable(ovation_tp_vdd);
				if(ret)
				{
					pr_err("%s: Could not enable touch vdd regulator\n", __func__);
					ret = -EBUSY;
					goto err_regulator_tp_vdd_enable;
				}
				ovation_tp_vdd_enabled = 1;
				msleep(5);
			}
			else
			{
				pr_err("%s: touch vdd regulator is not valid\n", __func__);
				ret = -ENODEV;
				goto err_regulator_tp_vdd_enable;
			}
		}
		if(ovation_lcd_tp_pwr_enabled[index] == 0)
		{
			if(ovation_lcd_tp_pwr[index] != NULL)
			{
				ret = regulator_enable(ovation_lcd_tp_pwr[index]);
				if(ret)
				{
					pr_err("%s: Could not enable \"%s\" regulator\n", __func__, supply_name);
					ret = -EBUSY;
					goto err_regulator_lcd_tp_pwr_enable;
				}
				ovation_lcd_tp_pwr_enabled[index] = 1;
			}
			else
			{
				pr_err("%s: \"%s\" regulator is not valid\n", __func__, supply_name);
				ret = -ENODEV;
				goto err_regulator_lcd_tp_pwr_enable;
			}
		}
	}
	goto err_return;

err_regulator_lcd_tp_pwr_enable:
	for(iLoop=0, n_users=0; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
	{
		if(ovation_lcd_tp_pwr_enabled[iLoop] != 0)
		{
			n_users++;
			break; /* We just need to check if there is at least one user for the lcd_tp supply */
		}
	}
	if(n_users == 0)
	{
		if(ovation_tp_vdd != NULL)
		{
			regulator_disable(ovation_tp_vdd);
			ovation_tp_vdd_enabled = 0;
		}
		else
		{
			pr_err("%s: touch vdd regulator is not valid\n", __func__);
		}
	}
err_regulator_tp_vdd_enable:
err_return:
	return ret;
}
EXPORT_SYMBOL(Vdd_LCD_CT_PEN_enable);

int Vdd_LCD_CT_PEN_disable(struct device *dev, const char *supply_name)
{
	int ret;
	int iLoop;
	int index;
	int n_users;

	for(iLoop=0, index=-1; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
	{
		if(!strcmp(supply_name, ovation_lcd_tp_supply[iLoop].supply))
		{
			index = iLoop;
			break;
		}
	}
	if(index >= 0)
	{
		if(ovation_lcd_tp_pwr_enabled[index] != 0)
		{
			if(ovation_lcd_tp_pwr[index] != NULL)
			{
				regulator_disable(ovation_lcd_tp_pwr[index]);
				ovation_lcd_tp_pwr_enabled[index] = 0;
			}
			else
			{
				pr_err("%s: \"%s\" regulator is not valid\n", __func__, supply_name);
			}
		}
		for(iLoop=0, n_users=0; iLoop < ARRAY_SIZE(ovation_lcd_tp_supply); iLoop++)
		{
			if(ovation_lcd_tp_pwr_enabled[iLoop] != 0)
			{
				n_users++;
				break; /* We just need to check if there is at least one user for the lcd_tp supply */
			}
		}
		if(n_users == 0)
		{
			if(ovation_tp_vdd_enabled != 0)
			{
				if(ovation_tp_vdd != NULL)
				{
					regulator_disable(ovation_tp_vdd);
					ovation_tp_vdd_enabled = 0;
				}
				else
				{
					pr_err("%s: touch vdd regulator is not valid\n", __func__);
				}
			}
			else
			{
				pr_err("%s: touch vdd regulator state is not valid\n", __func__);
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(Vdd_LCD_CT_PEN_disable);

void __init omap_ovation_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	/* Turn off the external FET for twl6032 charger */
	gpio_request ( EXT_FET_EN , "EXT-FET-EN" );
	gpio_direction_output ( EXT_FET_EN , 0 );

	/* EVT1a bringup only */
	if (system_rev == OVATION_EVT1A) {
		/* The GPIO for Charging level - 1=2A, 0=1A */
		gpio_request(44, "ic_chg_level");
		gpio_direction_output(44, 1);
	}

	ovation_mem_init();
	omap_board_config = ovation_config;
	omap_board_config_size = ARRAY_SIZE(ovation_config);
	omap4_create_board_props();
	omap4_audio_conf();
	omap4_i2c_init();
	ovation_lcd_touch_init();
	ovation_touch_init();
	omap_dmm_init();
	ovation_panel_init();
	ovation_pmic_mux_init();
	ovation_button_init();
	kxtj9_dev_init();
	omap4_register_ion();
	board_serial_init();
	bn_wilink_init();

	omap4_twl6030_hsmmc_init(mmc);
	usb_musb_init(&musb_board_data);

	omap_enable_smartreflex_on_init();
        if (enable_suspend_off)
                omap_pm_enable_off_mode();

	/* Enable GG bat_low irq to wake up device to inform framework to shutdown. */
	omap_mux_init_gpio(BQ27500_BAT_LOW_GPIO, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
}

static void __init omap_ovation_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init omap_ovation_reserve(void)
{

	omap_init_ram_size();
#ifdef CONFIG_ION_OMAP
	ovation_android_display_setup(get_omap_ion_platform_data());
	omap_ion_init();
#else
	ovation_android_display_setup(NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}


MACHINE_START(OMAP_OVATION, "OMAP4 ovation board")
	.boot_params	= 0x80000100,
	.reserve	= omap_ovation_reserve,
	.map_io		= omap_ovation_map_io,
	.init_early	= omap_ovation_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap_ovation_init,
	.timer		= &omap_timer,
MACHINE_END
