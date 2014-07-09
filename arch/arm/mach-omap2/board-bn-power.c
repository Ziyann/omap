/* B&N OMAP PMIC regulators board file.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Based on
 * mach-omap2/board-44xx-power.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#include <linux/i2c/twl.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/omap-serial.h>
#include <linux/mfd/twl6040-codec.h>
#ifdef CONFIG_INPUT_TWL6040_HSKEYS
#include <linux/input/twl6040-hskeys.h>
#endif
#include "common-board-devices.h"
#include "board-hummingbird.h"
#include "board-ovation.h"
#include "mux.h"

#define TWL6041_AUDPWRON_GPIO	112
#define OMAP_LINEOUT_DTC_GPIO	100

static struct regulator_consumer_supply vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies		= ARRAY_SIZE(vmmc_supply),
	.consumer_supplies		= vmmc_supply,
};

static struct regulator_init_data vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply vwlan_supply[] = {
	REGULATOR_SUPPLY("wlan-vio", NULL),
};

static struct regulator_init_data vusim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.boot_on		= true,
		.always_on		= true,
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies		= ARRAY_SIZE(vwlan_supply),
	.consumer_supplies		= vwlan_supply,
};

static struct regulator_init_data vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_init_data vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies		= ARRAY_SIZE(vcxio_supply),
	.consumer_supplies		= vcxio_supply,
};

static struct regulator_consumer_supply vdac_supply[] = {
	REGULATOR_SUPPLY("hdmi_vref", NULL),
};

static struct regulator_init_data vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies		= ARRAY_SIZE(vdac_supply),
	.consumer_supplies		= vdac_supply,
};

static struct regulator_consumer_supply vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

static struct regulator_init_data vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies		= ARRAY_SIZE(vusb_supply),
	.consumer_supplies		= vusb_supply,
};

static struct regulator_consumer_supply vaux_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};

static struct regulator_init_data vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies		= 1,
	.consumer_supplies		= vaux_supply,
};

static struct regulator_consumer_supply vaux2_supply[] = {
};

static struct regulator_init_data vaux2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies		= ARRAY_SIZE(vaux2_supply),
	.consumer_supplies		= vaux2_supply,
};

static struct regulator_consumer_supply touch_supply[] = {
	REGULATOR_SUPPLY("touch_vdd", NULL),
	REGULATOR_SUPPLY("bl_i2c_pup", NULL),
};

static struct regulator_init_data vaux3 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_ON,
		.boot_on		= true,
	},
	.num_consumer_supplies		= ARRAY_SIZE(touch_supply),
	.consumer_supplies		= touch_supply,
};


static struct regulator_consumer_supply clk32kg_supply[] = {
	REGULATOR_SUPPLY("clk32kg", NULL),
};

static struct regulator_init_data clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.boot_on		= true,
	},
	.num_consumer_supplies		= ARRAY_SIZE(clk32kg_supply),
	.consumer_supplies		= clk32kg_supply,
};

static struct regulator_init_data clk32kaudio = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data sysen = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data regen1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static int batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data bci_data = {
	.monitoring_interval	= 10,
	.max_charger_currentmA	= 1500,
	.max_charger_voltagemV	= 4560,
	.max_bat_voltagemV	= 4200,
	.low_bat_voltagemV	= 3300,
	.sense_resistor_mohm	= 15,
	.battery_tmp_tbl	= batt_table,
	.tblsize		= ARRAY_SIZE(batt_table),
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

#ifdef CONFIG_INPUT_TWL6040_HSKEYS
static struct hskeys_key_data bn_hskey_data[] = {
	{
		.key = KEY_PLAYPAUSE,
		.min_ref_volt = 0,
		.max_ref_volt = 55,
	},
	{
		.key = KEY_NEXTSONG,
		.min_ref_volt = 165,
		.max_ref_volt = 273,
	},
	{
		.key = KEY_PREVIOUSSONG,
		.min_ref_volt = 56,
		.max_ref_volt = 164,
	},
};

static struct hskeys_keymap_data bn_hskeymap_data = {
	.keymap = bn_hskey_data,
	.keymap_size = ARRAY_SIZE(bn_hskey_data),
};

static struct twl4030_codec_hskeys_data twl6040_hskeys = {
	.data  = &bn_hskeymap_data,
};
#endif

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x01,
	.hs_right_step	= 0x01,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
	.vddhf_gpo	= TWL6040_GPO1,
	.lineout_gpio 	= OMAP_LINEOUT_DTC_GPIO,
	.hook_gpadc_ch	= 2,
	.hook_ref_vol	= 450,
	.hs_jack_debounce	= 0,
};

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1 and ES1.2 (both share same ASICREV value)
	 */
	if (rev == TWL6040_REV_1_1)
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);

	return 0;
}

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
#ifdef CONFIG_INPUT_TWL6040_HSKEYS
	.hskeys		= &twl6040_hskeys,
#endif
	.audpwron_gpio	= TWL6041_AUDPWRON_GPIO,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
	.init		= twl6040_init,
};

static struct twl4030_madc_platform_data twl6030_gpadc = {
	.irq_line = -1,
};

static struct twl4030_platform_data twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* TWL6032 regulators at OMAP447X based SOMs */
	.ldo1		= &vpp,
	.ldo2		= &vaux1,
	.ldo3		= &vaux3,
	.ldo4		= &vaux2,
	.ldo5		= &vmmc,
	.ldo6		= &vcxio,
	.ldo7		= &vusim,
	.ldoln		= &vdac,
	.ldousb		= &vusb,

	/* TWL6030/6032 common resources */
	.vana		= &vana,
	.clk32kg	= &clk32kg,
	.clk32kaudio	= &clk32kaudio,

	/* children */
	.bci		= &bci_data,
	.usb		= &omap4_usbphy_data,
	.codec		= &twl6040_codec,
	.madc		= &twl6030_gpadc,

	/* External control pins */
	.sysen		= &sysen,
	.regen1		= &regen1,
};

void __init bn_power_init(void)
{
#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
	if(system_rev <= HUMMINGBIRD_EVT2)
		twldata.codec->audio->hs_jack_debounce = 1;
#endif
#ifdef CONFIG_MACH_OMAP_OVATION
	if(system_rev <= OVATION_EVT2)
		twldata.codec->audio->hs_jack_debounce = 1;
#endif
	omap4_pmic_init("twl6032", &twldata);
}
