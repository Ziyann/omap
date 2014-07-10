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

#include <linux/kernel.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#include <linux/i2c/twl.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/omap-serial.h>
#include <linux/mfd/twl6040.h>

#ifdef CONFIG_INPUT_TWL6040_HSKEYS
#include <linux/input/twl6040-hskeys.h>
#endif

#include <asm/system_info.h>

#include "common-board-devices.h"
#include "board-hummingbird.h"
#include "board-ovation.h"
#include "mux.h"

#define TWL6041_AUDPWRON_GPIO	112
#define OMAP_LINEOUT_DTC_GPIO	100

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
	.supply_regulator	= "V2V1",
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

static struct regulator_init_data regen1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
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

static struct twl6040_codec_data twl6040_codec = {
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

static int twl6040_platform_init(struct twl6040 *twl6040)
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

static struct twl6040_platform_data twl6040_data = {
	.codec		= &twl6040_codec,
#ifdef CONFIG_INPUT_TWL6040_HSKEYS
	.hskeys		= &twl6040_hskeys,
#endif
	.audpwron_gpio	= TWL6041_AUDPWRON_GPIO,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
	.platform_init	= twl6040_platform_init,
};

static struct twl4030_platform_data twldata = {
	/* TWL6032 regulators at OMAP447X based SOMs */
	.ldo3		= &vaux3,
	.ldo4		= &vaux2,
	.ldo7		= &vusim,
	.ldoln		= &vdac,

	/* TWL6030/6032 common resources */
	.clk32kg	= &clk32kg,

	/* External control pins */
	.regen1		= &regen1,
};

void __init bn_power_init(void)
{
#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
	if(system_rev <= HUMMINGBIRD_EVT2)
		twl6040_data.codec->hs_jack_debounce = 1;
#endif
#ifdef CONFIG_MACH_OMAP_OVATION
	if(system_rev <= OVATION_EVT2)
		twl6040_data.codec->hs_jack_debounce = 1;
#endif
	omap4_pmic_get_config(&twldata, TWL_COMMON_PDATA_USB |
			TWL_COMMON_PDATA_MADC | \
			TWL_COMMON_PDATA_BCI |
			TWL_COMMON_PDATA_THERMAL,
			TWL_COMMON_REGULATOR_VAUX1 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_V1V8 |
			TWL_COMMON_REGULATOR_V2V1 |
			TWL_COMMON_REGULATOR_SYSEN |
			TWL_COMMON_REGULATOR_CLK32KAUDIO);

	omap4_pmic_init("twl6032", &twldata, &twl6040_data, OMAP44XX_IRQ_SYS_2N);
}
