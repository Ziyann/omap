/*
 * OMAP SoC specific OPP Data helpers
 *
 * Copyright (C) 2009-2010 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 * Copyright (C) 2010 Nokia Corporation.
 *      Eduardo Valentin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_OMAP_OPP_DATA_H
#define __ARCH_ARM_MACH_OMAP2_OMAP_OPP_DATA_H

#include <plat/omap_hwmod.h>

#include "voltage.h"

/*
 * *BIG FAT WARNING*:
 * USE the following ONLY in opp data initialization common to an SoC.
 * DO NOT USE these in board files/pm core etc.
 */

/**
 * struct def_info - device info for OPP entry
 * @hwmod_name:	Name of the hwmod for this domain
 * @freq:	Frequency in hertz corresponding to this OPP
 * @clk_name: Clock to be configured as part of dvfs
 */
struct device_info {
	char *hwmod_name;
	char *clk_name;
	char *voltdm_name;
};

/**
 * struct omap_opp_def - OMAP OPP Definition
 * @hwmod_name:	Name of the hwmod for this domain
 * @freq:	Frequency in hertz corresponding to this OPP
 * @u_volt:	Nominal voltage in microvolts corresponding to this OPP
 * @default_available:	True/false - is this OPP available by default
 *
 * OMAP SOCs have a standard set of tuples consisting of frequency and voltage
 * pairs that the device will support per voltage domain. This is called
 * Operating Points or OPP. The actual definitions of OMAP Operating Points
 * varies over silicon within the same family of devices. For a specific
 * domain, you can have a set of {frequency, voltage} pairs and this is denoted
 * by an array of omap_opp_def. As the kernel boots and more information is
 * available, a set of these are activated based on the precise nature of
 * device the kernel boots up on. It is interesting to remember that each IP
 * which belongs to a voltage domain may define their own set of OPPs on top
 * of this - but this is handled by the appropriate driver.
 */
struct omap_opp_def {
	struct device_info *dev_info;
	unsigned long freq;
	unsigned long u_volt;

	bool default_available;
	char *opp_name;
};

/*
 * Initialization wrapper used to define an OPP for OMAP variants.
 */
#define OPP_INITIALIZER(_dev_info, _enabled, _freq, _uv, _opp_name)	\
{								\
	.dev_info	= _dev_info,				\
	.default_available	= _enabled,			\
	.freq		= _freq,				\
	.u_volt		= _uv,					\
	.opp_name	= _opp_name				\
}

/*
 * Initialization wrapper used to define SmartReflex process data
 * XXX Is this needed?  Just use C99 initializers in data files?
 */
#define VOLT_DATA_DEFINE(_v_nom, _efuse_offs, _errminlimit, _errgain, _opp_sel)  \
{								       \
	.volt_nominal	= _v_nom,				       \
	.sr_efuse_offs	= _efuse_offs,				       \
	.sr_errminlimit = _errminlimit,				       \
	.vp_errgain	= _errgain,				       \
	.opp_sel	= _opp_sel,				       \
}

#define VOLT_DATA_DEFINE_MARGIN(_v_nom, _v_margin, _efuse_offs, _errminlimit, _errgain, _opp_sel)  \
{								       \
	.volt_nominal	= _v_nom,				       \
	.volt_margin	= _v_margin,				       \
	.sr_efuse_offs	= _efuse_offs,				       \
	.sr_errminlimit = _errminlimit,				       \
	.vp_errgain	= _errgain,				       \
	.opp_sel	= _opp_sel,				       \
}

#define OMAP5_VOLT_DATA_DEFINE(_v_nom, _efuse_offs, _lvt_efuse_offs, _errminlimit, _errgain, _opp_sel)  \
{								       \
	.volt_nominal	= _v_nom,				       \
	.sr_efuse_offs	= _efuse_offs,				       \
	.lvt_sr_efuse_offs	= _lvt_efuse_offs,				       \
	.sr_errminlimit = _errminlimit,				       \
	.vp_errgain	= _errgain,				       \
	.opp_sel	= _opp_sel,				       \
}

#define OMAP5_VOLT_DATA_DEFINE_MARGIN(_v_nom, _v_margin, _efuse_offs, _lvt_efuse_offs, _errminlimit, _errgain, _opp_sel)  \
{								       \
	.volt_nominal	= _v_nom,				       \
	.volt_margin	= _v_margin,				       \
	.sr_efuse_offs	= _efuse_offs,				       \
	.lvt_sr_efuse_offs	= _lvt_efuse_offs,				       \
	.sr_errminlimit = _errminlimit,				       \
	.vp_errgain	= _errgain,				       \
	.opp_sel	= _opp_sel,				       \
}
/* Use this to initialize the default table */
extern int __initdata omap_init_opp_table(struct omap_opp_def *opp_def,
		u32 opp_def_size);
extern int __initdata set_device_opp(void);


extern struct omap_volt_data omap34xx_vddmpu_volt_data[];
extern struct omap_volt_data omap34xx_vddcore_volt_data[];
extern struct omap_vdd_dep_info omap34xx_vddmpu_dep_info[];
extern struct omap_volt_data omap36xx_vddmpu_volt_data[];
extern struct omap_volt_data omap36xx_vddcore_volt_data[];
extern struct omap_vdd_dep_info omap36xx_vddmpu_dep_info[];

extern struct omap_volt_data omap443x_vdd_mpu_volt_data[];
extern struct omap_volt_data omap443x_vdd_iva_volt_data[];
extern struct omap_volt_data omap443x_vdd_core_volt_data[];
extern struct omap_volt_data omap446x_vdd_mpu_volt_data[];
extern struct omap_volt_data omap446x_vdd_iva_volt_data[];
extern struct omap_volt_data omap446x_vdd_core_volt_data[];
extern struct omap_volt_data omap447x_vdd_mpu_volt_data[];
extern struct omap_volt_data omap447x_vdd_iva_volt_data[];
extern struct omap_volt_data omap447x_vdd_core_volt_data[];

extern struct omap_vdd_dep_info omap443x_vddmpu_dep_info[];
extern struct omap_vdd_dep_info omap443x_vddiva_dep_info[];
extern struct omap_vdd_dep_info omap446x_vddmpu_dep_info[];
extern struct omap_vdd_dep_info omap446x_vddiva_dep_info[];
extern struct omap_vdd_dep_info omap447x_vddmpu_dep_info[];
extern struct omap_vdd_dep_info omap447x_vddiva_dep_info[];

#ifdef CONFIG_ARCH_OMAP5_ES1
extern struct omap_volt_data omap5430_vdd_mpu_volt_data[];
extern struct omap_volt_data omap5430_vdd_mm_volt_data[];
extern struct omap_volt_data omap5430_vdd_core_volt_data[];

extern struct omap_vdd_dep_info omap5430_vddmpu_dep_info[];
extern struct omap_vdd_dep_info omap5430_vddmm_dep_info[];

extern struct omap_volt_data omap5432_vdd_mpu_volt_data[];
extern struct omap_volt_data omap5432_vdd_mm_volt_data[];
extern struct omap_volt_data omap5432_vdd_core_volt_data[];

extern struct omap_vdd_dep_info omap5432_vddmpu_dep_info[];
extern struct omap_vdd_dep_info omap5432_vddmm_dep_info[];
#else
extern struct omap_volt_data omap543x_vdd_mpu_volt_data[];
extern struct omap_volt_data omap543x_vdd_mm_volt_data[];
extern struct omap_volt_data omap543x_vdd_core_volt_data[];

extern struct omap_vdd_dep_info omap543x_vddmpu_dep_info[];
extern struct omap_vdd_dep_info omap543x_vddmm_dep_info[];
#endif

#endif		/* __ARCH_ARM_MACH_OMAP2_OMAP_OPP_DATA_H */
