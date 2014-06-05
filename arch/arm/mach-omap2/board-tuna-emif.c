/*
 * LPDDR2 data as per SAMSUNG data sheet
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>

#include <linux/platform_data/emif_plat.h>
#include "common.h"
#include "board-tuna.h"

struct lpddr2_timings lpddr2_samsung_4G_S4_timings[] = {
	[0] = {
		.max_freq	= 400000000,
		.tRPab		= 21000,
		.tRCD		= 18000,
		.tWR		= 15000,
		.tRAS_min	= 42000,
		.tRRD		= 10000,
		.tWTR		= 15000,
		.tXP		= 15000,
		.tRTP		= 15000,
		.tCKESR		= 15000,
		.tZQCS		= 90000,
		.tZQCL		= 360000,
		.tZQinit	= 1000000,
		.tDQSCK_max	= 11000,
		.tRAS_max_ns	= 70000,
		.tFAW		= 50000,
	},
	[1] = {
		.max_freq	= 200000000,
		.tRPab		= 21000,
		.tRCD		= 18000,
		.tWR		= 15000,
		.tRAS_min	= 42000,
		.tRRD		= 10000,
		.tWTR		= 20000,
		.tXP		= 15000,
		.tRTP		= 15000,
		.tCKESR		= 15000,
		.tZQCS		= 90000,
		.tZQCL		= 360000,
		.tZQinit	= 1000000,
		.tDQSCK_max	= 11000,
		.tRAS_max_ns	= 70000,
		.tFAW		= 50000,
	},
};

struct lpddr2_min_tck lpddr2_samsung_S4_min_tck = {
	.tRPab		= 3,
	.tRCD		= 3,
	.tWR		= 3,
	.tRASmin	= 3,
	.tRRD		= 2,
	.tWTR		= 2,
	.tXP		= 2,
	.tRTP		= 2,
	.tCKE		= 3,
	.tCKESR		= 3,
	.tFAW		= 8
};

struct ddr_device_info lpddr2_samsung_4G_S4_info = {
	.type		= DDR_TYPE_LPDDR2_S4,
	.density	= DDR_DENSITY_4Gb,
	.io_width	= DDR_IO_WIDTH_32,
	.manufacturer = "Samsung",
};

static struct __devinitdata emif_custom_configs custom_configs = {
	.mask	= EMIF_CUSTOM_CONFIG_LPMODE,
	.lpmode	= EMIF_LP_MODE_SELF_REFRESH,
	.lpmode_timeout_performance = 512,
	.lpmode_timeout_power = 512,
	/* only at OPP100 should we use performance value */
	.lpmode_freq_threshold = 400000000,
};

/*
 * LPDDR2 Configuration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	4 Gb
 *	EMIF2 - CS0 -	4 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */

void __init omap4_tuna_emif_init(void)
{
	omap_emif_set_device_details(1, &lpddr2_samsung_4G_S4_info,
			lpddr2_samsung_4G_S4_timings,
			ARRAY_SIZE(lpddr2_samsung_4G_S4_timings),
			&lpddr2_samsung_S4_min_tck, &custom_configs);
	omap_emif_set_device_details(2, &lpddr2_samsung_4G_S4_info,
			lpddr2_samsung_4G_S4_timings,
			ARRAY_SIZE(lpddr2_samsung_4G_S4_timings),
			&lpddr2_samsung_S4_min_tck, &custom_configs);
}
