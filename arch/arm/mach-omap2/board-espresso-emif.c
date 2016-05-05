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

#include "common.h"
#include "board-espresso.h"

struct ddr_device_info lpddr2_samsung_4G_S4_info = {
	.type		= DDR_TYPE_LPDDR2_S4,
	.density	= DDR_DENSITY_4Gb,
	.io_width	= DDR_IO_WIDTH_32,
	.cs1_used	= false,
	.cal_resistors_per_cs = false,
	.manufacturer	= "Samsung"
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

void __init omap4_espresso_emif_init(void)
{
	omap_emif_set_device_details(1, &lpddr2_samsung_4G_S4_info,
			lpddr2_jedec_timings,
			ARRAY_SIZE(lpddr2_jedec_timings),
			&lpddr2_jedec_min_tck, &custom_configs);
	omap_emif_set_device_details(2, &lpddr2_samsung_4G_S4_info,
			lpddr2_jedec_timings,
			ARRAY_SIZE(lpddr2_jedec_timings),
			&lpddr2_jedec_min_tck, &custom_configs);
}
