/*
 * arch/arm/mach-omap2/resetreason.c
 *
 * Copyright (C) 2011 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/string.h>
#include "prm-regbits-44xx.h"
#include "prcm44xx.h"
#include "prm44xx.h"
#include <plat/prcm.h>
#include "prminst44xx.h"
#include "resetreason.h"

#include <linux/proc_fs.h>

#define RESET_REASON_SIZE	256
static char resetreason[RESET_REASON_SIZE];
#define RESETREASON_PROCNAME "cpu/reset_reason"

static struct {
	const char *str;
	u32 mask;
} resetreason_flags[] = {
	{ "C2C ",			OMAP4430_C2C_RST_MASK },
	{ "IcePick ",			OMAP4430_ICEPICK_RST_MASK },
	{ "Voltage Manager ",		OMAP4430_VDD_MPU_VOLT_MGR_RST_MASK |
					OMAP4430_VDD_IVA_VOLT_MGR_RST_MASK |
					OMAP4430_VDD_CORE_VOLT_MGR_RST_MASK },
	{ "External Warm ",		OMAP4430_EXTERNAL_WARM_RST_MASK },
	{ "MPU Watchdog Timer ",	OMAP4430_MPU_WDT_RST_MASK },
	{ "Warm Software ",		OMAP4430_GLOBAL_WARM_SW_RST_MASK },
	{ "Cold ",			OMAP4430_GLOBAL_COLD_RST_MASK },
};

const char *omap4_get_resetreason(void)
{
	return resetreason;
}

static int __init resetreason_init(void)
{
	int i;
	u32 reasons = omap_prcm_get_reset_sources();

	memset(resetreason, 0, RESET_REASON_SIZE);

	for (i = 0; i < ARRAY_SIZE(resetreason_flags); i++)
		if (reasons & resetreason_flags[i].mask)
			strlcat(resetreason, resetreason_flags[i].str,
				sizeof(resetreason));

	strlcat(resetreason, "Reset", sizeof(resetreason));

	pr_info("OMAP boot reason is %s (PRM_RSTST=0x%x)\n", resetreason, reasons);

	return 0;
}

static int proc_resetreason_read(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
	strlcpy(page, resetreason, sizeof(resetreason));
	*eof = 1;

	return strlen(page);
}

static int __init resetreason_procfs_init(void)
{
	struct proc_dir_entry *proc_resetreason = create_proc_entry(RESETREASON_PROCNAME, S_IRUGO, NULL);
	if (proc_resetreason != NULL) {
		proc_resetreason->data = NULL;
		proc_resetreason->read_proc = (read_proc_t *)proc_resetreason_read;
		proc_resetreason->write_proc = NULL;
	}

	return 0;
}

postcore_initcall(resetreason_init);
module_init(resetreason_procfs_init);
