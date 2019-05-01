/*
 * Reboot reason special handler
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/notifier.h>

#include "common.h"
#include "omap4-sar-layout.h"

int omap_sar_reason(char *reason)
{
	char __iomem *sar_base;
	int offset = 0;

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);

	if (cpu_is_omap44xx())
		offset = OMAP4_REBOOT_REASON_OFFSET;
	else if (cpu_is_omap54xx())
		offset = OMAP5_REBOOT_REASON_OFFSET;
	else
		WARN("undefined chip, %s", __func__);

	strncpy(sar_base + offset, reason, OMAP_REBOOT_REASON_SIZE-1);

	/* always end with terminal symbol */
	*(sar_base + offset + OMAP_REBOOT_REASON_SIZE - 1) = '\0';

	return 0;
}

static int omap_reboot_notifier_call(struct notifier_block *this,
				     unsigned long code, void *cmd)
{
	char *reason = "normal";
	int ret;

	/* Save reboot mode in scratch memory */
	if (code == SYS_RESTART && cmd != NULL && *(char *)cmd)
		reason = cmd;
	else if (code == SYS_POWER_OFF)
		reason = "off";

	ret = omap_sar_reason(reason);

	return notifier_from_errno(ret);
}

static struct notifier_block omap_reboot_notifier = {
	.notifier_call = omap_reboot_notifier_call,
};

static int __init omap_reboot_reason_init(void)
{
	char __iomem *sar_base;
	int offset = 0;

	if (cpu_is_omap44xx())
		offset = OMAP4_REBOOT_REASON_OFFSET;
	else if (cpu_is_omap54xx())
		offset = OMAP5_REBOOT_REASON_OFFSET;
	else
		WARN("undefined chip, %s", __func__);

	sar_base = omap4_get_sar_ram_base();
	if (sar_base)
		strncpy(sar_base + offset, "", OMAP_REBOOT_REASON_SIZE);

	return register_reboot_notifier(&omap_reboot_notifier);
}
late_initcall(omap_reboot_reason_init);
