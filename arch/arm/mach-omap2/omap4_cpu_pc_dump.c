/*
 * omap4_cpu_pc_dump.c
 *
 * Dumps the program counter of one/both CPUs on OMAP4.
 *
 * Copyright (C) 2013 MM Solutions.
 * Author : Konstantin Buhchev <kbuhchev@mm-sol.com>
 * Based on work of Dimitar Dimitrov <dddimitrov@mm-sol.com>
 * Credits : Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/smp.h>
#include <linux/cpumask.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <plat/omap44xx.h>

#include "iomap.h"
#include "clock.h"
#include "clockdomain.h"

#define OMAP4_CPU_PC_BUFFER_SIZE	10
#define OMAP4_L3_CPU0_DEBUG_OFFSET	0x140000
#define OMAP4_L3_CPU1_DEBUG_OFFSET	0x142000
#define OMAP4_CPU_DEBUG_PC_OFFSET	0x84
#define EMU_IDLE_TO_WAKE_TIMEOUT_US	1000

static const int L3_EMU_CPU_debug_offset[NR_CPUS] = {
			L4_EMU_44XX_BASE + OMAP4_L3_CPU0_DEBUG_OFFSET
					+ OMAP4_CPU_DEBUG_PC_OFFSET,
			L4_EMU_44XX_BASE + OMAP4_L3_CPU1_DEBUG_OFFSET
					+ OMAP4_CPU_DEBUG_PC_OFFSET
		};

static struct {
	struct clockdomain *emu_sys_clkdm;
	struct clk *l3_main_3_ick;
	struct clk *l3_instr_ick;
	void __iomem *pc_ptr[NR_CPUS];
} omap4_cpu_pc_dump_iomap;

static int omap4_cpu_pc_buffer[NR_CPUS][OMAP4_CPU_PC_BUFFER_SIZE];

void omap4_cpu_pc_dump(struct cpumask *cpu_mask)
{
	int cpu, count;

	if (!omap4_cpu_pc_dump_iomap.emu_sys_clkdm)
		return;

	clkdm_wakeup(omap4_cpu_pc_dump_iomap.emu_sys_clkdm);

	/*
	* L3 instrumentarium clocks get enabled and then reset.
	* As a result, usecount remains 1, but clocks are disabled.
	* Clear the usecount to get them back in balance.
	*/
	if (omap4_cpu_pc_dump_iomap.l3_main_3_ick->usecount &&
		!(readl(omap4_cpu_pc_dump_iomap.l3_main_3_ick->enable_reg) &
		1 << omap4_cpu_pc_dump_iomap.l3_main_3_ick->enable_bit))
			omap4_cpu_pc_dump_iomap.l3_main_3_ick->usecount = 0;
	clk_enable(omap4_cpu_pc_dump_iomap.l3_main_3_ick);

	if (omap4_cpu_pc_dump_iomap.l3_instr_ick->usecount &&
		!(readl(omap4_cpu_pc_dump_iomap.l3_instr_ick->enable_reg) &
		1 << omap4_cpu_pc_dump_iomap.l3_instr_ick->enable_bit))
			omap4_cpu_pc_dump_iomap.l3_instr_ick->usecount = 0;
	clk_enable(omap4_cpu_pc_dump_iomap.l3_instr_ick);

	/* It takes several uS to get data available, poll the current CPU PC
	*  until it becomes non-zero. */
	for (count = 0; count < EMU_IDLE_TO_WAKE_TIMEOUT_US; count++) {
		if (readl(omap4_cpu_pc_dump_iomap.
					pc_ptr[raw_smp_processor_id()]))
			break;
		udelay(1);
	}

	for_each_cpu(cpu, cpu_mask)
		for (count = 0; count < OMAP4_CPU_PC_BUFFER_SIZE; count++)
			omap4_cpu_pc_buffer[cpu][count] =
				readl(omap4_cpu_pc_dump_iomap.pc_ptr[cpu]);

	clk_disable(omap4_cpu_pc_dump_iomap.l3_main_3_ick);
	clk_disable(omap4_cpu_pc_dump_iomap.l3_instr_ick);
	clkdm_allow_idle(omap4_cpu_pc_dump_iomap.emu_sys_clkdm);

	for_each_cpu(cpu, cpu_mask)
		for (count = 0; count < OMAP4_CPU_PC_BUFFER_SIZE; count++)
			pr_info("CPU%d PC (%d) : 0x%x\n", cpu, count,
					omap4_cpu_pc_buffer[cpu][count]);
}

void omap4_cpu_pc_dump_other_cpu(void)
{
	struct cpumask all = *cpu_present_mask;

	cpumask_clear_cpu(raw_smp_processor_id(), &all);
	omap4_cpu_pc_dump(&all);
}

#ifdef CONFIG_MAGIC_SYSRQ
static void omap4_cpu_pc_dump_sysrq(int dummy1)
{
	struct cpumask all = *cpu_present_mask;

	omap4_cpu_pc_dump(&all);
}

static struct sysrq_key_op sysrq_omap4_cpu_pc_dump_op = {
	.handler = omap4_cpu_pc_dump_sysrq,
	.help_msg = "dump CPU PC",
	.action_msg = "Dumps the program counter of both CPUs on OMAP4",
};
#else
static struct sysrq_key_op sysrq_omap4_cpu_pc_dump_op = { };
#endif

static int omap4_cpu_pc_dump_panic_cb(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpumask all = *cpu_present_mask;

	omap4_cpu_pc_dump(&all);
	return 0;
}

static struct notifier_block omap4_cpu_pc_dump_panic_nb = {
	.notifier_call = omap4_cpu_pc_dump_panic_cb,
};

static int __init omap4_cpu_pc_dump_init(void)
{
	unsigned int i;

	omap4_cpu_pc_dump_iomap.emu_sys_clkdm = clkdm_lookup("emu_sys_clkdm");
	if (!omap4_cpu_pc_dump_iomap.emu_sys_clkdm)
		goto out;

	omap4_cpu_pc_dump_iomap.l3_main_3_ick = clk_get(NULL, "l3_main_3_ick");
	if (IS_ERR(omap4_cpu_pc_dump_iomap.l3_main_3_ick))
		goto out;

	omap4_cpu_pc_dump_iomap.l3_instr_ick = clk_get(NULL, "l3_instr_ick");
	if (IS_ERR(omap4_cpu_pc_dump_iomap.l3_instr_ick))
		goto err1;

	for_each_possible_cpu(i) {
		omap4_cpu_pc_dump_iomap.pc_ptr[i] =
			ioremap_nocache(L3_EMU_CPU_debug_offset[i],
							sizeof(void *));
		if (!omap4_cpu_pc_dump_iomap.pc_ptr[i])
			goto err2;
	}

	atomic_notifier_chain_register(&panic_notifier_list,
				       &omap4_cpu_pc_dump_panic_nb);
	register_sysrq_key('d', &sysrq_omap4_cpu_pc_dump_op);

	pr_info("OMAP4 CPU PC dump initialized.\n");

	return 0;

err2:
	clk_put(omap4_cpu_pc_dump_iomap.l3_instr_ick);
err1:
	clk_put(omap4_cpu_pc_dump_iomap.l3_main_3_ick);
out:
	pr_err("\nOMAP4 CPU PC dump : init failed!\n\n");
	return 0;
}

subsys_initcall(omap4_cpu_pc_dump_init);
