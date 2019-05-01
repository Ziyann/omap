/*
 * OMAP4XXX L3 Interconnect error handling driver
 *
 * Copyright (C) 2011 Texas Corporation
 *	Santosh Shilimkar <santosh.shilimkar@ti.com>
 *	Sricharan <r.sricharan@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "omap_l3_noc.h"

static void dump_CPU1_stack(void *dummy)
{
	dump_stack();
}

static void l3_noc_show_status(int module_idx, void __iomem *base, u32 src_idx)
{
	char *target_name, *master_name = "UN IDENTIFIED";
	void __iomem *tbase;	/* L3 TARG Base */
	u32 k, std_err_main, masterid, clear, slave_offset;
	int saved_oip = oops_in_progress;

	/*
	* As interrupts are handled on CPU0, and L3 error might come from
	* CPU1, dump CPU1 stack too. Setting the oops_in_progress will
	* eliminate a deadlock warning from smp_call_function_single,
	* which cannot happen in our case.
	*/
	oops_in_progress = 1;
	smp_call_function_single(1, &dump_CPU1_stack, NULL, 0);
	oops_in_progress = saved_oip;

	tbase = base + *(l3_targ[module_idx] + src_idx);
	std_err_main =  __raw_readl(tbase + L3_TARG_STDERRLOG_MAIN);
	slave_offset = __raw_readl(tbase + L3_TARG_STDERRLOG_SLVOFSLSB);
	target_name = l3_targ_inst_name[module_idx][src_idx];
	if (std_err_main & CUSTOM_ERROR)
		masterid = __raw_readl(tbase + L3_TARG_STDERRLOG_CI_MSTADDR);
	else
		masterid = __raw_readl(tbase + L3_TARG_STDERRLOG_MSTADDR);

	for (k = 0; k < NUM_OF_L3_MASTERS; k++) {
		if (masterid == l3_masters[k].id)
			master_name = l3_masters[k].name;
	}

	WARN(true, "L3 %s error: MASTER:%s TARGET:%s at offset 0x%08x\n"
		"\tL3_TARG_STDHOSTHDR_COREREG\t\t%08X\n"
		"\tL3_TARG_STDHOSTHDR_VERSIONREG\t\t%08X\n"
		"\tL3_TARG_STDHOSTHDR_MAINCTLREG\t\t%08X\n"
		"\tL3_TARG_STDHOSTHDR_NTTPADDR_0\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_SVRTSTDLVL\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_SVRTCUSTOMLVL\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_MAIN\t\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_HDR\t\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_MSTADDR\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_SLVADDR\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_INFO\t\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_SLVOFSLSB\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_SLVOFSMSB\t\t%08X\n"
		"\tL3_TARG_STDERRLOG_CUSTOMINFO_INFO\t%08X\n"
		"\tL3_TARG_STDERRLOG_CUSTOMINFO_MSTADDR\t%08X\n"
		"\tL3_TARG_STDERRLOG_CUSTOMINFO_OPCODE\t%08X\n"
		"\tL3_TARG_ADDRSPACESIZELOG\t\t%08X\n",
		(std_err_main & CUSTOM_ERROR) ? "Custom" : "Standard",
		master_name, target_name, slave_offset,
		__raw_readl(tbase + L3_TARG_STDHOSTHDR_COREREG),
		__raw_readl(tbase + L3_TARG_STDHOSTHDR_VERSIONREG),
		__raw_readl(tbase + L3_TARG_STDHOSTHDR_MAINCTLREG),
		__raw_readl(tbase + L3_TARG_STDHOSTHDR_NTTPADDR_0),
		__raw_readl(tbase + L3_TARG_STDERRLOG_SVRTSTDLVL),
		__raw_readl(tbase + L3_TARG_STDERRLOG_SVRTCUSTOMLVL),
		std_err_main,
		__raw_readl(tbase + L3_TARG_STDERRLOG_HDR),
		__raw_readl(tbase + L3_TARG_STDERRLOG_MSTADDR),
		__raw_readl(tbase + L3_TARG_STDERRLOG_SLVADDR),
		__raw_readl(tbase + L3_TARG_STDERRLOG_INFO),
		slave_offset,
		__raw_readl(tbase + L3_TARG_STDERRLOG_SLVOFSMSB),
		__raw_readl(tbase + L3_TARG_STDERRLOG_CI_INFO),
		__raw_readl(tbase + L3_TARG_STDERRLOG_CI_MSTADDR),
		__raw_readl(tbase + L3_TARG_STDERRLOG_CI_OPCODE),
		__raw_readl(tbase + L3_TARG_ADDRSPACESIZELOG));

	/* clear the std error log */
	clear = std_err_main | CLEAR_STDERR_LOG;
	writel(clear, tbase + L3_TARG_STDERRLOG_MAIN);
}
/*
 * Interrupt Handler for L3 error detection.
 *	1) Identify the L3 clockdomain partition to which the error belongs to.
 *	2) Identify the slave where the error information is logged
 *	3) Print the logged information.
 *	4) Add dump stack to provide kernel trace.
 *
 * Two Types of errors :
 *	1) Custom errors in L3 :
 *		Target like DMM/FW/EMIF generates SRESP=ERR error
 *	2) Standard L3 error:
 *		- Unsupported CMD.
 *			L3 tries to access target while it is idle
 *		- OCP disconnect.
 *		- Address hole error:
 *			If DSS/ISS/FDIF/USBHOSTFS access a target where they
 *			do not have connectivity, the error is logged in
 *			their default target which is DMM2.
 *
 *	On High Secure devices, firewall errors are possible and those
 *	can be trapped as well. But the trapping is implemented as part
 *	secure software and hence need not be implemented here.
 */
static irqreturn_t l3_interrupt_handler(int irq, void *_l3)
{

	struct omap4_l3 *l3 = _l3;
	void __iomem *base;
	int inttype, i;
	u32 err_reg;

	/* Get the Type of interrupt */
	inttype = irq == l3->app_irq ? L3_APPLICATION_ERROR : L3_DEBUG_ERROR;

	for (i = 0; i < L3_MODULES; i++) {
		/*
		 * Read the regerr register of the clock domain
		 * to determine the source
		 */
		base = l3->l3_base[i];
		err_reg = __raw_readl(base + l3_flagmux[i] +
					+ L3_FLAGMUX_REGERR0 + (inttype << 3));

		/* Get the corresponding error and analyse */
		if (!err_reg)
			continue;

		l3_noc_show_status(i, base, __ffs(err_reg));

		/* Error found so break the for loop */
		break;
	}
	return IRQ_HANDLED;
}

static int __devinit omap4_l3_probe(struct platform_device *pdev)
{
	static struct omap4_l3 *l3;
	struct resource	*res;
	int ret;

	l3 = kzalloc(sizeof(*l3), GFP_KERNEL);
	if (!l3)
		return -ENOMEM;

	platform_set_drvdata(pdev, l3);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "couldn't find resource 0\n");
		ret = -ENODEV;
		goto err0;
	}

	l3->l3_base[0] = ioremap(res->start, resource_size(res));
	if (!l3->l3_base[0]) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "couldn't find resource 1\n");
		ret = -ENODEV;
		goto err1;
	}

	l3->l3_base[1] = ioremap(res->start, resource_size(res));
	if (!l3->l3_base[1]) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "couldn't find resource 2\n");
		ret = -ENODEV;
		goto err2;
	}

	l3->l3_base[2] = ioremap(res->start, resource_size(res));
	if (!l3->l3_base[2]) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err2;
	}

	/*
	 * Setup interrupt Handlers
	 */
	l3->debug_irq = platform_get_irq(pdev, 0);
	ret = request_irq(l3->debug_irq,
			l3_interrupt_handler,
			IRQF_DISABLED, "l3-dbg-irq", l3);
	if (ret) {
		pr_crit("L3: request_irq failed to register for 0x%x\n",
						OMAP44XX_IRQ_L3_DBG);
		goto err3;
	}

	l3->app_irq = platform_get_irq(pdev, 1);
	ret = request_irq(l3->app_irq,
			l3_interrupt_handler,
			IRQF_DISABLED, "l3-app-irq", l3);
	if (ret) {
		pr_crit("L3: request_irq failed to register for 0x%x\n",
						OMAP44XX_IRQ_L3_APP);
		goto err4;
	}

	return 0;

err4:
	free_irq(l3->debug_irq, l3);
err3:
	iounmap(l3->l3_base[2]);
err2:
	iounmap(l3->l3_base[1]);
err1:
	iounmap(l3->l3_base[0]);
err0:
	kfree(l3);
	return ret;
}

static int __devexit omap4_l3_remove(struct platform_device *pdev)
{
	struct omap4_l3 *l3 = platform_get_drvdata(pdev);

	free_irq(l3->app_irq, l3);
	free_irq(l3->debug_irq, l3);
	iounmap(l3->l3_base[0]);
	iounmap(l3->l3_base[1]);
	iounmap(l3->l3_base[2]);
	kfree(l3);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id l3_noc_match[] = {
	{.compatible = "ti,omap4-l3-noc", },
	{},
};
MODULE_DEVICE_TABLE(of, l3_noc_match);
#else
#define l3_noc_match NULL
#endif

static struct platform_driver omap4_l3_driver = {
	.probe		= omap4_l3_probe,
	.remove		= __devexit_p(omap4_l3_remove),
	.driver		= {
		.name		= "omap_l3_noc",
		.owner		= THIS_MODULE,
		.of_match_table = l3_noc_match,
	},
};

static int __init omap4_l3_init(void)
{
	return platform_driver_register(&omap4_l3_driver);
}
postcore_initcall_sync(omap4_l3_init);

static void __exit omap4_l3_exit(void)
{
	platform_driver_unregister(&omap4_l3_driver);
}
module_exit(omap4_l3_exit);
