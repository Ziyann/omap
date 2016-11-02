/*
 * omap_wdt.c
 *
 * Watchdog driver for the TI OMAP 16xx & 24xx/34xx 32KHz (non-secure) watchdog
 *
 * Author: MontaVista Software, Inc.
 *	 <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20030527: George G. Davis <gdavis@mvista.com>
 *	Initially based on linux-2.4.19-rmk7-pxa1/drivers/char/sa1100_wdt.c
 *	(c) Copyright 2000 Oleg Drokin <green@crimea.edu>
 *	Based on SoftDog driver by Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 * Copyright (c) 2004 Texas Instruments.
 *	1. Modified to support OMAP1610 32-KHz watchdog timer
 *	2. Ported to 2.6 kernel
 *
 * Copyright (c) 2005 David Brownell
 *	Use the driver model and standard identifiers; handle bigger timeouts.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/sysrq.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/nmi.h>
#include <linux/suspend.h>
#include <linux/pm_qos.h>
#include <mach/hardware.h>
#include <plat/prcm.h>

#include "omap_wdt.h"

extern struct atomic_notifier_head touch_watchdog_notifier_head;

static struct platform_device *omap_wdt_dev;

static unsigned timer_margin;
module_param(timer_margin, uint, 0);
MODULE_PARM_DESC(timer_margin, "initial watchdog timeout (in seconds)");

static int kernelpet = 0;
module_param(kernelpet, int, 0);
MODULE_PARM_DESC(kernelpet, "pet watchdog in kernel via irq");

static unsigned int wdt_trgr_pattern = 0x1234;
static DEFINE_SPINLOCK(wdt_lock);

struct omap_wdt_dev {
	void __iomem    *base;          /* physical */
	struct device   *dev;
	int             omap_wdt_users;
	struct resource *mem;
	struct miscdevice omap_wdt_miscdev;
	int irq;
	struct notifier_block nb;
	struct pm_qos_request pm_qos_request;
};

static void omap_wdt_ping(struct omap_wdt_dev *wdev)
{
	void __iomem    *base = wdev->base;

	if (pm_qos_request_active(&wdev->pm_qos_request))
		pm_qos_remove_request(&wdev->pm_qos_request);

	/* wait for posted write to complete */
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x08)
		cpu_relax();

	wdt_trgr_pattern = ~wdt_trgr_pattern;
	__raw_writel(wdt_trgr_pattern, (base + OMAP_WATCHDOG_TGR));

	/* wait for posted write to complete */
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x08)
		cpu_relax();
	/* reloaded WCRR from WLDR */
}

static void omap_wdt_enable(struct omap_wdt_dev *wdev)
{
	void __iomem *base = wdev->base;

	/* Sequence to enable the watchdog */
	__raw_writel(0xBBBB, base + OMAP_WATCHDOG_SPR);
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x10)
		cpu_relax();

	__raw_writel(0x4444, base + OMAP_WATCHDOG_SPR);
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x10)
		cpu_relax();
}

static void omap_wdt_disable(struct omap_wdt_dev *wdev)
{
	void __iomem *base = wdev->base;

	/* sequence required to disable watchdog */
	__raw_writel(0xAAAA, base + OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x10)
		cpu_relax();

	__raw_writel(0x5555, base + OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x10)
		cpu_relax();
}

static void omap_wdt_adjust_timeout(unsigned new_timeout)
{
	if (new_timeout < TIMER_MARGIN_MIN)
		new_timeout = TIMER_MARGIN_DEFAULT;
	if (new_timeout > TIMER_MARGIN_MAX)
		new_timeout = TIMER_MARGIN_MAX;
	timer_margin = new_timeout;
}

static void omap_wdt_set_timeout(struct omap_wdt_dev *wdev)
{
	u32 pre_margin;
	u32 delay_period;
	void __iomem *base = wdev->base;

	/* Find the right delay period */
	if (kernelpet) {
		pre_margin = GET_WLDR_VAL(timer_margin);
		delay_period = GET_WLDR_VAL(timer_margin / 2);
	} else {
		pre_margin = GET_WLDR_VAL(timer_margin + TIME_BEFORE_REBOOT);
		delay_period = GET_WLDR_VAL(timer_margin / 2);
	}

	/* just count up at 32 KHz */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x04)
		cpu_relax();

	__raw_writel(pre_margin, base + OMAP_WATCHDOG_LDR);
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x04)
		cpu_relax();

	/* Set delay interrupt to half the watchdog interval. */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 1 << 5)
		cpu_relax();
	__raw_writel(delay_period, base + OMAP_WATCHDOG_WDLY);
}

static irqreturn_t omap_wdt_interrupt_th(int irq, void *dev_id)
{
	struct omap_wdt_dev *wdev = dev_id;

	if (!pm_qos_request_active(&wdev->pm_qos_request)) {
		pr_crit("WA: WDT request PM_QOS_MEMORY_THROUGHPUT\n");

		/* pm_qos_add_request() require might_sleep() */
		pm_qos_add_request(&wdev->pm_qos_request,
			PM_QOS_MEMORY_THROUGHPUT,
			PM_QOS_MEMORY_THROUGHPUT_HIGH_VALUE);
	}

	return IRQ_HANDLED;
}

static irqreturn_t omap_wdt_interrupt(int irq, void *dev_id)
{
	struct omap_wdt_dev *wdev = dev_id;
	void __iomem *base = wdev->base;
	u32 i;

	if (!kernelpet) {
		/* If this interrupt is triggered this means that the user daemon is not able any more
		 * to ping the watchdog. Then try to collect as much as possible data to help debugging
		 */
		__handle_sysrq('d', 0);
		__handle_sysrq('w', 0);
		__handle_sysrq('l', 0);
		printk(KERN_ERR "Watchdog timeout!!! Board is going to reboot...\n");
	} else
		omap_wdt_ping(wdev);

	i = __raw_readl(base + OMAP_WATCHDOG_WIRQSTAT);
	__raw_writel(i, base + OMAP_WATCHDOG_WIRQSTAT);

	return (!kernelpet) ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static int omap_wdt_setup(struct omap_wdt_dev *wdev)
{
	void __iomem *base = wdev->base;

	if (test_and_set_bit(1, (unsigned long *)&(wdev->omap_wdt_users)))
		return -EBUSY;

	/* initialize prescaler */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x01)
		cpu_relax();

	__raw_writel((1 << 5) | (PTV << 2), base + OMAP_WATCHDOG_CNTRL);
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x01)
		cpu_relax();

	omap_wdt_set_timeout(wdev);
	omap_wdt_ping(wdev); /* trigger loading of new timeout value */

	/* Enable delay interrupt */
	if (wdev->irq)
		__raw_writel(0x2, base + OMAP_WATCHDOG_WIRQENSET);

	omap_wdt_enable(wdev);

	return 0;
}

static int omap_wdt_nb_func(struct notifier_block *nb, unsigned long val,
	void *v)
{
	struct omap_wdt_dev *wdev = container_of(nb, struct omap_wdt_dev, nb);

	spin_lock(&wdt_lock);
	printk(KERN_NOTICE "omap_wdt: pm notification has been recived -> ping watchdog\n");
	omap_wdt_ping(wdev);
	spin_unlock(&wdt_lock);

	return NOTIFY_OK;
}

/*
 *	Allow only one task to hold it open
 */
static int omap_wdt_open(struct inode *inode, struct file *file)
{
	struct omap_wdt_dev *wdev = platform_get_drvdata(omap_wdt_dev);
	int ret;

	pm_runtime_get_sync(wdev->dev);

	ret = omap_wdt_setup(wdev);

	if (ret) {
		pm_runtime_put_sync(wdev->dev);
		return ret;
	}

	/* Enable notif register notifier in order to handle suspend/resume seuqence:
         *  The notify callback will ping watchdog before and after freezing threads
         *  Thus it give time to the kernel to freeze/wake-up all the threads before
         *  watchdog experiration
         */
	if (!kernelpet) {
		wdev->nb.notifier_call = omap_wdt_nb_func;
		register_pm_notifier(&wdev->nb);
	}

	file->private_data = (void *)wdev;

	return nonseekable_open(inode, file);
}

static int omap_wdt_release(struct inode *inode, struct file *file)
{
	struct omap_wdt_dev *wdev = file->private_data;
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	void __iomem *base = wdev->base;
#endif

	/*
	 *      Shut off the timer unless NOWAYOUT is defined.
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	omap_wdt_disable(wdev);

	/* Disable notify chain */
	if (!kernelpet)
		unregister_pm_notifier(&wdev->nb);

	/* Disable delay interrupt */
	if (wdev->irq)
		__raw_writel(0x2, base + OMAP_WATCHDOG_WIRQENCLR);

	pm_runtime_put_sync(wdev->dev);
#else
	pr_crit("Unexpected close, not stopping!\n");
#endif
	wdev->omap_wdt_users = 0;

	return 0;
}

static ssize_t omap_wdt_write(struct file *file, const char __user *data,
		size_t len, loff_t *ppos)
{
	struct omap_wdt_dev *wdev = file->private_data;

	/* Refresh LOAD_TIME. */
	if (len) {
		spin_lock(&wdt_lock);
		omap_wdt_ping(wdev);
		spin_unlock(&wdt_lock);
	}
	return len;
}

static long omap_wdt_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct omap_wdt_dev *wdev;
	int new_margin;
	static const struct watchdog_info ident = {
		.identity = "OMAP Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	wdev = file->private_data;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg, &ident,
				sizeof(ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int __user *)arg);
	case WDIOC_GETBOOTSTATUS:
		if (cpu_is_omap16xx())
			return put_user(__raw_readw(ARM_SYSST),
					(int __user *)arg);
		return put_user(omap_prcm_get_reset_sources(),
				(int __user *)arg);
	case WDIOC_KEEPALIVE:
		spin_lock(&wdt_lock);
		omap_wdt_ping(wdev);
		spin_unlock(&wdt_lock);
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *)arg))
			return -EFAULT;
		omap_wdt_adjust_timeout(new_margin);

		spin_lock(&wdt_lock);
		omap_wdt_disable(wdev);
		omap_wdt_set_timeout(wdev);
		omap_wdt_enable(wdev);

		omap_wdt_ping(wdev);
		spin_unlock(&wdt_lock);
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int __user *)arg);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations omap_wdt_fops = {
	.owner = THIS_MODULE,
	.write = omap_wdt_write,
	.unlocked_ioctl = omap_wdt_ioctl,
	.open = omap_wdt_open,
	.release = omap_wdt_release,
	.llseek = no_llseek,
};

static int __devinit omap_wdt_probe(struct platform_device *pdev)
{
	struct resource *res, *mem, *res_irq;
	struct omap_wdt_dev *wdev;
	int ret;

	/* reserve static register mappings */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENOENT;
		goto err_get_resource;
	}

	if (omap_wdt_dev) {
		ret = -EBUSY;
		goto err_busy;
	}

	mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!mem) {
		ret = -EBUSY;
		goto err_busy;
	}

	wdev = kzalloc(sizeof(struct omap_wdt_dev), GFP_KERNEL);
	if (!wdev) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	wdev->omap_wdt_users = 0;
	wdev->mem = mem;
	wdev->dev = &pdev->dev;

	wdev->base = ioremap(res->start, resource_size(res));
	if (!wdev->base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res_irq) {
		ret = request_threaded_irq(res_irq->start, omap_wdt_interrupt,
			omap_wdt_interrupt_th, 0, dev_name(&pdev->dev), wdev);

		if (ret)
			goto err_irq;

		wdev->irq = res_irq->start;
	}

	platform_set_drvdata(pdev, wdev);

	/*
	 * Note: PM runtime functions must be used only when watchdog driver
	 * is enabling/disabling. Dynamic handling of clock of watchdog timer on
	 * OMAP4/5 (like provided with runtime API) will cause system failure
	 */
	pm_runtime_enable(wdev->dev);
	pm_runtime_irq_safe(wdev->dev);
	pm_runtime_get_sync(wdev->dev);

	/* Keep wdt enabled as per bootloader settings */
	/* omap_wdt_disable(wdev); */
	omap_wdt_adjust_timeout(timer_margin);

	wdev->omap_wdt_miscdev.parent = &pdev->dev;
	wdev->omap_wdt_miscdev.minor = WATCHDOG_MINOR;
	wdev->omap_wdt_miscdev.name = "watchdog";
	wdev->omap_wdt_miscdev.fops = &omap_wdt_fops;

	ret = misc_register(&(wdev->omap_wdt_miscdev));
	if (ret)
		goto err_misc;

	pr_info("OMAP Watchdog Timer Rev 0x%02x: initial timeout %d sec\n",
		__raw_readl(wdev->base + OMAP_WATCHDOG_REV) & 0xFF,
		timer_margin);

	omap_wdt_dev = pdev;

	if (kernelpet && wdev->irq) {
		wdev->nb.notifier_call = omap_wdt_nb_func;
		atomic_notifier_chain_register(&touch_watchdog_notifier_head,
			&wdev->nb);
		return omap_wdt_setup(wdev);
	}

	pm_runtime_put_sync(wdev->dev);
	return 0;

err_misc:
	pm_runtime_put_sync(wdev->dev);
	pm_runtime_disable(wdev->dev);
	platform_set_drvdata(pdev, NULL);

	if (wdev->irq)
		free_irq(wdev->irq, wdev);

err_irq:
	iounmap(wdev->base);

err_ioremap:
	wdev->base = NULL;
	kfree(wdev);

err_kzalloc:
	release_mem_region(res->start, resource_size(res));

err_busy:
err_get_resource:

	return ret;
}

static void omap_wdt_shutdown(struct platform_device *pdev)
{
	struct omap_wdt_dev *wdev = platform_get_drvdata(pdev);

	if (wdev->omap_wdt_users) {
		spin_lock(&wdt_lock);
		omap_wdt_ping(wdev);
		spin_unlock(&wdt_lock);
		pm_runtime_put_sync(wdev->dev);
	}
}

static int __devexit omap_wdt_remove(struct platform_device *pdev)
{
	struct omap_wdt_dev *wdev = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pm_runtime_disable(wdev->dev);
	if (!res)
		return -ENOENT;

	if (pm_qos_request_active(&wdev->pm_qos_request))
		pm_qos_remove_request(&wdev->pm_qos_request);

	misc_deregister(&(wdev->omap_wdt_miscdev));
	release_mem_region(res->start, resource_size(res));
	platform_set_drvdata(pdev, NULL);

	if (wdev->irq)
		free_irq(wdev->irq, wdev);

	if (kernelpet && wdev->irq) {
		omap_wdt_disable(wdev);
		atomic_notifier_chain_unregister(&touch_watchdog_notifier_head,
			&wdev->nb);
	}

	iounmap(wdev->base);

	kfree(wdev);
	omap_wdt_dev = NULL;

	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT ... not clear this is the best way to handle system suspend; and
 * it's very inappropriate for selective device suspend (e.g. suspending this
 * through sysfs rather than by stopping the watchdog daemon).  Also, this
 * may not play well enough with NOWAYOUT...
 */

static int omap_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap_wdt_dev *wdev = platform_get_drvdata(pdev);

	if (wdev->omap_wdt_users) {
		omap_wdt_disable(wdev);
		pm_runtime_put_sync(wdev->dev);
	}

	return 0;
}

static int omap_wdt_resume(struct platform_device *pdev)
{
	struct omap_wdt_dev *wdev = platform_get_drvdata(pdev);

	if (wdev->omap_wdt_users) {
		pm_runtime_get_sync(wdev->dev);
		omap_wdt_enable(wdev);
		omap_wdt_ping(wdev);
	}

	return 0;
}

#else
#define	omap_wdt_suspend	NULL
#define	omap_wdt_resume		NULL
#endif

static struct platform_driver omap_wdt_driver = {
	.probe		= omap_wdt_probe,
	.remove		= __devexit_p(omap_wdt_remove),
	.shutdown	= omap_wdt_shutdown,
	.suspend	= omap_wdt_suspend,
	.resume		= omap_wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "omap_wdt",
	},
};

module_platform_driver(omap_wdt_driver);

MODULE_AUTHOR("George G. Davis");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:omap_wdt");
