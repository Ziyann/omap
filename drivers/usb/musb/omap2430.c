/*
 * Copyright (C) 2005-2007 by Texas Instruments
 * Some code has been taken from tusb6010.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/usb/musb-omap.h>
#include <linux/usb/omap4_usb_phy.h>
#include <linux/mfd/omap_control.h>
#include <linux/wakelock.h>

#include "musb_core.h"
#include "omap2430.h"

struct omap2430_glue {
	struct device		*dev;
	struct platform_device	*musb;
	struct work_struct	omap_musb_mailbox_work;
	struct workqueue_struct *work_queue;
	struct device		*control_dev;
	struct clk              *phy_clk;
	enum omap_musb_vbus_id_status status;
	u8		core_power_state:1;
	struct mutex	core_power_mutex;
	struct mutex	core_access_mutex;
};
#define glue_to_musb(g)		platform_get_drvdata(g->musb)

struct omap2430_glue		*_glue;

static struct timer_list musb_idle_timer;

static void musb_do_idle(unsigned long _musb)
{
	struct musb	*musb = (void *)_musb;
	unsigned long	flags;
	u8	power;
	u8	devctl;

	spin_lock_irqsave(&musb->lock, flags);

	switch (musb->xceiv->state) {
	case OTG_STATE_A_WAIT_BCON:

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
	case OTG_STATE_A_SUSPEND:
		/* finish RESUME signaling? */
		if (musb->port1_status & MUSB_PORT_STAT_RESUME) {
			power = musb_readb(musb->mregs, MUSB_POWER);
			power &= ~MUSB_POWER_RESUME;
			dev_dbg(musb->controller, "root port resume stopped, power %02x\n", power);
			musb_writeb(musb->mregs, MUSB_POWER, power);
			musb->is_active = 1;
			musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
						| MUSB_PORT_STAT_RESUME);
			musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
			usb_hcd_poll_rh_status(musb_to_hcd(musb));
			/* NOTE: it might really be A_WAIT_BCON ... */
			musb->xceiv->state = OTG_STATE_A_HOST;
		}
		break;
	case OTG_STATE_A_HOST:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl &  MUSB_DEVCTL_BDEVICE)
			musb->xceiv->state = OTG_STATE_B_IDLE;
		else
			musb->xceiv->state = OTG_STATE_A_WAIT_BCON;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}


static void omap2430_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	unsigned long		default_timeout = jiffies + msecs_to_jiffies(3);
	static unsigned long	last_timer;

	if (timeout == 0)
		timeout = default_timeout;

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || ((musb->a_wait_bcon == 0)
			&& (musb->xceiv->state == OTG_STATE_A_WAIT_BCON))) {
		dev_dbg(musb->controller, "%s active, deleting timer\n",
			otg_state_string(musb->xceiv->state));
		del_timer(&musb_idle_timer);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout)) {
		if (!timer_pending(&musb_idle_timer))
			last_timer = timeout;
		else {
			dev_dbg(musb->controller, "Longer idle timer already pending, ignoring\n");
			return;
		}
	}
	last_timer = timeout;

	dev_dbg(musb->controller, "%s inactive, for idle timer for %lu ms\n",
		otg_state_string(musb->xceiv->state),
		(unsigned long)jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb_idle_timer, timeout);
}

static void omap2430_musb_set_vbus(struct musb *musb, int is_on)
{
	struct usb_otg	*otg = musb->xceiv->otg;
	u8		devctl;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	int ret = 1;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		if (musb->xceiv->state == OTG_STATE_A_IDLE) {
			int loops = 100;
			/* start the session */
			devctl |= MUSB_DEVCTL_SESSION;
			musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
			/*
			 * Wait for the musb to set as A device to enable the
			 * VBUS
			 */
			while (musb_readb(musb->mregs, MUSB_DEVCTL) & 0x80) {

				mdelay(5);
				cpu_relax();

				if (time_after(jiffies, timeout)
				    || loops-- <= 0) {
					dev_err(musb->controller,
					"configured as A device timeout");
					ret = -EINVAL;
					break;
				}
			}

			if (ret && otg->set_vbus)
				otg_set_vbus(otg, 1);
			otg->default_a = 1;
			MUSB_HST_MODE(musb);
		}
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		otg->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		MUSB_DEV_MODE(musb);
	}

	dev_dbg(musb->controller, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static int omap2430_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	return 0;
}

static inline void omap2430_low_level_exit(struct musb *musb)
{
	u32 l;

	/* in any role */
	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l |= ENABLEFORCE;	/* enable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);
}

static inline void omap2430_low_level_init(struct musb *musb)
{
	u32 l;

	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l &= ~ENABLEFORCE;	/* disable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);
}

static void omap2430_otg_interface(struct musb *musb, int enable)
{
	u32 val;

	val = musb_readl(musb->mregs, OTG_INTERFSEL);

	if (enable) {
		val &= ~ULPI_12PIN;	/* Disable ULPI */
		val |= UTMI_8BIT;	/* Enable UTMI  */
	} else {
		val |= ULPI_12PIN;	/* Enable ULPI */
		val &= ~UTMI_8BIT;	/* Disable UTMI  */
	}

	musb_writel(musb->mregs, OTG_INTERFSEL, val);
}

int omap_musb_mailbox(enum omap_musb_vbus_id_status status)
{
	struct omap2430_glue	*glue = _glue;
	struct musb		*musb = glue_to_musb(glue);

	/* After quickly calling this function is possibility to miss one event.
	 * Because variable 'glue-> status' will be overwritten with the last
	 * value. And work function will use exactly last value. This will affect
	 * the balance of calls pm_runtime_get/_put. To avoid this a check for
	 * pending and waiting to be completed previous event to the end.
	 */
	if (work_pending(&glue->omap_musb_mailbox_work))
		flush_work(&glue->omap_musb_mailbox_work);

	glue->status = status;
	if (!musb) {
		dev_err(glue->dev, "musb core is not yet ready\n");
		return -ENODEV;
	}

	queue_work(glue->work_queue, &glue->omap_musb_mailbox_work);

	return 0;
}

static void omap_musb_set_mailbox(struct omap2430_glue *glue)
{
	struct musb *musb = glue_to_musb(glue);
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct omap_musb_board_data *data = pdata->board_data;
	struct usb_otg *otg = musb->xceiv->otg;
	u32 val;
	int status;

	dev_dbg(dev, "PB %d::%s\n", __LINE__, __func__);
	/* Since the fact that omap_musb_mailbox() might drop event we should
	 * take care about duplicate events, which might lead to pm unbalance:
	 * Double resume will break the pm usecount.
	 * Double suspend enters the core in state which from it can't quit and
	 * resume normal operation. */
	mutex_lock(&glue->core_access_mutex);
	switch (glue->status) {
	case OMAP_MUSB_ID_GROUND:
		dev_dbg(dev, "ID GND\n");

		/* Protection of invalid sequence of mailbox settings */
		if (musb->xceiv->last_event != USB_EVENT_NONE)
			break;

		otg->default_a = true;
		musb->xceiv->state = OTG_STATE_A_IDLE;
		musb->xceiv->last_event = USB_EVENT_ID;
		if (!is_otg_enabled(musb) || musb->gadget_driver) {
			dev_dbg(dev, "PB %d::%s\n", __LINE__, __func__);
			pm_runtime_get_sync(dev);
			if (glue->control_dev) {
				val = AVALID | VBUSVALID;
				omap4_usb_phy_mailbox(glue->control_dev, val);
			}
			omap2430_musb_set_vbus(musb, 1);
			omap2430_otg_interface(musb, true);
		}
		break;

	case OMAP_MUSB_VBUS_VALID:
		dev_dbg(dev, "VBUS Connect\n");

		/* Protection of invalid sequence of mailbox settings */
		if ((musb->xceiv->last_event != USB_EVENT_NONE) ||
			(glue->core_power_state))
			break;

		otg->default_a = false;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		musb->xceiv->last_event = USB_EVENT_VBUS;
		if (musb->gadget_driver) {
			int err_ret = 0;
			dev_dbg(dev, "PB %d::%s\n", __LINE__, __func__);
			do {
				dev_info(dev, "pm_runtime_resume from runtime_status: %d\n",
					dev->power.runtime_status);
				status = pm_runtime_get_sync(dev);
				if (status < 0) {
					dev_err(dev, "pm_runtime_get_sync FAILED %d\n",
						status);
					pm_runtime_put_noidle(dev);
					if (++err_ret < 500) {
						dev_dbg(dev, "will try again (%d/500)in 100ms...\n",
							err_ret);
						msleep(100);
					} else {
						dev_err(dev, "pm_runtime_get_sync FAILED: Quit.\n");
						dev_dbg(dev, "dev->power.usage_count = %d\n",
							dev->power.usage_count);
						mutex_unlock(&glue->core_access_mutex);
						return;
					}
				} else {
					dev_info(dev, "pm_runtime_get_sync returned %d\n",
						status);
					dev_dbg(dev, "dev->power.usage_count = %d\n",
						dev->power.usage_count);
				}
			} while (status < 0);
		}

		if (musb->gadget_driver)
			omap2430_otg_interface(musb, true);

		if (glue->control_dev) {
			val = IDDIG | AVALID | VBUSVALID;
			omap4_usb_phy_mailbox(glue->control_dev, val);
		}

		glue->core_power_state = 1;
		break;

	case OMAP_MUSB_ID_FLOAT:
	case OMAP_MUSB_VBUS_OFF:
		dev_dbg(dev, "VBUS Disconnect\n");

		/* Protection of invalid sequence of mailbox settings */
		if ((musb->xceiv->last_event == USB_EVENT_NONE) ||
			(!glue->core_power_state))
			break;

		musb->xceiv->last_event = USB_EVENT_NONE;

		if (musb->gadget_driver)
			omap2430_otg_interface(musb, false);

		if (is_otg_enabled(musb) || is_peripheral_enabled(musb))
			if (musb->gadget_driver) {
				pm_runtime_mark_last_busy(dev);
				pm_runtime_put_autosuspend(dev);
			}

		if (data->interface_type == MUSB_INTERFACE_UTMI) {
			omap2430_musb_set_vbus(musb, 0);
			if (musb->xceiv->otg->set_vbus)
				otg_set_vbus(musb->xceiv->otg, 0);
		}

		if (glue->control_dev) {
			val = SESSEND | IDDIG;
			omap4_usb_phy_mailbox(glue->control_dev, val);
		}

		glue->core_power_state = 0;
		break;
	default:
		dev_dbg(dev, "ID float\n");
	}
	mutex_unlock(&glue->core_access_mutex);
}


static void omap_musb_mailbox_work(struct work_struct *mailbox_work)
{
	struct omap2430_glue *glue = container_of(mailbox_work,
				struct omap2430_glue, omap_musb_mailbox_work);
	omap_musb_set_mailbox(glue);
}

static int omap2430_musb_init(struct musb *musb)
{
	int status = 0;
	struct device *dev = musb->controller;
	struct omap2430_glue *glue = dev_get_drvdata(dev->parent);

	/* We require some kind of external transceiver, hooked
	 * up through ULPI.  TWL4030-family PMICs include one,
	 * which needs a driver, drivers aren't always needed.
	 */
	musb->xceiv = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!musb->xceiv) {
		pr_err("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	status = pm_runtime_get_sync(dev);
	if (status < 0) {
		dev_err(dev, "pm_runtime_get_sync FAILED %d\n", status);
		goto err1;
	}

	/* Initial state of interface is ULPI state. Expected from TWL6030 */
	omap2430_otg_interface(musb, false);

	pr_debug("HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			musb_readl(musb->mregs, OTG_REVISION),
			musb_readl(musb->mregs, OTG_SYSCONFIG),
			musb_readl(musb->mregs, OTG_SYSSTATUS),
			musb_readl(musb->mregs, OTG_INTERFSEL),
			musb_readl(musb->mregs, OTG_SIMENABLE));

	usb_phy_init(musb->xceiv);

	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long) musb);

	if (glue->status != OMAP_MUSB_UNKNOWN)
		omap_musb_set_mailbox(glue);

	pm_runtime_put_sync(musb->controller);
	return 0;

err1:
	return status;
}

static void omap2430_musb_enable(struct musb *musb)
{
	u32		val;
	u8		devctl;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	struct device *dev = musb->controller;
	struct omap2430_glue *glue = dev_get_drvdata(dev->parent);
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct omap_musb_board_data *data = pdata->board_data;

	switch (glue->status) {

	case OMAP_MUSB_ID_GROUND:
		if (glue->control_dev) {
			val = AVALID | VBUSVALID;
			omap4_usb_phy_mailbox(glue->control_dev, val);
		}

		if (data->interface_type != MUSB_INTERFACE_UTMI)
			break;
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		/* start the session */
		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		while (musb_readb(musb->mregs, MUSB_DEVCTL) &
				MUSB_DEVCTL_BDEVICE) {
			cpu_relax();

			if (time_after(jiffies, timeout)) {
				dev_err(dev, "configured as A device timeout");
				break;
			}
		}
		break;

	case OMAP_MUSB_VBUS_VALID:
		if (glue->control_dev) {
			val = IDDIG | AVALID | VBUSVALID;
			omap4_usb_phy_mailbox(glue->control_dev, val);
		}
		break;

	default:
		break;
	}
}

static void omap2430_musb_disable(struct musb *musb)
{
	u32 val;
	struct device *dev = musb->controller;
	struct omap2430_glue *glue = dev_get_drvdata(dev->parent);

	if (musb->xceiv->last_event && glue->control_dev) {
		val = SESSEND | IDDIG;
		omap4_usb_phy_mailbox(glue->control_dev, val);
	}
}

static int omap2430_musb_exit(struct musb *musb)
{
	del_timer_sync(&musb_idle_timer);

	omap2430_low_level_exit(musb);
	usb_put_phy(musb->xceiv);

	return 0;
}

static const struct musb_platform_ops omap2430_ops = {
	.init		= omap2430_musb_init,
	.exit		= omap2430_musb_exit,

	.set_mode	= omap2430_musb_set_mode,
	.try_idle	= omap2430_musb_try_idle,

	.set_vbus	= omap2430_musb_set_vbus,

	.enable		= omap2430_musb_enable,
	.disable	= omap2430_musb_disable,
};

static u64 omap2430_dmamask = DMA_BIT_MASK(32);

static int __devinit omap2430_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	struct platform_device		*musb;
	struct omap2430_glue		*glue;
	struct clk                      *phy_clk;
	int				ret = -ENOMEM;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		goto err0;
	}

	musb = platform_device_alloc("musb-hdrc", -1);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err1;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &omap2430_dmamask;
	musb->dev.coherent_dma_mask	= omap2430_dmamask;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->status			= OMAP_MUSB_UNKNOWN;
	glue->control_dev		= omap_control_get();

	pdata->platform_ops		= &omap2430_ops;

	platform_set_drvdata(pdev, glue);

	/*
	 * REVISIT if we ever have two instances of the wrapper, we will be
	 * in big trouble
	 */

	phy_clk = clk_get(&pdev->dev, "fck");

	if (IS_ERR(phy_clk)) {
                dev_err(&pdev->dev, "failed to get PHY clock\n");
                ret = PTR_ERR(phy_clk);
                goto err2;
        }

	ret = clk_enable(phy_clk);
        if (ret) {
                dev_err(&pdev->dev, "failed to enable PHY clock\n");
                goto err3;
        }

	glue->phy_clk	= phy_clk;

	_glue	= glue;


	mutex_init(&glue->core_power_mutex);
	mutex_init(&glue->core_access_mutex);

	glue->work_queue = create_singlethread_workqueue("omap2430_wq");
	if (!glue->work_queue) {
		dev_err(&pdev->dev, "failed to create a workqueue\n");
		ret = -ENOMEM;
		goto err4;
	}
	INIT_WORK(&glue->omap_musb_mailbox_work, omap_musb_mailbox_work);

	ret = platform_device_add_resources(musb, pdev->resource,
			pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto errq;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto errq;
	}

	pm_runtime_enable(&pdev->dev);

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto errq;
	}

	return 0;

errq:
	destroy_workqueue(glue->work_queue);
err4:
	platform_device_put(musb);
err3:
	clk_disable(phy_clk);
err2:
	clk_put(phy_clk);
err1:
	kfree(glue);
err0:
	return ret;
}

static int __devexit omap2430_remove(struct platform_device *pdev)
{
	struct omap2430_glue		*glue = platform_get_drvdata(pdev);

	cancel_work_sync(&glue->omap_musb_mailbox_work);
	destroy_workqueue(glue->work_queue);

	clk_disable(glue->phy_clk);
	clk_put(glue->phy_clk);

	platform_device_del(glue->musb);
	platform_device_put(glue->musb);
	kfree(glue);

	return 0;
}

#ifdef CONFIG_PM

static int omap2430_runtime_suspend(struct device *dev)
{
	struct omap2430_glue		*glue = dev_get_drvdata(dev);
	struct musb			*musb = glue_to_musb(glue);

	dev_dbg(dev, "PB %d::%s\n", __LINE__, __func__);
	mutex_lock(&glue->core_power_mutex);
	if (musb) {
		omap2430_low_level_exit(musb);
		usb_phy_set_suspend(musb->xceiv, 1);
	}
	mutex_unlock(&glue->core_power_mutex);

	return 0;
}

static int omap2430_runtime_resume(struct device *dev)
{
	struct omap2430_glue		*glue = dev_get_drvdata(dev);
	struct musb			*musb = glue_to_musb(glue);

	dev_dbg(dev, "PB %d::%s\n", __LINE__, __func__);
	mutex_lock(&glue->core_power_mutex);
	if (musb) {
		omap2430_low_level_init(musb);
		usb_phy_set_suspend(musb->xceiv, 0);
	}
	mutex_unlock(&glue->core_power_mutex);

	return 0;
}

static int omap2430_suspend_noirq(struct device *dev)
{
	struct omap2430_glue            *glue = dev_get_drvdata(dev);

	clk_disable(glue->phy_clk);
	return 0;
}

static int omap2430_resume_noirq(struct device *dev)
{
	struct omap2430_glue            *glue = dev_get_drvdata(dev);
	int ret;

	ret = clk_enable(glue->phy_clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}
	return 0;
}

static struct dev_pm_ops omap2430_pm_ops = {
	.resume_noirq = omap2430_resume_noirq,
	.suspend_noirq = omap2430_suspend_noirq,
	.runtime_suspend = omap2430_runtime_suspend,
	.runtime_resume = omap2430_runtime_resume,
};

#define DEV_PM_OPS	(&omap2430_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver omap2430_driver = {
	.probe		= omap2430_probe,
	.remove		= __devexit_p(omap2430_remove),
	.driver		= {
		.name	= "musb-omap2430",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_DESCRIPTION("OMAP2PLUS MUSB Glue Layer");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");

static int __init omap2430_init(void)
{
	return platform_driver_register(&omap2430_driver);
}
subsys_initcall(omap2430_init);

static void __exit omap2430_exit(void)
{
	platform_driver_unregister(&omap2430_driver);
}
module_exit(omap2430_exit);
