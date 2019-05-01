/*
 * omap-usb2.c - USB PHY, talking to musb controller in OMAP.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/usb/omap_usb.h>
#include <linux/usb/phy_companion.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/mfd/omap_control.h>
#include <linux/usb/omap4_usb_phy.h>

/**
 * omap_usb2_set_comparator - links the comparator present in the sytem with
 *	this phy
 * @comparator - the companion phy(comparator) for this phy
 *
 * The phy companion driver should call this API passing the phy_companion
 * filled with set_vbus and start_srp to be used by usb phy.
 *
 * For use by phy companion driver
 */
void omap_usb2_set_comparator(struct phy_companion *comparator)
{
	struct usb_phy	*x = usb_get_phy(USB_PHY_TYPE_USB2);
	struct omap_usb	*phy = phy_to_omapusb(x);

	phy->comparator = comparator;
}
EXPORT_SYMBOL_GPL(omap_usb2_set_comparator);

static int omap_usb2_suspend(struct usb_phy *, int);

int omap_usb2_charger_detect(struct phy_companion *comparator)
{
	struct usb_phy  *x = usb_get_phy(USB_PHY_TYPE_USB2);
	struct omap_usb *phy = phy_to_omapusb(x);
	int charger = 0;

	omap_usb2_suspend(x, 0);
	charger = omap_usb_charger_detect(phy->control_dev);
	omap_usb2_suspend(x, 1);

	return charger;
}
EXPORT_SYMBOL_GPL(omap_usb2_charger_detect);

static int omap_usb_set_vbus(struct usb_otg *otg, bool enabled)
{
	struct omap_usb *phy = phy_to_omapusb(otg->phy);

	if (phy->comparator)
		return phy->comparator->set_vbus(phy->comparator, enabled);
	else
		return -ENODEV;
}

static int omap_usb_start_srp(struct usb_otg *otg)
{
	struct omap_usb *phy = phy_to_omapusb(otg->phy);
	if (phy->comparator)
		return phy->comparator->start_srp(phy->comparator);
	else
		return -ENODEV;
}

static int omap_usb_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct usb_phy	*phy = otg->phy;

	otg->host = host;
	if (!host)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int omap_usb_set_peripheral(struct usb_otg *otg,
		struct usb_gadget *gadget)
{
	struct usb_phy	*phy = otg->phy;

	otg->gadget = gadget;
	if (!gadget)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int omap_usb2_suspend(struct usb_phy *x, int suspend)
{
	u32		ret;
	struct omap_usb *phy = phy_to_omapusb(x);

	dev_dbg(phy->dev, "%s(%d): phy->is_suspended=%d\n",
		__func__, suspend, phy->is_suspended);

	if (suspend && !phy->is_suspended) {
		omap4_usb_phy_power(phy->control_dev, 0);

		pm_runtime_put_sync(phy->dev);
		clk_disable(phy->wkupclk);
		clk_disable(phy->optclk);

		phy->is_suspended = 1;
	} else if (!suspend && phy->is_suspended) {
		if (phy->optclk) {
			ret = clk_enable(phy->optclk);
			if (ret) {
				dev_err(phy->dev,
					"Failed to enable optclk %d\n", ret);
				goto err3;
			}
		}
		ret = clk_enable(phy->wkupclk);
		if (ret) {
			dev_err(phy->dev, "Failed to enable wkupclk %d\n", ret);
			goto err2;
		}
		ret = pm_runtime_get_sync(phy->dev);
		if (ret < 0) {
			dev_err(phy->dev, "get_sync failed with err %d\n",
									ret);
			goto err1;
		}

		omap4_usb_phy_power(phy->control_dev, 1);

		phy->is_suspended = 0;
	}

	dev_dbg(phy->dev, "%s(%d) Leaving gracefully: phy->is_suspended=%d\n",
		__func__,  suspend, phy->is_suspended);
	return 0;

err1:
	clk_disable(phy->wkupclk);
err2:
	clk_disable(phy->optclk);
err3:
	dev_dbg(phy->dev, "%s(%d) Leaving through ERROR path: phy->is_suspended=%d\n",
		__func__, suspend, phy->is_suspended);
	return ret;
}

static int __devinit omap_usb2_probe(struct platform_device *pdev)
{
	struct omap_usb			*phy;
	struct usb_otg			*otg;
	struct clk			*optclk;

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		dev_err(&pdev->dev, "unable to allocate memory for USB2 PHY\n");
		return -ENOMEM;
	}

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for USB OTG\n");
		return -ENOMEM;
	}

	phy->dev		= &pdev->dev;

	phy->phy.dev		= phy->dev;
	phy->phy.label		= "omap-usb2";
	phy->phy.set_suspend	= omap_usb2_suspend;
	phy->phy.otg		= otg;

	phy->control_dev	= omap_control_get();
	if (IS_ERR(phy->control_dev)) {
		dev_err(&pdev->dev, "no control device present in system\n");
		return PTR_ERR(phy->control_dev);
	}

	phy->is_suspended	= 1;
	omap4_usb_phy_power(phy->control_dev, 0);

	otg->set_host		= omap_usb_set_host;
	otg->set_peripheral	= omap_usb_set_peripheral;
	otg->set_vbus		= omap_usb_set_vbus;
	otg->start_srp		= omap_usb_start_srp;
	otg->phy		= &phy->phy;

	phy->wkupclk = clk_get(phy->dev, "usb_phy_cm_clk32k");
	optclk = clk_get(phy->dev, "usb_otg_ss_refclk960m");
	if (!IS_ERR(optclk))
		phy->optclk = optclk;

	usb_add_phy(&phy->phy, USB_PHY_TYPE_USB2);

	platform_set_drvdata(pdev, phy);

	/* Start with disabled charger detection */
	omap_usb_charger_enable(phy->control_dev, 0);

	pm_runtime_enable(phy->dev);

	return 0;
}

static int __devexit omap_usb2_remove(struct platform_device *pdev)
{
	struct omap_usb	*phy = platform_get_drvdata(pdev);

	usb_remove_phy(&phy->phy);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver omap_usb2_driver = {
	.probe		= omap_usb2_probe,
	.remove		= __devexit_p(omap_usb2_remove),
	.driver		= {
		.name	= "omap-usb2",
		.owner	= THIS_MODULE,
	},
};

static int __init usb2_omap_init(void)
{
	return platform_driver_register(&omap_usb2_driver);
}
arch_initcall(usb2_omap_init);

static void __exit usb2_omap_exit(void)
{
	platform_driver_unregister(&omap_usb2_driver);
}
module_exit(usb2_omap_exit);

MODULE_ALIAS("platform: omap_usb2");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_DESCRIPTION("OMAP USB2 PHY DRIVER");
MODULE_LICENSE("GPL");
