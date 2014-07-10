/*
 * Hydis display driver
 *
 * Copyright (C) Texas Instruments
 * Author: Aleksandar Momiroski <x0171959@ti.com>
 *
 * Based on original version from:
 *	Jerry Alexander <x0135174@ti.com>
 *	Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/err.h>
#include <linux/mutex.h>

#include <video/omapdss.h>
#include <video/omap-panel-hydis.h>

/* HV080X01-100 Product Specification */
/*  (hfp + hsw + hbp + xres) % 4[mipi data lane] must be zero */
#define HYDIS_WIDTH	768
#define HYDIS_HEIGHT	1024
#define HYDIS_PCLK	51520
#define HYDIS_HFP	32
#define HYDIS_HSW	4
#define HYDIS_HBP	32
#define HYDIS_VFP	3
#define HYDIS_VSW	1
#define HYDIS_VBP	5


static struct omap_video_timings hydis_timings = {
	.x_res		= HYDIS_WIDTH,
	.y_res		= HYDIS_HEIGHT,
	.pixel_clock	= HYDIS_PCLK,
	.hfp		= HYDIS_HFP,
	.hsw            = HYDIS_HSW,
	.hbp            = HYDIS_HBP,
	.vfp            = HYDIS_VFP,
	.vsw            = HYDIS_VSW,
	.vbp            = HYDIS_VBP,
};

/* device private data structure */
struct hydis_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
};

static struct hydis_board_data *get_board_data(struct omap_dss_device *dssdev)
{
	return (struct hydis_board_data *)dssdev->data;
}

static void hydis_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hydis_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
}

static int hydis_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (hydis_timings.x_res != timings->x_res ||
			hydis_timings.y_res != timings->y_res ||
			hydis_timings.pixel_clock != timings->pixel_clock ||
			hydis_timings.hsw != timings->hsw ||
			hydis_timings.hfp != timings->hfp ||
			hydis_timings.hbp != timings->hbp ||
			hydis_timings.vsw != timings->vsw ||
			hydis_timings.vfp != timings->vfp ||
			hydis_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void hydis_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = hydis_timings.x_res;
	*yres = hydis_timings.y_res;
}

static int hydis_probe(struct omap_dss_device *dssdev)
{
	struct hydis_data *d2d;
	struct hydis_board_data *board_data = get_board_data(dssdev);
	int r = 0;

	dev_dbg(&dssdev->dev, "hydis_probe\n");

	hydis_timings.x_res = board_data->x_res;
	hydis_timings.y_res = board_data->y_res;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = hydis_timings;
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		return r;
	}

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r)
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r)
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");

	dev_dbg(&dssdev->dev, "hydis_probe done\n");

	/* do I need an err and kfree(d2d) */
	return r;
}

static void hydis_remove(struct omap_dss_device *dssdev)
{
	struct hydis_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	kfree(d2d);
}

static int hydis_power_on(struct omap_dss_device *dssdev)
{
	struct hydis_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	dev_dbg(&dssdev->dev, "power_on\n");

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);

	/* do extra job to match kozio registers (???) */
	dsi_videomode_panel_preinit(dssdev);

	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */
	dsi_video_mode_enable(dssdev, 0x3e);

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_disp_enable:
	omapdss_dsi_display_disable(dssdev, false, false);
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void hydis_power_off(struct omap_dss_device *dssdev)
{
	dsi_video_mode_disable(dssdev, false);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void hydis_disable(struct omap_dss_device *dssdev)
{
	struct hydis_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		hydis_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int hydis_enable(struct omap_dss_device *dssdev)
{
	struct hydis_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = hydis_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&d2d->lock);

	return r;
}

static int hydis_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	hydis_disable(dssdev);
	return 0;
}

static struct omap_dss_driver hydis_driver = {
	.probe		= hydis_probe,
	.remove		= hydis_remove,

	.enable		= hydis_enable,
	.disable	= hydis_disable,
	.suspend	= hydis_suspend,
	.resume		= NULL,

	.get_resolution	= hydis_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= hydis_get_timings,
	.set_timings	= hydis_set_timings,
	.check_timings	= hydis_check_timings,

	.driver         = {
		.name   = "hydis-panel",
		.owner  = THIS_MODULE,
	},
};

static int __init hydis_init(void)
{
	omap_dss_register_driver(&hydis_driver);
	return 0;
}

static void __exit hydis_exit(void)
{
	omap_dss_unregister_driver(&hydis_driver);
}

module_init(hydis_init);
module_exit(hydis_exit);

MODULE_AUTHOR("Aleksandar Momiroski <x0171959@ti.com>");
MODULE_DESCRIPTION("Hydis panel driver");
MODULE_LICENSE("GPL");
