/*
 * AUO panel support
 *
 * Copyright (c) 2011 Barnes & Noble
 * Based on panel-taal.c
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <asm/mach-types.h>
#include <video/omapdss.h>

static struct omap_video_timings auo_panel_timings = {
	.x_res          = 1440,
	.y_res          = 960,
	.pixel_clock    = 76800, /* in kHz */
	.hfp            = 1,
	.hsw            = 126,
	.hbp            = 1,
	.vfp            = 1,
	.vsw            = 14,
	.vbp            = 1,
};

struct panel_config {
	u32 width_in_um;
	u32 height_in_um;
};

static struct panel_config panel_configs[] = {
	{
		.width_in_um = 190000,
		.height_in_um = 127000,
	}
};

static void auo_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int auo_panel_probe(struct omap_dss_device *dssdev)
{
	int ret = 0;
	struct panel_config *panel_config = NULL;

	/* experimental setup - panel_config structure might be
	 * further changed internally if needed*/
	panel_config = &panel_configs[0];

	dssdev->panel.width_in_um = panel_config->width_in_um;
	dssdev->panel.height_in_um = panel_config->height_in_um;

	dssdev->panel.config	= OMAP_DSS_LCD_TFT;
	dssdev->panel.timings	= auo_panel_timings;

	return ret;
}

static void auo_panel_remove(struct omap_dss_device *dssdev)
{
}

static int auo_panel_start(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			return r;
	}

	r = omapdss_dpi_display_enable(dssdev);
	if (r && dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static void auo_panel_stop(struct omap_dss_device *dssdev)
{
	omapdss_dpi_display_disable(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void auo_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		auo_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int auo_panel_suspend(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	auo_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
err:
	return r;
}

static int auo_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	r = auo_panel_start(dssdev);
err:
	return r;
}

static int auo_panel_resume(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	r = auo_panel_start(dssdev);
err:
	return r;
}

static struct omap_dss_driver auo_driver = {
	.probe		= auo_panel_probe,
	.remove		= auo_panel_remove,
	.enable		= auo_panel_enable,
	.disable	= auo_panel_disable,
	.suspend	= auo_panel_suspend,
	.resume		= auo_panel_resume,
	.get_timings    = auo_get_timings,
	.set_timings    = dpi_set_timings,
	.check_timings  = dpi_check_timings,
	.driver		= {
		.name	= "auo",
		.owner	= THIS_MODULE,
	},
};

static int __init auo_lcd_init(void)
{
	return omap_dss_register_driver(&auo_driver);
}

static void __exit auo_lcd_exit(void)
{
	omap_dss_unregister_driver(&auo_driver);
}

module_init(auo_lcd_init);
module_exit(auo_lcd_exit);

MODULE_LICENSE("GPL");
