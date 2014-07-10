/*
 * Orise TCON based display panel driver.
 *
 * Copyright (C) Barnes & Noble Inc. 2012
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
#include <linux/delay.h>
#include <linux/gpio.h>

#include <video/omapdss.h>
#include <video/omap-panel-dsi.h>

/* device private data structure */
struct orise_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel_cmd;
	const struct panel_dsi_fps_data *current_fps;
	char cabc_mode[6];
};

static inline u8 bpp_to_datatype(int bpp)
{
	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */

	switch(bpp) {
	case 16:
		return 0x0e;
	case 18:
		return 0x1e;
	case 24:
		return 0x3e;
	default:
		pr_err("unsupported pixel size: %d", bpp);
		BUG();
	}

	return 0;
} 

static inline struct panel_dsi_fps_data ** get_dsi_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

static void orise_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void orise_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
}

static int orise_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (dssdev->panel.timings.x_res != timings->x_res ||
			dssdev->panel.timings.y_res != timings->y_res ||
			dssdev->panel.timings.pixel_clock != timings->pixel_clock ||
			dssdev->panel.timings.hsw != timings->hsw ||
			dssdev->panel.timings.hfp != timings->hfp ||
			dssdev->panel.timings.hbp != timings->hbp ||
			dssdev->panel.timings.vsw != timings->vsw ||
			dssdev->panel.timings.vfp != timings->vfp ||
			dssdev->panel.timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void orise_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static int orise_set_current_fps(struct omap_dss_device *dssdev, const char *fps)
{
	struct orise_data *odata = dev_get_drvdata(&dssdev->dev);
	struct panel_dsi_fps_data **fps_data = get_dsi_data(dssdev);
	int i = 0;
	struct panel_dsi_fps_data *cur;

	if (fps_data == NULL)
		return -EINVAL;

	while ((cur = fps_data[i++]) != NULL) {
		if (strcmp(cur->name, fps) == 0) {
			dssdev->clocks.dsi.regm = cur->regm;
			dssdev->clocks.dsi.tlpx = cur->tlpx;
			dssdev->clocks.dsi.tclk.zero = cur->tclk.zero;
			dssdev->clocks.dsi.tclk.prepare = cur->tclk.prepare;
			dssdev->clocks.dsi.tclk.trail = cur->tclk.trail;
			dssdev->clocks.dsi.ths.zero = cur->ths.zero;
			dssdev->clocks.dsi.ths.prepare = cur->ths.prepare;
			dssdev->clocks.dsi.ths.exit = cur->ths.exit;
			dssdev->clocks.dsi.ths.trail = cur->ths.trail;

			dsi_bus_lock(dssdev);
			// Only reconfigure if the display is active
			// still grab the locks to make sure we don't change
			// while executing the reconfig
			if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
				omap_dsi_reconfigure_dsi_clocks(dssdev);

			dsi_bus_unlock(dssdev);

			odata->current_fps = cur;
			return 0;
		}
	}

	return -EINVAL;
}

static ssize_t orise_get_current_fps(struct omap_dss_device *dssdev, char *buf, size_t len)

{
	struct orise_data *odata = dev_get_drvdata(&dssdev->dev);

	if (odata->current_fps) {
		return snprintf(buf, len, "%s", odata->current_fps->name);
	}

	return 0;
}

static ssize_t orise_get_fps(struct omap_dss_device *dssdev, char *buf, size_t len)
{
	struct panel_dsi_fps_data **fps_data = get_dsi_data(dssdev);
	struct panel_dsi_fps_data *cur;
	int i = 0;
	size_t r = 0;

	if (!fps_data)
		return 0;

	while ((cur = fps_data[i++]) != NULL) {
		r += snprintf(buf + r, len - r, " %s", cur->name);
	}

	return r;
}

static int orise_read_reg(struct omap_dss_device *dssdev, u16 reg, u8 *value)
{
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[3];

	buf[0] = 0x19;
	buf[1] = (reg & 0xFF);
	buf[2] = (reg >> 8) & 0xFF;

	// long write 0x39
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel_cmd, buf, sizeof(buf));

	if (r < 0) {
		dev_err(&dssdev->dev, "error writing long packet %d\n", r);
		return r;
	}

	// packet return
	r = dsi_vc_set_max_rx_packet_size(dssdev, d2d->channel_cmd, 1);	

	if (r < 0) {
		dev_err(&dssdev->dev, "error setting max rx packet size %d\n", r);
		return r;
	}

	// dcs read cmd
	r = dsi_vc_dcs_read(dssdev, d2d->channel_cmd, 0x2E, (u8 *) value, sizeof(*value));
	return r;
}

static int orise_write_reg(struct omap_dss_device *dssdev, u16 reg, u8 value)
{
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[4];

	buf[0] = 0x19;
	buf[1] = reg & 0xFF;
	buf[2] = (reg >> 8) & 0xFF;
	buf[3] = value;

	// long write 0x39
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel_cmd, buf, sizeof(buf));

	if (r < 0) {
		dev_err(&dssdev->dev, "error writing long packet %d\n", r);
		return r;
	}

	return r;
}

static ssize_t orise_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	u16 reg = 0;
	u8 value = 0;
	int r;
	
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		return -EAGAIN;
	}

	r = sscanf(buf, "%hx %hhx", &reg, &value);

	if (r < 1 || r > 2) {
		dev_warn(dev, "invalid format, should be either:\n\t to read - <page> <reg>\n\tto write - <page> <reg> <value>\n");
		return -EINVAL;
	}

	dsi_bus_lock(dssdev);
	
	if (r == 2) {
		r = orise_write_reg(dssdev, reg, value);
	} else {
		r = orise_read_reg(dssdev, reg, &value);
	
		if (r >= 0) {
			dev_info(&dssdev->dev, "%hx=%hhx\n", reg, value);
		}
	}
	dsi_bus_unlock(dssdev);

	return (r < 0) ? r : size;
}

static int orise_enable_reg(struct device *dev, u8 reg, int onoff)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 buf[2];
	int r;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		return -EAGAIN;
	}

	buf[0] = reg;
	
	if (onoff) {
		buf[1] = 0x55;
	} else {
		buf[1] = 0xAA;
	}

	dsi_bus_lock(dssdev);
	r = dsi_vc_gen_short_write_nosync(dssdev, d2d->channel_cmd, buf, sizeof(buf));
	dsi_bus_unlock(dssdev);

	return r;
}

static ssize_t orise_cabc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int enable = 0;
	int r;

	if (sscanf(buf, "%u", &enable) < 1) {
		dev_err(dev, "invalid value %s\n", buf);
		return -EINVAL;
	}

	r = orise_enable_reg(dev, 0x15, enable);

	return (r < 0) ? r : size;
}

static ssize_t orise_cabc_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	if (strncmp(buf, "none", 4) == 0) {
		r = orise_enable_reg(dev, 0x15, 0);
		
		if (r < 0) {
			dev_warn(&dssdev->dev, "failed to disable CABC %d\n", r);
		}
		
		strncpy(d2d->cabc_mode, buf, 4);
		d2d->cabc_mode[4] = 0;
	} else if (strncmp(buf, "video", 5) == 0) {
		r = orise_enable_reg(dev, 0x15, 0);
	
		// video mode does nothing for AUO	
		if (r < 0) {
			dev_warn(&dssdev->dev, "failed to enable CABC %d\n", r);
		}

/*		dsi_bus_lock(dssdev);
		r = orise_write_reg(dssdev, 0xC7, 0x20); 
		dsi_bus_unlock(dssdev);*/

		strncpy(d2d->cabc_mode, buf, 5);
		d2d->cabc_mode[5] = 0;
	} else {
		dev_info(dev, "invalid cabc mode string %s, allowed values are none, video\n", buf);
		return -EINVAL;
	}

	if (r < 0) { 
		dev_warn(dev, "failed to set CABC mode %d\n", r);
		return r;
	}

	
	return size;
}

static ssize_t orise_cabc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	
	return snprintf(buf, PAGE_SIZE, "%s\n", d2d->cabc_mode);	
}

static DEVICE_ATTR(cabc, S_IWGRP | S_IWUSR | S_IRUGO, NULL, orise_cabc_store);
static DEVICE_ATTR(reg, S_IWGRP | S_IWUSR, NULL, orise_reg_store);
static DEVICE_ATTR(cabc_mode, S_IWUSR | S_IWGRP | S_IRUGO , orise_cabc_mode_show, orise_cabc_mode_store);

static struct device_attribute *orise_sysfs_attrs[] = {
	&dev_attr_cabc,
	&dev_attr_reg,
	&dev_attr_cabc_mode,
	NULL,
};

static int orise_probe(struct omap_dss_device *dssdev)
{
	struct orise_data *d2d;
	struct device_attribute *attr;
	int i = 0;
	int r = 0;

	dev_dbg(&dssdev->dev, "orise_probe");

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		return r;
	}

	d2d->dssdev = dssdev;
	strcpy(d2d->cabc_mode, "none");

	if (get_dsi_data(dssdev))
		d2d->current_fps = get_dsi_data(dssdev)[0];

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r)
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r)
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");

	r = omap_dsi_request_vc(dssdev, &d2d->channel_cmd);
	if (r)
		dev_err(&dssdev->dev, "failed to get virtual channel_cmd\n");

	r = omap_dsi_set_vc_id(dssdev, d2d->channel_cmd, 0);

	while((attr = orise_sysfs_attrs[i++]) != NULL) {
		r = device_create_file(&dssdev->dev, attr);

		if (r)
			dev_err(&dssdev->dev, "failed to create sysfs file\n");
	}

	dev_dbg(&dssdev->dev, "orise_probe done\n");

	/* do I need an err and kfree(d2d) */
	return r;
}

static void orise_remove(struct omap_dss_device *dssdev)
{
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel_cmd);
	kfree(d2d);
}

static int orise_power_on(struct omap_dss_device *dssdev)
{
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
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

	if (!dssdev->skip_init) {
		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);

		/* do extra job to match kozio registers (???) */
		dsi_videomode_panel_preinit(dssdev);
	}

	dsi_video_mode_enable(dssdev, bpp_to_datatype(dssdev->ctrl.pixel_size));
	dssdev->skip_init = false;

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_disp_enable:
	omapdss_dsi_display_disable(dssdev, false, false);
	
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void orise_power_off(struct omap_dss_device *dssdev)
{
	dsi_video_mode_disable(dssdev, false);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void orise_disable(struct omap_dss_device *dssdev)
{
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		orise_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int orise_enable(struct omap_dss_device *dssdev)
{
	struct orise_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = orise_power_on(dssdev);

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

static int orise_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	orise_disable(dssdev);
	return 0;
}

static struct omap_dss_driver orise_driver = {
	.probe		= orise_probe,
	.remove		= orise_remove,

	.enable		= orise_enable,
	.disable	= orise_disable,
	.suspend	= orise_suspend,
	.resume		= NULL,

	.get_resolution	= orise_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= orise_get_timings,
	.set_timings	= orise_set_timings,
	.check_timings	= orise_check_timings,

	.set_current_fps	= orise_set_current_fps,
	.get_current_fps	= orise_get_current_fps,
	.get_fps		= orise_get_fps,

	.driver         = {
		.name   = "orise-panel",
		.owner  = THIS_MODULE,
	},
};

static int __init orise_init(void)
{
	omap_dss_register_driver(&orise_driver);
	return 0;
}

static void __exit orise_exit(void)
{
	omap_dss_unregister_driver(&orise_driver);
}

module_init(orise_init);
module_exit(orise_exit);

MODULE_AUTHOR("David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>");
MODULE_DESCRIPTION("Orise TCON display");
MODULE_LICENSE("GPL");
