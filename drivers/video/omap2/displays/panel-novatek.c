/*
 * Novatek TCON based display panel driver.
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
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c/maxim9606.h>
#include <linux/earlysuspend.h>

#include <video/omapdss.h>
#include <video/omap-panel-dsi.h>

/* device private data structure */
struct novatek_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel_cmd;
	struct dentry *dbg_dir;
	const struct panel_dsi_fps_data *current_fps;
	char cabc_mode[6];
};

static inline struct panel_dsi_fps_data ** get_dsi_fps_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

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

static void novatek_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int novatek_unlock_page(struct omap_dss_device *dssdev, int channel, u32 page)
{
	u8 data[2];

	data[0] = 0xf3;	
	data[1] = (0xa0 + page);

	return dsi_vc_gen_short_write_nosync(dssdev, channel, data, sizeof(data));
}

static int novatek_lock_page(struct omap_dss_device *dssdev, int channel)
{
	u8 data[2] = { 0, 0 };
	
	return dsi_vc_gen_short_write_nosync(dssdev, channel, data, 0);
}

static int novatek_write_reg(struct omap_dss_device *dssdev, int channel, u32 reg, u32 value)
{
	u8 data[2];
	
	data[0] = reg;
	data[1] = value;

	return dsi_vc_gen_short_write_nosync(dssdev, channel, data, sizeof(data));
}

static int novatek_read_reg(struct omap_dss_device *dssdev, int channel, u8 reg, u8 *value)
{
	return dsi_vc_gen_read_1(dssdev, channel, reg, value, 1);
}

static int novatek_unlock_and_write(struct omap_dss_device *dssdev, u8 page, u8 reg, u8 value)
{
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;

	r = novatek_unlock_page(dssdev, d2d->channel_cmd, page);

	if (r < 0) {
		dev_err(&dssdev->dev, "error unlocking page %02x %d\n", page, r);
		return r;
	}

	r = novatek_write_reg(dssdev, d2d->channel_cmd, reg, value);
	
	if (r < 0) {
		dev_err(&dssdev->dev, "error writing register %02x %02x %d\n", reg, value, r);
		// don't exit here, we need to relock the page
	}

	r = novatek_lock_page(dssdev, d2d->channel_cmd);

	if (r < 0) {
		dev_err(&dssdev->dev, "error locking page %02x %d\n", page, r);
	}

	return r;
}

static int novatek_unlock_and_read(struct omap_dss_device *dssdev, u8 page, u8 reg, u8 *value)
{
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;

	r = novatek_unlock_page(dssdev, d2d->channel0, page);

	if (r < 0) {
		dev_err(&dssdev->dev, "error unlocking page %02x %d\n", page, r);
		return r;
	}

	udelay(10);
	r = dsi_vc_set_max_rx_packet_size(dssdev, d2d->channel0, 1);

	if (r < 0) {
		dev_err(&dssdev->dev, "error setting rx packet size %d", r);
		goto err;
	}

	udelay(10);
	r = novatek_read_reg(dssdev, d2d->channel0, reg, value);

	if (r < 0) {
		dev_err(&dssdev->dev, "error reading register %02x %d\n", reg, r);
		// don't exit here, we need to relock the page
	}

err:
	udelay(10);
	r = novatek_lock_page(dssdev, d2d->channel0);

	if (r < 0) {
		dev_err(&dssdev->dev, "error locking page %02x %d\n", page, r);
	}

	return r;
}

static void novatek_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
}

static int novatek_check_timings(struct omap_dss_device *dssdev,
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

static void novatek_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static int novatek_set_current_fps(struct omap_dss_device *dssdev, const char *fps)
{
	struct novatek_data *ndata = dev_get_drvdata(&dssdev->dev);
	struct panel_dsi_fps_data **fps_data = get_dsi_fps_data(dssdev);
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

			ndata->current_fps = cur;
			return 0;
		}
	}

	return -EINVAL;
}

static ssize_t novatek_get_current_fps(struct omap_dss_device *dssdev, char *buf, size_t len)
{
	struct novatek_data *ndata = dev_get_drvdata(&dssdev->dev);

	if (ndata->current_fps) {
		return snprintf(buf, len, "%s", ndata->current_fps->name);
	}

	return 0;
}

static ssize_t novatek_get_fps(struct omap_dss_device *dssdev, char *buf, size_t len)
{
	struct panel_dsi_fps_data **fps_data = get_dsi_fps_data(dssdev);
	struct panel_dsi_fps_data *cur;
	int i = 0;
	size_t r = 0;

	if (fps_data == NULL)
		return 0;

	while ((cur = fps_data[i++]) != NULL) {
		r += snprintf(buf + r, len - r, " %s", cur->name);
	}

	return r;
}

static ssize_t novatek_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 page, reg, value = 0;
	int r;
	
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		return -EAGAIN;
	}

	r = sscanf(buf, "%hhx %hhx %hhx", &page, &reg, &value);

	if (r < 2 || r > 3) {
		dev_warn(dev, "invalid format, should be either:\n\t to read - <page> <reg>\n\tto write - <page> <reg> <value>\n");
		return -EINVAL;
	}

	dsi_bus_lock(dssdev);

	if (r == 3) {
		r = novatek_unlock_and_write(dssdev, page, reg, value);
	} else {
		dsi_video_mode_disable(dssdev, true);
		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, false);
		r = novatek_unlock_and_read(dssdev, page, reg, &value);
		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);
		dsi_video_mode_enable(dssdev, bpp_to_datatype(dssdev->ctrl.pixel_size));
	
		if (r >= 0) {
			dev_info(&dssdev->dev, "%hhx:%hhx=%hhx\n", page, reg, value);
		}
	}
		
	dsi_bus_unlock(dssdev);

	return (r < 0) ? r : size;
}

static ssize_t novatek_cabc_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	if (strncmp(buf, "none", 4) == 0) {
		dsi_bus_lock(dssdev);
		r = novatek_unlock_and_write(dssdev, 0, 0xE5, 0x0);

		if (r < 0) {
			dev_warn(dev, "failed to set CABC abrupt threshold %d\n", r);
		}

		r = novatek_unlock_and_write(dssdev, 0, 0xEA, 0x0);
		dsi_bus_unlock(dssdev);

		strncpy(d2d->cabc_mode, buf, 4);
		d2d->cabc_mode[4] = 0;

	} else if (strncmp(buf, "video", 5) == 0) {
		dsi_bus_lock(dssdev);
		r = novatek_unlock_and_write(dssdev, 0, 0xE5, 0xF0);
		
		if (r < 0) {
			dev_warn(dev, "failed to set CABC abrupt threshold %d\n", r);
		}

		// Quick hack to differentiate between LG and Samsung,
		// this value should come from the board data but... 
		if (dssdev->panel.timings.x_res == 900) {
			r = novatek_unlock_and_write(dssdev, 0, 0xEA, 0x50);
		} else {
			r = novatek_unlock_and_write(dssdev, 0, 0xEA, 0x80);
		}
		dsi_bus_unlock(dssdev);

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

static ssize_t novatek_cabc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
	
	return snprintf(buf, PAGE_SIZE, "%s\n", d2d->cabc_mode);	
}

static DEVICE_ATTR(reg, S_IWGRP | S_IWUSR, NULL, novatek_reg_store);
static DEVICE_ATTR(cabc_mode, S_IWUSR | S_IWGRP | S_IRUGO, novatek_cabc_mode_show, novatek_cabc_mode_store);

static struct device_attribute *novatek_sysfs_attrs[] = {
	&dev_attr_reg,
	&dev_attr_cabc_mode,
	NULL,
};

static int novatek_probe(struct omap_dss_device *dssdev)
{
	struct novatek_data *d2d;
	struct device_attribute *attr;
	int i = 0;
	int r = 0;

	dev_dbg(&dssdev->dev, "novatek_probe");

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		return r;
	}

	d2d->dssdev = dssdev;
	strcpy(d2d->cabc_mode, "none");
	
	if (get_dsi_fps_data(dssdev))
		d2d->current_fps = get_dsi_fps_data(dssdev)[0];

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
	if (r)
		dev_err(&dssdev->dev, "failed to set VC_ID1\n");

	while((attr = novatek_sysfs_attrs[i++]) != NULL) {
		r = device_create_file(&dssdev->dev, attr);

		if (r)
			dev_err(&dssdev->dev, "failed to create sysfs file\n");
	}

	dev_dbg(&dssdev->dev, "novatek_probe done\n");
	/* do I need an err and kfree(d2d) */
	return r;
}

static void novatek_remove(struct omap_dss_device *dssdev)
{
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel_cmd);
	kfree(d2d);
}

struct maxim9606 {
	struct i2c_client *client;
	struct maxim9606_platform_data *pdata;
	struct early_suspend suspend;
	bool softstart;
};

static int novatek_power_on(struct omap_dss_device *dssdev)
{
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
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
		msleep(1);
		r = dsi_vc_turn_on_peripheral(dssdev, d2d->channel0);
		msleep(1);

		if (r) {
			dev_err(&dssdev->dev, "turn on peripheral failed: %d", r);
		}
		/* do extra job to match kozio registers (???) */
		dsi_videomode_panel_preinit(dssdev);
		msleep(1);
	}

	dsi_video_mode_enable(dssdev, bpp_to_datatype(dssdev->ctrl.pixel_size));
	dssdev->skip_init=false;

	r = novatek_unlock_and_write(dssdev, 0, 0xE5, 0x0);

	if (r < 0) {
		dev_warn(&dssdev->dev, "failed to set CABC abrupt threshold %d\n", r);
	}

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_disp_enable:
	omapdss_dsi_display_disable(dssdev, false, false);
	
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void novatek_power_off(struct omap_dss_device *dssdev)
{
	dsi_video_mode_disable(dssdev, false);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void novatek_disable(struct omap_dss_device *dssdev)
{
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		novatek_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int novatek_enable(struct omap_dss_device *dssdev)
{
	struct novatek_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = novatek_power_on(dssdev);

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

static int novatek_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	novatek_disable(dssdev);
	return 0;
}

#define I2C_WRITE_MSG(a, d) { .addr = a, .flags = 0, .len = sizeof(d), .buf = d }

static void maxim9606_enable(struct maxim9606 *mx)
{
	struct timespec boost_on;
	struct timespec boost_done;

	int retry_count = 0;
	u8 value = 0;
	const u8 buf[2] = { 0x54, 0x4D };
	s32 r = 0;
	u8 test_mode[3] = { 0xFF, 0x54, 0x4D };
	u8 cmd1[2] = { 0x5F, 0x02 };
	u8 cmd2[2] = { 0x60, 0x14 };
	u8 cmd3[2] = { 0x10, 0xDF };
	u8 cmd4[2] = { 0x5F, 0x00 };
	u8 cmd5[2] = { 0xFF, 0x00 };

	struct i2c_msg seq[6] = {
		I2C_WRITE_MSG(0x74, test_mode),
		I2C_WRITE_MSG(0x74, cmd1),
		I2C_WRITE_MSG(0x74, cmd2),
		I2C_WRITE_MSG(0x74, cmd3),
		I2C_WRITE_MSG(0x74, cmd4),
		I2C_WRITE_MSG(0x74, cmd5),
	};

retry:
	if (retry_count > 5) {
		dev_err(&mx->client->dev, "retry count exceeded, bailing\n");
		return;
	}

	retry_count++;
	if (mx->pdata->power_on) {
		mx->pdata->power_on(&mx->client->dev);
	}

	if (!mx->softstart) {
		dev_info(&mx->client->dev, "soft start disabled");
		return;
	}

	msleep(20);

	ktime_get_ts(&boost_on);
	r = i2c_smbus_write_byte_data(mx->client, 0x10, 0xD7);
	udelay(1000);

	if (r < 0) {
		dev_warn(&mx->client->dev, "disable boost %d\n", r);

		if (mx->pdata->power_off) {
			mx->pdata->power_off(&mx->client->dev);
		}

		goto retry;
	}

	r = i2c_transfer(mx->client->adapter, seq, ARRAY_SIZE(seq));
	ktime_get_ts(&boost_done);

	if (r < 0) {
		dev_warn(&mx->client->dev, "soft start %d\n", r);
	}

	msleep(55);
	value = i2c_smbus_read_byte_data(mx->client, 0x39);
	
	if (r < 0 || value != 0) {
		dev_warn(&mx->client->dev, "maxim status %hhu %d\n", value, r);
	}

	if (ktime_us_delta(timespec_to_ktime(boost_done), timespec_to_ktime(boost_on)) > 3000) {
		dev_warn(&mx->client->dev, "boost on took too long, %lld us\n", ktime_us_delta(timespec_to_ktime(boost_done), timespec_to_ktime(boost_on)));

		if (mx->pdata->power_off) {
			mx->pdata->power_off(&mx->client->dev);
		}

		goto retry;
	}
}

static void maxim9606_disable(struct maxim9606 *mx)
{
	if (mx->pdata->power_off) {
		mx->pdata->power_off(&mx->client->dev);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void maxim9606_late_resume(struct early_suspend *h)
{
	struct maxim9606 *mx = container_of(h, struct maxim9606, suspend);
	maxim9606_enable(mx);
}

static void maxim9606_early_suspend(struct early_suspend *h)
{
	struct maxim9606 *mx = container_of(h, struct maxim9606, suspend);
	maxim9606_disable(mx);
}
#endif

static ssize_t maxim9606_softstart_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *cl = to_i2c_client(dev);
	struct maxim9606 *mx = i2c_get_clientdata(cl);
	int enable = 0;

	if (sscanf(buf, "%u", &enable) < 1) {
		dev_err(dev, "invalid value %s\n", buf);
		return -EINVAL;
	}
	
	mx->softstart = !!enable;
	return size;
}
	
static ssize_t maxim9606_softstart_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *cl = to_i2c_client(dev);
	struct maxim9606 *mx = i2c_get_clientdata(cl);

	return snprintf(buf, PAGE_SIZE, "%u\n", mx->softstart);	
}

static ssize_t maxim9606_show_reg(struct device *dev, u8 reg, char *buf)
{
	struct i2c_client *cl = to_i2c_client(dev);
	s32 value;

	value = i2c_smbus_read_byte_data(cl, reg);

	if (value < 0) {
		dev_warn(dev, "error reading reg %hhu %d\n", reg, value);
		return value;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t maxim9606_DELAY2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return maxim9606_show_reg(dev, 0x04, buf);
}

static ssize_t maxim9606_DELAY3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return maxim9606_show_reg(dev, 0x05, buf);
}

static ssize_t maxim9606_DELAY4_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return maxim9606_show_reg(dev, 0x06, buf);
}

#define DELAY2_VALUE (63)
#define DELAY3_VALUE (15)
#define DELAY4_VALUE (15)

static ssize_t maxim9606_update_mvp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *cl = to_i2c_client(dev);
	u8 to_mvp[3] = { 0x00, 0x01, 0x67 };
	u8 from_mvp[3] = { 0x00, 0x01, 0x76 }; 
	const u8 test_mode[2] = { 0x54, 0x4D };
	int try_count = 0;
	struct i2c_msg mvp;
	s32 DELAY2;
	s32 DELAY3;
	s32 DELAY4;
	int r;	

	DELAY2 = i2c_smbus_read_byte_data(cl, 0x04);	
	DELAY3 = i2c_smbus_read_byte_data(cl, 0x05);
	DELAY4 = i2c_smbus_read_byte_data(cl, 0x06);

	if ((DELAY2 < 0) || (DELAY3 < 0) || (DELAY4 < 0)) {
		dev_err(dev, "error reading DELAYs 2 %d 3 %d 4 %d\n", DELAY2, DELAY3, DELAY4);
		return -EIO;
	}

	if ((DELAY2 == DELAY2_VALUE) && 
		(DELAY3 == DELAY3_VALUE) && 
		(DELAY4 == DELAY4_VALUE)) {
		dev_info(dev, "already programmed\n");
		return size;
	}

	mvp.addr = cl->addr;
	mvp.flags = 0;
	mvp.len = sizeof(to_mvp);
	
retry:
	++try_count;

	if (try_count > 51) {
		dev_err(dev, "failed to program storage after 50 attempts, bailing\n");
		return -EIO;
	}

	r = i2c_smbus_read_byte_data(cl, 0x39);

	if (r) {
		// avdd not up correctly, issue jump start
		dev_warn(dev, "status NOK %d, jump starting", r);
		i2c_smbus_write_block_data(cl, 0xFF, sizeof(test_mode), test_mode);
		i2c_smbus_write_byte_data(cl, 0x5F, 0x02);
		i2c_smbus_write_byte_data(cl, 0x60, 0x14);
		i2c_smbus_write_byte_data(cl, 0x5F, 0x00); 
		i2c_smbus_write_byte_data(cl, 0xFF, 0x00);
		msleep(60);
		goto retry;
	}

	i2c_smbus_write_byte_data(cl, 0x04, DELAY2_VALUE); 
	i2c_smbus_write_byte_data(cl, 0x05, DELAY3_VALUE);
	i2c_smbus_write_byte_data(cl, 0x06, DELAY4_VALUE);

	mvp.addr = cl->addr;
	mvp.flags = 0;
	mvp.len = sizeof(to_mvp);
	mvp.buf = to_mvp;
	
	i2c_transfer(cl->adapter, &mvp, 1);
	return size;
}

static DEVICE_ATTR(softstart, S_IWUSR | S_IWGRP | S_IRUGO, maxim9606_softstart_show, maxim9606_softstart_store);
static DEVICE_ATTR(DELAY2, S_IRUGO, maxim9606_DELAY2_show, NULL);
static DEVICE_ATTR(DELAY3, S_IRUGO, maxim9606_DELAY3_show, NULL);
static DEVICE_ATTR(DELAY4, S_IRUGO, maxim9606_DELAY4_show, NULL);
static DEVICE_ATTR(update_mvp, S_IWGRP | S_IWUSR, NULL, maxim9606_update_mvp);

static struct device_attribute *maxim9606_sysfs_attrs[] = {
	&dev_attr_softstart,
	&dev_attr_DELAY2,
	&dev_attr_DELAY3,
	&dev_attr_DELAY4,
	&dev_attr_update_mvp,
	NULL,
};

static int maxim9606_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct maxim9606_platform_data *pdata = cl->dev.platform_data;
	s32 r = 0;
	int i = 0;
	struct device_attribute *attr;
	struct maxim9606 *mx;
	s32 DELAY2;
	s32 DELAY3;
	s32 DELAY4;
	
	if (pdata->request_resources) {
		pdata->request_resources(&cl->dev);
	}	

	mx = kzalloc(sizeof(struct maxim9606), GFP_KERNEL);

	if (!mx) 
		goto err_mem;

	mx->client = cl;
	mx->pdata = pdata;
	mx->softstart = true;

	i2c_set_clientdata(cl, mx);

	while((attr = maxim9606_sysfs_attrs[i++]) != NULL) {
		r = device_create_file(&cl->dev, attr);

		if (r)
			dev_err(&cl->dev, "failed to create sysfs file\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	// make sure this runs BEFORE the panel
	mx->suspend.level 	= EARLY_SUSPEND_LEVEL_DISABLE_FB + 2;
	mx->suspend.suspend = maxim9606_early_suspend;
	mx->suspend.resume	= maxim9606_late_resume;
	register_early_suspend(&mx->suspend);
#endif

	DELAY2 = i2c_smbus_read_byte_data(mx->client, 0x04);
	DELAY3 = i2c_smbus_read_byte_data(mx->client, 0x05);
	DELAY4 = i2c_smbus_read_byte_data(mx->client, 0x06);

	if ((DELAY2 != DELAY2_VALUE) || 
		(DELAY3 != DELAY3_VALUE) ||
		(DELAY4 != DELAY4_VALUE)) {
		dev_warn(&cl->dev, "DELAYs not programmed correctly, 2 %d 3 %d 4 %d, soft start disabled\n", DELAY2, DELAY3, DELAY4);
		mx->softstart = false;
	} 

	if (i2c_smbus_read_byte_data(mx->client, 0x39)) {
		maxim9606_enable(mx);
	} else if (mx->pdata->power_on) {
		mx->pdata->power_on(&mx->client->dev);
	}

	return 0;	

err_mem:
	return -ENOMEM;
}

static void maxim9606_shutdown(struct i2c_client *cl)
{
	struct maxim9606 *mx = i2c_get_clientdata(cl);
	maxim9606_disable(mx);
}

static int __devexit maxim9606_remove(struct i2c_client *cl)
{
	struct maxim9606 *mx = i2c_get_clientdata(cl);
	
	maxim9606_disable(mx);
	unregister_early_suspend(&mx->suspend);

	if (mx->pdata->release_resources) {
		mx->pdata->release_resources(&cl->dev);
	}	

	kfree(mx);
	return 0;
}

static struct omap_dss_driver novatek_driver = {
	.probe		= novatek_probe,
	.remove		= novatek_remove,

	.enable		= novatek_enable,
	.disable	= novatek_disable,
	.suspend	= novatek_suspend,
	.resume		= NULL,

	.get_resolution	= novatek_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= novatek_get_timings,
	.set_timings	= novatek_set_timings,
	.check_timings	= novatek_check_timings,

	.set_current_fps	= novatek_set_current_fps,
	.get_current_fps	= novatek_get_current_fps,
	.get_fps		= novatek_get_fps,

	.driver         = {
		.name   = "novatek-panel",
		.owner  = THIS_MODULE,
	},
};

static const struct i2c_device_id maxim9606_ids[] = {
	{ "maxim9606", 0x74 },
};

static struct i2c_driver maxim9606_driver = {
	.driver		= {
		.name	= "maxim9606",
	},

	.probe		= maxim9606_probe,
	.remove		= __devexit_p(maxim9606_remove),
	.shutdown	= maxim9606_shutdown,
	.id_table	= maxim9606_ids,
};

static int __init novatek_init(void)
{
	omap_dss_register_driver(&novatek_driver);
	
	if (i2c_add_driver(&maxim9606_driver) < 0) {
		pr_warn("Failed to register maxim9606 driver.\n"); 
	}

	return 0;
}

static void __exit novatek_exit(void)
{
	i2c_del_driver(&maxim9606_driver);
	omap_dss_unregister_driver(&novatek_driver);
}

module_init(novatek_init);
module_exit(novatek_exit);

MODULE_AUTHOR("David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>");
MODULE_DESCRIPTION("Novatek TCON display");
MODULE_LICENSE("GPL");
