/*
 * Remote processor resource manager common resources
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Miguel Vadillo <vadillo@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/rpmsg_resmgr.h>
#include <linux/remoteproc.h>
#include "rpmsg_resmgr_common.h"

struct rprm_i2c_depot {
	u32 id;
	struct device *dev;
};

struct rprm_rproc_depot {
	char name[16];
	struct rproc *rp;
};

static int rprm_gpio_request(void **handle, void *args, size_t len)
{
	int ret;
	struct rprm_gpio *gpio = args;

	if (sizeof *gpio != len)
		return -EINVAL;

	ret = gpio_request(gpio->id , "rpmsg_resmgr");
	if (ret) {
		pr_err("error providing gpio %d\n", gpio->id);
		return ret;
	}

	*handle = (void *)gpio->id;

	return 0;
}

static int rprm_gpio_release(void *handle)
{
	u32 id = (unsigned)handle;

	gpio_free(id);

	return 0;
}

static int rprm_gpio_get_info(void *handle, char *buf, size_t len)
{
	u32 id = (unsigned)handle;

	return snprintf(buf, len, "Id:%d\n", id);
}

static int rprm_i2c_request(void **handle, void *data, size_t len)
{
	struct rprm_i2c *i2c = data;
	struct i2c_adapter *adapter;
	struct rprm_i2c_depot *i2cd;
	int ret;

	if (len != sizeof *i2c)
		return -EINVAL;

	i2cd = kmalloc(sizeof *i2cd, GFP_KERNEL);
	if (!i2cd)
		return -ENOMEM;

	adapter = i2c_get_adapter(i2c->id);
	if (!adapter) {
		pr_err("could not get i2c%d adapter\n", i2c->id);
		ret = -EINVAL;
		goto err;
	}

	i2cd->dev = adapter->dev.parent;
	i2c_put_adapter(adapter);

	/* FIXME: don't use pm runtime framework */
	ret = pm_runtime_get_sync(i2cd->dev);
	/*
	 * pm_runtime_get_sync can return 1 in case it is already active,
	 * change it to 0 to indicate success.
	 */
	ret -= ret == 1;
	if (ret)
		goto err;

	i2cd->id = i2c->id;
	*handle = i2cd;

	return 0;
err:
	kfree(i2cd);
	return ret;
}

static int rprm_i2c_release(void *handle)
{
	struct rprm_i2c_depot *i2cd = handle;
	int ret;

	ret = pm_runtime_put_sync(i2cd->dev);
	if (ret) {
		pr_err("failed put sync %d\n", ret);
		return ret;
	}

	kfree(i2cd);
	return 0;
}

static int rprm_i2c_get_info(void *handle, char *buf, size_t len)
{
	struct rprm_i2c_depot *i2cd = handle;

	return snprintf(buf, len, "id:%d\n", i2cd->id);
}

static int rprm_rproc_request(void **handle, void *data, size_t len)
{
	struct rprm_rproc *rproc_data = data;
	struct rprm_rproc_depot *rprocd;
	int ret;

	if (len != sizeof *rproc_data)
		return -EINVAL;

	rprocd = kmalloc(sizeof *rprocd, GFP_KERNEL);
	if (!rprocd)
		return -ENOMEM;

	rprocd->rp = rproc_get_by_name(rproc_data->name);
	if (!rprocd->rp) {
		ret = -ENODEV;
		goto error;
	}

	strcpy(rprocd->name, rproc_data->name);
	*handle = rprocd;

	return 0;
error:
	kfree(rprocd);
	return ret;
}

static int rprm_rproc_release(void *handle)
{
	struct rprm_rproc_depot *rprocd = handle;

	rproc_put(rprocd->rp);
	kfree(rprocd);

	return 0;
}

static int rprm_rproc_get_info(void *handle, char *buf, size_t len)
{
	struct rprm_rproc_depot *rprocd = handle;

	return snprintf(buf, len, "Name:%s\n", rprocd->name);
}

static int _rproc_latency(struct device *rdev, void *handle, unsigned long val)
{
	struct rprm_rproc_depot *rprocd = handle;

	return rproc_set_constraints(rdev, rprocd->rp,
				     RPROC_CONSTRAINT_LATENCY, val);
}

static int _rproc_bandwidth(struct device *rdev, void *handle,
							unsigned long val)
{
	struct rprm_rproc_depot *rprocd = handle;

	return rproc_set_constraints(rdev, rprocd->rp,
				     RPROC_CONSTRAINT_BANDWIDTH, val);
}

static int _rproc_scale(struct device *rdev, void *handle, unsigned long val)
{
	struct rprm_rproc_depot *rprocd = handle;

	return rproc_set_constraints(rdev, rprocd->rp,
				     RPROC_CONSTRAINT_FREQUENCY, val);
}

static struct rprm_res_ops gpio_ops = {
	.request = rprm_gpio_request,
	.release = rprm_gpio_release,
	.get_info = rprm_gpio_get_info,
};

static struct rprm_res_ops i2c_ops = {
	.request = rprm_i2c_request,
	.release = rprm_i2c_release,
	.get_info = rprm_i2c_get_info,
};

static struct rprm_res_ops rproc_ops = {
	.request = rprm_rproc_request,
	.release = rprm_rproc_release,
	.get_info = rprm_rproc_get_info,
	.latency = _rproc_latency,
	.bandwidth = _rproc_bandwidth,
	.scale = _rproc_scale,
};

static struct rprm_res generic_res[] = {
	{
		.name = "gpio",
		.ops = &gpio_ops,
	},
	{
		.name = "i2c",
		.ops = &i2c_ops,
	},
	{
		.name = "rproc",
		.ops = &rproc_ops,
	},
};

static int __init rprm_resources_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(generic_res); i++) {
		generic_res[i].owner = THIS_MODULE;
		rprm_resource_register(&generic_res[i]);
	}

	return 0;
}

static void __exit rprm_resources_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(generic_res); i++)
		rprm_resource_unregister(&generic_res[i]);
}
module_init(rprm_resources_init);
module_exit(rprm_resources_exit);

MODULE_DESCRIPTION("Remote Processor Resource Manager common resources");
MODULE_LICENSE("GPL v2");
