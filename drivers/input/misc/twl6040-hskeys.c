/*
 * Headset key driver for TWL6040
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Author: Ivaylo Spasov <ispasov@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl6040-codec.h>
#include <linux/input/twl6040-hskeys.h>
#include "../../../sound/soc/codecs/twl6040.h"

struct keys_data {
	struct twl6040 *twl6040;
	struct twl4030_codec_hskeys_data *pdata;
	struct input_dev *input_dev;
	struct notifier_block nb;
};

static int twl6040_hskeys_update(struct notifier_block *nb, unsigned long event,
				void *_data)
{
	struct keys_data *misc_data;
	struct hskeys_keymap_data *hskeymap_data;
	size_t i;
	int gpadc_scaler;
	int keys_reported = 0;

	if (_data == NULL)
		return 0;

	misc_data = container_of(nb, struct keys_data, nb);
	hskeymap_data = misc_data->pdata->data;

	gpadc_scaler = *(int *)_data;
	for (i = 0; i < hskeymap_data->keymap_size; ++i) {
		if ( gpadc_scaler >= hskeymap_data->keymap[i].min_ref_volt &&
				gpadc_scaler <= hskeymap_data->keymap[i].max_ref_volt) {

			input_report_key(misc_data->input_dev,
				hskeymap_data->keymap[i].key, 1);

			input_report_key(misc_data->input_dev,
				hskeymap_data->keymap[i].key, 0);

			input_sync(misc_data->input_dev);

			++keys_reported;
		}
	}

	if (!keys_reported)
		dev_dbg(misc_data->input_dev->dev.parent, "%s: unrecognized voltage level %d\n",
				__func__, gpadc_scaler);
	else if (keys_reported != 1)
		dev_dbg(misc_data->input_dev->dev.parent, "%s: multple keys found for voltage level %d\n",
				__func__, gpadc_scaler);

	return 0;
}

static int twl6030_hskeys_probe(struct platform_device *pdev)
{
	struct keys_data *data;
	struct twl4030_codec_hskeys_data *pdata = pdev->dev.platform_data;
	struct hskeys_keymap_data *hskeymap_data;
	size_t i;
	int ret = 0;

	if (!pdata) {
		ret = -EBUSY;
		pr_err("%s pdata is null\n", __func__);
		goto err0;
	}

	data = kzalloc(sizeof(struct keys_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		pr_err("%s kzalloc alloc fail\n", __func__);
		goto err0;
	}

	data->pdata = pdata;
	data->twl6040 = dev_get_drvdata(pdev->dev.parent);

	platform_set_drvdata(pdev, data);

	data->input_dev = input_allocate_device();

	if (!data->input_dev) {
		ret = -ENOMEM;
		pr_err("%s Can't allocate input device\n", __func__);
		goto err1;
	}
	hskeymap_data = pdata->data;
	data->input_dev->name = "twl6040_hskeys";
	data->input_dev->dev.parent = &pdev->dev;
	data->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 0; i < hskeymap_data->keymap_size; ++i)
		set_bit(hskeymap_data->keymap[i].key, data->input_dev->keybit);

	ret = input_register_device(data->input_dev);
	if (ret) {
		pr_err("%s Can't register input device\n", __func__);
		goto err2;
	}
	data->nb.notifier_call = twl6040_hskeys_update;
	twl6040_register_hook_notifier(&data->nb);
	return 0;

err2:
	input_free_device(data->input_dev);
err1:
	kfree(data);
err0:
	return ret;
	return 0;
}

static int twl6030_hskeys_remove(struct platform_device *pdev)
{
	struct keys_data *data = platform_get_drvdata(pdev);
	twl6040_unregister_hook_notifier(&data->nb);
	input_free_device(data->input_dev);
	kfree(data);
	return 0;
}

struct platform_driver twl6030_hskeys_driver = {
	.probe	= twl6030_hskeys_probe,
	.remove	= twl6030_hskeys_remove,
	.driver	= {
		.name	= "twl6040-hskeys",
		.owner	= THIS_MODULE,
	},
};

static int __init twl6030_hskeys_init(void)
{
	return platform_driver_register(&twl6030_hskeys_driver);
}
module_init(twl6030_hskeys_init);

static void __exit twl6030_hskeys_exit(void)
{
	platform_driver_unregister(&twl6030_hskeys_driver);
}
module_exit(twl6030_hskeys_exit);

MODULE_AUTHOR("Ivaylo Spasov <ispasov@ti.com>");
MODULE_DESCRIPTION("TWL6030 Keys Driver");
MODULE_LICENSE("GPL");
