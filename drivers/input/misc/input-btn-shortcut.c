/* drivers/input/btn_shortcut.c
 *
 * Copyright (C) 2014 MM Solutions AD.
 * Author: Angel Shtilianov <ashtilianov@mm-sol.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <asm/system_misc.h>

#define BTN_SHORTCUT_NAME "btn_shortcut"

static int key_pwr;
static int key_vdn;
static int cnt_vdn;

extern int omap_sar_reason(char *reason);

static void btn_shortcut_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	struct device *dev = (handle && handle->dev) ?
					&handle->dev->dev : NULL;

	if (type != EV_KEY)
		return;

	dev_dbg(dev, "%s: Got event: Type:%d Code:%d Value:%d\n",
		__func__, type, code, value);

	if (code == KEY_POWER) {
		if (value != key_pwr) {
			cnt_vdn = 0;
			key_pwr = value;
		}
	} else if (code == KEY_VOLUMEDOWN) {
		if (value != key_vdn) {
			key_vdn = value;
			if (key_vdn && key_pwr && ++cnt_vdn == 5) {
				pr_err("---PWR + 5*Volume Down!---\n");
				omap_sar_reason("recovery");
				emergency_restart();
			}
		}
	}

	dev_dbg(dev, "%s: cnt_vdn=%d", __func__, cnt_vdn);
}

static int btn_shortcut_connect(struct input_handler *handler,
	struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = BTN_SHORTCUT_NAME;
	handle->private = handler->private;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	dev_dbg(&dev->dev, "using input dev %s for btn_shortcut\n", dev->name);

	return 0;

err_input_open_device:
	input_unregister_handle(handle);

err_input_register_handle:
	kfree(handle);

	return ret;
}

static void btn_shortcut_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id btn_shortcut_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, btn_shortcut_ids);

static struct input_handler shortcut_state = {
	.name = BTN_SHORTCUT_NAME,
	.event = btn_shortcut_event,
	.connect = btn_shortcut_connect,
	.disconnect = btn_shortcut_disconnect,
	.id_table = btn_shortcut_ids,
};

static int __init btn_shortcut_init(void)
{
	return input_register_handler(&shortcut_state);
}

static void __exit btn_shortcut_exit(void)
{
	input_unregister_handler(&shortcut_state);
}

module_init(btn_shortcut_init);
module_exit(btn_shortcut_exit);
