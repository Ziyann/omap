/**
 * Copyright (c) 2011 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pm.h>
#include <linux/vmalloc.h>
#include <linux/signal.h>
#ifdef CONFIG_ANDROID
#include <linux/device.h>
#endif
#include <linux/init.h>
#include <linux/bootmem.h>
#include <clockdomain.h>

#include "tf_protocol.h"
#include "tf_defs.h"
#include "tf_util.h"
#include "tf_conn.h"
#include "tf_comm.h"
#include "tf_zebra.h"

#include "s_version.h"

#define TF_PA_CTRL_START	0x1
#define TF_PA_CTRL_STOP		0x2

#ifdef CONFIG_ANDROID
static struct class *tf_ctrl_class;
#endif

#define TF_DEVICE_CTRL_BASE_NAME "tf_ctrl"

struct tf_pa_ctrl {
	u32 nPACommand;

	u32 pa_size;
	u8 *pa_buffer;

	u32 conf_size;
	u8 *conf_buffer;
};

static int tf_ctrl_check_omap_type(void)
{
	/* No need to do anything on a GP device */
	switch (omap_type()) {
	case OMAP2_DEVICE_TYPE_GP:
		dpr_info("SMC: Running on a GP device\n");
		return 0;

	case OMAP2_DEVICE_TYPE_EMU:
	case OMAP2_DEVICE_TYPE_SEC:
	/*case OMAP2_DEVICE_TYPE_TEST:*/
		dpr_info("SMC: Running on a EMU or HS device\n");
		return 1;

	default:
		pr_err("SMC: unknown omap type %x\n", omap_type());
		return -EFAULT;
	}
}

/*----------------------------------------------------------------------------*/

static int tf_ctrl_device_release(struct inode *inode, struct file *file)
{
	struct tf_connection *connection;

	dpr_info("%s(%u:%u, %p)\n",
		__func__, imajor(inode), iminor(inode), file);

	connection = tf_conn_from_file(file);
	tf_close(connection);

	dpr_info("%s(%p): Success\n", __func__, file);
	return 0;
}

/*----------------------------------------------------------------------------*/

#define IOCTL_TF_PA_CTRL _IOWR('z', 0xFF, struct tf_pa_ctrl)

static long tf_ctrl_device_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	int result = S_SUCCESS;
	struct tf_pa_ctrl pa_ctrl;
	struct tf_device *dev = tf_get_device();

	dpr_info("%s(%p, %u, %p)\n",
		__func__, file, ioctl_num, (void *) ioctl_param);

	mutex_lock(&dev->dev_mutex);

	if (ioctl_num != IOCTL_TF_PA_CTRL) {
		dpr_err("%s(%p): ioctl number is invalid (%p)\n",
			__func__, file, (void *)ioctl_num);

		result = -EFAULT;
		goto exit;
	}

	if ((ioctl_param & 0x3) != 0) {
		dpr_err("%s(%p): pointer is not word aligned (%p)\n",
			__func__, file, (void *)ioctl_param);

		result = -EFAULT;
		goto exit;
	}

	if (copy_from_user(&pa_ctrl, (struct tf_pa_ctrl *)ioctl_param,
			sizeof(struct tf_pa_ctrl))) {
		dpr_err("%s(%p): cannot access ioctl parameter (%p)\n",
			__func__, file, (void *)ioctl_param);

		result = -EFAULT;
		goto exit;
	}

	switch (pa_ctrl.nPACommand) {
	case TF_PA_CTRL_START: {
		struct tf_shmem_desc *shmem_desc = NULL;
		u32 shared_mem_descriptors[TF_MAX_COARSE_PAGES] = {0};
		u32 descriptor_count;
		u32 offset;
		struct tf_connection *connection;

		dpr_info("%s(%p): Start the SMC PA (%d bytes, conf %d bytes)\n",
			__func__, file, pa_ctrl.pa_size, pa_ctrl.conf_size);

		connection = tf_conn_from_file(file);

		if (dev->workspace_addr == 0) {
			result = -ENOMEM;
			goto start_exit;
		}

#ifdef SUPPORT_LARGE_PHYSICAL_ADDRESS_EXTENSION
		if (!cpu_is_omap54xx())
#endif
		{
			result = tf_validate_shmem_and_flags(
					(u32)pa_ctrl.conf_buffer,
					pa_ctrl.conf_size,
					TF_SHMEM_TYPE_READ);
			if (result != 0)
				goto start_exit;

			offset = 0;
			result = tf_map_shmem(
					connection,
					(u32)pa_ctrl.conf_buffer,
					TF_SHMEM_TYPE_READ,
					true, /* in user space */
					shared_mem_descriptors,
					&offset,
					pa_ctrl.conf_size,
					&shmem_desc,
					&descriptor_count);
			if (result != 0)
				goto start_exit;

			if (descriptor_count > 1) {
				dpr_err("%s(%p): configuration file is too long (%d)\n",
					__func__, file, descriptor_count);
				result = -ENOMEM;
				goto start_exit;
			}
		}

		result = tf_start(&dev->sm,
			dev->workspace_addr,
			dev->workspace_size,
			pa_ctrl.pa_buffer,
			pa_ctrl.pa_size,
			shared_mem_descriptors[0],
			offset,
			pa_ctrl.conf_size);
		if (result)
			dpr_err("SMC: start failed\n");
		else
			dpr_info("SMC: started\n");

start_exit:
		tf_unmap_shmem(connection, shmem_desc, true); /* full cleanup */
		break;
	}

	case TF_PA_CTRL_STOP:
		dpr_info("%s(%p): Stop the SMC PA\n", __func__, file);

		result = tf_power_management(&dev->sm,
			TF_POWER_OPERATION_SHUTDOWN);
		if (result)
			dpr_err("SMC: stop failed [0x%x]\n", result);
		else
			dpr_info("SMC: stopped\n");
		break;

	default:
		result = -EOPNOTSUPP;
		break;
	}

exit:
	mutex_unlock(&dev->dev_mutex);
	return result;
}

/*----------------------------------------------------------------------------*/

static int tf_ctrl_device_open(struct inode *inode, struct file *file)
{
	int error;
	struct tf_connection *connection = NULL;

	dpr_info("%s(%u:%u, %p)\n",
		__func__, imajor(inode), iminor(inode), file);

	/* Dummy lseek for non-seekable driver */
	error = nonseekable_open(inode, file);
	if (error != 0) {
		dpr_err("%s(%p): nonseekable_open failed (error %d)!\n",
			__func__, file, error);
		goto error;
	}

#ifndef CONFIG_ANDROID
	/*
	 * Check file flags. We only autthorize the O_RDWR access
	 */
	if (file->f_flags != O_RDWR) {
		dpr_err("%s(%p): invalid access mode %u\n",
			__func__, file, file->f_flags);
		error = -EACCES;
		goto error;
	}
#endif

	error = tf_ctrl_check_omap_type();
	if (error <= 0)
		return error;

	error = tf_open(tf_get_device(), file, &connection);
	if (error != 0) {
		dpr_err("%s(%p): tf_open failed (error %d)!\n",
			__func__, file, error);
		goto error;
	}

	file->private_data = connection;

	/*
	 * Successful completion.
	 */
	dpr_info("%s(%p): Success\n", __func__, file);
	return 0;

	/*
	 * Error handling.
	 */
error:
	tf_close(connection);
	dpr_info("%s(%p): Failure (error %d)\n",
		__func__, file, error);
	return error;
}

static const struct file_operations g_tf_ctrl_device_file_ops = {
	.owner = THIS_MODULE,
	.open = tf_ctrl_device_open,
	.release = tf_ctrl_device_release,
	.unlocked_ioctl = tf_ctrl_device_ioctl,
	.llseek = no_llseek,
};

int __devinit tf_ctrl_device_register(void)
{
	int error;
	struct tf_device *dev = tf_get_device();

	cdev_init(&dev->cdev_ctrl, &g_tf_ctrl_device_file_ops);
	dev->cdev_ctrl.owner = THIS_MODULE;

	error = register_chrdev_region(dev->dev_number + 1, 1,
		TF_DEVICE_CTRL_BASE_NAME);
	if (error)
		return error;

	error = cdev_add(&dev->cdev_ctrl,
		dev->dev_number + 1, 1);
	if (error) {
		cdev_del(&(dev->cdev_ctrl));
		unregister_chrdev_region(dev->dev_number + 1, 1);
		return error;
	}

#ifdef CONFIG_ANDROID
	tf_ctrl_class = class_create(THIS_MODULE, TF_DEVICE_CTRL_BASE_NAME);
	device_create(tf_ctrl_class, NULL,
		dev->dev_number + 1,
		NULL, TF_DEVICE_CTRL_BASE_NAME);
#endif

	mutex_init(&dev->dev_mutex);

	return error;
}

static void __devinit tf_allocate_workspace(void)
{
	struct tf_device *dev = tf_get_device();

	if (tf_early_init()) {
		printk(KERN_ERR "SMC early initialization failed\n");
		return;
	}

	tf_clock_timer_init();

	if (tf_ctrl_check_omap_type() <= 0)
		return;

	dev->workspace_size = pdata->secure_workspace_size;
	if (dev->workspace_size < 3*SZ_1M)
		dev->workspace_size = 3*SZ_1M;
	dev->workspace_addr = pdata->secure_workspace_addr;

	pr_info("SMC: Using workspace of 0x%x Bytes at 0x%x\n",
		dev->workspace_size,
		dev->workspace_addr);
	printk(KERN_INFO "SMC workspace 0x%x 0x%x\n",
		dev->workspace_size,
		dev->workspace_addr);

}

int __devinit tf_device_mshield_init(void)
{
	tf_allocate_workspace();
	return 0;
}

void tf_device_mshield_exit(void)
{
	struct tf_device *dev = tf_get_device();
	if (dev == NULL)
		return;

	if (tf_ctrl_class != NULL) {
		device_destroy(tf_ctrl_class, dev->dev_number + 1);
		class_destroy(tf_ctrl_class);
		tf_ctrl_class = NULL;
	}
	cdev_del(&(dev->cdev_ctrl));
	unregister_chrdev_region(dev->dev_number + 1, 1);

	dev->workspace_size = 0;
	dev->workspace_addr = 0;
}
