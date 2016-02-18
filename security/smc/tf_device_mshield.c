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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pm.h>
#include <linux/sysdev.h>
#include <linux/vmalloc.h>
#include <linux/signal.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#ifdef CONFIG_CMA
#include <linux/dma-contiguous.h>
#endif

#include "tf_protocol.h"
#include "tf_defs.h"
#include "tf_util.h"
#include "tf_conn.h"
#include "tf_comm.h"
#include "tf_zebra.h"

#include "s_version.h"

#define TF_PA_CTRL_START		0x1
#define TF_PA_CTRL_STOP		0x2

static struct class *tf_ctrl_class;

#ifdef CONFIG_CMA
// Dummy device for CMA
static struct device smc_cma_device = { 0, };

static phys_addr_t omap4_smc_cma_addr;
static struct page* omap4_smc_cma_pages;
static size_t omap4_smc_cma_pages_count;
#endif

#define TF_DEVICE_CTRL_BASE_NAME "tf_ctrl"

struct tf_pa_ctrl {
	u32 nPACommand;

	u32 pa_size;
	u8 *pa_buffer;

	u32 conf_size;
	u8 *conf_buffer;
};

#ifdef CONFIG_CMA
static bool omap_smc_allocate_memory(void)
{
        omap4_smc_cma_pages = dma_alloc_from_contiguous_fixed_addr(&smc_cma_device,
						omap4_smc_cma_addr, omap4_smc_cma_pages_count);
	if (!omap4_smc_cma_pages) {
		pr_err("CMA SMC pages allocation failed\n");
		return false;
	}

	return true;
}

static bool omap_smc_free_memory(void)
{
	if (!dma_release_from_contiguous(&smc_cma_device, omap4_smc_cma_pages,
					omap4_smc_cma_pages_count)) {
		pr_err("CMA SMC pages release failed\n");
		return false;
	}

	return true;
}
#endif

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
		dpr_err("%s(%p): ioctl command message pointer is not word "
			"aligned (%p)\n",
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
		struct tf_connection *connection;

		dpr_info("%s(%p): Start the SMC PA (%d bytes) with conf "
			"(%d bytes)\n",
			__func__, file, pa_ctrl.pa_size, pa_ctrl.conf_size);

		connection = tf_conn_from_file(file);

		if (dev->workspace_addr == 0) {
			result = -ENOMEM;
			goto start_exit;
		}

#ifdef CONFIG_CMA
		if (!omap_smc_allocate_memory()) {
			result = -ENOMEM;
			goto start_exit;
		}
#endif

		result = tf_start(&dev->sm,
			dev->workspace_addr,
			dev->workspace_size,
			pa_ctrl.pa_buffer,
			pa_ctrl.pa_size,
			pa_ctrl.conf_buffer,
			pa_ctrl.conf_size);
		if (result) {
#ifdef CONFIG_CMA
			omap_smc_free_memory();
#endif
			dpr_err("SMC: start failed\n");
		}
		else
			dpr_info("SMC: started\n");

start_exit:
		break;
	}

	case TF_PA_CTRL_STOP:
		dpr_info("%s(%p): Stop the SMC PA\n", __func__, file);

		result = tf_power_management(&dev->sm,
			TF_POWER_OPERATION_SHUTDOWN);
		if (result)
			dpr_err("SMC: stop failed [0x%x]\n", result);
		else {
			dpr_info("SMC: stopped\n");
#ifdef CONFIG_CMA
			omap_smc_free_memory();
#endif
		}
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
		dpr_err("%s(%p): "
			"nonseekable_open failed (error %d)!\n",
			__func__, file, error);
		goto error;
	}

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

int __init tf_ctrl_device_register(void)
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

	tf_ctrl_class = class_create(THIS_MODULE, TF_DEVICE_CTRL_BASE_NAME);
	device_create(tf_ctrl_class, NULL,
		dev->dev_number + 1,
		NULL, TF_DEVICE_CTRL_BASE_NAME);

	mutex_init(&dev->dev_mutex);

	return error;
}

static int __initdata smc_mem;
static int __initdata smc_address;

void __init tf_allocate_workspace(void)
{
	struct tf_device *dev = tf_get_device();
#ifdef CONFIG_CMA
	const size_t cma_alignment = PAGE_SIZE << max(MAX_ORDER, pageblock_order);
	phys_addr_t cma_region_addr;
	size_t cma_region_size;
#endif

	tf_clock_timer_init();

	if (tf_ctrl_check_omap_type() <= 0)
		return;

	dev->workspace_size = smc_mem;
	if (dev->workspace_size < 3*SZ_1M)
		dev->workspace_size = 3*SZ_1M;

	if (smc_address == 0)
#if 0
		dev->workspace_addr = (u32) __pa(__alloc_bootmem(
			dev->workspace_size, SZ_1M, __pa(MAX_DMA_ADDRESS)));
#else
		dev->workspace_addr = (u32) 0xBFD00000;
#endif
	else
		dev->workspace_addr = smc_address;

#ifdef CONFIG_CMA
	omap4_smc_cma_addr = dev->workspace_addr;
	omap4_smc_cma_pages_count = dev->workspace_size / PAGE_SIZE;
	cma_region_addr = round_down(omap4_smc_cma_addr, cma_alignment);
	cma_region_size = round_up(dev->workspace_size, cma_alignment);

	pr_info("Reserving CMA SMC region at address = 0x%x with size = 0x%x\n",
		cma_region_addr, cma_region_size);
	dma_declare_contiguous(&smc_cma_device, cma_region_size,
				cma_region_addr, 0);

	pr_info("SMC: will allocate workspace of 0x%x Bytes at (0x%x) once requested\n",
		dev->workspace_size,
		dev->workspace_addr);
#else
	pr_info("SMC: Allocated workspace of 0x%x Bytes at (0x%x)\n",
		dev->workspace_size,
 		dev->workspace_addr);
#endif
}

static int __init tf_mem_setup(char *str)
{
	smc_mem = memparse(str, &str);
	if (*str == '@') {
		str += 1;
		get_option(&str, &smc_address);
	}
	return 0;
}

#ifdef MODULE
int __init tf_device_mshield_init(char *smc_mem)
{
	if (smc_mem != NULL)
		tf_mem_setup(smc_mem);
	tf_allocate_workspace();
	return 0;
}

void __exit tf_device_mshield_exit(void)
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
#else
early_param("smc_mem", tf_mem_setup);
#endif
