/*
 * Remote Processor Framework
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * Shahid Akhtar <sakhtar@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/types.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/elf.h>
#include <linux/remoteproc.h>
#include <linux/rproc_drm.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/metricslog.h>

#include "remoteproc_internal.h"

#ifdef CONFIG_ION_OMAP
#include <linux/ion.h>
#include <linux/omap_ion.h>
#endif

#define RPROC_RELOAD_TIMEOUT_MS	5000

/**
 * enum rproc_secure_state - remote processor secure states
 * @RPROC_SECURE_OFF:		unsecure state
 * @RPROC_SECURE_RELOAD:	reloading before enteriing secure mode
 * @RPROC_SECURE_AUTHENTICATED:	code authenticated & firewalled
 * @RPROC_SECURE_ON:		secure mode
 */
enum rproc_secure_state {
	RPROC_SECURE_OFF		= 0,
	RPROC_SECURE_RELOAD		= 1,
	RPROC_SECURE_AUTHENTICATED	= 2,
	RPROC_SECURE_ON			= 3,
};

static DECLARE_COMPLETION(secure_reload_complete);
static DEFINE_MUTEX(secure_lock);
static enum rproc_secure_state secure_state;
static enum rproc_secure_state prev_secure_state;
static int secure_request;
static struct rproc_sec_params *secure_params;
static rproc_drm_invoke_service_t rproc_secure_drm_function;
static int rproc_secure_drm_service(
	enum rproc_service_enum service,
	struct rproc_sec_params *rproc_sec_params);

#define dev_to_rproc(dev) container_of(dev, struct rproc, dev)


/**
 * rproc_secure_init() - initialize rproc_secure module
 * @rproc: remote processor
 *
 * Initializes all the state variables and objects required
 * by the rproc_secure module. This is needed so that rprocs
 * can be registered & unregistered (when used as modules)
 * without rebooting.
 */
void rproc_secure_init(struct rproc *rproc)
{
	dev_dbg(&rproc->dev, "init secure service\n");

	if (strcmp(rproc->name, "ipu_c0"))
		return;

	secure_request = 0;
	secure_params = NULL;
	secure_state = RPROC_SECURE_OFF;
}

static int rproc_exit_secure_playback(struct rproc *rproc)
{
	const int max_retries = 20;
	int try = 0;
	int ret;

	/*
	 * Retry a few times in case we accidentally stumble upon some
	 * userspace security session.
	 */
	do {
		/* invoke service to exit secure mode */
#if defined(CONFIG_DRM_WIDEVINE)
		ret = rproc_secure_drm_service(EXIT_SECURE_PLAYBACK,
								secure_params);
#else
		ret = rproc_secure_drm_service(RPROC_EXIT_SECURE_PLAYBACK,
								secure_params);
#endif
		if (ret == -EBUSY) {
			dev_warn(&rproc->dev,
				"EBUSY disabling secure mode, try %d/%d\n",
				try + 1, max_retries);
			msleep(20);
		}
	} while (ret == -EBUSY && ++try < max_retries);

	return ret;
}


/**
 * rproc_secure_reset() - reset secure service and firewalls
 * @rproc: remote processor
 *
 * Reset the state of the DRM service
 */
void rproc_secure_reset(struct rproc *rproc)
{
	const bool rproc_has_crashed = (rproc->state == RPROC_CRASHED);
	int ret;

	dev_dbg(&rproc->dev, "reseting secure service\n");

	if (strcmp(rproc->name, "ipu_c0"))
		return;

	/*
	 * Reset firewalls when we're either:
	 *   - exiting from secure playback mode
	 *   - recovering from rproc crash
	 * When recovering we practically reboot the remote processor, so
	 * consequently there should be no firewalls while kernel
	 * reloads the firmware.
	 */
	if ((!secure_request && secure_state) || rproc_has_crashed) {
		/* invoke service to exit secure mode */
		ret = rproc_exit_secure_playback(rproc);
		if (ret)
			dev_err(&rproc->dev,
				"error disabling secure mode 0x%x\n", ret);
	}

#if defined(CONFIG_DRM_WIDEVINE)
	kfree(secure_params);
	secure_params = NULL;
	rproc_secure_drm_service(AUTHENTICATION_A0, NULL);
#endif
}

/**
 * rproc_is_secure() - check if the rproc is secure
 * @rproc: remote processor
 *
 * This function is called by the remoteproc core driver code to
 * check if the processor is going through a secure->non-secure
 * transition.
 */
bool rproc_is_secure(struct rproc *rproc)
{
	if (strcmp(rproc->name, "ipu_c0"))
		return 0;

	return secure_state ? 1 : 0;
}

/**
 * rproc_secure_get_mode() - get the current requested secure mode
 * @rproc: remote processor
 *
 * This function is called by the remoteproc core driver code to
 * retrieve the current requested mode.
 */
int rproc_secure_get_mode(struct rproc *rproc)
{
	if (strcmp(rproc->name, "ipu_c0"))
		return 0;

	return secure_request;
}

/**
 * rproc_secure_get_ttb() - get the ttbr value
 * @rproc: remote processor
 *
 * This function is called by the remoteproc core driver code to get
 * the iommu ttbr address to program the iommu for secure mode. The
 * ttbr address is obtained during the parsing of the firmware data
 * image, and serves no purpose in regular mode.
 */
int rproc_secure_get_ttb(struct rproc *rproc)
{
	if (strcmp(rproc->name, "ipu_c0"))
		return 0;

	if (!secure_params)
		return 0;

	return secure_params->ducati_page_table_address;
}

/**
 * rproc_secure_parse_fw() - Parse the fw for secure sections
 * @rproc: rproc handle
 *
 * This function parses the fw image to locate sections and location
 * of memory that is to be firewalled for secure playback. In addition,
 * it looks at all the carveout memory entries to update a list of
 * parameters that are passed onto the secure service. The parameters
 * also include the location of the secure input and secure_output
 * buffers.
 *
 * This function should be called after the resource table has been
 * processed and before configuring the iommu. There are mainly 4
 * secure sections expected in a firmware image. All the sections are
 * needed for booting the processor in secure mode.
 *
 * The .certificate section contains certificate used for authentication.
 * The .iommu_ttbr section contains the L1 table for the MMU. The
 * .secure_params section contains the struct tf_rproc_sec_params that
 * contains all data passed to secure service. The .iommu_ttbr_updates
 * section contains details of carveout sections that are dynamically
 * loaded and the TTBR entries for these sections are updated here.
 *
 * Returns 0 on success or if image is unsigned
 */
int rproc_secure_parse_fw(struct rproc *rproc, const u8 *elf_data)
{
	int ret = 0;
	void *ptr;
	u32  ttbr_da = 0, toc_da = 0, *ttbr = NULL, i;
	struct device *dev = &rproc->dev;
	struct rproc_iommu_ttbr_update *ttbru = NULL;
	struct rproc_sec_params *secure_params_local = NULL;
	struct rproc_mem_entry *maps = NULL;
	struct elf32_hdr *ehdr = (struct elf32_hdr *)elf_data;
	struct elf32_shdr *shdr = (struct elf32_shdr *)
					(elf_data + ehdr->e_shoff);
	const char *name_sect = elf_data + shdr[ehdr->e_shstrndx].sh_offset;
#ifdef CONFIG_ION_OMAP
	struct ion_platform_heap *tiler_2d_heap;
#endif

	if (strcmp(rproc->name, "ipu_c0"))
		return 0;

	/* parse fw image for required data sections */
	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		u32 filesz = shdr->sh_size;
		u32 da = shdr->sh_addr;
		const char *name = name_sect + shdr->sh_name;

		if (!strcmp(name, ".certificate")) {
			ptr = rproc_da_to_va(rproc, da, filesz);
			toc_da = da;
		} else if (!strcmp(name, ".iommu_ttbr")) {
			ptr = rproc_da_to_va(rproc, da, filesz);
			ttbr = (u32 *) ptr;
			ttbr_da = da;
		} else if (!strcmp(name, ".secure_params")) {
			ptr = rproc_da_to_va(rproc, da, filesz);
			secure_params_local = (struct rproc_sec_params *) ptr;
		} else if (!strcmp(name, ".iommu_ttbr_updates")) {
			ptr = rproc_da_to_va(rproc, da, filesz);
			ttbru = (struct rproc_iommu_ttbr_update *) ptr;
		} else {
			continue;
		}

		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da,
								filesz);
			ret = -EINVAL;
			goto out;
		}
	}

	/* assume image is unsigned if required sections are missing */
	if (!toc_da || !ttbr || !secure_params_local || !ttbru) {
		secure_params = NULL;
		dev_dbg(dev, "ipu firmware image is not signed\n");
		goto out;
	}

	/* update params for passing onto secure service */
	list_for_each_entry(maps, &rproc->carveouts, node) {
		if (maps->memregion == RPROC_MEMREGION_CODE) {
			secure_params_local->ducati_code = maps->dma;
			secure_params_local->ducati_code_size = maps->len;

			secure_params_local->ducati_toc_address =
				maps->dma - maps->da + toc_da;
			secure_params_local->ducati_page_table_address =
				maps->dma - maps->da + ttbr_da;
		}
		if (maps->memregion == RPROC_MEMREGION_DATA) {
			secure_params_local->ducati_data = maps->dma;
			secure_params_local->ducati_data_size = maps->len;
		}
		if (maps->memregion == RPROC_MEMREGION_SMEM) {
			secure_params_local->ducati_base_address = maps->dma;
			secure_params_local->ducati_smem = maps->dma;
			secure_params_local->ducati_smem_size = maps->len;
		}

		/* update TTBR entries for this carveout region */
		for (i = 0; i < ttbru->count; i++)
			ttbr[ttbru->index + i] += maps->dma;
		ttbru++;
	}

	/* find location of vring and iobufs */
	list_for_each_entry(maps, &rproc->mappings, node) {
		if (maps->memregion == RPROC_MEMREGION_VRING) {
			secure_params_local->ducati_vring_smem = maps->dma;
			secure_params_local->ducati_vring_size = maps->len;
		}
		if (maps->memregion == RPROC_MEMREGION_1D) {
			secure_params_local->secure_buffer_address = maps->dma;
			secure_params_local->secure_buffer_size = maps->len;
		}
	}

#ifdef CONFIG_ION_OMAP
	tiler_2d_heap = omap_ion_get2d_heap();
	if (!tiler_2d_heap)
		BUG();
	secure_params_local->decoded_buffer_address = (dma_addr_t) tiler_2d_heap->base;
	secure_params_local->decoded_buffer_size = (uint32_t) tiler_2d_heap->size;
	dev_dbg(dev, "decode buff addr = %x, decode buff size = %d\n",
			secure_params_local->decoded_buffer_address, secure_params_local->decoded_buffer_size);

	/* copy the local secure params into a direct-mapped kernel memory */
	secure_params = kzalloc(sizeof(*secure_params), GFP_KERNEL);
	if (secure_params)
		*secure_params = *secure_params_local;
	else
		ret = -ENOMEM;
#endif

out:
	return ret;
}

void secure_log_metrics(char *msg)
{
	struct timespec ts = current_kernel_time();
	char buf[512];
	snprintf(buf, sizeof(buf),
		"ducati:def:%s=1;CT;1:HI,timestamp=%lu;TI;1:NR",
		msg,
		ts.tv_sec * 1000 + ts.tv_nsec / NSEC_PER_MSEC);
	log_to_metrics(ANDROID_LOG_INFO, "ducati_metrics", buf);
}

/**
 * rproc_secure_boot() - Secure ducati code during boot
 * @rproc: rproc handle
 *
 * This function invoke the different secure services to authenticate
 * the ipu code and data sections (blocking calls). A bunch of secure
 * parameters are passed to allow the secure side to configure itself
 * and authenticate the image properly.
 *
 * This function should be called after all code has been loaded to
 * memory, and just before releasing the processor reset.
 *
 * Returns 0 on success, or an appropriate error code otherwise
 */
int rproc_secure_boot(struct rproc *rproc)
{
	int ret = 0;
	struct device *dev = &rproc->dev;

	if (strcmp(rproc->name, "ipu_c0"))
		return 0;

	prev_secure_state = secure_state;

	/* enter secure authentication process */
	if (secure_request) {
#if defined(CONFIG_DRM_WIDEVINE)
		/* TODO: consolidate the back to back authentication calls */
		/* validate boot section */
		ret = rproc_secure_drm_service(AUTHENTICATION_A1,
							secure_params);
		if (ret) {
			dev_err(dev, "failed to validate boot code 0x%x\n",
									ret);
			goto out;
		}

		/* validate all code */
		ret = rproc_secure_drm_service(AUTHENTICATION_A2,
							secure_params);
		if (ret) {
			dev_err(dev, "failed to authenticate code 0x%x\n", ret);
			goto out;
		}

		/* invoke secure service for entering secure mode */
		ret = rproc_secure_drm_service(
				ENTER_SECURE_PLAYBACK_AFTER_AUTHENTICATION,
				secure_params);
#else
		ret = rproc_secure_drm_service(
				RPROC_ENTER_SECURE_PLAYBACK,
				secure_params);
#endif
		if (ret)
			dev_err(dev, "failed to install firewalls 0x%x\n", ret);
		else {
			secure_state = RPROC_SECURE_ON;
			secure_log_metrics("loaded_in_secure_mode");
		}
	} else {
		secure_state = RPROC_SECURE_OFF;
	}

#if defined(CONFIG_DRM_WIDEVINE)
out:
#endif
	return ret;
}

/**
 * rproc_set_secure() - Enable/Disable secure mode
 * @name: name of remote core
 * @enable: 1 to enable and 0 to disable secure mode
 *
 * Called by user to enable secure mode for ducati code. If the
 * ducati image is not authenticated, the image will be reloaded
 * and authenticated before firewalls are enabled. The reload is
 * done only once.
 *
 * Returns 0 on success
 */
int rproc_set_secure(const char *name, bool enable)
{
	unsigned long timeout = msecs_to_jiffies(RPROC_RELOAD_TIMEOUT_MS);
	int ret = 0;

	if (strcmp(name, "ipu_c0"))
		return -EINVAL;

	if (!secure_params)
		return -ENODEV;

	mutex_lock(&secure_lock);

	/* trigger a reload in secure mode */
	secure_state = RPROC_SECURE_RELOAD;
	secure_request = enable;
	init_completion(&secure_reload_complete);
	ret = rproc_reload(name);
	if (ret)
		goto out;
	if (wait_for_completion_timeout(&secure_reload_complete, timeout) == 0)
		pr_err("Timeout waiting for rproc to boot.\n");

	if (enable && secure_state != RPROC_SECURE_ON)
		ret = -EACCES;
	else if (!enable && secure_state != RPROC_SECURE_OFF)
		ret = -EACCES;

	if (!ret)
		pr_info("Ducati successfully %s secure mode.\n",
				enable ? "entered" : "exited");
	else
		pr_err("Ducati FAILED to %s secure mode!\n",
				enable ? "enter" : "exit");
out:
	mutex_unlock(&secure_lock);
	return ret;
}
EXPORT_SYMBOL(rproc_set_secure);

void rproc_secure_complete(struct rproc *rproc)
{
	if (strcmp(rproc->name, "ipu_c0"))
		return;

	if (prev_secure_state == RPROC_SECURE_RELOAD)
		complete_all(&secure_reload_complete);
}

void core_log_metrics(char *msg)
{
	struct timespec ts = current_kernel_time();
	char buf[512];
	snprintf(buf, sizeof(buf),
		"ducati:def:%s=1;CT;1:HI,timestamp=%lu;TI;1:NR",
		msg,
		ts.tv_sec * 1000 + ts.tv_nsec / NSEC_PER_MSEC);
	log_to_metrics(ANDROID_LOG_INFO, "ducati_metrics", buf);
}

/**
 * Resembles rproc_set_secure, but secure state machine handling is
 * sufficiently different during recovery to deem a separate function.
 */
int rproc_secure_recover(struct rproc *rproc)
{
	enum rproc_secure_state crashed_secure_state;
	unsigned long timeout = msecs_to_jiffies(RPROC_RELOAD_TIMEOUT_MS);
	int ret = 0;

	if (strcmp(rproc->name, "ipu_c0")) {
		rproc_recover(rproc);
		return 0;
	}

	dev_err(&rproc->dev, "trying to recover %s\n", rproc->name);
	core_log_metrics("recovering_from_crash");

	if (!secure_params) {
		/*
		 * There is no secure playback integrated, so take
		 * a shortcut
		 */
		return rproc_reload(rproc->name);
	}

	mutex_lock(&secure_lock);

	crashed_secure_state = secure_state;

	/*
	 * We don't touch secure_request because we want to trigger a
	 * reload in whatever mode we have been during the crash.
	 */
	secure_state = RPROC_SECURE_RELOAD;
	init_completion(&secure_reload_complete);
	ret = rproc_reload(rproc->name);

	if (wait_for_completion_timeout(&secure_reload_complete, timeout) == 0)
		pr_err("Timeout waiting for rproc to recover.\n");

	if (crashed_secure_state != secure_state)
		ret = -EACCES;

	pr_info("Ducati %s in %s mode.\n",
			ret ? "FAILED to recover" : "successfully recovered",
			secure_state == RPROC_SECURE_ON ?
					"secure" : "non-secure");

	mutex_unlock(&secure_lock);

	return ret;
}

/**
 * rproc_secure_drm_service() - invoke the specified rproc_drm service
 * @service: the secure service that is being invoked
 * @rproc_sec_params: secure params needed by the service
 *
 * returns 0 on success
 */
static int rproc_secure_drm_service(enum rproc_service_enum service,
				    struct rproc_sec_params *rproc_sec_params)
{
	if (!rproc_secure_drm_function)
		return -ENOSYS;

	return rproc_secure_drm_function(service, rproc_sec_params);
}

/**
 * rproc_register_drm_service() - called by the rproc_drm module
 *				  to register the service.
 * @drm_service: the rproc_drm service that is being registered
 *
 * remoteproc calls this rproc_drm.rproc_drm_invoke_service upon calls
 *  to rproc_drm_invoke_service
 *
 * returns 0 on success
 */
int rproc_register_drm_service(rproc_drm_invoke_service_t drm_service)
{
	if (drm_service == NULL)
		return -EINVAL;

	if (rproc_secure_drm_function)
		return -EEXIST;

	rproc_secure_drm_function = drm_service;
	return 0;
}
EXPORT_SYMBOL(rproc_register_drm_service);

/**
 * rproc_unregister_drm_service() - called by the rproc_drm module
 *				    to unregister the service.
 * @drm_service: the rproc_drm service that is being registered
 *
 * remoteproc calls this rproc_drm.rproc_drm_invoke_service upon calls
 *  to rproc_drm_invoke_service
 *
 * returns 0 on success
 */
int rproc_unregister_drm_service(rproc_drm_invoke_service_t drm_service)
{
	if (drm_service == NULL)
		return -EINVAL;

	if (drm_service != rproc_secure_drm_function)
		return -EINVAL;

	rproc_secure_drm_function = NULL;
	return 0;

}
EXPORT_SYMBOL(rproc_unregister_drm_service);
