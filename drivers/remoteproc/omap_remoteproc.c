/*
 * OMAP Remote Processor driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Mark Grosen <mgrosen@ti.com>
 * Suman Anna <s-anna@ti.com>
 * Hari Kanigeri <h-kanigeri2@ti.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/hwspinlock.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>

#include <plat/mailbox.h>
#include <plat/remoteproc.h>
#include <plat/dmtimer.h>

#include <linux/metricslog.h>

#include "omap_remoteproc.h"
#include "remoteproc_internal.h"

/* 1 sec is fair enough time for suspending an OMAP device */
#define DEF_SUSPEND_TIMEOUT 1000

/* How many pending messages/requests we can buffer. */
#define QUEUE_SIZE	256

/**
 * union oproc_pm_qos - holds the pm qos structure
 *			ipu being in core domain uses pm_qos API
 *			dsp can use the dev_pm_qos APIs
 * @pm_qos:	element associated with the pm_qos requests
 * @dev_pm_qos:	element associated with the dev_pm_qos requests
 *
 */
union oproc_pm_qos {
	struct pm_qos_request pm_qos;
	struct dev_pm_qos_request dev_pm_qos;
};

/**
 * struct hwspinlock_info - stores the remoteproc's hwspinlock state variables
 * @num_locks_va: kernel virtual address pointing to no. of spinlocks
 * @state_va: kernel virtual address of the 1st element of state array
 */
struct hwspinlock_info {
	unsigned *num_locks_va;
	unsigned long *state_va;
};

/**
 * struct omap_rproc - omap remote processor state
 * @mbox: omap mailbox handle
 * @nb: notifier block that will be invoked on inbound mailbox messages
 * @rproc: rproc handle
 * @boot_reg: virtual address of the register where the bootaddr is stored
 * @lat_req: for requesting latency constraints for rproc
 * @bw_req: for requesting L3 bandwidth constraints on behalf of rproc
 * @pm_comp: completion needed for suspend respond
 * @idle: address to the idle register
 * @idle_mask: mask of the idle register
 * @suspend_timeout: max time it can wait for the suspend respond
 * @suspend_acked: flag that says if the suspend request was acked
 * @suspended: flag that says if rproc suspended
 * @need_kick: flag that says if vrings need to be kicked on resume
 * @hwlock_info: virtual addresses of hwspinlock states shared by rproc
 *
 */
struct omap_rproc {
	struct omap_mbox *mbox;
	struct notifier_block nb;
	struct rproc *rproc;
	void __iomem *boot_reg;
	union oproc_pm_qos lat_req;
	struct pm_qos_request bw_req;
	struct completion pm_comp;
	void __iomem *idle;
	u32 idle_mask;
	unsigned long suspend_timeout;
	bool suspend_acked;
	bool suspended;
	bool need_kick;
	struct hwspinlock_info hwlock_info;

	spinlock_t queue_lock;
	DECLARE_KFIFO_PTR(queue_kfifo, uint32_t);
	struct task_struct *queue_thread;
	struct mutex thread_lock;
};

static int _vq_interrupt_thread(void *d)
{
	struct rproc *rproc = d;
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;

	current->flags |= PF_MEMALLOC;

	mutex_lock(&oproc->thread_lock);
	do {
		uint32_t msg;
		int ret;

		spin_lock_irq(&oproc->queue_lock);
		set_current_state(TASK_INTERRUPTIBLE);
		ret = kfifo_out(&oproc->queue_kfifo, &msg, 1);
		spin_unlock_irq(&oproc->queue_lock);

		if (ret == 1) {
			set_current_state(TASK_RUNNING);
			if (rproc_vq_interrupt(rproc, msg) == IRQ_NONE)
				dev_dbg(dev, "no message was found in vqid 0x0\n");
		} else {
			if (kthread_should_stop()) {
				set_current_state(TASK_RUNNING);
				break;
			}
			mutex_unlock(&oproc->thread_lock);
			schedule();
			mutex_lock(&oproc->thread_lock);
		}
	} while (1);
	mutex_unlock(&oproc->thread_lock);

	return 0;
}

void proc_log_metrics(char *msg)
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
 * omap_rproc_mbox_callback() - inbound mailbox message handler
 * @this: notifier block
 * @index: unused
 * @data: mailbox payload
 *
 * This handler is invoked by omap's mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we also have some out-of-band values
 * that indicates different events. Those values are deliberately very
 * big so they don't coincide with virtqueue indices.
 */
static int omap_rproc_mbox_callback(struct notifier_block *this,
					unsigned long index, void *data)
{
	mbox_msg_t msg = (mbox_msg_t) data;
	struct omap_rproc *oproc = container_of(this, struct omap_rproc, nb);
	struct device *dev = oproc->rproc->dev.parent;
	const char *name = oproc->rproc->name;
	unsigned long flags;
	int ret;

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	switch (msg) {
	case RP_MBOX_BOOTINIT_DONE:
		break;
	case RP_MBOX_CRASH:
		/* remoteproc detected an exception, notify the rproc core.
		 * The remoteproc core will handle the recovery. */
		dev_err(dev, "omap rproc %s crashed\n", name);
		proc_log_metrics("Crashed");
		rproc_error_reporter(oproc->rproc, RPROC_ERR_EXCEPTION);
		break;
	case RP_MBOX_ECHO_REPLY:
		dev_info(dev, "received echo reply from %s\n", name);
		break;
	case RP_MBOX_SUSPEND_ACK:
	case RP_MBOX_SUSPEND_CANCEL:
		oproc->suspend_acked = msg == RP_MBOX_SUSPEND_ACK;
		complete(&oproc->pm_comp);
		break;
	default:
		if (msg >= RP_MBOX_END_MSG) {
			dev_info(dev, "Dropping unknown message %x", msg);
			return NOTIFY_DONE;
		}

		spin_lock_irqsave(&oproc->queue_lock, flags);
		ret = kfifo_in(&oproc->queue_kfifo, &msg, 1);
		spin_unlock_irqrestore(&oproc->queue_lock, flags);
		if (ret == 1)
			wake_up_process(oproc->queue_thread);
	}

	return NOTIFY_DONE;
}

/* kick a virtqueue */
static void omap_rproc_kick(struct rproc *rproc, int vqid)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = oproc->rproc->dev.parent;
	int ret;

	/* if suspended set need kick flag to kick on resume */
	if (oproc->suspended) {
		oproc->need_kick = true;
		return;
	}

	if (oproc->mbox == NULL) {
		dev_warn(dev, "mbox not initialised yet. Skipping kick.\n");
		return;
	}

	/* send the index of the triggered virtqueue in the mailbox payload */
	ret = omap_mbox_msg_send(oproc->mbox, vqid);
	if (ret)
		dev_err(dev, "omap_mbox_msg_send failed: %d\n", ret);
}

static int
omap_rproc_set_latency(struct device *dev, struct rproc *rproc, long val)
{
	struct platform_device *pdev = to_platform_device(rproc->dev.parent);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc = rproc->priv;
	int ret = 0;

	/* Call device specific api if any */
	if (pdata->ops && pdata->ops->set_latency)
		return pdata->ops->set_latency(dev, rproc, val);

	if (!strcmp(rproc->name, "ipu_c0"))
		/* calling the C-state API for ipu since it is in core pd */
		pm_qos_update_request(&oproc->lat_req.pm_qos, val);
	else {
		ret = dev_pm_qos_update_request(
				&oproc->lat_req.dev_pm_qos, val);
		/*
		 * dev_pm_qos_update_request returns 0 or 1 on success depending
		 * on if the constraint changed or not (same request). So,
		 * return 0 in both cases
		 */
		ret = ret > 0 ? 0 : ret;
	}

	return ret;
}

static int
omap_rproc_set_bandwidth(struct device *dev, struct rproc *rproc, long val)
{
	struct omap_rproc *oproc = rproc->priv;

	pm_qos_update_request(&oproc->bw_req, val);

	return 0;
}

static int
omap_rproc_set_frequency(struct device *dev, struct rproc *rproc, long val)
{
	struct platform_device *pdev = to_platform_device(rproc->dev.parent);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;

	/* Call device specific api if any */
	if (pdata->ops && pdata->ops->set_frequency)
		return pdata->ops->set_frequency(dev, rproc, val);

	/* TODO: call platform specific */

	return 0;
}

static irqreturn_t omap_rproc_watchdog_isr(int irq, void *p)
{
	struct rproc *rproc = p;
	struct device *dev = rproc->dev.parent;
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
	struct omap_dm_timer *timer = NULL;
	int i;

	for (i = 0; i < pdata->timers_cnt; i++) {
		if (irq == omap_dm_timer_get_irq(timers[i].odt)) {
			timer = timers[i].odt;
			break;
		}
	}

	if (!timer) {
		dev_err(dev, "invalid timer\n");
		return IRQ_NONE;
	}
	omap_dm_timer_write_status(timer, OMAP_TIMER_INT_OVERFLOW);

	rproc_error_reporter(rproc, RPROC_ERR_WATCHDOG);

	return IRQ_HANDLED;
}

/*
 * Power up the remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met.
 */
static int omap_rproc_start(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
	int ret, i;

	/* load remote processor boot address if needed. */
	if (oproc->boot_reg)
		writel(rproc->bootaddr, oproc->boot_reg);

	spin_lock_init(&oproc->queue_lock);

	ret = kfifo_alloc(&oproc->queue_kfifo, QUEUE_SIZE, GFP_KERNEL);
	if (ret) {
		printk(KERN_ERR "error kfifo_alloc\n");
		return ret;
	}

	mutex_init(&oproc->thread_lock);

	oproc->queue_thread = kthread_run(_vq_interrupt_thread, rproc,
			"vp_interrupt_thread/%d", rproc->index);
	if (IS_ERR(oproc->queue_thread))
		return PTR_ERR(oproc->queue_thread);

	oproc->nb.notifier_call = omap_rproc_mbox_callback;

	/* every omap rproc is assigned a mailbox instance for messaging */
	oproc->mbox = omap_mbox_get(pdata->mbox_name, &oproc->nb);
	if (IS_ERR(oproc->mbox)) {
		ret = PTR_ERR(oproc->mbox);
		dev_err(dev, "omap_mbox_get failed: %d\n", ret);
		return ret;
	}

	/*
	 * ping the remote processor. this is only for sanity-sake;
	 * there is no functional effect whatsoever.
	 *
	 * note that the reply will _not_ arrive immediately: this message
	 * will wait in the mailbox fifo until the remote processor is booted.
	 */
	ret = omap_mbox_msg_send(oproc->mbox, RP_MBOX_ECHO_REQUEST);
	if (ret) {
		dev_err(dev, "omap_mbox_msg_send failed: %d\n", ret);
		goto put_mbox;
	}

	for (i = 0; i < pdata->timers_cnt; i++) {
		timers[i].odt = omap_dm_timer_request_specific(timers[i].id);
		if (!timers[i].odt) {
			ret = -EBUSY;
			dev_err(dev, "request for timer %d failed: %d\n",
							timers[i].id, ret);
			goto err_timers;
		}
		omap_dm_timer_set_source(timers[i].odt, OMAP_TIMER_SRC_SYS_CLK);

		if (timers[i].is_wdt) {
			ret = request_irq(omap_dm_timer_get_irq(timers[i].odt),
					omap_rproc_watchdog_isr, IRQF_SHARED,
					"rproc-wdt", rproc);
			if (ret) {
				dev_err(dev,
					"error requesting irq for timer %d\n",
								timers[i].id);
				omap_dm_timer_free(timers[i].odt);
				timers[i].odt = NULL;
				goto err_timers;
			}
			/* clean counter, remoteproc proc will set the value */
			omap_dm_timer_set_load(timers[i].odt, 0, 0);
		}
		omap_dm_timer_start(timers[i].odt);
	}

	ret = pdata->device_enable(pdev);
	if (ret) {
		dev_err(dev, "omap_device_enable failed: %d\n", ret);
		goto err_timers;
	}

	return 0;

err_timers:
	while (i--) {
		omap_dm_timer_stop(timers[i].odt);
		if (timers[i].is_wdt)
			free_irq(omap_dm_timer_get_irq(timers[i].odt), rproc);

		omap_dm_timer_free(timers[i].odt);
		timers[i].odt = NULL;
	}

put_mbox:
	omap_mbox_put(oproc->mbox, &oproc->nb);
	return ret;
}

/**
 * Helper function to reset spinlocks
 *
 * There is a possibility that the remote processor was
 * holding spinlocks when an exception occured. One way
 * of dealing with it would be for the driver to unlock
 * all hwspinlocks that were held by the remote processor.
 */
static inline void reset_hwspinlock(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	int id;

	/* reset only the spinlocks that were marked held */
	if (!oproc->hwlock_info.num_locks_va || !oproc->hwlock_info.state_va) {
		dev_err(dev, "invalid va\n");
		return;
	}
	for_each_set_bit(id, oproc->hwlock_info.state_va,
					*(oproc->hwlock_info.num_locks_va))
		__hwspin_lock_reset(id);
}

/* power off the remote processor */
static int omap_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc = rproc->priv;
	struct omap_rproc_timers_info *timers = pdata->timers;
	int ret, i;

	/* reset the hwspinlocks held by remote processor */
	reset_hwspinlock(rproc);

	ret = pdata->device_shutdown(pdev);
	if (ret)
		return ret;

	for (i = 0; i < pdata->timers_cnt; i++) {
		omap_dm_timer_stop(timers[i].odt);
		if (timers[i].is_wdt)
			free_irq(omap_dm_timer_get_irq(timers[i].odt), rproc);

		omap_dm_timer_free(timers[i].odt);
		timers[i].odt = NULL;
	}

	omap_mbox_put(oproc->mbox, &oproc->nb);
	oproc->mbox = NULL;

	kthread_stop(oproc->queue_thread);
	kfifo_free(&oproc->queue_kfifo);

	return 0;
}

static int omap_rproc_cb_barrier(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	int fifo_empty;

	do {
		mutex_lock(&oproc->thread_lock);
		spin_lock_irq(&oproc->queue_lock);
		fifo_empty = kfifo_is_empty(&oproc->queue_kfifo);
		spin_unlock_irq(&oproc->queue_lock);
		mutex_unlock(&oproc->thread_lock);
		if (!fifo_empty)
			schedule();
	} while (!fifo_empty);

	return 0;
}

static bool _rproc_idled(struct omap_rproc *oproc)
{
	return !oproc->idle || readl(oproc->idle) & oproc->idle_mask;
}

static int _suspend(struct rproc *rproc, bool auto_suspend)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc *oproc = rproc->priv;
	struct omap_rproc_timers_info *timers = pdata->timers;
	unsigned long to = msecs_to_jiffies(oproc->suspend_timeout);
	unsigned long ta = jiffies + to;
	int ret, i;

	init_completion(&oproc->pm_comp);
	oproc->suspend_acked = false;
	omap_mbox_msg_send(oproc->mbox,
		auto_suspend ? RP_MBOX_SUSPEND : RP_MBOX_SUSPEND_FORCED);
	ret = wait_for_completion_timeout(&oproc->pm_comp, to);
	if (!oproc->suspend_acked) {
		dev_err(dev, "The request isn't accepted by Ducati\n");
		return -EBUSY;
	}

	/*
	 * FIXME: Ducati side is returning the ACK message before saving the
	 * context, becuase the function which saves the context is a
	 * SYSBIOS function that can not be modified until a new SYSBIOS
	 * release is done. However, we can know that Ducati already saved
	 * the context once it reaches idle again (after saving the context
	 * ducati executes WFI instruction), so this way we can workaround
	 * this problem.
	 */
	if (oproc->idle) {
		while (!_rproc_idled(oproc)) {
			if (time_after(jiffies, ta)) {
				dev_err(dev, "Ducati can't reach idle state\n");
				return -ETIME;
			}
			schedule();
		}
	}

	ret = pdata->device_shutdown(pdev);
	if (ret)
		return ret;

	for (i = 0; i < pdata->timers_cnt; i++)
		omap_dm_timer_stop(timers[i].odt);

	omap_mbox_disable(oproc->mbox);

	oproc->suspended = true;

	return 0;
}

static int omap_rproc_suspend(struct rproc *rproc, bool auto_suspend)
{
	struct omap_rproc *oproc = rproc->priv;

	if (auto_suspend && !_rproc_idled(oproc)) {
		struct device *dev = rproc->dev.parent;

		dev_err(dev, "Unexpected module standby status.\n");
		return -EBUSY;
	}
	return _suspend(rproc, auto_suspend);
}

static int _resume_kick(int id, void *p, void *rproc)
{
	omap_rproc_kick(rproc, id);
	return 0;
}

static int omap_rproc_resume(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc *oproc = rproc->priv;
	struct omap_rproc_timers_info *timers = pdata->timers;
	int ret, i;

	/* boot address could be lost after suspend, so restore it */
	if (oproc->boot_reg)
		writel(rproc->bootaddr, oproc->boot_reg);

	omap_mbox_enable(oproc->mbox);

	/*
	 * if need_kick flag is true, we need to kick all the vrings as
	 * we do not know which vrings were tried to be kicked while the
	 * rproc was suspended. We can optimize later, however this scenario
	 * is very rarely, so it is not big deal.
	 */
	if (oproc->need_kick) {
		idr_for_each(&rproc->notifyids, _resume_kick, rproc);
		oproc->need_kick = false;
	}

	for (i = 0; i < pdata->timers_cnt; i++)
		omap_dm_timer_start(timers[i].odt);

	ret = pdata->device_enable(pdev);
	if (ret) {
		for (i = 0; i < pdata->timers_cnt; i++)
			omap_dm_timer_stop(timers[i].odt);
		omap_mbox_disable(oproc->mbox);
		return ret;
	}

	oproc->suspended = false;

	return 0;
}

static inline int omap_rproc_handle_hwspin_rsc(struct rproc *rproc,
				struct fw_rsc_custom_spinlock *hw_rsc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;

	oproc->hwlock_info.num_locks_va = (unsigned *)rproc_da_to_va(rproc,
				hw_rsc->num_locks_da, sizeof(unsigned *));
	oproc->hwlock_info.state_va = (unsigned long *)rproc_da_to_va(rproc,
					hw_rsc->da, sizeof(unsigned long *));

	if (!oproc->hwlock_info.num_locks_va || !oproc->hwlock_info.state_va) {
		dev_err(dev, "Invalid hwspinlock resource info\n");
		return -EINVAL;
	}

	return 0;
}

static inline int omap_rproc_handle_custom_rsc(struct rproc *rproc,
					struct fw_rsc_custom *rsc)
{
	struct device *dev = rproc->dev.parent;
	int ret = -EINVAL;

	switch (rsc->sub_type) {
	case OMAP_RSC_HWSPIN:
		ret = omap_rproc_handle_hwspin_rsc(rproc,
			(struct fw_rsc_custom_spinlock *)rsc->data);
		break;
	default:
		dev_err(dev, "%s: handling unknown type %d\n", __func__,
								rsc->sub_type);
	}
	return ret;
}

static struct rproc_ops omap_rproc_ops = {
	.start			= omap_rproc_start,
	.stop			= omap_rproc_stop,
	.kick			= omap_rproc_kick,
	.cb_barrier		= omap_rproc_cb_barrier,
	.suspend		= omap_rproc_suspend,
	.resume			= omap_rproc_resume,
	.set_latency		= omap_rproc_set_latency,
	.set_bandwidth		= omap_rproc_set_bandwidth,
	.set_frequency		= omap_rproc_set_frequency,
	.handle_custom_rsc	= omap_rproc_handle_custom_rsc,
};

static inline int omap_rproc_add_lat_request(struct omap_rproc *oproc)
{
	struct rproc *rproc = oproc->rproc;
	int ret = 0;

	if (!strcmp(rproc->name, "ipu_c0"))
		pm_qos_add_request(&oproc->lat_req.pm_qos,
				PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	else
		ret = dev_pm_qos_add_request(rproc->dev.parent,
			&oproc->lat_req.dev_pm_qos, PM_QOS_DEFAULT_VALUE);

	return ret;
}

static inline void omap_rproc_remove_lat_request(struct omap_rproc *oproc)
{
	struct rproc *rproc = oproc->rproc;

	if (!strcmp(rproc->name, "ipu_c0"))
		pm_qos_remove_request(&oproc->lat_req.pm_qos);
	else
		dev_pm_qos_remove_request(&oproc->lat_req.dev_pm_qos);
}

static int __devinit omap_rproc_probe(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc;
	struct rproc *rproc;
	int ret;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(&pdev->dev, pdata->name, &omap_rproc_ops,
				pdata->firmware, pdata->pool_data,
				sizeof(*oproc));
	if (!rproc)
		return -ENOMEM;

	oproc = rproc->priv;
	oproc->rproc = rproc;
	oproc->suspend_timeout = pdata->suspend_timeout ? : DEF_SUSPEND_TIMEOUT;
	init_completion(&oproc->pm_comp);

	if (pdata->idle_addr) {
		oproc->idle = ioremap(pdata->idle_addr, sizeof(u32));
		if (!oproc->idle)
			goto free_rproc;
		oproc->idle_mask = pdata->idle_mask;
	}

	if (pdata->boot_reg) {
		oproc->boot_reg = ioremap(pdata->boot_reg, sizeof(u32));
		if (!oproc->boot_reg)
			goto iounmap;
	}

	platform_set_drvdata(pdev, rproc);

	ret = omap_rproc_add_lat_request(oproc);
	if (ret)
		goto iounmap;

	pm_qos_add_request(&oproc->bw_req, PM_QOS_MEMORY_THROUGHPUT,
				PM_QOS_MEMORY_THROUGHPUT_DEFAULT_VALUE);

	ret = rproc_register(rproc);
	if (ret)
		goto remove_req;

	return 0;

remove_req:
	pm_qos_remove_request(&oproc->bw_req);
	omap_rproc_remove_lat_request(oproc);
iounmap:
	if (oproc->idle)
		iounmap(oproc->idle);
	if (oproc->boot_reg)
		iounmap(oproc->boot_reg);
free_rproc:
	rproc_free(rproc);
	return ret;
}

static int __devexit omap_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct omap_rproc *oproc = rproc->priv;

	if (oproc->idle)
		iounmap(oproc->idle);

	if (oproc->boot_reg)
		iounmap(oproc->boot_reg);

	pm_qos_remove_request(&oproc->bw_req);
	omap_rproc_remove_lat_request(oproc);
	return rproc_unregister(rproc);
}

static struct platform_driver omap_rproc_driver = {
	.probe = omap_rproc_probe,
	.remove = __devexit_p(omap_rproc_remove),
	.driver = {
		.name = "omap-rproc",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(omap_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote Processor control driver");
