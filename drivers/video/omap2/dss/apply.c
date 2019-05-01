/*
 * Copyright (C) 2011 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
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

#define DSS_SUBSYS_NAME "APPLY"

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/ratelimit.h>
#include <linux/seq_file.h>

#include <video/omapdss.h>

#include "dss.h"
#include "dss_features.h"
#include <linux/pm_qos.h>
#include <linux/trapz.h> /* ACOS_MOD_ONELINE */

bool band_changed;
static int pm_init_cstr = 0;
extern struct pm_qos_request req;
#define OVERLAY_AREA_BW_THRESHOLD (1920*1080)

struct callback_states {
	/*
	 * Keep track of callbacks at the last 3 levels of pipeline:
	 * info, shadow registers and in DISPC registers.
	 *
	 * Note: We zero the function pointer when moving from one level to
	 * another to avoid checking for dirty and shadow_dirty fields that
	 * are not common between overlay and manager cache structures.
	 */
	struct omapdss_ovl_cb info, shadow, dispc;
	bool dispc_displayed;
	bool shadow_enabled;
};

/*
 * We have 4 levels of cache for the dispc settings. First two are in SW and
 * the latter two in HW.
 *
 *       set_info()
 *          v
 * +--------------------+
 * |     user_info      |
 * +--------------------+
 *          v
 *        apply()
 *          v
 * +--------------------+
 * |       info         |
 * +--------------------+
 *          v
 *      write_regs()
 *          v
 * +--------------------+
 * |  shadow registers  |
 * +--------------------+
 *          v
 * VFP or lcd/digit_enable
 *          v
 * +--------------------+
 * |      registers     |
 * +--------------------+
 */

struct ovl_priv_data {

	bool user_info_dirty;
	struct omap_overlay_info user_info;

	bool info_dirty;
	struct omap_overlay_info info;

	/* callback data for the last 3 states */
	struct callback_states cb;
	/* overlay's channel in DISPC */
	int dispc_channel;

	bool shadow_info_dirty;

	bool extra_info_dirty;
	bool shadow_extra_info_dirty;

	bool enabled;
	enum omap_channel channel;
	u32 fifo_low, fifo_high;

	/*
	 * True if overlay is to be enabled. Used to check and calculate configs
	 * for the overlay before it is enabled in the HW.
	 */
	bool enabling;
};

struct mgr_priv_data {

	bool user_info_dirty;
	struct omap_overlay_manager_info user_info;

	bool info_dirty;
	struct omap_overlay_manager_info info;

	bool shadow_info_dirty;

	/* If true, GO bit is up and shadow registers cannot be written.
	 * Never true for manual update displays */
	bool busy;

	/* If true, dispc output is enabled */
	bool updating;

	/* If true, a display is enabled using this manager */
	bool enabled;

	/* callback data for the last 3 states */
	struct callback_states cb;

	bool skip_init;

};

static struct {
	struct ovl_priv_data ovl_priv_data_array[MAX_DSS_OVERLAYS];
	struct mgr_priv_data mgr_priv_data_array[MAX_DSS_MANAGERS];
	struct writeback_cache_data writeback_cache;

	bool fifo_merge_dirty;
	bool fifo_merge;

	bool irq_enabled;
	u32 comp_irq_enabled;
} dss_data;

/* propagating callback info between states */
static inline void
dss_ovl_configure_cb(struct callback_states *st, int i, bool enabled)
{
	/* complete info in shadow */
	dss_ovl_cb(&st->shadow, i, DSS_COMPLETION_ECLIPSED_SHADOW);

	/* propagate info to shadow */
	st->shadow = st->info;
	st->shadow_enabled = enabled;
	/* info traveled to shadow */
	st->info.fn = NULL;
}

static inline void
dss_ovl_program_cb(struct callback_states *st, int i)
{
	/* mark previous programming as completed */
	dss_ovl_cb(&st->dispc, i, st->dispc_displayed ?
				DSS_COMPLETION_RELEASED : DSS_COMPLETION_TORN);

	/* mark shadow info as programmed, not yet displayed */
	dss_ovl_cb(&st->shadow, i, DSS_COMPLETION_PROGRAMMED);

	/* if overlay/manager is not enabled, we are done now */
	if (!st->shadow_enabled) {
		dss_ovl_cb(&st->shadow, i, DSS_COMPLETION_RELEASED);
		st->shadow.fn = NULL;
	}

	/* propagate shadow to dispc */
	st->dispc = st->shadow;
	st->shadow.fn = NULL;
	st->dispc_displayed = false;
}

/* protects dss_data */
static spinlock_t data_lock;
/* lock for blocking functions */
static DEFINE_MUTEX(apply_lock);
static DECLARE_COMPLETION(extra_updated_completion);

static void dss_register_vsync_isr(void);

static struct ovl_priv_data *get_ovl_priv(struct omap_overlay *ovl)
{
	return &dss_data.ovl_priv_data_array[ovl->id];
}

static struct mgr_priv_data *get_mgr_priv(struct omap_overlay_manager *mgr)
{
	return &dss_data.mgr_priv_data_array[mgr->id];
}

void dss_apply_init(void)
{
	const int num_ovls = dss_feat_get_num_ovls();
	int i;

	spin_lock_init(&data_lock);

	for (i = 0; i < num_ovls; ++i) {
		struct ovl_priv_data *op;

		op = &dss_data.ovl_priv_data_array[i];

		op->info.global_alpha = 255;

		switch (i) {
		case 0:
			op->info.zorder = 0;
			break;
		case 1:
			op->info.zorder =
				dss_has_feature(FEAT_ALPHA_FREE_ZORDER) ? 3 : 0;
			break;
		case 2:
			op->info.zorder =
				dss_has_feature(FEAT_ALPHA_FREE_ZORDER) ? 2 : 0;
			break;
		case 3:
			op->info.zorder =
				dss_has_feature(FEAT_ALPHA_FREE_ZORDER) ? 1 : 0;
			break;
		}

		op->user_info = op->info;
	}
}

static bool ovl_manual_update(struct omap_overlay *ovl)
{
	return ovl->manager->device->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE;
}

static bool mgr_manual_update(struct omap_overlay_manager *mgr)
{
	return mgr->device->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE;
}

static int dss_check_settings_low(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev, bool applying)
{
	struct omap_overlay_info *oi;
	struct omap_overlay_manager_info *mi;
	struct omap_overlay *ovl;
	struct omap_overlay_info *ois[MAX_DSS_OVERLAYS];
	struct ovl_priv_data *op;
	struct mgr_priv_data *mp;

	mp = get_mgr_priv(mgr);

	if (applying && mp->user_info_dirty)
		mi = &mp->user_info;
	else
		mi = &mp->info;

	/* collect the infos to be tested into the array */
	list_for_each_entry(ovl, &mgr->overlays, list) {
		op = get_ovl_priv(ovl);

		if (!op->enabled && !op->enabling)
			oi = NULL;
		else if (applying && op->user_info_dirty)
			oi = &op->user_info;
		else
			oi = &op->info;

		ois[ovl->id] = oi;
	}

	return dss_mgr_check(mgr, dssdev, mi, ois);
}

/*
 * check manager and overlay settings using overlay_info from data->info
 */
static int dss_check_settings(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev)
{
	return dss_check_settings_low(mgr, dssdev, false);
}

/*
 * check manager and overlay settings using overlay_info from ovl->info if
 * dirty and from data->info otherwise
 */
static int dss_check_settings_apply(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev)
{
	return dss_check_settings_low(mgr, dssdev, true);
}

static bool need_isr(void)
{
	const int num_mgrs = dss_feat_get_num_mgrs();
	int i;

	for (i = 0; i < num_mgrs; ++i) {
		struct omap_overlay_manager *mgr;
		struct mgr_priv_data *mp;
		struct omap_overlay *ovl;

		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);

		if (!mp->enabled)
			continue;

		if (mgr_manual_update(mgr)) {
			/* to catch FRAMEDONE */
			if (mp->updating)
				return true;
		} else {
			/* to catch GO bit going down */
			if (mp->busy)
				return true;

			/* to write new values to registers */
			if (mp->info_dirty)
				return true;

			/* to set GO bit */
			if (mp->shadow_info_dirty)
				return true;

			list_for_each_entry(ovl, &mgr->overlays, list) {
				struct ovl_priv_data *op;

				op = get_ovl_priv(ovl);

				/*
				 * NOTE: we check extra_info flags even for
				 * disabled overlays, as extra_infos need to be
				 * always written.
				 */

				/* to write new values to registers */
				if (op->extra_info_dirty)
					return true;

				/* to set GO bit */
				if (op->shadow_extra_info_dirty)
					return true;

				if (!op->enabled)
					continue;

				/* to write new values to registers */
				if (op->info_dirty)
					return true;

				/* to set GO bit */
				if (op->shadow_info_dirty)
					return true;
			}
		}
	}

	return false;
}

static bool need_go(struct omap_overlay_manager *mgr)
{
	struct omap_overlay *ovl;
	struct mgr_priv_data *mp;
	struct ovl_priv_data *op;

	mp = get_mgr_priv(mgr);

	if (mp->shadow_info_dirty)
		return true;

	list_for_each_entry(ovl, &mgr->overlays, list) {
		op = get_ovl_priv(ovl);
		if (op->shadow_info_dirty || op->shadow_extra_info_dirty)
			return true;
	}

	return false;
}

/* returns true if an extra_info field is currently being updated */
static bool extra_info_update_ongoing(void)
{
	const int num_ovls = omap_dss_get_num_overlays();
	struct ovl_priv_data *op;
	struct omap_overlay *ovl;
	struct mgr_priv_data *mp;
	int i;

	for (i = 0; i < num_ovls; ++i) {
		ovl = omap_dss_get_overlay(i);
		op = get_ovl_priv(ovl);

		if (!ovl->manager)
			continue;

		mp = get_mgr_priv(ovl->manager);

		if (!mp->enabled)
			continue;

		if (!mp->updating)
			continue;

		if (op->extra_info_dirty || op->shadow_extra_info_dirty)
			return true;
	}

	return false;
}

/* wait until no extra_info updates are pending */
static void wait_pending_extra_info_updates(void)
{
	bool updating;
	unsigned long flags;
	unsigned long t;
	int r;

	spin_lock_irqsave(&data_lock, flags);

	updating = extra_info_update_ongoing();

	if (!updating) {
		spin_unlock_irqrestore(&data_lock, flags);
		return;
	}

	init_completion(&extra_updated_completion);

	spin_unlock_irqrestore(&data_lock, flags);

	t = msecs_to_jiffies(500);
	r = wait_for_completion_timeout(&extra_updated_completion, t);
	if (r == 0)
		DSSWARN("timeout in wait_pending_extra_info_updates\n");
	else if (r < 0)
		DSSERR("wait_pending_extra_info_updates failed: %d\n", r);
}

int dss_mgr_wait_for_go(struct omap_overlay_manager *mgr)
{
	unsigned long timeout = msecs_to_jiffies(500);
	struct mgr_priv_data *mp;
	u32 irq;
	int r;
	int i;
	struct omap_dss_device *dssdev = mgr->device;

	if (!dssdev || dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	if (mgr_manual_update(mgr))
		return 0;

	r = dispc_runtime_get();
	if (r)
		return r;

	irq = dispc_mgr_get_vsync_irq(mgr->id);

	mp = get_mgr_priv(mgr);
	i = 0;
	while (1) {
		unsigned long flags;
		bool shadow_dirty, dirty;

		spin_lock_irqsave(&data_lock, flags);
		dirty = mp->info_dirty;
		shadow_dirty = mp->shadow_info_dirty;
		spin_unlock_irqrestore(&data_lock, flags);

		if (!dirty && !shadow_dirty) {
			r = 0;
			break;
		}

		/* 4 iterations is the worst case:
		 * 1 - initial iteration, dirty = true (between VFP and VSYNC)
		 * 2 - first VSYNC, dirty = true
		 * 3 - dirty = false, shadow_dirty = true
		 * 4 - shadow_dirty = false */
		if (i++ == 3) {
			DSSERR("mgr(%d)->wait_for_go() not finishing\n",
					mgr->id);
			r = 0;
			break;
		}

		r = omap_dispc_wait_for_irq_interruptible_timeout(irq, timeout);
		if (!r)
			mgr->device->first_vsync = true;

		if (r == -ERESTARTSYS)
			break;

		if (r) {
			DSSERR("mgr(%d)->wait_for_go() timeout\n", mgr->id);
			break;
		}
	}

	dispc_runtime_put();

	return r;
}

int dss_mgr_wait_for_go_ovl(struct omap_overlay *ovl)
{
	unsigned long timeout = msecs_to_jiffies(500);
	struct ovl_priv_data *op;
	struct omap_dss_device *dssdev;
	u32 irq;
	int r;
	int i;

	if (!ovl->manager)
		return 0;

	dssdev = ovl->manager->device;

	if (!dssdev || dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	if (ovl_manual_update(ovl))
		return 0;

	r = dispc_runtime_get();
	if (r)
		return r;

	irq = dispc_mgr_get_vsync_irq(ovl->manager->id);

	op = get_ovl_priv(ovl);
	i = 0;
	while (1) {
		unsigned long flags;
		bool shadow_dirty, dirty;

		spin_lock_irqsave(&data_lock, flags);
		dirty = op->info_dirty;
		shadow_dirty = op->shadow_info_dirty;
		spin_unlock_irqrestore(&data_lock, flags);

		if (!dirty && !shadow_dirty) {
			r = 0;
			break;
		}

		/* 4 iterations is the worst case:
		 * 1 - initial iteration, dirty = true (between VFP and VSYNC)
		 * 2 - first VSYNC, dirty = true
		 * 3 - dirty = false, shadow_dirty = true
		 * 4 - shadow_dirty = false */
		if (i++ == 3) {
			DSSERR("ovl(%d)->wait_for_go() not finishing\n",
					ovl->id);
			r = 0;
			break;
		}

		r = omap_dispc_wait_for_irq_interruptible_timeout(irq, timeout);
		if (r == -ERESTARTSYS)
			break;

		if (r) {
			DSSERR("ovl(%d)->wait_for_go() timeout\n", ovl->id);
			break;
		}
	}

	dispc_runtime_put();

	return r;
}

static void dss_ovl_write_regs(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	struct omap_overlay_info *oi;
	bool ilace, replication;
	struct mgr_priv_data *mp;
	u16 x_decim, y_decim;
	bool five_taps = true;
	int r;
	struct writeback_cache_data *wbc;
	bool m2m_with_ovl = false;
	bool m2m_with_mgr = false;

	DSSDBGF("%d", ovl->id);

	if (!op->enabled || !op->info_dirty)
		return;

	if (dss_has_feature(FEAT_WB)) {
		/* check if this overlay is source for wb, ignore mgr sources
		 * here */
		wbc = &dss_data.writeback_cache;
		if (wbc->enabled && omap_dss_check_wb(wbc, ovl->id, -1)) {
			DSSDBG("wb->enabled=%d for plane:%d\n",
						wbc->enabled, ovl->id);
			m2m_with_ovl = true;
		}
		/* check if this overlay is source for manager, which is source
		 * for wb, ignore ovl sources */
		if (wbc->mode == OMAP_WB_MEM2MEM_MODE &&
				omap_dss_check_wb(wbc, -1, op->channel)) {
			DSSDBG("check wb mgr wb->enabled=%d for plane:%d\n",
							wbc->enabled, ovl->id);
			m2m_with_mgr = true;
		}
	}

	oi = &op->info;

	replication = dss_use_replication(ovl->manager->device, oi->color_mode);

	ilace = ovl->manager->device->type == OMAP_DISPLAY_TYPE_VENC;

	if (m2m_with_mgr || m2m_with_ovl) {
		/* do not calculate a pixel clock and decimation for WB */
		x_decim = 1;
		y_decim = 1;
		five_taps = true;
		r = 0;
	} else
		r = dispc_scaling_decision(ovl->id, oi, op->channel,
					&x_decim, &y_decim, &five_taps);

	r = r ? : dispc_ovl_setup(ovl->id, oi, ilace,
			replication, x_decim, y_decim, five_taps,
						m2m_with_ovl || m2m_with_mgr);
	if (r) {
		/*
		 * We can't do much here, as this function can be called from
		 * vsync interrupt.
		 */
		DSSERR("dispc_ovl_setup failed for ovl %d\n", ovl->id);

		/* This will leave fifo configurations in a nonoptimal state */
		op->enabled = false;
		dispc_ovl_enable(ovl->id, false);
		dss_ovl_configure_cb(&op->cb, ovl->id, op->enabled);
		return;
	}

	mp = get_mgr_priv(ovl->manager);

	op->info_dirty = false;
	if (mp->updating) {
		dss_ovl_configure_cb(&op->cb, ovl->id, op->enabled);
		op->shadow_info_dirty = true;
	}
}

static void dss_ovl_write_regs_extra(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	struct mgr_priv_data *mp;
	struct writeback_cache_data *wbc;
	bool m2m_with_ovl = false;
	bool m2m_with_mgr = false;

	DSSDBGF("%d", ovl->id);

	if (!op->extra_info_dirty)
		return;

	if (dss_has_feature(FEAT_WB)) {
		/*
		 * Check, if this overlay is source for wb, then ignore mgr
		 * sources here.
		 */
		wbc = &dss_data.writeback_cache;
		if (wbc->enabled && omap_dss_check_wb(wbc, ovl->id, -1)) {
			DSSDBG("wb->enabled=%d for plane:%d\n",
						wbc->enabled, ovl->id);
			m2m_with_ovl = true;
		}
		/*
		 * Check, if this overlay is source for manager, which is
		 * source for wb, then ignore ovl sources.
		 */
		if (wbc->enabled && omap_dss_check_wb(wbc, -1, op->channel)) {
			DSSDBG("check wb mgr wb->enabled=%d for plane:%d\n",
							wbc->enabled, ovl->id);
			m2m_with_mgr = true;
		}
	}

	/* note: write also when op->enabled == false, so that the ovl gets
	 * disabled */

	dispc_ovl_enable(ovl->id, op->enabled);

	if (!m2m_with_ovl)
		dispc_ovl_set_channel_out(ovl->id, op->channel);
	else
		dispc_set_wb_channel_out(ovl->id);

	dispc_ovl_set_fifo_threshold(ovl->id, op->fifo_low, op->fifo_high);

	mp = get_mgr_priv(ovl->manager);

	op->extra_info_dirty = false;
	if (mp->updating) {
		dss_ovl_configure_cb(&op->cb, ovl->id, op->enabled);
		op->shadow_extra_info_dirty = true;
	}
}

static void dss_mgr_write_regs(struct omap_overlay_manager *mgr)
{
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	struct omap_overlay *ovl;
	struct ovl_priv_data *op;
	int used_ovls = 0;

	DSSDBGF("%d", mgr->id);

	if (!mp->enabled && !mp->info.wb_only)
		return;

	WARN_ON(mp->busy);

	/* Commit overlay settings */
	list_for_each_entry(ovl, &mgr->overlays, list) {
		dss_ovl_write_regs(ovl);
		dss_ovl_write_regs_extra(ovl);
		op = get_ovl_priv(ovl);
		if (op->channel == mgr->id && op->enabled)
			used_ovls++;
	}

	if (mp->info_dirty) {
		dispc_mgr_setup(mgr->id, &mp->info);
		mp->info_dirty = false;
		if (mp->updating) {
			dss_ovl_configure_cb(&mp->cb, mgr->id, used_ovls);
			mp->shadow_info_dirty = true;
		}
	}
}

static void dss_write_regs_common(void)
{
	const int num_mgrs = omap_dss_get_num_overlay_managers();
	int i;

	if (!dss_data.fifo_merge_dirty)
		return;

	for (i = 0; i < num_mgrs; ++i) {
		struct omap_overlay_manager *mgr;
		struct mgr_priv_data *mp;

		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);

		if (mp->enabled) {
			if (dss_data.fifo_merge_dirty) {
				dispc_enable_fifomerge(dss_data.fifo_merge);
				dss_data.fifo_merge_dirty = false;
			}

			if (mp->updating)
				mp->shadow_info_dirty = true;
		}
	}
}

static void dss_write_regs(void)
{
	const int num_mgrs = omap_dss_get_num_overlay_managers();
	int i;

	dss_write_regs_common();

	for (i = 0; i < num_mgrs; ++i) {
		struct omap_overlay_manager *mgr;
		struct mgr_priv_data *mp;
		int r;

		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);

		if (!mp->enabled || mgr_manual_update(mgr) || mp->busy)
			if (!mp->info.wb_only)
				continue;

		r = dss_check_settings(mgr, mgr->device);
		if (r) {
			DSSERR("cannot write registers for manager %s: "
					"illegal configuration\n", mgr->name);
			continue;
		}

		dss_mgr_write_regs(mgr);
	}
}

static void dss_set_go_bits(void)
{
	const int num_mgrs = omap_dss_get_num_overlay_managers();
	int i;

	for (i = 0; i < num_mgrs; ++i) {
		struct omap_overlay_manager *mgr;
		struct mgr_priv_data *mp;

		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);

		if (!mp->enabled || mgr_manual_update(mgr) || mp->busy)
			continue;

		if (!need_go(mgr))
			continue;

		mp->busy = true;

		if (!dss_data.irq_enabled && need_isr())
			dss_register_vsync_isr();

		DSSDBG("%s %d\n",__FUNCTION__,mp->skip_init);
		if(mp->skip_init)
			mp->skip_init = false;
		else
			dispc_mgr_go(mgr->id);
	}

}

static void dss_wb_write_regs(void)
{
	struct writeback_cache_data *wbc;
	int r = 0;
	if (dss_has_feature(FEAT_WB))
		wbc = &dss_data.writeback_cache;
	else
		wbc = NULL;
	/* setup WB for capture mode */
	if (wbc && wbc->enabled && wbc->dirty) {
		/* writeback is enabled for this plane - set accordingly */
		r = dispc_setup_wb(wbc);
		if (r)
			DSSERR("dispc_setup_wb failed with error %d\n", r);
		wbc->dirty = false;
		wbc->shadow_dirty = true;
	}
}
static void dss_wb_ovl_enable(void)
{
	struct writeback_cache_data *wbc;
	struct omap_overlay *ovl;
	const int num_ovls = dss_feat_get_num_ovls();
	int i;

	if (dss_has_feature(FEAT_WB))
		wbc = &dss_data.writeback_cache;
	else
		wbc = NULL;
	if (dss_has_feature(FEAT_WB)) {
		/* Enable WB plane and source plane */
		DSSDBG("configure manager wbc->shadow_dirty = %d",
		wbc->shadow_dirty);
		if (wbc->shadow_dirty && wbc->enabled) {
			switch (wbc->source) {
			case OMAP_WB_GFX:
			case OMAP_WB_VID1:
			case OMAP_WB_VID2:
			case OMAP_WB_VID3:
				wbc->shadow_dirty = false;
				dispc_ovl_enable(OMAP_DSS_WB, true);
				break;
			case OMAP_WB_LCD1:
			case OMAP_WB_LCD2:
			case OMAP_WB_TV:
				dispc_ovl_enable(OMAP_DSS_WB, true);
				wbc->shadow_dirty = false;
				break;
			}
		} else if (wbc->dirty && !wbc->enabled) {
			if (wbc->mode == OMAP_WB_MEM2MEM_MODE &&
				wbc->source >= OMAP_WB_GFX) {
				/* This is a workaround. According to TRM
				 * we should disable the manager but it will
				 * cause blinking of panel. WA is to disable
				 * pipe which was used as source of WB and do
				 * dummy enable and disable of WB.
				 */
				dispc_ovl_enable(OMAP_DSS_WB, true);
				dispc_ovl_enable(OMAP_DSS_WB, false);
			} else if (wbc->mode == OMAP_WB_MEM2MEM_MODE &&
					wbc->source < OMAP_WB_GFX) {
				/* This is a workaround that prevents SYNC_LOST
				 * on changing pipe channelout from manager
				 * which was used as a source of wb to another
				 * manager. Manager could free pipes after wb
				 * will send SYNC message but that will start
				 * wb capture. To prevent that we reconnect the
				 * pipe from the manager to wb and do a dummy
				 * enabling and disabling of wb - the pipe will
				 * be freed and capture won't start because
				 * source pipe is switched off. */
				for (i = 0; i < num_ovls; ++i) {
					ovl = omap_dss_get_overlay(i);

					if ((int)ovl->manager->id ==
						(int)wbc->source) {
						dispc_ovl_enable(i, false);
						dispc_setup_wb_source(
							OMAP_DSS_GFX + i);
						dispc_set_wb_channel_out(i);
						dispc_ovl_enable(
							OMAP_DSS_WB, true);
						dispc_ovl_enable(
							OMAP_DSS_WB, false);
					}
				}
			} else
				/* capture mode case */
				dispc_ovl_enable(OMAP_DSS_WB, false);

			wbc->dirty = false;
		}
	}
}

static void dss_wb_set_go_bits(void)
{
	struct writeback_cache_data *wbc;
	if (dss_has_feature(FEAT_WB))
		wbc = &dss_data.writeback_cache;
	else
		wbc = NULL;
	/* WB GO bit has to be used only in case of
	 * capture mode and not in memory mode
	 */
	if (wbc && wbc->mode != OMAP_WB_MEM2MEM_MODE)
		dispc_go_wb();
}

static void mgr_clear_shadow_dirty(struct omap_overlay_manager *mgr)
{
	struct omap_overlay *ovl;
	struct mgr_priv_data *mp;
	struct ovl_priv_data *op;

	mp = get_mgr_priv(mgr);

	if (mp->shadow_info_dirty)
		dss_ovl_program_cb(&mp->cb, mgr->id);

	mp->shadow_info_dirty = false;

	list_for_each_entry(ovl, &mgr->overlays, list) {
		op = get_ovl_priv(ovl);
		if (op->shadow_info_dirty || op->shadow_extra_info_dirty) {
			dss_ovl_program_cb(&op->cb, ovl->id);
			op->dispc_channel = op->channel;
		}
		op->shadow_info_dirty = false;
		op->shadow_extra_info_dirty = false;
	}
}

static void schedule_completion_irq(void);

static void dss_completion_irq_handler(void *data, u32 mask)
{
	struct mgr_priv_data *mp;
	struct ovl_priv_data *op;
	struct omap_overlay_manager *mgr;
	struct omap_overlay *ovl;
	const int num_ovls = ARRAY_SIZE(dss_data.ovl_priv_data_array);
	const int num_mgrs = MAX_DSS_MANAGERS;
	const u32 masks[] = {
		DISPC_IRQ_FRAMEDONE | DISPC_IRQ_VSYNC,
		DISPC_IRQ_FRAMEDONETV | DISPC_IRQ_EVSYNC_EVEN |
		DISPC_IRQ_EVSYNC_ODD,
		DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2
	};
	int i;

	spin_lock(&data_lock);

	for (i = 0; i < num_mgrs; i++) {
		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);
		if (mask & masks[i]) {
			if (mgr && mgr->device)
				mgr->device->first_vsync = true;
			dss_ovl_cb(&mp->cb.dispc, i, DSS_COMPLETION_DISPLAYED);
			mp->cb.dispc_displayed = true;
		}
	}

	/* notify all overlays on that manager */
	for (i = 0; i < num_ovls; i++) {
		ovl = omap_dss_get_overlay(i);
		op = get_ovl_priv(ovl);
		if (mask & masks[op->channel]) {
			dss_ovl_cb(&op->cb.dispc, i, DSS_COMPLETION_DISPLAYED);
			op->cb.dispc_displayed = true;
		}
	}

	schedule_completion_irq();

	spin_unlock(&data_lock);
}

static void schedule_completion_irq(void)
{
	struct mgr_priv_data *mp;
	struct ovl_priv_data *op;
	const int num_ovls = ARRAY_SIZE(dss_data.ovl_priv_data_array);
	const int num_mgrs = MAX_DSS_MANAGERS;
	const u32 masks[] = {
		DISPC_IRQ_FRAMEDONE | DISPC_IRQ_VSYNC,
		DISPC_IRQ_FRAMEDONETV | DISPC_IRQ_EVSYNC_EVEN |
		DISPC_IRQ_EVSYNC_ODD,
		DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2
	};
	u32 mask = 0;
	int i;

	for (i = 0; i < num_mgrs; i++) {
		mp = &dss_data.mgr_priv_data_array[i];
		if (mp->cb.dispc.fn && (mp->cb.dispc.mask &
					DSS_COMPLETION_DISPLAYED))
			mask |= masks[i];
	}

	/* notify all overlays on that manager */
	for (i = 0; i < num_ovls; i++) {
		op = &dss_data.ovl_priv_data_array[i];
		if (op->cb.dispc.fn && op->enabled &&
				(op->cb.dispc.mask & DSS_COMPLETION_DISPLAYED))
			mask |= masks[op->channel];
	}

	if (mask != dss_data.comp_irq_enabled) {
		if (dss_data.comp_irq_enabled)
			omap_dispc_unregister_isr_nosync(
					dss_completion_irq_handler, NULL,
					dss_data.comp_irq_enabled);
		if (mask)
			omap_dispc_register_isr(dss_completion_irq_handler,
					NULL, mask);
		dss_data.comp_irq_enabled = mask;
	}
}

void dss_mgr_start_update(struct omap_overlay_manager *mgr)
{
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	unsigned long flags;
	int r;

	spin_lock_irqsave(&data_lock, flags);

	WARN_ON(mp->updating);

	r = dss_check_settings(mgr, mgr->device);
	if (r) {
		DSSERR("cannot start manual update: illegal configuration\n");
		spin_unlock_irqrestore(&data_lock, flags);
		return;
	}

	dss_mgr_write_regs(mgr);

	dss_write_regs_common();

	mp->updating = true;

	if (!dss_data.irq_enabled && need_isr())
		dss_register_vsync_isr();

	dispc_mgr_enable(mgr->id, true);

	/* for manually updated displays invoke dsscomp callbacks manually,
	 * as logic that relays on shadow_dirty flag can't correctly release
	 * previous composition
	*/
	dss_ovl_configure_cb(&mp->cb, mgr->id, true);
	dss_ovl_program_cb(&mp->cb, mgr->id);

	mgr_clear_shadow_dirty(mgr);
	spin_unlock_irqrestore(&data_lock, flags);
}

static void dss_apply_irq_handler(void *data, u32 mask);

static void dss_register_vsync_isr(void)
{
	const int num_mgrs = dss_feat_get_num_mgrs();
	u32 mask;
	int r, i;

	mask = 0;
	for (i = 0; i < num_mgrs; ++i)
		mask |= dispc_mgr_get_vsync_irq(i);

	for (i = 0; i < num_mgrs; ++i)
		mask |= dispc_mgr_get_framedone_irq(i);

	r = omap_dispc_register_isr(dss_apply_irq_handler, NULL, mask);
	WARN_ON(r);

	dss_data.irq_enabled = true;
}

static void dss_unregister_vsync_isr(void)
{
	const int num_mgrs = dss_feat_get_num_mgrs();
	u32 mask;
	int r, i;

	mask = 0;
	for (i = 0; i < num_mgrs; ++i)
		mask |= dispc_mgr_get_vsync_irq(i);

	for (i = 0; i < num_mgrs; ++i)
		mask |= dispc_mgr_get_framedone_irq(i);

	r = omap_dispc_unregister_isr_nosync(dss_apply_irq_handler, NULL, mask);
	WARN_ON(r);

	dss_data.irq_enabled = false;
}

static void dss_apply_irq_handler(void *data, u32 mask)
{
	const int num_mgrs = dss_feat_get_num_mgrs();
	int i;
	bool extra_updating;

	spin_lock(&data_lock);

	/* clear busy, updating flags, shadow_dirty flags */
	for (i = 0; i < num_mgrs; i++) {
		struct omap_overlay_manager *mgr;
		struct mgr_priv_data *mp;
		bool was_updating;

		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);

		if (!mp->enabled)
			continue;

		was_updating = mp->updating;
		mp->updating = dispc_mgr_is_enabled(i);

		if (!mgr_manual_update(mgr)) {
			bool was_busy = mp->busy;
			mp->busy = dispc_mgr_go_busy(i);

			if (was_busy && !mp->busy) {
				if (mgr && mgr->device)
					mgr->device->first_vsync = true;
				mgr_clear_shadow_dirty(mgr);
			}
		}
	}

	schedule_completion_irq();

	/* ACOS_MOD_BEGIN */
	TRAPZ_DESCRIBE(TRAPZ_KERN_DISP_DSS, DssIrqHandler,
		"dss_apply_irq_handler: "
		"Handle the VSYNC interrupt - acknowledge/clear CPU interrupt lines");
	TRAPZ_LOG(TRAPZ_LOG_DEBUG, 0, TRAPZ_KERN_DISP_DSS, DssIrqHandler,
		0, 0, 0, 0);
	/* ACOS_MOD_END */

	dss_write_regs();
	dss_set_go_bits();

	extra_updating = extra_info_update_ongoing();
	if (!extra_updating)
		complete_all(&extra_updated_completion);

	if (!need_isr())
		dss_unregister_vsync_isr();

	spin_unlock(&data_lock);
}

int dss_mgr_blank(struct omap_overlay_manager *mgr,
			bool wait_for_go)
{
	struct ovl_priv_data *op;
	struct mgr_priv_data *mp;
	unsigned long flags;
	int r, r_get, i;

	DSSDBG("dss_mgr_blank(%s,wait=%d)\n", mgr->name, wait_for_go);

	r = dispc_runtime_get();
	r_get = r;
	/* still clear cache even if failed to get clocks, just don't config */


	/* disable overlays in overlay user info structs and in data info */
	for (i = 0; i < omap_dss_get_num_overlays(); i++) {
		struct omap_overlay *ovl;

		ovl = omap_dss_get_overlay(i);

		if (ovl->manager != mgr)
			continue;

		r = ovl->disable(ovl);

		spin_lock_irqsave(&data_lock, flags);
		op = get_ovl_priv(ovl);

		/* complete unconfigured info */
		if (op->user_info_dirty)
			dss_ovl_cb(&op->user_info.cb, i,
					DSS_COMPLETION_ECLIPSED_SET);
		dss_ovl_cb(&op->cb.info, i, DSS_COMPLETION_ECLIPSED_CACHE);
		op->cb.info.fn = NULL;

		op->user_info_dirty = false;
		op->info_dirty = true;
		op->enabled = false;
		spin_unlock_irqrestore(&data_lock, flags);
	}

	spin_lock_irqsave(&data_lock, flags);
	/* dirty manager */
	mp = get_mgr_priv(mgr);
	if (mp->user_info_dirty)
		dss_ovl_cb(&mp->user_info.cb, mgr->id,
				DSS_COMPLETION_ECLIPSED_SET);
	dss_ovl_cb(&mp->cb.info, mgr->id, DSS_COMPLETION_ECLIPSED_CACHE);
	mp->cb.info.fn = NULL;
	mp->user_info.cb.fn = NULL;
	mp->info_dirty = true;
	mp->user_info_dirty = false;

	/*
	 * TRICKY: Enable apply irq even if not waiting for vsync, so that
	 * DISPC programming takes place in case GO bit was on.
	 */
	if (!dss_data.irq_enabled) {
		u32 mask;

		mask = DISPC_IRQ_VSYNC	| DISPC_IRQ_EVSYNC_ODD |
			DISPC_IRQ_EVSYNC_EVEN | DISPC_IRQ_FRAMEDONETV |
			DISPC_IRQ_FRAMEDONE | DISPC_IRQ_FRAMEDONE2;
		if (dss_has_feature(FEAT_MGR_LCD2))
			mask |= DISPC_IRQ_VSYNC2;

		r = omap_dispc_register_isr(dss_apply_irq_handler, NULL, mask);
		dss_data.irq_enabled = true;
	}

	if (!r_get) {
		dss_write_regs();
		dss_set_go_bits();
	}

	if (r_get || !wait_for_go) {
		/* pretend that programming has happened */
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			op = &dss_data.ovl_priv_data_array[i];
			if (op->channel != mgr->id)
				continue;
			if (op->info_dirty)
				dss_ovl_configure_cb(&op->cb, i, false);
			if (op->shadow_info_dirty) {
				dss_ovl_program_cb(&op->cb, i);
				op->dispc_channel = op->channel;
				op->shadow_info_dirty = false;
			} else {
				pr_warn("ovl%d-shadow is not dirty\n", i);
			}
		}

		if (mp->info_dirty)
			dss_ovl_configure_cb(&mp->cb, i, false);
		if (mp->shadow_info_dirty) {
			dss_ovl_program_cb(&mp->cb, i);
			mp->shadow_info_dirty = false;
		} else {
			pr_warn("mgr%d-shadow is not dirty\n", mgr->id);
		}
	}

	spin_unlock_irqrestore(&data_lock, flags);

	if (wait_for_go)
		mgr->wait_for_go(mgr);

	if (!r_get)
		dispc_runtime_put();

	return r;
}

int omap_dss_manager_unregister_callback(struct omap_overlay_manager *mgr,
					 struct omapdss_ovl_cb *cb)
{
	unsigned long flags;
	int r = 0;
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	spin_lock_irqsave(&data_lock, flags);
	if (mp->user_info_dirty &&
	    mp->user_info.cb.fn == cb->fn &&
	    mp->user_info.cb.data == cb->data)
		mp->user_info.cb.fn = NULL;
	else
		r = -EPERM;
	spin_unlock_irqrestore(&data_lock, flags);
	return r;
}

static void omap_dss_mgr_apply_ovl(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op;

	op = get_ovl_priv(ovl);

	if (!op->user_info_dirty)
		return;

	/* complete unconfigured info */
	dss_ovl_cb(&op->cb.info, ovl->id,
		   DSS_COMPLETION_ECLIPSED_CACHE);

	op->cb.info = op->user_info.cb;
	op->user_info.cb.fn = NULL;

	op->user_info_dirty = false;
	op->info_dirty = true;
	op->info = op->user_info;
}

static void omap_dss_mgr_apply_mgr(struct omap_overlay_manager *mgr)
{
	struct mgr_priv_data *mp;

	mp = get_mgr_priv(mgr);

	if (!mp->user_info_dirty)
		return;

	/* complete unconfigured info */
	dss_ovl_cb(&mp->cb.info, mgr->id,
		   DSS_COMPLETION_ECLIPSED_CACHE);

	mp->cb.info = mp->user_info.cb;
	mp->user_info.cb.fn = NULL;

	mp->user_info_dirty = false;
	mp->info_dirty = true;
	mp->info = mp->user_info;
	mp->skip_init = mgr->device->skip_init;
	DSSDBG("%s %d\n",__FUNCTION__,mp->skip_init);
}

void dss_tput_request(u32 tput)
{
	if(!pm_init_cstr) {
		pm_qos_add_request(&req, PM_QOS_MEMORY_THROUGHPUT,
					 tput);
		pm_init_cstr = 1;
	}
	else
		pm_qos_update_request(&req, tput);
}

int omap_dss_mgr_apply(struct omap_overlay_manager *mgr)
{
	unsigned long flags;
	struct omap_overlay *ovl;
	struct omap_overlay_manager_info info;
	int r;

	DSSDBG("omap_dss_mgr_apply(%s)\n", mgr->name);

	mgr->get_manager_info(mgr, &info);

	/* Set OPP constraint on CORE only when it needed */
	if (mgr->device && (mgr->device->state == OMAP_DSS_DISPLAY_ACTIVE))
		if (omap_dss_overlay_ensure_bw())
			dss_tput_request(PM_QOS_MEMORY_THROUGHPUT_HIGH_VALUE);

	spin_lock_irqsave(&data_lock, flags);

	if (!mgr->device) {
		pr_info_ratelimited("cannot aply mgr(%s)--invalid device\n",
				mgr->name);
		r = -ENODEV;
		goto done;
	}

	if (!info.wb_only) {
		r = dss_check_settings_apply(mgr, mgr->device);
		if (r) {
			DSSERR("failed to apply: illegal configuration.\n");
			goto done;
		}
	}

	/* Configure overlays */
	list_for_each_entry(ovl, &mgr->overlays, list)
		omap_dss_mgr_apply_ovl(ovl);

	if (mgr->device->state != OMAP_DSS_DISPLAY_ACTIVE && !info.wb_only) {
		struct writeback_cache_data *wbc;

		if (dss_has_feature(FEAT_WB))
			wbc = &dss_data.writeback_cache;
		else
			wbc = NULL;

		/* in case, if WB was configured with MEM2MEM with manager
		 * mode, but manager, which is source for WB, is not marked as
		 * wb_only, then skip apply operation. We have such case, when
		 * composition was sent to disable pipes, which are sources for
		 * WB.
		 */
		if (wbc && wbc->mode == OMAP_WB_MEM2MEM_MODE &&
			(int)wbc->source == (int)mgr->id && mgr->device &&
			mgr->device->state != OMAP_DSS_DISPLAY_ACTIVE) {
			r = 0;
			goto done;
		}

		pr_info_ratelimited("cannot apply mgr(%s) on inactive device\n",
				mgr->name);
		r = -ENODEV;
		goto done;
	}

	/* Configure manager */
	omap_dss_mgr_apply_mgr(mgr);
done:
	spin_unlock_irqrestore(&data_lock, flags);

	return r;
}

int omap_dss_wb_mgr_apply(struct omap_overlay_manager *mgr,
		struct omap_writeback *wb)
{
	struct writeback_cache_data *wbc;
	unsigned long flags;

	DSSDBG("omap_dss_wb_mgr_apply(%s)\n", mgr->name);
	if (!wb) {
		printk(KERN_ERR "[%s][%d] No WB!\n", __FILE__, __LINE__);
		return -EINVAL;
	}

	/* skip composition, if manager is enabled. It happens when HDMI/TV
	 * physical layer is activated in the time, when MEM2MEM with manager
	 * mode is used.
	 */
	if (wb->info.source == OMAP_WB_TV &&
			dispc_mgr_is_enabled(OMAP_DSS_CHANNEL_DIGIT) &&
				wb->info.mode == OMAP_WB_MEM2MEM_MODE) {
		DSSERR("manager %d busy, dropping\n", mgr->id);
		return -EBUSY;
	}

	spin_lock_irqsave(&data_lock, flags);
	wbc = &dss_data.writeback_cache;

	if (wb && wb->info.enabled) {
		/* if source is an overlay, mode cannot be capture */
		if ((wb->info.source >= OMAP_WB_GFX) &&
			(wb->info.mode != OMAP_WB_MEM2MEM_MODE))
			return -EINVAL;
		wbc->enabled = true;
		wbc->mode = wb->info.mode;
		wbc->color_mode = wb->info.dss_mode;
		wbc->out_width = wb->info.out_width;
		wbc->out_height = wb->info.out_height;
		wbc->width = wb->info.width;
		wbc->height = wb->info.height;

		wbc->paddr = wb->info.paddr;
		wbc->p_uv_addr = wb->info.p_uv_addr;

		wbc->capturemode = wb->info.capturemode;
		wbc->burst_size = BURST_SIZE_X8;

		/*
		 * only these FIFO values work in WB capture mode for all
		 * downscale scenarios. Other FIFO values cause a SYNC_LOST
		 * on LCD due to b/w issues.
		 */
		wbc->fifo_high = 0x10;
		wbc->fifo_low = 0x8;
		wbc->source = wb->info.source;
		wbc->force_1d = wb->info.force_1d;

		wbc->rotation = wb->info.rotation;
		wbc->rotation_type = wb->info.rotation_type;

		wbc->dirty = true;
		wbc->shadow_dirty = false;
	} else if (wb && (wbc->enabled != wb->info.enabled)) {
		/* disable WB if not disabled already*/
		wbc->enabled = wb->info.enabled;
		wbc->dirty = true;
		wbc->shadow_dirty = false;
	}

	dss_wb_write_regs();
	dss_wb_ovl_enable();
	dss_wb_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static void seq_print_cb(struct seq_file *s, struct omapdss_ovl_cb *cb)
{
	if (!cb->fn) {
		seq_printf(s, "(none)\n");
		return;
	}

	seq_printf(s, "mask=%c%c%c%c [%p] %pf\n",
		   (cb->mask & DSS_COMPLETION_CHANGED) ? 'C' : '-',
		   (cb->mask & DSS_COMPLETION_PROGRAMMED) ? 'P' : '-',
		   (cb->mask & DSS_COMPLETION_DISPLAYED) ? 'D' : '-',
		   (cb->mask & DSS_COMPLETION_RELEASED) ? 'R' : '-',
		   cb->data,
		   cb->fn);
}
#endif

void seq_print_cbs(struct omap_overlay_manager *mgr, struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS
	struct mgr_priv_data *mp;
	unsigned long flags;

	spin_lock_irqsave(&data_lock, flags);

	mp = get_mgr_priv(mgr);

	seq_printf(s, "  DISPC pipeline:\n\n"
				"    user_info:%13s ", mp->user_info_dirty ?
				"DIRTY" : "clean");
	seq_print_cb(s, &mp->user_info.cb);
	seq_printf(s, "    info:%12s ", mp->info_dirty ? "DIRTY" : "clean");
	seq_print_cb(s, &mp->cb.info);
	seq_printf(s, "    shadow:  %s %s ", mp->cb.shadow_enabled ? "ACT" :
				"off", mp->shadow_info_dirty ?
				"DIRTY" : "clean");
	seq_print_cb(s, &mp->cb.shadow);
	seq_printf(s, "    dispc:%12s ", mp->cb.dispc_displayed ?
				"DISPLAYED" : "");
	seq_print_cb(s, &mp->cb.dispc);
	seq_printf(s, "\n");

	spin_unlock_irqrestore(&data_lock, flags);
#endif
}

static void dss_apply_ovl_enable(struct omap_overlay *ovl, bool enable)
{
	struct ovl_priv_data *op;

	op = get_ovl_priv(ovl);

	if (op->enabled == enable)
		return;

	op->enabled = enable;
	op->extra_info_dirty = true;
}

static void dss_apply_ovl_fifo_thresholds(struct omap_overlay *ovl,
		u32 fifo_low, u32 fifo_high)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);

	if (op->fifo_low == fifo_low && op->fifo_high == fifo_high)
		return;

	op->fifo_low = fifo_low;
	op->fifo_high = fifo_high;
	op->extra_info_dirty = true;
}

static void dss_apply_fifo_merge(bool use_fifo_merge)
{
	if (dss_data.fifo_merge == use_fifo_merge)
		return;

	dss_data.fifo_merge = use_fifo_merge;
	dss_data.fifo_merge_dirty = true;
}

static void dss_ovl_setup_fifo(struct omap_overlay *ovl,
		bool use_fifo_merge)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	struct omap_dss_device *dssdev;
	u32 fifo_low, fifo_high;

	if (!op->enabled && !op->enabling)
		return;

	dssdev = ovl->manager->device;

	dispc_ovl_compute_fifo_thresholds(ovl->id, &fifo_low, &fifo_high,
			use_fifo_merge, ovl_manual_update(ovl));

	dss_apply_ovl_fifo_thresholds(ovl, fifo_low, fifo_high);
}

static void dss_mgr_setup_fifos(struct omap_overlay_manager *mgr,
		bool use_fifo_merge)
{
	struct omap_overlay *ovl;
	struct mgr_priv_data *mp;

	mp = get_mgr_priv(mgr);

	if (!mp->enabled)
		return;

	list_for_each_entry(ovl, &mgr->overlays, list)
		dss_ovl_setup_fifo(ovl, use_fifo_merge);
}

static void dss_setup_fifos(bool use_fifo_merge)
{
	const int num_mgrs = omap_dss_get_num_overlay_managers();
	struct omap_overlay_manager *mgr;
	int i;

	for (i = 0; i < num_mgrs; ++i) {
		mgr = omap_dss_get_overlay_manager(i);
		dss_mgr_setup_fifos(mgr, use_fifo_merge);
	}
}

static int get_num_used_managers(void)
{
	const int num_mgrs = omap_dss_get_num_overlay_managers();
	struct omap_overlay_manager *mgr;
	struct mgr_priv_data *mp;
	int i;
	int enabled_mgrs;

	enabled_mgrs = 0;

	for (i = 0; i < num_mgrs; ++i) {
		mgr = omap_dss_get_overlay_manager(i);
		mp = get_mgr_priv(mgr);

		if (!mp->enabled)
			continue;

		enabled_mgrs++;
	}

	return enabled_mgrs;
}

static int get_num_used_overlays(void)
{
	const int num_ovls = omap_dss_get_num_overlays();
	struct omap_overlay *ovl;
	struct ovl_priv_data *op;
	struct mgr_priv_data *mp;
	int i;
	int enabled_ovls;

	enabled_ovls = 0;

	for (i = 0; i < num_ovls; ++i) {
		ovl = omap_dss_get_overlay(i);
		op = get_ovl_priv(ovl);

		if (!op->enabled && !op->enabling)
			continue;

		mp = get_mgr_priv(ovl->manager);

		if (!mp->enabled)
			continue;

		enabled_ovls++;
	}

	return enabled_ovls;
}

static bool get_use_fifo_merge(void)
{
	int enabled_mgrs = get_num_used_managers();
	int enabled_ovls = get_num_used_overlays();

	if (!dss_has_feature(FEAT_FIFO_MERGE))
		return false;

	/*
	 * In theory the only requirement for fifomerge is enabled_ovls <= 1.
	 * However, if we have two managers enabled and set/unset the fifomerge,
	 * we need to set the GO bits in particular sequence for the managers,
	 * and wait in between.
	 *
	 * This is rather difficult as new apply calls can happen at any time,
	 * so we simplify the problem by requiring also that enabled_mgrs <= 1.
	 * In practice this shouldn't matter, because when only one overlay is
	 * enabled, most likely only one output is enabled.
	 */

	return enabled_mgrs <= 1 && enabled_ovls <= 1;
}

int dss_mgr_enable(struct omap_overlay_manager *mgr)
{
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	unsigned long flags;
	int r;
	bool fifo_merge;

	mutex_lock(&apply_lock);

	if (mp->enabled)
		goto out;

	spin_lock_irqsave(&data_lock, flags);

	mp->enabled = true;

	r = dss_check_settings(mgr, mgr->device);
	if (r) {
		DSSERR("failed to enable manager %d: check_settings failed\n",
				mgr->id);
		goto err;
	}

	/* step 1: setup fifos/fifomerge before enabling the manager */

	fifo_merge = get_use_fifo_merge();
	dss_setup_fifos(fifo_merge);
	dss_apply_fifo_merge(fifo_merge);

	dss_write_regs();
	dss_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);

	/* wait until fifo config is in */
	wait_pending_extra_info_updates();

	/* step 2: enable the manager */
	spin_lock_irqsave(&data_lock, flags);

	if (!mgr_manual_update(mgr))
		mp->updating = true;

	spin_unlock_irqrestore(&data_lock, flags);

	if (!mgr_manual_update(mgr))
		dispc_mgr_enable(mgr->id, true);

out:
	mutex_unlock(&apply_lock);

	return 0;

err:
	mp->enabled = false;
	spin_unlock_irqrestore(&data_lock, flags);
	mutex_unlock(&apply_lock);
	return r;
}

void dss_mgr_disable(struct omap_overlay_manager *mgr)
{
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	unsigned long flags;
	bool fifo_merge;

	mutex_lock(&apply_lock);

	if (!mp->enabled)
		goto out;

	if (!mgr_manual_update(mgr))
		dispc_mgr_enable(mgr->id, false);

	spin_lock_irqsave(&data_lock, flags);

	mp->updating = false;
	mp->enabled = false;

	fifo_merge = get_use_fifo_merge();
	dss_setup_fifos(fifo_merge);
	dss_apply_fifo_merge(fifo_merge);

	dss_write_regs();
	dss_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);

	wait_pending_extra_info_updates();
out:
	mutex_unlock(&apply_lock);
}

int dss_mgr_set_info(struct omap_overlay_manager *mgr,
		struct omap_overlay_manager_info *info)
{
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	unsigned long flags;
	int r;

	r = dss_mgr_simple_check(mgr, info);
	if (r)
		return r;

	spin_lock_irqsave(&data_lock, flags);

	mp->user_info = *info;
	mp->user_info_dirty = true;

	spin_unlock_irqrestore(&data_lock, flags);

	return 0;
}

void dss_mgr_get_info(struct omap_overlay_manager *mgr,
		struct omap_overlay_manager_info *info)
{
	struct mgr_priv_data *mp = get_mgr_priv(mgr);
	unsigned long flags;

	spin_lock_irqsave(&data_lock, flags);

	*info = mp->user_info;

	spin_unlock_irqrestore(&data_lock, flags);
}

int dss_mgr_set_device(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev)
{
	int r;

	mutex_lock(&apply_lock);

	if (dssdev->manager) {
		DSSERR("display '%s' already has a manager '%s'\n",
			       dssdev->name, dssdev->manager->name);
		r = -EINVAL;
		goto err;
	}

	if ((mgr->supported_displays & dssdev->type) == 0) {
		DSSERR("display '%s' does not support manager '%s'\n",
			       dssdev->name, mgr->name);
		r = -EINVAL;
		goto err;
	}

	dssdev->manager = mgr;
	mgr->device = dssdev;

	mutex_unlock(&apply_lock);

	return 0;
err:
	mutex_unlock(&apply_lock);
	return r;
}

int dss_mgr_unset_device(struct omap_overlay_manager *mgr)
{
	int r;

	mutex_lock(&apply_lock);

	if (!mgr->device) {
		DSSERR("failed to unset display, display not set.\n");
		r = -EINVAL;
		goto err;
	}

	/*
	 * Don't allow currently enabled displays to have the overlay manager
	 * pulled out from underneath them
	 */
	if (mgr->device->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	mgr->device->manager = NULL;
	mgr->device = NULL;

	mutex_unlock(&apply_lock);

	return 0;
err:
	mutex_unlock(&apply_lock);
	return r;
}


int dss_ovl_set_info(struct omap_overlay *ovl,
		struct omap_overlay_info *info)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;
	int r;

	r = dss_ovl_simple_check(ovl, info);
	if (r)
		return r;

	spin_lock_irqsave(&data_lock, flags);

	op->user_info = *info;
	op->user_info_dirty = true;

	spin_unlock_irqrestore(&data_lock, flags);

	return 0;
}

void dss_ovl_get_info(struct omap_overlay *ovl,
		struct omap_overlay_info *info)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;

	spin_lock_irqsave(&data_lock, flags);

	*info = op->user_info;

	spin_unlock_irqrestore(&data_lock, flags);
}

int dss_ovl_set_manager(struct omap_overlay *ovl,
		struct omap_overlay_manager *mgr)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;
	int r;

	if (!mgr)
		return -EINVAL;

	mutex_lock(&apply_lock);

	if (ovl->manager) {
		DSSERR("overlay '%s' already has a manager '%s'\n",
				ovl->name, ovl->manager->name);
		r = -EINVAL;
		goto err;
	}

	spin_lock_irqsave(&data_lock, flags);

	if (op->enabled) {
		spin_unlock_irqrestore(&data_lock, flags);
		DSSERR("overlay has to be disabled to change the manager\n");
		r = -EINVAL;
		goto err;
	}

	op->channel = mgr->id;
	op->extra_info_dirty = true;

	ovl->manager = mgr;
	list_add_tail(&ovl->list, &mgr->overlays);

	spin_unlock_irqrestore(&data_lock, flags);

	/* XXX: When there is an overlay on a DSI manual update display, and
	 * the overlay is first disabled, then moved to tv, and enabled, we
	 * seem to get SYNC_LOST_DIGIT error.
	 *
	 * Waiting doesn't seem to help, but updating the manual update display
	 * after disabling the overlay seems to fix this. This hints that the
	 * overlay is perhaps somehow tied to the LCD output until the output
	 * is updated.
	 *
	 * Userspace workaround for this is to update the LCD after disabling
	 * the overlay, but before moving the overlay to TV.
	 */

	mutex_unlock(&apply_lock);

	return 0;
err:
	mutex_unlock(&apply_lock);
	return r;
}

int dss_ovl_unset_manager(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;
	int r;

	mutex_lock(&apply_lock);

	if (!ovl->manager) {
		DSSERR("failed to detach overlay: manager not set\n");
		r = -EINVAL;
		goto err;
	}

	spin_lock_irqsave(&data_lock, flags);

	if (op->enabled) {
		spin_unlock_irqrestore(&data_lock, flags);
		DSSERR("overlay has to be disabled to unset the manager\n");
		r = -EINVAL;
		goto err;
	}

	op->channel = -1;

	ovl->manager = NULL;
	list_del(&ovl->list);

	spin_unlock_irqrestore(&data_lock, flags);

	mutex_unlock(&apply_lock);

	return 0;
err:
	mutex_unlock(&apply_lock);
	return r;
}

bool dss_ovl_is_enabled(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;
	bool e;

	spin_lock_irqsave(&data_lock, flags);

	e = op->enabled;

	spin_unlock_irqrestore(&data_lock, flags);

	return e;
}

int dss_mgr_set_ovls(struct omap_overlay_manager *mgr)
{
	unsigned long flags;
	int i;
	if (!mgr || !mgr->ovls) {
		DSSERR("null pointer\n");
		return -EINVAL;
	}
	if (mgr->num_ovls > dss_feat_get_num_ovls()) {
		DSSERR("Invalid number of overlays passed\n");
		return -EINVAL;
	}
	mutex_lock(&apply_lock);
	spin_lock_irqsave(&data_lock, flags);

	for (i = 0; i < mgr->num_ovls; i++) {
		if (mgr != mgr->ovls[i]->manager) {
			DSSERR("Invalid mgr for ovl#%d\n", mgr->ovls[i]->id);
			spin_unlock_irqrestore(&data_lock, flags);
			mutex_unlock(&apply_lock);
			return -EINVAL;
		}
		/* Enable the overlay */
		if (mgr->ovls[i]->enabled) {
			struct ovl_priv_data *op = get_ovl_priv(mgr->ovls[i]);
			op->enabling = true;
			dss_setup_fifos(false);
			dss_apply_fifo_merge(false);
			op->enabling = false;
			dss_apply_ovl_enable(mgr->ovls[i], true);
		} else {
			dss_apply_ovl_enable(mgr->ovls[i], false);
		}
	}
	dss_write_regs();
	dss_set_go_bits();
	spin_unlock_irqrestore(&data_lock, flags);
	wait_pending_extra_info_updates();
	mutex_unlock(&apply_lock);
	return 0;
}

int dss_ovl_enable(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;
	bool fifo_merge;
	int r;

	mutex_lock(&apply_lock);

	if (op->enabled) {
		r = 0;
		goto err1;
	}

	if (ovl->manager == NULL || ovl->manager->device == NULL) {
		r = -EINVAL;
		goto err1;
	}

	spin_lock_irqsave(&data_lock, flags);

	op->enabling = true;

	r = dss_check_settings(ovl->manager, ovl->manager->device);
	if (r) {
		DSSERR("failed to enable overlay %d: check_settings failed\n",
				ovl->id);
		goto err2;
	}

	/* step 1: configure fifos/fifomerge for currently enabled ovls */

	fifo_merge = get_use_fifo_merge();
	dss_setup_fifos(fifo_merge);
	dss_apply_fifo_merge(fifo_merge);

	dss_write_regs();
	dss_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);

	/* wait for fifo configs to go in */
	wait_pending_extra_info_updates();

	/* step 2: enable the overlay */
	spin_lock_irqsave(&data_lock, flags);

	op->enabling = false;
	dss_apply_ovl_enable(ovl, true);

	dss_write_regs();
	dss_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);

	/* wait for overlay to be enabled */
	wait_pending_extra_info_updates();

	mutex_unlock(&apply_lock);

	return 0;
err2:
	op->enabling = false;
	spin_unlock_irqrestore(&data_lock, flags);
err1:
	mutex_unlock(&apply_lock);
	return r;
}

int dss_ovl_disable(struct omap_overlay *ovl)
{
	struct ovl_priv_data *op = get_ovl_priv(ovl);
	unsigned long flags;
	bool fifo_merge;
	int r;

	mutex_lock(&apply_lock);

	if (!op->enabled) {
		r = 0;
		goto err;
	}

	if (ovl->manager == NULL || ovl->manager->device == NULL) {
		r = -EINVAL;
		goto err;
	}

	/* step 1: disable the overlay */
	spin_lock_irqsave(&data_lock, flags);

	dss_apply_ovl_enable(ovl, false);

	dss_write_regs();
	dss_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);

	/* wait for the overlay to be disabled */
	wait_pending_extra_info_updates();

	/* step 2: configure fifos/fifomerge */
	spin_lock_irqsave(&data_lock, flags);

	fifo_merge = get_use_fifo_merge();
	dss_setup_fifos(fifo_merge);
	dss_apply_fifo_merge(fifo_merge);

	dss_write_regs();
	dss_set_go_bits();

	spin_unlock_irqrestore(&data_lock, flags);

	/* wait for fifo config to go in */
	wait_pending_extra_info_updates();

	mutex_unlock(&apply_lock);

	return 0;

err:
	mutex_unlock(&apply_lock);
	return r;
}

bool omap_dss_overlay_ensure_bw(void)
{
	int i;
	struct omap_overlay *ovl;
	int num_planes_enabled = 0;
	bool high_res_screen = false;
	struct ovl_priv_data *op;
	struct omap_overlay_info *oi;

	/* Check if DSS need higher OPP on CORE or not */
	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {

		ovl = omap_dss_get_overlay(i);
		op = get_ovl_priv(ovl);
		oi = &op->info;

		if (!op->enabled)
			continue;

		/* Check for high resolution screes, 1080p */
		if ((oi->width * oi->height) >= OVERLAY_AREA_BW_THRESHOLD)
			high_res_screen = true;

		++num_planes_enabled;
	}

	if ((num_planes_enabled > 1) && high_res_screen) {
		if (!band_changed)
			band_changed = true;
		return true;
	} else if (band_changed)
		band_changed = false;

	return false;
}
EXPORT_SYMBOL(omap_dss_overlay_ensure_bw);
