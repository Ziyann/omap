/*
 * linux/drivers/video/omap2/dss/dispc.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#define DSS_SUBSYS_NAME "DISPC"

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/export.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>

#include <plat/clock.h>
#include "../../../drivers/staging/omapdrm/omap_dmm_tiler.h"
#include <mach-omap2/clockdomain.h>

#include <video/omapdss.h>

#include "dss.h"
#include "dss_features.h"
#include "dispc.h"

/* DISPC */
#define DISPC_SZ_REGS			SZ_4K

#define DISPC_IRQ_MASK_ERROR            (DISPC_IRQ_GFX_FIFO_UNDERFLOW | \
					 DISPC_IRQ_OCP_ERR | \
					 DISPC_IRQ_VID1_FIFO_UNDERFLOW | \
					 DISPC_IRQ_VID2_FIFO_UNDERFLOW | \
					 DISPC_IRQ_SYNC_LOST | \
					 DISPC_IRQ_SYNC_LOST_DIGIT)

#define DISPC_MAX_NR_ISRS		8

static struct clockdomain *l3_1_clkdm, *l3_2_clkdm;

struct omap_dispc_isr_data {
	omap_dispc_isr_t	isr;
	void			*arg;
	u32			mask;
};

#define REG_GET(idx, start, end) \
	FLD_GET(dispc_read_reg(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end)				\
	dispc_write_reg(idx, FLD_MOD(dispc_read_reg(idx), val, start, end))

struct dispc_irq_stats {
	unsigned long last_reset;
	unsigned irq_count;
	unsigned irqs[32];
};

static struct {
	struct platform_device *pdev;
	void __iomem    *base;

	int		ctx_loss_cnt;

	int irq;
	struct clk *dss_clk;

	u32	fifo_size[MAX_DSS_OVERLAYS];

	spinlock_t irq_lock;
	u32 irq_error_mask;
	struct omap_dispc_isr_data registered_isr[DISPC_MAX_NR_ISRS];
	u32 error_irqs;
	struct work_struct error_work;

	bool		ctx_valid;
	u32		ctx[DISPC_SZ_REGS / sizeof(u32)];

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spinlock_t irq_stats_lock;
	struct dispc_irq_stats irq_stats;
#endif
} dispc;

enum omap_color_component {
	/* used for all color formats for OMAP3 and earlier
	 * and for RGB and Y color component on OMAP4
	 */
	DISPC_COLOR_COMPONENT_RGB_Y		= 1 << 0,
	/* used for UV component for
	 * OMAP_DSS_COLOR_YUV2, OMAP_DSS_COLOR_UYVY, OMAP_DSS_COLOR_NV12
	 * color formats on OMAP4
	 */
	DISPC_COLOR_COMPONENT_UV		= 1 << 1,
};

static void _omap_dispc_set_irqs(void);

static inline void dispc_write_reg(const u16 idx, u32 val)
{
	__raw_writel(val, dispc.base + idx);
}

static inline u32 dispc_read_reg(const u16 idx)
{
	return __raw_readl(dispc.base + idx);
}

#define SR(reg) \
	dispc.ctx[DISPC_##reg / sizeof(u32)] = dispc_read_reg(DISPC_##reg)
#define RR(reg) \
	dispc_write_reg(DISPC_##reg, dispc.ctx[DISPC_##reg / sizeof(u32)])

static void dispc_save_context(void)
{
	int i, j;

	DSSDBG("dispc_save_context\n");

	SR(IRQENABLE);
	SR(CONTROL);
	SR(CONFIG);
	SR(LINE_NUMBER);
	if (dss_has_feature(FEAT_ALPHA_FIXED_ZORDER) ||
			dss_has_feature(FEAT_ALPHA_FREE_ZORDER))
		SR(GLOBAL_ALPHA);
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		SR(CONTROL2);
		SR(CONFIG2);
	}

	for (i = 0; i < dss_feat_get_num_mgrs(); i++) {
		SR(DEFAULT_COLOR(i));
		SR(TRANS_COLOR(i));
		SR(SIZE_MGR(i));
		if (i == OMAP_DSS_CHANNEL_DIGIT)
			continue;
		SR(TIMING_H(i));
		SR(TIMING_V(i));
		SR(POL_FREQ(i));
		SR(DIVISORo(i));

		SR(DATA_CYCLE1(i));
		SR(DATA_CYCLE2(i));
		SR(DATA_CYCLE3(i));

		if (dss_has_feature(FEAT_CPR)) {
			SR(CPR_COEF_R(i));
			SR(CPR_COEF_G(i));
			SR(CPR_COEF_B(i));
		}
	}

	if (dss_has_feature(FEAT_MFLAG))
		SR(GLOBAL_MFLAG);


	for (i = 0; i < dss_feat_get_num_ovls(); i++) {
		SR(OVL_BA0(i));
		SR(OVL_BA1(i));
		SR(OVL_POSITION(i));
		SR(OVL_SIZE(i));
		SR(OVL_ATTRIBUTES(i));
		SR(OVL_FIFO_THRESHOLD(i));
		SR(OVL_ROW_INC(i));
		SR(OVL_PIXEL_INC(i));
		if (dss_has_feature(FEAT_PRELOAD))
			SR(OVL_PRELOAD(i));
		if (i == OMAP_DSS_GFX) {
			SR(OVL_WINDOW_SKIP(i));
			SR(OVL_TABLE_BA(i));
		if (dss_has_feature(FEAT_MFLAG))
			SR(OVL_MFLAG_THRESHOLD(i));
			continue;
		}
		SR(OVL_FIR(i));
		SR(OVL_PICTURE_SIZE(i));
		SR(OVL_ACCU0(i));
		SR(OVL_ACCU1(i));

		for (j = 0; j < 8; j++)
			SR(OVL_FIR_COEF_H(i, j));

		for (j = 0; j < 8; j++)
			SR(OVL_FIR_COEF_HV(i, j));

		for (j = 0; j < 5; j++)
			SR(OVL_CONV_COEF(i, j));

		if (dss_has_feature(FEAT_FIR_COEF_V)) {
			for (j = 0; j < 8; j++)
				SR(OVL_FIR_COEF_V(i, j));
		}

		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			SR(OVL_BA0_UV(i));
			SR(OVL_BA1_UV(i));
			SR(OVL_FIR2(i));
			SR(OVL_ACCU2_0(i));
			SR(OVL_ACCU2_1(i));

			for (j = 0; j < 8; j++)
				SR(OVL_FIR_COEF_H2(i, j));

			for (j = 0; j < 8; j++)
				SR(OVL_FIR_COEF_HV2(i, j));

			for (j = 0; j < 8; j++)
				SR(OVL_FIR_COEF_V2(i, j));
		}
		if (dss_has_feature(FEAT_ATTR2))
			SR(OVL_ATTRIBUTES2(i));
	}

	if (dss_has_feature(FEAT_CORE_CLK_DIV))
		SR(DIVISOR);

	dispc.ctx_loss_cnt = dss_get_ctx_loss_count(&dispc.pdev->dev);
	dispc.ctx_valid = true;

	DSSDBG("context saved, ctx_loss_count %d\n", dispc.ctx_loss_cnt);
}

static void dispc_restore_context(void)
{
	int i, j, ctx;

	DSSDBG("dispc_restore_context\n");

	if (!dispc.ctx_valid)
		return;

	ctx = dss_get_ctx_loss_count(&dispc.pdev->dev);

	if (ctx >= 0 && ctx == dispc.ctx_loss_cnt)
		return;

	DSSDBG("ctx_loss_count: saved %d, current %d\n",
			dispc.ctx_loss_cnt, ctx);

	/*RR(IRQENABLE);*/
	/*RR(CONTROL);*/
	RR(CONFIG);
	RR(LINE_NUMBER);
	if (dss_has_feature(FEAT_ALPHA_FIXED_ZORDER) ||
			dss_has_feature(FEAT_ALPHA_FREE_ZORDER))
		RR(GLOBAL_ALPHA);
	if (dss_has_feature(FEAT_MGR_LCD2))
		RR(CONFIG2);

	if (dss_has_feature(FEAT_MFLAG))
		RR(GLOBAL_MFLAG);

	for (i = 0; i < dss_feat_get_num_mgrs(); i++) {
		RR(DEFAULT_COLOR(i));
		RR(TRANS_COLOR(i));
		RR(SIZE_MGR(i));
		if (i == OMAP_DSS_CHANNEL_DIGIT)
			continue;
		RR(TIMING_H(i));
		RR(TIMING_V(i));
		RR(POL_FREQ(i));
		RR(DIVISORo(i));

		RR(DATA_CYCLE1(i));
		RR(DATA_CYCLE2(i));
		RR(DATA_CYCLE3(i));

		if (dss_has_feature(FEAT_CPR)) {
			RR(CPR_COEF_R(i));
			RR(CPR_COEF_G(i));
			RR(CPR_COEF_B(i));
		}
	}

	for (i = 0; i < dss_feat_get_num_ovls(); i++) {
		RR(OVL_BA0(i));
		RR(OVL_BA1(i));
		RR(OVL_POSITION(i));
		RR(OVL_SIZE(i));
		RR(OVL_ATTRIBUTES(i));
		RR(OVL_FIFO_THRESHOLD(i));
		RR(OVL_ROW_INC(i));
		RR(OVL_PIXEL_INC(i));
		if (dss_has_feature(FEAT_PRELOAD))
			RR(OVL_PRELOAD(i));
		if (i == OMAP_DSS_GFX) {
			RR(OVL_WINDOW_SKIP(i));
			RR(OVL_TABLE_BA(i));
			continue;
		}
		RR(OVL_FIR(i));
		RR(OVL_PICTURE_SIZE(i));
		RR(OVL_ACCU0(i));
		RR(OVL_ACCU1(i));

		for (j = 0; j < 8; j++)
			RR(OVL_FIR_COEF_H(i, j));

		for (j = 0; j < 8; j++)
			RR(OVL_FIR_COEF_HV(i, j));

		for (j = 0; j < 5; j++)
			RR(OVL_CONV_COEF(i, j));

		if (dss_has_feature(FEAT_FIR_COEF_V)) {
			for (j = 0; j < 8; j++)
				RR(OVL_FIR_COEF_V(i, j));
		}

		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			RR(OVL_BA0_UV(i));
			RR(OVL_BA1_UV(i));
			RR(OVL_FIR2(i));
			RR(OVL_ACCU2_0(i));
			RR(OVL_ACCU2_1(i));

			for (j = 0; j < 8; j++)
				RR(OVL_FIR_COEF_H2(i, j));

			for (j = 0; j < 8; j++)
				RR(OVL_FIR_COEF_HV2(i, j));

			for (j = 0; j < 8; j++)
				RR(OVL_FIR_COEF_V2(i, j));
		}
		if (dss_has_feature(FEAT_ATTR2))
			RR(OVL_ATTRIBUTES2(i));
		if (dss_has_feature(FEAT_MFLAG))
			RR(OVL_MFLAG_THRESHOLD(i));
	}

	if (dss_has_feature(FEAT_CORE_CLK_DIV))
		RR(DIVISOR);

	/* enable last, because LCD & DIGIT enable are here */
	RR(CONTROL);
	if (dss_has_feature(FEAT_MGR_LCD2))
		RR(CONTROL2);
	/* clear spurious SYNC_LOST_DIGIT interrupts */
	dispc_write_reg(DISPC_IRQSTATUS, DISPC_IRQ_SYNC_LOST_DIGIT);

	/*
	 * enable last so IRQs won't trigger before
	 * the context is fully restored
	 */
	RR(IRQENABLE);

	DSSDBG("context restored\n");
}

#undef SR
#undef RR

int dispc_runtime_get(void)
{
	int r;

	DSSDBG("dispc_runtime_get\n");

	r = pm_runtime_get_sync(&dispc.pdev->dev);
	WARN_ON(r < 0);
	return r < 0 ? r : 0;
}

void dispc_runtime_put(void)
{
	int r;

	DSSDBG("dispc_runtime_put\n");

	r = pm_runtime_put_sync(&dispc.pdev->dev);
	WARN_ON(r < 0);
}

static inline bool dispc_mgr_is_lcd(enum omap_channel channel)
{
	if (channel == OMAP_DSS_CHANNEL_LCD ||
			channel == OMAP_DSS_CHANNEL_LCD2)
		return true;
	else
		return false;
}

static struct omap_dss_device *dispc_mgr_get_device(enum omap_channel channel)
{
	struct omap_overlay_manager *mgr =
		omap_dss_get_overlay_manager(channel);

	return mgr ? mgr->device : NULL;
}

u32 dispc_mgr_get_vsync_irq(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_IRQ_VSYNC;
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_IRQ_VSYNC2;
	case OMAP_DSS_CHANNEL_DIGIT:
		return DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_EVSYNC_EVEN |
			DISPC_IRQ_FRAMEDONETV;
	default:
		BUG();
	}
}

u32 dispc_mgr_get_framedone_irq(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_IRQ_FRAMEDONE;
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_IRQ_FRAMEDONE2;
	case OMAP_DSS_CHANNEL_DIGIT:
		return 0;
	default:
		BUG();
	}
}

bool dispc_mgr_go_busy(enum omap_channel channel)
{
	int bit;

	if (dispc_mgr_is_lcd(channel))
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		return REG_GET(DISPC_CONTROL2, bit, bit) == 1;
	else
		return REG_GET(DISPC_CONTROL, bit, bit) == 1;
}
void dispc_set_dithering(enum omap_channel channel)
{
	int temp;

	if (channel == OMAP_DSS_CHANNEL_LCD)
	{
		temp = 2;
		REG_FLD_MOD(DISPC_CONTROL, temp, 31, 30);
		temp = 1;
		REG_FLD_MOD(DISPC_CONTROL, temp, 7, 7);
	}
}

void dispc_mgr_go(enum omap_channel channel)
{
	int bit;
	bool enable_bit, go_bit;
	DSSDBG("%s\n",__FUNCTION__);

	if (dispc_mgr_is_lcd(channel))
		bit = 0; /* LCDENABLE */
	else
		bit = 1; /* DIGITALENABLE */

	/* if the channel is not enabled, we don't need GO */
	if (channel == OMAP_DSS_CHANNEL_LCD2)
		enable_bit = REG_GET(DISPC_CONTROL2, bit, bit) == 1;
	else
		enable_bit = REG_GET(DISPC_CONTROL, bit, bit) == 1;

	if (!enable_bit)
		return;

	if (dispc_mgr_is_lcd(channel))
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		go_bit = REG_GET(DISPC_CONTROL2, bit, bit) == 1;
	else
		go_bit = REG_GET(DISPC_CONTROL, bit, bit) == 1;

	if (go_bit) {
		DSSERR("GO bit not down for channel %d\n", channel);
		return;
	}

	DSSDBG("GO %s\n", channel == OMAP_DSS_CHANNEL_LCD ? "LCD" :
		(channel == OMAP_DSS_CHANNEL_LCD2 ? "LCD2" : "DIGIT"));

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, 1, bit, bit);
	else
		REG_FLD_MOD(DISPC_CONTROL, 1, bit, bit);
}

static void dispc_ovl_write_firh_reg(enum omap_plane plane, int reg, u32 value)
{
	dispc_write_reg(DISPC_OVL_FIR_COEF_H(plane, reg), value);
}

static void dispc_ovl_write_firhv_reg(enum omap_plane plane, int reg, u32 value)
{
	dispc_write_reg(DISPC_OVL_FIR_COEF_HV(plane, reg), value);
}

static void dispc_ovl_write_firv_reg(enum omap_plane plane, int reg, u32 value)
{
	dispc_write_reg(DISPC_OVL_FIR_COEF_V(plane, reg), value);
}

static void dispc_ovl_write_firh2_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_OVL_FIR_COEF_H2(plane, reg), value);
}

static void dispc_ovl_write_firhv2_reg(enum omap_plane plane, int reg,
		u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_OVL_FIR_COEF_HV2(plane, reg), value);
}

static void dispc_ovl_write_firv2_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_OVL_FIR_COEF_V2(plane, reg), value);
}

static void dispc_ovl_set_scale_coef(enum omap_plane plane, int fir_hinc,
				int fir_vinc, int five_taps,
				enum omap_color_component color_comp)
{
	const struct dispc_coef *h_coef, *v_coef;
	int i;

	h_coef = dispc_ovl_get_scale_coef(fir_hinc, true);
	v_coef = dispc_ovl_get_scale_coef(fir_vinc, five_taps);

	for (i = 0; i < 8; i++) {
		u32 h, hv;

		h = FLD_VAL(h_coef[i].hc0_vc00, 7, 0)
			| FLD_VAL(h_coef[i].hc1_vc0, 15, 8)
			| FLD_VAL(h_coef[i].hc2_vc1, 23, 16)
			| FLD_VAL(h_coef[i].hc3_vc2, 31, 24);
		hv = FLD_VAL(h_coef[i].hc4_vc22, 7, 0)
			| FLD_VAL(v_coef[i].hc1_vc0, 15, 8)
			| FLD_VAL(v_coef[i].hc2_vc1, 23, 16)
			| FLD_VAL(v_coef[i].hc3_vc2, 31, 24);

		if (color_comp == DISPC_COLOR_COMPONENT_RGB_Y) {
			dispc_ovl_write_firh_reg(plane, i, h);
			dispc_ovl_write_firhv_reg(plane, i, hv);
		} else {
			dispc_ovl_write_firh2_reg(plane, i, h);
			dispc_ovl_write_firhv2_reg(plane, i, hv);
		}

	}

	if (five_taps) {
		for (i = 0; i < 8; i++) {
			u32 v;
			v = FLD_VAL(v_coef[i].hc0_vc00, 7, 0)
				| FLD_VAL(v_coef[i].hc4_vc22, 15, 8);
			if (color_comp == DISPC_COLOR_COMPONENT_RGB_Y)
				dispc_ovl_write_firv_reg(plane, i, v);
			else
				dispc_ovl_write_firv2_reg(plane, i, v);
		}
	}
}

void dispc_mgr_setup_color_conv_coef(enum omap_plane plane,
		const struct omap_dss_cconv_coefs *ct)
{
#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 0), CVAL(ct->rcr, ct->ry));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 1), CVAL(ct->gy,  ct->rcb));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 2), CVAL(ct->gcb, ct->gcr));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 3), CVAL(ct->bcr, ct->by));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 4), CVAL(0, ct->bcb));

#undef CVAL

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), ct->full_range, 11, 11);
}

void dispc_mgr_setup_wb_color_conv_coef(void)
{
	const struct wb_color_conv {
		int  yr,  crr,  cbr;
		int  yg,  crg,  cbg;
		int  yb,  crb,  cbb;
		int  full_range;
	}  ct = {
		66,  112,  -38,
		129, -94,  -74,
		25,  -18,   112,
		0,
	};
#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))
	dispc_write_reg(DISPC_OVL_CONV_COEF(OMAP_DSS_WB, 0),
			CVAL(ct.yg, ct.yr));
	dispc_write_reg(DISPC_OVL_CONV_COEF(OMAP_DSS_WB, 1),
			CVAL(ct.crr,  ct.yb));
	dispc_write_reg(DISPC_OVL_CONV_COEF(OMAP_DSS_WB, 2),
			CVAL(ct.crb, ct.crg));
	dispc_write_reg(DISPC_OVL_CONV_COEF(OMAP_DSS_WB, 3),
			CVAL(ct.cbg, ct.cbr));
	dispc_write_reg(DISPC_OVL_CONV_COEF(OMAP_DSS_WB, 4),
			CVAL(0, ct.cbb));
#undef CVAL
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(OMAP_DSS_WB),
			ct.full_range, 11, 11);
}


static void dispc_ovl_set_ba0(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA0(plane), paddr);
}

static void dispc_ovl_set_ba1(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA1(plane), paddr);
}

static void dispc_ovl_set_ba0_uv(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA0_UV(plane), paddr);
}

static void dispc_ovl_set_ba1_uv(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA1_UV(plane), paddr);
}

static void dispc_ovl_set_pos(enum omap_plane plane, int x, int y)
{
	u32 val = FLD_VAL(y, 26, 16) | FLD_VAL(x, 10, 0);

	dispc_write_reg(DISPC_OVL_POSITION(plane), val);
}

static void dispc_ovl_set_pic_size(enum omap_plane plane, int width, int height)
{
	u32 val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);

	if (plane == OMAP_DSS_GFX)
		dispc_write_reg(DISPC_OVL_SIZE(plane), val);
	else
		dispc_write_reg(DISPC_OVL_PICTURE_SIZE(plane), val);
}

static void dispc_ovl_set_vid_size(enum omap_plane plane, int width, int height)
{
	u32 val;

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);

	dispc_write_reg(DISPC_OVL_SIZE(plane), val);
}

static void dispc_ovl_set_1d_tiled_mode(enum omap_plane plane, bool force_1d)
{
	struct omap_overlay *ovl = omap_dss_get_overlay(plane);

	if ((ovl->caps & OMAP_DSS_OVL_CAP_FORCE_1D) == 0)
		return;

	if (plane == OMAP_DSS_GFX)
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), force_1d, 16, 16);
	else
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), force_1d, 20, 20);
}

static void dispc_ovl_set_zorder(enum omap_plane plane, u8 zorder)
{
	struct omap_overlay *ovl = omap_dss_get_overlay(plane);

	if ((ovl->caps & OMAP_DSS_OVL_CAP_ZORDER) == 0)
		return;

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), zorder, 27, 26);
}

static void dispc_ovl_enable_zorder_planes(void)
{
	int i;

	if (!dss_has_feature(FEAT_ALPHA_FREE_ZORDER))
		return;

	for (i = 0; i < dss_feat_get_num_ovls(); i++)
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(i), 1, 25, 25);
}

static void dispc_ovl_set_pre_mult_alpha(enum omap_plane plane, bool enable)
{
	struct omap_overlay *ovl = omap_dss_get_overlay(plane);

	if ((ovl->caps & OMAP_DSS_OVL_CAP_PRE_MULT_ALPHA) == 0)
		return;

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable ? 1 : 0, 28, 28);
}

static void dispc_ovl_setup_global_alpha(enum omap_plane plane, u8 global_alpha)
{
	static const unsigned shifts[] = { 0, 8, 16, 24, };
	int shift;
	struct omap_overlay *ovl = omap_dss_get_overlay(plane);

	if ((ovl->caps & OMAP_DSS_OVL_CAP_GLOBAL_ALPHA) == 0)
		return;

	shift = shifts[plane];
	REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, shift + 7, shift);
}

static void dispc_ovl_set_pix_inc(enum omap_plane plane, s32 inc)
{
	dispc_write_reg(DISPC_OVL_PIXEL_INC(plane), inc);
}

static void dispc_ovl_set_row_inc(enum omap_plane plane, s32 inc)
{
	dispc_write_reg(DISPC_OVL_ROW_INC(plane), inc);
}

static void dispc_ovl_set_color_mode(enum omap_plane plane,
		enum omap_color_mode color_mode)
{
	u32 m = 0;
	if (plane != OMAP_DSS_GFX) {
		switch (color_mode) {
		case OMAP_DSS_COLOR_NV12:
			m = 0x0; break;
		case OMAP_DSS_COLOR_RGBX16:
			m = 0x1; break;
		case OMAP_DSS_COLOR_RGBA16:
			m = 0x2; break;
		case OMAP_DSS_COLOR_BGRA32:
			m = 0x3; break;
		case OMAP_DSS_COLOR_RGB12U:
			m = 0x4; break;
		case OMAP_DSS_COLOR_ARGB16:
			m = 0x5; break;
		case OMAP_DSS_COLOR_RGB16:
			m = 0x6; break;
		case OMAP_DSS_COLOR_ARGB16_1555:
			m = 0x7; break;
		case OMAP_DSS_COLOR_RGB24U:
			m = 0x8; break;
		case OMAP_DSS_COLOR_RGB24P:
			m = 0x9; break;
		case OMAP_DSS_COLOR_YUV2:
			m = 0xa; break;
		case OMAP_DSS_COLOR_UYVY:
			m = 0xb; break;
		case OMAP_DSS_COLOR_ARGB32:
			m = 0xc; break;
		case OMAP_DSS_COLOR_RGBA32:
			m = 0xd; break;
		case OMAP_DSS_COLOR_RGBX32:
			m = 0xe; break;
		case OMAP_DSS_COLOR_XRGB16_1555:
			m = 0xf; break;
		default:
			BUG(); break;
		}
	} else {
		switch (color_mode) {
		case OMAP_DSS_COLOR_CLUT1:
			m = 0x0; break;
		case OMAP_DSS_COLOR_CLUT2:
			m = 0x1; break;
		case OMAP_DSS_COLOR_CLUT4:
			m = 0x2; break;
		case OMAP_DSS_COLOR_CLUT8:
			m = 0x3; break;
		case OMAP_DSS_COLOR_BGRA32:
			m = 0x3; break;
		case OMAP_DSS_COLOR_RGB12U:
			m = 0x4; break;
		case OMAP_DSS_COLOR_ARGB16:
			m = 0x5; break;
		case OMAP_DSS_COLOR_RGB16:
			m = 0x6; break;
		case OMAP_DSS_COLOR_ARGB16_1555:
			m = 0x7; break;
		case OMAP_DSS_COLOR_RGB24U:
			m = 0x8; break;
		case OMAP_DSS_COLOR_RGB24P:
			m = 0x9; break;
		case OMAP_DSS_COLOR_RGBX16:
			m = 0xa; break;
		case OMAP_DSS_COLOR_RGBA16:
			m = 0xb; break;
		case OMAP_DSS_COLOR_ARGB32:
			m = 0xc; break;
		case OMAP_DSS_COLOR_RGBA32:
			m = 0xd; break;
		case OMAP_DSS_COLOR_RGBX32:
			m = 0xe; break;
		case OMAP_DSS_COLOR_XRGB16_1555:
			m = 0xf; break;
		default:
			BUG(); break;
		}
	}

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), m, 4, 1);
}

void dispc_ovl_set_channel_out(enum omap_plane plane, enum omap_channel channel)
{
	int shift;
	u32 val;
	int chan = 0, chan2 = 0;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 8;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
	case OMAP_DSS_VIDEO3:
		shift = 16;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		switch (channel) {
		case OMAP_DSS_CHANNEL_LCD:
			chan = 0;
			chan2 = 0;
			break;
		case OMAP_DSS_CHANNEL_DIGIT:
			chan = 1;
			chan2 = 0;
			break;
		case OMAP_DSS_CHANNEL_LCD2:
			chan = 0;
			chan2 = 1;
			break;
		default:
			BUG();
		}

		val = FLD_MOD(val, chan, shift, shift);
		val = FLD_MOD(val, chan2, 31, 30);
	} else {
		val = FLD_MOD(val, channel, shift, shift);
	}
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

static enum omap_channel dispc_ovl_get_channel_out(enum omap_plane plane)
{
	int shift;
	u32 val;
	enum omap_channel channel;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 8;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
	case OMAP_DSS_VIDEO3:
		shift = 16;
		break;
	default:
		BUG();
	}

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));

	if (dss_has_feature(FEAT_MGR_LCD2)) {
		if (FLD_GET(val, 31, 30) == 0)
			channel = FLD_GET(val, shift, shift);
		else
			channel = OMAP_DSS_CHANNEL_LCD2;
	} else {
		channel = FLD_GET(val, shift, shift);
	}

	return channel;
}

void dispc_set_wb_channel_out(enum omap_plane plane)
{
	int shift;
	u32 val;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 8;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
	case OMAP_DSS_VIDEO3:
		shift = 16;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	val = FLD_MOD(val, 0, shift, shift);
	val = FLD_MOD(val, 3, 31, 30);

	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

static void dispc_ovl_set_burst_size(enum omap_plane plane,
		enum omap_burst_size burst_size)
{
	static const unsigned shifts[] = { 6, 14, 14, 14, };
	int shift;

	shift = shifts[plane];
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), burst_size, shift + 1, shift);
}

static void dispc_configure_burst_sizes(void)
{
	int i;
	const int burst_size = BURST_SIZE_X8;

	/* Configure burst size always to maximum size */
	for (i = 0; i < omap_dss_get_num_overlays(); ++i)
		dispc_ovl_set_burst_size(i, burst_size);
}

static u32 dispc_ovl_get_burst_size(enum omap_plane plane)
{
	unsigned unit = dss_feat_get_burst_size_unit();
	/* burst multiplier is always x8 (see dispc_configure_burst_sizes()) */
	return unit * 8;
}

void dispc_enable_gamma_table(bool enable)
{
	/*
	 * This is partially implemented to support only disabling of
	 * the gamma table.
	 */
	if (enable) {
		DSSWARN("Gamma table enabling for TV not yet supported");
		return;
	}

	REG_FLD_MOD(DISPC_CONFIG, enable, 9, 9);
}

static void dispc_mgr_enable_cpr(enum omap_channel channel, bool enable)
{
	u16 reg;

	if (channel == OMAP_DSS_CHANNEL_LCD)
		reg = DISPC_CONFIG;
	else if (channel == OMAP_DSS_CHANNEL_LCD2)
		reg = DISPC_CONFIG2;
	else
		return;

	REG_FLD_MOD(reg, enable, 15, 15);
}

static void dispc_mgr_set_cpr_coef(enum omap_channel channel,
		struct omap_dss_cpr_coefs *coefs)
{
	u32 coef_r, coef_g, coef_b;

	if (!dispc_mgr_is_lcd(channel))
		return;

	coef_r = FLD_VAL(coefs->rr, 31, 22) | FLD_VAL(coefs->rg, 20, 11) |
		FLD_VAL(coefs->rb, 9, 0);
	coef_g = FLD_VAL(coefs->gr, 31, 22) | FLD_VAL(coefs->gg, 20, 11) |
		FLD_VAL(coefs->gb, 9, 0);
	coef_b = FLD_VAL(coefs->br, 31, 22) | FLD_VAL(coefs->bg, 20, 11) |
		FLD_VAL(coefs->bb, 9, 0);

	dispc_write_reg(DISPC_CPR_COEF_R(channel), coef_r);
	dispc_write_reg(DISPC_CPR_COEF_G(channel), coef_g);
	dispc_write_reg(DISPC_CPR_COEF_B(channel), coef_b);
}

static void dispc_ovl_set_vid_color_conv(enum omap_plane plane, bool enable)
{
	u32 val;

	BUG_ON(plane == OMAP_DSS_GFX);

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	val = FLD_MOD(val, enable, 9, 9);
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

static void dispc_ovl_enable_replication(enum omap_plane plane, bool enable)
{
	static const unsigned shifts[] = { 5, 10, 10, 10 };
	int shift;

	shift = shifts[plane];
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable, shift, shift);
}

void dispc_mgr_set_lcd_size(enum omap_channel channel, u16 width, u16 height)
{
	u32 val;
	BUG_ON(width > dss_feat_get_param_max(FEAT_PARAM_MGR_WIDTH) ||
		height > dss_feat_get_param_max(FEAT_PARAM_MGR_HEIGHT));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(DISPC_SIZE_MGR(channel), val);
}

void dispc_set_digit_size(u16 width, u16 height)
{
	u32 val;
	BUG_ON(width > dss_feat_get_param_max(FEAT_PARAM_MGR_WIDTH) ||
		height > dss_feat_get_param_max(FEAT_PARAM_MGR_HEIGHT));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(DISPC_SIZE_MGR(OMAP_DSS_CHANNEL_DIGIT), val);
}

static void dispc_read_plane_fifo_sizes(void)
{
	u32 size;
	int plane;
	u8 start, end;
	u32 unit;

	unit = dss_feat_get_buffer_size_unit();

	dss_feat_get_reg_field(FEAT_REG_FIFOSIZE, &start, &end);

	for (plane = 0; plane < dss_feat_get_num_ovls(); ++plane) {
		size = REG_GET(DISPC_OVL_FIFO_SIZE_STATUS(plane), start, end);
		size *= unit;
		dispc.fifo_size[plane] = size;
	}
}

static u32 dispc_ovl_get_fifo_size(enum omap_plane plane)
{
	return dispc.fifo_size[plane];
}

static void dispc_ovl_set_mflag_attribute(enum dispc_mflag_ctrl ctrl)
{
	REG_FLD_MOD(DISPC_GLOBAL_MFLAG, ctrl, 1, 0);
}

static void dispc_ovl_set_mflag_start(enum dispc_mflag_start start)
{
	REG_FLD_MOD(DISPC_GLOBAL_MFLAG, start, 2, 2);
}

void dispc_ovl_set_global_mflag(enum omap_plane plane, bool mflag)
{
	u32 fifosize, unit;
	u8 bit;

	/* Set the ARBITRATION bit to give
	 * highest priority to the pipeline.
	 * MFLAG is applicable only to high
	 * priority pipes.
	 */
	 if (plane == OMAP_DSS_GFX)
		bit = 14;
	 else
		bit = 23;

	 REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), mflag, bit, bit);

	 dispc_ovl_set_mflag_attribute(DISPC_MFLAG_CTRL_ENABLE);
	 /* Allows the mflag signal to start at the beginning of each
	  * frame even if the DMA buffer is empty */
	 dispc_ovl_set_mflag_start(DISPC_MFLAG_START_ENABLE);

	 if (dss_has_feature(FEAT_MFLAG)) {
		fifosize = dispc_ovl_get_fifo_size(plane);
		unit = dss_feat_get_buffer_size_unit();
		/* As per the simultaion team suggestion, below thesholds are set:
		* HT = fifosize * 5/8;
		* LT = fifosize * 4/8;
		*/
		dispc_write_reg(DISPC_OVL_MFLAG_THRESHOLD(plane),
			FLD_VAL((fifosize * 5) / (8 * unit), 31, 16) |
			FLD_VAL((fifosize * 4) / (8 * unit), 15, 0));
	 }
}

void dispc_ovl_set_fifo_threshold(enum omap_plane plane, u32 low, u32 high)
{
	u8 hi_start, hi_end, lo_start, lo_end;
	u32 unit;

	unit = dss_feat_get_buffer_size_unit();

	/* For WB pipe, thresholds are hard-coded to optimum value.
	 * The below WARN_ON is not valid in this case.
	 * TODO: To check with HW team on the limitation w.r.t
	 * fofo thresholds.
	*/
	if (plane != OMAP_DSS_WB) {
		WARN_ON(low % unit != 0);
		WARN_ON(high % unit != 0);
		low /= unit;
		high /= unit;
	}

	dss_feat_get_reg_field(FEAT_REG_FIFOHIGHTHRESHOLD, &hi_start, &hi_end);
	dss_feat_get_reg_field(FEAT_REG_FIFOLOWTHRESHOLD, &lo_start, &lo_end);

	DSSDBG("fifo(%d) threshold (bytes), old %u/%u, new %u/%u\n",
			plane,
			REG_GET(DISPC_OVL_FIFO_THRESHOLD(plane),
				lo_start, lo_end) * unit,
			REG_GET(DISPC_OVL_FIFO_THRESHOLD(plane),
				hi_start, hi_end) * unit,
			low * unit, high * unit);

	/* preload to high threshold to avoid FIFO underflow , NA for WB */
	if (plane != OMAP_DSS_WB)
		dispc_write_reg(DISPC_OVL_PRELOAD(plane), min(high, 0xfffu));

	dispc_write_reg(DISPC_OVL_FIFO_THRESHOLD(plane),
			FLD_VAL(high, hi_start, hi_end) |
			FLD_VAL(low, lo_start, lo_end));

	if (plane == OMAP_DSS_GFX)
		dispc_ovl_set_global_mflag(OMAP_DSS_GFX, true);
}

void dispc_enable_fifomerge(bool enable)
{
	if (!dss_has_feature(FEAT_FIFO_MERGE)) {
		WARN_ON(enable);
		return;
	}

	DSSDBG("FIFO merge %s\n", enable ? "enabled" : "disabled");
	REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 14, 14);
}

void dispc_ovl_compute_fifo_thresholds(enum omap_plane plane,
		u32 *fifo_low, u32 *fifo_high, bool use_fifomerge,
		bool manual_update)
{
	/*
	 * All sizes are in bytes. Both the buffer and burst are made of
	 * buffer_units, and the fifo thresholds must be buffer_unit aligned.
	 */

	unsigned buf_unit = dss_feat_get_buffer_size_unit();
	unsigned ovl_fifo_size, total_fifo_size, burst_size;
	int i;

	burst_size = dispc_ovl_get_burst_size(plane);
	ovl_fifo_size = dispc_ovl_get_fifo_size(plane);

	if (use_fifomerge) {
		total_fifo_size = 0;
		for (i = 0; i < omap_dss_get_num_overlays(); ++i)
			total_fifo_size += dispc_ovl_get_fifo_size(i);
	} else {
		total_fifo_size = ovl_fifo_size;
	}

	/*
	 * We use the same low threshold for both fifomerge and non-fifomerge
	 * cases, but for fifomerge we calculate the high threshold using the
	 * combined fifo size
	 */

	if (manual_update && dss_has_feature(FEAT_OMAP3_DSI_FIFO_BUG)) {
		*fifo_low = ovl_fifo_size - burst_size * 2;
		*fifo_high = total_fifo_size - burst_size;
	} else {
		if (cpu_is_omap44xx()) {
			/* optimization of power consumption for OMAP4 */
			*fifo_low = (ovl_fifo_size / 2);
			*fifo_high = total_fifo_size - buf_unit;
		} else {
			/* TODO: which should be the numbers for OMAP5?
			 * Set safest values for now */
			*fifo_low = ovl_fifo_size - burst_size;
			*fifo_high = total_fifo_size - buf_unit;
		}
	}
}

static void dispc_ovl_set_fir(enum omap_plane plane,
				int hinc, int vinc,
				enum omap_color_component color_comp)
{
	u32 val;

	if (color_comp == DISPC_COLOR_COMPONENT_RGB_Y) {
		u8 hinc_start, hinc_end, vinc_start, vinc_end;

		dss_feat_get_reg_field(FEAT_REG_FIRHINC,
					&hinc_start, &hinc_end);
		dss_feat_get_reg_field(FEAT_REG_FIRVINC,
					&vinc_start, &vinc_end);
		val = FLD_VAL(vinc, vinc_start, vinc_end) |
				FLD_VAL(hinc, hinc_start, hinc_end);

		dispc_write_reg(DISPC_OVL_FIR(plane), val);
	} else {
		val = FLD_VAL(vinc, 28, 16) | FLD_VAL(hinc, 12, 0);
		dispc_write_reg(DISPC_OVL_FIR2(plane), val);
	}
}

static void dispc_ovl_set_vid_accu0(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	u8 hor_start, hor_end, vert_start, vert_end;

	dss_feat_get_reg_field(FEAT_REG_HORIZONTALACCU, &hor_start, &hor_end);
	dss_feat_get_reg_field(FEAT_REG_VERTICALACCU, &vert_start, &vert_end);

	val = FLD_VAL(vaccu, vert_start, vert_end) |
			FLD_VAL(haccu, hor_start, hor_end);

	dispc_write_reg(DISPC_OVL_ACCU0(plane), val);
}

static void dispc_ovl_set_vid_accu1(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	u8 hor_start, hor_end, vert_start, vert_end;

	dss_feat_get_reg_field(FEAT_REG_HORIZONTALACCU, &hor_start, &hor_end);
	dss_feat_get_reg_field(FEAT_REG_VERTICALACCU, &vert_start, &vert_end);

	val = FLD_VAL(vaccu, vert_start, vert_end) |
			FLD_VAL(haccu, hor_start, hor_end);

	dispc_write_reg(DISPC_OVL_ACCU1(plane), val);
}

static void dispc_ovl_set_vid_accu2_0(enum omap_plane plane, int haccu,
		int vaccu)
{
	u32 val;

	val = FLD_VAL(vaccu, 26, 16) | FLD_VAL(haccu, 10, 0);
	dispc_write_reg(DISPC_OVL_ACCU2_0(plane), val);
}

static void dispc_ovl_set_vid_accu2_1(enum omap_plane plane, int haccu,
		int vaccu)
{
	u32 val;

	val = FLD_VAL(vaccu, 26, 16) | FLD_VAL(haccu, 10, 0);
	dispc_write_reg(DISPC_OVL_ACCU2_1(plane), val);
}

static void dispc_ovl_set_scale_param(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool five_taps, u8 rotation,
		enum omap_color_component color_comp)
{
	int fir_hinc, fir_vinc;

	fir_hinc = 1024 * orig_width / out_width;
	fir_vinc = 1024 * orig_height / out_height;

	dispc_ovl_set_scale_coef(plane, fir_hinc, fir_vinc, five_taps,
				color_comp);
	dispc_ovl_set_fir(plane, fir_hinc, fir_vinc, color_comp);
}

static void dispc_ovl_set_scaling_common(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, enum omap_color_mode color_mode,
		u8 rotation)
{
	int accu0 = 0;
	int accu1 = 0;
	u32 l;

	dispc_ovl_set_scale_param(plane, orig_width, orig_height,
				out_width, out_height, five_taps,
				rotation, DISPC_COLOR_COMPONENT_RGB_Y);
	l = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));

	/* RESIZEENABLE and VERTICALTAPS */
	l &= ~((0x3 << 5) | (0x1 << 21));
	l |= (orig_width != out_width) ? (1 << 5) : 0;
	l |= (orig_height != out_height) ? (1 << 6) : 0;
	l |= five_taps ? (1 << 21) : 0;

	/* VRESIZECONF and HRESIZECONF */
	if (dss_has_feature(FEAT_RESIZECONF)) {
		l &= ~(0x3 << 7);
		l |= (orig_width <= out_width) ? 0 : (1 << 7);
		l |= (orig_height <= out_height) ? 0 : (1 << 8);
	}

	/* LINEBUFFERSPLIT */
	if (dss_has_feature(FEAT_LINEBUFFERSPLIT)) {
		l &= ~(0x1 << 22);
		l |= five_taps ? (1 << 22) : 0;
	}

	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), l);

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	if (ilace && !fieldmode) {
		accu1 = 0;
		accu0 = ((1024 * orig_height / out_height) / 2) & 0x3ff;
		if (accu0 >= 1024/2) {
			accu1 = 1024/2;
			accu0 -= accu1;
		}
	}

	dispc_ovl_set_vid_accu0(plane, 0, accu0);
	dispc_ovl_set_vid_accu1(plane, 0, accu1);
}

static void dispc_ovl_set_scaling_uv(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, enum omap_color_mode color_mode,
		u8 rotation)
{
	int scale_x = out_width != orig_width;
	int scale_y = out_height != orig_height;

	if (!dss_has_feature(FEAT_HANDLE_UV_SEPARATE))
		return;
	if ((color_mode != OMAP_DSS_COLOR_YUV2 &&
			color_mode != OMAP_DSS_COLOR_UYVY &&
			color_mode != OMAP_DSS_COLOR_NV12)) {
		/* reset chroma resampling for RGB formats, NA for WB */
		if (plane != OMAP_DSS_WB)
			REG_FLD_MOD(DISPC_OVL_ATTRIBUTES2(plane), 0, 8, 8);
		return;
	}
	switch (color_mode) {
	case OMAP_DSS_COLOR_NV12:
		if (plane != OMAP_DSS_WB) {
			/* UV is subsampled by 2 vertically*/
			orig_height >>= 1;
			/* UV is subsampled by 2 horz.*/
			orig_width >>= 1;
		} else {
			/* UV is downsampled by 2 vertically*/
			out_height >>= 1;
			/* UV is downsampled by 2 horz.*/
			out_width >>= 1;
		}
		break;
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		/*For YUV422 with 90/270 rotation,
		 *we don't upsample chroma
		 */
		if (rotation == OMAP_DSS_ROT_0 ||
			rotation == OMAP_DSS_ROT_180) {
			if (plane != OMAP_DSS_WB) {
				/* UV is subsampled by 2 hrz*/
				orig_width >>= 1;
			} else {
				out_width >>= 1;
			}
		}
		/* must use FIR for YUV422 if rotated */
		if (rotation != OMAP_DSS_ROT_0)
			scale_x = scale_y = true;
		break;
	default:
		BUG();
	}

	if (out_width != orig_width)
		scale_x = true;
	if (out_height != orig_height)
		scale_y = true;

	dispc_ovl_set_scale_param(plane, orig_width, orig_height,
			out_width, out_height, five_taps,
				rotation, DISPC_COLOR_COMPONENT_UV);

	if (plane != OMAP_DSS_WB)
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES2(plane),
			(scale_x || scale_y) ? 1 : 0, 8, 8);
	/* set H scaling */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), scale_x ? 1 : 0, 5, 5);
	/* set V scaling */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), scale_y ? 1 : 0, 6, 6);

	dispc_ovl_set_vid_accu2_0(plane, 0x80, 0);
	dispc_ovl_set_vid_accu2_1(plane, 0x80, 0);
}

static void dispc_ovl_set_scaling(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, enum omap_color_mode color_mode,
		u8 rotation)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_ovl_set_scaling_common(plane,
			orig_width, orig_height,
			out_width, out_height,
			ilace, five_taps,
			fieldmode, color_mode,
			rotation);

	dispc_ovl_set_scaling_uv(plane,
		orig_width, orig_height,
		out_width, out_height,
		ilace, five_taps,
		fieldmode, color_mode,
		rotation);
}

static void dispc_ovl_set_rotation_attrs(enum omap_plane plane, u8 rotation,
		bool mirroring, enum omap_color_mode color_mode,
		enum omap_dss_rotation_type type)
{
	if (plane != OMAP_DSS_WB) {
		bool row_repeat = false;
		int vidrot = 0;

		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
				color_mode == OMAP_DSS_COLOR_UYVY) {

			if (mirroring) {
				switch (rotation) {
				case OMAP_DSS_ROT_0:
					vidrot = 2;
					break;
				case OMAP_DSS_ROT_90:
					vidrot = 1;
					break;
				case OMAP_DSS_ROT_180:
					vidrot = 0;
					break;
				case OMAP_DSS_ROT_270:
					vidrot = 3;
					break;
				}
			} else {
				switch (rotation) {
				case OMAP_DSS_ROT_0:
					vidrot = 0;
					break;
				case OMAP_DSS_ROT_90:
					vidrot = 1;
					break;
				case OMAP_DSS_ROT_180:
					vidrot = 2;
					break;
				case OMAP_DSS_ROT_270:
					vidrot = 3;
					break;
				}
			}

			if (rotation == OMAP_DSS_ROT_90 ||
						rotation == OMAP_DSS_ROT_270)
				row_repeat = true;
			else
				row_repeat = false;
		} else if (color_mode == OMAP_DSS_COLOR_NV12) {
			/* WA for OMAP4+ UV plane overread HW bug */
			vidrot = 1;
		}

		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), vidrot, 13, 12);
		if (dss_has_feature(FEAT_ROWREPEATENABLE))
			REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane),
				row_repeat ? 1 : 0, 18, 18);
	}

	if (color_mode == OMAP_DSS_COLOR_NV12) {
		/* this will never happen for GFX */
		/* 1D NV12 buffer is always non-rotated or vert. mirrored */
		bool doublestride = (rotation == OMAP_DSS_ROT_0 ||
				     rotation == OMAP_DSS_ROT_180) &&
				     type == OMAP_DSS_ROT_TILER;
		/* DOUBLESTRIDE */
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), doublestride, 22, 22);
	}
}

static int color_mode_to_bpp(enum omap_color_mode color_mode)
{
	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
		return 1;
	case OMAP_DSS_COLOR_CLUT2:
		return 2;
	case OMAP_DSS_COLOR_CLUT4:
		return 4;
	case OMAP_DSS_COLOR_CLUT8:
	case OMAP_DSS_COLOR_NV12:
		return 8;
	case OMAP_DSS_COLOR_RGB12U:
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_ARGB16:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
	case OMAP_DSS_COLOR_RGBA16:
	case OMAP_DSS_COLOR_RGBX16:
	case OMAP_DSS_COLOR_ARGB16_1555:
	case OMAP_DSS_COLOR_XRGB16_1555:
		return 16;
	case OMAP_DSS_COLOR_RGB24P:
		return 24;
	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		return 32;
	default:
		BUG();
	}
}

static s32 pixinc(int pixels, u8 ps)
{
	if (!cpu_is_omap44xx() && !cpu_is_omap54xx())
		BUG_ON(pixels == 0);
	return 1 + (pixels - 1) * ps;
}

static void calc_tiler_row_rotation(struct tiler_view_t *view,
		u16 width, int bpp, int y_decim,
		s32 *row_inc, unsigned *offset1, bool ilace)
{
	/* assume TB. We worry about swapping top/bottom outside of this call */

	if (ilace) {
		/* even and odd frames are interleaved */

		/* offset1 is always at an odd line */
		*offset1 = view->v_inc * (y_decim | 1);
		y_decim *= 2;
	}
	*row_inc = view->v_inc * y_decim + 1 - width * bpp;

	DSSDBG("ps: %d/%d, width: %d, offset1: %d, height: %d, row_inc:%d\n",
			view->bpp, bpp, view->width, *offset1, view->height,
			*row_inc);

	return;
}

static void calc_vrfb_rotation_offset(u8 rotation, bool mirror,
		u16 screen_width,
		u16 width, u16 height,
		enum omap_color_mode color_mode, bool fieldmode,
		unsigned int field_offset,
		unsigned *offset0, unsigned *offset1,
		s32 *row_inc, s32 *pix_inc)
{
	u8 ps;

	/* FIXME CLUT formats */
	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
	case OMAP_DSS_COLOR_CLUT2:
	case OMAP_DSS_COLOR_CLUT4:
	case OMAP_DSS_COLOR_CLUT8:
		BUG();
		return;
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		ps = 4;
		break;
	default:
		ps = color_mode_to_bpp(color_mode) / 8;
		break;
	}

	DSSDBG("calc_rot(%d): scrw %d, %dx%d\n", rotation, screen_width,
			width, height);

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	switch (rotation + mirror * 4) {
	case OMAP_DSS_ROT_0:
	case OMAP_DSS_ROT_180:
		/*
		 * If the pixel format is YUV or UYVY divide the width
		 * of the image by 2 for 0 and 180 degree rotation.
		 */
		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY)
			width = width >> 1;
	case OMAP_DSS_ROT_90:
	case OMAP_DSS_ROT_270:
		*offset1 = 0;
		if (field_offset)
			*offset0 = field_offset * screen_width * ps;
		else
			*offset0 = 0;

		*row_inc = pixinc(1 + (screen_width - width) +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	case OMAP_DSS_ROT_0 + 4:
	case OMAP_DSS_ROT_180 + 4:
		/* If the pixel format is YUV or UYVY divide the width
		 * of the image by 2  for 0 degree and 180 degree
		 */
		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY)
			width = width >> 1;
	case OMAP_DSS_ROT_90 + 4:
	case OMAP_DSS_ROT_270 + 4:
		*offset1 = 0;
		if (field_offset)
			*offset0 = field_offset * screen_width * ps;
		else
			*offset0 = 0;
		*row_inc = pixinc(1 - (screen_width + width) -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	default:
		BUG();
	}
}

static void calc_dma_rotation_offset(u8 rotation, bool mirror,
		u16 screen_width,
		u16 width, u16 height,
		enum omap_color_mode color_mode, bool fieldmode,
		unsigned int field_offset,
		unsigned *offset0, unsigned *offset1,
		s32 *row_inc, s32 *pix_inc, int x_decim, int y_decim)
{
	u8 ps;
	u16 fbw, fbh;

	/* FIXME CLUT formats */
	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
	case OMAP_DSS_COLOR_CLUT2:
	case OMAP_DSS_COLOR_CLUT4:
	case OMAP_DSS_COLOR_CLUT8:
		BUG();
		return;
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		if (cpu_is_omap44xx() || cpu_is_omap54xx()) {
			/* on OMAP4/5 YUYV is handled as 32-bit data */
			ps = 4;
			screen_width /= 2;
			break;
		}
		/* fall through */
	default:
		ps = color_mode_to_bpp(color_mode) / 8;
		break;
	}

	DSSDBG("calc_rot(%d): scrw %d, %dx%d\n", rotation, screen_width,
			width, height);

	/* width & height are overlay sizes, convert to fb sizes */

	if (rotation == OMAP_DSS_ROT_0 || rotation == OMAP_DSS_ROT_180) {
		fbw = width;
		fbh = height;
	} else {
		fbw = height;
		fbh = width;
	}

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	switch (rotation + mirror * 4) {
	case OMAP_DSS_ROT_0:
		*offset1 = 0;
		if (field_offset)
			*offset0 = *offset1 + field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(1 + (y_decim * screen_width - fbw * x_decim) +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(x_decim, ps);
		break;
	case OMAP_DSS_ROT_90:
		*offset1 = screen_width * (fbh - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 + field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * (fbh - 1) + 1 +
				(fieldmode ? 1 : 0), ps);
		*pix_inc = pixinc(-screen_width, ps);
		break;
	case OMAP_DSS_ROT_180:
		*offset1 = (screen_width * (fbh - 1) + fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-1 -
				(screen_width - fbw) -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(-1, ps);
		break;
	case OMAP_DSS_ROT_270:
		*offset1 = (fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-screen_width * (fbh - 1) - 1 -
				(fieldmode ? 1 : 0), ps);
		*pix_inc = pixinc(screen_width, ps);
		break;

	/* mirroring */
	case OMAP_DSS_ROT_0 + 4:
		*offset1 = (fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 + field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * 2 - 1 +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(-1, ps);
		break;

	case OMAP_DSS_ROT_90 + 4:
		*offset1 = 0;
		if (field_offset)
			*offset0 = *offset1 + field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-screen_width * (fbh - 1) + 1 +
				(fieldmode ? 1 : 0),
				ps);
		*pix_inc = pixinc(screen_width, ps);
		break;

	case OMAP_DSS_ROT_180 + 4:
		*offset1 = screen_width * (fbh - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(1 - screen_width * 2 -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	case OMAP_DSS_ROT_270 + 4:
		*offset1 = (screen_width * (fbh - 1) + fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * (fbh - 1) - 1 -
				(fieldmode ? 1 : 0),
				ps);
		*pix_inc = pixinc(-screen_width, ps);
		break;

	default:
		BUG();
	}
}

static unsigned long calc_fclk_five_taps(enum omap_channel channel, u16 width,
		u16 height, u16 out_width, u16 out_height,
		enum omap_color_mode color_mode)
{
	u32 fclk = 0;
	u64 tmp, pclk = dispc_mgr_pclk_rate(channel);

	if (cpu_is_omap44xx() || cpu_is_omap54xx()) {
		/* do conservative TRM value on OMAP4 ES1.0 */
		if (omap_rev() == OMAP4430_REV_ES1_0)
			return pclk * DIV_ROUND_UP(width, out_width) *
					DIV_ROUND_UP(height, out_height);

		/* since 4430 ES2.0, fclk requirement only depends on width */
		pclk *= max(width, out_width);
		do_div(pclk, out_width);
		return pclk;
	}

	if (height <= out_height && width <= out_width)
		return (unsigned long) pclk;

	if (height > out_height) {
		struct omap_dss_device *dssdev = dispc_mgr_get_device(channel);
		unsigned int ppl = dssdev->panel.timings.x_res;

		tmp = pclk * height * out_width;
		do_div(tmp, 2 * out_height * ppl);
		fclk = tmp;

		if (height > 2 * out_height) {
			if (ppl == out_width)
				return 0;

			tmp = pclk * (height - 2 * out_height) * out_width;
			do_div(tmp, 2 * out_height * (ppl - out_width));
			fclk = max(fclk, (u32) tmp);
		}
	}

	if (width > out_width) {
		tmp = pclk * width;
		do_div(tmp, out_width);
		fclk = max(fclk, (u32) tmp);

		if (color_mode == OMAP_DSS_COLOR_RGB24U)
			fclk <<= 1;
	}

	return fclk;
}

static unsigned long calc_fclk(enum omap_channel channel, u16 width,
		u16 height, u16 out_width, u16 out_height)
{
	unsigned int hf, vf;
	unsigned long pclk = dispc_mgr_pclk_rate(channel);

	/* on OMAP4/5 three-tap and five-tap clock requirements are the same */
	if (cpu_is_omap44xx() || cpu_is_omap54xx())
		return calc_fclk_five_taps(channel, width, height, out_width,
					out_height, 0);

	/*
	 * FIXME how to determine the 'A' factor
	 * for the no downscaling case ?
	 */

	if (width > 3 * out_width)
		hf = 4;
	else if (width > 2 * out_width)
		hf = 3;
	else if (width > out_width)
		hf = 2;
	else
		hf = 1;

	if (height > out_height)
		vf = 2;
	else
		vf = 1;

	if (cpu_is_omap24xx()) {
		if (vf > 1 && hf > 1)
			return pclk * 4;
		else
			return pclk * 2;
	} else if (cpu_is_omap34xx()) {
		return pclk * vf * hf;
	} else {
		if (hf > 1)
			return DIV_ROUND_UP(pclk, out_width) * width;
		else
			return pclk;
	}
}

int dispc_scaling_decision(enum omap_plane plane, struct omap_overlay_info *oi,
			enum omap_channel channel,
			u16 *x_decim, u16 *y_decim, bool *five_taps)
{
	const int maxdownscale = dss_feat_get_param_max(FEAT_PARAM_DOWNSCALE);
	int bpp = color_mode_to_bpp(oi->color_mode);

	/*
	 * For now only whole byte formats on OMAP4/5 can be predecimated.
	 * Later SDMA decimation support may be added
	 */
	bool can_decimate_x = (cpu_is_omap44xx() || cpu_is_omap54xx()) &&
						!(bpp & 7);
	bool can_decimate_y = can_decimate_x;

	bool can_scale = plane != OMAP_DSS_GFX;

	u16 in_width, in_height;
	unsigned long fclk = 0, fclk5 = 0;
	int min_factor, max_factor;	/* decimation search limits */
	int x, y;			/* decimation search variables */
	unsigned long fclk_max = dispc_fclk_rate();
	u16 y_decim_limit = oi->rotation_type == OMAP_DSS_ROT_TILER ? 2 : 16;

	/* No decimation for bitmap formats */
	if (oi->color_mode == OMAP_DSS_COLOR_CLUT1 ||
	    oi->color_mode == OMAP_DSS_COLOR_CLUT2 ||
	    oi->color_mode == OMAP_DSS_COLOR_CLUT4 ||
	    oi->color_mode == OMAP_DSS_COLOR_CLUT8) {
		*x_decim = 1;
		*y_decim = 1;
		*five_taps = false;
		return 0;
	}

	/* restrict search region based on whether we can decimate */
	if (!can_decimate_x) {
		if (oi->min_x_decim > 1)
			return -EINVAL;
		oi->min_x_decim = 1;
		oi->max_x_decim = 1;
	} else {
		if (oi->max_x_decim > 16)
			oi->max_x_decim = 16;
	}

	if (!can_decimate_y) {
		if (oi->min_y_decim > 1)
			return -EINVAL;
		oi->min_y_decim = 1;
		oi->max_y_decim = 1;
	} else {
		if (oi->max_y_decim > y_decim_limit)
			oi->max_y_decim = y_decim_limit;
	}

	/*
	 * Find best supported quality.  In the search algorithm, we make use
	 * of the fact, that increased decimation in either direction will have
	 * lower quality.  However, we do not differentiate horizontal and
	 * vertical decimation even though they may affect quality differently
	 * given the exact geometry involved.
	 *
	 * Also, since the clock calculations are abstracted, we cannot make
	 * assumptions on how decimation affects the clock rates in our search.
	 *
	 * We search the whole search region in increasing layers from
	 * min_factor to max_factor.  In each layer we search in increasing
	 * factors alternating between x and y axis:
	 *
	 *   x:	1	2	3
	 * y:
	 * 1	1st |	3rd |	6th |
	 *	----+	    |	    |
	 * 2	2nd	4th |	8th |
	 *	------------+	    |
	 * 3	5th	7th	9th |
	*	--------------------+
	 */
	min_factor = min(oi->min_x_decim, oi->min_y_decim);
	max_factor = max(oi->max_x_decim, oi->max_y_decim);
	x = oi->min_x_decim;
	y = oi->min_y_decim;
	while (1) {
		if (x < oi->min_x_decim || x > oi->max_x_decim ||
				y < oi->min_y_decim || y > oi->max_y_decim)
			goto loop;

		in_width = DIV_ROUND_UP(oi->width, x);
		in_height = DIV_ROUND_UP(oi->height, y);

		/* Use 5-tap filter unless must use 3-tap,
		 * even in no-scaling case, for not to lose sharpening filters
		 */
		if (!(cpu_is_omap44xx() || cpu_is_omap54xx()))
			*five_taps = in_width <= 1024;
		else if (omap_rev() == OMAP4430_REV_ES1_0)
			*five_taps = in_width <= 1280;
		else
			*five_taps = true;

		if (in_width == oi->out_width && in_height == oi->out_height)
			break;

		if (!can_scale)
			goto loop;

		if (oi->out_width * maxdownscale < in_width ||
				oi->out_height * maxdownscale < in_height)
			goto loop;

		/*
		 * Predecimation on OMAP4 still fetches the whole lines
		 * :TODO: How does it affect the required clock speed?
		 */
		fclk = calc_fclk(channel, in_width, in_height,
					oi->out_width, oi->out_height);
		fclk5 = *five_taps ?
			calc_fclk_five_taps(channel, in_width, in_height,
					oi->out_width, oi->out_height,
					oi->color_mode) : 0;

		DSSDBG("%d*%d,%d*%d->%d,%d requires %lu(3T), %lu(5T) Hz\n",
				in_width, x, in_height, y, oi->out_width,
				oi->out_height, fclk, fclk5);

		/* for now we always use 5-tap unless 3-tap is required */
		if (*five_taps)
			fclk = fclk5;

		/* OMAP2/3 has a scaler size limitation */
		if (!(cpu_is_omap44xx() || cpu_is_omap54xx()) && in_width >
				(1024 << !*five_taps))
			goto loop;

		DSSDBG("required fclk rate = %lu Hz\n", fclk);
		DSSDBG("current fclk rate = %lu Hz\n", fclk_max);

		if (fclk > fclk_max)
			goto loop;
		break;

loop:
		/* err if exhausted search region */
		if (x == oi->max_x_decim && y == oi->max_y_decim) {
			DSSERR("failed to set up scaling %u*%u to %u*%u, "
				"required fclk rate = %lu Hz, current = %lu Hz"
				"\n", oi->width, oi->height, oi->out_width,
				oi->out_height, fclk, fclk_max);
			return -EINVAL;
		}

		/* get to next factor */
		if (x == y) {
			x = min_factor;
			y++;
		} else {
			swap(x, y);
			if (x < y)
				x++;
		}
	}

	*x_decim = x;
	*y_decim = y;
	return 0;
}

static int dispc_ovl_calc_scaling(enum omap_plane plane,
		enum omap_channel channel, u16 width, u16 height,
		u16 out_width, u16 out_height,
		enum omap_color_mode color_mode, bool *five_taps)
{
	struct omap_overlay *ovl = omap_dss_get_overlay(plane);
	const int maxdownscale = dss_feat_get_param_max(FEAT_PARAM_DOWNSCALE);

	if (width == out_width && height == out_height)
		return 0;

	if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0)
		return -EINVAL;

	if (out_width * maxdownscale < width)
		return -EINVAL;

	if (out_height * maxdownscale < height)
		return -EINVAL;

	return 0;
}

static void dispc_enable_arbitration(enum omap_plane plane, bool enable)
{
	DSSDBG("dispc_enable_arbitration %d, %d\n", plane, enable);

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable ? 1 : 0, 14, 14);

	return;
}

int dispc_ovl_setup(enum omap_plane plane, struct omap_overlay_info *oi,
		bool ilace, bool replication, int x_decim, int y_decim,
		bool five_taps, bool source_of_wb)
{
	struct omap_overlay *ovl = omap_dss_get_overlay(plane);
	bool fieldmode = 0;
	int r, cconv = 0;
	unsigned offset0, offset1;
	s32 row_inc;
	s32 pix_inc;
	u16 frame_height = oi->height;
	int pixpg = 1;
	u32 orig_width, orig_height;
	unsigned int field_offset = 0;
	u8 global_alpha;
	u16 outw, outh;
	enum omap_channel channel;
	unsigned long flags;
	u32 udf_mask;

	channel = dispc_ovl_get_channel_out(plane);

	DSSDBG("dispc_ovl_setup %d, pa %x, pa_uv %x", plane, oi->paddr,
			oi->p_uv_addr);
	DSSDBG(" sw %d, %d,%d, ", oi->screen_width, oi->pos_x, oi->pos_y);
	DSSDBG("%dx%d -> %dx%d, ", oi->width, oi->height, oi->out_width,
			oi->out_height);
	DSSDBG("decim %dx%d, cmode %x, ", x_decim, y_decim, oi->color_mode);
	DSSDBG("rot %d, mir %d, ", oi->rotation, oi->mirror);
	DSSDBG("ilace %d chan %d repl %d\n", ilace, channel, replication);

	if (oi->paddr == 0)
		return -EINVAL;

	outw = oi->out_width == 0 ? oi->width : oi->out_width;
	outh = oi->out_height == 0 ? oi->height : oi->out_height;
	if (outw == 0 || outh == 0)
		return -EINVAL;

	/*
	 * Make sure UDF is disabled for ovls that are source for WB -
	 * Errata i642 for OMAP4.
	 */

	if (!cpu_is_omap44xx())
		goto skip_errata;

	switch (plane) {
	case OMAP_DSS_GFX:
		udf_mask = DISPC_IRQ_GFX_FIFO_UNDERFLOW;
		break;
	case OMAP_DSS_VIDEO1:
		udf_mask = DISPC_IRQ_VID1_FIFO_UNDERFLOW;
		break;
	case OMAP_DSS_VIDEO2:
		udf_mask = DISPC_IRQ_VID2_FIFO_UNDERFLOW;
		break;
	case OMAP_DSS_VIDEO3:
		udf_mask = DISPC_IRQ_VID3_FIFO_UNDERFLOW;
		break;
	default:
		udf_mask = 0;
	}

	if (source_of_wb) {
		/* if the ovl UDF is enabled, disable it */
		if (dispc.irq_error_mask & udf_mask) {
			DSSDBG("%s: disable irq irq_error_mask:0x%x \
				mask:0x%x\n", __func__, dispc.irq_error_mask,
								udf_mask);
			spin_lock_irqsave(&dispc.irq_lock, flags);
			dispc.irq_error_mask &= ~udf_mask;
			_omap_dispc_set_irqs();
			spin_unlock_irqrestore(&dispc.irq_lock, flags);
		}
	} else {
		/* enable UDF if irq_error_mask needs it and its currently
		 * disabled */
		if (!(dispc.irq_error_mask & udf_mask)) {
			DSSDBG("%s: enable irq irq_error_mask:0x%x \
				mask:0x%x\n", __func__, dispc.irq_error_mask,
								udf_mask);
			spin_lock_irqsave(&dispc.irq_lock, flags);
			dispc.irq_error_mask |= udf_mask;
			_omap_dispc_set_irqs();
			spin_unlock_irqrestore(&dispc.irq_lock, flags);
		}
	}

skip_errata:

	if (ilace && oi->height == outh)
		fieldmode = 1;

	if (ilace) {
		if (fieldmode)
			oi->height /= 2;
		oi->pos_y /= 2;
		outh /= 2;

		DSSDBG("adjusting for ilace: height %d, pos_y %d, "
				"out_height %d\n",
				oi->height, oi->pos_y, outh);
	}

	if (!dss_feat_color_mode_supported(plane, oi->color_mode))
		return -EINVAL;

	/* predecimate */

	/* adjust for group-of-pixels*/
	if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE) &&
	    (oi->color_mode == OMAP_DSS_COLOR_YUV2 ||
	     oi->color_mode == OMAP_DSS_COLOR_UYVY))
		pixpg = 2;

	if (oi->rotation & 1)
		oi->height /= pixpg;
	else
		oi->width /= pixpg;

	/* we need to remember the original block's size for rotations */
	orig_width  = oi->width;
	orig_height = oi->height;

	oi->width = DIV_ROUND_UP(oi->width, x_decim);
	oi->height = DIV_ROUND_UP(oi->height, y_decim);

	/* NV12 width has to be even (height apparently does not) */
	if (oi->color_mode == OMAP_DSS_COLOR_NV12)
		oi->width &= ~1;

	r = dispc_ovl_calc_scaling(plane, channel, oi->width, oi->height,
			outw, outh, oi->color_mode,
			&five_taps);
	if (r)
		return r;

	if (oi->color_mode == OMAP_DSS_COLOR_YUV2 ||
			oi->color_mode == OMAP_DSS_COLOR_UYVY ||
			oi->color_mode == OMAP_DSS_COLOR_NV12)
		cconv = 1;

	if (ilace && !fieldmode) {
		/*
		 * when downscaling the bottom field may have to start several
		 * source lines below the top field. Unfortunately ACCUI
		 * registers will only hold the fractional part of the offset
		 * so the integer part must be added to the base address of the
		 * bottom field.
		 */
		if (!oi->height || oi->height == outh)
			field_offset = 0;
		else
			field_offset = oi->height / outh / 2;
	}

	/* Fields are independent but interleaved in memory. */
	if (fieldmode)
		field_offset = 1;

	/* default values */
	row_inc = 0x1;
	pix_inc = 0x1;
	offset0 = 0x0;
	offset1 = 0x0;

	/*
	 * :HACK: we piggy back on UV separate feature for TILER to avoid
	 * having to keep rebase our FEAT_ enum until they add TILER.
	 */
	if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
		/* set BURSTTYPE */
		bool use_tiler = oi->rotation_type == OMAP_DSS_ROT_TILER;
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), use_tiler, 29, 29);
	}

	if (oi->rotation_type == OMAP_DSS_ROT_TILER) {
		struct tiler_view_t view = {0};
		u16 tiler_width = orig_width, tiler_height = orig_height;
		int bpp = color_mode_to_bpp(oi->color_mode) / 8;
		/* tiler needs 0-degree width & height */
		if (oi->rotation & 1)
			swap(tiler_width, tiler_height);

		tilview_create(&view, oi->paddr, tiler_width, tiler_height);
		tilview_rotate(&view, oi->rotation * 90);
		tilview_flip(&view, oi->mirror, false);
		oi->paddr = view.tsptr;

		/* we cannot do TB field interlaced in rotated view */
		pix_inc = 1 + (x_decim - 1) * bpp * pixpg;
		calc_tiler_row_rotation(&view, oi->width * x_decim, bpp * pixpg,
					y_decim, &row_inc, &offset1, ilace);
		DSSDBG("w, h = %u %u\n", tiler_width, tiler_height);

		if (oi->p_uv_addr) {
			tilview_create(&view, oi->p_uv_addr, tiler_width / 2,
					tiler_height / 2);
			tilview_rotate(&view, oi->rotation * 90);
			tilview_flip(&view, oi->mirror, false);
			oi->p_uv_addr = view.tsptr;
		}

	} else if (oi->rotation_type == OMAP_DSS_ROT_DMA) {
		calc_dma_rotation_offset(oi->rotation, oi->mirror,
				oi->screen_width, oi->width, frame_height,
				oi->color_mode, fieldmode, field_offset,
				&offset0, &offset1, &row_inc, &pix_inc,
				x_decim, y_decim);
	} else {
		calc_vrfb_rotation_offset(oi->rotation, oi->mirror,
				oi->screen_width, oi->width, frame_height,
				oi->color_mode, fieldmode, field_offset,
				&offset0, &offset1, &row_inc, &pix_inc);
	}

	DSSDBG("offset0 %u, offset1 %u, row_inc %d, pix_inc %d\n",
			offset0, offset1, row_inc, pix_inc);

	/* adjust back to pixels */
	if (oi->rotation & 1)
		oi->height *= pixpg;
	else
		oi->width *= pixpg;

	dispc_ovl_set_color_mode(plane, oi->color_mode);

	dispc_ovl_set_ba0(plane, oi->paddr + offset0);
	dispc_ovl_set_ba1(plane, oi->paddr + offset1);

	if (OMAP_DSS_COLOR_NV12 == oi->color_mode) {
		dispc_ovl_set_ba0_uv(plane, oi->p_uv_addr + offset0);
		dispc_ovl_set_ba1_uv(plane, oi->p_uv_addr + offset1);
	}

	/* Force 1D tiled mode */
	if (ovl->caps & OMAP_DSS_OVL_CAP_FORCE_1D)
		dispc_ovl_set_1d_tiled_mode(plane, oi->force_1d);

	if (dss_has_feature(FEAT_MFLAG)) {
		dispc_ovl_set_global_mflag(ovl->id, oi->mflag_en);
	} else if (plane == OMAP_DSS_GFX) {
		dispc_enable_arbitration(plane,
					channel == OMAP_DSS_CHANNEL_DIGIT);
	}

	dispc_ovl_set_row_inc(plane, row_inc);
	dispc_ovl_set_pix_inc(plane, pix_inc);

	DSSDBG("%d,%d %dx%d -> %dx%d\n", oi->pos_x, oi->pos_y, oi->width,
			oi->height, outw, outh);

	dispc_ovl_set_pos(plane, oi->pos_x, oi->pos_y);

	dispc_ovl_set_pic_size(plane, oi->width, oi->height);

	if (ovl->caps & OMAP_DSS_OVL_CAP_SCALE) {
		dispc_ovl_set_scaling(plane, oi->width, oi->height,
				   outw, outh,
				   ilace, five_taps, fieldmode,
				   oi->color_mode, oi->rotation);
		dispc_ovl_set_vid_size(plane, outw, outh);
		dispc_ovl_set_vid_color_conv(plane, cconv);
	}

	dispc_ovl_set_rotation_attrs(plane, oi->rotation, oi->mirror,
			oi->color_mode, oi->rotation_type);

	dispc_ovl_set_zorder(plane, oi->zorder);
	dispc_ovl_set_pre_mult_alpha(plane, oi->pre_mult_alpha);

	/* Errata OMAP5 ES1.0: Premultiply alpha global condition issue */
	global_alpha = oi->global_alpha;
	if ((omap_rev() == OMAP5430_REV_ES1_0 ||
			omap_rev() == OMAP5432_REV_ES1_0)
			&& oi->pre_mult_alpha &&
			global_alpha == 0xFF &&	!cconv) {
		if (oi->color_mode == OMAP_DSS_COLOR_ARGB16 ||
		    oi->color_mode == OMAP_DSS_COLOR_ARGB32 ||
		    oi->color_mode == OMAP_DSS_COLOR_ARGB16_1555 ||
		    oi->color_mode == OMAP_DSS_COLOR_RGBA16 ||
		    oi->color_mode == OMAP_DSS_COLOR_RGBA32)
			global_alpha--;
	}

	dispc_ovl_setup_global_alpha(plane, global_alpha);

	dispc_ovl_enable_replication(plane, replication);

	if (plane != OMAP_DSS_GFX)
		dispc_mgr_setup_color_conv_coef(plane, &oi->cconv);

	if (plane == OMAP_DSS_GFX) {
		if (channel == OMAP_DSS_CHANNEL_DIGIT ||
			omap_rev() == OMAP5430_REV_ES1_0 ||
			omap_rev() == OMAP5432_REV_ES1_0)
			dispc_enable_arbitration(plane, true);
		else
			dispc_enable_arbitration(plane, false);
	}

	return 0;
}

int dispc_ovl_enable(enum omap_plane plane, bool enable)
{
	DSSDBG("dispc_ovl_enable %d, %d\n", plane, enable);

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable ? 1 : 0, 0, 0);

	return 0;
}

void dispc_setup_wb_source(enum omap_writeback_source source)
{
	/* configure wb source */
	if (source == OMAP_WB_TV)
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(OMAP_DSS_WB), 2, 18, 16);
	else if (source == OMAP_WB_LCD2)
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(OMAP_DSS_WB), 1, 18, 16);
	else
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(OMAP_DSS_WB), source, 18, 16);
}

/* Writeback*/
int dispc_setup_wb(struct writeback_cache_data *wb)
{
	u8 rotation = wb->rotation, mirror = 0;
	unsigned long tiler_width, tiler_height;
	u32 paddr = wb->paddr;
	u32 puv_addr = wb->p_uv_addr; /* relevant for NV12 format only */
	u16 out_width = wb->out_width;
	u16 out_height = wb->out_height;
	u16 width = wb->width;
	u16 height = wb->height;
	int x_decim = 1, y_decim = 1;
	bool ilace = false, fieldmode = false;
	unsigned int offset0 = 0, offset1 = 0;

	enum omap_color_mode color_mode = wb->color_mode;  /* output color */
	int pixpg = (color_mode &
		(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)) ? 2 : 1;

	u32 fifo_low = wb->fifo_low;
	u32 fifo_high = wb->fifo_high;
	enum omap_writeback_source source = wb->source;

	enum omap_plane plane = OMAP_DSS_WB;

	int maxdownscale = 4;
	bool five_taps = 1;
	int cconv = 0;
	s32 row_inc;
	s32 pix_inc;

	DSSDBG("dispc_setup_wb\n");
	DSSDBG("Maxds = %d\n", maxdownscale);
	DSSDBG("out_width, width = %d, %d\n", (int) out_width, (int) width);
	DSSDBG("out_height, height = %d, %d\n", (int) out_height, (int) height);

	if (paddr == 0) {
		printk(KERN_ERR "dispc_setup_wb paddr NULL\n");
		return -EINVAL;
	}

	switch (color_mode) {
	case OMAP_DSS_COLOR_NV12:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		cconv = 1;
	default:
		/* check for invalid color format happens later */
		break;
	}

	/* color conversion to NV12 reduces scaling factor */
	if (cconv == 1 && color_mode == OMAP_DSS_COLOR_NV12)
		maxdownscale = 2;

	dispc_setup_wb_source(source);

	/* change resolution of manager if it works in MEM2MEM mode */
	if (wb->mode == OMAP_WB_MEM2MEM_MODE) {
		if (source == OMAP_WB_TV)
			dispc_set_digit_size(out_width, out_height);
		else if (source == OMAP_WB_LCD2)
			dispc_mgr_set_lcd_size(OMAP_DSS_CHANNEL_LCD2, out_width,
								out_height);
		else if (source == OMAP_WB_LCD1)
			dispc_mgr_set_lcd_size(OMAP_DSS_CHANNEL_LCD, out_width,
								out_height);
	}

	/*
	 * configure wb mode:
	 * 0-capture-mode; 1-memory-to-memory mode
	 */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), wb->mode, 19, 19);

	/* predecimate */
	/* adjust for group-of-pixels*/
	if (rotation & 1)
		height /= pixpg;
	else
		width /= pixpg;

	tiler_width = out_width;
	tiler_height = out_height;

	/* NV12 width has to be even (height apparently does not) */
	if (color_mode == OMAP_DSS_COLOR_NV12)
		out_width &= ~1;

	/* validate scaling */
	if (out_width < width / maxdownscale ||
			out_width > width) {
		printk(KERN_ERR "dispc_setup_wb out_width not in range\n");
		return -EINVAL;
	}

	if (out_height < height / maxdownscale ||
			out_height > height){
		printk(KERN_ERR "dispc_setup_wb out_height not in range\n");
		return -EINVAL;
	}

	pix_inc = 0x1;
	if ((paddr >= 0x60000000) && (paddr <= 0x7fffffff)) {
		struct tiler_view_t view = {0};
		int bpp = color_mode_to_bpp(color_mode) / 8;

		/* tiler needs 0-degree width & height */
		if (rotation & 1)
			swap(tiler_width, tiler_height);

		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
		    color_mode == OMAP_DSS_COLOR_UYVY)
			tiler_width /= 2;

		tilview_create(&view, paddr, tiler_width, tiler_height);
		tilview_rotate(&view, rotation * 90);
		tilview_flip(&view, mirror, false);
		paddr = view.tsptr;

		/* we cannot do TB field interlaced in rotated view */
		calc_tiler_row_rotation(&view, out_width*x_decim, bpp * pixpg,
				y_decim, &row_inc, &offset1, ilace);

		DSSDBG("w, h = %ld,%ld\n", tiler_width, tiler_height);

		if (puv_addr) {
			tilview_create(&view, puv_addr, tiler_width / 2,
					       tiler_height / 2);
			tilview_rotate(&view, rotation * 90);
			tilview_flip(&view, mirror, false);
			puv_addr = view.tsptr;
		}

		DSSDBG("rotated addresses: 0x%0x, 0x%0x\n",
						paddr, puv_addr);
		/* set BURSTTYPE if rotation is non-zero */
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), 0x1, 8, 8);
	} else
		row_inc = 1;
	/* adjust back to pixels */
	if (rotation & 1)
		height *= pixpg;
	else
		width *= pixpg;

	DSSDBG("offset0 %u, offset1 %u, row_inc %d, pix_inc %d\n",
			offset0, offset1, row_inc, pix_inc);

	dispc_ovl_set_color_mode(plane, color_mode);

	dispc_ovl_set_ba0(plane, paddr + offset0);
	dispc_ovl_set_ba1(plane, paddr + offset1);

	if (OMAP_DSS_COLOR_NV12 == color_mode) {
		dispc_ovl_set_ba0_uv(plane, puv_addr + offset0);
		dispc_ovl_set_ba1_uv(plane, puv_addr + offset1);
	}

	dispc_ovl_set_rotation_attrs(plane, rotation, mirror,
				color_mode, wb->rotation_type);

	/* Force 1D tiled mode */
#if 0
	if (cpu_is_omap54xx() && (omap_rev() == OMAP5430_REV_ES2_0 ||
				  omap_rev() == OMAP5432_REV_ES2_0))
		dispc_ovl_set_1d_tiled_mode(plane, wb->force_1d);
#endif

	dispc_ovl_set_row_inc(plane, row_inc);
	dispc_ovl_set_pix_inc(plane, pix_inc);

	DSSDBG("%dx%d -> %p,%p %dx%d\n",
	       width, height,
	       (void *)paddr, (void *)puv_addr, out_width, out_height);

	/*WB PIC_SIZE is the final destination size*/
	dispc_ovl_set_pic_size(plane, out_width, out_height);
	/*WB size is the size delivered by the manager*/
	dispc_ovl_set_vid_size(plane, width, height);

	dispc_ovl_set_fifo_threshold(plane, fifo_low, fifo_high);

	dispc_ovl_set_scaling(plane,
			width, height,
			out_width, out_height,
			ilace, five_taps,
			fieldmode, wb->color_mode,
			rotation);
	/* configure wb burst size  */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), wb->burst_size, 15, 14);
	/*
	 * configure delay of 'n' lines after framedone for WB pipe flush
	 * TODO: delay value of 3 is empirical (working for all scenarios)
	 * need to work out value based on pix.clock, FIFO HT, etc
	 */
	if (wb->mode == OMAP_WB_CAPTURE_MODE)
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES2(plane), 0x3, 7, 0);

	/* TODO: need correct calculation for truncation bit */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), 0x0, 10, 10);

	if (cconv)
		dispc_mgr_setup_wb_color_conv_coef();

	dispc_ovl_set_vid_color_conv(plane, cconv);

	return 0;
}

void dispc_go_wb()
{
	if (REG_GET(DISPC_CONTROL2, 6, 6)) {
		DSSERR("GO bit already set for WB\n");
		return;
	}

	REG_FLD_MOD(DISPC_CONTROL2, 1, 6, 6);
	DSSDBG("dispc_go_wb\n");
}

static void dispc_disable_isr(void *data, u32 mask)
{
	struct completion *compl = data;
	complete(compl);
}

void dispc_enable_lcd_out(enum omap_channel channel, bool enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD2) {
		REG_FLD_MOD(DISPC_CONTROL2, enable ? 1 : 0, 0, 0);
		/* flush posted write */
		dispc_read_reg(DISPC_CONTROL2);
	} else {
		REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 0, 0);
		dispc_read_reg(DISPC_CONTROL);
	}
}

static void dispc_mgr_enable_lcd_out(enum omap_channel channel, bool enable)
{
	struct completion frame_done_completion;
	bool is_on;
	int r;
	u32 irq;

	/* When we disable LCD output, we need to wait until frame is done.
	 * Otherwise the DSS is still working, and turning off the clocks
	 * prevents DSS from going to OFF mode */
	is_on = channel == OMAP_DSS_CHANNEL_LCD2 ?
			REG_GET(DISPC_CONTROL2, 0, 0) :
			REG_GET(DISPC_CONTROL, 0, 0);

	irq = channel == OMAP_DSS_CHANNEL_LCD2 ? DISPC_IRQ_FRAMEDONE2 :
			DISPC_IRQ_FRAMEDONE;

	if (!enable && is_on) {
		init_completion(&frame_done_completion);

		r = omap_dispc_register_isr(dispc_disable_isr,
				&frame_done_completion, irq);

		if (r)
			DSSERR("failed to register FRAMEDONE isr\n");
	}

	if (!(cpu_is_omap54xx() && ((omap_rev() <= OMAP5430_REV_ES1_0) ||
		(omap_rev() == OMAP5432_REV_ES1_0))))
		/*
		 * For OMAP5 ES1.0, _enable_lcd_out is done as part of
		 * dsi_handle_lcd_en_timing_pre() and
		 * dsi_handle_lcd_en_timing_post()
		 */
		dispc_enable_lcd_out(channel, enable);

	if (!enable && is_on) {
		if (!wait_for_completion_timeout(&frame_done_completion,
					msecs_to_jiffies(100)))
			DSSERR("timeout waiting for FRAME DONE\n");

		r = omap_dispc_unregister_isr_sync(dispc_disable_isr,
				&frame_done_completion, irq);

		if (r)
			DSSERR("failed to unregister FRAMEDONE isr\n");
	}
}

static void _enable_digit_out(bool enable)
{
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 1, 1);
	/* flush posted write */
	dispc_read_reg(DISPC_CONTROL);
}

static void dispc_mgr_enable_digit_out(bool enable)
{
	struct completion frame_done_completion;
	enum dss_hdmi_venc_clk_source_select src;
	int r, i;
	u32 irq_mask;
	int num_irqs;

	if (REG_GET(DISPC_CONTROL, 1, 1) == enable)
		return;

	src = dss_get_hdmi_venc_clk_source();

	if (enable) {
		unsigned long flags;
		/* When we enable digit output, we'll get an extra digit
		 * sync lost interrupt, that we need to ignore */
		spin_lock_irqsave(&dispc.irq_lock, flags);
		dispc.irq_error_mask &= ~DISPC_IRQ_SYNC_LOST_DIGIT;
		_omap_dispc_set_irqs();
		spin_unlock_irqrestore(&dispc.irq_lock, flags);
	}

	/* When we disable digit output, we need to wait until fields are done.
	 * Otherwise the DSS is still working, and turning off the clocks
	 * prevents DSS from going to OFF mode. And when enabling, we need to
	 * wait for the extra sync losts */
	init_completion(&frame_done_completion);

	if (src == DSS_HDMI_M_PCLK && enable == false) {
		irq_mask = DISPC_IRQ_FRAMEDONETV;
		num_irqs = 1;
	} else {
		irq_mask = DISPC_IRQ_EVSYNC_EVEN | DISPC_IRQ_EVSYNC_ODD;
		/* XXX I understand from TRM that we should only wait for the
		 * current field to complete. But it seems we have to wait for
		 * both fields */
		num_irqs = 2;
	}

	r = omap_dispc_register_isr(dispc_disable_isr, &frame_done_completion,
			irq_mask);
	if (r)
		DSSERR("failed to register %x isr\n", irq_mask);

	_enable_digit_out(enable);

	for (i = 0; i < num_irqs; ++i) {
		if (!wait_for_completion_timeout(&frame_done_completion,
					msecs_to_jiffies(100)))
			DSSERR("timeout waiting for digit out to %s\n",
					enable ? "start" : "stop");
	}

	r = omap_dispc_unregister_isr_sync(dispc_disable_isr,
			&frame_done_completion, irq_mask);
	if (r)
		DSSERR("failed to unregister %x isr\n", irq_mask);

	if (enable) {
		unsigned long flags;
		spin_lock_irqsave(&dispc.irq_lock, flags);
		dispc.irq_error_mask |= DISPC_IRQ_SYNC_LOST_DIGIT;
		dispc_write_reg(DISPC_IRQSTATUS, DISPC_IRQ_SYNC_LOST_DIGIT);
		_omap_dispc_set_irqs();
		spin_unlock_irqrestore(&dispc.irq_lock, flags);
	}
}

bool dispc_mgr_is_enabled(enum omap_channel channel)
{
	if (channel == OMAP_DSS_CHANNEL_LCD)
		return !!REG_GET(DISPC_CONTROL, 0, 0);
	else if (channel == OMAP_DSS_CHANNEL_DIGIT)
		return !!REG_GET(DISPC_CONTROL, 1, 1);
	else if (channel == OMAP_DSS_CHANNEL_LCD2)
		return !!REG_GET(DISPC_CONTROL2, 0, 0);
	else
		BUG();
}

void dispc_mgr_enable(enum omap_channel channel, bool enable)
{
	if (dispc_mgr_is_lcd(channel))
		dispc_mgr_enable_lcd_out(channel, enable);
	else if (channel == OMAP_DSS_CHANNEL_DIGIT)
		dispc_mgr_enable_digit_out(enable);
	else
		BUG();
}

void dispc_lcd_enable_signal_polarity(bool act_high)
{
	if (!dss_has_feature(FEAT_LCDENABLEPOL))
		return;

	REG_FLD_MOD(DISPC_CONTROL, act_high ? 1 : 0, 29, 29);
}

void dispc_lcd_enable_signal(bool enable)
{
	if (!dss_has_feature(FEAT_LCDENABLESIGNAL))
		return;

	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 28, 28);
}

void dispc_pck_free_enable(bool enable)
{
	if (!dss_has_feature(FEAT_PCKFREEENABLE))
		return;

	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 27, 27);
}

void dispc_mgr_enable_fifohandcheck(enum omap_channel channel, bool enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONFIG2, enable ? 1 : 0, 16, 16);
	else
		REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 16, 16);
}


void dispc_mgr_set_lcd_display_type(enum omap_channel channel,
		enum omap_lcd_display_type type)
{
	int mode;

	switch (type) {
	case OMAP_DSS_LCD_DISPLAY_STN:
		mode = 0;
		break;

	case OMAP_DSS_LCD_DISPLAY_TFT:
		mode = 1;
		break;

	default:
		BUG();
		return;
	}

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, mode, 3, 3);
	else
		REG_FLD_MOD(DISPC_CONTROL, mode, 3, 3);
}

void dispc_set_loadmode(enum omap_dss_load_mode mode)
{
	REG_FLD_MOD(DISPC_CONFIG, mode, 2, 1);
}


static void dispc_mgr_set_default_color(enum omap_channel channel, u32 color)
{
	dispc_write_reg(DISPC_DEFAULT_COLOR(channel), color);
}

static void dispc_mgr_set_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type type,
		u32 trans_key)
{
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, type, 11, 11);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		REG_FLD_MOD(DISPC_CONFIG, type, 13, 13);
	else /* OMAP_DSS_CHANNEL_LCD2 */
		REG_FLD_MOD(DISPC_CONFIG2, type, 11, 11);

	dispc_write_reg(DISPC_TRANS_COLOR(ch), trans_key);
}

static void dispc_mgr_enable_trans_key(enum omap_channel ch, bool enable)
{
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 10, 10);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		REG_FLD_MOD(DISPC_CONFIG, enable, 12, 12);
	else /* OMAP_DSS_CHANNEL_LCD2 */
		REG_FLD_MOD(DISPC_CONFIG2, enable, 10, 10);
}

static void dispc_mgr_enable_alpha_fixed_zorder(enum omap_channel ch,
		bool enable)
{
	if (!dss_has_feature(FEAT_ALPHA_FIXED_ZORDER))
		return;

	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 18, 18);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		REG_FLD_MOD(DISPC_CONFIG, enable, 19, 19);
}

void dispc_mgr_setup(enum omap_channel channel,
		struct omap_overlay_manager_info *info)
{
	dispc_mgr_set_default_color(channel, info->default_color);
	dispc_mgr_set_trans_key(channel, info->trans_key_type, info->trans_key);
	dispc_mgr_enable_trans_key(channel, info->trans_enabled);
	dispc_mgr_enable_alpha_fixed_zorder(channel,
			info->partial_alpha_enabled);
	if (dss_has_feature(FEAT_CPR)) {
		dispc_mgr_enable_cpr(channel, info->cpr_enable);
		dispc_mgr_set_cpr_coef(channel, &info->cpr_coefs);
	}
}

void dispc_mgr_set_tft_data_lines(enum omap_channel channel, u8 data_lines)
{
	int code;

	switch (data_lines) {
	case 12:
		code = 0;
		break;
	case 16:
		code = 1;
		break;
	case 18:
		code = 2;
		break;
	case 24:
		code = 3;
		break;
	default:
		BUG();
		return;
	}

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, code, 9, 8);
	else
		REG_FLD_MOD(DISPC_CONTROL, code, 9, 8);
}

void dispc_mgr_set_io_pad_mode(enum dss_io_pad_mode mode)
{
	u32 l;
	int gpout0, gpout1;

	switch (mode) {
	case DSS_IO_PAD_MODE_RESET:
		gpout0 = 0;
		gpout1 = 0;
		break;
	case DSS_IO_PAD_MODE_RFBI:
		gpout0 = 1;
		gpout1 = 0;
		break;
	case DSS_IO_PAD_MODE_BYPASS:
		gpout0 = 1;
		gpout1 = 1;
		break;
	default:
		BUG();
		return;
	}

	l = dispc_read_reg(DISPC_CONTROL);
	l = FLD_MOD(l, gpout0, 15, 15);
	l = FLD_MOD(l, gpout1, 16, 16);
	dispc_write_reg(DISPC_CONTROL, l);
}

void dispc_mgr_enable_stallmode(enum omap_channel channel, bool enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, enable, 11, 11);
	else
		REG_FLD_MOD(DISPC_CONTROL, enable, 11, 11);
}

static bool _dispc_lcd_timings_ok(int hsw, int hfp, int hbp,
		int vsw, int vfp, int vbp)
{
	if (cpu_is_omap24xx() || omap_rev() < OMAP3430_REV_ES3_0) {
		if (hsw < 1 || hsw > 64 ||
				hfp < 1 || hfp > 256 ||
				hbp < 1 || hbp > 256 ||
				vsw < 1 || vsw > 64 ||
				vfp < 0 || vfp > 255 ||
				vbp < 0 || vbp > 255)
			return false;
	} else {
		if (hsw < 1 || hsw > 256 ||
				hfp < 1 || hfp > 4096 ||
				hbp < 1 || hbp > 4096 ||
				vsw < 1 || vsw > 256 ||
				vfp < 0 || vfp > 4095 ||
				vbp < 0 || vbp > 4095)
			return false;
	}

	return true;
}

bool dispc_lcd_timings_ok(struct omap_video_timings *timings)
{
	return _dispc_lcd_timings_ok(timings->hsw, timings->hfp,
			timings->hbp, timings->vsw,
			timings->vfp, timings->vbp);
}

static void _dispc_mgr_set_lcd_timings(enum omap_channel channel, int hsw,
		int hfp, int hbp, int vsw, int vfp, int vbp)
{
	u32 timing_h, timing_v;

	if (cpu_is_omap24xx() || omap_rev() < OMAP3430_REV_ES3_0) {
		timing_h = FLD_VAL(hsw-1, 5, 0) | FLD_VAL(hfp-1, 15, 8) |
			FLD_VAL(hbp-1, 27, 20);

		timing_v = FLD_VAL(vsw-1, 5, 0) | FLD_VAL(vfp, 15, 8) |
			FLD_VAL(vbp, 27, 20);
	} else {
		timing_h = FLD_VAL(hsw-1, 7, 0) | FLD_VAL(hfp-1, 19, 8) |
			FLD_VAL(hbp-1, 31, 20);

		timing_v = FLD_VAL(vsw-1, 7, 0) | FLD_VAL(vfp, 19, 8) |
			FLD_VAL(vbp, 31, 20);
	}

	dispc_write_reg(DISPC_TIMING_H(channel), timing_h);
	dispc_write_reg(DISPC_TIMING_V(channel), timing_v);
}

/* change name to mode? */
void dispc_mgr_set_lcd_timings(enum omap_channel channel,
		struct omap_video_timings *timings)
{
	unsigned xtot, ytot;
	unsigned long ht, vt;

	if (!_dispc_lcd_timings_ok(timings->hsw, timings->hfp,
				timings->hbp, timings->vsw,
				timings->vfp, timings->vbp))
		BUG();

	_dispc_mgr_set_lcd_timings(channel, timings->hsw, timings->hfp,
			timings->hbp, timings->vsw, timings->vfp,
			timings->vbp);

	dispc_mgr_set_lcd_size(channel, timings->x_res, timings->y_res);

	xtot = timings->x_res + timings->hfp + timings->hsw + timings->hbp;
	ytot = timings->y_res + timings->vfp + timings->vsw + timings->vbp;

	ht = (timings->pixel_clock * 1000) / xtot;
	vt = (timings->pixel_clock * 1000) / xtot / ytot;

	DSSDBG("channel %d xres %u yres %u\n", channel, timings->x_res,
			timings->y_res);
	DSSDBG("pck %u\n", timings->pixel_clock);
	DSSDBG("hsw %d hfp %d hbp %d vsw %d vfp %d vbp %d\n",
			timings->hsw, timings->hfp, timings->hbp,
			timings->vsw, timings->vfp, timings->vbp);

	DSSDBG("hsync %luHz, vsync %luHz\n", ht, vt);
}

static void dispc_mgr_set_lcd_divisor(enum omap_channel channel, u16 lck_div,
		u16 pck_div)
{
	BUG_ON(lck_div < 1);
	BUG_ON(pck_div < 1);

	dispc_write_reg(DISPC_DIVISORo(channel),
			FLD_VAL(lck_div, 23, 16) | FLD_VAL(pck_div, 7, 0));
}

static void dispc_mgr_get_lcd_divisor(enum omap_channel channel, int *lck_div,
		int *pck_div)
{
	u32 l;
	l = dispc_read_reg(DISPC_DIVISORo(channel));
	*lck_div = FLD_GET(l, 23, 16);
	*pck_div = FLD_GET(l, 7, 0);
}

unsigned long dispc_fclk_rate(void)
{
	struct platform_device *dsidev;
	unsigned long r = 0;

	switch (dss_get_dispc_clk_source()) {
	case OMAP_DSS_CLK_SRC_FCK:
		r = clk_get_rate(dispc.dss_clk);
		break;
	case OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(0);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	case OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(1);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	default:
		BUG();
	}

	return r;
}

unsigned long dispc_mgr_lclk_rate(enum omap_channel channel)
{
	struct platform_device *dsidev;
	int lcd;
	unsigned long r;
	u32 l;

	l = dispc_read_reg(DISPC_DIVISORo(channel));

	lcd = FLD_GET(l, 23, 16);

	switch (dss_get_lcd_clk_source(channel)) {
	case OMAP_DSS_CLK_SRC_FCK:
		r = clk_get_rate(dispc.dss_clk);
		break;
	case OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(0);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	case OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(1);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	default:
		BUG();
	}

	return r / lcd;
}

unsigned long dispc_mgr_pclk_rate(enum omap_channel channel)
{
	unsigned long r;

	if (dispc_mgr_is_lcd(channel)) {
		int pcd;
		u32 l;

		l = dispc_read_reg(DISPC_DIVISORo(channel));

		pcd = FLD_GET(l, 7, 0);

		r = dispc_mgr_lclk_rate(channel);

		return r / pcd;
	} else {
		struct omap_dss_device *dssdev =
			dispc_mgr_get_device(channel);

		switch (dssdev->type) {
		case OMAP_DISPLAY_TYPE_VENC:
			return venc_get_pixel_clock();
		case OMAP_DISPLAY_TYPE_HDMI:
			return hdmi_get_pixel_clock();
		default:
			BUG();
		}
	}
}

void dispc_dump_clocks(struct seq_file *s)
{
	int lcd, pcd;
	u32 l;
	enum omap_dss_clk_source dispc_clk_src = dss_get_dispc_clk_source();
	enum omap_dss_clk_source lcd_clk_src;

	if (dispc_runtime_get())
		return;

	seq_printf(s, "- DISPC -\n");

	seq_printf(s, "dispc fclk source = %s (%s)\n",
			dss_get_generic_clk_source_name(dispc_clk_src),
			dss_feat_get_clk_source_name(dispc_clk_src));

	seq_printf(s, "fck\t\t%-16lu\n", dispc_fclk_rate());

	if (dss_has_feature(FEAT_CORE_CLK_DIV)) {
		seq_printf(s, "- DISPC-CORE-CLK -\n");
		l = dispc_read_reg(DISPC_DIVISOR);
		lcd = FLD_GET(l, 23, 16);

		seq_printf(s, "lck\t\t%-16lulck div\t%u\n",
				(dispc_fclk_rate()/lcd), lcd);
	}
	seq_printf(s, "- LCD1 -\n");

	lcd_clk_src = dss_get_lcd_clk_source(OMAP_DSS_CHANNEL_LCD);

	seq_printf(s, "lcd1_clk source = %s (%s)\n",
		dss_get_generic_clk_source_name(lcd_clk_src),
		dss_feat_get_clk_source_name(lcd_clk_src));

	dispc_mgr_get_lcd_divisor(OMAP_DSS_CHANNEL_LCD, &lcd, &pcd);

	seq_printf(s, "lck\t\t%-16lulck div\t%u\n",
			dispc_mgr_lclk_rate(OMAP_DSS_CHANNEL_LCD), lcd);
	seq_printf(s, "pck\t\t%-16lupck div\t%u\n",
			dispc_mgr_pclk_rate(OMAP_DSS_CHANNEL_LCD), pcd);
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		seq_printf(s, "- LCD2 -\n");

		lcd_clk_src = dss_get_lcd_clk_source(OMAP_DSS_CHANNEL_LCD2);

		seq_printf(s, "lcd2_clk source = %s (%s)\n",
			dss_get_generic_clk_source_name(lcd_clk_src),
			dss_feat_get_clk_source_name(lcd_clk_src));

		dispc_mgr_get_lcd_divisor(OMAP_DSS_CHANNEL_LCD2, &lcd, &pcd);

		seq_printf(s, "lck\t\t%-16lulck div\t%u\n",
				dispc_mgr_lclk_rate(OMAP_DSS_CHANNEL_LCD2), lcd);
		seq_printf(s, "pck\t\t%-16lupck div\t%u\n",
				dispc_mgr_pclk_rate(OMAP_DSS_CHANNEL_LCD2), pcd);
	}

	dispc_runtime_put();
}

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
void dispc_dump_irqs(struct seq_file *s)
{
	unsigned long flags;
	struct dispc_irq_stats stats;

	spin_lock_irqsave(&dispc.irq_stats_lock, flags);

	stats = dispc.irq_stats;
	memset(&dispc.irq_stats, 0, sizeof(dispc.irq_stats));
	dispc.irq_stats.last_reset = jiffies;

	spin_unlock_irqrestore(&dispc.irq_stats_lock, flags);

	seq_printf(s, "period %u ms\n",
			jiffies_to_msecs(jiffies - stats.last_reset));

	seq_printf(s, "irqs %d\n", stats.irq_count);
#define PIS(x) \
	seq_printf(s, "%-20s %10d\n", #x, stats.irqs[ffs(DISPC_IRQ_##x)-1]);

	PIS(FRAMEDONE);
	PIS(VSYNC);
	PIS(EVSYNC_EVEN);
	PIS(EVSYNC_ODD);
	PIS(ACBIAS_COUNT_STAT);
	PIS(PROG_LINE_NUM);
	PIS(GFX_FIFO_UNDERFLOW);
	PIS(GFX_END_WIN);
	PIS(PAL_GAMMA_MASK);
	PIS(OCP_ERR);
	PIS(VID1_FIFO_UNDERFLOW);
	PIS(VID1_END_WIN);
	PIS(VID2_FIFO_UNDERFLOW);
	PIS(VID2_END_WIN);
	if (dss_feat_get_num_ovls() > 3) {
		PIS(VID3_FIFO_UNDERFLOW);
		PIS(VID3_END_WIN);
	}
	PIS(SYNC_LOST);
	PIS(SYNC_LOST_DIGIT);
	PIS(WAKEUP);
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		PIS(FRAMEDONE2);
		PIS(VSYNC2);
		PIS(ACBIAS_COUNT_STAT2);
		PIS(SYNC_LOST2);
	}
#undef PIS
}
#endif

void dispc_dump_regs(struct seq_file *s)
{
	int i, j;
	const char *mgr_names[] = {
		[OMAP_DSS_CHANNEL_LCD]		= "LCD",
		[OMAP_DSS_CHANNEL_DIGIT]	= "TV",
		[OMAP_DSS_CHANNEL_LCD2]		= "LCD2",
	};
	const char *ovl_names[] = {
		[OMAP_DSS_GFX]		= "GFX",
		[OMAP_DSS_VIDEO1]	= "VID1",
		[OMAP_DSS_VIDEO2]	= "VID2",
		[OMAP_DSS_VIDEO3]	= "VID3",
	};
	const char **p_names;

#define DUMPREG(r) seq_printf(s, "%-50s %08x\n", #r, dispc_read_reg(r))

	if (dispc_runtime_get())
		return;

	/* DISPC common registers */
	DUMPREG(DISPC_REVISION);
	DUMPREG(DISPC_SYSCONFIG);
	DUMPREG(DISPC_SYSSTATUS);
	DUMPREG(DISPC_IRQSTATUS);
	DUMPREG(DISPC_IRQENABLE);
	DUMPREG(DISPC_CONTROL);
	DUMPREG(DISPC_CONFIG);
	DUMPREG(DISPC_CAPABLE);
	DUMPREG(DISPC_LINE_STATUS);
	DUMPREG(DISPC_LINE_NUMBER);
	if (dss_has_feature(FEAT_ALPHA_FIXED_ZORDER) ||
			dss_has_feature(FEAT_ALPHA_FREE_ZORDER))
		DUMPREG(DISPC_GLOBAL_ALPHA);
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		DUMPREG(DISPC_CONTROL2);
		DUMPREG(DISPC_CONFIG2);
	}

#undef DUMPREG

#define DISPC_REG(i, name) name(i)
#define DUMPREG(i, r) seq_printf(s, "%s(%s)%*s %08x\n", #r, p_names[i], \
	48 - strlen(#r) - strlen(p_names[i]), " ", \
	dispc_read_reg(DISPC_REG(i, r)))

	p_names = mgr_names;

	/* DISPC channel specific registers */
	for (i = 0; i < dss_feat_get_num_mgrs(); i++) {
		DUMPREG(i, DISPC_DEFAULT_COLOR);
		DUMPREG(i, DISPC_TRANS_COLOR);
		DUMPREG(i, DISPC_SIZE_MGR);

		if (i == OMAP_DSS_CHANNEL_DIGIT)
			continue;

		DUMPREG(i, DISPC_DEFAULT_COLOR);
		DUMPREG(i, DISPC_TRANS_COLOR);
		DUMPREG(i, DISPC_TIMING_H);
		DUMPREG(i, DISPC_TIMING_V);
		DUMPREG(i, DISPC_POL_FREQ);
		DUMPREG(i, DISPC_DIVISORo);
		DUMPREG(i, DISPC_SIZE_MGR);

		DUMPREG(i, DISPC_DATA_CYCLE1);
		DUMPREG(i, DISPC_DATA_CYCLE2);
		DUMPREG(i, DISPC_DATA_CYCLE3);

		if (dss_has_feature(FEAT_CPR)) {
			DUMPREG(i, DISPC_CPR_COEF_R);
			DUMPREG(i, DISPC_CPR_COEF_G);
			DUMPREG(i, DISPC_CPR_COEF_B);
		}
	}

	p_names = ovl_names;

	for (i = 0; i < dss_feat_get_num_ovls(); i++) {
		DUMPREG(i, DISPC_OVL_BA0);
		DUMPREG(i, DISPC_OVL_BA1);
		DUMPREG(i, DISPC_OVL_POSITION);
		DUMPREG(i, DISPC_OVL_SIZE);
		DUMPREG(i, DISPC_OVL_ATTRIBUTES);
		DUMPREG(i, DISPC_OVL_FIFO_THRESHOLD);
		DUMPREG(i, DISPC_OVL_FIFO_SIZE_STATUS);
		DUMPREG(i, DISPC_OVL_ROW_INC);
		DUMPREG(i, DISPC_OVL_PIXEL_INC);
		if (dss_has_feature(FEAT_PRELOAD))
			DUMPREG(i, DISPC_OVL_PRELOAD);

		if (i == OMAP_DSS_GFX) {
			DUMPREG(i, DISPC_OVL_WINDOW_SKIP);
			DUMPREG(i, DISPC_OVL_TABLE_BA);
			continue;
		}

		DUMPREG(i, DISPC_OVL_FIR);
		DUMPREG(i, DISPC_OVL_PICTURE_SIZE);
		DUMPREG(i, DISPC_OVL_ACCU0);
		DUMPREG(i, DISPC_OVL_ACCU1);
		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			DUMPREG(i, DISPC_OVL_BA0_UV);
			DUMPREG(i, DISPC_OVL_BA1_UV);
			DUMPREG(i, DISPC_OVL_FIR2);
			DUMPREG(i, DISPC_OVL_ACCU2_0);
			DUMPREG(i, DISPC_OVL_ACCU2_1);
		}
		if (dss_has_feature(FEAT_ATTR2))
			DUMPREG(i, DISPC_OVL_ATTRIBUTES2);
		if (dss_has_feature(FEAT_PRELOAD))
			DUMPREG(i, DISPC_OVL_PRELOAD);
	}

#undef DISPC_REG
#undef DUMPREG

#define DISPC_REG(plane, name, i) name(plane, i)
#define DUMPREG(plane, name, i) \
	seq_printf(s, "%s_%d(%s)%*s %08x\n", #name, i, p_names[plane], \
	46 - strlen(#name) - strlen(p_names[plane]), " ", \
	dispc_read_reg(DISPC_REG(plane, name, i)))

	/* Video pipeline coefficient registers */

	/* start from OMAP_DSS_VIDEO1 */
	for (i = 1; i < dss_feat_get_num_ovls(); i++) {
		for (j = 0; j < 8; j++)
			DUMPREG(i, DISPC_OVL_FIR_COEF_H, j);

		for (j = 0; j < 8; j++)
			DUMPREG(i, DISPC_OVL_FIR_COEF_HV, j);

		for (j = 0; j < 5; j++)
			DUMPREG(i, DISPC_OVL_CONV_COEF, j);

		if (dss_has_feature(FEAT_FIR_COEF_V)) {
			for (j = 0; j < 8; j++)
				DUMPREG(i, DISPC_OVL_FIR_COEF_V, j);
		}

		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			for (j = 0; j < 8; j++)
				DUMPREG(i, DISPC_OVL_FIR_COEF_H2, j);

			for (j = 0; j < 8; j++)
				DUMPREG(i, DISPC_OVL_FIR_COEF_HV2, j);

			for (j = 0; j < 8; j++)
				DUMPREG(i, DISPC_OVL_FIR_COEF_V2, j);
		}
	}

	dispc_runtime_put();

#undef DISPC_REG
#undef DUMPREG
}

static void dispc_mgr_get_pol_freq(enum omap_channel channel,
			enum omap_panel_config *config, u8 *acbi, u8 *acb)
{
	u32 l = dispc_read_reg(DISPC_POL_FREQ(channel));
	*config = 0;

	if (FLD_GET(l, 17, 17))
		*config |= OMAP_DSS_LCD_ONOFF;
	if (FLD_GET(l, 16, 16))
		*config |= OMAP_DSS_LCD_RF;
	if (FLD_GET(l, 15, 15))
		*config |= OMAP_DSS_LCD_IEO;
	if (FLD_GET(l, 14, 14))
		*config |= OMAP_DSS_LCD_IPC;
	if (FLD_GET(l, 13, 13))
		*config |= OMAP_DSS_LCD_IHS;
	if (FLD_GET(l, 12, 12))
		*config |= OMAP_DSS_LCD_IVS;

	*acbi = FLD_GET(l, 11, 8);
	*acb = FLD_GET(l, 7, 0);
}

static void _dispc_mgr_set_pol_freq(enum omap_channel channel, bool onoff,
		bool rf, bool ieo, bool ipc, bool ihs, bool ivs, u8 acbi,
		u8 acb)
{
	u32 l = 0;

	DSSDBG("onoff %d rf %d ieo %d ipc %d ihs %d ivs %d acbi %d acb %d\n",
			onoff, rf, ieo, ipc, ihs, ivs, acbi, acb);

	l |= FLD_VAL(onoff, 17, 17);
	l |= FLD_VAL(rf, 16, 16);
	l |= FLD_VAL(ieo, 15, 15);
	l |= FLD_VAL(ipc, 14, 14);
	l |= FLD_VAL(ihs, 13, 13);
	l |= FLD_VAL(ivs, 12, 12);
	l |= FLD_VAL(acbi, 11, 8);
	l |= FLD_VAL(acb, 7, 0);

	dispc_write_reg(DISPC_POL_FREQ(channel), l);
}

void dispc_mgr_set_pol_freq(enum omap_channel channel,
		enum omap_panel_config config, u8 acbi, u8 acb)
{
	_dispc_mgr_set_pol_freq(channel, (config & OMAP_DSS_LCD_ONOFF) != 0,
			(config & OMAP_DSS_LCD_RF) != 0,
			(config & OMAP_DSS_LCD_IEO) != 0,
			(config & OMAP_DSS_LCD_IPC) != 0,
			(config & OMAP_DSS_LCD_IHS) != 0,
			(config & OMAP_DSS_LCD_IVS) != 0,
			acbi, acb);
}

/* with fck as input clock rate, find dispc dividers that produce req_pck */
void dispc_find_clk_divs(bool is_tft, unsigned long req_pck, unsigned long fck,
		struct dispc_clock_info *cinfo)
{
	u16 pcd_min, pcd_max;
	unsigned long best_pck;
	u16 best_ld, cur_ld;
	u16 best_pd, cur_pd;

	pcd_min = dss_feat_get_param_min(FEAT_PARAM_DSS_PCD);
	pcd_max = dss_feat_get_param_max(FEAT_PARAM_DSS_PCD);

	if (!is_tft)
		pcd_min = 3;

	best_pck = 0;
	best_ld = 0;
	best_pd = 0;

	for (cur_ld = 1; cur_ld <= 255; ++cur_ld) {
		unsigned long lck = fck / cur_ld;

		for (cur_pd = pcd_min; cur_pd <= pcd_max; ++cur_pd) {
			unsigned long pck = lck / cur_pd;
			long old_delta = abs(best_pck - req_pck);
			long new_delta = abs(pck - req_pck);

			if (best_pck == 0 || new_delta < old_delta) {
				best_pck = pck;
				best_ld = cur_ld;
				best_pd = cur_pd;

				if (pck == req_pck)
					goto found;
			}

			if (pck < req_pck)
				break;
		}

		if (lck / pcd_min < req_pck)
			break;
	}

found:
	cinfo->lck_div = best_ld;
	cinfo->pck_div = best_pd;
	cinfo->lck = fck / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;
}

/* calculate clock rates using dividers in cinfo */
int dispc_calc_clock_rates(unsigned long dispc_fclk_rate,
		struct dispc_clock_info *cinfo)
{
	if (cinfo->lck_div > 255 || cinfo->lck_div == 0)
		return -EINVAL;
	if (cinfo->pck_div < 1 || cinfo->pck_div > 255)
		return -EINVAL;

	cinfo->lck = dispc_fclk_rate / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;

	return 0;
}

int dispc_mgr_set_clock_div(enum omap_channel channel,
		struct dispc_clock_info *cinfo)
{
	DSSDBG("lck = %lu (%u)\n", cinfo->lck, cinfo->lck_div);
	DSSDBG("pck = %lu (%u)\n", cinfo->pck, cinfo->pck_div);

	/* In case DISPC_CORE_CLK == PCLK, IPC must work on rising edge */
	if (dss_has_feature(FEAT_CORE_CLK_DIV) &&
			(cinfo->lck_div * cinfo->pck_div == 1)) {
		u8 acb, acbi;
		enum omap_panel_config config;
		dispc_mgr_get_pol_freq(channel, &config, &acbi, &acb);
		config |= OMAP_DSS_LCD_IPC;
		dispc_mgr_set_pol_freq(channel, config, acbi, acb);
	}

	dispc_mgr_set_lcd_divisor(channel, cinfo->lck_div, cinfo->pck_div);

	return 0;
}

int dispc_mgr_get_clock_div(enum omap_channel channel,
		struct dispc_clock_info *cinfo)
{
	unsigned long fck;

	fck = dispc_fclk_rate();

	cinfo->lck_div = REG_GET(DISPC_DIVISORo(channel), 23, 16);
	cinfo->pck_div = REG_GET(DISPC_DIVISORo(channel), 7, 0);

	cinfo->lck = fck / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;

	return 0;
}

/* dispc.irq_lock has to be locked by the caller */
static void _omap_dispc_set_irqs(void)
{
	u32 mask;
	u32 old_mask;
	int i;
	struct omap_dispc_isr_data *isr_data;

	mask = dispc.irq_error_mask;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];

		if (isr_data->isr == NULL)
			continue;

		mask |= isr_data->mask;
	}

	old_mask = dispc_read_reg(DISPC_IRQENABLE);
	/* clear the irqstatus for newly enabled irqs */
	dispc_write_reg(DISPC_IRQSTATUS, (mask ^ old_mask) & mask);

	dispc_write_reg(DISPC_IRQENABLE, mask);
}

int omap_dispc_register_isr(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	int ret;
	unsigned long flags;
	struct omap_dispc_isr_data *isr_data;

	if (isr == NULL)
		return -EINVAL;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	/* check for duplicate entry */
	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];
		if (isr_data->isr == isr && isr_data->arg == arg &&
				isr_data->mask == mask) {
			ret = -EINVAL;
			goto err;
		}
	}

	isr_data = NULL;
	ret = -EBUSY;

	/* WARN and return if DISPC is suspended, as otherwise we
	*  will crash while trying to write DSS registers. */
	if (WARN(pm_runtime_suspended(&dispc.pdev->dev),
		"Trying to register ISR while DISPC is suspended, exiting!\n"))
		goto err;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];

		if (isr_data->isr != NULL)
			continue;

		isr_data->isr = isr;
		isr_data->arg = arg;
		isr_data->mask = mask;
		ret = 0;

		break;
	}

	if (ret)
		goto err;

	_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return 0;
err:
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap_dispc_register_isr);

/* WARNING: callback might be executed even after this function returns! */
int omap_dispc_unregister_isr_nosync(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	unsigned long flags;
	int ret = -EINVAL;
	struct omap_dispc_isr_data *isr_data;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];
		if (isr_data->isr != isr || isr_data->arg != arg ||
				isr_data->mask != mask)
			continue;

		/* found the correct isr */

		isr_data->isr = NULL;
		isr_data->arg = NULL;
		isr_data->mask = 0;

		ret = 0;
		break;
	}

	if (ret == 0) {
		if (pm_runtime_suspended(&dispc.pdev->dev))
			DSSERR("Trying to disable IRQ while DSS is off!\n");
		else
			_omap_dispc_set_irqs();
	}

	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap_dispc_unregister_isr_nosync);

/*
 * Ensure that callback <isr> will NOT be executed after this function
 * returns. Must be called from sleepable context, though!
 */
int omap_dispc_unregister_isr_sync(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int ret;

	ret = omap_dispc_unregister_isr_nosync(isr, arg, mask);

	/* Non-atomic context is not really needed. But if we're called
	 * from atomic context, it is probably from DISPC IRQ, where we
	 * will deadlock.
	 */
	might_sleep();

#if defined(CONFIG_SMP)
	/* DISPC IRQ executes callbacks with dispc.irq_lock released, so
	 * there is a chance that a callback be executed even though it
	 * has been unregistered. Do disable/enable to act as a barrier, and
	 * ensure that after returning from this function, the DISPC IRQ
	 * will use an updated callback array, and NOT its cached copy.
	 *
	 * This is SMP-only issue because unregister_isr_nosync disables
	 * IRQs.
	 */
	synchronize_irq(dispc.irq);
#endif

	return ret;
}
EXPORT_SYMBOL(omap_dispc_unregister_isr_sync);

#ifdef DEBUG
static void print_irq_status(u32 status)
{
	if ((status & dispc.irq_error_mask) == 0)
		return;

	printk(KERN_DEBUG "DISPC IRQ: 0x%x: ", status);

#define PIS(x) \
	if (status & DISPC_IRQ_##x) \
		printk(#x " ");
	PIS(GFX_FIFO_UNDERFLOW);
	PIS(OCP_ERR);
	PIS(VID1_FIFO_UNDERFLOW);
	PIS(VID2_FIFO_UNDERFLOW);
	if (dss_feat_get_num_ovls() > 3)
		PIS(VID3_FIFO_UNDERFLOW);
	PIS(SYNC_LOST);
	PIS(SYNC_LOST_DIGIT);
	if (dss_has_feature(FEAT_MGR_LCD2))
		PIS(SYNC_LOST2);
#undef PIS

	printk("\n");
}
#endif

/* Called from dss.c. Note that we don't touch clocks here,
 * but we presume they are on because we got an IRQ. However,
 * an irq handler may turn the clocks off, so we may not have
 * clock later in the function. */
static irqreturn_t omap_dispc_irq_handler(int irq, void *arg)
{
	int i;
	u32 irqstatus, irqenable;
	u32 handledirqs = 0;
	u32 unhandled_errors;
	struct omap_dispc_isr_data *isr_data;
	struct omap_dispc_isr_data registered_isr[DISPC_MAX_NR_ISRS];

	spin_lock(&dispc.irq_lock);

	irqstatus = dispc_read_reg(DISPC_IRQSTATUS);
	irqenable = dispc_read_reg(DISPC_IRQENABLE);

	/* IRQ is not for us */
	if (!(irqstatus & irqenable)) {
		spin_unlock(&dispc.irq_lock);
		return IRQ_NONE;
	}

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_lock(&dispc.irq_stats_lock);
	dispc.irq_stats.irq_count++;
	dss_collect_irq_stats(irqstatus, dispc.irq_stats.irqs);
	spin_unlock(&dispc.irq_stats_lock);
#endif

#ifdef DEBUG
	if (dss_debug)
		print_irq_status(irqstatus);
#endif
	/* Ack the interrupt. Do it here before clocks are possibly turned
	 * off */
	dispc_write_reg(DISPC_IRQSTATUS, irqstatus);
	/* flush posted write */
	dispc_read_reg(DISPC_IRQSTATUS);

	/* make a copy and unlock, so that isrs can unregister
	 * themselves */
	memcpy(registered_isr, dispc.registered_isr,
			sizeof(registered_isr));

	spin_unlock(&dispc.irq_lock);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &registered_isr[i];

		if (!isr_data->isr)
			continue;

		if (isr_data->mask & irqstatus) {
			isr_data->isr(isr_data->arg, irqstatus);
			handledirqs |= isr_data->mask;
		}
	}

	spin_lock(&dispc.irq_lock);

	unhandled_errors = irqstatus & ~handledirqs & dispc.irq_error_mask;

	if (unhandled_errors) {
		dispc.error_irqs |= unhandled_errors;

		dispc.irq_error_mask &= ~unhandled_errors;
		_omap_dispc_set_irqs();

		schedule_work(&dispc.error_work);
	}

	spin_unlock(&dispc.irq_lock);

	return IRQ_HANDLED;
}

static void dispc_error_worker(struct work_struct *work)
{
	int i;
	u32 errors;
	unsigned long flags;
	static const unsigned fifo_underflow_bits[] = {
		DISPC_IRQ_GFX_FIFO_UNDERFLOW,
		DISPC_IRQ_VID1_FIFO_UNDERFLOW,
		DISPC_IRQ_VID2_FIFO_UNDERFLOW,
		DISPC_IRQ_VID3_FIFO_UNDERFLOW,
	};

	static const unsigned sync_lost_bits[] = {
		DISPC_IRQ_SYNC_LOST,
		DISPC_IRQ_SYNC_LOST_DIGIT,
		DISPC_IRQ_SYNC_LOST2,
	};

	spin_lock_irqsave(&dispc.irq_lock, flags);
	errors = dispc.error_irqs;
	dispc.error_irqs = 0;
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	dispc_runtime_get();

	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
		struct omap_overlay *ovl;
		unsigned bit;
		ovl = omap_dss_get_overlay(i);
		if (!ovl)
			continue;
		bit = fifo_underflow_bits[i];

		if (bit & errors) {
			DSSERR("FIFO UNDERFLOW on %s, disabling the overlay\n",
					ovl->name);
			ovl->disable(ovl);
			mdelay(50);
		}
	}

	for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
		struct omap_overlay_manager *mgr;
		unsigned bit;

		mgr = omap_dss_get_overlay_manager(i);
		if ((!mgr) || !mgr->device) {
			DSSERR("mgr or device is NULL\n");
			break;
		}

		if (!mgr->device->first_vsync) {
			DSSERR("First SYNC_LOST.. ignoring\n");
			break;
		}

		bit = sync_lost_bits[i];

		if (bit & errors) {
			struct omap_dss_device *dssdev = mgr->device;
			bool enable;
			if (dssdev && dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
				break;

			pr_err_ratelimited("SYNC_LOST on channel %s, restarting"
					" the output with video overlays "
					"disabled\n", mgr->name);

			enable = dssdev->state;

			mgr->device->sync_lost_error = true;
			dssdev->driver->disable(dssdev);
			mgr->device->sync_lost_error = false;

			for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);
				if (!ovl)
					continue;
				if (ovl->id != OMAP_DSS_GFX &&
						ovl->manager == mgr)
					dispc_ovl_enable(ovl->id, false);
			}

			dispc_mgr_go(mgr->id);
			mdelay(50);
			if (enable && dssdev->driver && dssdev->driver->enable)
				dssdev->driver->enable(dssdev);
		}
	}

	if (errors & DISPC_IRQ_OCP_ERR) {
		DSSERR("OCP_ERR\n");
		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);
			if (mgr && mgr->device && mgr->device->driver)
				mgr->device->driver->disable(mgr->device);
		}
	}

	if (errors & DISPC_IRQ_WBINCOMPLETE)
		DSSERR("WB FIFO flushed before completion\n");

	spin_lock_irqsave(&dispc.irq_lock, flags);
	dispc.irq_error_mask |= errors;
	_omap_dispc_set_irqs();
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	dispc_runtime_put();
}

int omap_dispc_wait_for_irq_timeout(u32 irqmask, unsigned long timeout)
{
	void dispc_irq_wait_handler(void *data, u32 mask)
	{
		complete((struct completion *)data);
	}

	int r;
	DECLARE_COMPLETION_ONSTACK(completion);

	r = omap_dispc_register_isr(dispc_irq_wait_handler, &completion,
			irqmask);

	if (r)
		return r;

	timeout = wait_for_completion_timeout(&completion, timeout);

	omap_dispc_unregister_isr_sync(dispc_irq_wait_handler,
		&completion, irqmask);

	if (timeout == 0)
		return -ETIMEDOUT;

	if (timeout == -ERESTARTSYS)
		return -ERESTARTSYS;

	return 0;
}

int omap_dispc_wait_for_irq_interruptible_timeout(u32 irqmask,
		unsigned long timeout)
{
	void dispc_irq_wait_handler(void *data, u32 mask)
	{
		complete((struct completion *)data);
	}

	int r;
	DECLARE_COMPLETION_ONSTACK(completion);

	r = omap_dispc_register_isr(dispc_irq_wait_handler, &completion,
			irqmask);

	if (r)
		return r;

	timeout = wait_for_completion_interruptible_timeout(&completion,
			timeout);

	omap_dispc_unregister_isr_sync(dispc_irq_wait_handler,
		&completion, irqmask);

	if (timeout == 0)
		return -ETIMEDOUT;

	if (timeout == -ERESTARTSYS)
		return -ERESTARTSYS;

	return 0;
}

#ifdef CONFIG_OMAP2_DSS_FAKE_VSYNC
void dispc_fake_vsync_irq(void)
{
	u32 irqstatus = DISPC_IRQ_VSYNC;
	int i;

	WARN_ON(!in_interrupt());

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		struct omap_dispc_isr_data *isr_data;
		isr_data = &dispc.registered_isr[i];

		if (!isr_data->isr)
			continue;

		if (isr_data->mask & irqstatus)
			isr_data->isr(isr_data->arg, irqstatus);
	}
}
#endif

static void _omap_dispc_initialize_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	memset(dispc.registered_isr, 0, sizeof(dispc.registered_isr));

	dispc.irq_error_mask = DISPC_IRQ_MASK_ERROR;
	if (dss_has_feature(FEAT_MGR_LCD2))
		dispc.irq_error_mask |= DISPC_IRQ_SYNC_LOST2;
	if (dss_feat_get_num_ovls() > 3)
		dispc.irq_error_mask |= DISPC_IRQ_VID3_FIFO_UNDERFLOW;

	/* there's SYNC_LOST_DIGIT waiting after enabling the DSS,
	 * so clear it */
	dispc_write_reg(DISPC_IRQSTATUS, dispc_read_reg(DISPC_IRQSTATUS));

	_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);
}

void dispc_enable_sidle(void)
{
	REG_FLD_MOD(DISPC_SYSCONFIG, 2, 4, 3);	/* SIDLEMODE: smart idle */
}

void dispc_disable_sidle(void)
{
	REG_FLD_MOD(DISPC_SYSCONFIG, 1, 4, 3);	/* SIDLEMODE: no idle */
}

static void _omap_dispc_initial_config(void)
{
	u32 l;

	/* Exclusively enable DISPC_CORE_CLK and set divider to 1 */
	if (dss_has_feature(FEAT_CORE_CLK_DIV)) {
		l = dispc_read_reg(DISPC_DIVISOR);
		/* Use DISPC_DIVISOR.LCD, instead of DISPC_DIVISOR1.LCD */
		l = FLD_MOD(l, 1, 0, 0);
		l = FLD_MOD(l, 1, 23, 16);
		dispc_write_reg(DISPC_DIVISOR, l);
	}

	if (cpu_is_omap44xx()) {
		l3_1_clkdm = clkdm_lookup("l3_1_clkdm");
		l3_2_clkdm = clkdm_lookup("l3_2_clkdm");
	} else if (cpu_is_omap54xx() && (omap_rev() <= OMAP5430_REV_ES1_0)) {
		l3_1_clkdm = clkdm_lookup("l3main1_clkdm");
		l3_2_clkdm = clkdm_lookup("l3main2_clkdm");
	}

	/* FUNCGATED */
	if (dss_has_feature(FEAT_FUNCGATED))
		REG_FLD_MOD(DISPC_CONFIG, 1, 9, 9);

	REG_FLD_MOD(DISPC_CONFIG, 1, 17, 17);

	dispc_set_loadmode(OMAP_DSS_LOAD_FRAME_ONLY);

	dispc_read_plane_fifo_sizes();

	dispc_configure_burst_sizes();

	dispc_ovl_enable_zorder_planes();
}

/* DISPC HW IP initialisation */
static int omap_dispchw_probe(struct platform_device *pdev)
{
	u32 rev;
	int r = 0;
	struct resource *dispc_mem;
	struct clk *clk;

	dispc.pdev = pdev;

	spin_lock_init(&dispc.irq_lock);

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_lock_init(&dispc.irq_stats_lock);
	dispc.irq_stats.last_reset = jiffies;
#endif

	INIT_WORK(&dispc.error_work, dispc_error_worker);

	dispc_mem = platform_get_resource(dispc.pdev, IORESOURCE_MEM, 0);
	if (!dispc_mem) {
		DSSERR("can't get IORESOURCE_MEM DISPC\n");
		return -EINVAL;
	}

	dispc.base = devm_ioremap(&pdev->dev, dispc_mem->start,
				  resource_size(dispc_mem));
	if (!dispc.base) {
		DSSERR("can't ioremap DISPC\n");
		return -ENOMEM;
	}

	dispc.irq = platform_get_irq(dispc.pdev, 0);
	if (dispc.irq < 0) {
		DSSERR("platform_get_irq failed\n");
		return -ENODEV;
	}

	r = devm_request_irq(&pdev->dev, dispc.irq, omap_dispc_irq_handler,
			     IRQF_SHARED, "OMAP DISPC", dispc.pdev);
	if (r < 0) {
		DSSERR("request_irq failed\n");
		return r;
	}

	clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(clk)) {
		DSSERR("can't get fck\n");
		r = PTR_ERR(clk);
		return r;
	}

	dispc.dss_clk = clk;

	pm_runtime_enable(&pdev->dev);

	r = dispc_runtime_get();
	if (r)
		goto err_runtime_get;

	_omap_dispc_initial_config();

	_omap_dispc_initialize_irq();

	rev = dispc_read_reg(DISPC_REVISION);
	dev_dbg(&pdev->dev, "OMAP DISPC rev %d.%d\n",
	       FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

	dispc_runtime_put();

	return 0;

err_runtime_get:
	pm_runtime_disable(&pdev->dev);
	clk_put(dispc.dss_clk);
	return r;
}

static int omap_dispchw_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	clk_put(dispc.dss_clk);

	return 0;
}

static int dispc_runtime_suspend(struct device *dev)
{
	dispc_save_context();

	/*
	 * OMAP4,5 ERRATUM i740: Mstandby and disconnect protocol issue
	 * Workaround:
	 * Restore L3_1 CD to HW_AUTO, when DSS module idles.
	 */
	if (cpu_is_omap44xx() || (cpu_is_omap54xx() &&
			(omap_rev() <= OMAP5430_REV_ES1_0))) {
		clkdm_allow_idle(l3_1_clkdm);
		clkdm_allow_idle(l3_2_clkdm);
	}

	return 0;
}

static int dispc_runtime_resume(struct device *dev)
{

	/*
	 * OMAP4,5 ERRATUM i740: Mstandby and disconnect protocol issue
	 * Impacts: all OMAP4 and OMAP5_ES1 devices
	 * Simplfied Description:
	 * issue #1: The handshake between IP modules on L3_1
	 * peripherals with PRCM has a limitation in a certain time
	 * window of L4 clock cycle. Due to the fact that a wrong
	 * variant of stall signal was used in circuit of PRCM, the
	 * intitator-interconnect protocol is broken when the time
	 * window is hit where the PRCM requires the interconnect to go
	 * to idle while intitator asks to wakeup.
	 * Issue #2: DISPC asserts a sub-mstandby signal for a short
	 * period. In this time interval, IP block requests
	 * disconnection of Master port, and results in Mstandby and
	 * wait request to PRCM. In parallel, if mstandby is de-asserted
	 * by DISPC simultaneously, interconnect requests for a
	 * reconnect for one cycle alone resulting in a disconnect
	 * protocol violation and a deadlock of the system.
	 *
	 * Workaround:
	 * L3_1 clock domain must not be programmed in HW_AUTO if
	 * Static dependency with DSS is enabled and DSS clock domain
	 * is ON.
	 */
	if (cpu_is_omap44xx() || (cpu_is_omap54xx() &&
			(omap_rev() <= OMAP5430_REV_ES1_0))) {
		clkdm_deny_idle(l3_1_clkdm);
		clkdm_deny_idle(l3_2_clkdm);
	}

	dispc_restore_context();

	return 0;
}

static const struct dev_pm_ops dispc_pm_ops = {
	.runtime_suspend = dispc_runtime_suspend,
	.runtime_resume = dispc_runtime_resume,
};

static struct platform_driver omap_dispchw_driver = {
	.remove         = omap_dispchw_remove,
	.driver         = {
		.name   = "omapdss_dispc",
		.owner  = THIS_MODULE,
		.pm	= &dispc_pm_ops,
	},
};

int dispc_init_platform_driver(void)
{
	return platform_driver_probe(&omap_dispchw_driver, omap_dispchw_probe);
}

void dispc_uninit_platform_driver(void)
{
	return platform_driver_unregister(&omap_dispchw_driver);
}
