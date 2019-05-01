/*
 * linux/arch/arm/mach-omap2/hsmmc.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pm_qos.h>
#include <mach/hardware.h>
#include <plat/mmc.h>
#include <plat/omap-pm.h>
#include <plat/mux.h>
#include <plat/omap_device.h>

#include "mux.h"
#include "hsmmc.h"
#include "control.h"

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define OMAP5_ES20_CONTROL_PBIAS_SDCARD_IO_PWRRDNZ	(1 << 26)
#define OMAP5_ES20_CONTROL_PBIAS_SDCARD_BIAS_PWRDNZ	(1 << 27)

static u16 control_pbias_offset;
static u16 control_devconf1_offset;
static u16 control_mmc1;

#define HSMMC_NAME_LEN	9

/* To handle OPP Scaling */
#define QOS_MMC			(250*4*1000)
#define MMC_OPP_CLOCK_THRESHOLD	104000000
#define MMC_OPP_NOM_FCLK	96000000
#define MMC_OPP_100_FCLK	192000000

#if (defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4) ||\
		defined(CONFIG_ARCH_OMAP5)) && defined(CONFIG_PM)

static int hsmmc_get_context_loss(struct device *dev)
{
	return omap_pm_get_dev_context_loss_count(dev);
}

#else
#define hsmmc_get_context_loss NULL
#endif

static void omap_hsmmc1_before_set_reg(struct device *dev, int slot,
				  int power_on, int vdd)
{
	u32 reg, prog_io;
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	if (mmc->slots[0].remux)
		mmc->slots[0].remux(dev, slot, power_on);

	/*
	 * Assume we power both OMAP VMMC1 (for CMD, CLK, DAT0..3) and the
	 * card with Vcc regulator (from twl4030 or whatever).  OMAP has both
	 * 1.8V and 3.0V modes, controlled by the PBIAS register.
	 *
	 * In 8-bit modes, OMAP VMMC1A (for DAT4..7) needs a supply, which
	 * is most naturally TWL VSIM; those pins also use PBIAS.
	 *
	 * FIXME handle VMMC1A as needed ...
	 */
	if (power_on) {
		if (cpu_is_omap2430()) {
			reg = omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1);
			if ((1 << vdd) >= MMC_VDD_30_31)
				reg |= OMAP243X_MMC1_ACTIVE_OVERWRITE;
			else
				reg &= ~OMAP243X_MMC1_ACTIVE_OVERWRITE;
			omap_ctrl_writel(reg, OMAP243X_CONTROL_DEVCONF1);
		}

		if (mmc->slots[0].internal_clock) {
			reg = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
			reg |= OMAP2_MMCSDIO1ADPCLKISEL;
			omap_ctrl_writel(reg, OMAP2_CONTROL_DEVCONF0);
		}

		reg = omap_ctrl_readl(control_pbias_offset);
		if (cpu_is_omap3630()) {
			/* Set MMC I/O to 52Mhz */
			prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
			prog_io |= OMAP3630_PRG_SDMMC1_SPEEDCTRL;
			omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);
		} else {
			reg |= OMAP2_PBIASSPEEDCTRL0;
		}
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, control_pbias_offset);
	} else {
		reg = omap_ctrl_readl(control_pbias_offset);
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, control_pbias_offset);
	}
}

static void omap_hsmmc1_after_set_reg(struct device *dev, int slot,
				 int power_on, int vdd)
{
	u32 reg;

	/* 100ms delay required for PBIAS configuration */
	msleep(100);

	if (power_on) {
		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= (OMAP2_PBIASLITEPWRDNZ0 | OMAP2_PBIASSPEEDCTRL0);
		if ((1 << vdd) <= MMC_VDD_165_195)
			reg &= ~OMAP2_PBIASLITEVMODE0;
		else
			reg |= OMAP2_PBIASLITEVMODE0;
		omap_ctrl_writel(reg, control_pbias_offset);
	} else {
		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= (OMAP2_PBIASSPEEDCTRL0 | OMAP2_PBIASLITEPWRDNZ0 |
			OMAP2_PBIASLITEVMODE0);
		omap_ctrl_writel(reg, control_pbias_offset);
	}
}

static void omap4_hsmmc1_before_set_reg(struct device *dev, int slot,
				  int power_on, int vdd)
{
	u32 reg;

	/*
	 * Assume we power both OMAP VMMC1 (for CMD, CLK, DAT0..3) and the
	 * card with Vcc regulator (from twl4030 or whatever).  OMAP has both
	 * 1.8V and 3.0V modes, controlled by the PBIAS register.
	 */
	reg = omap4_ctrl_pad_readl(control_pbias_offset);
	reg &= ~(OMAP4_MMC1_PBIASLITE_PWRDNZ_MASK |
		OMAP4_MMC1_PWRDNZ_MASK |
		OMAP4_MMC1_PBIASLITE_VMODE_MASK);
	omap4_ctrl_pad_writel(reg, control_pbias_offset);
}

static void omap4_hsmmc1_after_set_reg(struct device *dev, int slot,
				 int power_on, int vdd)
{
	u32 reg;
	unsigned long timeout;

	if (power_on) {
		reg = omap4_ctrl_pad_readl(control_pbias_offset);
		reg |= OMAP4_MMC1_PBIASLITE_PWRDNZ_MASK;
		if ((1 << vdd) <= MMC_VDD_165_195)
			reg &= ~OMAP4_MMC1_PBIASLITE_VMODE_MASK;
		else
			reg |= OMAP4_MMC1_PBIASLITE_VMODE_MASK;
		reg |= (OMAP4_MMC1_PBIASLITE_PWRDNZ_MASK |
			OMAP4_MMC1_PWRDNZ_MASK);
		omap4_ctrl_pad_writel(reg, control_pbias_offset);

		timeout = jiffies + msecs_to_jiffies(5);
		do {
			reg = omap4_ctrl_pad_readl(control_pbias_offset);
			if (!(reg & OMAP4_MMC1_PBIASLITE_VMODE_ERROR_MASK))
				break;
			usleep_range(100, 200);
		} while (!time_after(jiffies, timeout));

		if (reg & OMAP4_MMC1_PBIASLITE_VMODE_ERROR_MASK) {
			pr_err("Pbias Voltage is not same as LDO\n");
			/* Caution : On VMODE_ERROR Power Down MMC IO */
			reg &= ~(OMAP4_MMC1_PWRDNZ_MASK);
			omap4_ctrl_pad_writel(reg, control_pbias_offset);
		}
	}
}

/* OMAP5 ES2.0 PBIAS setting procedure */
static void omap5_es2_before_set_reg(struct device *dev, int slot,
					int power_on, int vdd)
{
	u32 reg;

	reg = omap4_ctrl_pad_readl(control_pbias_offset);
	reg &= ~(OMAP5_ES20_CONTROL_PBIAS_SDCARD_IO_PWRRDNZ);
	omap4_ctrl_pad_writel(reg, control_pbias_offset);
	udelay(10);
	reg &= ~(OMAP5_ES20_CONTROL_PBIAS_SDCARD_BIAS_PWRDNZ);
	omap4_ctrl_pad_writel(reg, control_pbias_offset);
}

static void omap5_es2_after_set_reg(struct device *dev, int slot,
				 int power_on, int vdd)
{
	u32 reg;

	if (power_on) {
		reg = omap4_ctrl_pad_readl(control_pbias_offset);
		reg |= OMAP5_ES20_CONTROL_PBIAS_SDCARD_BIAS_PWRDNZ;
		omap4_ctrl_pad_writel(reg, control_pbias_offset);
		udelay(150);
		reg |= OMAP5_ES20_CONTROL_PBIAS_SDCARD_IO_PWRRDNZ;
		omap4_ctrl_pad_writel(reg, control_pbias_offset);
		udelay(150);
	}
}

/* Enable clock pull up or clock pull down */
static void omap5_clk_pull_up(struct device *dev, int slot, bool up)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;
	struct omap_mux_partition *p_mmc_clk = mmc->slots[0].p_mmc_clk;
	struct omap_mux *mux_mmc_clk = mmc->slots[0].mux_mmc_clk;
	u16 r_sdcard_clk = 0;

	if ((!p_mmc_clk) || (!mux_mmc_clk)) {
		printk(KERN_ERR "Unable to get SD Card Mux reference\n");
		return;
	}
	r_sdcard_clk = omap_mux_read(p_mmc_clk, mux_mmc_clk->reg_offset);
	if (up)
		r_sdcard_clk |= (0x1 << 4);
	else
		r_sdcard_clk &= ~(0x1 << 4);

	omap_mux_write(p_mmc_clk, r_sdcard_clk, mux_mmc_clk->reg_offset);
}

static void hsmmc2_select_input_clk_src(struct omap_mmc_platform_data *mmc)
{
	u32 reg;

	reg = omap_ctrl_readl(control_devconf1_offset);
	if (mmc->slots[0].internal_clock)
		reg |= OMAP2_MMCSDIO2ADPCLKISEL;
	else
		reg &= ~OMAP2_MMCSDIO2ADPCLKISEL;
	omap_ctrl_writel(reg, control_devconf1_offset);
}

static void hsmmc2_before_set_reg(struct device *dev, int slot,
				   int power_on, int vdd)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	if (mmc->slots[0].remux)
		mmc->slots[0].remux(dev, slot, power_on);

	if (power_on)
		hsmmc2_select_input_clk_src(mmc);
}

static int am35x_hsmmc2_set_power(struct device *dev, int slot,
				  int power_on, int vdd)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	if (power_on)
		hsmmc2_select_input_clk_src(mmc);

	return 0;
}

static int nop_mmc_set_power(struct device *dev, int slot, int power_on,
							int vdd)
{
	return 0;
}

static inline void omap_hsmmc_mux(struct omap_mmc_platform_data *mmc_controller,
			int controller_nr)
{
	if (gpio_is_valid(mmc_controller->slots[0].switch_pin) &&
		(mmc_controller->slots[0].switch_pin < OMAP_MAX_GPIO_LINES))
		omap_mux_init_gpio(mmc_controller->slots[0].switch_pin,
					OMAP_PIN_INPUT_PULLUP);
	if (gpio_is_valid(mmc_controller->slots[0].gpio_wp) &&
		(mmc_controller->slots[0].gpio_wp < OMAP_MAX_GPIO_LINES))
		omap_mux_init_gpio(mmc_controller->slots[0].gpio_wp,
					OMAP_PIN_INPUT_PULLUP);
	if (cpu_is_omap34xx()) {
		if (controller_nr == 0) {
			omap_mux_init_signal("sdmmc1_clk",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc1_cmd",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc1_dat0",
				OMAP_PIN_INPUT_PULLUP);
			if (mmc_controller->slots[0].caps &
				(MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA)) {
				omap_mux_init_signal("sdmmc1_dat1",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat2",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat3",
					OMAP_PIN_INPUT_PULLUP);
			}
			if (mmc_controller->slots[0].caps &
						MMC_CAP_8_BIT_DATA) {
				omap_mux_init_signal("sdmmc1_dat4",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat5",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat6",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat7",
					OMAP_PIN_INPUT_PULLUP);
			}
		}
		if (controller_nr == 1) {
			/* MMC2 */
			omap_mux_init_signal("sdmmc2_clk",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc2_cmd",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc2_dat0",
				OMAP_PIN_INPUT_PULLUP);

			/*
			 * For 8 wire configurations, Lines DAT4, 5, 6 and 7
			 * need to be muxed in the board-*.c files
			 */
			if (mmc_controller->slots[0].caps &
				(MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA)) {
				omap_mux_init_signal("sdmmc2_dat1",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat2",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat3",
					OMAP_PIN_INPUT_PULLUP);
			}
			if (mmc_controller->slots[0].caps &
							MMC_CAP_8_BIT_DATA) {
				omap_mux_init_signal("sdmmc2_dat4.sdmmc2_dat4",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat5.sdmmc2_dat5",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat6.sdmmc2_dat6",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat7.sdmmc2_dat7",
					OMAP_PIN_INPUT_PULLUP);
			}
		}

		/*
		 * For MMC3 the pins need to be muxed in the board-*.c files
		 */
	}
}

static int omap_hsmmc_opp_scale_init(struct omap_mmc_platform_data *pdata)
{
	pm_qos_add_request(&pdata->pm_qos_request, PM_QOS_MEMORY_THROUGHPUT,
				PM_QOS_MEMORY_THROUGHPUT_DEFAULT_VALUE);
	return 0;
}

static int omap_hsmmc_opp_scale(struct omap_mmc_platform_data *pdata,
					 unsigned int clock)
{
	int ret = 0;
	if (clock > MMC_OPP_CLOCK_THRESHOLD) {
		pm_qos_update_request(&pdata->pm_qos_request, QOS_MMC);
		if (clk_get_rate(pdata->fclk) != MMC_OPP_100_FCLK)
			ret = clk_set_rate(pdata->fclk, MMC_OPP_100_FCLK);
	} else {
		if (clk_get_rate(pdata->fclk) != MMC_OPP_NOM_FCLK)
			ret = clk_set_rate(pdata->fclk, MMC_OPP_NOM_FCLK);
	}
	return ret;
}

static int omap_hsmmc_opp_relax(struct omap_mmc_platform_data *pdata)
{
	pm_qos_update_request(&pdata->pm_qos_request, PM_QOS_DEFAULT_VALUE);
	return 0;
}

static int omap_hsmmc_set_clks_src(struct device *dev, unsigned int id)
{
	struct clk *fclk_child;
	struct clk *fclk_parent;
	int r;
	char *child_name;
	char *parent_name;

	/* Cannot change clock sources for MMC id > 1 */
	if (id > 1)
		return 0;

	if (cpu_is_omap54xx()) {
		parent_name = "dpll_per_m2x2_ck";
		if (id == 0)
			child_name = "mmc1_fclk_mux";
		else
			child_name = "mmc2_fclk_mux";
	} else if (cpu_is_omap44xx()) {
		parent_name = "func_96m_fclk";
		if (id == 0)
			child_name = "mmc1_fck";
		else
			child_name = "mmc2_fck";
	} else
		return 0;

	fclk_parent = clk_get(dev, parent_name);
	if (IS_ERR_OR_NULL(fclk_parent))
		return -EINVAL;
	fclk_child = clk_get(dev, child_name);
	if (IS_ERR_OR_NULL(fclk_child)) {
		clk_put(fclk_child);
		return -EINVAL;
	}

	r = clk_set_parent(fclk_child, fclk_parent);
	if (IS_ERR_VALUE(r)) {
		clk_put(fclk_child);
		clk_put(fclk_parent);
		return -EINVAL;
	}
	clk_put(fclk_child);
	clk_put(fclk_parent);
	return 0;
}

static int __init
omap_hsmmc_max_min(u8 slot, unsigned long *max, unsigned long *min)
{
	if (cpu_is_omap54xx()) {
		switch (slot) {
		case 0:
		case 1:
			*max = 192000000;
			break;
		case 2:
		case 3:
		case 4:
			*max = 48000000;
			break;
		default:
			return -EINVAL;
		}
	} else if (cpu_is_omap44xx()) {
		switch (slot) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			*max = 48000000;
			break;
		default:
			return -EINVAL;
		}
	} else if (cpu_is_omap34xx()) {
		switch (slot) {
		case 0:
		case 1:
		case 2:
			*max = 96000000;
			break;
		default:
			return -EINVAL;
		}
	} else if (cpu_is_omap24xx()) {
		switch (slot) {
		case 0:
		case 1:
			*max = 96000000;
			break;
		default:
			return -EINVAL;
		}
	} else
		return -EINVAL;
	*min = 40000;
	return 0;
}

/* Set Silicon specific capabilities */
static u32 __init omap_hsmmc_si_spec_caps(struct omap2_hsmmc_info *c)
{
	u32 caps = 0;
	if (cpu_is_omap54xx()) {
		if (c->mmc == 1) {
			caps |= (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
				MMC_CAP_UHS_DDR50);
			caps |= MMC_CAP_UHS_SDR104;
		}
	}
	return caps;
}

static u32 __init omap_hsmmc_si_spec_caps2(struct omap2_hsmmc_info *c)
{
	u32 caps2 = c->caps2;
	if (cpu_is_omap54xx()) {
		if (c->mmc == 2) {
			caps2 |= MMC_CAP2_HS200_1_8V_SDR;
		}
	}
	return caps2;
}

static int __init omap_hsmmc_pdata_init(struct omap2_hsmmc_info *c,
					struct omap_mmc_platform_data *mmc)
{
	char *hc_name;
	unsigned long max_freq, min_freq;

	hc_name = kzalloc(sizeof(char) * (HSMMC_NAME_LEN + 1), GFP_KERNEL);
	if (!hc_name) {
		pr_err("Cannot allocate memory for controller slot name\n");
		kfree(hc_name);
		return -ENOMEM;
	}

	if (c->name)
		strncpy(hc_name, c->name, HSMMC_NAME_LEN);
	else
		snprintf(hc_name, (HSMMC_NAME_LEN + 1), "mmc%islot%i",
								c->mmc, 1);
	mmc->slots[0].name = hc_name;
	mmc->nr_slots = 1;
	mmc->slots[0].caps = c->caps;
	mmc->slots[0].caps |= omap_hsmmc_si_spec_caps(c);
	mmc->slots[0].caps2 |= omap_hsmmc_si_spec_caps2(c);
	mmc->slots[0].pm_caps = c->pm_caps;
	mmc->slots[0].internal_clock = !c->ext_clock;
	mmc->slots[0].housekeeping_ms = c->housekeeping_ms;
	mmc->dma_mask = 0xffffffff;

	mmc->set_clk_src = omap_hsmmc_set_clks_src;
	if (omap_hsmmc_max_min(c->mmc - 1, &max_freq, &min_freq)) {
		pr_err("Invalid mmc slot");
		kfree(hc_name);
		return -EINVAL;
	}

	if (c->max_freq >  0)
		mmc->max_freq = min(c->max_freq, max_freq);
	else
		mmc->max_freq = max_freq;
	mmc->max_si_freq = max_freq;
	mmc->min_freq = min_freq;

	if ((c->mmc <= 2) && (cpu_is_omap54xx())) {
		mmc->opp_scale_init = omap_hsmmc_opp_scale_init;
		mmc->opp_scale = omap_hsmmc_opp_scale;
		mmc->opp_relax = omap_hsmmc_opp_relax;
	}

	if (cpu_is_omap44xx() || cpu_is_omap54xx())
		mmc->reg_offset = OMAP4_MMC_REG_OFFSET;
	else
		mmc->reg_offset = 0;

	mmc->get_context_loss_count = hsmmc_get_context_loss;

	mmc->slots[0].switch_pin = c->gpio_cd;
	mmc->slots[0].gpio_wp = c->gpio_wp;

	mmc->slots[0].remux = c->remux;
	mmc->slots[0].init_card = c->init_card;

	if (c->cover_only)
		mmc->slots[0].cover = 1;

	if (c->nonremovable)
		mmc->slots[0].nonremovable = 1;

	if (c->power_saving)
		mmc->slots[0].power_saving = 1;

	if (c->no_off)
		mmc->slots[0].no_off = 1;

	if (c->no_off_init)
		mmc->slots[0].no_regulator_off_init = c->no_off_init;

	if (c->vcc_aux_disable_is_sleep)
		mmc->slots[0].vcc_aux_disable_is_sleep = 1;

	if (c->mmc_data) {
		memcpy(&mmc->slots[0].mmc_data, c->mmc_data,
				sizeof(struct mmc_platform_data));
		mmc->slots[0].card_detect =
				(int (*)(struct device *dev, int slot))c->mmc_data->status;
	}
	/*
	 * NOTE:  MMC slots should have a Vcc regulator set up.
	 * This may be from a TWL4030-family chip, another
	 * controllable regulator, or a fixed supply.
	 *
	 * temporary HACK: ocr_mask instead of fixed supply
	 */
	if (cpu_is_omap3505() || cpu_is_omap3517())
		mmc->slots[0].ocr_mask = MMC_VDD_165_195 |
					 MMC_VDD_26_27 |
					 MMC_VDD_27_28 |
					 MMC_VDD_29_30 |
					 MMC_VDD_30_31 |
					 MMC_VDD_31_32;
	else
		mmc->slots[0].ocr_mask = c->ocr_mask;

	mmc->slots[0].built_in = c->built_in;
	if (!cpu_is_omap3517() && !cpu_is_omap3505())
		mmc->slots[0].features |= HSMMC_HAS_PBIAS;

	if ((cpu_is_omap44xx() && (omap_rev() > OMAP4430_REV_ES1_0)) ||
	    cpu_is_omap54xx())
		mmc->slots[0].features |= HSMMC_HAS_UPDATED_RESET;

	switch (c->mmc) {
	case 1:
		if (mmc->slots[0].features & HSMMC_HAS_PBIAS) {
			/* on-chip level shifting via PBIAS0/PBIAS1 */
			if (cpu_is_omap54xx()) {
				mmc->slots[0].before_set_reg =
					omap5_es2_before_set_reg;
				mmc->slots[0].after_set_reg =
					omap5_es2_after_set_reg;
				omap_mux_get_by_name("sdcard_clk",
						&mmc->slots[0].p_mmc_clk,
						&mmc->slots[0].mux_mmc_clk);
				mmc->slots[0].clk_pull_up = omap5_clk_pull_up;
			} else if (cpu_is_omap44xx()) {
				mmc->slots[0].before_set_reg =
						omap4_hsmmc1_before_set_reg;
				mmc->slots[0].after_set_reg =
						omap4_hsmmc1_after_set_reg;
			} else {
				mmc->slots[0].before_set_reg =
						omap_hsmmc1_before_set_reg;
				mmc->slots[0].after_set_reg =
						omap_hsmmc1_after_set_reg;
			}
		}

		if (cpu_is_omap3517() || cpu_is_omap3505())
			mmc->slots[0].set_power = nop_mmc_set_power;

		/* OMAP3630 HSMMC1 supports only 4-bit */
		if (cpu_is_omap3630() &&
				(c->caps & MMC_CAP_8_BIT_DATA)) {
			c->caps &= ~MMC_CAP_8_BIT_DATA;
			c->caps |= MMC_CAP_4_BIT_DATA;
			mmc->slots[0].caps = c->caps;
		}
		break;
	case 2:
		if (cpu_is_omap3517() || cpu_is_omap3505())
			mmc->slots[0].set_power = am35x_hsmmc2_set_power;

		if (c->ext_clock)
			c->transceiver = 1;
		if (c->transceiver && (c->caps & MMC_CAP_8_BIT_DATA)) {
			c->caps &= ~MMC_CAP_8_BIT_DATA;
			c->caps |= MMC_CAP_4_BIT_DATA;
		}
		if (mmc->slots[0].features & HSMMC_HAS_PBIAS) {
			/* off-chip level shifting, or none */
			mmc->slots[0].before_set_reg = hsmmc2_before_set_reg;
			mmc->slots[0].after_set_reg = NULL;
		}
		break;
	case 3:
	case 4:
	case 5:
		mmc->slots[0].before_set_reg = NULL;
		mmc->slots[0].after_set_reg = NULL;
		break;
	default:
		pr_err("MMC%d configuration not supported!\n", c->mmc);
		kfree(hc_name);
		return -ENODEV;
	}
	return 0;
}

static int omap_hsmmc_done;

void omap_hsmmc_late_init(struct omap2_hsmmc_info *c)
{
	struct platform_device *pdev;
	struct omap_mmc_platform_data *mmc_pdata;
	int res;

	if (omap_hsmmc_done != 1)
		return;

	omap_hsmmc_done++;

	for (; c->mmc; c++) {
		if (!c->deferred)
			continue;

		pdev = c->pdev;
		if (!pdev)
			continue;

		mmc_pdata = pdev->dev.platform_data;
		if (!mmc_pdata)
			continue;

		mmc_pdata->slots[0].switch_pin = c->gpio_cd;
		mmc_pdata->slots[0].gpio_wp = c->gpio_wp;

		res = omap_device_register(pdev);
		if (res)
			pr_err("Could not late init MMC %s\n",
			       c->name);
	}
}

#define MAX_OMAP_MMC_HWMOD_NAME_LEN		16

static void __init omap_hsmmc_init_one(struct omap2_hsmmc_info *hsmmcinfo,
					int ctrl_nr)
{
	struct omap_hwmod *oh;
	struct omap_hwmod *ohs[1];
	struct omap_device *od;
	struct platform_device *pdev;
	char oh_name[MAX_OMAP_MMC_HWMOD_NAME_LEN];
	struct omap_mmc_platform_data *mmc_data;
	struct omap_mmc_dev_attr *mmc_dev_attr;
	char *name;
	int res;

	mmc_data = kzalloc(sizeof(struct omap_mmc_platform_data), GFP_KERNEL);
	if (!mmc_data) {
		pr_err("Cannot allocate memory for mmc device!\n");
		return;
	}

	res = omap_hsmmc_pdata_init(hsmmcinfo, mmc_data);
	if (res < 0)
		goto free_mmc;

	omap_hsmmc_mux(mmc_data, (ctrl_nr - 1));

	name = "omap_hsmmc";
	res = snprintf(oh_name, MAX_OMAP_MMC_HWMOD_NAME_LEN,
		     "mmc%d", ctrl_nr);
	WARN(res >= MAX_OMAP_MMC_HWMOD_NAME_LEN,
	     "String buffer overflow in MMC%d device setup\n", ctrl_nr);

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		goto free_name;
	}
	ohs[0] = oh;
	if (oh->dev_attr != NULL) {
		mmc_dev_attr = oh->dev_attr;
		mmc_data->controller_flags = mmc_dev_attr->flags;
		/*
		 * erratum 2.1.1.128 doesn't apply if board has
		 * a transceiver is attached
		 */
		if (hsmmcinfo->transceiver)
			mmc_data->controller_flags &=
				~OMAP_HSMMC_BROKEN_MULTIBLOCK_READ;
	}

	pdev = platform_device_alloc(name, ctrl_nr - 1);
	if (!pdev) {
		pr_err("Could not allocate pdev for %s\n", name);
		goto free_name;
	}
	dev_set_name(&pdev->dev, "%s.%d", pdev->name, pdev->id);

	od = omap_device_alloc(pdev, ohs, 1, NULL, 0);
	if (!od) {
		pr_err("Could not allocate od for %s\n", name);
		goto put_pdev;
	}

	res = platform_device_add_data(pdev, mmc_data,
			      sizeof(struct omap_mmc_platform_data));
	if (res) {
		pr_err("Could not add pdata for %s\n", name);
		goto put_pdev;
	}

	hsmmcinfo->pdev = pdev;

	if (hsmmcinfo->deferred)
		goto free_mmc;

	res = omap_device_register(pdev);
	if (res) {
		pr_err("Could not register od for %s\n", name);
		goto free_od;
	}

	goto free_mmc;

free_od:
	omap_device_delete(od);

put_pdev:
	platform_device_put(pdev);

free_name:
	kfree(mmc_data->slots[0].name);

free_mmc:
	kfree(mmc_data);
}

void __init omap_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	u32 reg;

	if (omap_hsmmc_done)
		return;

	omap_hsmmc_done = 1;

	if (!(cpu_is_omap44xx() || cpu_is_omap54xx())) {
		if (cpu_is_omap2430()) {
			control_pbias_offset = OMAP243X_CONTROL_PBIAS_LITE;
			control_devconf1_offset = OMAP243X_CONTROL_DEVCONF1;
		} else {
			control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
			control_devconf1_offset = OMAP343X_CONTROL_DEVCONF1;
		}
	} else if (cpu_is_omap44xx()) {
		control_pbias_offset =
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE;
		control_mmc1 = OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_MMC1;
		reg = omap4_ctrl_pad_readl(control_mmc1);
		reg |= (OMAP4_SDMMC1_PUSTRENGTH_GRP0_MASK |
			OMAP4_SDMMC1_PUSTRENGTH_GRP1_MASK);
		reg &= ~(OMAP4_SDMMC1_PUSTRENGTH_GRP2_MASK |
			OMAP4_SDMMC1_PUSTRENGTH_GRP3_MASK);
		reg |= (OMAP4_SDMMC1_DR0_SPEEDCTRL_MASK |
			OMAP4_SDMMC1_DR1_SPEEDCTRL_MASK |
			OMAP4_SDMMC1_DR2_SPEEDCTRL_MASK);
		omap4_ctrl_pad_writel(reg, control_mmc1);
	} else if (cpu_is_omap54xx()) {
		control_pbias_offset =
			OMAP5_CTRL_MODULE_CORE_PAD_CONTROL_PBIAS;
	}

	for (; controllers->mmc; controllers++)
		omap_hsmmc_init_one(controllers, controllers->mmc);

}

#endif
