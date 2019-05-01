/*
 * OMAP Voltage Controller (VC) interface
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/bug.h>


#include "voltage.h"
#include "vc.h"
#include "prm-regbits-34xx.h"
#include "prm-regbits-44xx.h"
#include "pm.h"
#include "common.h"

/**
 * struct omap_vc_channel_cfg - describe the cfg_channel bitfield
 * @sa: bit for slave address
 * @rav: bit for voltage configuration register
 * @rac: bit for command configuration register
 * @racen: enable bit for RAC
 * @cmd: bit for command value set selection
 *
 * Channel configuration bits, common for OMAP3+
 * OMAP3 register: PRM_VC_CH_CONF
 * OMAP4 register: PRM_VC_CFG_CHANNEL
 * OMAP5 register: PRM_VC_SMPS_<voltdm>_CONFIG
 */
struct omap_vc_channel_cfg {
	u8 sa;
	u8 rav;
	u8 rac;
	u8 racen;
	u8 cmd;
};

static struct omap_vc_channel_cfg vc_default_channel_cfg = {
	.sa    = BIT(0),
	.rav   = BIT(1),
	.rac   = BIT(2),
	.racen = BIT(3),
	.cmd   = BIT(4),
};

/*
 * On OMAP3+, all VC channels have the above default bitfield
 * configuration, except the OMAP4 MPU channel.  This appears
 * to be a freak accident as every other VC channel has the
 * default configuration, thus creating a mutant channel config.
 */
static struct omap_vc_channel_cfg vc_mutant_channel_cfg = {
	.sa    = BIT(0),
	.rav   = BIT(2),
	.rac   = BIT(3),
	.racen = BIT(4),
	.cmd   = BIT(1),
};

static struct omap_vc_channel_cfg *vc_cfg_bits;
#define CFG_CHANNEL_MASK 0x1f

#define VDD_AUTO_RET_DISABLE	0
#define VDD_AUTO_SLEEP		1
#define VDD_AUTO_RET		2

/**
 * omap_vc_config_channel - configure VC channel to PMIC mappings
 * @voltdm: pointer to voltagdomain defining the desired VC channel
 *
 * Configures the VC channel to PMIC mappings for the following
 * PMIC settings
 * - i2c slave address (SA)
 * - voltage configuration address (RAV)
 * - command configuration address (RAC) and enable bit (RACEN)
 * - command values for ON, ONLP, RET and OFF (CMD)
 *
 * This function currently only allows flexible configuration of the
 * non-default channel.  Starting with OMAP4, there are more than 2
 * channels, with one defined as the default (on OMAP4, it's MPU.)
 * Only the non-default channel can be configured.
 */
static int omap_vc_config_channel(struct voltagedomain *voltdm)
{
	struct omap_vc_channel *vc = voltdm->vc;

	/*
	 * For default channel, the only configurable bit is RACEN.
	 * All others must stay at zero (see function comment above.)
	 */
	if (vc->flags & OMAP_VC_CHANNEL_DEFAULT)
		vc->cfg_channel &= vc_cfg_bits->racen;

	voltdm->rmw(CFG_CHANNEL_MASK << vc->cfg_channel_sa_shift,
		    vc->cfg_channel << vc->cfg_channel_sa_shift,
		    vc->cfg_channel_reg);

	return 0;
}

/* Voltage scale and accessory APIs */
int omap_vc_pre_scale(struct voltagedomain *voltdm,
		      unsigned long target_volt,
		      struct omap_volt_data *target_v,
		      u8 *target_vsel, u8 *current_vsel)
{
	struct omap_vc_channel *vc;
	u32 vc_cmdval;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return -EINVAL;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return -EINVAL;
	}

	/* Check if sufficient pmic info is available for this vdd */
	if (!voltdm->pmic) {
		pr_err("%s: Insufficient pmic info to scale the vdd_%s\n",
			__func__, voltdm->name);
		return -EINVAL;
	}

	if (!voltdm->pmic->uv_to_vsel) {
		pr_err("%s: PMIC function to convert voltage in uV to"
			"vsel not registered. Hence unable to scale voltage"
			"for vdd_%s\n", __func__, voltdm->name);
		return -ENODATA;
	}

	if (!voltdm->read || !voltdm->write) {
		pr_err("%s: No read/write API for accessing vdd_%s regs\n",
			__func__, voltdm->name);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(target_v)) {
		pr_err("%s: No target_v info to scale vdd_%s\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(voltdm->vc_param)) {
		pr_err("%s: No vc_param info for vdd_%s\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(target_vsel)) {
		pr_err("%s: No target_vsel info to scale vdd_%s\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(current_vsel)) {
		pr_err("%s: No current_vsel info to scale vdd_%s\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	*target_vsel = voltdm->pmic->uv_to_vsel(target_volt);
	*current_vsel = voltdm->read(voltdm->vp->voltage);

	/* Setting the ON voltage to the new target voltage */
	vc_cmdval = voltdm->read(vc->cmdval_reg);
	vc_cmdval &= ~vc->common->cmd_on_mask;
	vc_cmdval |= (*target_vsel << vc->common->cmd_on_shift);
	voltdm->write(vc_cmdval, vc->cmdval_reg);

	voltdm->vc_param->on = target_volt;

	omap_vp_update_errorgain(voltdm, target_v);

	return 0;
}

void omap_vc_post_scale(struct voltagedomain *voltdm,
			unsigned long target_volt,
			struct omap_volt_data *target_vdata,
			u8 target_vsel, u8 current_vsel)
{
	struct omap_vc_channel *vc;
	u32 smps_steps = 0, smps_delay = 0;
	u8 on_vsel, onlp_vsel;
	u32 val;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return;
	}

	if (IS_ERR_OR_NULL(target_vdata)) {
		pr_err("%s: No target_vdata info to scale vdd_%s\n",
		       __func__, voltdm->name);
		return;
	}

	/* Check if sufficient pmic info is available for this vdd */
	if (!voltdm->pmic) {
		pr_err("%s: Insufficient pmic info to scale the vdd_%s\n",
		       __func__, voltdm->name);
		return;
	}

	if (!voltdm->write) {
		pr_err("%s: No write API for accessing vdd_%s regs\n",
		       __func__, voltdm->name);
		return;
	}

	smps_steps = abs(target_vsel - current_vsel);
	/* SMPS slew rate / step size. 2us added as buffer. */
	smps_delay = DIV_ROUND_UP(smps_steps * voltdm->pmic->step_size,
				  voltdm->pmic->slew_rate) + 2;
	udelay(smps_delay);

	voltdm->curr_volt = target_vdata;

	/* Set up the on voltage for wakeup from lp and OFF */
	onlp_vsel = target_vsel;
	on_vsel = target_vsel;
	val = (on_vsel << vc->common->cmd_on_shift) |
	       (onlp_vsel << vc->common->cmd_onlp_shift) |
	       vc->setup_voltage_common;
	voltdm->write(val, vc->cmdval_reg);
}

/* vc_bypass_scale - VC bypass method of voltage scaling */
int omap_vc_bypass_scale(struct voltagedomain *voltdm,
				struct omap_volt_data *target_v)
{
	struct omap_vc_channel *vc;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 vc_valid, vc_bypass_val_reg, vc_bypass_value;
	u8 target_vsel, current_vsel;
	unsigned long target_volt;
	int ret;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(target_v)) {
		pr_err("%s: No target_v info to scale vdd_%s\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return -EINVAL;
	}

	target_volt = omap_get_operation_voltage(target_v);

	ret = omap_vc_pre_scale(voltdm, target_volt,
				target_v, &target_vsel, &current_vsel);
	if (ret)
		return ret;

	vc_valid = vc->common->valid;
	vc_bypass_val_reg = vc->common->bypass_val_reg;
	vc_bypass_value = (target_vsel << vc->common->data_shift) |
		(vc->volt_reg_addr << vc->common->regaddr_shift) |
		(vc->i2c_slave_addr << vc->common->slaveaddr_shift);

	voltdm->write(vc_bypass_value, vc_bypass_val_reg);
	voltdm->write(vc_bypass_value | vc_valid, vc_bypass_val_reg);

	vc_bypass_value = voltdm->read(vc_bypass_val_reg);
	/*
	 * Loop till the bypass command is acknowledged from the SMPS.
	 * NOTE: This is legacy code. The loop count and retry count needs
	 * to be revisited.
	 */
	while (vc_bypass_value & vc_valid) {
		loop_cnt++;

		if (retries_cnt > 10) {
			pr_warning("%s: Retry count exceeded\n", __func__);
			return -ETIMEDOUT;
		}

		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = voltdm->read(vc_bypass_val_reg);
	}

	omap_vc_post_scale(voltdm, target_volt, target_v, target_vsel,
			   current_vsel);
	return 0;
}

static int omap_vc_bypass_send_value(struct voltagedomain *voltdm,
		struct omap_vc_channel *vc, u8 sa, u8 reg, u32 data)
{
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 vc_valid, vc_bypass_val_reg, vc_bypass_value;

	if (IS_ERR_OR_NULL(vc->common)) {
		pr_err("%s voldm=%s bad value for vc->common\n",
				__func__, voltdm->name);
		return -EINVAL;
	}

	vc_valid = vc->common->valid;
	vc_bypass_val_reg = vc->common->bypass_val_reg;
	vc_bypass_value = (data << vc->common->data_shift) |
		(reg << vc->common->regaddr_shift) |
		(sa << vc->common->slaveaddr_shift);

	voltdm->write(vc_bypass_value, vc_bypass_val_reg);
	voltdm->write(vc_bypass_value | vc_valid, vc_bypass_val_reg);

	vc_bypass_value = voltdm->read(vc_bypass_val_reg);
	/*
	 * Loop till the bypass command is acknowledged from the SMPS.
	 * NOTE: This is legacy code. The loop count and retry count needs
	 * to be revisited.
	 */
	while (vc_bypass_value & vc_valid) {
		loop_cnt++;

		if (retries_cnt > 10) {
			pr_warning("%s: Retry count exceeded\n", __func__);
			return -ETIMEDOUT;
		}

		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = voltdm->read(vc_bypass_val_reg);
	}

	return 0;

}

/**
 * omap_vc_bypass_send_i2c_msg() - Function to control PMIC registers over SRI2C
 * @voltdm:	voltage domain
 * @slave_addr:	slave address of the device.
 * @reg_addr:	register address to access
 * @data:	what do we want to write there
 *
 * Many simpler PMICs with a single I2C interface still have configuration
 * registers that may need population. Typical being slew rate configurations
 * thermal shutdown configuration etc. When these PMICs are hooked on I2C_SR,
 * this function allows these configuration registers to be accessed.
 *
 * WARNING: Though this could be used for voltage register configurations over
 * I2C_SR, DONOT use it for that purpose, all the Voltage controller's internal
 * information is bypassed using this function and must be used judiciously.
 */
int omap_vc_bypass_send_i2c_msg(struct voltagedomain *voltdm, u8 slave_addr,
				u8 reg_addr, u8 data)
{
	struct omap_vc_channel *vc;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return -EINVAL;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return -EINVAL;
	}

	return omap_vc_bypass_send_value(voltdm, vc, slave_addr,
					reg_addr, data);
}


/* Set oscillator setup time for omap3 */
static void omap3_set_clksetup(u32 usec, struct voltagedomain *voltdm)
{
	voltdm->write(omap_usec_to_32k(usec), OMAP3_PRM_CLKSETUP_OFFSET);
}

/**
 * omap3_set_i2c_timings - sets i2c sleep timings for a channel
 * @voltdm: channel to configure
 * @off_mode: select whether retention or off mode values used
 *
 * Calculates and sets up voltage controller to use I2C based
 * voltage scaling for sleep modes. This can be used for either off mode
 * or retention. Off mode has additionally an option to use sys_off_mode
 * pad, which uses a global signal to program the whole power IC to
 * off-mode.
 */
static void omap3_set_i2c_timings(struct voltagedomain *voltdm, bool off_mode)
{
	unsigned long voltsetup1;
	u32 tgt_volt;

	/*
	 * Oscillator is shut down only if we are using sys_off_mode pad,
	 * thus we set a minimal setup time here
	 */
	omap3_set_clksetup(1, voltdm);

	if (off_mode)
		tgt_volt = voltdm->vc_param->off;
	else
		tgt_volt = voltdm->vc_param->ret;

	/* There are NO instantaneous PMICs in existance */
	BUG_ON(!voltdm->pmic->slew_rate);

	voltsetup1 = DIV_ROUND_UP(voltdm->vc_param->on - tgt_volt,
				  voltdm->pmic->slew_rate);

	voltsetup1 = voltsetup1 * voltdm->sys_clk.rate / 8 / 1000000 + 1;

	voltdm->rmw(voltdm->vfsm->voltsetup_mask,
		voltsetup1 << __ffs(voltdm->vfsm->voltsetup_mask),
		voltdm->vfsm->voltsetup_reg);

	/*
	 * pmic is not controlling the voltage scaling during retention,
	 * thus set voltsetup2 to 0
	 */
	voltdm->write(0, OMAP3_PRM_VOLTSETUP2_OFFSET);
}

/**
 * omap3_set_off_timings - sets off-mode timings for a channel
 * @voltdm: channel to configure
 *
 * Calculates and sets up off-mode timings for a channel. Off-mode
 * can use either I2C based voltage scaling, or alternatively
 * sys_off_mode pad can be used to send a global command to power IC.
 * This function first checks which mode is being used, and calls
 * omap3_set_i2c_timings() if the system is using I2C control mode.
 * sys_off_mode has the additional benefit that voltages can be
 * scaled to zero volt level with TWL4030 / TWL5030, I2C can only
 * scale to 600mV.
 */
static void omap3_set_off_timings(struct voltagedomain *voltdm)
{
	unsigned long clksetup;
	unsigned long voltsetup2;
	unsigned long voltsetup2_old;
	u32 val;
	u32 tstart, tshut;

	/* check if sys_off_mode is used to control off-mode voltages */
	val = voltdm->read(OMAP3_PRM_VOLTCTRL_OFFSET);
	if (!(val & OMAP3430_SEL_OFF_MASK)) {
		/* No, omap is controlling them over I2C */
		omap3_set_i2c_timings(voltdm, true);
		return;
	}

	omap_pm_get_oscillator(&tstart, &tshut);
	omap3_set_clksetup(tstart, voltdm);

	clksetup = voltdm->read(OMAP3_PRM_CLKSETUP_OFFSET);

	/* There are NO instantaneous PMICs in existance */
	BUG_ON(!voltdm->pmic->slew_rate);

	/* voltsetup 2 in us */
	voltsetup2 = DIV_ROUND_UP(voltdm->vc_param->on,
				  voltdm->pmic->slew_rate);

	/* convert to 32k clk cycles */
	voltsetup2 = DIV_ROUND_UP(voltsetup2 * 32768, 1000000);

	voltsetup2_old = voltdm->read(OMAP3_PRM_VOLTSETUP2_OFFSET);

	/*
	 * Update voltsetup2 if higher than current value (needed because
	 * we have multiple channels with different ramp times), also
	 * update voltoffset always to value recommended by TRM
	 */
	if (voltsetup2 > voltsetup2_old) {
		voltdm->write(voltsetup2, OMAP3_PRM_VOLTSETUP2_OFFSET);
		voltdm->write(clksetup - voltsetup2,
			OMAP3_PRM_VOLTOFFSET_OFFSET);
	} else
		voltdm->write(clksetup - voltsetup2_old,
			OMAP3_PRM_VOLTOFFSET_OFFSET);

	/*
	 * omap is not controlling voltage scaling during off-mode,
	 * thus set voltsetup1 to 0
	 */
	voltdm->rmw(voltdm->vfsm->voltsetup_mask, 0,
		voltdm->vfsm->voltsetup_reg);

	/* voltoffset must be clksetup minus voltsetup2 according to TRM */
	voltdm->write(clksetup - voltsetup2, OMAP3_PRM_VOLTOFFSET_OFFSET);
}

static void __init omap3_vc_init_channel(struct voltagedomain *voltdm)
{
	omap3_set_off_timings(voltdm);
}

/**
 * omap4_calc_volt_ramp - calculates voltage ramping delays on omap4
 * @voltdm: channel to calculate values for
 * @voltage_diff: voltage difference in microvolts
 * @add_usec:	additional time in uSec to add
 * @clk_rate:	sys clk rate
 *
 * Calculates voltage ramp prescaler + counter values for a voltage
 * difference on omap4. Returns a field value suitable for writing to
 * VOLTSETUP register for a channel in following format:
 * bits[8:9] prescaler ... bits[0:5] counter. See OMAP4 TRM for reference.
 */
static u32 omap4_calc_volt_ramp(struct voltagedomain *voltdm, u32 voltage_diff,
		u32 add_usec, u32 clk_rate)

{
	u32 prescaler;
	u32 cycles;
	u32 time;

	/* There are NO instantaneous PMICs in existance */
	BUG_ON(!voltdm->pmic->slew_rate);

	time = DIV_ROUND_UP(voltage_diff, voltdm->pmic->slew_rate);

	time += add_usec;

	cycles = voltdm->sys_clk.rate / 1000 * time / 1000;

	cycles /= 64;
	prescaler = 0;

	/* shift to next prescaler until no overflow */

	/* scale for div 256 = 64 * 4 */
	if (cycles > 63) {
		cycles /= 4;
		prescaler++;
	}

	/* scale for div 512 = 256 * 2 */
	if (cycles > 63) {
		cycles /= 2;
		prescaler++;
	}

	/* scale for div 2048 = 512 * 4 */
	if (cycles > 63) {
		cycles /= 4;
		prescaler++;
	}

	/* check for overflow => invalid ramp time */
	if (cycles > 63) {
		pr_warning("%s: invalid setuptime for vdd_%s\n", __func__,
			   voltdm->name);
		return 0;
	}

	cycles++;

	return (prescaler << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(cycles << OMAP4430_RAMP_UP_COUNT_SHIFT);
}

/**
 * omap4_set_volt_ramp_time - set voltage ramp timings for a channel
 * @voltdm: channel to configure
 * @off_mode: whether off-mode values are used
 *
 * Calculates and sets the voltage ramp up / down values for a channel.
 */
static void omap4_set_volt_ramp_time(struct voltagedomain *voltdm,
	bool off_mode)

{
	u32 val;
	u32 rampu, rampd;
	int offset;

	if (off_mode) {
		rampu = rampd = omap4_calc_volt_ramp(voltdm,
			voltdm->vc_param->on - voltdm->vc_param->off,
			voltdm->pmic->switch_on_time, voltdm->sys_clk.rate);
		offset = voltdm->vfsm->voltsetup_off_reg;
	} else {
		rampu = rampd = omap4_calc_volt_ramp(voltdm,
			voltdm->vc_param->on - voltdm->vc_param->ret,
			0, voltdm->sys_clk.rate);
		if (!strcmp(voltdm->name, "mpu")) {
			rampu = omap4_calc_volt_ramp(voltdm,
				voltdm->vc_param->on - voltdm->vc_param->ret,
				100, voltdm->sys_clk.rate);
		}
		offset = voltdm->vfsm->voltsetup_reg;
	}

	if (!rampd || !rampu)
		return;

	val = voltdm->read(offset);

	val |= rampd << OMAP4430_RAMP_DOWN_COUNT_SHIFT;

	val |= rampu << OMAP4430_RAMP_UP_COUNT_SHIFT;

	voltdm->write(val, offset);
}

static void omap4_set_timings(struct voltagedomain *voltdm)
{
	omap4_set_volt_ramp_time(voltdm, true);
	omap4_set_volt_ramp_time(voltdm, false);
}

static int omap4_vc_sleep(struct voltagedomain *voltdm, u8 target_state)
{
	u32 val = VDD_AUTO_RET_DISABLE;
	u32 voltctrl;

	/* Return if voltdm does not support autoret */
	if (!voltdm->auto_ret)
		return 0;

	switch (target_state) {
	case PWRDM_POWER_OFF:
	case PWRDM_POWER_OSWR:
	case PWRDM_POWER_CSWR:
		val = VDD_AUTO_RET;
		break;
	}

	voltctrl = voltdm->read(voltdm->vc->common->voltctrl_reg);

	voltctrl &= ~voltdm->vc->voltctrl_mask;

	voltctrl |= val << __ffs(voltdm->vc->voltctrl_mask);

	voltdm->write(voltctrl, voltdm->vc->common->voltctrl_reg);

	return 0;
}

static int omap4_vc_wakeup(struct voltagedomain *voltdm)
{
	u32 voltctrl;

	/* Return if voltdm does not support autoret */
	if (!voltdm->auto_ret)
		return 0;

	voltctrl = voltdm->read(voltdm->vc->common->voltctrl_reg);

	voltctrl &= ~voltdm->vc->voltctrl_mask;

	voltdm->write(voltctrl, voltdm->vc->common->voltctrl_reg);

	return 0;
}

/* OMAP4 specific voltage init functions */
static void __init omap4_vc_init_channel(struct voltagedomain *voltdm)
{
	static bool is_initialized;
	struct omap_voltdm_pmic *pmic = voltdm->pmic;
	struct omap_vc_channel *vc = voltdm->vc;
	u32 vc_val = 0;

	omap4_set_timings(voltdm);

	voltdm->sleep = omap4_vc_sleep;
	voltdm->wakeup = omap4_vc_wakeup;

	if (is_initialized)
		return;

	is_initialized = true;

	if (pmic->i2c_high_speed) {
		vc_val |= pmic->i2c_hscll_low << OMAP4430_HSCLL_SHIFT;
		vc_val |= pmic->i2c_hscll_high << OMAP4430_HSCLH_SHIFT;
	}

	vc_val |= pmic->i2c_scll_low << OMAP4430_SCLL_SHIFT;
	vc_val |= pmic->i2c_scll_high << OMAP4430_SCLH_SHIFT;

	if (vc_val)
		voltdm->write(vc_val, vc->common->i2c_clk_reg);
}

/**
 * omap_vc_i2c_init - initialize I2C interface to PMIC
 * @voltdm: voltage domain containing VC data
 *
 * Use PMIC supplied settings for I2C high-speed mode and
 * master code (if set) and program the VC I2C configuration
 * register.
 *
 * The VC I2C configuration is common to all VC channels,
 * so this function only configures I2C for the first VC
 * channel registers.  All other VC channels will use the
 * same configuration.
 */
static void __init omap_vc_i2c_init(struct voltagedomain *voltdm)
{
	struct omap_vc_channel *vc = voltdm->vc;
	static bool initialized;
	static bool i2c_high_speed;
	u8 mcode;

	if (initialized) {
		if (voltdm->pmic->i2c_high_speed != i2c_high_speed)
			pr_warn("%s: I2C config for vdd_%s does not match other channels (%u).",
				__func__, voltdm->name, i2c_high_speed);
		return;
	}

	/*
	 * Dont depend on bootloader settings OR defaults, start with a blank
	 * slate
	 */
	voltdm->write(0x0, vc->common->i2c_cfg_reg);

	i2c_high_speed = voltdm->pmic->i2c_high_speed;
	if (i2c_high_speed)
		voltdm->rmw(vc->common->i2c_cfg_hsen_mask,
			    vc->common->i2c_cfg_hsen_mask,
			    vc->common->i2c_cfg_reg);

	mcode = voltdm->pmic->i2c_mcode;
	if (mcode)
		voltdm->rmw(vc->common->i2c_mcode_mask,
			    mcode << __ffs(vc->common->i2c_mcode_mask),
			    vc->common->i2c_cfg_reg);

	initialized = true;
}

/**
 * omap_vc_calc_vsel - calculate vsel value for a channel
 * @voltdm: channel to calculate value for
 * @uvolt: microvolt value to convert to vsel
 *
 * Converts a microvolt value to vsel value for the used PMIC.
 * This checks whether the microvolt value is out of bounds, and
 * adjusts the value accordingly. If unsupported value detected,
 * warning is thrown.
 */
static u8 omap_vc_calc_vsel(struct voltagedomain *voltdm, u32 uvolt)
{
	if (voltdm->pmic->vddmin > uvolt)
		uvolt = voltdm->pmic->vddmin;
	if (voltdm->pmic->vddmax < uvolt) {
		WARN(1, "%s: voltage not supported by pmic: %u vs max %u\n",
		     __func__, uvolt, voltdm->pmic->vddmax);
		/* Lets try maximum value anyway */
		uvolt = voltdm->pmic->vddmax;
	}

	return voltdm->pmic->uv_to_vsel(uvolt);
}

void __init omap_vc_init_channel(struct voltagedomain *voltdm)
{
	struct omap_vc_channel *vc;
	u8 on_vsel, onlp_vsel, ret_vsel, off_vsel;
	u32 val;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return;
	}

	if (!voltdm->pmic || !voltdm->pmic->uv_to_vsel) {
		pr_err("%s: No PMIC info for vdd_%s\n", __func__, voltdm->name);
		return;
	}

	if (!voltdm->read || !voltdm->write) {
		pr_err("%s: No read/write API for accessing vdd_%s regs\n",
			__func__, voltdm->name);
		return;
	}

	if (IS_ERR_OR_NULL(voltdm->rmw)) {
		pr_err("%s: No rmw API for reading vdd_%s regs\n",
		       __func__, voltdm->name);
		return;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return;
	}

	if (IS_ERR_OR_NULL(voltdm->vc_param)) {
		pr_err("%s: No vc_param info for vdd_%s\n",
		       __func__, voltdm->name);
		return;
	}

	vc->cfg_channel = 0;
	if (vc->flags & OMAP_VC_CHANNEL_CFG_MUTANT)
		vc_cfg_bits = &vc_mutant_channel_cfg;
	else
		vc_cfg_bits = &vc_default_channel_cfg;

	/* get PMIC/board specific settings */
	vc->i2c_slave_addr = voltdm->pmic->i2c_slave_addr;
	vc->volt_reg_addr = voltdm->pmic->volt_reg_addr;
	vc->cmd_reg_addr = voltdm->pmic->cmd_reg_addr;

	/* Configure the i2c slave address for this VC */
	voltdm->rmw(vc->smps_sa_mask,
		    vc->i2c_slave_addr << __ffs(vc->smps_sa_mask),
		    vc->smps_sa_reg);
	vc->cfg_channel |= vc_cfg_bits->sa;

	/*
	 * Configure the PMIC register addresses.
	 */
	voltdm->rmw(vc->smps_volra_mask,
		    vc->volt_reg_addr << __ffs(vc->smps_volra_mask),
		    vc->smps_volra_reg);
	vc->cfg_channel |= vc_cfg_bits->rav;

	if (vc->cmd_reg_addr) {
		voltdm->rmw(vc->smps_cmdra_mask,
			    vc->cmd_reg_addr << __ffs(vc->smps_cmdra_mask),
			    vc->smps_cmdra_reg);
		vc->cfg_channel |= vc_cfg_bits->rac;
	}

	if (vc->cmd_reg_addr == vc->volt_reg_addr)
		vc->cfg_channel |= vc_cfg_bits->racen;

	/* Set up the on, inactive, retention and off voltage */
	on_vsel = omap_vc_calc_vsel(voltdm, voltdm->vc_param->on);
	onlp_vsel = omap_vc_calc_vsel(voltdm, voltdm->vc_param->onlp);
	ret_vsel = omap_vc_calc_vsel(voltdm, voltdm->vc_param->ret);
	off_vsel = voltdm->pmic->uv_to_vsel(voltdm->vc_param->off);
	vc->setup_voltage_common =
	       (ret_vsel << vc->common->cmd_ret_shift) |
	       (off_vsel << vc->common->cmd_off_shift);
	val = (on_vsel << vc->common->cmd_on_shift) |
	       (onlp_vsel << vc->common->cmd_onlp_shift) |
	       vc->setup_voltage_common;
	voltdm->write(val, vc->cmdval_reg);
	vc->cfg_channel |= vc_cfg_bits->cmd;

	/* Channel configuration */
	omap_vc_config_channel(voltdm);

	omap_vc_i2c_init(voltdm);

	if (cpu_is_omap34xx())
		omap3_vc_init_channel(voltdm);
	else if (cpu_is_omap44xx() || cpu_is_omap54xx())
		omap4_vc_init_channel(voltdm);
}

