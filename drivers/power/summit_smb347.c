/*
 * summit_smb347.c
 *
 * Summit SMB347 Charger detection Driver
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
 * Donald Chan (hoiho@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/smb347.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/thermal_framework.h>
#include <linux/usb/omap_usb.h>

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#define THERMO_METRICS_STR_LEN 128
#endif

#define DRIVER_NAME			"smb347"
#define DRIVER_VERSION			"1.0"
#define DRIVER_AUTHOR			"Donald Chan"

/* CAUTION: Setting this param will force summit to override default current
 * limitation for USB charging while connected to PC via USB 2.0. This is
 * against the USB specification! However, AICL will be used to determine the
 * port power capability and avoid physical damage to the supplying port. */
static short int allow_unsafe_pc_usb_in_current;

module_param(allow_unsafe_pc_usb_in_current, short,
	S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(allow_unsafe_pc_usb_in_current,
		 "Used to allow drawing current higher than 500mA USB port");

enum {
	SMB347_USB_MODE_1,
	SMB347_USB_MODE_5,
	SMB347_USB_MODE_HC,
};

struct smb347_priv {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct power_supply ac;
	struct power_supply usb;
	atomic_t usb_online;
	atomic_t ac_online;
	struct otg_transceiver *otg_xceiv;
	struct mutex lock;
	struct notifier_block usb_notifier;
	struct regulator *regulator;
	struct thermal_dev *tdev;
	int max_thermal_charge_current;
	int bad_battery;
	int charge_current;
	int thermal_affect;
	atomic_t charger_on;
	/* Many of the registers cannot change without IRQ or register write.
	 * So cache their contents to save some I2C transactions. Note that
	 * still there are volatile registers that do change periodically,
	 * like temperature and actual charge current. In such cases we make
	 * sure to issue a direct I2C read. */
	u8	reg_cache[SMB_REGISTERS_CNT];
	 /* True if the reg_cache[] non-volatile contents are out of sync
	  * with the actual HW state. In other words, we've had either an IRQ
	  * or a register write. */
	u8	reg_cache_dirty;
	struct wake_lock smb_wl;
};

/* smb347_priv us used to protect i2c transactions performed on the smb347
 * device. There is possible concurency from the following sources:
 * 1. Thermal framework
 * 2. SMB347 threadeed IRQ
 * 3. Sysfs entries
 * 4. blocking call chain notifier from twl6030
 * All these can ferform read-modify-write on the device registers over i2c.
 * If not protected such kind of transaction may result in overwriting register
 * data by concurent RMW oerations on a single register. */

static const struct i2c_device_id summit_smb347_id[] =  {
	{ "smb347", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, summit_smb347_id);

static const int smb347_aicl_results[] = {
				 300,  500,  700,  900,
				1200, 1500, 1800, 2000,
				2200, 2500, 2500, 2500,
				2500, 2500, 2500, 2500
			};

static const int smb347_fast_charge_currents[] = {
				 700,  900, 1200, 1500,
				1800, 2000, 2200, 2500
			};

static const int smb347_precharge_currents[] = { 100, 150, 200, 250 };

static const int smb347_input_current[] = {
	 300,  500,  700,  900, 1200,
	1500, 1800, 2000, 2200, 2500
};

static int smb347_i2c_read(struct i2c_client *client,
			u8 reg_num, u8 *value);

static int smb347_i2c_write(struct i2c_client *client,
			u8 reg_num, u8 value);

static int smb347_i2c_read_block(struct i2c_client *i2c_client,
			u8 reg_num_start, u8 len, u8 *buff);

static int smb347_apply_in_ch_on_sett(struct smb347_priv *priv,
			u8 in_val, u8 ch_val, int suspend);

/* summit_smb347_update_reg_cache takes care bout the register cache updates.
 * Caller MUST hold priv->lock to avoid registr updates in the middle of an
 * operation!!!
 * Rrturns 0 on success or negative on error */
static int summit_smb347_update_reg_cache(struct smb347_priv *priv,
						unsigned int irq_st_upd)
{
	int status = 0;

	WARN_ON(!mutex_is_locked(&priv->lock));

	if (priv->reg_cache_dirty) {
		status = smb347_i2c_read_block(priv->i2c_client,
				SMB347_CHARGE_CURRENT, SMB_REG_MAX_BURST,
				 &priv->reg_cache[SMB347_CHARGE_CURRENT]);
		if (status)
			status = -1;
		else if (irq_st_upd) {
			status = smb347_i2c_read_block(priv->i2c_client,
					SMB347_COMMAND_REG_A, SMB_REG_MAX_BURST,
					&priv->reg_cache[SMB347_COMMAND_REG_A]);
			if (status)
				status = -2;
		} else {
			status = smb347_i2c_read_block(priv->i2c_client,
					SMB347_COMMAND_REG_A, 4,
					&priv->reg_cache[SMB347_COMMAND_REG_A]);
			if (status)
				status = -3;
			else
				status = smb347_i2c_read_block(priv->i2c_client,
					SMB347_STATUS_REG_A, 5,
					&priv->reg_cache[SMB347_STATUS_REG_A]);
			if (status)
				status = -4;
		}

		if (status)
			dev_err(priv->dev, "%s: SMB I2C registers access error code: %d\n",
					__func__, status);
		else
			priv->reg_cache_dirty = 0;
	}
	return status;
}

static inline int smb347_suspended(struct smb347_priv *priv)
{
	int ret = 0;

	ret = summit_smb347_update_reg_cache(priv, 0);
	if (ret)
		return ret;
	else
		return (priv->reg_cache[SMB347_COMMAND_REG_A] &
			SMB347_CMDA_USBIN_SUSPEND);
}

static int smb347_func_aicl_enabled(struct smb347_priv *priv)
{
	int ret = 0;

	if (summit_smb347_update_reg_cache(priv, 0))
		ret = -1;
	else
		ret = 0 != (priv->reg_cache[SUMMIT_SMB347_FUNCTIONS] &
				SMB347_FN_AICL);

	return ret;
}

static int smb347_status_usbin_in_use(struct smb347_priv *priv)
{
	int ret = 0;

	if (summit_smb347_update_reg_cache(priv, 0))
		ret = -1;
	else
		ret = 0 != (priv->reg_cache[SMB347_STATUS_REG_E] &
				ST_E_USBIN_IN_USE);

	return ret;
}

static int smb347_status_usbin_mode(struct smb347_priv *priv)
{	if (summit_smb347_update_reg_cache(priv, 0))
		return -1;
	else
		return (priv->reg_cache[SMB347_STATUS_REG_E] &
				ST_E_INPUT_MODE_MASK);
}

static int smb347_status_aicl_completed(struct smb347_priv *priv)
{
	if (summit_smb347_update_reg_cache(priv, 0))
		return -1;
	else
		return (0 != (priv->reg_cache[SMB347_STATUS_REG_E] &
				ST_E_AICL_COMPLETE));
}

static int smb347_status_apsd_completed(struct smb347_priv *priv)
{
	if (summit_smb347_update_reg_cache(priv, 0))
		return -1;
	else
		return (0 != (priv->reg_cache[SMB347_STATUS_REG_D] &
				SMB347_APSD_COMPLETE));
}

static int smb_347_status_aicl_current(struct smb347_priv *priv)
{
	if (summit_smb347_update_reg_cache(priv, 0))
		return -1;
	else
		return (smb347_aicl_results[SMB347_AICL_RESULT(
				priv->reg_cache[SMB347_STATUS_REG_E])]);
}

static const char *smb347_apsd_result_string(u8 value)
{
	switch (SMB347_APSD_RESULT(value)) {
	case SMB347_APSD_RESULT_CDP:
		return "CDP";
		break;
	case SMB347_APSD_RESULT_DCP:
		return "DCP";
		break;
	case SMB347_APSD_RESULT_OTHER:
		return "Other Downstream Port";
		break;
	case SMB347_APSD_RESULT_SDP:
		return "SDP";
		break;
	case SMB347_APSD_RESULT_ACA:
		return "ADA charger";
		break;
	case SMB347_APSD_RESULT_TBD_1:
	case SMB347_APSD_RESULT_TBD_2:
		return "TBD";
		break;
	case -1:
		return "not run";
		break;
	case SMB347_APSD_RESULT_NONE:
	default:
		return "unknown";
		break;
	}
}

/* smb347_config caller MUST must hold priv->lock to avoid operation
 * concurency during operation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
static int smb347_config(struct smb347_priv *priv, int enable)
{
	int ret = 0;
	unsigned char value = 0xff;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_i2c_read(priv->i2c_client,
			SMB347_COMMAND_REG_A, &value);
	if (ret)
		goto err_out;

	if (enable)
		value |= SMB347_CMDA_NV_CONF_WR_ENABLE;
	else
		value &= ~SMB347_CMDA_NV_CONF_WR_ENABLE;

	ret = smb347_i2c_write(priv->i2c_client,
			SMB347_COMMAND_REG_A, value);

err_out:
	if (ret)
		dev_err(priv->dev,
			"Unable to %s writes to CONFIG regs (err:%d)\n",
			enable ? "enable" : "disable", ret);
	return ret;
}

/* smb347_disable_aicl caller MUST must hold priv->lock to avoid operation
 * concurency during operation */

static int smb347_disable_aicl(struct smb347_priv *priv, int disable)
{
	int ret = 0;
	u8 value = 0xff;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_i2c_read(priv->i2c_client,
				SUMMIT_SMB347_FUNCTIONS, &value);
	if (ret)
		goto err_out;

	if (disable)
		value &= ~SMB347_FN_AICL;
	else
		value |= SMB347_FN_AICL;

	ret = smb347_i2c_write(priv->i2c_client,
				SUMMIT_SMB347_FUNCTIONS, value);

err_out:
	smb347_config(priv, 0);

	return ret;
}

/* smb347_redo_apsd caller MUST must hold priv->lock to protect register
 * consistency during operation */

static int smb347_redo_apsd(struct smb347_priv *priv)
{
	u8 value;
	int ret;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_i2c_read(priv->i2c_client,
				SUMMIT_SMB347_CHARGE_CONTROL, &value);
	if (ret)
		goto err_out;

	value &= ~SMB347_CHRG_CTL_FLEX_CHARGE;
	ret = smb347_i2c_write(priv->i2c_client,
				SUMMIT_SMB347_CHARGE_CONTROL, value);
	if (ret)
		goto err_out;

	msleep(20);

	value |= SMB347_CHRG_CTL_FLEX_CHARGE;
	ret = smb347_i2c_write(priv->i2c_client,
				SUMMIT_SMB347_CHARGE_CONTROL, value);
	if (ret)
		goto err_out;

err_out:
	smb347_config(priv, 0);
	return ret;
}

/* smb347_redo_aicl caller MUST must hold priv->lock to protect register
 * consistency during operation */
static int smb347_redo_aicl(struct smb347_priv *priv)
{

	u8 temp;
	int ret = 0;

	WARN_ON(!mutex_is_locked(&priv->lock));

	/* For protection from previous time requiring operation */
	msleep(20);

	ret = smb347_i2c_read(priv->i2c_client,
					SUMMIT_SMB347_FUNCTIONS, &temp);
	if (ret)
		return ret;

	/* Disable AICL */
	temp &= ~SMB347_FN_AICL;
	ret = smb347_i2c_write(priv->i2c_client,
					SUMMIT_SMB347_FUNCTIONS, temp);
	if (ret)
		return ret;

	msleep(20);

	/* Enable AICL */
	temp |= SMB347_FN_AICL;

	ret = smb347_i2c_write(priv->i2c_client,
				SUMMIT_SMB347_FUNCTIONS, temp);
	if (ret)
		return ret;

	return ret;
}

static int smb347_fast_charge_current_limit(struct smb347_priv *priv)
{
	int ret = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_i2c_read(priv->i2c_client,
			SMB347_CHARGE_CURRENT, &value);
	if (ret)
		goto err_out;

	/* Limit Fast Charge Current to 1800 mA */
	value &= ~CHARGE_CURR_MASK;
	value |= CHARGE_CURR_1800;

	ret = smb347_i2c_write(priv->i2c_client, SMB347_CHARGE_CURRENT, value);

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);
	return ret;
}

static int smb347_set_termination_current(struct smb347_priv *priv)
{
	int ret = -1;
	u8 val = 0xff;

	mutex_lock(&priv->lock);
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_i2c_read(priv->i2c_client,
					SMB347_CHARGE_CURRENT, &val);
	if (ret)
		goto err_out;

	/* Clear bit 2, set bits 0 and 1 (Termination current = 150 mA) */
	val &= ~TERM_CURR_MASK;
	val |= TERM_CURR_150;

	ret = smb347_i2c_write(priv->i2c_client, SMB347_CHARGE_CURRENT, val);

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to set termination current: %d\n",
			__func__, ret);

	return ret;
}

static int smb347_set_charge_timeout_interrupt(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client,
			SUMMIT_SMB347_INTERRUPT_STAT, &value);
	if (ret)
		goto err_out;

	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	/* Set bit 7 for enable Charge-Timeout interrupt */
	value |= SMB347_STAT_IRQ_CHARGE_TIMEOUT;

	ret = smb347_i2c_write(priv->i2c_client,
			SUMMIT_SMB347_INTERRUPT_STAT, value);

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to write SUMMIT_SMB347_INTERRUPT_STAT: %d\n",
			__func__, ret);

	return ret;
}

#if defined(CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM)
static int smb347_set_current_setting(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0;

	mutex_lock(&priv->lock);

	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	value = CHARGE_CURR_1800 | PRECHARGE_CURR_250 | TERM_CURR_250;

	ret = smb347_i2c_write(priv->i2c_client,
			       SMB347_CHARGE_CURRENT, value));

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to apply charge settings, err=%d\n",
			__func__, ret);

	return ret;
}

static int smb347_set_current_compensation(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	mutex_lock(&priv->lock);
	ret = smb347_i2c_read(priv->i2c_client,
				SUMMIT_SMB347_OTG_THERM_CONTROL, &value);
	if (ret)
		goto err_out;

	err = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	value &= ~SMB347_OTH_CTL_CHG_CURR_COMP_MASK;

	/* Set bit 7 (Charge current compensation = 900 mA) */
	value |= SMB347_OTH_CTL_CHG_CURR_COMP_900;

	ret = smb347_i2c_write(priv->i2c_client,
			SUMMIT_SMB347_OTG_THERM_CONTROL, value);

err_out:
	smb347_config(priv, 0)
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to set current compensation, err=%d\n",
			__func__, ret);

	return ret;
}

static int smb347_set_temperature_threshold(struct smb347_priv *priv)
{
	int ret = -1;

	mutex_lock(&priv->lock);

	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	/* Set bit 7 (Hard Cold temp = 0 degree C) */
	/* Set bit 5 (Hard Hot temp = 60 degree C) */
	/* Clear bits 2 and 3 (Soft Cold temp = 15 degree C) */
	/* Set bit 0 (Soft Hot temp = 45 degree C) */
	/* value = 0xA1; */

	ret = smb347_i2c_write(priv->i2c_client,
				SUMMIT_SMB347_CELL_TEMP,
				SMB347_CELL_HARD_LIM_COLD_ALRM_0 |
				SMB347_CELL_HARD_LIM_HOT_ALRM_60 |
				SMB347_CELL_SOFT_LIM_COLD_ALRM_15 |
				SMB347_CELL_SOFT_LIM_HOT_ALRM_45);

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to set temperature threshold, err=%d\n",
			__func__, ret);

	return ret;
}

static int smb347_set_temperature_interrupt(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	mutex_lock(&priv->lock);
	ret = smb347_i2c_read(priv->i2c_client,
			SUMMIT_SMB347_FAULT_INTERRUPT, &value);
	if (ret)
		goto err_out;

	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	value = SMB347_FAULT_IRQ_AICL_COMPLETE |
		SMB347_FAULT_IRQ_TEMP_OUT_SOFT_LIMITS |
		SMB347_FAULT_IRQ_TEMP_OUT_HARD_LIMITS;

	ret = smb347_i2c_write(priv->i2c_client,
			SUMMIT_SMB347_FAULT_INTERRUPT, value);

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to write SUMMIT_SMB347_FAULT_INTERRUPT: %d\n",
			__FUNCTION__, ret);
	}

	return ret;
}
#endif

static void smb347_pr_creg(struct smb347_priv *priv, u8 reg)
{
	u8 reg_val = priv->reg_cache[reg];
	dev_info(priv->dev, "REG-0x%02X=0x%02X\n", reg, reg_val);
}

static int smb347_set_charge_control(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	mutex_lock(&priv->lock);
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_i2c_read(priv->i2c_client, SMB347_ENABLE_CONTROL, &value);
	if (ret)
		goto err_out;

	ret = -1;

	/* Clear bits 5 and 6 */
	value &= ~SMB347_ENABLE_CTL_I2C_0CMD_MASK;

	/* Set bit 5 */
	value |= SMB347_ENABLE_CTL_I2C_0CMD_CH_ENA;

	/* Clear bits 4, Set HC mode to Register Control */
	value &= ~SMB347_ENABLE_CTL_USB_HC_PIN_nREG_CTL;

	/* Trigger IRQ when APSD Done, ==! */
	value |= SMB347_ENABLE_CTL_APSD_DONE_IRQ_ENA;

	ret = smb347_i2c_write(priv->i2c_client,
			SMB347_ENABLE_CONTROL, value);

err_out:
	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to set charge control, err:%d\n",
			__func__, ret);
	return ret;
}

/* must be called under priv->lock held by caller !!! */
static int smb347_switch_mode(struct smb347_priv *priv, int mode)
{
	int ret = 0;
	unsigned char value = 0xff;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_i2c_read(priv->i2c_client, SMB347_COMMAND_REG_B, &value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_COMMAND_REG_B: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		switch (mode) {
		case SMB347_USB_MODE_1:
			dev_info(priv->dev, "Switching to USB1 mode\n");
			value &= ~SMB347_CMDB_MODE_MASK;
			break;
		case SMB347_USB_MODE_5:
			dev_info(priv->dev, "Switching to USB5 mode\n");
			value &= ~SMB347_CMDB_MODE_MASK;
			value |= SMB347_CMDB_USB5_MODE;
			break;
		case SMB347_USB_MODE_HC:
			dev_info(priv->dev, "Switching to HC mode\n");
			value &= ~SMB347_CMDB_MODE_MASK;
			value |= SMB347_CMDB_HC_MODE;
			break;
		default:
			dev_err(priv->dev, "Unknown USB mode: %d\n", mode);
			return -1;
		}

		ret = smb347_i2c_write(priv->i2c_client,
					SMB347_COMMAND_REG_B, value);
		if (ret) {
			dev_err(priv->dev,
				"%s: Unable to write SMB347_COMMAND_REG_B: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

/* must be called under priv->lock held by caller */
static int smb347_change_timeout_usb(struct smb347_priv *priv, int enable)
{
	u8 value = 0xff;
	int ret = 0;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_i2c_read(priv->i2c_client,
				SUMMIT_SMB347_STAT_TIMERS, &value);
	if (ret)
		goto err_out;

	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	value &= ~SMB347_TIMERS_CHARGE_COMPLETE_TO_MASK;
	if (enable)
		value |= SMB347_TIMERS_CHARGE_COMPLETE_TO_1527;
	else
		value |= SMB347_TIMERS_CHARGE_COMPLETE_TO_764;

	ret = smb347_i2c_write(priv->i2c_client,
			SUMMIT_SMB347_STAT_TIMERS, value);

err_out:
	smb347_config(priv, 0);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable charge timeout, err: %d\n", __func__, ret);
	return ret;
}
#ifdef CONFIG_AMAZON_METRICS_LOG
void usb_log_metrics(char *usbmsg)
{
	struct timespec ts = current_kernel_time();
	char buf[512];
	snprintf(buf, sizeof(buf),
		"usb_connection_status:def:%s=1;CT;1,timestamp=%lu;TI;1:NR",
		usbmsg,
		ts.tv_sec * 1000 + ts.tv_nsec / NSEC_PER_MSEC);
	log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
}
#endif

/* smb347_handle_apsd_complete takes care to parse power source type and set
 * the operation mode. Caller MUST hold priv->lock to protect the registrs
 * from update in the middle of the operation!!! */
static int smb347_handle_apsd_complete(struct smb347_priv *priv)
{
	int type = POWER_SUPPLY_TYPE_UNKNOWN;

	/* No need to re-read STATUS_D, since we know APSD has completed by
	 * checking STATUS_D^3 right before we call this handler. If false
	 * positive (eg cable already disconnected) disconnect event is
	 * pending, which will cause execution of the whole event chain, not
	 just skipping the APSD handling */

	dev_info(priv->dev, "Detected charger: %s\n",
		smb347_apsd_result_string(
			priv->reg_cache[SMB347_STATUS_REG_D]));
#ifdef CONFIG_AMAZON_METRICS_LOG
	usb_log_metrics("usb_connected");
#endif
	switch (SMB347_APSD_RESULT(priv->reg_cache[SMB347_STATUS_REG_D])) {
	case SMB347_APSD_RESULT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 1);
		smb347_switch_mode(priv,
				   (allow_unsafe_pc_usb_in_current != 0) ?
				   SMB347_USB_MODE_HC : SMB347_USB_MODE_5);
		smb347_change_timeout_usb(priv, 1);
		break;

	case SMB347_APSD_RESULT_CDP:
		type = POWER_SUPPLY_TYPE_USB;
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 1);
		smb347_switch_mode(priv, SMB347_USB_MODE_HC);
		smb347_change_timeout_usb(priv, 1);
		break;

	case SMB347_APSD_RESULT_OTHER:
		/* Switch to HC mode for other charging port */
		smb347_switch_mode(priv, SMB347_USB_MODE_HC);

	case SMB347_APSD_RESULT_DCP:
		atomic_set(&priv->ac_online, 1);
		atomic_set(&priv->usb_online, 0);

		type = POWER_SUPPLY_TYPE_USB_DCP;

		smb347_change_timeout_usb(priv, 0);
		break;

	case SMB347_APSD_RESULT_TBD_1:
	case SMB347_APSD_RESULT_TBD_2:
		atomic_set(&priv->ac_online, 1);
		atomic_set(&priv->usb_online, 0);
		smb347_change_timeout_usb(priv, 0);
		break;

	default:
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 0);
		break;
	}

	power_supply_changed(&priv->usb);
	power_supply_changed(&priv->ac);

	return type;
}

static irqreturn_t summit_smb347_irq(int irq, void *data)
{
	struct smb347_priv *priv = (struct smb347_priv *)data;
	u8 tmp = 0;
	int i;

	dev_dbg(priv->dev, "in %s\n", __func__);

	mutex_lock(&priv->lock);
	priv->reg_cache_dirty = 1;
	i = summit_smb347_update_reg_cache(priv, 1);

	if (i < 0) {
		dev_err(priv->dev, "%s: Cache update error!\n", __func__);
		mutex_unlock(&priv->lock);
		return IRQ_HANDLED;
	}

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "smb347 REGS IRQ: ", DUMP_PREFIX_NONE, 16, 1,
			&priv->reg_cache, 64, 1);
#endif

	/* Check for USBIN Power events */
	if (priv->reg_cache[SMB347_INTSTAT_REG_F] & SMB347_INTSTF_POK_IRQ) {
		/* DO NOT send chargers POWER events on USB controller !!!!!
		* If we have disabled the charger we will get status
		* disconnected, which sent to the USB/PHY causes state machine
		* missmatch. USB PHY has it's own mechanishm to detect VBUS
		* and handle the changes. */
		if ((priv->reg_cache[SMB347_INTSTAT_REG_F] &
			SMB347_INTSTF_POK_STA) != 0) {
			if (!wake_lock_active(&priv->smb_wl)) {
				wake_lock(&priv->smb_wl);
				dev_info(priv->dev, "Wake locked.\n");
			}
			atomic_set(&priv->charger_on, 1);
			twl6030_usb_event(TWL6030_USB_EVENT_VBUS_ON);
		} else {
			if (wake_lock_active(&priv->smb_wl)) {
				wake_unlock(&priv->smb_wl);
				dev_info(priv->dev, "Wake unlocked.\n");
			}
			atomic_set(&priv->charger_on, 0);
		}
		smb347_pr_creg(priv, SMB347_ENABLE_CONTROL);
		smb347_pr_creg(priv, SMB347_STATUS_REG_D);
	}

	dev_dbg(priv->dev, "%s: INTSTAT_REG_A is 0x%02x\n",
		__func__, priv->reg_cache[SMB347_INTSTAT_REG_A]);
	tmp = priv->reg_cache[SMB347_INTSTAT_REG_A];
	for (i = 0; i < 4; i++) {
		if ((tmp & SMB347_INTSTX_FLMASK) == SMB347_INTSTX_IRQ_RISE) {
			dev_warn(priv->dev,
				"%s: Temerature interrupt occurs: 0x%02X\n",
				__func__,
				priv->reg_cache[SMB347_INTSTAT_REG_A]);
			break;
		}
		tmp >>= 2;
	}

	if (RISE_IRQ(SMB347_INTSTB, OVR_VOL))
		dev_warn(priv->dev,
			"%s: Battery over voltage interrupt occurs: 0x%02X\n",
			__func__, priv->reg_cache[SMB347_INTSTAT_REG_B]);

	if (RISE_IRQ(SMB347_INTSTC, INT_TRM))
		dev_warn(priv->dev,
			"%s: Internal temp limit interrupt occurs: 0x%02X\n",
			__func__, priv->reg_cache[SMB347_INTSTAT_REG_C]);

	/* Check for APSD Complete */
	if (RISE_IRQ(SMB347_INTSTD, APSD_COMPLETE) &&
		smb347_status_apsd_completed(priv) &&
		atomic_read(&priv->charger_on))
		twl6030_usb_event(TWL6030_USB_EVENT_VBUS_DETECT);

	/* Check for AICL Complete */
	if (RISE_IRQ(SMB347_INTSTD, AICL_COMPLETE) &&
		smb347_status_aicl_completed(priv))
		dev_info(priv->dev, "AICL result: %d mA\n",
			smb_347_status_aicl_current(priv));

	/* Check for pre-charging timeout */
	if (RISE_IRQ(SMB347_INTSTD, PRECHG_COMPLETE_TO))
		dev_warn(priv->dev, "Pre-charging complete timeout!\n");

	/* Check for charging timeout */
	if (RISE_IRQ(SMB347_INTSTD, CHG_COMPLETE_TO))
		dev_warn(priv->dev, "Charge completed timeout!\n");

	memset(&priv->reg_cache[SMB347_INTSTAT_REG_A], 0, 6);
	mutex_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int smb347_i2c_read_block(struct i2c_client *i2c_client,
				u8 reg_num_start, u8 len, u8 *buff)
{
	return (len != i2c_smbus_read_i2c_block_data(i2c_client,
				reg_num_start, len, buff));
}

static int smb347_i2c_read(struct i2c_client *i2c_client,
				u8 reg_num, u8 *value)
{
	struct smb347_priv *priv = i2c_get_clientdata(i2c_client);
	s32 error;

	error = i2c_smbus_read_byte_data(i2c_client, reg_num);

	if (error < 0) {
		dev_err(priv->dev,
			"i2c error at %s: %d\n", __FUNCTION__, error);
		return error;
	}

	priv->reg_cache[reg_num] = (unsigned char) (error & 0xff);
	*value = (unsigned char) (error & 0xff);
	return 0;
}

static int smb347_i2c_write(struct i2c_client *i2c_client,
				u8 reg_num, u8 value)
{
	struct smb347_priv *priv = i2c_get_clientdata(i2c_client);
	s32 error;

	error = i2c_smbus_write_byte_data(i2c_client, reg_num, value);

	if (error < 0) {
		dev_err(priv->dev,
			"i2c error at %s: %d\n", __FUNCTION__, error);
	}

	priv->reg_cache_dirty = 1;

	return error;
}

static int summit_smb347_read_id(struct smb347_priv *priv, int *id)
{
	int error = 0;
	unsigned char value = 0xff;

	error = smb347_i2c_read(priv->i2c_client,
				SUMMIT_SMB347_ID, &value);

	if (!error)
		*id = value;

	return error;
}

/* Enable/disable charging */
static int smb347_enable_charging(struct smb347_priv *priv, int enable)
{
	int ret = -1;
	unsigned char value = 0xff;

	WARN_ON(!mutex_is_locked(&priv->lock));

	ret = smb347_i2c_read(priv->i2c_client, SMB347_COMMAND_REG_A, &value);
	if (ret)
		goto err_out;

	if (enable)
		value |= SMB347_CMDA_CHARGING_ENABLE;
	else
		value &= ~SMB347_CMDA_CHARGING_ENABLE;

	ret = smb347_i2c_write(priv->i2c_client, SMB347_COMMAND_REG_A, value);

err_out:

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to set charging, err=%d\n",
			__func__, ret);

	return ret;
}

static enum power_supply_property smb347_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property smb347_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


/* USB property */
static int smb347_get_usb_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct smb347_priv *priv = container_of(ps, struct smb347_priv, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&priv->usb_online);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* AC property */
static int smb347_get_ac_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct smb347_priv *priv = container_of(ps, struct smb347_priv, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&priv->ac_online);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb347_usb_notifier_cb(struct notifier_block *nb,
			unsigned long val, void *data)
{
	struct smb347_priv *priv;
	enum power_supply_type *supply = data;

	priv = container_of(nb, struct smb347_priv, usb_notifier);

	switch (val) {
	case TWL6030_USB_EVENT_VBUS_ON:
		dev_info(priv->dev, "in %s, VBUS ON.\n",
			__func__);
		mutex_lock(&priv->lock);
		if (atomic_read(&priv->charger_on))
			dev_info(priv->dev, "Restarting VBUS detection.\n");
		else {
			dev_info(priv->dev, "Starting VBUS detection.\n");
			atomic_set(&priv->charger_on, 1);
		}
		smb347_redo_apsd(priv);
		mutex_unlock(&priv->lock);
		break;

	case TWL6030_USB_EVENT_VBUS_OFF:
		dev_info(priv->dev, "in %s, VBUS OFF\n", __func__);
		/* USB disconnected */
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 0);

		power_supply_changed(&priv->ac);
		power_supply_changed(&priv->usb);

		dev_info(priv->dev, "USB disconnected\n");
#ifdef CONFIG_AMAZON_METRICS_LOG
		usb_log_metrics("usb_disconnected");
#endif
		break;

	case TWL6030_USB_EVENT_VBUS_DETECT:
		dev_info(priv->dev, "%s-EVENT_VBUS DETECT | charger_on = %d\n",
				__func__, atomic_read(&priv->charger_on));
		mutex_lock(&priv->lock);
		*supply = smb347_handle_apsd_complete(priv);
		mutex_unlock(&priv->lock);
		break;
	}
	return 0;
}

static ssize_t smb347_status_a_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, voltage = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
					SMB347_STATUS_REG_A, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB347_STATUS_REG_A: %d\n", error);
		goto err_out;
	}

	len += sprintf(buf + len, "SMB347_STATUS_REG_A = 0x%02x\n\n", value);
	len += sprintf(buf + len, "Thermal Regulation Status: %s\n",
			(value & SMB347_STA_THERM_SOFT_LIM_REG_STATUS) ?
			"Active" : "Inactive");
        len += sprintf(buf + len, "THERM Soft Limit Regulation Status: %s\n",
			(value & SMB347_STA_THERM_REG_STATUS) ?
			"Active" : "Inactive");

	voltage = 3500 + (value & SMB347_STA_ACTUAL_FLOAT_VOLT_MASK) * 20;

	/* Max out at 4500 mV */
	if (voltage > 4500) voltage = 4500;

	len += sprintf(buf + len,
		"Actual Float Voltage after compensation: %d mV\n", voltage);

err_out:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_a, S_IRUGO, smb347_status_a_show, NULL);

static inline int smb347_charging_current(int reg)
{
	int curr = -1;

	if (reg & SMB347_STB_FAST_nPRE_CHARGING)
		curr = smb347_fast_charge_currents[reg &
				SMB347_STB_FC_CURRENT_MASK];
	else
		curr = smb347_precharge_currents[(reg &
				SMB347_STB_PC_CURRENT_MASK) >>
				SMB347_STB_PC_CURRENT_SHIFT];

	return curr;
}

static ssize_t smb347_status_b_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	int ret = 0;
	int curr = 0;
	u8 reg_b_val;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_C, &reg_b_val);
	if (ret)
		goto err_out;

	len += sprintf(buf + len, "SMB347_STATUS_REG_B = 0x%02x\n\n",
		       reg_b_val);

	len += sprintf(buf + len, "USB Suspend Mode: %s\n",
			(reg_b_val & SMB347_STB_USB_SUSPEND_MODE) ?
			"Active" : "Inactive");

	curr = smb347_charging_current(reg_b_val);

	if (curr != -1)
		len += sprintf(buf + len, "Actual Charge Current after "
				"compensation: %d mA\n", curr);
	else
		len += sprintf(buf + len, "Actual Charge Current after "
				"compensation: Unknown\n");

err_out:
	mutex_unlock(&priv->lock);

	if (ret)
		len += sprintf(buf + len, "Error reading data!\n");

	return len;
}
static DEVICE_ATTR(status_b, S_IRUGO, smb347_status_b_show, NULL);

static ssize_t smb347_status_c_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	int ret = 0;
	u8 reg_c_val;

	mutex_lock(&priv->lock);
	ret = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_C, &reg_c_val);
	if (ret)
		goto err_out;

	len += sprintf(buf + len, "SMB347_STATUS_REG_C = 0x%02x\n\n",
			reg_c_val);

	len += sprintf(buf + len, "Charging Enable/Disable: %s\n",
			(reg_c_val & SMB347_STC_CHARGING_ENABLED) ?
			"Enabled" : "Disabled");

	switch (SMB347_STC_CHARGING_STATUS(reg_c_val)) {
	case SMB347_CHARGING_STATUS_NOT_CHARGING:
		len += sprintf(buf + len, "Charging Status: Not charging\n");
		break;
	case SMB347_CHARGING_STATUS_PRE_CHARGING:
		len += sprintf(buf + len, "Charging Status: Pre-charging\n");
		break;
	case SMB347_CHARGING_STATUS_FAST_CHARGING:
		len += sprintf(buf + len, "Charging Status: Fast-charging\n");
		break;
	case SMB347_CHARGING_STATUS_TAPER_CHARGING:
		len += sprintf(buf + len, "Charging Status: Taper-charging\n");
		break;
	default:
		len += sprintf(buf + len, "Charging Status: Unknown\n");
		break;
	}

	len += sprintf(buf + len, "Charger %s hold-off status\n",
			(reg_c_val & SMB347_STC_CHARGER_IN_HOLDOFF) ?
			"in" : "not in");

	len += sprintf(buf + len, "Vbatt %c 2.1 V\n",
			(reg_c_val & SMB347_STC_VABAT_LESS_THAN2_1V) ?
			'<' : '>');

	len += sprintf(buf + len,
			(reg_c_val & SMB347_STC_FULL_CHARGE_CYCLE) ?
			"At least one charging cycle has terminated\n" :
			"No full charge cycle has occurred\n");

	if (reg_c_val & SMB347_STC_CHARGER_ERROR)
		len += sprintf(buf + len,
			"Charger has encountered an error\n");

	len += sprintf(buf + len, "Charger error %s an IRQ signal\n",
			(reg_c_val & SMB347_STC_CHARGER_ERR_ASSERTS_IRQ) ?
			"asserts" : "does not assert");

err_out:
	mutex_unlock(&priv->lock);

	if (ret)
		len += sprintf(buf + len, "Error reading data!\n");

	return len;
}
static DEVICE_ATTR(status_c, S_IRUGO, smb347_status_c_show, NULL);

static ssize_t smb347_status_d_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	int ret = 0;
	u8 reg_d_val;

	mutex_lock(&priv->lock);
	ret = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_D, &reg_d_val);
	if (ret)
		goto err_out;

	len += sprintf(buf + len, "SMB347_STATUS_REG_D = 0x%02x\n\n",
		       reg_d_val);

	if (smb347_status_apsd_completed(priv))
		len += sprintf(buf + len, "APSD completed, result: %s\n",
				smb347_apsd_result_string(reg_d_val));
	else
		len += sprintf(buf + len, "APSD not completed\n");

err_out:
	mutex_unlock(&priv->lock);

	if (ret)
		len += sprintf(buf + len, "Error reading data!\n");

	return len;
}
static DEVICE_ATTR(status_d, S_IRUGO, smb347_status_d_show, NULL);

static ssize_t smb347_status_e_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	int ret = 0;
	u8 reg_e_val;

	mutex_lock(&priv->lock);
	ret = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_E, &reg_e_val);
	if (ret)
		goto err_out;

	len += sprintf(buf + len, "SMB347_STATUS_REG_E = 0x%02x\n\n",
			reg_e_val);

	len += sprintf(buf + len, "USBIN Input: %s\n",
		(reg_e_val & ST_E_USBIN_IN_USE) ? "In Use" : "Not In Use");

	switch (SMB347_USB1_5_HC_MODE(reg_e_val)) {
	case SMB347_HC_MODE:
		len += sprintf(buf + len, "In HC mode\n");
		break;
	case SMB347_USB1_MODE:
		len += sprintf(buf + len, "In USB1 mode\n");
		break;
	case SMB347_USB5_MODE:
		len += sprintf(buf + len, "In USB5 mode\n");
		break;
	}

	if (reg_e_val & ST_E_AICL_COMPLETE)
		len += sprintf(buf + len,
			"AICL Completed, Result = %d mA\n",
			smb_347_status_aicl_current(priv));
	else
		len += sprintf(buf + len, "AICL not completed\n");

err_out:
	mutex_unlock(&priv->lock);

	if (ret)
		len += sprintf(buf + len, "Error reading data!\n");

	return len;
}
static DEVICE_ATTR(status_e, S_IRUGO, smb347_status_e_show, NULL);

/* smb347_modify_input_current caller MUST have priv->lock obtained ! */
static int smb347_set_input_current(struct smb347_priv *priv,
						int input_current)
{
	int ret = 0;
	unsigned char value = ((input_current << 4) | input_current) & 0xff;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_i2c_write(priv->i2c_client,
				SUMMIT_SMB347_INPUT_CURR_LIMIT, value);

err_out:
	smb347_config(priv, 0);

	if (ret)
		dev_err(priv->dev,
			"%s: Unable to set SMB347_INPUT_CURR_LIMIT: %d\n",
			__func__, ret);

	return ret;
}

/* smb347_modify_input_current caller MUST have priv->lock obtained ! */
static int smb347_modify_input_current(struct smb347_priv *priv,
						int input_current)
{
	int i = 0, ret = -1;

	WARN_ON(!mutex_is_locked(&priv->lock));
	if (input_current < smb347_input_current[0]) {
		dev_info(priv->dev,
		"The value(%d) is too small thus change to the lowest setting(%d)",
			input_current, smb347_input_current[0]);
		input_current = smb347_input_current[0];
	}

	for (i = ARRAY_SIZE(smb347_input_current) - 1; i >= 0; i--) {
		if (smb347_input_current[i] == input_current) {
			break;
		} else if (smb347_input_current[i] < input_current) {
			dev_info(priv->dev,
			"The value(%d) is not in the steps thus change to the proper setting(%d)",
				input_current, smb347_input_current[i]);
			break;
		}
	}

	ret = smb347_set_input_current(priv, i);

	dev_dbg(priv->dev, "Change input current limit to %d\n",
					smb347_input_current[i]);
	return ret;
}

/* smb347_force_charging_setting MUST haver obtained priv->lock ! */
static int smb347_force_charging_setting(struct smb347_priv *priv, int force)
{
	int ret = 0;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_disable_aicl(priv, force);
	if (ret) {
		dev_err(priv->dev,
			"Failed to disable AICL function\n");
		goto done;
	}

	if (force) {
		/* Switch to HC mode */
		dev_dbg(priv->dev, "Disable AICL and force HC mode.\n");
		ret = smb347_switch_mode(priv, SMB347_USB_MODE_HC);
	} else {
		/* Redo APSD */
		dev_dbg(priv->dev, "Enable AICL and redo APSD.\n");
		smb347_redo_apsd(priv);
	}

done:
	return ret;
}

/*  smb347_get_usbin_power_caps caller MUST hold priv->lock ! */
static int smb347_get_usbin_power_caps(struct smb347_priv *priv)
{
	int curr = 0;

	WARN_ON(!mutex_is_locked(&priv->lock));
	if (summit_smb347_update_reg_cache(priv, 0))
		curr = -1;
	else if (smb347_status_usbin_in_use(priv)) {
		switch (smb347_status_usbin_mode(priv)) {
		case ST_E_INPUT_MODE_HC:
			if (smb347_func_aicl_enabled(priv) &&
				smb347_status_aicl_completed(priv))
				curr = smb_347_status_aicl_current(priv);
			break;

		case ST_E_INPUT_MODE_USB1:
			if (smb347_status_apsd_completed(priv))
				curr = 100;
			break;

		case ST_E_INPUT_MODE_USB5:
			if (smb347_status_apsd_completed(priv))
				curr = 500;
			break;
		}
	}

	return curr;
}

static ssize_t smb347_charge_input_uamps_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	int curr;

	mutex_lock(&priv->lock);

	curr = smb347_get_usbin_power_caps(priv);

	if (curr > 0)
		curr *= 1000;
	len = sprintf(buf + len, "%d\n", curr);

	mutex_unlock(&priv->lock);

	return len;
}

static DEVICE_ATTR(charge_input_uamps, S_IRUGO | S_IRUSR | S_IRGRP,
			smb347_charge_input_uamps_show,
			NULL);

static ssize_t smb347_charge_input_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;

	mutex_lock(&priv->lock);
	len = sprintf(buf + len, "%d\n", smb347_get_usbin_power_caps(priv));
	mutex_unlock(&priv->lock);

	return len;
}

static ssize_t smb347_charge_input_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtol(buf, NULL, 10);

	mutex_lock(&priv->lock);

	if (value < 0) {
		//reset input limit to default 1800mA
		if (smb347_modify_input_current(priv, 1800) ||
			smb347_force_charging_setting(priv, 0)) {
			dev_err(priv->dev,
				"Reset AICL failed\n");
			len = -1;
			goto done;
		}
	} else {
		if (smb347_modify_input_current(priv, value) ||
			smb347_force_charging_setting(priv, 1)) {
			dev_err(priv->dev,
				"Set input manually failed\n");
			len = -1;
			goto done;
		}
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_input, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_charge_input_show,
			smb347_charge_input_store);

static ssize_t smb347_usb_input_current_override_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;

	mutex_lock(&priv->lock);
	len = sprintf(buf + len, "Override %s\n",
		      allow_unsafe_pc_usb_in_current ? "enabled" : "disabled");
	mutex_unlock(&priv->lock);

	return len;
}

static ssize_t smb347_usb_input_current_override_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value;
	int ret = kstrtouint(buf, 0, &value);

	if (ret) {
		dev_err(priv->dev, "Unable to parse input!\n");
		return -1;
	}

	mutex_lock(&priv->lock);

	if (value > 0) {
		ret = smb347_switch_mode(priv, SMB347_USB_MODE_HC);
		if (ret)
			goto err_out;
		allow_unsafe_pc_usb_in_current = 1;
	} else {
		if (SMB347_APSD_RESULT_SDP == SMB347_APSD_RESULT(
				priv->reg_cache[SMB347_STATUS_REG_D])) {

			ret = smb347_switch_mode(priv, SMB347_USB_MODE_5);
			if (ret)
				goto err_out;
		}
		allow_unsafe_pc_usb_in_current = 0;
	}

err_out:
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev, "%s: Unable to apply settings!!! %d",
			__func__, ret);

	return len;
}

static DEVICE_ATTR(usb_input_override, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_usb_input_current_override_show,
			smb347_usb_input_current_override_store);

static ssize_t smb347_charge_current_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, curr = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_B, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	curr = smb347_charging_current(value);

	if (curr != -1) {
		len += sprintf(buf + len, "%d\n", curr);
	} else {
		len += sprintf(buf + len, "-1\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static int smb347_force_precharge(struct smb347_priv *priv, int flag)
{
	int ret = -1;
	unsigned char value = 0xff;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client, SMB347_COMMAND_REG_A, &value);
	if (ret)
		goto err_out;

	if (flag)
		value &= ~SMB347_CMDA_FC_ENABLE;
	else
		value |= SMB347_CMDA_FC_ENABLE;

	ret = smb347_i2c_write(priv->i2c_client,
				SMB347_COMMAND_REG_A, value);

err_out:
	mutex_unlock(&priv->lock);

	if (ret)
		dev_err(priv->dev, "Error forcing precharge!\n");

	return ret;
}

static int smb347_modify_charge_current(struct smb347_priv *priv,
			int fast_charge_current, int precharge_current)
{
	int ret = -1;
	unsigned char value = 0xff;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client, SMB347_CHARGE_CURRENT, &value);
	if (ret)
		goto err_out;

	if (fast_charge_current != -1) {
		value &= ~CHARGE_CURR_MASK;
		value |= (fast_charge_current << 5);
	}

	if (precharge_current != -1) {
		value &= ~PRECHARGE_CURR_MASK;
		value |= (precharge_current << 3);
	}

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM_FTM
	/* FTM would set maximum charge current for VBAT_IN. */
	/* value = 0xFF; */
	/* ?????? WHY ??????*/
	/* value = CHARGE_CURR_1800 | PRECHARGE_CURR_250 | TERM_CURR_600; */
#endif

	ret = smb347_i2c_write(priv->i2c_client,
			SMB347_CHARGE_CURRENT, value);

err_out:
	mutex_unlock(&priv->lock);
	return ret;
}

static ssize_t smb347_charge_current_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int i = 0, value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

#ifndef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM_FTM
	if (value < 0) {
		dev_err(priv->dev, "Invalid charge current %d mA\n", value);
		goto done;
	}
#endif

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM_FTM
	smb347_modify_input_current(priv, value);
#endif

	if (value >= 1800 && priv->max_thermal_charge_current >= 1800) {
		/* Adjust to 1800 mA */
		value = 1800;
		priv->thermal_affect = 0;
	} else if (value > priv->max_thermal_charge_current) {
		/* Adjust to the new limit current */
		value = priv->max_thermal_charge_current;
		priv->thermal_affect = 1;
	} else priv->thermal_affect = 0;

	/* Locate the first smaller current in fast charge current */
	for (i = ARRAY_SIZE(smb347_fast_charge_currents) - 1; i >= 0; i--)
		if (smb347_fast_charge_currents[i] <= value)
			break;

	if (i >= 0) {
		/* Disable force precharge, set fast charge current */
		if (smb347_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Enable writes to CONFIG regs\n");

		if (smb347_force_precharge(priv, 0)) {
			dev_warn(priv->dev,
				"Unable to disable force pre-charge\n");
		} else if (smb347_modify_charge_current(priv, i, -1)) {
			dev_warn(priv->dev,
				"Unable to modify fast charge current\n");
		}

		if (smb347_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Disabled writes to CONFIG regs\n");

		priv->charge_current = smb347_fast_charge_currents[i];
		goto done;
	}

	/* Locate the first smaller current in precharge current */
	for (i = ARRAY_SIZE(smb347_precharge_currents) - 1; i >= 0; i--)
		if (smb347_precharge_currents[i] <= value)
			break;

	if (i >= 0) {
		/* Force precharge, set pre-charge current */
		if (smb347_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Enable writes to CONFIG regs\n");

		if (smb347_force_precharge(priv, 1)) {
			dev_warn(priv->dev,
				"Unable to force pre-charge\n");
		} else if (smb347_modify_charge_current(priv, -1, i)) {
			dev_warn(priv->dev,
				"Unable to modify pre-charge current\n");
		}

		if (smb347_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Disabled writes to CONFIG regs\n");

		priv->charge_current = smb347_precharge_currents[i];
		goto done;
	}

	dev_warn(priv->dev,
		"Unable to find a valid charge current setting for %d mA\n",
		value);

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_current, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_charge_current_show,
			smb347_charge_current_store);

static ssize_t smb347_charge_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_C, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & SMB347_STC_CHARGING_ENABLED)
		len += sprintf(buf + len, "1\n");
	else
		len += sprintf(buf + len, "0\n");

done:
	mutex_unlock(&priv->lock);
	return len;
}

#define SIG_STOP_CHARGE 32154
#define SIG_START_CHARGE 17698
static int runin_stop_charge_check(struct smb347_priv *priv, int sign)
{
	static int stop_flag = 0;

	if (sign == SIG_STOP_CHARGE) {
		stop_flag = 1;
	} else if (sign == SIG_START_CHARGE) {
		stop_flag = 0;
	}

	return stop_flag;
}

static ssize_t smb347_charge_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM_FTM
	smb347_force_charging_setting(priv, 1);
#endif

	if (runin_stop_charge_check(priv, value)) {
		value = 0;
	}

	if (value) {
		smb347_enable_charging(priv, 1);
	} else {
		smb347_enable_charging(priv, 0);
	}

	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_enable, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_charge_enable_show,
			smb347_charge_enable_store);

static ssize_t smb347_suspend_mode_show(struct device *dev,
                       struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	error = smb347_i2c_read(priv->i2c_client,
				SMB347_COMMAND_REG_A, &value);
	if (error) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & SMB347_CMDA_USBIN_SUSPEND)
		len += sprintf(buf + len, "1\n");
	else
		len += sprintf(buf + len, "0\n");

done:
	mutex_unlock(&priv->lock);
	return len;
}

/* caller must hold priv->lock */
static int smb347_suspend_mode(struct smb347_priv *priv, int enable)
{
	int ret = 0;
	unsigned char value = 0xff;

	WARN_ON(!mutex_is_locked(&priv->lock));
	ret = smb347_i2c_read(priv->i2c_client,
				SMB347_COMMAND_REG_A, &value);
	if (ret)
		goto err_out;

	if (enable)
		value |= SMB347_CMDA_USBIN_SUSPEND;
	else
		value &= ~(SMB347_CMDA_USBIN_SUSPEND);


	ret = smb347_i2c_write(priv->i2c_client,
			SMB347_COMMAND_REG_A, value);

err_out:
	return ret;
}

static ssize_t smb347_suspend_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int enable;
	int ret = kstrtouint(buf, 0, &enable);

	if (ret) {
		dev_err(priv->dev, "Unable to parse input!\n");
		return -1;
	}

	mutex_lock(&priv->lock);

	if (smb347_suspend_mode(priv, enable))
		dev_err(priv->dev,
			"%s: Error appling suspend mode: %d\n", __func__, ret);

	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(suspend_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		smb347_suspend_mode_show,
		smb347_suspend_mode_store);

static ssize_t smb347_bad_battery_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;

        mutex_lock(&priv->lock);
	len += sprintf(buf + len, "%d\n", priv->bad_battery);
        mutex_unlock(&priv->lock);
        return len;
}

static ssize_t smb347_bad_battery_store(struct device *dev,
                       struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int status = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);
	if (status)
		priv->bad_battery = 1;
	else
		priv->bad_battery = 0;
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(bad_battery, S_IRUGO | S_IWUSR | S_IWGRP,
                       smb347_bad_battery_show,
                       smb347_bad_battery_store);

#ifdef CONFIG_CHARGER_SMB347_DEBUG_SYSFS
static u8 g_reg = 0;
static ssize_t smb347_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	u8 data = 0xff;

	mutex_lock(&priv->lock);

	if (smb347_i2c_read(priv->i2c_client, g_reg, &data) < 0) {
		len += sprintf(buf + len, "REG 0x%02X=-1\n", g_reg);
		goto done;
	}

	len += sprintf(buf + len, "REG 0x%02X=0x%02X\n", g_reg, data);

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb347_reg_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int reg = 0, value = 0;
	char cmd = 0;

	mutex_lock(&priv->lock);

	if (sscanf(buf, "%c %x %x", &cmd, &reg, &value) <= 0) {
		dev_err(priv->dev, "Get user-space data failed\n");
		goto done;
	}

	if (cmd == 'r' || cmd == 'R') { //read
		g_reg = (u8)reg;
	} else if (cmd == 'w' || cmd == 'W') { //write
		if (smb347_config(priv, 1)) {
			dev_err(priv->dev,
				"Enable writes to CONFIG regs failed\n");
			goto done;
		}

		if (smb347_i2c_write(priv->i2c_client,
					(u8)reg, (u8)value) < 0) {
			dev_err(priv->dev,
				"Write reg(0x%02X) failed\n", (u8)reg);
		}

		if (smb347_config(priv, 0)) {
			dev_err(priv->dev,
				"Disable writes to CONFIG regs failed\n");
			goto done;
		}
	} else { //default
		dev_info(priv->dev, "Unknow command (%c)\n", cmd);
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_reg_show,
			smb347_reg_store);
#endif /* CONFIG_CHARGER_SMB347_DEBUG_SYSFS */

static ssize_t smb347_precharge_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_INTSTAT_REG_D, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & SMB347_INTSTD_PRECHG_COMPLETE_TO_STA) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(precharge_timeout, S_IRUGO,
			smb347_precharge_timeout_show, NULL);

static ssize_t smb347_complete_charge_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_INTSTAT_REG_D, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto err_out;
	}

	if (value & SMB347_INTSTD_CHG_COMPLETE_TO_STA)
		len += sprintf(buf + len, "1\n");
	else
		len += sprintf(buf + len, "0\n");

err_out:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(complete_charge_timeout, S_IRUGO,
			smb347_complete_charge_timeout_show, NULL);

static ssize_t smb347_charge_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 reg_c_val = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
			SMB347_STATUS_REG_C, &reg_c_val))) {
		len += sprintf(buf + len, "error\n");
		goto done;
	}

	switch (SMB347_STC_CHARGING_STATUS(reg_c_val)) {
	case SMB347_CHARGING_STATUS_NOT_CHARGING:
		len += sprintf(buf + len, "not-charging\n");
		break;
	case SMB347_CHARGING_STATUS_PRE_CHARGING:
		len += sprintf(buf + len, "pre-charging\n");
		break;
	case SMB347_CHARGING_STATUS_FAST_CHARGING:
		len += sprintf(buf + len, "fast-charging\n");
		break;
	case SMB347_CHARGING_STATUS_TAPER_CHARGING:
		len += sprintf(buf + len, "taper-charging\n");
		break;
	default:
		len += sprintf(buf + len, "unknown\n");
		break;
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(charge_status, S_IRUGO,
			smb347_charge_status_show, NULL);

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM_FTM
static ssize_t smb347_thermal_disable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int disable = 0, ret = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_THERMAL_CONTROL, &value);
	if (ret < 0) {
		disable = -1;
		goto done;
	}

	if (value & (1 << 4))
		disable = 1;

done:
	len += sprintf(buf + len, "%d\n", disable);
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb347_thermal_disable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtol(buf, NULL, 10);
	int ret = 0;
	u8 data = 0;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_THERMAL_CONTROL, &data);
	if (ret < 0) {
		dev_err(priv->dev, "Read Thermal_control failed (%d)\n", ret);
		len = -1;
		goto done;
	}

	if (value > 0) { //disable thermal
		data |= (1 << 4);
	} else { //enable
		data &= ~(1 << 4);
	}

	if (smb347_config(priv, 1)) {
		dev_err(priv->dev, "Enable writes to CONFIG regs failed\n");
		len = -1;
		goto done;
	}

	ret = smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_THERMAL_CONTROL, (u8)data);
	if (ret < 0) {
		dev_err(priv->dev, "Write Thermal_control failed (%d)\n", ret);
		len = -1;
	}

	if (smb347_config(priv, 0)) {
		dev_err(priv->dev, "Disable writes to CONFIG regs failed\n");
		len = -1;
	}
done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(thermal_disable, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_thermal_disable_show,
			smb347_thermal_disable_store);
#endif

static struct attribute *smb347_attrs[] = {
	&dev_attr_status_a.attr,
	&dev_attr_status_b.attr,
	&dev_attr_status_c.attr,
	&dev_attr_status_d.attr,
	&dev_attr_status_e.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_charge_enable.attr,
        &dev_attr_suspend_mode.attr,
        &dev_attr_bad_battery.attr,
	&dev_attr_charge_input.attr,
	&dev_attr_charge_input_uamps.attr,
	&dev_attr_usb_input_override.attr,
#ifdef CONFIG_CHARGER_SMB347_DEBUG_SYSFS
	&dev_attr_reg.attr,
#endif
	&dev_attr_complete_charge_timeout.attr,
	&dev_attr_precharge_timeout.attr,
	&dev_attr_charge_status.attr,
#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM_FTM
	&dev_attr_thermal_disable.attr,
#endif
	NULL,
};

static struct attribute_group smb347_attrs_group = {
	.attrs = smb347_attrs,
};

#define THERMAL_PREFIX "Thermal Policy: Charger cooling agent: "
#define THERMAL_INFO(fmt, args...) do { printk(KERN_INFO THERMAL_PREFIX fmt, ## args); } while(0)

#ifdef THERMAL_DEBUG
#define THERMAL_DBG(fmt, args...) do { printk(KERN_DEBUG THERMAL_PREFIX fmt, ## args); } while(0)
#else
#define THERMAL_DBG(fmt, args...) do {} while(0)
#endif

/* smb347_apply_in_ch_sett MUST be called under priv->lock obtained !!! */
static int smb347_apply_in_ch_on_sett(struct smb347_priv *priv,
					u8 in_val, u8 ch_val, int suspend)
{
	u8 temp;
	int ret = 0;

	WARN_ON(!mutex_is_locked(&priv->lock));
	if (smb347_suspended(priv) != suspend)
		smb347_suspend_mode(priv, suspend);

	smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_INPUT_CURR_LIMIT,
			 in_val);
	smb347_redo_aicl(priv);

	/* Set the charging current to 1800 mA. */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_CHARGE_CURRENT, &temp);
	if (ret) {
		pr_err("%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",
		       __func__, ret);
		goto err_out;
	}
	temp &= ~CHARGE_CURR_MASK;
	temp |= ch_val;
	ret = smb347_i2c_write(priv->i2c_client, SMB347_CHARGE_CURRENT, temp);
	if (ret) {
		pr_err("%s: Unable to write SMB347_CHARGE_CURRENT: %d\n",
		       __func__, ret);
		goto err_out;
	}

err_out:
	return ret;
}

static int smb347_apply_cooling(struct thermal_dev *dev,
				int level)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev->dev));
	int current_limit, previous_max_charge_current;
	int ret = 0;
	static int previous_cooling_level = 0, new_cooling_level = 0;
#ifdef CONFIG_AMAZON_METRICS_LOG
	char *thermal_metric_prefix = "charger_cooling:def:monitor=1;CT;1";
	char buf[THERMO_METRICS_STR_LEN];
#endif
	/* transform into current limitation */
	current_limit = thermal_cooling_device_reduction_get(dev, level);

	if (current_limit < 0 || current_limit > 1800)
		return -EINVAL;

	THERMAL_DBG("%s : %d : %d\n",__func__,__LINE__,current_limit);

	/* for logging ...*/
	previous_max_charge_current = priv->max_thermal_charge_current;

	new_cooling_level = level;

	if (current_limit != previous_max_charge_current) {
		mutex_lock(&priv->lock);
		if (smb347_config(priv, 1))
			goto err_out;


		switch (current_limit){
			case 1800:
				ret = smb347_apply_in_ch_on_sett(priv,
					DCIN_LIMIT_1800 | USBIN_LIMIT_1800,
					CHARGE_CURR_1800, 0);
				priv->max_thermal_charge_current = 1800;
				break;

			case 1500:
				ret = smb347_apply_in_ch_on_sett(priv,
					DCIN_LIMIT_1800 | USBIN_LIMIT_1500,
					CHARGE_CURR_1500, 0);
				priv->max_thermal_charge_current = 1500;
				break;

			case 1200:
				ret = smb347_apply_in_ch_on_sett(priv,
					DCIN_LIMIT_1800 | USBIN_LIMIT_1200,
					CHARGE_CURR_1200, 0);
				priv->max_thermal_charge_current = 1200;
				break;

			case 900:
				ret = smb347_apply_in_ch_on_sett(priv,
					DCIN_LIMIT_1800 | USBIN_LIMIT_900,
					CHARGE_CURR_900, 0);
				priv->max_thermal_charge_current = 900;
				break;

			case 0:
				ret = smb347_apply_in_ch_on_sett(priv,
					DCIN_LIMIT_1800 | USBIN_LIMIT_1800,
					CHARGE_CURR_0, 1);
				priv->max_thermal_charge_current = 1800;
				break;
		}
		smb347_config(priv,0);
err_out:
		mutex_unlock(&priv->lock);

		THERMAL_INFO("max charge current transision from %d to %d %s",
				previous_max_charge_current,
				priv->max_thermal_charge_current,
				(ret == 0) ? "Success." : "FAILED!");
	}

#ifdef CONFIG_AMAZON_METRICS_LOG
		if ( previous_cooling_level == 0 ) {
			snprintf(buf, THERMO_METRICS_STR_LEN,
				"%s,throttling_start=1;CT;1:NR",
				thermal_metric_prefix);
			log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
		} else if ( new_cooling_level == 0 ) {
			snprintf(buf, THERMO_METRICS_STR_LEN,
				"%s,throttling_stop=1;CT;1:NR",
				thermal_metric_prefix);
			log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
		}
#endif
	previous_cooling_level = new_cooling_level;

	return ret;
}

static struct thermal_dev_ops smb347_cooling_ops = {
	.cool_device = smb347_apply_cooling,
};

static struct thermal_dev case_thermal_dev = {
	.name		= "charger_cooling",
	.domain_name	= "case",
	.dev_ops	= &smb347_cooling_ops,
};

static int summit_smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct smb347_priv *priv = NULL;
	struct smb347_platform_data *pdata = NULL;
	struct regulator *regulator = NULL;
	int ret = 0, chip_id = 0;
	struct thermal_dev *tdev = NULL;
#ifdef CONFIG_CHARGER_SMB347_DEBUG
	u8 value = 0;
	int i = 0;
#endif

	/*
	 * FIXME: Enable regulators for GPIO0 here,
	 *        which we really shouldn't
	 */
	pdata = dev_get_platdata(&client->dev);

	tdev = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (tdev == NULL){
		ret = -ENOMEM;
		goto err5;
	}

	if (pdata && pdata->regulator_name) {
		regulator = regulator_get(&client->dev, pdata->regulator_name);

		if (IS_ERR(regulator)) {
			pr_err("Unable to get regulator for gpio0\n");
			ret = -EINVAL;
			goto err6;
		}

		if (regulator_set_voltage(regulator, 1800000, 1800000)) {
			pr_err("Unable to set regulator to 1.8V\n");
			ret = -EINVAL;
			goto err6;
		}

		regulator_enable(regulator);
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("Out of memory\n");
		ret = -ENOMEM;
		goto err5;
	}

	priv->regulator = regulator;

	/* Set up I2C structs */
	priv->i2c_client = client;
	i2c_set_clientdata(client, priv);

	/* Set up mutex */
	mutex_init(&priv->lock);

	/* Set up dev pointer */
	priv->dev = &client->dev;

	/* Check for device ID */
	if (summit_smb347_read_id(priv, &chip_id) < 0) {
		client->addr = SUMMIT_SMB347_I2C_ADDRESS_SECONDARY;
		if (summit_smb347_read_id(priv, &chip_id) < 0) {
			pr_err("Unable to detect device ID\n");
			ret = -ENODEV;
			goto err5;
		}
	}

	dev_info(priv->dev, "SMB347 detected, addr=0x%02x chip_id=0x%0x\n",
		 client->addr, chip_id);

	priv->usb_notifier.notifier_call = smb347_usb_notifier_cb;

	ret = twl6030_usb_register_notifier(&priv->usb_notifier);
	if (ret) {
		dev_err(priv->dev, "otg_register_notifier failed: %d\n", ret);
		goto err4;
	}

	/* Set up and register the power_supply structs */
	priv->ac.name = "smb347_ac";
	priv->ac.type = POWER_SUPPLY_TYPE_MAINS;
	priv->ac.get_property = smb347_get_ac_property;
	priv->ac.properties = smb347_charger_props;
	priv->ac.num_properties = ARRAY_SIZE(smb347_charger_props);

	priv->usb.name = "smb347_usb";
	priv->usb.type = POWER_SUPPLY_TYPE_USB;
	priv->usb.get_property = smb347_get_usb_property;
	priv->usb.properties = smb347_usb_props;
	priv->usb.num_properties = ARRAY_SIZE(smb347_usb_props);
	priv->charge_current = 0;
	priv->thermal_affect = 0;

	ret = power_supply_register(&client->dev, &priv->ac);
	if (ret) {
		dev_err(priv->dev,
			"failed to register ac power supply: %d\n", ret);
		goto err3;
	}

	ret = power_supply_register(&client->dev, &priv->usb);
	if (ret) {
		dev_err(priv->dev,
			"failed to register usb power supply: %d\n", ret);
		goto err2;
	}

	/* Register the sysfs nodes */
	ret = sysfs_create_group(&priv->dev->kobj, &smb347_attrs_group);
	if (ret) {
		dev_err(priv->dev, "Unable to create sysfs group\n");
		goto err2;
	}

	memcpy(tdev, &case_thermal_dev, sizeof(struct thermal_dev));
	tdev->dev = priv->dev;
	ret = thermal_cooling_dev_register(tdev);

	if (ret < 0)
		goto err1;

	priv->tdev = tdev;

	wake_lock_init(&priv->smb_wl, WAKE_LOCK_SUSPEND, "smb347_wakelock");

	/* Issue Power on Reset before initialization */
	dev_info(priv->dev, "Issue device reset...\n");
	smb347_i2c_write(priv->i2c_client,
			 SMB347_COMMAND_REG_B, SMB347_CMDB_RESET_SMB);

	/* 20ms are required by SMB347 to complete POR */
	msleep(20);

	dev_info(priv->dev, "Reset wait complete.\n");

#ifdef CONFIG_CHARGER_SMB347_DEBUG
	/* Dump config, command and status registers */
	for (i = 0; i <= 0xe; i++) {
		ret = smb347_i2c_read(priv->i2c_client, i, &value);
		dev_info(priv->dev, "cfg_reg=0x%x, value=0x%x\n", i, value);
	}

	for (i = 0x30; i <= 0x33; i++) {
		ret = smb347_i2c_read(priv->i2c_client, i, &value);
		dev_info(priv->dev, "cmd_reg=0x%x, value=0x%x\n", i, value);
	}

	for (i = 0x3b; i <= 0x3f; i++) {
		ret = smb347_i2c_read(priv->i2c_client, i, &value);
		dev_info(priv->dev, "stats_reg=%d, value=0x%x\n", i, value);
	}
#endif

	/* disable USIBIN */
	mutex_lock(&priv->lock);
	dev_info(priv->dev, "Disable USB_Input.\n");
	smb347_suspend_mode(priv, 1);
	mutex_unlock(&priv->lock);

	/* Limit the fast charge current to 1800 mA */
	smb347_fast_charge_current_limit(priv);

	/* for thermal logging;
	 * Make sure max_thermal_charge_current has the same value
	 * set by above smb347_fast_charge_current_limit
	 */
	priv->max_thermal_charge_current = 1800;

	/* Enable charge enable control via I2C (active low) */
	smb347_set_charge_control(priv);

	/* Limit termination current to 150 mA */
	smb347_set_termination_current(priv);

	/* Enable Charge-Timeout interrupt */
	smb347_set_charge_timeout_interrupt(priv);
#if defined(CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM)
	/* Limit the fast charge current to 1800mA,
	   pre-charge current to 250mA and
	   Terminal current to 250mA by HW requirements */
	smb347_set_current_setting(priv);

	/* Set Charge Curretn Comesation to 900mA */
	smb347_set_current_compensation(priv);

	/* Limit Hard/Soft temperature to 60/0(Hard), 45/15(Soft) and
	   enable Temerature interrupt */
	smb347_set_temperature_threshold(priv);
	smb347_set_temperature_interrupt(priv);
#endif
	/* Init charger_on flag */
	atomic_set(&priv->charger_on, 0);

	mutex_lock(&priv->lock);
	dev_info(priv->dev, "Setup complete.\n");
	/* performing full reg_cache update will clear the interr status
	 * registes. After the IRQ is registered, resuming USBIN will cause the
	 * same events as cable plugin and the VBUS will be redetected */
	priv->reg_cache_dirty = 1;
	summit_smb347_update_reg_cache(priv, 1);

	if (smb347_status_usbin_in_use(priv)) {
		atomic_set(&priv->charger_on, 1);
		/* redo apsd will clear apsd done flag, causing new detection
		 * cycle after USBIN is enabled */
		dev_info(priv->dev, "%s: Force supply detection.\n", __func__);
		smb347_redo_apsd(priv);
	}
	mutex_unlock(&priv->lock);

	/* Configure IRQ as GPIO input */
	gpio_request(GPIO_SMB347_IRQ, "smb347-irq");
	gpio_direction_input(GPIO_SMB347_IRQ);

	/* Enable interrupt */
	if (priv->i2c_client->irq != -1) {
		ret = request_threaded_irq(priv->i2c_client->irq,
			NULL, summit_smb347_irq, IRQF_TRIGGER_RISING,
			"smb347_irq", priv);
		if (ret) {
			dev_err(priv->dev, "Unable to set up threaded IRQ\n");
			goto err1;
		}
	}

	/* Finally enable USBIN */
	mutex_lock(&priv->lock);
	smb347_suspend_mode(priv, 0);
	dev_info(priv->dev, "%s: USB Input power source enabled.\n", __func__);
	mutex_unlock(&priv->lock);

	return 0;

err1:
	thermal_cooling_dev_unregister(tdev);
	kfree(tdev);
err2:
	power_supply_unregister(&priv->usb);
err3:
	power_supply_unregister(&priv->ac);
err4:
	twl6030_usb_unregister_notifier(&priv->usb_notifier);
err5:
	i2c_set_clientdata(client, NULL);
err6:
	if (priv && priv->regulator)
		regulator_put(priv->regulator);

	kfree(priv);

	return ret;
}

static int summit_smb347_remove(struct i2c_client *client)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);

	twl6030_usb_unregister_notifier(&priv->usb_notifier);

	/* Free IRQ */
	free_irq(priv->i2c_client->irq, priv);

	sysfs_remove_group(&priv->dev->kobj, &smb347_attrs_group);

	power_supply_unregister(&priv->usb);
	power_supply_unregister(&priv->ac);

	if (priv && priv->regulator)
		regulator_put(priv->regulator);

	i2c_set_clientdata(client, NULL);

	thermal_cooling_dev_unregister(priv->tdev);
	kfree(priv->tdev);
	wake_lock_destroy(&priv->smb_wl);
	kfree(priv);

	return 0;
}

static void summit_smb347_shutdown(struct i2c_client *client)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);
	int ret = 0;

	dev_info(priv->dev, "Shutting down\n");

	mutex_lock(&priv->lock);
	ret = smb347_config(priv, 1);
	if (ret)
		goto err_out;

	ret = smb347_apply_in_ch_on_sett(priv,
					 DCIN_LIMIT_1800 | USBIN_LIMIT_1800,
					 CHARGE_CURR_1800, 0);
	if (ret)
		goto err_out;

err_out:
	if (ret) {
		dev_err(priv->dev, "%s: Unable to apply settings!!! %d",
			__func__, ret);
	}

	smb347_config(priv, 0);
	mutex_unlock(&priv->lock);

	summit_smb347_remove(client);

}

static int summit_smb347_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);

	dev_info(priv->dev, "Entering suspend, event = 0x%04x\n", mesg.event);

	synchronize_irq(priv->i2c_client->irq);
	dev_info(priv->dev, "Finishing suspend\n");

	return 0;
}

static int summit_smb347_resume(struct i2c_client *client)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);

	dev_info(priv->dev, "Finishing resume\n");

	return 0;
}

static unsigned short normal_i2c[] = { SUMMIT_SMB347_I2C_ADDRESS, I2C_CLIENT_END };

static struct i2c_driver summit_smb347_i2c_driver = {
	.driver = {
			.name = "smb347",
		},
	.probe = summit_smb347_probe,
	.remove = summit_smb347_remove,
	.id_table = summit_smb347_id,
	.address_list = normal_i2c,
	.suspend = summit_smb347_suspend,
	.resume = summit_smb347_resume,
	.shutdown = summit_smb347_shutdown,
};

static int __init summit_smb347_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Summit SMB347 Driver\n");

	ret = i2c_add_driver(&summit_smb347_i2c_driver);
	if (ret) {
		printk(KERN_ERR "summit_smb347: Could not add driver\n");
		return ret;
	}

	printk(KERN_INFO "SMB347 Driver added\n");

	return 0;
}

static void __exit summit_smb347_exit(void)
{
	i2c_del_driver(&summit_smb347_i2c_driver);
}

module_init(summit_smb347_init);
module_exit(summit_smb347_exit);

MODULE_DESCRIPTION("Summit SMB347 Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
