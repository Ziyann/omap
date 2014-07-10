/*
 * drivers/power/bq2419x_charger_.c

 * BQ24195 battery charging driver

 * Copyright (C) 2012 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.

 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/i2c/bq2419x.h>
#include <plat/dmtimer.h>
#include <linux/clk.h>

#define BQ2419x_WDT_TIMEOUT		WatchDog_160s
#define BQ2419x_BOOSTBACK_THRESHOLD_LO	3200
#define BQ2419x_BOOSTBACK_THRESHOLD_HI	3500
#define BQ2419x_LOW_BATTEMP_ICHR_LIMIT	((8*64)+500) /* 1012mA */

/* NOTE: When REGN is on (such as: a normal charging mode), the WDT timer has +/-15% clock variation.
 * When REGN is off (such as:  a HIZ mode), the WDT timer has +/-30% clock variation.
 */
#define BQ2419x_OMAP_DMTIMER_INTERVAL	110

static enum power_supply_property bq2419x_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};

static enum power_supply_property bq2419x_wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};

static u8 global_charge_state;
static struct omap_dm_timer *wdt_timer_ptr;
static bool threshold_voltage_checked = true;
static unsigned long time_previous;
static bool stopdmtimershutdown = false;

extern bool bq27x00_is_battery_present(void);
extern int bq27x00_get_battery_temperature(void);

bool bq2419x_is_charging_done(void);
static int bqEnDPDM(struct bq2419x_device_info *di, int enable);

/*
 * Return reuired battery property or error.
  */
static int bq2419x_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{

	struct bq2419x_device_info *di = container_of(psy,
						      struct bq2419x_device_info,
						      usb);
	int ret = -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (USB_EVENT_VBUS == di->power_type);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = USB_CURRENT_LIMIT_HIGH;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*
 * Return reuired battery property or error.
  */
static int bq2419x_wall_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{

	struct bq2419x_device_info *di = container_of(psy,
						      struct bq2419x_device_info,
						      wall);
	int ret = -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (USB_EVENT_CHARGER == di->power_type);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = AC_CURRENT_LIMIT;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/* HOW-NOT: USB event comes before we register in the notifiers,
* so we do not know that there is SDP/DCP connected on boot, so
* we get it manually on probe. */
int twl6030_usbotg_get_status(void);

/* i2c read/write util functions */
static int bq2419x_write_block(struct bq2419x_device_info *di, u8 *value,
			       u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[1];
	int ret;

	*value		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= value;
	msg[0].len	= num_bytes + 1;

	ret = i2c_transfer(di->client->adapter, msg, 1);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

static int bq2419x_read_block(struct bq2419x_device_info *di, u8 *value,
			      u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[2];
	u8 buf;
	int ret;

	buf		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= &buf;
	msg[0].len	= 1;

	msg[1].addr	= di->client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= value;
	msg[1].len	= num_bytes;

	ret = i2c_transfer(di->client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

static int bq2419x_write_byte(struct bq2419x_device_info *di, u8 value, u8 reg)
{
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };

	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return bq2419x_write_block(di, temp_buffer, reg, 1);
}

static int bq2419x_read_byte(struct bq2419x_device_info *di, u8 *value, u8 reg)
{
	return bq2419x_read_block(di, value, reg, 1);
}
/* end i2c read/write util functions */

bool bq2419x_is_charging_done(void)
{
	return (3 == global_charge_state);
}
EXPORT_SYMBOL(bq2419x_is_charging_done);

/* Start of Functions Related to 19x Functionality */

//////////////////////////////////////////
////------------------------------------------//
////					REG 0					//
////------------------------------------------//
////////////////////////////////////////////////

/*************************************************************
* bqEnHIZ:Disable or enable the high impedance mode
* Accepted Inputs: ENABLE, DISABLE
*
* Returns:
*		-EINVAL(22): Invalid Input
*		  0: I2C Write Success
*		 <0: I2C Write Fail
*
* Can be modified to send Ack bit as the success code
*
**************************************************************/
static int bqEnHIZ(struct bq2419x_device_info *di, int enable)
{
	int success;
	int regbits;
	unsigned int Reg00Val;
	u8 data;
	int rc = -EINVAL;

	if ((enable != ENABLE) && (enable != DISABLE)) {
		rc = -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg00Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			rc = success;
		} else {
			Reg00Val = data;
			regbits = enable << ENHIZ_LSHIFT;
			Reg00Val &= ENHIZ_MASK;
			Reg00Val |= regbits;
			data = Reg00Val;
			success = bq2419x_write_byte(di, data, Reg00Address);
			rc = success;
		}
	}

	/* When write any command to REG00, Bit 0 and Bit1 will be reset to 0.
	 Only valid input current limit settings through I2C are 100mA and 1.2A.
	 Write REG07 bit 7 DPDM_EN=1 after REG00 write in order to recover to default
	 input current limit setting (such as 500mA or 3A). */

	if (BQ24196_REV_1_3 == di->bqchip_version)
		bqEnDPDM(di, ENABLE);

	return rc;
}

#if 0
static int bqSetVINDPM(struct bq2419x_device_info *di, int vdpm)
{
	///*************************************************************
	//* bqSetVINDPM:
	//*
	//* Accepted Inputs: VINDPM_MIN >= vreg <= VINDPM_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by VINDPM_MIN,
	//* VINDPM_MAX variables defined in .h file. If invalid voltage*
	//* is detected regulation voltage will be kept as it is.
	//**************************************************************
	int code = 0;
	int vregbits = 0;
	int success;
	unsigned int Reg00Val;
	u8 data;

	if ((vdpm < VINDPM_MIN) || (vdpm > VINDPM_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg00Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg00Val = data;
		code = ((vdpm - VINDPM_MIN)/VINDPM_STEP);
		vregbits = code << VINDPM_LSHIFT;
		Reg00Val &= VINDPM_MASK;
		Reg00Val |= vregbits;
		data = Reg00Val;
		success = bq2419x_write_byte(di, data, Reg00Address);
		return success;
	}
}
#endif

static int bqSetIINDPM(struct bq2419x_device_info *di, int code)
{

	///*************************************************************
	//* bqSetIINDPM: Changes input current limit, actual current
	//*				is the lesser of the I2C and ILIM settings
	//*
	//* Accepted Inputs: IINLIM_100MA, IINLIM_150MA, IINLIM_500MA
	//* IINLIM_900MA, IINLIM_1200MA, IINLIM_1500MA, IINLIM_2000MA,
	//* IINLIM_3000MA
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg00Val;
	u8 data;

	if ((code != IINLIM_100MA) && (code != IINLIM_150MA) &&
	    (code != IINLIM_500MA) && (code != IINLIM_900MA) &&
	    (code != IINLIM_1200MA) && (code != IINLIM_1500MA) &&
	    (code != IINLIM_2000MA) && (code != IINLIM_3000MA)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg00Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg00Val = data;
		regbits = code << IINDPM_LSHIFT;
		Reg00Val &= IINDPM_MASK;
		Reg00Val |= regbits;
		data = Reg00Val;
		success = bq2419x_write_byte(di, data, Reg00Address);
		return success;
	}
}

////////////////////////////////////////////////
////------------------------------------------//
////					REG 1					//
////------------------------------------------//
////////////////////////////////////////////////
//
static int bqRstREG(struct bq2419x_device_info *di)
{
	///*************************************************************
	//* bqRstREG:Resets Register Settings
	//* Accepted Inputs: None
	//*
	//* Returns:
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg01Val;
	u8 data = 0x00;

	success = bq2419x_read_byte(di, &data, Reg01Address );
	if (success) {
		dev_err(di->dev, "Unable to read byte from charger\n");
		return success;
	}
	Reg01Val = data;
	regbits = RESET << RESETREG_LSHIFT ;
	Reg01Val &= RESETREG_MASK;
	Reg01Val |= regbits;
	data = Reg01Val;
	success = bq2419x_write_byte(di, data, Reg01Address);
	return success;
}


static int bqRstWatchDog(struct bq2419x_device_info *di)
{
	///*************************************************************
	//* bqRstWatchDog:Resets WatchDog Timer
	//* Accepted Inputs: None
	//*
	//* Returns:
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg01Val;
	u8 data;

	success = bq2419x_read_byte(di, &data, Reg01Address );
	if (success) {
		dev_err(di->dev, "Unable to read byte from charger\n");
		return success;
	}
	Reg01Val = data;
	regbits = RESET << RESETWATCHDOG_LSHIFT ;
	Reg01Val &= RESETWATCHDOG_MASK;
	Reg01Val |= regbits;
	data = Reg01Val;
	success = bq2419x_write_byte(di, data, Reg01Address);
	return success;
}


static int bqSetCHGCONFIG(struct bq2419x_device_info *di, int code)
{
	///*************************************************************
	//* bqSetCHGCONFIG: Charger Configuration: Disable, Charge
	//*					Battery, or OTG
	//*
	//* Accepted Inputs: DISABLE, CHARGE_BATTERY, OTG
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg01Val;
	u8 data;

	if ((code != DISABLE) && (code != CHARGE_BATTERY) && (code != OTG)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg01Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg01Val = data;
		regbits = code << CHGCONFIG_LSHIFT;
		Reg01Val &= CHGCONFIG_MASK;
		Reg01Val |= regbits;
		data = Reg01Val;
		success = bq2419x_write_byte(di, data, Reg01Address);
		return success;
	}
}

static int bqSetSYSMIN(struct bq2419x_device_info *di, int vlimit)
{
	///*************************************************************
	//* bqSetSYSMIN:
	//*
	//* Accepted Inputs: SYSMIN_MIN >= vlimit <= SYSMIN_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by SYSMIN_MIN,
	//* SYSMIN_MAX variables defined in .h file. If invalid voltage*
	//* is detected regulation voltage will be kept as it is.
	//**************************************************************/
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg01Val;
	u8 data;

	if ((vlimit < SYSMIN_MIN) || (vlimit > SYSMIN_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg01Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg01Val = data;
		code = ((vlimit - SYSMIN_MIN)/SYSMIN_STEP);
		regbits = code << SYSMIN_LSHIFT;
		Reg01Val &= SYSMIN_MASK;
		Reg01Val |= regbits;
		data = Reg01Val;
		success = bq2419x_write_byte(di, data, Reg01Address);
		dev_dbg(di->dev, "bqSetSYSMIN: %d\n", vlimit);
		return success;
	}
}

#if 0
static int bqSetOTGILIM(struct bq2419x_device_info *di, int code)
{
	int success;
	int regbits;
	unsigned int Reg01Val;
	u8 data;

	if ((code != BOOSTLIM_500mA) && (code != BOOSTLIM_1300mA)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg01Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg01Val = data;
		regbits = code << BOOSTLIM_LSHIFT;
		Reg01Val &= BOOSTLIM_MASK;
		Reg01Val |= regbits;
		data = Reg01Val;
		success = bq2419x_write_byte(di, data, Reg01Address);
		return success;
	}
}
#endif


////////////////////////////////////////////////
////------------------------------------------//
////					REG 2					//
////------------------------------------------//
////////////////////////////////////////////////
static int bqSetFASTCHRG(struct bq2419x_device_info *di, int ichg)
{
	///*************************************************************
	//* bqSetFASTCHRG:
	//*
	//* Accepted Inputs: ICHG_MIN >= ichg <= ICHG_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by ICHG_MIN,
	//* ICHG_MAX variables defined in .h file. If invalid voltage
	//* is detected regulation voltage will be kept as it is.
	//**************************************************************/
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg02Val;
	u8 data;

	if ((ichg < ICHG_MIN) || (ichg > ICHG_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg02Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg02Val = data;
		code = ((ichg - ICHG_MIN)/ICHG_STEP);
		regbits = code << ICHG_LSHIFT;
		Reg02Val &= ICHG_MASK;
		Reg02Val |= regbits;
		data = Reg02Val;
		success = bq2419x_write_byte(di, data, Reg02Address);
		return success;
	}
}


static int bqGetFASTCHRG(struct bq2419x_device_info *di)
{
	int success;
	unsigned int Reg02Val;
	u8 data;

	success = bq2419x_read_byte(di, &data, Reg02Address);
	if (success) {
		dev_err(di->dev, "Unable to read byte from charger\n");
		return success;
	}

	Reg02Val = data & ~ICHG_MASK;
	Reg02Val >>= ICHG_LSHIFT;
	Reg02Val = ((Reg02Val * ICHG_STEP) + ICHG_MIN);

	return Reg02Val;
}


#if 0
////////////////////////////////////////////////
////------------------------------------------//
////					REG 3					//
////------------------------------------------//
////////////////////////////////////////////////
static int bqSetPRECHRG(struct bq2419x_device_info *di, int iprechg)
{
	///*************************************************************
	//* bqSetPRECHRG:
	//*
	//* Accepted Inputs: PRECHG_MIN >= iprechg <= PRECHG_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by PRECHG_MIN,
	//* PRECHG_MAX variables defined in .h file. If invalid voltage
	//* is detected regulation voltage will be kept as it is.
	//**************************************************************/
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg03Val;
	u8 data;

	if ((iprechg < PRECHG_MIN) || (iprechg > PRECHG_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg03Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg03Val = data;
		code = ((iprechg - PRECHG_MIN)/PRECHG_STEP);
		regbits = code << PRECHG_LSHIFT;
		Reg03Val &= PRECHG_MASK;
		Reg03Val |= regbits;
		data = Reg03Val;
		success = bq2419x_write_byte(di, data, Reg03Address);
		return success;
	}
}
#endif

#if 0
static int bqSetTERMCHRG(struct bq2419x_device_info *di, int iterm)
{
	///*************************************************************
	//* bqSetTERMCHRG:
	//*
	//* Accepted Inputs: ITERM_MIN >= iterm <= ITERM_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by ITERM_MIN,
	//* ITERM_MAX variables defined in .h file. If invalid voltage
	//* is detected regulation voltage will be kept as it is.
	//**************************************************************/
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg03Val;
	u8 data;

	if ((iterm < ITERM_MIN) || (iterm > ITERM_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg03Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg03Val = data;
		code = ((iterm - ITERM_MIN)/ITERM_STEP);
		regbits = code << ITERM_LSHIFT;
		Reg03Val &= ITERM_MASK;
		Reg03Val |= regbits;
		data = Reg03Val;
		success = bq2419x_write_byte(di, data, Reg03Address);
		return success;
	}
}
#endif

////////////////////////////////////////////////
////------------------------------------------//
////					REG 4					//
////------------------------------------------//
////////////////////////////////////////////////
#if 0
static int bqSetChgVoltage(struct bq2419x_device_info *di, int vreg)
{
	///*************************************************************
	//* bqSetChgVoltage: Send battery regulation voltage in mV and
	//* the function will calculate the closest value without
	//* going above the desired value. Function will calculate
	//* the I2C code and store it in vregbits. Reg03Val keeps track*
	//* of the overall register value as there are other features
	//* that can be programmed on this register.
	//* Accepted Inputs: VREGMIN >= vreg <= VREG_MAX
	//*
	//* Returns:
	//*		-1: Invalid Regulation Voltage
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by VREG_MAX, VREG_MIN
	//*       variables defined in .h file. If invalid voltage is
	//*		detected regulation voltage will be kept as it is.
	//**************************************************************/
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg04Val;
	u8 data;

	if ((vreg < VREG_MIN) || (vreg > VREG_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg04Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg04Val = data;
		code = ((vreg - VREG_MIN)/VREG_STEP);
		regbits = code << VREG_LSHIFT;
		Reg04Val &= VREG_MASK;
		Reg04Val |= regbits;
		data = Reg04Val;
		success = bq2419x_write_byte(di, data, Reg04Address);
		return success;
	}
}
#endif

#if 0
static int bqSetBATLOWV(struct bq2419x_device_info *di, int setting)
{
	///*************************************************************
	//* bqSetBATLOWV: BATLOWV setting 2.8V or 3.0V
	//* Accepted Inputs: BATLOWV_2800mV, BATLOWV_3000mV
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg04Val;
	u8 data;

	if ((setting != BATLOWV_2800mV) && (setting != BATLOWV_3000mV)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg04Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg04Val = data;
		regbits = setting << BATLOWV_LSHIFT;
		Reg04Val &= BATLOWV_MASK;
		Reg04Val |= regbits;
		data = Reg04Val;
		success = bq2419x_write_byte(di, data, Reg04Address);
		return success;
	}
}
#endif

static int bqSetRECHRG(struct bq2419x_device_info *di, int setting)
{
	///*************************************************************
	//* bqSetRECHRG: Battery Recharge Threshold setting 100mV or
	//*				300mV
	//* Accepted Inputs: VRECHG_100mV, VRECHG_300mV
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg04Val;
	u8 data;

	if ((setting != VRECHG_100mV) && (setting != VRECHG_300mV)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg04Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg04Val = data;
		regbits = setting << VRECHG_LSHIFT;
		Reg04Val &= VRECHG_MASK;
		Reg04Val |= regbits;
		data = Reg04Val;
		success = bq2419x_write_byte(di, data, Reg04Address);
		return success;
	}
}


////////////////////////////////////////////////
////------------------------------------------//
////					REG 5					//
////------------------------------------------//
////////////////////////////////////////////////
#if 0
static int bqEnTERM(struct bq2419x_device_info *di, int enable)
{
	///*************************************************************
	//* bqEnTERM:Disable or enable Charge Termination
	//* Accepted Inputs: ENABLE, DISABLE
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg05Val;
	u8 data;

	if ((enable != ENABLE) && (enable != DISABLE)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg05Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg05Val = data;
		regbits = enable << ENTERM_LSHIFT;
		Reg05Val &= ENTERM_MASK;
		Reg05Val |= regbits;
		data = Reg05Val;
		success = bq2419x_write_byte(di, data, Reg05Address);
		return success;
	}
}
#endif

#if 0
static int bqTERMSTAT(struct bq2419x_device_info *di, int enable)
{
	///*************************************************************
	//* bqTERMSTAT: Matches ITERM or Indicates before actual
	//*				termination on STAT
	//* Accepted Inputs: TERMSTAT_ITERM, TERMSTAT_EARLY
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg05Val;
	u8 data;

	if ((enable != TERMSTAT_ITERM) && (enable != TERMSTAT_EARLY)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg05Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg05Val = data;
		regbits = enable << TERMSTAT_LSHIFT;
		Reg05Val &= TERMSTAT_MASK;
		Reg05Val |= regbits;
		data = Reg05Val;
		success = bq2419x_write_byte(di, data, Reg05Address);
		return success;
	}
}
#endif

static int bqSetWatchDog(struct bq2419x_device_info *di, int code)
{
	///*************************************************************
	//* bqSetWatchDog:
	//*
	//* Accepted Inputs: DISABLE, WatchDog_40s, WatchDog_80s,
	//*					WatchDog_160s
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg05Val;
	u8 data;

	if ((code != DISABLE) && (code != WatchDog_40s) &&
	    (code != WatchDog_80s) && (code != WatchDog_160s)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg05Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg05Val = data;
		regbits = code << WatchDog_LSHIFT;
		Reg05Val = Reg05Val & WatchDog_MASK;
		Reg05Val = regbits | Reg05Val;
		data = Reg05Val;
		success = bq2419x_write_byte(di, data, Reg05Address);
		return success;
	}
}

#if 0
static int bqEnTIMER(struct bq2419x_device_info *di, int enable)
{
	///*************************************************************
	//* bqEnTIMER:Disable or enable Safety Timer Setting
	//* Accepted Inputs: ENABLE, DISABLE
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg05Val;
	u8 data;

	if ((enable != ENABLE) && (enable != DISABLE)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg05Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg05Val = data;
		regbits = enable << ENTIMER_LSHIFT;
		Reg05Val &= ENTIMER_MASK;
		Reg05Val |= regbits;
		data = Reg05Val;
		success = bq2419x_write_byte(di, data, Reg05Address);
		return success;
	}
}
#endif

static int bqSetFastChgTimer(struct bq2419x_device_info *di, int code)
{
	///*************************************************************
	//* bqSetFastChgTimer:
	//*
	//* Accepted Inputs: CHGTIMER_5h, CHGTIMER_8h, CHGTIMER_12h,
	//*					CHGTIMER_20h
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg05Val;
	u8 data;

	if ((code != CHGTIMER_5h) && (code != CHGTIMER_8h) &&
	    (code != CHGTIMER_12h) && (code != CHGTIMER_20h)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg05Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg05Val = data;
		regbits = code << CHGTIMER_LSHIFT;
		Reg05Val &= CHGTIMER_MASK;
		Reg05Val |= regbits;
		data = Reg05Val;
		success = bq2419x_write_byte(di, data, Reg05Address);
		return success;
	}
}

////////////////////////////////////////////////
////------------------------------------------//
////					REG 6					//
////------------------------------------------//
////////////////////////////////////////////////
#if 0
static int bqSetBATCOMP(struct bq2419x_device_info *di, int resistor)
{
	///*************************************************************
	//* bqSetBATCOMP:
	//*
	//* Accepted Inputs: BATCOMP_MIN >= resistor <= BATCOMP_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by BATCOMP_MIN,
	//* BATCOMP_MAX variables defined in .h file. If invalid
	//* voltage is detected regulation voltage will be kept as it
	//* is.
	//**************************************************************/
	//
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg06Val;
	u8 data;

	if ((resistor < BATCOMP_MIN) || (resistor > BATCOMP_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg06Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg06Val = data;
		code = ((resistor - BATCOMP_MIN)/BATCOMP_STEP);
		regbits = code << BATCOMP_LSHIFT;
		Reg06Val &= BATCOMP_MASK;
		Reg06Val |= regbits;
		data = Reg06Val;
		success = bq2419x_write_byte(di, data, Reg06Address);
		return success;
	}
}

static int bqSetVCLAMP(struct bq2419x_device_info *di, int vclamp)
{
	///*************************************************************
	//* bqSetVCLAMP:
	//*
	//* Accepted Inputs: VCLAMP_MIN >= vclamp <= VCLAMP_MAX
	//*
	//* Returns:
	//*		-1: Invalid Setting
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//* NOTE: Accepted values are determine by VCLAMP_MIN,
	//* VCLAMP_MAX variables defined in .h file. If invalid voltage*
	//* is detected regulation voltage will be kept as it is.		 *
	//**************************************************************/
	int code = 0;
	int regbits = 0;
	int success;
	unsigned int Reg06Val;
	u8 data;

	if ((vclamp < VCLAMP_MIN) || (vclamp > VCLAMP_MAX)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg06Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg06Val = data;
		code = ((vclamp - VCLAMP_MIN)/VCLAMP_STEP);
		regbits = code << VCLAMP_LSHIFT;
		Reg06Val &= VCLAMP_MASK;
		Reg06Val |= regbits;
		data = Reg06Val;
		success =  bq2419x_write_byte(di, data, Reg06Address);
		return success;
	}
}

static int bqSetTREG(struct bq2419x_device_info *di, int code)
{
	///*************************************************************
	//* bqSetTREG:
	//*
	//* Accepted Inputs: TREG_60C, TREG_80C, TREG_100C, TREG_120C
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg06Val;
	u8 data;

	if ((code != TREG_60C) && (code != TREG_80C) &&
	    (code != TREG_100C) && (code != TREG_120C)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg06Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg06Val = data;
		regbits = code << TREG_LSHIFT;
		Reg06Val &= TREG_MASK;
		Reg06Val |= regbits;
		data = Reg06Val;
		success =  bq2419x_write_byte(di, data, Reg06Address);
		return success;
	}
}
#endif

//////////////////////////////////////////////
//------------------------------------------//
//					REG 7					//
//------------------------------------------//
//////////////////////////////////////////////
static int bqEnDPDM(struct bq2419x_device_info *di, int enable)
{
	///*************************************************************
	//* bqEnDPDM:Disable or enable D+/D- Detection
	//* Accepted Inputs: ENABLE, DISABLE
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg07Val;
	u8 data;

	if ((enable != ENABLE) && (enable != DISABLE)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg07Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg07Val = data;
		regbits = enable << ENDPDM_LSHIFT;
		Reg07Val &= ENDPDM_MASK;
		Reg07Val |= regbits;
		data = Reg07Val;
		success =  bq2419x_write_byte(di, data, Reg07Address);
		return success;
	}
}

#if 0
static int bqEnTMR2X(struct bq2419x_device_info *di, int enable)
{
	///*************************************************************
	//* bqEnTMR2X:Disable or enable 2x Extened Safety Timer
	//* Accepted Inputs: ENABLE, DISABLE
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg07Val;
	u8 data;

	if ((enable != ENABLE) && (enable != DISABLE)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg07Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg07Val = data;
		regbits = enable << EN2XTIMER_LSHIFT;
		Reg07Val &= EN2XTIMER_MASK;
		Reg07Val |= regbits;
		data = Reg07Val;
		success =  bq2419x_write_byte(di, data, Reg07Address);
		return success;
	}
}

static int bqOffBATFET(struct bq2419x_device_info *di, int enable)
{
	///*************************************************************
	//* bqOffBATFET:Disable or enable 2x Extened Safety Timer
	//* Accepted Inputs: ENABLE, DISABLE
	//*
	//* Returns:
	//*		-1: Invalid Input
	//*		 0: I2C Write Fail
	//*		 1: I2C Write Success
	//*
	//* Can be modified to send Ack bit as the success code
	//*
	//**************************************************************/
	int success;
	int regbits;
	unsigned int Reg07Val;
	u8 data;

	if ((enable != ENABLE) && (enable != DISABLE)) {
		return -EINVAL;
	} else {
		success = bq2419x_read_byte(di, &data, Reg07Address );
		if (success) {
			dev_err(di->dev, "Unable to read byte from charger\n");
			return success;
		}
		Reg07Val = data;
		regbits = enable << OFFBATFET_LSHIFT;
		Reg07Val &= OFFBATFET_MASK;
		Reg07Val |= regbits;
		data = Reg07Val;
		success =  bq2419x_write_byte(di, data, Reg07Address);
		return success;
	}
}
#endif

/* End of Functions Related to 19x Functionality */

extern int bq27x00_read_voltage_mv(void);
static bool bq2419x_is_vbat_in_range(void);

static void check_and_set_Vsysmin(struct bq2419x_device_info *di)
{
	/* Always set WDT before modifying Vsysmin */
	bqSetWatchDog(di, BQ2419x_WDT_TIMEOUT);
	bqRstWatchDog(di);

	if (di->boostback_fix_required) {
		if(bq2419x_is_vbat_in_range())
			bqSetSYSMIN(di, BQ2419x_BOOSTBACK_THRESHOLD_LO);
		else
			bqSetSYSMIN(di, BQ2419x_BOOSTBACK_THRESHOLD_HI);
	}
}

static void adjust_fast_charging_current_limit(struct bq2419x_device_info *di)
{
	const int current_temp = bq27x00_get_battery_temperature();
	const int current_ichr = bqGetFASTCHRG(di);
	int target_ichr_limit = current_ichr;

	if ((current_ichr >= ICHG_DEFAULT) && (current_temp <= 160))
		target_ichr_limit = BQ2419x_LOW_BATTEMP_ICHR_LIMIT;
	else if ((current_ichr < ICHG_DEFAULT) && (current_temp >= 180))
		target_ichr_limit = ICHG_DEFAULT;

	if (target_ichr_limit != current_ichr)
		bqSetFASTCHRG(di, target_ichr_limit);

	dev_dbg(di->dev, "adj_fast_ichg_lim: %d, %d, %d\n",
		current_ichr, current_temp, target_ichr_limit);
}

static void bq2419x_start_dmtimer(struct bq2419x_device_info *di, bool active)
{
	if (stopdmtimershutdown == false ) {
		dev_dbg(di->dev, "bq2419x_start_dmtimer: %d\n", active);

		if (active) {
			omap_dm_timer_stop(wdt_timer_ptr);
			omap_dm_timer_set_prescaler(wdt_timer_ptr, 0);
			omap_dm_timer_set_int_enable(wdt_timer_ptr,
					OMAP_TIMER_INT_MATCH | OMAP_TIMER_INT_OVERFLOW);
			omap_dm_timer_set_load_start(wdt_timer_ptr, 1, di->tick_rate);
			check_and_set_Vsysmin(di);
		} else {
			omap_dm_timer_stop(wdt_timer_ptr);
			omap_dm_timer_disable(wdt_timer_ptr);
			bqSetSYSMIN(di, BQ2419x_BOOSTBACK_THRESHOLD_HI);
		}
	}
}

static void bq2419x_charger_show_status(struct bq2419x_device_info *di)
{
	const u8 read_reg = di->reg08;
	const u8 vbus_state = (read_reg >> 6) & 0x03;
	const u8 charge_state = (read_reg >> 4) & 0x03;
	const u8 dpm_state = read_reg & (1<<3);
	const u8 pg_state = read_reg & (1<<2);
	static u8 pg_state_old = 0;
	const u8 therm_state = read_reg & (1<<1);
	const u8 vsys_state = read_reg & (1<<0);

	if (pg_state) {
		dev_dbg(di->dev,"S: Power Good\n");
		if (!pg_state_old)
			bqEnDPDM(di, ENABLE);
	} else {
		dev_dbg(di->dev,"S: Not Power Good\n");
	}

	global_charge_state = charge_state;
	pg_state_old = pg_state;

	switch (charge_state)
	{
	case 3:
		dev_dbg(di->dev,"S: CHARGE STATE = %d (CHARGE DONE)\n", charge_state);

		/* If we get multiple Charge Done interrupts within 3 seconds interval,
		 * meaning the battery is aged therefore need to set a lower recharge voltage.
		 */
		if (threshold_voltage_checked) {
			const unsigned long time_now = jiffies;
			printk("diff=%lu\n", msecs_to_jiffies(time_now - time_previous));

			if(time_before_eq(time_now, (time_previous + msecs_to_jiffies(3000)))
				&& (time_previous != 0)) {
				bqSetRECHRG(di, VRECHG_300mV);
				threshold_voltage_checked = false;
				time_previous = 0;
				dev_dbg(di->dev, "setting VRECHG to 300mV\n");
			}
			else
				time_previous = time_now;
		}
		break;
	case 2:
		dev_dbg(di->dev,"S: CHARGE STATE = %d (FAST CHARGING)\n",charge_state);
		break;
	case 1:
		dev_dbg(di->dev,"S: CHARGE STATE = %d (PRECHARGE)\n",charge_state);
		break;
	case 0:
		dev_dbg(di->dev,"S: CHARGE STATE = %d (NOT CHARGING)\n",charge_state);
		break;
	default:
		dev_dbg(di->dev,"S: CHARGE STATE = %d\n",charge_state);
		break;
	}
	switch (vbus_state)
	{
	case 3:
		dev_dbg(di->dev,"S: VBUS STATE = %d (OTG)\n",vbus_state);
		break;
	case 2:
		dev_dbg(di->dev,"S: VBUS STATE = %d (Adapter port)\n",vbus_state);
		break;
	case 1:
		dev_dbg(di->dev,"S: VBUS STATE = %d (USB host)\n",vbus_state);
		break;
	case 0:
		dev_dbg(di->dev,"S: VBUS STATE = %d (Unknown)\n",vbus_state);
		break;
	default:
		dev_dbg(di->dev,"S: VBUS STATE = %d\n",vbus_state);
		break;
	}
	if (dpm_state)
		dev_dbg(di->dev,"S: Dynamic Power Management enabled\n");
	if (therm_state)
		dev_dbg(di->dev,"S: Thermal regulation\n");
	if (vsys_state)
		dev_dbg(di->dev,"S: Battery too low\n");
}

static void bq2419x_charger_handle_error(struct bq2419x_device_info *di)
{
	const u8 read_reg = di->reg09;
	const u8 watchdog_fault = read_reg & 0x80;
	const u8 otg_fault = read_reg & 0x40;
	const u8 chrg_fault = (read_reg >> 4) & 0x03;
	const u8 bat_fault = read_reg & 0x08;
	const u8 ntc_fault = read_reg & 0x07;

	switch (chrg_fault)
	{
	case 3:
		dev_dbg(di->dev,"E: Charger fault: Charge (Safe) timer expiration\n");
		break;
	case 2:
		dev_dbg(di->dev,"E: Charger fault: Thermal shutdown\n");
		break;
	case 1:
		dev_dbg(di->dev,"E: Charger fault: Input fault (OVP or bad source)\n");
		break;
	case 0:
		break;
	default:
		dev_dbg(di->dev,"E: Charger fault: %d\n",chrg_fault);
		break;
	}
	switch (ntc_fault)
	{
	case 6:
		dev_dbg(di->dev,"E: Ntc fault: Hot\n");
		break;
	case 5:
		dev_dbg(di->dev,"E: Ntc fault: Cold\n");
		break;
	case 0:
		break;
	default:
		dev_dbg(di->dev,"E: Ntc fault: %d\n",ntc_fault);
		break;
	}

	if (watchdog_fault)
		dev_dbg(di->dev,"E: Watchdog timer expired\n");
	if (otg_fault)
		dev_dbg(di->dev,"E: OTG fault: Cannot start boost function\n");
	if (bat_fault)
		dev_dbg(di->dev,"E: Battery fault: System OVP\n");

}

static void bq2419x_interrupt_work_func(struct work_struct *work)
{
	struct bq2419x_device_info *di = container_of(work,
						      struct bq2419x_device_info,
						      iwork);
	char reg08, reg09;
	int ret;

	ret = bq2419x_read_byte(di, &reg08, Reg08Address );
	if (ret) {
		dev_err(di->dev, "Unable to read reg08 from charger\n");
		return;
	}
	ret = bq2419x_read_byte(di, &reg09, Reg09Address );
	if (ret) {
		dev_err(di->dev, "Unable to read reg09-1 from charger\n");
		return;
	}
	if (di->bqchip_version == BQ24196_REV_1_3) {
		/* This second read is due to errata in PG1.3 */
		ret = bq2419x_read_byte(di, &reg09, Reg09Address );
		if (ret) {
			dev_err(di->dev, "Unable to read reg09-2 from charger\n");
			return;
		}
	}
	di->reg08 = reg08;
	di->reg09 = reg09;
	dev_dbg(di->dev, "INTERRUPT 08=%x, 09=%x\n", di->reg08, di->reg09);

	bq2419x_charger_show_status(di);
	bq2419x_charger_handle_error(di);
}

static irqreturn_t bq24196_interrupt(int irq, void *dev)
{
	struct bq2419x_device_info * const di = dev_get_drvdata(dev);

	schedule_work(&di->iwork);
	return IRQ_HANDLED;
}

static void bq2419x_event_work_func(struct work_struct *work)
{
	struct bq2419x_device_info * const di = container_of(work,
						      struct bq2419x_device_info,
						      ework);

	mutex_lock(&di->lock);
	switch (di->event) {
	case USB_EVENT_VBUS:
		dev_dbg(di->dev, "USB_EVENT_VBUS\n");
		di->power_type = di->event;

		bq2419x_start_dmtimer(di, true);
		adjust_fast_charging_current_limit(di);

		gpio_direction_output(di->gpio_psel, 1);
		gpio_direction_output(di->gpio_ce, 0);

		if (di->bqchip_version == BQ24196_REV_1_3) {
			bqEnDPDM(di, ENABLE);
		} else {
			bqSetIINDPM(di, IINLIM_500MA);
		}

		bqSetFastChgTimer(di, di->stimer_sdp);
		bqSetCHGCONFIG(di, CHARGE_BATTERY);

		power_supply_changed(&di->usb);
		power_supply_changed(&di->wall);

		break;
	case USB_EVENT_CHARGER:
		dev_dbg(di->dev, "USB_EVENT_CHARGER\n");
		di->power_type = di->event;

		bq2419x_start_dmtimer(di, true);
		adjust_fast_charging_current_limit(di);

		gpio_direction_output(di->gpio_psel, 0);
		gpio_direction_output(di->gpio_ce, 0);

		if (di->bqchip_version == BQ24196_REV_1_3) {
			bqEnDPDM(di, ENABLE);
		} else {
			bqSetIINDPM(di, IINLIM_2000MA);
		}
		bqSetFastChgTimer(di, di->stimer_dcp);
		bqSetCHGCONFIG(di, CHARGE_BATTERY);

		power_supply_changed(&di->usb);
		power_supply_changed(&di->wall);

		break;
	case USB_EVENT_NONE:
		dev_dbg(di->dev, "USB_EVENT_NONE\n");
		di->power_type = di->event;

		bqSetCHGCONFIG(di, DISABLE);
		gpio_direction_output(di->gpio_ce, 1);
		gpio_direction_output(di->gpio_psel, 1);

		if (!threshold_voltage_checked) {
			bqSetRECHRG(di, VRECHG_100mV);
			threshold_voltage_checked = true;
			dev_dbg(di->dev, "setting VRECHG to 100mV\n");
		}

		bq2419x_start_dmtimer(di, false);

		bqRstREG(di);

		power_supply_changed(&di->usb);
		power_supply_changed(&di->wall);

		break;
	case USB_EVENT_ID:
		dev_dbg(di->dev, "USB_EVENT_ID");
		bqSetCHGCONFIG(di, OTG);
		di->power_type = di->event;

		bq2419x_start_dmtimer(di, true);

		power_supply_changed(&di->usb);
		power_supply_changed(&di->wall);
		break;
	case USB_EVENT_ENUMERATED:
		/* Nothing to do for this one */
		dev_dbg(di->dev, "USB_EVENT_ENUMERATED\n");
		if (di->bqchip_version == BQ24196_REV_1_3)
			bqEnDPDM(di, ENABLE);
		break;
	case USB_EVENT_NO_CONTACT:
		dev_dbg(di->dev, "USB_EVENT_NO_CONTACT\n");
		/* This is not in the charging state, this event is followed
		by a USB_EVENT_NONE once VBUS is removed */

		if (di->boostback_fix_required)
			bq2419x_start_dmtimer(di, true);

		break;
	default:
		dev_dbg(di->dev, "(%s): unknown cmd\n", __func__);
		break;
	}

	bq2419x_charger_handle_error(di);
	mutex_unlock(&di->lock);
}

static int bq2419x_charger_event(struct notifier_block *nb, unsigned long event,
				 void *_data)
{
	struct bq2419x_device_info * const di =
		container_of(nb, struct bq2419x_device_info, nb);

	di->cfg_params = 1;
	di->event = event;
	schedule_work(&di->ework);
	return 0;
}

static bool bq2419x_is_vbat_in_range(void)
{
	const int v = bq27x00_read_voltage_mv();
	pr_debug("bq2419x_is_vbat_in_range: vbat = %d\n", v);

	return (v > 3400000 ? true : false);
}

#ifdef DEBUG_BQ2419X_REG

static long dgbtest_OTG = 0xdeadbeef;
static ssize_t bq2419x_set_dgbtest_OTG(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	long val;
	int retval;
	retval = strict_strtol(buf, 10, &val);
	if ((retval < 0))
		return -EINVAL;

	dgbtest_OTG = val;
	return count;
}

static ssize_t bq2419x_show_dgbtest_OTG(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%lu\n", dgbtest_OTG);
}

static ssize_t bq2419x_set_regNN(int reg,struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int retval = 0;
	unsigned long val = 0;
	u8 data = 0;
	struct bq2419x_device_info *di = dev_get_drvdata(dev);
	retval = strict_strtol(buf, 16, &val);
	data = (u8)val;
	retval = bq2419x_write_byte(di, data, reg);
	dev_dbg(di->dev, "(set) ret=%d, %d=%s==%x\n",retval, reg, buf, data);
	return count;
}

static ssize_t bq2419x_show_regNN(int reg, struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int retval = 0;
	u8 read_reg = 0;
	struct bq2419x_device_info *di = dev_get_drvdata(dev);
	retval = bq2419x_read_byte(di, &read_reg, reg);
	dev_dbg(di->dev, "(get) ret=%d, %d=%x\n", retval, reg, read_reg);
	return sprintf(buf, "%x\n", read_reg);
}

static ssize_t bq2419x_set_reg00(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(0,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg00(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(0,dev,attr,buf);
}
static ssize_t bq2419x_set_reg01(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(1,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg01(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(1,dev,attr,buf);
}
static ssize_t bq2419x_set_reg02(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(2,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg02(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(2,dev,attr,buf);
}
static ssize_t bq2419x_set_reg03(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(3,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg03(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(3,dev,attr,buf);
}
static ssize_t bq2419x_set_reg04(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(4,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg04(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(4,dev,attr,buf);
}
static ssize_t bq2419x_set_reg05(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(5,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg05(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(5,dev,attr,buf);
}
static ssize_t bq2419x_set_reg06(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(6,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg06(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(6,dev,attr,buf);
}
static ssize_t bq2419x_set_reg07(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(7,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg07(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(7,dev,attr,buf);
}
static ssize_t bq2419x_set_reg08(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(8,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg08(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(8,dev,attr,buf);
}
static ssize_t bq2419x_set_reg09(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(9,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg09(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(9,dev,attr,buf);
}
static ssize_t bq2419x_set_reg10(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return bq2419x_set_regNN(10,dev,attr,buf,count);
}
static ssize_t bq2419x_show_reg10(struct device *dev, struct device_attribute *attr, char *buf)
{
	return bq2419x_show_regNN(10,dev,attr,buf);
}

static DEVICE_ATTR(dgbtest_OTG, S_IWUSR | S_IRUGO,
		   bq2419x_show_dgbtest_OTG,
		   bq2419x_set_dgbtest_OTG);
static DEVICE_ATTR(reg00, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg00,
		   bq2419x_set_reg00);
static DEVICE_ATTR(reg01, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg01,
		   bq2419x_set_reg01);
static DEVICE_ATTR(reg02, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg02,
		   bq2419x_set_reg02);
static DEVICE_ATTR(reg03, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg03,
		   bq2419x_set_reg03);
static DEVICE_ATTR(reg04, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg04,
		   bq2419x_set_reg04);
static DEVICE_ATTR(reg05, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg05,
		   bq2419x_set_reg05);
static DEVICE_ATTR(reg06, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg06,
		   bq2419x_set_reg06);
static DEVICE_ATTR(reg07, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg07,
		   bq2419x_set_reg07);
static DEVICE_ATTR(reg08, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg08,
		   bq2419x_set_reg08);
static DEVICE_ATTR(reg09, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg09,
		   bq2419x_set_reg09);
static DEVICE_ATTR(reg10, S_IWUSR | S_IRUGO,
		   bq2419x_show_reg10,
		   bq2419x_set_reg10);

static struct attribute *bq2419x_attributes[] = {
	&dev_attr_dgbtest_OTG.attr,
	&dev_attr_reg00.attr,
	&dev_attr_reg01.attr,
	&dev_attr_reg02.attr,
	&dev_attr_reg03.attr,
	&dev_attr_reg04.attr,
	&dev_attr_reg05.attr,
	&dev_attr_reg06.attr,
	&dev_attr_reg07.attr,
	&dev_attr_reg08.attr,
	&dev_attr_reg09.attr,
	&dev_attr_reg10.attr,
	NULL,
};

static const struct attribute_group bq2419x_attr_group = {
	.attrs = bq2419x_attributes,
};

#endif /* DEBUG_BQ2419X_REG */

static ssize_t bq2419x_set_charge_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2419x_device_info * const di = dev_get_drvdata(dev);

	if (count > 1) {
		if ('1' == buf[0]) {
			bqSetCHGCONFIG(di, CHARGE_BATTERY);
			dev_dbg(di->dev, "charger enabled\n");
		} else if ('2' == buf[0]) {
			bqSetCHGCONFIG(di, OTG);
			bq2419x_start_dmtimer(di, true);
			dev_dbg(di->dev, "OTG boost mode enabled\n");
		} else {
			bqSetCHGCONFIG(di, DISABLE);
			dev_dbg(di->dev, "charger disabled\n");
		}
	}
	return count;
}

static ssize_t bq2419x_show_charge_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bq2419x_device_info * const di = dev_get_drvdata(dev);
	int retval = 0;
	u8 read_reg = 0;

	retval = bq2419x_read_byte(di, &read_reg, Reg01Address);
	dev_dbg(di->dev, "read reg01: 0x%x\n", read_reg);
	read_reg &= ~CHGCONFIG_MASK;
	read_reg >>= CHGCONFIG_LSHIFT;

	return sprintf(buf, "%x\n", read_reg);
}

static ssize_t bq2419x_set_hiz_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2419x_device_info * const di = dev_get_drvdata(dev);

	if (count > 1) {
		if ('1' == buf[0]) {
			bqEnHIZ(di, 1);
			dev_dbg(di->dev, "HiZ enabled\n");
		} else {
			bqEnHIZ(di, 0);
			dev_dbg(di->dev, "HiZ disabled\n");
		}
	}
	return count;
}

static ssize_t bq2419x_show_hiz_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bq2419x_device_info * const di = dev_get_drvdata(dev);
	int retval = 0;
	u8 read_reg = 0;

	retval = bq2419x_read_byte(di, &read_reg, Reg00Address);
	dev_dbg(di->dev, "read reg00: 0x%x\n", read_reg);
	read_reg &= ~ENHIZ_MASK;
	read_reg >>= ENHIZ_LSHIFT;

	return sprintf(buf, "%x\n", read_reg);
}

static DEVICE_ATTR(charge_status, S_IWUSR | S_IRUSR,
		   bq2419x_show_charge_status,
		   bq2419x_set_charge_status);
static DEVICE_ATTR(hiz_enable, S_IWUSR | S_IRUSR,
		   bq2419x_show_hiz_enable,
		   bq2419x_set_hiz_enable);

static struct attribute *bq2419x_mfg_attributes[] = {
	&dev_attr_charge_status.attr,
	&dev_attr_hiz_enable.attr,
	NULL,
};

static const struct attribute_group bq2419x_mfg_attr_group = {
	.attrs = bq2419x_mfg_attributes,
};

static void bq2419x_wdt_work_func(struct work_struct *work)
{
	struct bq2419x_device_info * const di = container_of(work,
						      struct bq2419x_device_info,
						      wdt_work);
	mutex_lock(&di->lock);
	check_and_set_Vsysmin(di);
	adjust_fast_charging_current_limit(di);
	/* A second WDT reset is needed here to ensure WDT RST CMD went through */
	bqRstWatchDog(di);
	wake_unlock(&di->timer_wakelock);
	mutex_unlock(&di->lock);
}

static irqreturn_t bq2419x_wdt_irq_handler(int irq, void *dev)
{
	struct bq2419x_device_info * const di = dev_get_drvdata(dev);

	omap_dm_timer_write_status(wdt_timer_ptr, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_read_status(wdt_timer_ptr);

	wake_lock_timeout(&di->timer_wakelock, msecs_to_jiffies(10000));
	schedule_work(&di->wdt_work);

	return IRQ_HANDLED;
}

static unsigned int omap_get_clk_load(u32 seconds, u32 milliseconds)
{
	u32 tick_rate, cycles;

	if (!seconds && !milliseconds) {
		pr_debug("omap_get_clk_load err\n");
		return 0;
	}

	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(wdt_timer_ptr));
	cycles = (tick_rate * seconds) + (tick_rate * milliseconds / 1000);

	return 0xffffffff - cycles;
}

static int __devinit bq2419x_charger_probe(struct i2c_client *client,
					   const struct i2c_device_id *id)
{
	struct bq2419x_device_info *di;
	struct bq2419x_platform_data *pdata = NULL;
	int ret;
	u8 read_reg = 0;

	pr_debug("bq2419x probe\n");
	pdata = client->dev.platform_data;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &client->dev;
	di->client = client;
	di->gpio_int = pdata->gpio_int;
	di->gpio_ce = pdata->gpio_ce;
	di->gpio_psel = pdata->gpio_psel;
	di->stimer_sdp = pdata->stimer_sdp;
	di->stimer_dcp = pdata->stimer_dcp;
	di->boostback_fix_required = true;

	i2c_set_clientdata(client, di);

	/* identify the chip */
	ret = bq2419x_read_byte(di, &read_reg, Reg10Address);
	if (ret < 0) {
		dev_err(di->dev, "chip not present at address 0x%x\n",
								client->addr);
		ret = -EINVAL;
		goto err_kfree;
	}

	di->bqchip_version = (read_reg & BQ2419x_PN_REV_BIT_MASK);
	dev_dbg(di->dev, "read_reg=0x%x\n", read_reg);

	switch (di->bqchip_version) {
	case BQ24196_REV_1_3:
		dev_dbg(di->dev, "identified chip BQ24196_REV_1_3\n");
		di->boostback_fix_required = true;
	break;
	case BQ24196_REV_1_4:
		dev_dbg(di->dev, "identified chip BQ24196_REV_1_4\n");
		di->boostback_fix_required = false;
	break;
	case BQ24196_REV_1_2:
		dev_dbg(di->dev, "identified chip BQ24196_REV_1_2\n");
		di->boostback_fix_required = true;
	break;
	default:
		dev_dbg(di->dev, "unsupported bq chip\n");
		dev_dbg(di->dev, "Chip address 0x%x", client->addr);
		dev_dbg(di->dev, "bq chip version reg value 0x%x", read_reg);
		ret = -EINVAL;
		goto err_kfree;
	break;
	}

	if (DevID == client->addr) {
		dev_dbg(di->dev, "found chip at 0x6b\n");
	} else {
		dev_err(di->dev, "not the chip 0x%x\n", client->addr);
		ret = -EINVAL;
		goto err_kfree;
	}

	/* Register interrupt and its work */
	mutex_init(&di->lock);
	wake_lock_init(&di->timer_wakelock, WAKE_LOCK_SUSPEND, "bq2419x_timer_lock");
	INIT_WORK(&di->iwork, bq2419x_interrupt_work_func);
	INIT_WORK(&di->wdt_work, bq2419x_wdt_work_func);

	wdt_timer_ptr = omap_dm_timer_request();
	if (!wdt_timer_ptr) {
		dev_err(di->dev, "no more gp timers available\n");
		ret = -EINVAL;
		goto err_kfree;
	}

	ret = omap_dm_timer_set_source(wdt_timer_ptr, OMAP_TIMER_SRC_SYS_CLK);
	if (ret)
		dev_err(di->dev, "omap_dm_timer_set_source failed: %d\n", ret);

	ret = omap_dm_timer_set_prescaler(wdt_timer_ptr, 0);
	if (ret)
		dev_err(di->dev, "omap_dm_timer_set_source failed: %d\n", ret);

	ret = request_irq(omap_dm_timer_get_irq(wdt_timer_ptr), bq2419x_wdt_irq_handler,
		IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL, "bq2419x timer", di->dev);
	if (ret) {
		dev_err(di->dev, "request_irq timer_irq failed\n");
		ret = di->gpio_psel = -ENODEV;
		goto err_kfree;
	}

	di->tick_rate = omap_get_clk_load((BQ2419x_OMAP_DMTIMER_INTERVAL/2), 0);
	dev_dbg(di->dev, "omap_get_clk_load: 0x%x\n", di->tick_rate);

	ret = omap_dm_timer_set_int_enable(wdt_timer_ptr,
		OMAP_TIMER_INT_MATCH | OMAP_TIMER_INT_OVERFLOW);
	if (ret)
		dev_err(di->dev, "omap_dm_timer_set_int_enable failed: %d\n", ret);

	omap_dm_timer_disable(wdt_timer_ptr);

	if (gpio_is_valid(di->gpio_psel)) {
		if (gpio_request(di->gpio_psel, "gpio_psel")) {
			dev_err(di->dev, "gpio_psel pin not available\n");
			ret = di->gpio_psel = -ENODEV;
			goto err_kfree;
		} else
			gpio_direction_output(di->gpio_psel, 1);
	}

	if (gpio_is_valid(di->gpio_int)) {
		if (gpio_request(di->gpio_int, "gpio_int")) {
			dev_err(di->dev, "gpio_int pin not available\n");
			ret = di->gpio_int = -ENODEV;
			goto err_kfree;
		} else
			gpio_direction_input(di->gpio_int);
	}

	if (gpio_is_valid(di->gpio_ce)) {
		if (gpio_request(di->gpio_ce, "gpio_ce")) {
			dev_err(di->dev, "gpio_ce pin not available\n");
			ret = di->gpio_ce = -ENODEV;
			goto err_kfree;
		} else
			gpio_direction_output(di->gpio_ce, 1);
	}

	ret = request_irq(OMAP_GPIO_IRQ(di->gpio_int),
			  bq24196_interrupt,
			  IRQF_TRIGGER_FALLING,
			  "chg_nint",
			  di->dev);
	if (ret < 0) {
		dev_err(di->dev, "%s: cannot regitser IRQ %d, ret=%d\n", __func__,
							di->gpio_int, ret);
		goto err_kfree;
	}

	di->nb.notifier_call = bq2419x_charger_event;

	di->active = 0;
	di->params.enable = 1;
	di->cfg_params = 1;
	di->enable_iterm = 1;

	di->usb.name = "bq2419x-usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = bq2419x_usb_props;
	di->usb.num_properties = ARRAY_SIZE(bq2419x_usb_props);
	di->usb.get_property = bq2419x_usb_get_property;
	di->usb.external_power_changed = NULL;

	di->wall.name = "bq2419x-wall";
	di->wall.type = POWER_SUPPLY_TYPE_MAINS;
	di->wall.properties = bq2419x_wall_props;
	di->wall.num_properties = ARRAY_SIZE(bq2419x_wall_props);
	di->wall.get_property = bq2419x_wall_get_property;
	di->wall.external_power_changed = NULL;

	ret = power_supply_register(di->dev, &di->usb);
	if (ret) {
		dev_err(di->dev, "failed to register usb: %d\n", ret);
		goto err_pwrsys;
	}

	ret = power_supply_register(di->dev, &di->wall);
	if (ret) {
		dev_err(di->dev, "failed to register wall: %d\n", ret);
		goto err_pwrsys;
	}

#ifdef DEBUG_BQ2419X_REG
	ret = sysfs_create_group(&di->dev->kobj, &bq2419x_attr_group);
	if (ret) {
		dev_dbg(di->dev, "could not create sysfs files\n");
		goto err_sysfs;
	}
#endif

	ret = sysfs_create_group(&di->dev->kobj, &bq2419x_mfg_attr_group);
	if (ret) {
		dev_dbg(di->dev, "could not create mfg sysfs files\n");
		goto err_sysfs;
	}

	INIT_WORK(&di->ework, bq2419x_event_work_func);

	/* only reset charger if battery is present to allow boot without battery */
	if (bq27x00_is_battery_present()) {
		dev_dbg(di->dev, "resting registers\n");
		bqRstREG(di);
	}

	di->otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (di->otg) {
		ret = usb_register_notifier(di->otg, &di->nb);
		if (ret) {
			dev_err(di->dev, "otg register notifier"
						" failed %d\n", ret);
			goto err_otg;
		}
		di->event = twl6030_usbotg_get_status();
		if (di->event)
			schedule_work(&di->ework);
	} else {
		dev_err(di->dev, "otg_get_transceiver failed %d\n", ret);
		goto err_otg;
	}

	return 0;

err_otg:

#ifdef DEBUG_BQ2419X_REG
	sysfs_remove_group(&di->dev->kobj, &bq2419x_attr_group);
#endif /* DEBUG_BQ2419X_REG */
	sysfs_remove_group(&di->dev->kobj, &bq2419x_mfg_attr_group);
	cancel_work_sync(&di->ework);

err_sysfs:
	free_irq(di->gpio_int, NULL);
	cancel_work_sync(&di->iwork);

err_pwrsys:
err_kfree:
	kfree(di);

	return ret;
}

static int __devexit bq2419x_charger_remove(struct i2c_client *client)
{
	struct bq2419x_device_info *di = i2c_get_clientdata(client);

	bqRstREG(di);
	free_irq(OMAP_GPIO_IRQ(di->gpio_int),di->dev);
	cancel_work_sync(&di->ework);
	cancel_work_sync(&di->iwork);

#ifdef DEBUG_BQ2419X_REG
	sysfs_remove_group(&client->dev.kobj, &bq2419x_attr_group);
#endif /* DEBUG_BQ2419X_REG */

	sysfs_remove_group(&client->dev.kobj, &bq2419x_mfg_attr_group);
	flush_scheduled_work();
	usb_unregister_notifier(di->otg, &di->nb);
	kfree(di);

	return 0;
}

static void bq2419x_charger_shutdown(struct i2c_client *client)
{
	struct bq2419x_device_info * const di = i2c_get_clientdata(client);
	free_irq(OMAP_GPIO_IRQ(di->gpio_int),di->dev);
	usb_unregister_notifier(di->otg, &di->nb);
	stopdmtimershutdown = true;
	omap_dm_timer_stop(wdt_timer_ptr);
	omap_dm_timer_disable(wdt_timer_ptr);
	omap_dm_timer_free(wdt_timer_ptr);
	cancel_work_sync(&di->ework);
	cancel_work_sync(&di->iwork);
	cancel_work_sync(&di->wdt_work);
	bqRstREG(di);
	dev_dbg(di->dev, "%s \n", __func__);
}

static int bq2419x_charger_suspend(struct device *dev)
{
	return 0;
}

static int bq2419x_charger_resume(struct device *dev)
{
	struct platform_device * const pdev = to_platform_device(dev);
	struct bq2419x_device_info * const di = platform_get_drvdata(pdev);

	cancel_work_sync(&di->iwork);
	schedule_work(&di->iwork);

	return 0;
}

static const struct i2c_device_id bq2419x_id[] = {
	{ "bq24196", 0 },
	{},
};

static const struct dev_pm_ops pm_ops = {
	.suspend	= bq2419x_charger_suspend,
	.resume		= bq2419x_charger_resume,
};

static struct i2c_driver bq2419x_charger_driver = {
	.probe		= bq2419x_charger_probe,
	.remove		= __devexit_p(bq2419x_charger_remove),
	.shutdown	= bq2419x_charger_shutdown,
	.id_table	= bq2419x_id,
	.driver		= {
		.name	= "bq24196",
		.pm		= &pm_ops,
	},
};

static int __init bq2419x_charger_init(void)
{
	pr_debug("bq2419x driver init\n");
	return i2c_add_driver(&bq2419x_charger_driver);
}
module_init(bq2419x_charger_init);

static void __exit bq2419x_charger_exit(void)
{
	pr_debug("bq2419x driver exit\n");
	i2c_del_driver(&bq2419x_charger_driver);
}
module_exit(bq2419x_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
