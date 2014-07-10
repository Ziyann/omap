/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/usb/otg.h>
#include <linux/power/bq27x00_battery.h>

#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>

#define DRIVER_VERSION			"1.2.0"

#define BQ27x00_REG_CONTROL		(0x00)
#define BQ27x00_CONTROL_STATUS		(0x0000)
#define BQ27x00_CONTROL_STATUS_INITCOMP	BIT(7)
#define BQ27x00_CONTROL_DEVICE_TYPE	(0x0001)
#define BQ27x00_CONTROL_FW_VERSION	(0x0002)
#define BQ27x00_CONTROL_DF_VERSION	(0x001F)
#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_NAC			0x0C	/* Nominal available capaciy */
#define BQ27x00_REG_LMD			0x12	/* Last measured discharge */
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_AE			0x22	/* Available enery */
#define BQ27x00_REG_TTECP		0x26
#define BQ27x00_REG_CYCT		0x2A	/* Cycle count total */

#define BQ27000_REG_RSOC		0x0B	/* Relative State-of-Charge */
#define BQ27000_REG_ILMD		0x76	/* Initial last measured discharge */
#define BQ27000_FLAG_EDVF		BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_EDV1		BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_CI			BIT(4) /* Capacity Inaccurate flag */
#define BQ27000_FLAG_FC			BIT(5)
#define BQ27000_FLAG_CHGS		BIT(7) /* Charge state flag */

#define BQ27500_REG_SOC			0x2C
#define BQ27500_REG_DCAP		0x3C	/* Design capacity */
#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27500_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27500_FLAG_BAT_DET		BIT(3)
#define BQ27500_FLAG_FC			BIT(9)
#define BQ27500_FLAG_XCHG		BIT(10)
#define BQ27500_FLAG_CHG_INH		BIT(11)
#define BQ27500_FLAG_OTD		BIT(14)
#define BQ27500_FLAG_OTC		BIT(15)

#define BQ27000_RS			20	/* Resistor sense */

#define BQ27x00_REG_BUFFER_START	BQ27x00_REG_TEMP
#define BQ27x00_REG_BUFFER_END		BQ27500_REG_SOC
#define BQ27x00_REG_BUFFER_SIZE		(BQ27500_REG_SOC - BQ27x00_REG_TEMP + 1 + sizeof(char))

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read) (struct bq27x00_device_info *di, u8 reg, bool single);
};

enum bq27x00_chip { BQ27000, BQ27500 };

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int flags;
};

struct bq27x00_device_info {
	struct device *dev;
	int id;
	enum bq27x00_chip chip;
	struct bq27x00_reg_cache cache;
	int charge_design_full;
	unsigned long last_update;
	struct delayed_work work;
	struct work_struct	event_work;
	struct power_supply bat;
	struct bq27x00_access_methods bus;
	struct mutex lock;
	struct otg_transceiver	*otg;
	struct notifier_block	nb;
	enum usb_xceiv_events current_usb_event;

	/* Cached property registers - snatched upon device probe */
	int sys_device_type;
	int sys_fw_version;
	int rapid_poll_cycle;
	struct rw_semaphore	bulk_reg_cache_lock;
	uint8_t bulk_reg_cache[BQ27x00_REG_BUFFER_SIZE];

	struct work_struct bat_low_work;
	struct wake_lock bat_low_wakelock;

	int gpio_ce;
	int gpio_soc_int;
	int gpio_bat_low;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
};

/*
 * Because the battery may not report accurate status on the first
 * poll we check every 500ms for the first 5 seconds. 5 seconds
 * was empirically determined to be an ok interval for keeping
 * the charge status accurate-ish
 */
#define T_POLL_PLUG_MS        1000 /* ms */
#define T_POLL_PLUG_MAX        5 /* iterations */

int bq27x00_read_voltage_mv(void);

static unsigned int show_bulk_cache_reg_dump = 0;
module_param(show_bulk_cache_reg_dump, uint, 0644);
MODULE_PARM_DESC(show_bulk_cache_reg_dump, "1 enable i2c cached reg dump");


#ifdef CONFIG_CHARGER_BQ2419x
extern bool bq2419x_is_charging_done(void);
#endif /* CHARGER_BQ2419x */

/* HOW-NOT: USB event comes before we register in the notifiers,
* so we do not know that there is SDP/DCP connected on boot, so
* we get it manually on probe. */
extern int twl6030_usbotg_get_status(void);

static struct bq27x00_device_info *bq27x00_di;

static unsigned int poll_interval = 30000;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in ms - "
		 "0 disables polling");
static bool g_pause_i2c = false;

static int bq27x00_read_control(u16 reg, int *rt_value,
				struct bq27x00_device_info *di);
static int bq27x00_register_cached_read(struct bq27x00_device_info *di, u8 reg,
				       bool single);
static int bq27x00_register_bulk_cache(struct bq27x00_device_info *di);
static void bq27x00_hw_reset(struct bq27x00_device_info *di);
static void bq27x00_event_work_func(struct work_struct *work);
static int usb_xceiv_events_callback(struct notifier_block *nb,
				unsigned long event,
				void *_data);

static int need_to_shutdown = 0;
static int is_chargermode = 0;
static __init int chargermode_setup(char *s)
{
	if(0 == strcmp(s,"charger"))
	{
		printk("boot into charger mode, doesn't handle bat_low irq'\n");
		is_chargermode = 1;
	}
	return 0;
}
__setup("androidboot.mode=", chargermode_setup);

bool bq27x00_is_battery_present(void) {
	return ((bq27x00_di->cache.flags & BQ27500_FLAG_BAT_DET) ? true : false);
}
EXPORT_SYMBOL(bq27x00_is_battery_present);

int bq27x00_get_battery_temperature(void)
{
	return bq27x00_di->cache.temperature;
}
EXPORT_SYMBOL(bq27x00_get_battery_temperature);

static ssize_t pause_i2c_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return snprintf(buf, 2, "%u\n", (g_pause_i2c ? 1 : 0));
}

static ssize_t pause_i2c_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	if (count > 1) {
		if ('0' == buf[0])
			g_pause_i2c = false;
		else
			g_pause_i2c = true;
	}
	return count;
}

static struct device_attribute dev_attr_pause_i2c = {
	.attr = {
		 .name = "pause_i2c",
		 .mode = 0660},
	.show = pause_i2c_show,
	.store = pause_i2c_store
};

static ssize_t hw_reset_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return 0;
}

static ssize_t hw_reset_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	if (count > 1) {
		if ('1' == buf[0])
			bq27x00_hw_reset(bq27x00_di);
	}
	return count;
}

static struct device_attribute dev_attr_hw_reset = {
	.attr = {
		 .name = "hw_reset",
		 .mode = (S_IWUSR | S_IRUSR)},
	.show = hw_reset_show,
	.store = hw_reset_store
};

/*
 * Common code for BQ27x00 devices
 */

static inline int bq27x00_register_uncached_read(struct bq27x00_device_info *di, u8 reg,
					bool single)
{
	return di->bus.read(di, reg, single);
}

static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
			       bool single)
{
	return bq27x00_register_cached_read(di, reg, single);
}

static int bq27x00_register_cached_read(struct bq27x00_device_info *di, u8 reg,
				       bool single)
{
	int res = 0;
	struct i2c_client *const client = to_i2c_client(di->dev);

	if (unlikely
	    ((reg < BQ27x00_REG_BUFFER_START)
	     || (reg > BQ27x00_REG_BUFFER_END))) {
		dev_err(di->dev,
			"error reading cache register out of bound: reg=%d\n",
			reg);
		return -EFAULT;
	}

	if (unlikely(!client->adapter))
		return -ENODEV;

	reg -= BQ27x00_REG_BUFFER_START;

	down_read(&di->bulk_reg_cache_lock);

	if (!single) {
		res =
		    di->bulk_reg_cache[reg] +
		    (di->bulk_reg_cache[reg + 1] << 8);
	} else
		res = di->bulk_reg_cache[reg];

	if (show_bulk_cache_reg_dump) {
		printk("reg = 0x%x, res = %d\n", reg, res);
	}

	up_read(&di->bulk_reg_cache_lock);

	return res;
}

/*
 * Read all registers and save them to a local buffer. This minimizes
 * the overall I2C transfers, and prevents BQ lockup due to excessive
 * I2C communication.
 */
static int bq27x00_register_bulk_cache(struct bq27x00_device_info *di)
{
	struct i2c_client *const client = to_i2c_client(di->dev);
	int stat;
	uint8_t regaddr_start = BQ27x00_REG_BUFFER_START;
	int retry_counter = 10;
	int i;

	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &regaddr_start,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = BQ27x00_REG_BUFFER_SIZE,
		 .buf = di->bulk_reg_cache,
		 }
	};

	/* printk(KERN_INFO "bq27x00_register_bulk_cache\n"); */

	if (g_pause_i2c)
		return -EBUSY;

	down_write(&di->bulk_reg_cache_lock);
	do {
		stat = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

		if (stat < 0)
			dev_err(di->dev, "I2C read error: %d\n", stat);
		else if (stat != ARRAY_SIZE(msgs)) {
			dev_err(di->dev, "I2C read N mismatch: %d\n", stat);
			stat = -EIO;
		} else
			stat = 0;

		if (stat != 0)
			bq27x00_hw_reset(di);

	} while ((stat != 0) && (retry_counter-- > 0));

	if (show_bulk_cache_reg_dump) {
		dev_warn(di->dev,"--- bq27250 cahced reg dump ---\n");
		for (i = 0; i < BQ27x00_REG_BUFFER_SIZE; i++) {
			printk("0x%02x ", di->bulk_reg_cache[i]);
			if (!((i+1) % 20))
				printk("\n");

		}
		dev_warn(di->dev,"--------------------------------\n");
	}

	up_write(&di->bulk_reg_cache_lock);

	return stat;
}

/*
 * Return the battery Relative State-of-Charge
 */
static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc;


	if (di->chip == BQ27500)
		rsoc = bq27x00_read(di, BQ27500_REG_SOC, false);
	else
		rsoc = bq27x00_read(di, BQ27000_REG_RSOC, true);

	if (rsoc < 0) {
		dev_dbg(di->dev, "error reading relative State-of-Charge\n");
		rsoc = 100;
	}

	if ((rsoc == 0) || (rsoc == 0xffff)) {
		/* set capacity to full if no battery */
		if (!(di->cache.flags & BQ27500_FLAG_BAT_DET))
		{
			dev_dbg(di->dev, "NO BATTERY FOUND!\n");
			rsoc = 1;
		}
	}

	rsoc = min(rsoc, 100);
	rsoc = max(rsoc, 0);

	return rsoc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27x00_read(di, reg, false);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (di->chip == BQ27500)
		charge *= 1000;
	else
		charge = charge * 3570 / BQ27000_RS;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_lmd(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_ilmd(struct bq27x00_device_info *di)
{
	int ilmd;

	if (di->chip == BQ27500)
		ilmd = bq27x00_register_uncached_read(di, BQ27500_REG_DCAP, false);
	else
		ilmd = bq27x00_register_uncached_read(di, BQ27000_REG_ILMD, true);

	if (ilmd < 0) {
		dev_dbg(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}

	if (di->chip == BQ27500)
		ilmd *= 1000;
	else
		ilmd = ilmd * 256 * 3570 / BQ27000_RS;

	return ilmd;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_temperature(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (di->chip == BQ27500)
		temp -= 2731;
	else
		temp = ((temp * 5) - 5463) / 2;

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = { 0, };
	bq27x00_register_bulk_cache(di);

	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (cache.flags >= 0) {

		cache.capacity = bq27x00_battery_read_rsoc(di);
		/* if we got bat_low irq from gg, force android to shutdown.
		 * it is reflected by RemainingCapacity instead of SoC
		 * so we cannot sure the exact SoC, maybe 1 or 2
		 */
		if(unlikely(need_to_shutdown))
			cache.capacity = 1;
		cache.time_to_empty =
		    bq27x00_battery_read_time(di, BQ27x00_REG_TTE);
		cache.time_to_empty_avg =
		    bq27x00_battery_read_time(di, BQ27x00_REG_TTECP);
		cache.time_to_full =
		    bq27x00_battery_read_time(di, BQ27x00_REG_TTF);
		cache.charge_full = bq27x00_battery_read_lmd(di);
		cache.cycle_count = bq27x00_battery_read_cyct(di);
		cache.temperature = bq27x00_battery_read_temperature(di);

		/* We only have to read charge design full once */
		if (unlikely(di->charge_design_full <= 0))
			di->charge_design_full = bq27x00_battery_read_ilmd(di);
	}

	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0) {
		di->cache = cache;
		power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
}

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
	    container_of(work, struct bq27x00_device_info, work.work);

	unsigned int polling_interval = poll_interval;

	bq27x00_update(di);

	if (di->rapid_poll_cycle < T_POLL_PLUG_MAX) {
			polling_interval = T_POLL_PLUG_MS;
			++di->rapid_poll_cycle;
	}

	if (polling_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, msecs_to_jiffies(polling_interval) / 4);
		schedule_delayed_work(&di->work, msecs_to_jiffies(polling_interval));
	}
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
				   union power_supply_propval *val)
{
	int curr;

	curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (di->chip == BQ27500) {
		/* bq27500 returns signed value */
		val->intval = (int)((s16) curr) * 1000;
	} else {
		if (di->cache.flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27000_RS;
	}

	return 0;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
				  union power_supply_propval *val)
{
	int status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (di->chip == BQ27500) {
		if ((USB_EVENT_NONE == di->current_usb_event) ||
			(di->cache.flags & BQ27500_FLAG_DSC))
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (USB_EVENT_CHARGER == di->current_usb_event) {

#ifdef CONFIG_CHARGER_BQ2419x
			/* Return STATUS_FULL if bq2419x reporting status 3 (Charging Done) */
			if (bq2419x_is_charging_done())
				status = POWER_SUPPLY_STATUS_FULL;
			else
				status = POWER_SUPPLY_STATUS_CHARGING;
#else
			status = POWER_SUPPLY_STATUS_CHARGING;

#endif /* CHARGER_BQ2419x */
		}
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(&di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	val->intval = status;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
				   union power_supply_propval *val, bool cached)
{
	int volt;

	volt = (cached ? bq27x00_read(di, BQ27x00_REG_VOLT, false) :
			bq27x00_register_uncached_read(di, BQ27x00_REG_VOLT, false));
	if (volt < 0)
		return volt;

	val->intval = volt * 1000;

	return 0;
}

int bq27x00_read_voltage_mv(void)
{
	union power_supply_propval val;

	bq27x00_battery_voltage(bq27x00_di, &val, false);
	return val.intval;
}
EXPORT_SYMBOL_GPL(bq27x00_read_voltage_mv);
/* */

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_energy(struct bq27x00_device_info *di,
			union power_supply_propval *val)
{
	int ae;

	ae = bq27x00_read(di, BQ27x00_REG_AE, false);
	if (ae < 0) {
		dev_err(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27500)
		ae *= 1000;
	else
		ae = ae * 29200 / BQ27000_RS;

	val->intval = ae;

	return 0;
}

static int bq27510_battery_health(struct bq27x00_device_info *di,
			union power_supply_propval *val)
{
	int ret = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	const int flags = di->cache.flags;

	if ((flags & BQ27500_FLAG_OTC) || (flags & BQ27500_FLAG_OTD))
		ret = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (flags & (BQ27500_FLAG_XCHG | BQ27500_FLAG_CHG_INH)) {
		if (di->cache.temperature < 0)
			ret = POWER_SUPPLY_HEALTH_COLD;
		else
			ret = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (flags & BQ27500_FLAG_BAT_DET)
		ret = POWER_SUPPLY_HEALTH_GOOD;

	val->intval = ret;

	return 0;
}

static void bq27x00_hw_reset(struct bq27x00_device_info *di)
{
	int retry_counter = 5;
	int initcomp = 0;

	printk(KERN_WARNING "+bq27x00_hw_reset: gpio_ce:%d\n", di->gpio_ce);

	gpio_request(di->gpio_ce , "gpio_ce");
	gpio_direction_output(di->gpio_ce, 1);
	msleep(200);
	gpio_direction_output(di->gpio_ce, 0);
	/* Wait 3 seconds then start polling the INITCOMP bit every 500ms */
	msleep(2500);

	do {
		msleep(500);
		bq27x00_read_control(BQ27x00_CONTROL_STATUS,
			&initcomp, bq27x00_di);
	} while (!(initcomp & BQ27x00_CONTROL_STATUS_INITCOMP)
				&& (retry_counter-- > 0));

	printk(KERN_WARNING "-bq27x00_hw_reset: %d\n", retry_counter);

}

static int bq27x00_simple_value(int value, union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info * const di = to_bq27x00_device_info(psy);

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27510_battery_health(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27x00_battery_voltage(di, val, true);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ((di->cache.flags & BQ27500_FLAG_BAT_DET) ? 1 : 0);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27x00_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->cache.temperature;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_battery_energy(di, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static ssize_t bq275200_get_device_type(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev->parent);
	return sprintf(buf, "0x%04x\n", di->sys_device_type);
}

static DEVICE_ATTR(device_type, S_IRUGO, bq275200_get_device_type, NULL);

static ssize_t bq275200_get_fw_version(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev->parent);
	return sprintf(buf, "0x%04x\n", di->sys_fw_version);
}

static DEVICE_ATTR(fw_version, S_IRUGO, bq275200_get_fw_version, NULL);

static ssize_t bq275200_get_df_version(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev->parent);
	int ret;

	if (unlikely(bq27x00_read_control(BQ27x00_CONTROL_DF_VERSION,
				   &ret, di) < 0))
		dev_err(di->dev, "failed to read df_version\n");

	return sprintf(buf, "0x%04x\n", ret);
}

static DEVICE_ATTR(df_version, S_IRUGO, bq275200_get_df_version, NULL);


static int bq27x00_get_device_version(struct bq27x00_device_info *di)
{
	int ret;

	int retry_counter = 3;

	do {
		ret = bq27x00_read_control(BQ27x00_CONTROL_DEVICE_TYPE,
					   &di->sys_device_type, di);
		if (unlikely(ret < 0)) {
			dev_err(di->dev, "failed to read device_type: %d\n", ret);
			bq27x00_hw_reset(di);
		}
	} while (retry_counter-- > 0);


	ret = bq27x00_read_control(BQ27x00_CONTROL_FW_VERSION,
				   &di->sys_fw_version, di);
	if (unlikely(ret < 0))
		dev_err(di->dev, "failed to read fw_version: %d\n", ret);

	return ret;
}

static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;

	init_rwsem(&di->bulk_reg_cache_lock);
	mutex_init(&di->lock);

	INIT_WORK(&di->event_work, bq27x00_event_work_func);
	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	ret = bq27x00_get_device_version(di);
	if (ret) {
		dev_err(di->dev, "failed to get device version: %d\n", ret);
		return ret;
	}

	ret = device_create_file(di->bat.dev, &dev_attr_pause_i2c);
	if (unlikely(ret))
		dev_err(di->dev, "failed to create pause_i2c sysfs: %d\n", ret);

	ret = device_create_file(di->bat.dev, &dev_attr_hw_reset);
	if (unlikely(ret))
		dev_err(di->dev, "failed to create hw_reset sysfs: %d\n", ret);

	ret = device_create_file(di->bat.dev, &dev_attr_device_type);
	if (unlikely(ret))
		dev_err(di->dev, "failed to create device_type sysfs: %d\n",
			ret);

	ret = device_create_file(di->bat.dev, &dev_attr_fw_version);
	if (unlikely(ret))
		dev_err(di->dev, "failed to create fw_version sysfs: %d\n",
			ret);

	ret = device_create_file(di->bat.dev, &dev_attr_df_version);
	if (unlikely(ret))
		dev_err(di->dev, "failed to create df_version sysfs: %d\n",
			ret);


	bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	/*
	 * power_supply_unregister call bq27x00_battery_get_property which
	 * call bq27x00_battery_poll.
	 * Make sure that bq27x00_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;
	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}

/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (unlikely(g_pause_i2c))
		return -EBUSY;

	if (unlikely(!client->adapter))
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static void bq27x00_event_work_func(struct work_struct *work)
{
	struct bq27x00_device_info * const di = container_of(work,
						      struct bq27x00_device_info,
						      event_work);

	cancel_delayed_work_sync(&di->work);
	di->rapid_poll_cycle = 0;
	schedule_delayed_work(&di->work,
			msecs_to_jiffies(T_POLL_PLUG_MS*2));
}

static int usb_xceiv_events_callback(struct notifier_block *nb,
				unsigned long event,
				void *_data)
{
	struct bq27x00_device_info * const di =
		container_of(nb, struct bq27x00_device_info, nb);

	bool perform_rapid_poll = false;

	switch (event) {
	case USB_EVENT_VBUS: /* fall-thru */
	case USB_EVENT_CHARGER: /* fall-thru */
		need_to_shutdown = 0;
	case USB_EVENT_NONE:
		perform_rapid_poll = true;
		di->current_usb_event = event;
		break;
	case USB_EVENT_ENUMERATED:
		if (!perform_rapid_poll)
			perform_rapid_poll = true;
		break;
	case USB_EVENT_ID: /* fall-thru */
	default:
		break;
	}

	if (perform_rapid_poll)
		schedule_work(&di->event_work);

	return 0;
}

static void bq27x00_bat_low_interrupt_work_func(struct work_struct *work)
{

	struct bq27x00_device_info *di = container_of(work,
						      struct bq27x00_device_info,
						      bat_low_work);
	bq27x00_update(di);
	printk("%s, SoC:%d\n",__func__, bq27x00_battery_read_rsoc(di));
}

static irqreturn_t bq27x00_bat_low_interrupt(int irq, void *dev)
{
	struct bq27x00_device_info * const di = dev_get_drvdata(dev);
	printk("BQ27x00 BAT LOW INTERRUPT...\n");
	/* report 1% when we get the low SoC interrupt
	 * This will allow Android to prompt the user for 60 sec to attach their charger.
	 */
	wake_lock_timeout(&di->bat_low_wakelock, 60 * HZ);
	if(!is_chargermode)
		need_to_shutdown = 1;
	schedule_work(&di->bat_low_work);
	return IRQ_HANDLED;
}

static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	struct bq27x00_platform_data *pdata = NULL;
	int num;
	int retval = 0;

	pr_debug("bq27x00 probe i2c\n");
	pdata = client->dev.platform_data;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	bq27x00_di = di;

	di->id = num;
	di->dev = &client->dev;
	di->chip = id->driver_data;
	di->bat.name = name;
	di->bus.read = &bq27x00_read_i2c;

	di->gpio_ce = pdata->gpio_ce;
	di->gpio_soc_int = pdata->gpio_soc_int;
	di->gpio_bat_low = pdata->gpio_bat_low;


	/* Register interrupt and its work */
	INIT_WORK(&di->bat_low_work, bq27x00_bat_low_interrupt_work_func);

	if (gpio_is_valid(di->gpio_bat_low)) {
		if (gpio_request(di->gpio_bat_low, "gpio_bat_low_int")) {
			dev_err(di->dev, "gpio_bat_low_int pin not available\n");
			retval = -ENODEV;
			goto batt_failed_3;
		} else
			gpio_direction_input(di->gpio_bat_low);
	}
	retval = request_irq(OMAP_GPIO_IRQ(di->gpio_bat_low),
			  bq27x00_bat_low_interrupt,
			  IRQF_TRIGGER_FALLING,
			  "bat_nlow",
			  di->dev);
	if (retval < 0) {
		dev_err(di->dev, "%s: cannot regitser IRQ %d, retval=%d\n", __func__,
							di->gpio_bat_low, retval);
		goto batt_failed_3;
	}
	wake_lock_init(&di->bat_low_wakelock, WAKE_LOCK_SUSPEND, "bq27x00_bat_low_lock");
	printk("finish bat_low irq. gpio_bat_low:%d\n", di->gpio_bat_low);

	retval = bq27x00_powersupply_init(di);
	if (retval)
		goto batt_failed_3;

	i2c_set_clientdata(client, di);

	di->nb.notifier_call = usb_xceiv_events_callback;
	di->otg = otg_get_transceiver();
	if (di->otg) {
		retval = otg_register_notifier(di->otg, &di->nb);
		if (retval) {
			dev_err(di->dev, "otg register notifier"
						" failed %d\n", retval);
			goto batt_failed_3;
		}
		di->current_usb_event = twl6030_usbotg_get_status();
	} else {
		dev_err(di->dev, "otg_get_transceiver failed %d\n", retval);
		goto batt_failed_3;
	}

	return 0;

batt_failed_3:
	kfree(di);
	cancel_work_sync(&di->bat_low_work);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);
	cancel_work_sync(&di->bat_low_work);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

static int bq27x00_read_control(u16 reg, int *rt_value,
				struct bq27x00_device_info *di)
{
	struct i2c_client *const client = to_i2c_client(di->dev);
	struct i2c_msg msg[1];
	unsigned char data[4];
	int err;

	if (unlikely(!client->adapter))
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = data;

	data[0] = BQ27x00_REG_CONTROL;
	data[1] = reg & 0xff;
	data[2] = (reg >> 8) & 0xff;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		msleep(2);
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 1;
		msg->buf = data;

		data[0] = BQ27x00_REG_CONTROL;
		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0) {
			msg->len = 2;
			msg->flags = I2C_M_RD;
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				*rt_value = data[0] | (data[1] << 8);
				return 0;
			}
		}
	}
	return err;
}

static const struct i2c_device_id bq27x00_id[] = {
	{"bq27200", BQ27000},	/* bq27200 is same as bq27000, but with i2c */
	{"bq27500", BQ27500},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq27x00_id);

static int bq27x00_battery_suspend(struct device *dev)
{
	return 0;
}

static int bq27x00_battery_resume(struct device *dev)
{
	struct platform_device * const pdev = to_platform_device(dev);
	struct bq27x00_device_info * const di = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&di->work);
	di->rapid_poll_cycle = 0;
	schedule_delayed_work(&di->work,
		msecs_to_jiffies(T_POLL_PLUG_MS));

	return 0;
}

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		   .name = "bq27x00-battery",
		   },
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

#else

static inline int bq27x00_battery_i2c_init(void)
{
	return 0;
}

static inline void bq27x00_battery_i2c_exit(void)
{
};

#endif

/* platform specific code */
#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM

static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
				 bool single)
{
	struct device *dev = di->dev;
	struct bq27x00_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}


static int __devinit bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27x00_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	pr_debug("bq27x00 battery probe platform\n");

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27000;

	di->bat.name = pdata->name ? : dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	bq27x00_di = di;

	return 0;

err_free:
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

static int __devexit bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static const struct dev_pm_ops pm_ops = {
	.suspend = bq27x00_battery_suspend,
	.resume = bq27x00_battery_resume,
};

static struct platform_driver bq27000_battery_driver = {
	.probe = bq27000_battery_probe,
	.remove = __devexit_p(bq27000_battery_remove),
	.driver = {
		   .name = "bq27000-battery",
		   .owner = THIS_MODULE,
		   .pm = &pm_ops
		   },
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27000 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

#else

static inline int bq27x00_battery_platform_init(void)
{
	return 0;
}

static inline void bq27x00_battery_platform_exit(void)
{
};

#endif

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	ret = bq27x00_battery_platform_init();
	if (ret)
		bq27x00_battery_i2c_exit();

	return ret;
}

module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
}

module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
