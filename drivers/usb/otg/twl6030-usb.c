/*
 * twl6030_usb - TWL6030 USB transceiver, talking to OMAP OTG driver.
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: Hema HK <hemahk@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/usb/otg.h>
#include <linux/usb/phy_companion.h>
#include <linux/usb/omap_usb.h>
#include <linux/usb/musb-omap.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <plat/usb.h>
#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif

/* usb register definitions */
#define USB_VENDOR_ID_LSB		0x00
#define USB_VENDOR_ID_MSB		0x01
#define USB_PRODUCT_ID_LSB		0x02
#define USB_PRODUCT_ID_MSB		0x03
#define USB_VBUS_CTRL_SET		0x04
#define USB_VBUS_CTRL_CLR		0x05
#define USB_ID_CTRL_SET			0x06
#define USB_ID_CTRL_CLR			0x07
#define USB_VBUS_INT_SRC		0x08
#define USB_VBUS_INT_LATCH_SET		0x09
#define USB_VBUS_INT_LATCH_CLR		0x0A
#define USB_VBUS_INT_EN_LO_SET		0x0B
#define USB_VBUS_INT_EN_LO_CLR		0x0C
#define USB_VBUS_INT_EN_HI_SET		0x0D
#define USB_VBUS_INT_EN_HI_CLR		0x0E
#define USB_ID_INT_SRC			0x0F
#define USB_ID_INT_LATCH_SET		0x10
#define USB_ID_INT_LATCH_CLR		0x11

#define USB_ID_INT_EN_LO_SET		0x12
#define USB_ID_INT_EN_LO_CLR		0x13
#define USB_ID_INT_EN_HI_SET		0x14
#define USB_ID_INT_EN_HI_CLR		0x15
#define USB_OTG_ADP_CTRL		0x16
#define USB_OTG_ADP_HIGH		0x17
#define USB_OTG_ADP_LOW			0x18
#define USB_OTG_ADP_RISE		0x19
#define USB_OTG_REVISION		0x1A

#define USB_ID_INT_MASK			0x1F

#define CONTROLLER_INT_MASK		0x00

/* to be moved to LDO */
#define TWL6030_MISC2			0xE5
#define TWL6030_CFG_LDO_PD2		0xF5
#define TWL6030_BACKUP_REG		0xFA

/* bits in MISC2 Register */
#define VUSB_IN_PMID			BIT(3)
#define VUSB_IN_VBAT			BIT(4)

#define STS_HW_CONDITIONS		0x21

/* In module TWL6030_MODULE_PM_MASTER */
#define STS_HW_CONDITIONS		0x21
#define STS_USB_ID			BIT(2)

/* In module TWL6030_MODULE_PM_RECEIVER */
#define VUSB_CFG_TRANS			0x71
#define VUSB_CFG_STATE			0x72
#define VUSB_CFG_VOLTAGE		0x73

/* in module TWL6030_MODULE_MAIN_CHARGE */

#define CHARGERUSB_CTRL1		0x8

#define BOOST_MODE_OFF_MASK		0x00
#define BOOST_MODE_ON_MASK		0x40
#define BOOST_MODE_STATE_MASK		0x60

#define CONTROLLER_STAT1		0x03
#define	VBUS_DET			BIT(2)

#define VBUS_TIMEOUT_LOCK		(500)

struct twl6030_usb {
	struct phy_companion	comparator;
	struct device		*dev;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	struct regulator		*usb3v3;

	/* used to set vbus, in atomic path */
	struct work_struct	set_vbus_work;

	int			irq1;
	int			irq2;
	bool			irq_enabled;
	bool			vbus_enable;
	bool			regulator_enabled;
	unsigned long		features;
	u32			errata;
	u8			vbus_pre_state;
	enum omap_musb_vbus_id_status state;
	enum power_supply_type supply_type;

	struct notifier_block usb_notifier;
	struct workqueue_struct *work_queue;
	struct work_struct works[TWL6030_USB_EVENT_COUNT];
	struct wake_lock usb_lock;
};

static BLOCKING_NOTIFIER_HEAD(notifier_list);

static void twl6030_enable_ldo_input_supply(struct twl6030_usb *twl,
		bool enable);

#define	comparator_to_twl(x) container_of((x), struct twl6030_usb, comparator)
/*-------------------------------------------------------------------------*/

/*
 * State machine true table:
 * Valid cases:
 * -------+---------+---------+-----------------------------------------------
 *        |   current state   |
 * event  +---------+---------+ Actions and next state
 *        |  state  | supply  |
 * -------+---------+---------+-----------------------------------------------
 * VBUS_ON|   OFF   |    /    | Enable 3.3V, wake_lock, Notify chain
 *        |         |         | Next state: ON / UNKNOWN
 * -------+---------+---------+-----------------------------------------------
 *  OFF   |  !OFF   |    /    | Disable 3.3V, wake_unlock, Notify chain
 *        |         |         | Notify MUSB with MailBox
 *        |         |         | Next state: OFF / UNKNOWN
 * -------+---------+---------+-----------------------------------------------
 * DETECT |   ON    | UNKNOWN | Notify chain, Notify MUSB with MailBox
 *        |         |         | Next state: DETECT / !UNKNOWN
 *        |         |         | NOTE: if return supply UNKNOWN notify
 *        |         |         |       chain with last event
 * -------+---------+---------+-----------------------------------------------
 * OTG_GND|   OFF   | UNKNOWN | Notify chain, Notify MUSB with MailBox
 *        |         |         | Next state: OTG_GND / UNKNOWN
 * -------+---------+---------+-----------------------------------------------
 * OTG_OFF| OTG_GND |    /    | Notify chain
 *        |         |         | Next state: OFF / not change
 * -------+---------+---------+-----------------------------------------------
 *
 * Invalid transitions:
 * -------+---------+---------+-----------------------------------------------
 *  ON    |  !OFF   |    /    | VBUS is already ON
 * -------+---------+---------+-----------------------------------------------
 *  OFF   |   OFF   |    /    | VBUS is already OFF
 * -------+---------+---------+-----------------------------------------------
 * DETECT |   OFF   |    /    | Supply detection during VBUS OFF
 * -------+---------+---------+-----------------------------------------------
 * DETECT | DETECT  |    /    | Supply type is already detected
 * -------+---------+---------+-----------------------------------------------
 * DETECT | OTG_GND |    /    | Supply type is already selected
 * -------+---------+---------+-----------------------------------------------
 * OTG_GND|  !OFF   |    /    | OTG detect on incorrect state
 * -------+---------+---------+-----------------------------------------------
 * OTG_OFF|  OFF    |    /    | Just ignore
 * -------+---------+---------+-----------------------------------------------
 *
 * Chain callback actions:
 *   ON     - Start APSD
 *   OFF    - Stop APSD
 *   DETECT - Return VBUS type
 *
 */

static char *twl6030_event_str[] = {
	"VBUS_OFF", "VBUS_ON", "VBUS_DETECT", "OTG_GND", "OTG_OFF" };

#define IS_DATA_STREAM(supply) \
	(supply == POWER_SUPPLY_TYPE_USB || supply == POWER_SUPPLY_TYPE_USB_CDP)

/*
 * Manage TWL6030 USB wake_lock as follows:
 *  timeout <  0 - Necessarily infinitely locking
 *  timeout =  0 - Necessarily unlock
 *  timeout >  0 - Set new timeout if current state is not locked infinitely
 */
static void twl6030_wake_ctrl(struct twl6030_usb *twl, int timeout)
{
	unsigned long flags;

	dev_dbg(twl->dev, "%s(%d): wakelock is %s...\n", __func__, timeout,
		wake_lock_active(&twl->usb_lock) ? "locked" : "unlocked");

	spin_lock_irqsave(&twl->lock, flags);

	if (timeout > 0) {
		/* Set wake-lock by timeout if possible */
		if (!wake_lock_active(&twl->usb_lock)) {
			wake_unlock(&twl->usb_lock);
			timeout = msecs_to_jiffies(timeout);
			wake_lock_timeout(&twl->usb_lock, timeout);
		}
	} else {
		/* Set infinite wake-lock by timeout if possible */
		wake_unlock(&twl->usb_lock);
		if (timeout < 0)
			wake_lock(&twl->usb_lock);
	}

	spin_unlock_irqrestore(&twl->lock, flags);
}

/* Handle event TWL6030_USB_EVENT_VBUS_ON */
static void twl6030_hnd_vbus_on(struct work_struct *work)
{
	enum twl6030_usb_events event = TWL6030_USB_EVENT_VBUS_ON;
	struct twl6030_usb *twl;
	int ret = -EINVAL;
	twl = container_of(work, struct twl6030_usb, works[event]);

	switch (twl->state) {
	case TWL6030_USB_EVENT_VBUS_OFF:
	case TWL6030_USB_EVENT_OTG_OFF:
		dev_info(twl->dev,
			"Execute event %-14s, state=%s, supply=%d\n",
			twl6030_event_str[event], twl6030_event_str[twl->state],
			twl->supply_type);

		if (!twl->regulator_enabled) {
			twl6030_enable_ldo_input_supply(twl, true);
			regulator_enable(twl->usb3v3);
			twl->regulator_enabled = 1;
		}

		twl->state = event;
		twl->supply_type = POWER_SUPPLY_TYPE_UNKNOWN;

		ret = blocking_notifier_call_chain(&notifier_list, event, NULL);
#ifdef CONFIG_AMAZON_METRICS_LOG
		{
		char buf[128];
		snprintf(buf, sizeof(buf),
			"%s:twl6030:vbus_on=1;CT;1,state=%d;DV;1:NR",
			__func__, twl->state);
		log_to_metrics(ANDROID_LOG_INFO, "USBCableEvent", buf);
		}
#endif
		return;

	default:
		dev_dbg(twl->dev, "VBUS is already ON!\n");
		break;
	}

	if (ret)
		return;

	/* shouldn't be here in any case */
	dump_stack();
}

/* Handle event TWL6030_USB_EVENT_VBUS_OFF */
static void twl6030_hnd_vbus_off(struct work_struct *work)
{
	enum twl6030_usb_events event = TWL6030_USB_EVENT_VBUS_OFF;
	struct twl6030_usb *twl;
	int ret = -EINVAL;
	unsigned long flags;
	twl = container_of(work, struct twl6030_usb, works[event]);

	switch (twl->state) {
	case TWL6030_USB_EVENT_VBUS_ON:
	case TWL6030_USB_EVENT_VBUS_DETECT:
	case TWL6030_USB_EVENT_OTG_GND:
	case TWL6030_USB_EVENT_OTG_OFF:
		dev_info(twl->dev, "Execute event %-14s, state=%s, supply=%d\n",
			twl6030_event_str[event], twl6030_event_str[twl->state],
			twl->supply_type);
		ret = blocking_notifier_call_chain(&notifier_list, event, NULL);
#ifdef CONFIG_AMAZON_METRICS_LOG
		{
		char buf[128];
		snprintf(buf, sizeof(buf),
			"%s:twl6030:vbus_off=1;CT;1,state=%d;DV;1:NR",
			__func__, twl->state);
		log_to_metrics(ANDROID_LOG_INFO, "USBCableEvent", buf);
		}
#endif
		break;

	default:
		dev_dbg(twl->dev, "VBUS is already OFF!\n");
		break;
	}

	if (ret)
		return;

	if (IS_DATA_STREAM(twl->supply_type))
		omap_musb_mailbox(OMAP_MUSB_VBUS_OFF);

	if (twl->regulator_enabled) {
		dev_info(twl->dev, "%s: regulator is enabled, going to disable it...\n",
			__func__);
		regulator_disable(twl->usb3v3);
		twl6030_enable_ldo_input_supply(twl, false);
		twl->regulator_enabled = 0;
	} else {
		dev_err(twl->dev, "%s: regulator was NOT enabled\n", __func__);
	}

	spin_lock_irqsave(&twl->lock, flags);
	if (wake_lock_active(&twl->usb_lock))
		wake_unlock(&twl->usb_lock);
	wake_lock_timeout(&twl->usb_lock, msecs_to_jiffies(1500));
	spin_unlock_irqrestore(&twl->lock, flags);

	sysfs_notify(&twl->dev->kobj, NULL, "vbus");

	twl->state = event;
	twl->supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
}

/* Handle event TWL6030_USB_EVENT_VBUS_DETECT */
static void twl6030_hnd_detect(struct work_struct *work)
{
	enum twl6030_usb_events event = TWL6030_USB_EVENT_VBUS_DETECT;
	struct twl6030_usb *twl;
	int ret = -EINVAL;
	twl = container_of(work, struct twl6030_usb, works[event]);

	switch (twl->state) {
	case TWL6030_USB_EVENT_VBUS_ON:
		dev_info(twl->dev, "Execute event %-14s, state=%s, supply=%d\n",
			twl6030_event_str[event], twl6030_event_str[twl->state],
			twl->supply_type);
		dev_dbg(twl->dev, "before notifier_call_chain,ret=%d\n", ret);
		ret = blocking_notifier_call_chain(&notifier_list, event,
			&twl->supply_type);
		dev_dbg(twl->dev, "after notifier_call_chain,ret=%d\n", ret);
#ifdef CONFIG_AMAZON_METRICS_LOG
		{
		char buf[128];
		snprintf(buf, sizeof(buf),
			"%s:twl6030:detect=1;CT;1,state=%d;DV;1:NR",
			__func__, twl->state);
		log_to_metrics(ANDROID_LOG_INFO, "USBCableEvent", buf);
		}
#endif
		break;

	case TWL6030_USB_EVENT_VBUS_OFF:
	case TWL6030_USB_EVENT_OTG_OFF:
		/* Set timeout to wait for VBUS ON detection */
		dev_dbg(twl->dev, "Supply detection during VBUS OFF!\n");
		break;

	case TWL6030_USB_EVENT_VBUS_DETECT:
		dev_dbg(twl->dev, "Supply type is already detected\n");
		break;

	case TWL6030_USB_EVENT_OTG_GND:
		dev_dbg(twl->dev, "Supply type is already selected!\n");
		break;
	}

	if (ret)
		return;

	if (twl->supply_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		/* If supplier detection fail, send last event, and wait agian */
		blocking_notifier_call_chain(&notifier_list, twl->state, NULL);
		return;
	}

	twl->state = event;
	if (IS_DATA_STREAM(twl->supply_type))
		omap_musb_mailbox(OMAP_MUSB_VBUS_VALID);
	sysfs_notify(&twl->dev->kobj, NULL, "vbus");
}

/* Handle event TWL6030_USB_EVENT_OTG_GND */
static void twl6030_hnd_otg_gnd(struct work_struct *work)
{
	enum twl6030_usb_events event = TWL6030_USB_EVENT_OTG_GND;
	struct twl6030_usb *twl;
	int ret = -EINVAL;
	twl = container_of(work, struct twl6030_usb, works[event]);

	switch (twl->state) {
	case TWL6030_USB_EVENT_VBUS_OFF:
	case TWL6030_USB_EVENT_OTG_OFF:
		dev_dbg(twl->dev, "Execute event %-14s, state=%s, supply=%d\n",
			twl6030_event_str[event], twl6030_event_str[twl->state],
			twl->supply_type);
		ret = blocking_notifier_call_chain(&notifier_list, event, NULL);
#ifdef CONFIG_AMAZON_METRICS_LOG
		{
		char buf[128];
		snprintf(buf, sizeof(buf),
			"%s:twl6030:otg_gnd=1;CT;1,state=%d;DV;1:NR",
			__func__, twl->state);
		log_to_metrics(ANDROID_LOG_INFO, "USBCableEvent", buf);
		}
#endif
		break;

	default:
		dev_dbg(twl->dev, "OTG detect on incorrect state!\n");
		break;
	}

	if (ret)
		return;
#ifndef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
	twl6030_enable_ldo_input_supply(twl, true);
	regulator_enable(twl->usb3v3);
#endif
	omap_musb_mailbox(OMAP_MUSB_ID_GROUND);

	twl->state = event;
	twl->supply_type = POWER_SUPPLY_TYPE_USB;
}

/* Handle event TWL6030_USB_EVENT_OTG_OFF */
static void twl6030_hnd_otg_off(struct work_struct *work)
{
	enum twl6030_usb_events event = TWL6030_USB_EVENT_OTG_OFF;
	struct twl6030_usb *twl;
	int ret = -EINVAL;
	twl = container_of(work, struct twl6030_usb, works[event]);

	dev_dbg(twl->dev, "Execute event %-14s, state=%s, supply=%d\n",
		twl6030_event_str[event], twl6030_event_str[twl->state],
		twl->supply_type);

	switch (twl->state) {
	case TWL6030_USB_EVENT_OTG_GND:
		ret = blocking_notifier_call_chain(&notifier_list, event, NULL);
#ifdef CONFIG_AMAZON_METRICS_LOG
		{
		char buf[128];
		snprintf(buf, sizeof(buf),
			"%s:twl6030:otg_off=1;CT;1,state=%d;DV;1:NR",
			__func__, twl->state);
		log_to_metrics(ANDROID_LOG_INFO, "USBCableEvent", buf);
		}
#endif
		break;

	case TWL6030_USB_EVENT_VBUS_ON:
		break;

	default:
		dev_dbg(twl->dev, "OTG OFF on incorrect state!\n");
		break;
	}

	if (ret)
		return;
#ifndef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
	regulator_disable(twl->usb3v3);
	twl6030_enable_ldo_input_supply(twl, false);
#endif
	omap_musb_mailbox(OMAP_MUSB_VBUS_OFF);

	twl->state = TWL6030_USB_EVENT_VBUS_OFF;
	twl->supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
}

int twl6030_usb_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&notifier_list, nb);
}
EXPORT_SYMBOL_GPL(twl6030_usb_register_notifier);

int twl6030_usb_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&notifier_list, nb);
}
EXPORT_SYMBOL_GPL(twl6030_usb_unregister_notifier);

static struct twl6030_usb *twl6030_priv;
int twl6030_usb_event(enum twl6030_usb_events event)
{
	struct twl6030_usb *twl = twl6030_priv;

	if (!twl)
		return -EINVAL;

	if (event < 0 || event >= TWL6030_USB_EVENT_COUNT)
		return -EINVAL;

	dev_info(twl->dev, "Receive event %-14s, state=%s, supply=%d\n",
		twl6030_event_str[event], twl6030_event_str[twl->state],
		twl->supply_type);
#ifndef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
	if (event != TWL6030_USB_EVENT_VBUS_OFF)
		twl6030_wake_ctrl(twl, VBUS_TIMEOUT_LOCK);
#endif

	return queue_work(twl->work_queue, &twl->works[event]);
}
EXPORT_SYMBOL_GPL(twl6030_usb_event);

static inline int twl6030_writeb(struct twl6030_usb *twl, u8 module,
						u8 data, u8 address)
{
	int ret = 0;

	ret = twl_i2c_write_u8(module, data, address);
	if (ret < 0)
		dev_err(twl->dev,
			"Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline u8 twl6030_readb(struct twl6030_usb *twl, u8 module, u8 address)
{
	u8 data, ret = 0;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_err(twl->dev,
			"readb[0x%x,0x%x] Error %d\n",
					module, address, ret);
	return ret;
}

static int twl6030_start_srp(struct phy_companion *comparator)
{
	struct twl6030_usb *twl = comparator_to_twl(comparator);

	twl6030_writeb(twl, TWL_MODULE_USB, 0x24, USB_VBUS_CTRL_SET);
	twl6030_writeb(twl, TWL_MODULE_USB, 0x84, USB_VBUS_CTRL_SET);

	mdelay(100);
	twl6030_writeb(twl, TWL_MODULE_USB, 0xa0, USB_VBUS_CTRL_CLR);

	return 0;
}

static void twl6030_enable_ldo_input_supply(struct twl6030_usb *twl,
		bool enable)
{
	u8 misc2_data = 0;

	misc2_data = twl6030_readb(twl, TWL6030_MODULE_ID0, TWL6030_MISC2);
	misc2_data &= ~(VUSB_IN_PMID | VUSB_IN_VBAT);
	if (enable)
		misc2_data |= VUSB_IN_PMID;
	twl6030_writeb(twl, TWL6030_MODULE_ID0, misc2_data, TWL6030_MISC2);
	dev_info(twl->dev, "%s(%s)", __func__, enable ? "enable" : "disable");
}

static int twl6030_usb_ldo_init(struct twl6030_usb *twl)
{
	/* Set to OTG_REV 1.3 and turn on the ID_WAKEUP_COMP */
	twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x1, TWL6030_BACKUP_REG);

	/* Program CFG_LDO_PD2 register and set VUSB bit */
	twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x1, TWL6030_CFG_LDO_PD2);

	twl->usb3v3 = regulator_get(twl->dev, "vusb");
	if (IS_ERR(twl->usb3v3))
		return -ENODEV;

	/* Program the USB_VBUS_CTRL_SET and set VBUS_ACT_COMP bit */
	twl6030_writeb(twl, TWL_MODULE_USB, 0x4, USB_VBUS_CTRL_SET);

	/*
	 * Program the USB_ID_CTRL_SET register to enable GND drive
	 * and the ID comparators
	 */
	twl6030_writeb(twl, TWL_MODULE_USB, 0x14, USB_ID_CTRL_SET);

	/* Disable LDO before disabling his input supply */
	regulator_force_disable(twl->usb3v3);
	twl6030_enable_ldo_input_supply(twl, false);
	twl->regulator_enabled = 0;

	return 0;
}

static ssize_t twl6030_usb_vbus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct twl6030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);

	switch (twl->state) {
	case OMAP_MUSB_VBUS_VALID:
	       ret = snprintf(buf, PAGE_SIZE, "vbus\n");
	       break;
	case OMAP_MUSB_ID_GROUND:
	       ret = snprintf(buf, PAGE_SIZE, "id\n");
	       break;
	case OMAP_MUSB_VBUS_OFF:
	case OMAP_MUSB_ID_FLOAT:
	       ret = snprintf(buf, PAGE_SIZE, "none\n");
	       break;
	default:
	       ret = snprintf(buf, PAGE_SIZE, "UNKNOWN\n");
	}
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}
static DEVICE_ATTR(vbus, 0444, twl6030_usb_vbus_show, NULL);

static irqreturn_t twl6030_usb_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	u8 vbus_state, hw_state;

	vbus_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);
	if (vbus_state & STS_USB_ID)
		return IRQ_HANDLED;

	hw_state = twl6030_readb(twl, TWL_MODULE_MAIN_CHARGE, CONTROLLER_STAT1);
	dev_dbg(twl->dev, "in %s, vbus_state: %02x, hw_state: %02x\n",
			__func__, vbus_state, hw_state);

	/* filter out duplicate state */
	if ((hw_state & VBUS_DET) == twl->vbus_pre_state)
		return IRQ_HANDLED;

	twl->vbus_pre_state = (hw_state & VBUS_DET);

	if (hw_state & VBUS_DET) {
#ifdef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
		wake_lock(&twl->usb_lock);
#endif
		twl6030_usb_event(TWL6030_USB_EVENT_VBUS_ON);
	} else
		twl6030_usb_event(TWL6030_USB_EVENT_VBUS_OFF);

	return IRQ_HANDLED;
}

static irqreturn_t twl6030_usbotg_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	u8 hw_state;

	hw_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);
	if (hw_state & STS_USB_ID)
		twl6030_usb_event(TWL6030_USB_EVENT_OTG_GND);
	else
		twl6030_usb_event(TWL6030_USB_EVENT_OTG_OFF);

	return IRQ_HANDLED;
}

static int twl6030_enable_irq(struct twl6030_usb *twl)
{
	twl6030_writeb(twl, TWL_MODULE_USB, 0x1, USB_ID_INT_EN_HI_SET);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_STS_C);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);
	twl6030_usb_irq(twl->irq2, twl);
	twl6030_usbotg_irq(twl->irq1, twl);

	return 0;
}

static int twl6030_notifier_cb(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct twl6030_usb *twl;
#ifndef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
	enum power_supply_type *supply = data;
	int charger_type;
#endif
	int status = USB_EVENT_NONE;

	twl = container_of(nb, struct twl6030_usb, usb_notifier);

	switch (event) {
	case TWL6030_USB_EVENT_VBUS_ON:
#ifdef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
		/* Disable OMAP charger detection */
		__raw_writel(0x40000000,
			OMAP2_L4_IO_ADDRESS(0x4a100000 + (0x620)));
#else
		/* Trigger immediately VBUS type detection */
		twl6030_usb_event(TWL6030_USB_EVENT_VBUS_DETECT);
#endif
		break;

	case TWL6030_USB_EVENT_VBUS_OFF:
		break;

	case TWL6030_USB_EVENT_VBUS_DETECT:
		if (!data)
			return -EINVAL;
#ifndef CONFIG_TWL6030_USB_DISABLE_VBUS_TYPE_DETECTION
		charger_type = omap_usb2_charger_detect(&twl->comparator);
		if (charger_type == POWER_SUPPLY_TYPE_USB_DCP)
			*supply = POWER_SUPPLY_TYPE_USB_DCP;
		else
			*supply = POWER_SUPPLY_TYPE_USB;
#endif
		break;

	case TWL6030_USB_EVENT_OTG_GND:
		twl6030_writeb(twl, TWL_MODULE_USB, 0x1, USB_ID_INT_EN_HI_CLR);
		twl6030_writeb(twl, TWL_MODULE_USB, 0x10, USB_ID_INT_EN_HI_SET);
		status = USB_EVENT_ID;
		twl6030_writeb(twl, TWL_MODULE_USB, status, USB_ID_INT_LATCH_CLR);
		break;

	case TWL6030_USB_EVENT_OTG_OFF:
		twl6030_writeb(twl, TWL_MODULE_USB, 0x10, USB_ID_INT_EN_HI_CLR);
		twl6030_writeb(twl, TWL_MODULE_USB, 0x1, USB_ID_INT_EN_HI_SET);
		status = USB_EVENT_NONE;
		twl6030_writeb(twl, TWL_MODULE_USB, status, USB_ID_INT_LATCH_CLR);
		break;
	}
	return 0;
}

static void otg_set_vbus_work(struct work_struct *data)
{
	struct twl6030_usb *twl = container_of(data, struct twl6030_usb,
								set_vbus_work);

	/*
	 * Start driving VBUS. Set OPA_MODE bit and clear HZ_MODE bit
	 * in CHARGERUSB_CTRL1 register. This enables Boost mode for OTG
	 * purpose.
	 * Since in OTG operating mode internal USB charger used as VBUS supply
	 * then it should not be in high-impedance mode on VBUS.
	 */
	if (twl->vbus_enable)
		twl6030_writeb(twl, TWL_MODULE_MAIN_CHARGE,
			       BOOST_MODE_ON_MASK, CHARGERUSB_CTRL1);
	else
		twl6030_writeb(twl, TWL_MODULE_MAIN_CHARGE,
			       BOOST_MODE_OFF_MASK, CHARGERUSB_CTRL1);
}

static int twl6030_set_vbus(struct phy_companion *comparator, bool enabled)
{
	struct twl6030_usb *twl = comparator_to_twl(comparator);

	if (twl->vbus_enable != enabled) {
		twl->vbus_enable = enabled;
		schedule_work(&twl->set_vbus_work);
	}

	return 0;
}

static void twl6030_get_vbus(struct twl6030_usb *twl)
{
	u8 ctrl1_data;

	/*
	 * Read CHARGERUSB_CTRL1 register and set vbus_enable flag
	 * if BOOST mode for OTG purpose is enabled and USB charger
	 * is not in high-impedance mode on VBUS.
	 */
	ctrl1_data = twl6030_readb(twl, TWL_MODULE_MAIN_CHARGE,
				   CHARGERUSB_CTRL1);
	if ((ctrl1_data & BOOST_MODE_STATE_MASK) == BOOST_MODE_ON_MASK)
		twl->vbus_enable = true;
	else
		twl->vbus_enable = false;
}

static int __devinit twl6030_usb_probe(struct platform_device *pdev)
{
	struct twl6030_usb	*twl;
	int			status, err;
	struct twl4030_usb_data *pdata;
	struct device *dev = &pdev->dev;
	pdata = dev->platform_data;

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	twl->dev		= &pdev->dev;
	twl->irq1		= platform_get_irq(pdev, 0);
	twl->irq2		= platform_get_irq(pdev, 1);
	twl->features		= pdata->features;
	twl->errata		= pdata->errata;
	twl->vbus_pre_state	= 0;
	twl->state		= TWL6030_USB_EVENT_VBUS_OFF;
	twl->supply_type	= POWER_SUPPLY_TYPE_UNKNOWN;

	twl->comparator.set_vbus	= twl6030_set_vbus;
	twl->comparator.start_srp	= twl6030_start_srp;

	omap_usb2_set_comparator(&twl->comparator);

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = twl6030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(twl);
		return err;
	}

	twl6030_get_vbus(twl);

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	INIT_WORK(&twl->set_vbus_work, otg_set_vbus_work);

	twl->irq_enabled = true;
	status = request_threaded_irq(twl->irq1, NULL, twl6030_usbotg_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl6030_usb", twl);
	if (status < 0) {
		dev_err(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq1, status);
		device_remove_file(twl->dev, &dev_attr_vbus);
		kfree(twl);
		return status;
	}

	status = request_threaded_irq(twl->irq2, NULL, twl6030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl6030_usb", twl);
	if (status < 0) {
		dev_err(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq2, status);
		free_irq(twl->irq1, twl);
		device_remove_file(twl->dev, &dev_attr_vbus);
		kfree(twl);
		return status;
	}

	twl->usb_notifier.notifier_call = twl6030_notifier_cb;

	if (twl6030_usb_register_notifier(&twl->usb_notifier)) {
		dev_err(twl->dev, "otg_register_notifier failed!\n");
	}

	/* To avoid any need to use memory barrier must be create
	 * single threaded work queue. This ensures the use of common
	 * variables on a single CPU which avoids problems between
	 * L1 and L2 cache.
	 */
	twl->work_queue = create_singlethread_workqueue("twl6030_usb");
	INIT_WORK(&twl->works[TWL6030_USB_EVENT_VBUS_ON], twl6030_hnd_vbus_on);
	INIT_WORK(&twl->works[TWL6030_USB_EVENT_VBUS_OFF], twl6030_hnd_vbus_off);
	INIT_WORK(&twl->works[TWL6030_USB_EVENT_VBUS_DETECT], twl6030_hnd_detect);
	INIT_WORK(&twl->works[TWL6030_USB_EVENT_OTG_GND], twl6030_hnd_otg_gnd);
	INIT_WORK(&twl->works[TWL6030_USB_EVENT_OTG_OFF], twl6030_hnd_otg_off);

	wake_lock_init(&twl->usb_lock, WAKE_LOCK_SUSPEND, "twl6030_usb");

	twl6030_priv = twl;

	if (twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0xFD, CONTROLLER_INT_MASK))
		dev_warn(&pdev->dev, "Could not disable charger interrupts\n");

	twl6030_enable_irq(twl);
	dev_info(&pdev->dev, "Initialized TWL6030 USB module\n");

	return 0;
}

static int __exit twl6030_usb_remove(struct platform_device *pdev)
{
	struct twl6030_usb *twl = platform_get_drvdata(pdev);

	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
		REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
			REG_INT_MSK_STS_C);
	free_irq(twl->irq1, twl);
	free_irq(twl->irq2, twl);
	regulator_put(twl->usb3v3);
	device_remove_file(twl->dev, &dev_attr_vbus);
	cancel_work_sync(&twl->set_vbus_work);
	flush_workqueue(twl->work_queue);
	destroy_workqueue(twl->work_queue);
	wake_lock_destroy(&twl->usb_lock);
	kfree(twl);
	twl6030_priv = NULL;

	return 0;
}

static struct platform_driver twl6030_usb_driver = {
	.probe		= twl6030_usb_probe,
	.remove		= __exit_p(twl6030_usb_remove),
	.driver		= {
		.name	= "twl6030_usb",
		.owner	= THIS_MODULE,
	},
};

static int __init twl6030_usb_init(void)
{
	return platform_driver_register(&twl6030_usb_driver);
}
subsys_initcall(twl6030_usb_init);

static void __exit twl6030_usb_exit(void)
{
	platform_driver_unregister(&twl6030_usb_driver);
}
module_exit(twl6030_usb_exit);

MODULE_ALIAS("platform:twl6030_usb");
MODULE_AUTHOR("Hema HK <hemahk@ti.com>");
MODULE_DESCRIPTION("TWL6030 USB transceiver driver");
MODULE_LICENSE("GPL");
