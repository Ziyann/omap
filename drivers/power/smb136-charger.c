/*
 *  Basic SMB136 Battery Charger Driver
 *
 *  Copyright (C) 2016 OMAP4 AOSP Project
 *  Copyright (C) 2011 Samsung Electronics
 *  Ikkeun Kim <iks.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/smb136-charger.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/battery.h>

/* SMB136 Registers. */
#define SMB136_CHARGE_CURRENT			0x00
#define SMB136_INPUT_CURRENTLIMIT		0x01
#define SMB136_FLOAT_VOLTAGE			0x02
#define SMB136_CHARGE_CONTROL_A			0x03
#define SMB136_CHARGE_CONTROL_B			0x04
#define SMB136_PIN_ENABLE_CONTROL		0x05
#define SMB136_OTG_CONTROL			0x06
#define SMB136_SAFTY				0x09

#define SMB136_COMMAND_A			0x31
#define SMB136_STATUS_D				0x35
#define SMB136_STATUS_E				0x36

struct smb136_charger {
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	bool			mains_online;
	bool			usb_online;
	bool			usb_hc_mode;
	const struct smb136_charger_platform_data *pdata;
};

static int smb136_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	if (!client)
		return -ENODEV;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb136_hw_init(struct smb136_charger *smb)
{
	/* Change USB5/1/HC Control from Pin to I2C */
	smb136_i2c_write(smb->client, SMB136_PIN_ENABLE_CONTROL, 0x8);

	/* Disable Automatic Input Current Limit */
	/* Set it to 1.3A */
	smb136_i2c_write(smb->client, SMB136_INPUT_CURRENTLIMIT, 0xE6);

	/* Automatic Recharge Disabed */
	smb136_i2c_write(smb->client, SMB136_CHARGE_CONTROL_A, 0x8c);

	/* Safty timer Disabled */
	smb136_i2c_write(smb->client, SMB136_CHARGE_CONTROL_B, 0x28);

	/* Disable USB D+/D- Detection */
	smb136_i2c_write(smb->client, SMB136_OTG_CONTROL, 0x28);

	/* Set Output Polarity for STAT */
	smb136_i2c_write(smb->client, SMB136_FLOAT_VOLTAGE, 0xCA);

	/* Re-load Enable */
	smb136_i2c_write(smb->client, SMB136_SAFTY, 0x4b);

	return 0;
}

static int smb136_update_charger(struct smb136_charger *smb)
{
	if (smb->usb_hc_mode) {
		/* HC mode */
		smb136_i2c_write(smb->client, SMB136_COMMAND_A, 0x8c);

		/* Set charging current limit to 1.5A */
		smb136_i2c_write(smb->client, SMB136_CHARGE_CURRENT, 0xF4);

		dev_info(&smb->client->dev,
			"charging current limit set to 1.5A\n");
	} else if (smb->usb_online) {
		/* USBIN 500mA mode */
		smb136_i2c_write(smb->client, SMB136_COMMAND_A, 0x88);

		/* Set charging current limit to 500mA */
		smb136_i2c_write(smb->client, SMB136_CHARGE_CURRENT, 0x14);
		
		dev_info(&smb->client->dev,
			"charging current limit set to 0.5A\n");
	} else {
		/* USB 100mA Mode, USB5/1 Current Levels */
		/* Prevent in-rush current */
		smb136_i2c_write(smb->client, SMB136_COMMAND_A, 0x80);
		udelay(10);

		/* Set charge current to 100mA */
		/* Prevent in-rush current */
		smb136_i2c_write(smb->client, SMB136_CHARGE_CURRENT, 0x14);
		udelay(10);
	}
}

static int smb136_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb136_charger *smb =
		container_of(psy, struct smb136_charger, mains);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smb->mains_online;
		return 0;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static int smb136_mains_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	struct smb136_charger *smb =
		container_of(psy, struct smb136_charger, mains);
	bool oldval;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		oldval = smb->mains_online;

		smb->mains_online = val->intval;

		if (smb->mains_online != oldval)
			power_supply_changed(psy);
		return 0;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int smb136_mains_property_is_writeable(struct power_supply *psy,
					     enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property smb136_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb136_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	struct smb136_charger *smb =
		container_of(psy, struct smb136_charger, usb);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smb->usb_online;
		return 0;

	case POWER_SUPPLY_PROP_USB_HC:
		val->intval = smb->usb_hc_mode;
		return 0;

	default:
		break;
	}
	return -EINVAL;
}

static int smb136_usb_set_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   const union power_supply_propval *val)
{
	int ret = -EINVAL;
	struct smb136_charger *smb =
		container_of(psy, struct smb136_charger, usb);
	bool oldval;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		oldval = smb->usb_online;
		smb->usb_online = val->intval;

		if (smb->usb_online != oldval)
			power_supply_changed(psy);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_USB_HC:
		smb->usb_hc_mode = val->intval;
		break;
	default:
		break;
	}

	smb136_update_charger(smb);

	return ret;
}

static int smb136_usb_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_USB_HC:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property smb136_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_HC,
};

static int smb136_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct smb136_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb136_charger *smb;
	int ret;

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	smb136_hw_init(smb);

	smb->client = client;
	smb->pdata = pdata;

	smb->mains.name = "smb136-mains";
	smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
	smb->mains.get_property = smb136_mains_get_property;
	smb->mains.set_property = smb136_mains_set_property;
	smb->mains.property_is_writeable = smb136_mains_property_is_writeable;
	smb->mains.properties = smb136_mains_properties;
	smb->mains.num_properties = ARRAY_SIZE(smb136_mains_properties);

	smb->usb.name = "smb136-usb";
	smb->usb.type = POWER_SUPPLY_TYPE_USB;
	smb->usb.get_property = smb136_usb_get_property;
	smb->usb.set_property = smb136_usb_set_property;
	smb->usb.property_is_writeable = smb136_usb_property_is_writeable;
	smb->usb.properties = smb136_usb_properties;
	smb->usb.num_properties = ARRAY_SIZE(smb136_usb_properties);

	if (pdata->supplied_to) {
		smb->mains.supplied_to = pdata->supplied_to;
		smb->mains.num_supplicants = pdata->num_supplicants;
		smb->usb.supplied_to = pdata->supplied_to;
		smb->usb.num_supplicants = pdata->num_supplicants;
	}

	ret = power_supply_register(dev, &smb->mains);
	if (ret < 0)
		return ret;

	ret = power_supply_register(dev, &smb->usb);
	if (ret < 0) {
		power_supply_unregister(&smb->mains);
		return ret;
	}

	dev_dbg(&client->dev, "probed\n");

	return 0;
}

static int __devexit smb136_remove(struct i2c_client *client)
{
	struct smb136_charger *smb = i2c_get_clientdata(client);

	power_supply_unregister(&smb->usb);
	power_supply_unregister(&smb->mains);

	return 0;
}

static const struct i2c_device_id smb136_id[] = {
	{ "smb136", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb136_id);

static struct i2c_driver smb136_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "smb136",
	},
	.id_table	= smb136_id,
	.probe	= smb136_probe,
	.remove	= __devexit_p(smb136_remove),
	.command = NULL,
};

static int __init smb136_init(void)
{
	return i2c_add_driver(&smb136_i2c_driver);
}
module_init(smb136_init);

static void __exit smb136_exit(void)
{
	i2c_del_driver(&smb136_i2c_driver);
}
module_exit(smb136_exit);

MODULE_DESCRIPTION("SMB136 battery charger driver");
MODULE_LICENSE("GPL");
