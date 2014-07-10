/*
 * LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _LP855X_H
#define _LP855X_H

#define BL_CTL_SHFT	(0)
#define BRT_MODE_SHFT	(1)
#define BRT_MODE_MASK	(0x06)

/* Enable backlight. Only valid when BRT_MODE=10(I2C only) */
#define ENABLE_BL	(1)
#define DISABLE_BL	(0)

#define I2C_CONFIG(id)	id ## _I2C_CONFIG
#define PWM_CONFIG(id)	id ## _PWM_CONFIG

/* DEVICE CONTROL register - LP8550 */
#define LP8550_PWM_CONFIG	(LP8550_PWM_ONLY << BRT_MODE_SHFT)
#define LP8550_I2C_CONFIG	((ENABLE_BL << BL_CTL_SHFT) | \
				(LP8550_I2C_ONLY << BRT_MODE_SHFT))

/* DEVICE CONTROL register - LP8551 */
#define LP8551_PWM_CONFIG	LP8550_PWM_CONFIG
#define LP8551_I2C_CONFIG	LP8550_I2C_CONFIG

/* DEVICE CONTROL register - LP8552 */
#define LP8552_PWM_CONFIG	LP8550_PWM_CONFIG
#define LP8552_I2C_CONFIG	LP8550_I2C_CONFIG

/* DEVICE CONTROL register - LP8553 */
#define LP8553_PWM_CONFIG	LP8550_PWM_CONFIG
#define LP8553_I2C_CONFIG	LP8550_I2C_CONFIG

/* DEVICE CONTROL register - LP8556 */
#define LP8556_PWM_CONFIG	(LP8556_PWM_ONLY << BRT_MODE_SHFT)
#define LP8556_COMB1_CONFIG	(LP8556_COMBINED1 << BRT_MODE_SHFT)
#define LP8556_I2C_CONFIG	((ENABLE_BL << BL_CTL_SHFT) | \
				(LP8556_I2C_ONLY << BRT_MODE_SHFT))
#define LP8556_COMB2_CONFIG	(LP8556_COMBINED2 << BRT_MODE_SHFT)

#define LP8556_FAST_CONFIG	BIT(7) /* use it if EPROMs should be maintained
					  when exiting the low power mode */

#ifdef CONFIG_MACH_OMAP4_BOWSER
/* CONFIG register - LP8557 */
#define LP8557_PWM_STANDBY	BIT(7)
#define LP8557_PWM_FILTER	BIT(6)
#define LP8557_RELOAD_EPROM	BIT(3)	/* use it if EPROMs should be reset
					   when the backlight turns on */
#define LP8557_DISABLE_LEDS	BIT(2)
#define LP8557_PWM_CONFIG	LP8557_PWM_ONLY
#define LP8557_I2C_CONFIG	LP8557_I2C_ONLY
#define LP8557_COMB1_CONFIG	LP8557_COMBINED1
#define LP8557_COMB2_CONFIG	LP8557_COMBINED2
#endif

/* ROM area boundary */
#define EEPROM_START	(0xA0)
#define EEPROM_END	(0xA7)
#define EPROM_START	(0xA0)
#define EPROM_END	(0xAF)

enum lp855x_chip_id {
	LP8550,
	LP8551,
	LP8552,
	LP8553,
	LP8556,
#ifdef CONFIG_MACH_OMAP4_BOWSER
	LP8557,
#endif
};

enum lp855x_brightness_ctrl_mode {
	PWM_BASED = 1,
	REGISTER_BASED,
};

enum lp8550_brighntess_source {
	LP8550_PWM_ONLY,
	LP8550_I2C_ONLY = 2,
};

enum lp8551_brighntess_source {
	LP8551_PWM_ONLY = LP8550_PWM_ONLY,
	LP8551_I2C_ONLY = LP8550_I2C_ONLY,
};

enum lp8552_brighntess_source {
	LP8552_PWM_ONLY = LP8550_PWM_ONLY,
	LP8552_I2C_ONLY = LP8550_I2C_ONLY,
};

enum lp8553_brighntess_source {
	LP8553_PWM_ONLY = LP8550_PWM_ONLY,
	LP8553_I2C_ONLY = LP8550_I2C_ONLY,
};

enum lp8556_brightness_source {
	LP8556_PWM_ONLY,
	LP8556_COMBINED1,	/* pwm + i2c before the shaper block */
	LP8556_I2C_ONLY,
	LP8556_COMBINED2,	/* pwm + i2c after the shaper block */
};

#ifdef CONFIG_MACH_OMAP4_BOWSER
enum lp8557_brightness_source {
	LP8557_PWM_ONLY,
	LP8557_I2C_ONLY,
	LP8557_COMBINED1,	/* pwm + i2c after the shaper block */
	LP8557_COMBINED2,	/* pwm + i2c before the shaper block */
};
#endif

struct lp855x_pwm_data {
	void (*pwm_set_intensity) (int brightness, int max_brightness);
	int (*pwm_get_intensity) (int max_brightness);
};

struct lp855x_rom_data {
	u8 addr;
	u8 val;
};

#ifdef CONFIG_MACH_OMAP_BN
/* CFG5 8556 */
#define CFG5_REG		0xA5
#define CFG6_REG		0xA6
#define PWM_DIRECT		(0x1 << 7)

#define PS_MODE_6P6D		(0x00 << 4)
#define PS_MODE_5P5D		(0x01 << 4)
#define PS_MODE_4P4D		(0x02 << 4)
#define PS_MODE_3P3D		(0x03 << 4)
#define PS_MODE_2P2D		(0x04 << 4)
#define PS_MODE_3P6D		(0x05 << 4)
#define PS_MODE_2P6D		(0x06 << 4)
#define PS_MODE_1P6D		(0x07 << 4)

#define PWM_FREQ4808HZ		0x00
#define PWM_FREQ6010HZ		0x01
#define PWM_FREQ2712HZ		0x02
#define PWM_FREQ8414HZ		0x03
#define PWM_FREQ6916HZ		0x04
#define PWM_FREQ12020HZ		0x05
#define PWM_FREQ13222HZ		0x06
#define PWM_FREQ14424HZ		0x07
#define PWM_FREQ15626HZ		0x08
#define PWM_FREQ16828HZ		0x09
#define PWM_FREQ10830HZ		0x0A
#define PWM_FREQ19232HZ		0x0B
#define PWM_FREQ24040HZ		0x0C
#define PWM_FREQ38848HZ		0x0D
#define PWM_FREQ33656HZ		0x0E
#define PWM_FREQ38464HZ		0x0F

#define BOOST_FREQ312KHZ	0x00
#define BOOST_FREQ625KHZ	0x40
#define BOOST_FREQ1250KHZ	0x80
#endif

/**
 * struct lp855x_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @mode : brightness control by pwm or lp855x register
 * @device_control : value of DEVICE CONTROL register
 * @initial_brightness : initial value of backlight brightness
 * @pwm_data : platform specific pwm generation functions.
		Only valid when mode is PWM_BASED.
 * @load_new_rom_data :
	0 : use default configuration data
	1 : update values of eeprom or eprom registers on loading driver
 * @size_program : total size of lp855x_rom_data
 * @rom_data : list of new eeprom/eprom registers
 * @gpio_en : num of GPIO driving enable pin (CONFIG_MACH_OMAP4_BOWSER)
 */
struct lp855x_platform_data {
	char *name;
	enum lp855x_brightness_ctrl_mode mode;
	u8 device_control;
	int initial_brightness;
	int max_brightness;
	struct lp855x_pwm_data pwm_data;
	u8 load_new_rom_data;
	int size_program;
	struct lp855x_rom_data *rom_data;
#ifdef CONFIG_MACH_OMAP4_BOWSER
	int gpio_en;
	const char *regulator_name;
#endif
#ifdef CONFIG_MACH_OMAP_BN
	u8 led_setting;
	u8 boost_freq;
	u8 nonlinearity_factor;
	int (*request_resources)(struct device *dev);
	int (*release_resources)(struct device *dev);
	int (*power_on)(struct device *dev);
	int (*power_off)(struct device *dev);
#endif
};

#endif
