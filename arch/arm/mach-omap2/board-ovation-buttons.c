/*
 * Button and Button LED support OMAP44xx ovation.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>

#include "mux.h"
#include "board-ovation.h"

static int ovation_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data ovation_keymap_data = {
	.keymap                 = ovation_keymap,
	.keymap_size            = ARRAY_SIZE(ovation_keymap),
};


static struct omap4_keypad_platform_data ovation_keypad_data = {
	.keymap_data            = &ovation_keymap_data,
	.rows                   = 2,
	.cols                   = 1,
};

void keyboard_mux_init(void)
{
	/* Column mode */
	omap_mux_init_signal("kpd_col0.kpd_col0",
			OMAP_WAKEUP_EN | OMAP_MUX_MODE0);
	/* Row mode */
	omap_mux_init_signal("kpd_row0.kpd_row0",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE0 |
			OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row1.kpd_row1",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE0 |
			OMAP_INPUT_EN);
}


static struct gpio_led ovation_gpio_leds[] = {
	{
		.name	= "omap4:green:debug0",
		.gpio	= 82,
	},

};

static struct gpio_led_platform_data ovation_led_data = {
	.leds	= ovation_gpio_leds,
	.num_leds = ARRAY_SIZE(ovation_gpio_leds),
};

static struct platform_device ovation_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &ovation_led_data,
	},
};

/* GPIO_KEY for Tablet */
static struct gpio_keys_button ovation_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_HOME,
		.gpio			= 32,
		.desc			= "SW1",
		.active_low		= 1,
		.wakeup			= 1,
	},
	[1] = {
		.code                   = SW_LID,
		.type                   = EV_SW,
		.gpio                   = 31,
		.desc                   = "HALL",
		.active_low             = 1,
		.wakeup                 = 1,
		.debounce_interval      = 5,
	},
};

void gpio_key_buttons_mux_init(void)
{
	/* Hall sensor */
	omap_mux_init_gpio(31, OMAP_PIN_INPUT |
			OMAP_PIN_OFF_WAKEUPENABLE);
}

static struct gpio_keys_platform_data ovation_gpio_keys = {
	.buttons		= ovation_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(ovation_gpio_keys_buttons),
	.rep			= 0,
};

static struct platform_device ovation_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &ovation_gpio_keys,
	},
};

static struct platform_device *ovation_devices[] __initdata = {
	&ovation_leds_gpio,
	&ovation_gpio_keys_device,
};

int __init ovation_button_init(void)
{
	int status;
	platform_add_devices(ovation_devices, ARRAY_SIZE(ovation_devices));
	keyboard_mux_init();
	gpio_key_buttons_mux_init();
	status = omap4_keyboard_init(&ovation_keypad_data);

	if (status)
		pr_err("Keypad initialization failed: %d\n", status);
	return 0;

}
