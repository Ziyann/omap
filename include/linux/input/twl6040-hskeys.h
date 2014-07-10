/*
 * Headset key driver for TWL6040
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Author: Ivaylo Spasov <ispasov@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_TWL6040_HSKEYS_H
#define _LINUX_TWL6040_HSKEYS_H

struct hskeys_key_data {
	int key;
	int max_ref_volt;
	int min_ref_volt;
};

struct hskeys_keymap_data {
	struct hskeys_key_data *keymap;
	size_t keymap_size;
};

#endif /* _LINUX_TWL6040_HSKEYS_H */
