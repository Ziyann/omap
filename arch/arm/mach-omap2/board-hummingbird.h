/*
 * arch/arm/mach-omap2/board-hummingbird.h
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_OMAP_BOARD_HUMMINGBIRD_H
#define _MACH_OMAP_BOARD_HUMMINGBIRD_H

#define HUMMINGBIRD_EVT0		0x0
#define HUMMINGBIRD_EVT0B		0x1
#define HUMMINGBIRD_EVT1		0x2
#define HUMMINGBIRD_EVT2		0x3
#define HUMMINGBIRD_DVT			0x4
#define HUMMINGBIRD_PVT			0x5

int hummingbird_touch_init(void);
int hummingbird_sensor_init(void);
void omap4_create_board_props(void);
int hummingbird_panel_init(void);
int hummingbird_button_init(void);
struct omap_ion_platform_data;
void hummingbird_android_display_setup(struct omap_ion_platform_data *ion);
void bn_power_init(void);
void bn_wilink_init(void);

#endif
