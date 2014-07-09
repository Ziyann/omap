/*
 * arch/arm/mach-omap2/board-ovation.h
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

#ifndef _MACH_OMAP_BOARD_OVATION_H
#define _MACH_OMAP_BOARD_OVATION_H

#define OVATION_EVT0		0x0
#define OVATION_EVT0B		0x1
#define OVATION_EVT0C		0x2
#define OVATION_EVT1A		0x3
#define OVATION_EVT1B		0x4
#define OVATION_EVT2		0x5
#define OVATION_DVT		0x6
#define OVATION_PVT		0x7

int ovation_touch_init(void);
int ovation_sensor_init(void);
void omap4_create_board_props(void);
int ovation_panel_init(void);
int ovation_button_init(void);
void bn_power_init(void);
void bn_wilink_init(void);
struct omap_ion_platform_data;
void ovation_android_display_setup(struct omap_ion_platform_data *ion);

#endif
