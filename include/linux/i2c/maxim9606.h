/*
 * Maxim 9606 PMIC header.
 *
 * Copyright (C) Barnes & Noble Inc. 2012
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MAXIM9606_H
#define _MAXIM9606_H

struct maxim9606_platform_data {
	int (*request_resources)(struct device *dev);
	int (*release_resources)(struct device *dev);
	int (*power_on)(struct device *dev);
	int (*power_off)(struct device *dev);
};

#endif /* _MAXIM9606_H */
