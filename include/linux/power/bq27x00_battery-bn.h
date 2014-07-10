#ifndef __LINUX_BQ27X00_BATTERY_H__
#define __LINUX_BQ27X00_BATTERY_H__

struct bq27x00_platform_data {
	const char *name;
	int (*read)(struct device *dev, unsigned int);
	int gpio_ce;
	int gpio_soc_int;
	int gpio_bat_low;
};
#endif
