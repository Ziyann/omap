/* B&N common wifi board file.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wakelock.h>
#include <linux/wl12xx.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/delay.h>
#include <plat/omap-serial.h>
#include <plat/gpio.h>
#include "board-ovation.h"
#include "mux.h"

#define WILINK_UART_DEV_NAME	"/dev/ttyO3"

#define GPIO_WIFI_PWEN		114

#define GPIO_WIFI_IRQ_EVT0B	115
#define GPIO_WIFI_PMENA_EVT0B	118

#define GPIO_WIFI_IRQ_EVT0C	147
#define GPIO_WIFI_PMENA_EVT0C	115

#define GPIO_WIFI_IRQ_EVT1A	148
#define GPIO_WIFI_PMENA_EVT1A	115

static struct regulator_consumer_supply vmmc3_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.2"),
};

static struct regulator_init_data vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vmmc3_supply),
	.consumer_supplies	= vmmc3_supply,
};

static struct fixed_voltage_config vwlan_pdata = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio 			= -EINVAL,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &vmmc3,
};

static struct platform_device vwlan_device = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev = {
		.platform_data = &vwlan_pdata,
	},
};

static struct regulator_consumer_supply wlan_vbat_supply[] = {
	REGULATOR_SUPPLY("wlan-vbat", NULL),
};

static struct regulator_init_data wlan_vbat = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wlan_vbat_supply),
	.consumer_supplies	= wlan_vbat_supply,
};

static struct fixed_voltage_config vsys_wlan_pdata = {
	.supply_name		= "vsys-wlan",
	.microvolts		= 3875000, /* follows VSYS 3.50-4.25V */
	.gpio 			= GPIO_WIFI_PWEN,
	.startup_delay		= 200, /* 200usec */
	.enable_high		= 1,
	.enabled_at_boot	= 1,
	.init_data		= &wlan_vbat,
};

static struct platform_device vsys_wlan_device = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev = {
		.platform_data = &vsys_wlan_pdata,
	},
};

/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is
 * capable enough to wake-up the OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

void bn_wilink_set_power(bool enable)
{
	static struct regulator *wl12xx_clk_32k_in = NULL;
	static struct regulator *wl12xx_vbat = NULL;
	static struct regulator *wl12xx_vio = NULL;

	if (IS_ERR_OR_NULL(wl12xx_clk_32k_in))
		wl12xx_clk_32k_in = regulator_get(NULL, "clk32kg");

	if (IS_ERR_OR_NULL(wl12xx_vio))
		wl12xx_vio = regulator_get(NULL, "wlan-vio");

	if (IS_ERR_OR_NULL(wl12xx_vbat))
		wl12xx_vbat = regulator_get(NULL, "wlan-vbat");

	if (!wl12xx_clk_32k_in || !wl12xx_vbat || !wl12xx_vio) {
		pr_err("Failed to get wl12xx regulators\n");
		return;
	}

	if (enable) {
		pr_info("Enable wl12xx power\n");
		regulator_enable(wl12xx_vbat);
		regulator_enable(wl12xx_vio);
		regulator_enable(wl12xx_clk_32k_in);
		mdelay(100);
	} else {
		regulator_disable(wl12xx_clk_32k_in);
		regulator_disable(wl12xx_vio);
		regulator_disable(wl12xx_vbat);
		pr_info("Disable wl12xx power\n");
	}
}

static int plat_chip_enable(void)
{
	bn_wilink_set_power(true);
	return plat_uart_enable();
}

static int plat_chip_disable(void)
{
	bn_wilink_set_power(false);
	return plat_uart_disable();
}

/* wl2xx WiFi platform data */
static struct wl12xx_platform_data wl12xx_pdata = {
	.irq = -EINVAL,
	.board_ref_clock = WL12XX_REFCLOCK_38,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_38_4,
	.set_power = bn_wilink_set_power,
};

/* wl12xx BT platform data */
static struct ti_st_plat_data ti_st_pdata = {
	.nshutdown_gpio = 83,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_chip_enable,
	.chip_disable = plat_chip_disable,
};

static struct platform_device ti_st_device = {
	.name		= "kim",
	.id		= -1,
	.dev = {
		.platform_data = &ti_st_pdata,
	},
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static void __init bn_wilink_mux_init(int gpio_irq, int gpio_pmena)
{
	omap_mux_init_gpio(gpio_irq, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(gpio_pmena, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_WIFI_PWEN, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("uart2_cts.sdmmc3_clk",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rts.sdmmc3_cmd",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rx.sdmmc3_dat0",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_tx.sdmmc3_dat1",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("abe_mcbsp1_dx.sdmmc3_dat2",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("abe_mcbsp1_fsx.sdmmc3_dat3",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
}

void __init bn_wilink_init(void)
{
#if defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
	pr_info("Using Hummingbird wifi configuration\n");
	bn_wilink_mux_init(GPIO_WIFI_IRQ_EVT1A, GPIO_WIFI_PMENA_EVT1A);
	vwlan_pdata.gpio = GPIO_WIFI_PMENA_EVT1A;
	wl12xx_pdata.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ_EVT1A);
#else
	switch (system_rev) {
	case OVATION_EVT0:
	case OVATION_EVT0B:
		pr_info("Using Ovation ETV0B wifi configuration\n");
		bn_wilink_mux_init(GPIO_WIFI_IRQ_EVT0B, GPIO_WIFI_PMENA_EVT0B);
		vwlan_pdata.gpio = GPIO_WIFI_PMENA_EVT0B;
		wl12xx_pdata.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ_EVT0B);
		break;
	case OVATION_EVT0C:
		pr_info("Using Ovation ETV0C wifi configuration\n");
		bn_wilink_mux_init(GPIO_WIFI_IRQ_EVT0C, GPIO_WIFI_PMENA_EVT0C);
		vwlan_pdata.gpio = GPIO_WIFI_PMENA_EVT0C;
		wl12xx_pdata.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ_EVT0C);
		break;
	case OVATION_EVT1A:
	default:
		pr_info("Using Ovation ETV1A wifi configuration\n");
		bn_wilink_mux_init(GPIO_WIFI_IRQ_EVT1A, GPIO_WIFI_PMENA_EVT1A);
		vwlan_pdata.gpio = GPIO_WIFI_PMENA_EVT1A;
		wl12xx_pdata.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ_EVT1A);
		break;
	}
#endif

	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");

	if (wl12xx_set_platform_data(&wl12xx_pdata)) {
		pr_err("Error setting wl12xx data\n");
		return;
	}

	platform_device_register(&vwlan_device);
	platform_device_register(&vsys_wlan_device);
	platform_device_register(&ti_st_device);
	platform_device_register(&btwilink_device);
}
