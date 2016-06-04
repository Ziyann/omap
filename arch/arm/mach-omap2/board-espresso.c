/* arch/arm/mach-omap2/board-espresso.c
 *
 * Copyright (C) 2011 Samsung Electronics Co, Ltd.
 *
 * Based on mach-omap2/board-espresso.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/memblock.h>
#include <linux/omap_ion.h>
#include <linux/reboot.h>
#include <linux/sysfs.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/cpu.h>
#include <plat/drm.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/remoteproc.h>
#include <plat/usb.h>

#include <mach/id.h>

#include <asm/hardware/gic.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <asm/system_info.h>
#include <asm/system_misc.h>

#include "mach/omap-secure.h"

#include "board-espresso.h"
#include "common.h"
#include "control.h"
#include "mux.h"
#include "omap_ram_console.h"
#include "omap4_ion.h"
#include "omap4-sar-layout.h"

#include "sec_muxtbl.h"

/* gpio to distinguish WiFi and USA-BBY (P51xx)
 *
 * HW_REV4 | HIGH | LOW
 * --------+------+------
 *         |IrDA O|IrDA X
 */
#define GPIO_HW_REV4		41

#define GPIO_TA_NCONNECTED	32

#define ESPRESSO_MEM_BANK_0_SIZE	0x20000000
#define ESPRESSO_MEM_BANK_0_ADDR	0x80000000
#define ESPRESSO_MEM_BANK_1_SIZE	0x20000000
#define ESPRESSO_MEM_BANK_1_ADDR	0xA0000000

#define OMAP_SW_BOOT_CFG_ADDR	0x4A326FF8
#define REBOOT_FLAG_NORMAL	(1 << 0)
#define REBOOT_FLAG_RECOVERY	(1 << 1)
#define REBOOT_FLAG_POWER_OFF	(1 << 4)
#define REBOOT_FLAG_DOWNLOAD	(1 << 5)

#define ESPRESSO_ATTR_RO(_type, _name, _show) \
	struct kobj_attribute espresso_##_type##_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static struct platform_device bcm4330_bluetooth_device = {
	.name		= "bcm4330_bluetooth",
	.id		= -1,
};

static struct platform_device *espresso_devices[] __initdata = {
	&bcm4330_bluetooth_device,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#else
	.mode		= MUSB_PERIPHERAL,
#endif
	.power		= 500,
};

/* Board identification */

static bool _board_has_modem = true;
static bool _board_is_espresso10 = true;
static bool _board_is_bestbuy_variant = false;

/*
 * Sets the board type
 */
static __init int setup_board_type(char *str)
{
	int lcd_id;

	/* We reset the console loglevel here back to verbose, as our
	 * bootloaders pass loglevel=0 to the kernel cmdline.
	 * This is the most convinient place to do so, as this method
	 * gets executed right after parsing the loglevel param.
	 */
	console_loglevel = 15;

	if (kstrtoint(str, 0, &lcd_id)) {
		pr_err("************************************************\n");
		pr_err("Cannot parse lcd_panel_id command line parameter\n");
		pr_err("Failed to detect board type, assuming espresso10\n");
		pr_err("************************************************\n");
		return 1;
	}

	/*
	 * P51xx bootloaders pass lcd_id=1 and on some older lcd_id=0,
	 * everything else is P31xx.
	 */
	if (lcd_id > 1)
		_board_is_espresso10 = false;

	return 0;
}
early_param("lcd_panel_id", setup_board_type);

/*
 * Sets whether the device is a wifi-only variant
 */
static int __init espresso_set_subtype(char *str)
{
	#define CARRIER_WIFI_ONLY "wifi-only"

	if (!strncmp(str, CARRIER_WIFI_ONLY, strlen(CARRIER_WIFI_ONLY)))
		_board_has_modem = false;

	return 0;
}
__setup("androidboot.carrier=", espresso_set_subtype);

/*
 * Sets whether the device is a Best Buy wifi-only variant
 */
static int __init espresso_set_vendor_type(char *str)
{
	unsigned int vendor;

	if (kstrtouint(str, 0, &vendor))
		return 0;

	if (vendor == 0)
		_board_is_bestbuy_variant = true;

	return 0;
}
__setup("sec_vendor=", espresso_set_vendor_type);

bool board_is_espresso10(void) {
	return _board_is_espresso10;
}

bool board_has_modem(void) {
	return _board_has_modem;
}

bool board_is_bestbuy_variant(void) {
	return _board_is_bestbuy_variant;
}

/* Board identification end */

static void espresso_power_off_charger(void)
{
	pr_err("Rebooting into bootloader for charger.\n");
	arm_pm_restart('t', NULL);
}

static int espresso_reboot_call(struct notifier_block *this,
				unsigned long code, void *cmd)
{
	u32 flag = REBOOT_FLAG_NORMAL;
	char *blcmd = "RESET";

	if (code == SYS_POWER_OFF) {
		flag = REBOOT_FLAG_POWER_OFF;
		blcmd = "POFF";
		if (!gpio_get_value(GPIO_TA_NCONNECTED))
			pm_power_off = espresso_power_off_charger;
	} else if (code == SYS_RESTART) {
		if (cmd) {
			if (!strcmp(cmd, "recovery"))
				flag = REBOOT_FLAG_RECOVERY;
			else if (!strcmp(cmd, "download"))
				flag = REBOOT_FLAG_DOWNLOAD;
		}
	}

	omap_writel(flag, OMAP_SW_BOOT_CFG_ADDR);
	omap_writel(*(u32 *) blcmd, OMAP_SW_BOOT_CFG_ADDR - 0x04);

	return NOTIFY_DONE;
}

static struct notifier_block espresso_reboot_notifier = {
	.notifier_call = espresso_reboot_call,
};

static void __init espresso10_update_board_type(void)
{
	/* because omap4_mux_init is not called when this function is
	 * called, padconf reg must be configured by low-level function. */
	omap_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT,
		    OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE +
		    OMAP4_CTRL_MODULE_PAD_GPMC_A17_OFFSET);

	gpio_request(GPIO_HW_REV4, "HW_REV4");
	if (gpio_get_value(GPIO_HW_REV4))
		_board_is_bestbuy_variant = true;
}

#define ESPRESSO_ATTR_RO(_type, _name, _show) \
	struct kobj_attribute espresso_##_type##_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static ssize_t espresso_board_type_show(struct kobject *kobj,
	 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "espresso%s%s", board_is_espresso10() ? "10" : "",
					board_has_modem() ? "" : "wifi");
}

static ESPRESSO_ATTR_RO(board, type, espresso_board_type_show);
static struct attribute *espresso_board_prop_attrs[] = {
	&espresso_board_prop_attr_type.attr,
	NULL,
};

static struct attribute_group espresso_board_prop_attr_group = {
	.attrs = espresso_board_prop_attrs,
};

static void __init omap4_espresso_create_board_props(void)
{
	struct kobject *board_kobj;
	int ret = 0;

	board_kobj = kobject_create_and_add("board", NULL);
	if (!board_kobj)
		goto err_board_obj;

	ret = sysfs_create_group(board_kobj, &espresso_board_prop_attr_group);
	if (ret)
		goto err_board_sysfs_create;

	return;

err_board_sysfs_create:
	kobject_put(board_kobj);
err_board_obj:
	if (!board_kobj || ret)
		pr_err("failed to create espresso board properties\n");
}

static void __init sec_common_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create class (sec)!\n");
}

static void __init espresso_init(void)
{
	omap4_mux_init(NULL, NULL, OMAP_PACKAGE_CBS);

	omap4_espresso_emif_init();

	if (board_is_espresso10()) {
		espresso10_update_board_type();
		if (board_is_bestbuy_variant() && system_rev >= 7)
			sec_muxtbl_init(SEC_MACHINE_ESPRESSO10_USA_BBY, system_rev);
		sec_muxtbl_init(SEC_MACHINE_ESPRESSO10, system_rev);
	} else
		sec_muxtbl_init(SEC_MACHINE_ESPRESSO, system_rev);

	register_reboot_notifier(&espresso_reboot_notifier);

	/* initialize sec common infrastructures */
	sec_common_init();

	/* initialize board props */
	omap_create_board_props();
	omap4_espresso_create_board_props();

	/* initialize each drivers */
	omap4_espresso_serial_init();
	omap4_espresso_pmic_init();
	omap_sdrc_init(NULL, NULL);
	omap4_espresso_charger_init();
#ifdef CONFIG_ION_OMAP
	omap4_register_ion();
#endif
	platform_add_devices(espresso_devices, ARRAY_SIZE(espresso_devices));
	omap_init_dmm_tiler();
	omap4_espresso_sdio_init();
	usb_musb_init(&musb_board_data);
	omap4_espresso_connector_init();
	omap4_espresso_display_init();
	omap4_espresso_input_init();
	omap4_espresso_wifi_init();
	omap4_espresso_sensors_init();
	omap4_espresso_jack_init();
	omap4_espresso_modem_init();
}

static void __init espresso_reserve(void)
{
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);
	omap4_espresso_memory_display_init();
	omap4_ion_init();
	omap4_secure_workspace_addr_default();
	omap_reserve();
}

MACHINE_START(OMAP4_ESPRESSO, "OMAP4 Espresso board")
	/* Maintainer: Daniel Jarai */
	.atag_offset	= 0x100,
	.reserve	= espresso_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= espresso_init,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
