/*
 *  This file contains work-arounds for many known SD/MMC
 *  and SDIO hardware bugs.
 *
 *  Copyright (c) 2011 Andrei Warkentin <andreiw@motorola.com>
 *  Copyright (c) 2011 Pierre Tardy <tardyp@gmail.com>
 *  Inspired from pci fixup code:
 *  Copyright (c) 1999 Martin Mares <mj@ucw.cz>
 *
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/scatterlist.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>

#include "mmc_ops.h"

#ifndef SDIO_VENDOR_ID_TI
#define SDIO_VENDOR_ID_TI		0x0097
#endif

#ifndef SDIO_DEVICE_ID_TI_WL1271
#define SDIO_DEVICE_ID_TI_WL1271	0x4076
#endif

/*
 * This hook just adds a quirk for all sdio devices
 */
static void add_quirk_for_sdio_devices(struct mmc_card *card, int data)
{
	if (mmc_card_sdio(card))
		card->quirks |= data;
}

static const struct mmc_fixup mmc_fixup_methods[] = {
	/* by default sdio devices are considered CLK_GATING broken */
	/* good cards will be whitelisted as they are tested */
	SDIO_FIXUP(SDIO_ANY_ID, SDIO_ANY_ID,
		   add_quirk_for_sdio_devices,
		   MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL1271,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL1271,
		   add_quirk, MMC_QUIRK_NONSTD_FUNC_IF),

	SDIO_FIXUP(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL1271,
		   add_quirk, MMC_QUIRK_DISABLE_CD),

	END_FIXUP
};

void mmc_fixup_device(struct mmc_card *card, const struct mmc_fixup *table)
{
	const struct mmc_fixup *f;
	u64 rev = cid_rev_card(card);

	/* Non-core specific workarounds. */
	if (!table)
		table = mmc_fixup_methods;

	for (f = table; f->vendor_fixup; f++) {
		if ((f->manfid == CID_MANFID_ANY ||
		     f->manfid == card->cid.manfid) &&
		    (f->oemid == CID_OEMID_ANY ||
		     f->oemid == card->cid.oemid) &&
		    (f->name == CID_NAME_ANY ||
		     !strncmp(f->name, card->cid.prod_name,
			      sizeof(card->cid.prod_name))) &&
		    (f->cis_vendor == card->cis.vendor ||
		     f->cis_vendor == (u16) SDIO_ANY_ID) &&
		    (f->cis_device == card->cis.device ||
		     f->cis_device == (u16) SDIO_ANY_ID) &&
		    rev >= f->rev_start && rev <= f->rev_end) {
			dev_dbg(&card->dev, "calling %pF\n", f->vendor_fixup);
			f->vendor_fixup(card, f->data);
		}
	}
}
EXPORT_SYMBOL(mmc_fixup_device);

/* GCC 4.8 and above mangle the eMMC firmware patching code... */
#if __GNUC__ > 4 || ( __GNUC__ == 4 && __GNUC_MINOR__ >= 8 )
#pragma GCC push_options
/* As a workaround, drop the optimization level */
#pragma GCC optimize ("O1")
#endif

/*
 * Quirk code to fix bug in wear leveling firmware for certain Samsung emmc
 * chips
 */
static int mmc_movi_vendor_cmd(struct mmc_card *card, unsigned int arg)
{
	struct mmc_command cmd = {0};
	int err;
	u32 status;

	/* CMD62 is vendor CMD, it's not defined in eMMC spec. */
	cmd.opcode = 62;
	cmd.flags = MMC_RSP_R1B;
	cmd.arg = arg;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return err;

	do {
		err = mmc_send_status(card, &status);
		if (err)
			return err;
		if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
			break;
		if (mmc_host_is_spi(card->host))
			break;
	} while (R1_CURRENT_STATE(status) == R1_STATE_PRG);

	return err;
}

static int mmc_movi_erase_cmd(struct mmc_card *card,
			unsigned int arg1, unsigned int arg2)
{
	struct mmc_command cmd = {0};
	int err;
	u32 status;

	cmd.opcode = MMC_ERASE_GROUP_START;
	cmd.flags = MMC_RSP_R1;
	cmd.arg = arg1;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE_GROUP_END;
	cmd.flags = MMC_RSP_R1;
	cmd.arg = arg2;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE;
	cmd.flags = MMC_RSP_R1B;
	cmd.arg = 0x00000000;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return err;

	do {
		err = mmc_send_status(card, &status);
		if (err)
			return err;
		if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
			break;
		if (mmc_host_is_spi(card->host))
			break;
	} while (R1_CURRENT_STATE(status) == R1_STATE_PRG);

	return err;
}

#define TEST_MMC_FW_PATCHING

#if defined(CONFIG_MMC_SAMSUNG_SMART) || defined(TEST_MMC_FW_PATCHING)
static struct mmc_command wcmd;
static struct mmc_data wdata;

static int mmc_movi_read_cmd(struct mmc_card *card, u8 *buffer)
{
	struct mmc_request brq;
	struct scatterlist sg;

	brq.cmd = &wcmd;
	brq.data = &wdata;

	wcmd.opcode = MMC_READ_SINGLE_BLOCK;
	wcmd.arg = 0;
	wcmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	wdata.blksz = 512;
	brq.stop = NULL;
	wdata.blocks = 1;
	wdata.flags = MMC_DATA_READ;

	wdata.sg = &sg;
	wdata.sg_len = 1;

	sg_init_one(&sg, buffer, 512);

	mmc_set_data_timeout(&wdata, card);

	mmc_wait_for_req(card->host, &brq);
	if (wcmd.error)
		return wcmd.error;
	if (wdata.error)
		return wdata.error;
	return 0;
}
#endif /* CONFIG_MMC_SAMSUNG_SMART || TEST_MMC_FW_PATCHING */

/*
 * Copy entire page when wear leveling is happened
 */
static int mmc_set_wearlevel_page(struct mmc_card *card)
{
	int err, errx = 0;
#ifdef TEST_MMC_FW_PATCHING
	void *buffer;
	buffer = kmalloc(512, GFP_KERNEL);
	if (!buffer) {
		pr_err("Fail to alloc memory for set wearlevel\n");
		return -ENOMEM;
	}
#endif /* TEST_MMC_FW_PATCHING */

	/* modification vendor cmd */
	/* enter vendor command mode */
	err = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (err)
		goto out;

	err = mmc_movi_vendor_cmd(card, 0x10210000);
	if (err)
		goto out;

	/* set value 0x000000FF : It's hidden data
	 * When in vendor command mode, the erase command is used to
	 * patch the firmware in the internal sram.
	 */
	err = mmc_movi_erase_cmd(card, 0x0004DD9C, 0x000000FF);
	if (err) {
		pr_err("Fail to Set WL value1\n");
		goto err_set_wl;
	}

	/* set value 0xD20228FF : It's hidden data */
	err = mmc_movi_erase_cmd(card, 0x000379A4, 0xD20228FF);
	if (err) {
		pr_err("Fail to Set WL value2\n");
		goto err_set_wl;
	}

err_set_wl:
	/* exit vendor command mode */
	errx = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (errx)
		goto out;

	errx = mmc_movi_vendor_cmd(card, 0x00DECCEE);
	if (errx || err)
		goto out;

#ifdef TEST_MMC_FW_PATCHING
	/* read and verify vendor cmd */
	/* enter vendor cmd */
	err = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (err)
		goto out;

	err = mmc_movi_vendor_cmd(card, 0x10210002);
	if (err)
		goto out;

	err = mmc_movi_erase_cmd(card, 0x0004DD9C, 0x00000004);
	if (err) {
		pr_err("Fail to Check WL value1\n");
		goto err_check_wl;
	}

	err = mmc_movi_read_cmd(card, (u8 *)buffer);
	if (err) {
		pr_err("Fail to Read value1\n");
		goto err_check_wl;
	}
	pr_debug("buffer[0] is 0x%x\n", *(u8 *)buffer);

	err = mmc_movi_erase_cmd(card, 0x000379A4, 0x00000004);
	if (err) {
		pr_err("Fail to Check WL value2\n");
		goto err_check_wl;
	}

	err = mmc_movi_read_cmd(card, (u8 *)buffer);
	if (err) {
		pr_err("Fail to Read value2\n");
		goto err_check_wl;
	}
	pr_debug("buffer[0] is 0x%x\n", *(u8 *)buffer);

err_check_wl:
	/* exit vendor cmd mode */
	errx = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (errx)
		goto out;

	errx = mmc_movi_vendor_cmd(card, 0x00DECCEE);
	if (errx || err)
		goto out;

#endif /* TEST_MMC_FW_PATCHING */

 out:
#ifdef TEST_MMC_FW_PATCHING
	kfree(buffer);
#endif /* TEST_MMC_FW_PATCHING */
	if (err)
		return err;
	return errx;
}

void mmc_fixup_samsung_fw(struct mmc_card *card)
{
	int err;

	mmc_claim_host(card->host);
	err = mmc_set_wearlevel_page(card);
	mmc_release_host(card->host);
	if (err)
		pr_err("%s : Failed to fixup Samsung emmc firmware(%d)\n",
			mmc_hostname(card->host), err);
}

#ifdef CONFIG_MMC_SAMSUNG_SMART
static int mmc_samsung_smart_read(struct mmc_card *card, u8 *rdblock)
{
	int err, errx;

	/* enter vendor Smart Report mode */
	err = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (err) {
		pr_err("Failed entering Smart Report mode(1, %d)\n", err);
		return err;
	}
	err = mmc_movi_vendor_cmd(card, 0x0000CCEE);
	if (err) {
		pr_err("Failed entering Smart Report mode(2, %d)\n", err);
		return err;
	}

	/* read Smart Report */
	err = mmc_movi_read_cmd(card, rdblock);
	if (err)
		pr_err("Failed reading Smart Report(%d)\n", err);
		/* Do NOT return yet; we must leave Smart Report mode.*/

	/* exit vendor Smart Report mode */
	errx = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (errx)
		pr_err("Failed exiting Smart Report mode(1, %d)\n", errx);
	else {
		errx = mmc_movi_vendor_cmd(card, 0x00DECCEE);
		if (errx)
			pr_err("Failed exiting Smart Report mode(2, %d)\n",
									errx);
	}
	if (err)
		return err;
	return errx;
}
ssize_t mmc_samsung_smart_parse(u32 *report, char *for_sysfs)
{
	unsigned size = PAGE_SIZE;
	unsigned wrote;
	unsigned i;
	u32 val;
	char *str;
	static const struct {
		char *fmt;
		unsigned val_index;
	} to_output[] = {
		{ "super block size              : %u\n", 1 },
		{ "super page size               : %u\n", 2 },
		{ "optimal write size            : %u\n", 3 },
		{ "read reclaim count            : %u\n", 20 },
		{ "optimal trim size             : %u\n", 21 },
		{ "number of banks               : %u\n", 4 },
		{ "initial bad blocks per bank   : %u",	  5 },
		{ ",%u",				  8 },
		{ ",%u",				  11 },
		{ ",%u\n",				  14 },
		{ "runtime bad blocks per bank   : %u",	  6 },
		{ ",%u",				  9 },
		{ ",%u",				  12 },
		{ ",%u\n",				  15 },
		{ "reserved blocks left per bank : %u",	  7 },
		{ ",%u",				  10 },
		{ ",%u",				  13 },
		{ ",%u\n",				  16 },
		{ "all erase counts (min,avg,max): %u",	  18 },
		{ ",%u",				  19 },
		{ ",%u\n",				  17 },
		{ "SLC erase counts (min,avg,max): %u",	  31 },
		{ ",%u",				  32 },
		{ ",%u\n",				  30 },
		{ "MLC erase counts (min,avg,max): %u",	  34 },
		{ ",%u",				  35 },
		{ ",%u\n",				  33 },
	};

	/* A version field just in case things change. */
	wrote = scnprintf(for_sysfs, size,
				"version                       : %u\n", 0);
	size -= wrote;
	for_sysfs += wrote;

	/* The error mode. */
	val = le32_to_cpu(report[0]);
	switch (val) {
	case 0xD2D2D2D2:
		str = "Normal";
		break;
	case 0x5C5C5C5C:
		str = "RuntimeFatalError";
		break;
	case 0xE1E1E1E1:
		str = "MetaBrokenError";
		break;
	case 0x37373737:
		str = "OpenFatalError";
		val = 0; /* Remaining data is unreliable. */
		break;
	default:
		str = "Invalid";
		val = 0; /* Remaining data is unreliable. */
		break;
	}
	wrote = scnprintf(for_sysfs, size,
				"error mode                    : %s\n", str);
	size -= wrote;
	for_sysfs += wrote;
	/* Exit if we can't rely on the remaining data. */
	if (!val)
		return PAGE_SIZE - size;

	for (i = 0; i < ARRAY_SIZE(to_output); i++) {
		wrote = scnprintf(for_sysfs, size, to_output[i].fmt,
				  le32_to_cpu(report[to_output[i].val_index]));
		size -= wrote;
		for_sysfs += wrote;
	}

	return PAGE_SIZE - size;
}
ssize_t mmc_samsung_smart_handle(struct mmc_card *card, char *buf)
{
	int err;
	u32 *buffer;
	ssize_t len;

	buffer = kmalloc(512, GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to alloc memory for Smart Report\n");
		return 0;
	}

	mmc_claim_host(card->host);
	err = mmc_samsung_smart_read(card, (u8 *)buffer);
	mmc_release_host(card->host);

	if (err)
		len = 0;
	else
		len = mmc_samsung_smart_parse(buffer, buf);

	kfree(buffer);
	return len;
}
#endif /* CONFIG_MMC_SAMSUNG_SMART */

#if __GNUC__ > 4 || ( __GNUC__ == 4 && __GNUC_MINOR__ >= 8 )
#pragma GCC pop_options
#endif
