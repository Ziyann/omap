/*
 * MMC definitions for OMAP2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct mmc_card;
struct mmc_platform_data;

struct omap2_hsmmc_info {
	u8	mmc;		/* controller 1/2/3 */
	u32	caps;		/* 4/8 wires and any additional host
				 * capabilities OR'd (ref. linux/mmc/host.h) */
	u32 caps2;
	u32	pm_caps;	/* PM capabilities */
	bool	transceiver;	/* MMC-2 option */
	bool	ext_clock;	/* use external pin for input clock */
	bool	cover_only;	/* No card detect - just cover switch */
	bool	nonremovable;	/* Nonremovable e.g. eMMC */
	bool	power_saving;	/* Try to sleep or power off when possible */
	bool	no_off;		/* power_saving and power is not to go off */
	bool	no_off_init;	/* no power off when not in MMC sleep state */
	bool	vcc_aux_disable_is_sleep; /* Regulator off remapped to sleep */
	bool	deferred;	/* mmc needs a deferred probe */
	int	gpio_cd;	/* or -EINVAL */
	int	gpio_wp;	/* or -EINVAL */
	char	*name;		/* or NULL for default */
	struct platform_device *pdev;	/* mmc controller instance */
	int	ocr_mask;	/* temporary HACK */
	/* built_in: Use this flag to keep power to MMC/SDIO card
	 * during suspend in case the driver for the card
	 * (Eg: SDIO/Wifi driver) has not yet been loaded
	 */
	int	built_in;
	/* Maximum clock, if constrained by external circuitly,
	 * or 0 for default
	 */
	unsigned long	max_freq;
	/* Remux (pad configuration) when powering on/off */
	void (*remux)(struct device *dev, int slot, int power_on);
	/* init some special card */
	void (*init_card)(struct mmc_card *card);
	struct mmc_platform_data *mmc_data;
	int housekeeping_ms;	/* [ms] Card require housekeeping */
};

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

void omap_hsmmc_init(struct omap2_hsmmc_info *);
void omap_hsmmc_late_init(struct omap2_hsmmc_info *);

#else

static inline void omap_hsmmc_init(struct omap2_hsmmc_info *info)
{
}

static inline void omap_hsmmc_late_init(struct omap2_hsmmc_info *info)
{
}

#endif
