/*
 * drivers/mmc/host/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * Authors:
 *	Syed Mohammed Khasim	<x0khasim@ti.com>
 *	Madhusudhan		<madhu.cr@ti.com>
 *	Mohit Jalori		<mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <plat/dma.h>
#include <mach/hardware.h>
#include <plat/board.h>
#include <plat/mmc.h>
#include <plat/cpu.h>
#include <linux/trapz.h> /* ACOS_MOD_ONELINE */

#define OMAP_V1V8_SIGEN_V1V8	(1 << 19)

/* OMAP HSMMC Host Controller Registers */
#define OMAP_HSMMC_SYSCONFIG	0x0010
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CSRE		0x0024
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_PSTATE	0x0124
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_AC12		0x013C
#define OMAP_HSMMC_CAPA		0x0140
#define OMAP_HSMMC_ADMA_ES	0x0154
#define OMAP_HSMMC_ADMA_SAL	0x0158
#define OMAP_HSMMC_CAPA2	0x0144
#define OMAP_HSMMC_DLL		0x0034

#define VS18			(1 << 26)
#define VS30			(1 << 25)
#define SDVS18			(0x5 << 9)
#define SDVS30			(0x6 << 9)
#define SDVS33			(0x7 << 9)
#define SDVS_MASK		0x00000E00
#define SDVSCLR			0xFFFFF1FF
#define SDVSDET			0x00000400
#define AUTOIDLE		0x1
#define SDBP			(1 << 8)
#define DTO			0xe
#define ICE			0x1
#define ICS			0x2
#define CEN			(1 << 2)
#define CLKD_MAX		0x3FF
#define CLKD_MASK		0x0000FFC0
#define CLKD_SHIFT		6
#define DTO_MASK		0x000F0000
#define DTO_SHIFT		16
#define INT_EN_MASK		0x307F0033
#define BWR_ENABLE		(1 << 4)
#define BRR_ENABLE		(1 << 5)
#define DTO_ENABLE		(1 << 20)
#define INIT_STREAM		(1 << 1)
#define ACEN_ACMD12		(1 << 2)
#define DP_SELECT		(1 << 21)
#define DDIR			(1 << 4)
#define DMA_EN			0x1
#define MSBS			(1 << 5)
#define BCE			(1 << 1)
#define FOUR_BIT		(1 << 1)
#define DDR			(1 << 19)
#define DW8			(1 << 5)
#define BRR			(1 << 5)
#define CLKEXTFREE		(1 << 16)
#define PADEN			(1 << 15)
#define CLEV			(1 << 24)
#define DLEV			(0xF << 20)
#define CC			0x1
#define TC			0x02
#define OD			0x1
#define ERR			(1 << 15)
#define CMD_TIMEOUT		(1 << 16)
#define DATA_TIMEOUT		(1 << 20)
#define CMD_CRC			(1 << 17)
#define DATA_CRC		(1 << 21)
#define CARD_ERR		(1 << 28)
#define STAT_CLEAR		0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define DUAL_VOLT_OCR_BIT	7
#define SRC			(1 << 25)
#define SRD			(1 << 26)
#define SOFTRESET		(1 << 1)
#define RESETDONE		(1 << 0)
#define DMAS			(0x2 << 3)
#define CAPA_ADMA_SUPPORT       (1 << 19)
#define ADMA_XFER_VALID		(1 << 0)
#define ADMA_XFER_END		(1 << 1)
#define ADMA_XFER_EN_INT	(1 << 2)
#define ADMA_XFER_LINK		(1 << 4)
#define ADMA_XFER_DESC		(1 << 5)
#define DMA_MNS_ADMA_MODE	(1 << 20)
#define ADMA_ERR		(1 << 25)
#define ADMA_XFER_INT		(1 << 3)
#define AC12_SCLK_SEL		(1 << 23)
#define AC12_UHSMC_MASK		(7 << 16)
#define AC12_UHSMC_SDR50	(2 << 16)
#define AC12_UHSMC_SDR104	(3 << 16)
#define CAPA2_TSDR50		(1 << 13)
#define CAPA2_SDR104		(1 << 1)
#define DLL_LOCK		(1 << 0)
#define DLL_CALIB		(1 << 1)
#define DLL_UNLOCK_STICKY	(1 << 2)
#define DLL_SWT			(1 << 20)
#define DLL_FORCE_SR_C_MASK	(0x7F << 13)
#define DLL_FORCE_SR_C_SHIFT	13
#define DLL_FORCE_VALUE		(1 << 12)
#define DLL_RESET		(1 << 31)

/* Block Length at max can be 1024 */
#define HSMMC_BLK_SIZE		512
#define DMA_TABLE_NUM_ENTRIES	1024
#define ADMA_TABLE_SZ \
	(DMA_TABLE_NUM_ENTRIES * sizeof(struct adma_desc_table))

#define SDMA_XFER	1
#define ADMA_XFER	2
/*
 * According to TRM, It is possible to transfer
 * upto 64KB per ADMA table entry.
 * But 64KB = 0x10000 cannot be represented
 * using a 16bit integer in 1 ADMA table row.
 * Hence rounding it to a lesser value.
 */
#define ADMA_MAX_XFER_PER_ROW (60 * 1024)
#define ADMA_MAX_BLKS_PER_ROW (ADMA_MAX_XFER_PER_ROW / 512)


#define MMC_AUTOSUSPEND_DELAY	100
#define MMC_TIMEOUT_MS		20
#define MMC_FSM_RESET_US	100
#define MAX_PHASE_DELAY		0x7F
#define DRIVER_NAME		"omap_hsmmc"

#define VDD_SIGNAL_VOLTAGE_180	1800000
#define VDD_SIGNAL_VOLTAGE_330	3300000
#define VDD_165_195		(ffs(MMC_VDD_165_195) - 1)

#define EMMC_HSDDR_SD_SDR25_MAX	52000000
#define SD_SDR50_MAX_FREQ	104000000

#define AUTO_CMD12		(1 << 0)	/* Auto CMD12 support */

/* Errata definitions */
#define OMAP_HSMMC_ERRATA_I761	BIT(0)
#define OMAP_HSMMC_ERRATA_FSMR	BIT(1)

/*
 * One controller can have multiple slots, like on some omap boards using
 * omap.c controller driver. Luckily this is not currently done on any known
 * omap_hsmmc.c device.
 */
#define mmc_slot(host)		(host->pdata->slots[host->slot_id])

/*
 * MMC Host controller read/write API's
 */
#define OMAP_HSMMC_READ(base, reg)	\
	readl((base) + OMAP_HSMMC_##reg)

#define OMAP_HSMMC_WRITE(base, reg, val) \
	writel((val), (base) + OMAP_HSMMC_##reg)

struct adma_desc_table {
	u16 attr;
	u16 length;
	dma_addr_t addr;
};

struct omap_hsmmc_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	clk		*fclk;
	struct	clk		*dbclk;
	/*
	 * vcc == configured supply
	 * vcc_aux == optional
	 *   -	MMC1, supply for DAT4..DAT7
	 *   -	MMC2/MMC2, external level shifter voltage supply, for
	 *	chip (SDIO, eMMC, etc) or transceiver (MMC2 only)
	 */
	struct	regulator	*vcc;
	struct	regulator	*vcc_aux;
	void	__iomem		*base;
	resource_size_t		mapbase;
	spinlock_t		irq_lock; /* Prevent races with irq handler */
	struct completion	buf_ready;
	unsigned int		dma_sg_idx;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	u32			*buffer;
	u32			bytesleft;
	int			suspended;
	int			irq;
	int			dma_type, dma_ch;
	struct adma_desc_table	*adma_table;
	dma_addr_t		phy_adma_table;
	int			dma_line_tx, dma_line_rx;
	int			slot_id;
	int			response_busy;
	int			context_loss;
	int			vdd;
	int			protect_card;
	int			reqs_blocked;
	int			use_reg;
	int			req_in_progress;
	unsigned long		req_ts;
	unsigned int		flags;
	unsigned int		errata;
	int			regulator_enabled;
	int			tuning_done;
	int			tuning_fsrc;
	u32			tuning_uhsmc;
	u32			tuning_opcode;

	int			mapped_data_cnt;

	struct	omap_mmc_platform_data	*pdata;
};
static u32 tuning_data[16];
static u32 ref_tuning[16] = { 0x00FF0FFF, 0xCCC3CCFF, 0xFFCC3CC3, 0xEFFEFFFE,
				0xDDFFDFFF, 0xFBFFFBFF, 0xFF7FFFBF, 0xEFBDF777,
				0xF0FFF0FF, 0x3CCCFC0F, 0xCFCC33CC, 0xEEFFEFFF,
				0xFDFFFDFF, 0xFFBFFFDF, 0xFFF7FFBB, 0xDE7B7FF7,
				};

static int
omap_hsmmc_prepare_data(struct omap_hsmmc_host *host, struct mmc_request *req);

static void omap_hsmmc_status_notify_cb(int card_present, void *dev_id)
{
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *)dev_id;
	unsigned int status, oldstat;

	pr_debug("%s: card_present %d\n", mmc_hostname(host->mmc),
		card_present);

	if (!mmc_slot(host).mmc_data.status) {
		mmc_detect_change(host->mmc, 0);
		return;
	}

	status = mmc_slot(host).mmc_data.status(mmc_dev(host->mmc));

	oldstat = mmc_slot(host).mmc_data.card_present;
	mmc_slot(host).mmc_data.card_present = status;
	if (status ^ oldstat) {
		pr_debug("%s: Slot status change detected (%d -> %d)\n",
			mmc_hostname(host->mmc), oldstat, status);
		mmc_detect_change(host->mmc, 0);
	}
}

static inline int omap_hsmmc_set_dll(struct omap_hsmmc_host *host, int count)
{
	u32 dll;

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll &= ~(DLL_FORCE_SR_C_MASK);
	dll &= ~DLL_CALIB;
	dll |= (count << DLL_FORCE_SR_C_SHIFT);
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	dll |= DLL_CALIB;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	dll &= ~DLL_CALIB;
	dll = OMAP_HSMMC_READ(host->base, DLL);

	return 0;
}

static int omap_hsmmc_card_detect(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* NOTE: assumes card detect signal is active-low */
	return !gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

static int omap_hsmmc_get_wp(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* NOTE: assumes write protect signal is active-high */
	return gpio_get_value_cansleep(mmc->slots[0].gpio_wp);
}

static int omap_hsmmc_get_cover_state(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* NOTE: assumes card detect signal is active-low */
	return !gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

#ifdef CONFIG_PM

static int omap_hsmmc_suspend_cdirq(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	disable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

static int omap_hsmmc_resume_cdirq(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	enable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

#else

#define omap_hsmmc_suspend_cdirq	NULL
#define omap_hsmmc_resume_cdirq		NULL

#endif

#ifdef CONFIG_REGULATOR

static int omap_hsmmc_set_power(struct device *dev, int slot, int power_on,
				   int vdd_iopower)
{
	struct omap_hsmmc_host *host =
		platform_get_drvdata(to_platform_device(dev));
	struct mmc_ios *ios = &host->mmc->ios;
	int ret = 0;
	u32 ac12 = 0;

	/*
	 * If we don't see a Vcc regulator, assume it's a fixed
	 * voltage always-on regulator.
	 */
	if (!host->vcc)
		return 0;
	/*
	 * With DT, never turn OFF the regulator. This is because
	 * the pbias cell programming support is still missing when
	 * booting with Device tree
	 */
	if (dev->of_node && !vdd_iopower)
		return 0;

	if (mmc_slot(host).before_set_reg)
		mmc_slot(host).before_set_reg(dev, slot, power_on, vdd_iopower);

	/*
	 * Assume Vcc regulator is used only to power the card ... OMAP
	 * VDDS is used to power the pins, optionally with a transceiver to
	 * support cards using voltages other than VDDS (1.8V nominal).  When a
	 * transceiver is used, DAT3..7 are muxed as transceiver control pins.
	 *
	 * In some cases this regulator won't support enable/disable;
	 * e.g. it's a fixed rail for a WLAN chip.
	 *
	 * In other cases vcc_aux switches interface power.  Example, for
	 * eMMC cards it represents VccQ.  Sometimes transceivers or SDIO
	 * chips/cards need an interface voltage rail too.
	 */
	if (power_on) {
		ret = mmc_regulator_set_ocr(host->mmc, host->vcc, ios->vdd);
		/* Enable interface voltage rail, if needed */
		if (host->vcc_aux) {
			ac12 = OMAP_HSMMC_READ(host->base, AC12);
			if (vdd_iopower == VDD_165_195) {
				ret = regulator_set_voltage(host->vcc_aux,
				VDD_SIGNAL_VOLTAGE_180, VDD_SIGNAL_VOLTAGE_180);
				ac12 |= OMAP_V1V8_SIGEN_V1V8;
			} else {
				ret = regulator_set_voltage(host->vcc_aux,
				VDD_SIGNAL_VOLTAGE_330, VDD_SIGNAL_VOLTAGE_330);
				ac12 &= ~OMAP_V1V8_SIGEN_V1V8;
			}
			OMAP_HSMMC_WRITE(host->base, AC12, ac12);
			/* According to HSMMC TRM, the BIAS voltage needs upto
			 * 5ms to stabilize before turning on the IOs
			 */
			msleep(5);
		}
		if (host->vcc_aux && !host->regulator_enabled && !ret) {
			ret = regulator_enable(host->vcc_aux);
			if (!ret)
				host->regulator_enabled = 1;
		}
	} else {
		/* Shut down the rail */
		if (host->vcc_aux && regulator_is_enabled(host->vcc_aux) > 0) {
			ret = regulator_disable(host->vcc_aux);
			if (!ret)
				host->regulator_enabled = 0;
		} else {
				host->regulator_enabled = 0;
		}
		if (host->vcc) {
			/* Then proceed to shut down the local regulator */
			ret = mmc_regulator_set_ocr(host->mmc,
						host->vcc, 0);
		}
	}

	if (mmc_slot(host).after_set_reg)
		mmc_slot(host).after_set_reg(dev, slot, power_on, vdd_iopower);

	return ret;
}

static int omap_hsmmc_reg_get(struct omap_hsmmc_host *host)
{
	struct regulator *reg;
	int ocr_value = 0;

	mmc_slot(host).set_power = omap_hsmmc_set_power;

	reg = regulator_get(host->dev, "vmmc");
	if (IS_ERR(reg)) {
		dev_dbg(host->dev, "vmmc regulator missing\n");
	} else {
		host->vcc = reg;
		ocr_value = mmc_regulator_get_ocrmask(reg);
		if (!mmc_slot(host).ocr_mask) {
			mmc_slot(host).ocr_mask = ocr_value;
		} else {
			if (!(mmc_slot(host).ocr_mask & ocr_value)) {
				dev_err(host->dev, "ocrmask %x is not supported\n",
					mmc_slot(host).ocr_mask);
				mmc_slot(host).ocr_mask = 0;
				return -EINVAL;
			}
		}

		/* Allow an aux regulator */
		reg = regulator_get(host->dev, "vmmc_aux");
		host->vcc_aux = IS_ERR(reg) ? NULL : reg;

		/* For eMMC do not power off when not in sleep state */
		if (mmc_slot(host).no_regulator_off_init)
			return 0;
		/*
		* UGLY HACK:  workaround regulator framework bugs.
		* When the bootloader leaves a supply active, it's
		* initialized with zero usecount ... and we can't
		* disable it without first enabling it.  Until the
		* framework is fixed, we need a workaround like this
		* (which is safe for MMC, but not in general).
		*/
		if (regulator_is_enabled(host->vcc) > 0 ||
		    (host->vcc_aux && regulator_is_enabled(host->vcc_aux))) {
			int vdd = ffs(mmc_slot(host).ocr_mask) - 1;

			mmc_slot(host).set_power(host->dev, host->slot_id,
						 1, vdd);
			mmc_slot(host).set_power(host->dev, host->slot_id,
						 0, 0);
		}
	}

	return 0;
}

static void omap_hsmmc_reg_put(struct omap_hsmmc_host *host)
{
	regulator_put(host->vcc);
	regulator_put(host->vcc_aux);
	mmc_slot(host).set_power = NULL;
}

static inline int omap_hsmmc_have_reg(void)
{
	return 1;
}

#else

static inline int omap_hsmmc_reg_get(struct omap_hsmmc_host *host)
{
	return -EINVAL;
}

static inline void omap_hsmmc_reg_put(struct omap_hsmmc_host *host)
{
}

static inline int omap_hsmmc_have_reg(void)
{
	return 0;
}

#endif

static int omap_hsmmc_gpio_init(struct omap_mmc_platform_data *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->slots[0].switch_pin)) {
		if (pdata->slots[0].cover)
			pdata->slots[0].get_cover_state =
					omap_hsmmc_get_cover_state;
		else
			pdata->slots[0].card_detect = omap_hsmmc_card_detect;
		pdata->slots[0].card_detect_irq =
				gpio_to_irq(pdata->slots[0].switch_pin);
		ret = gpio_request(pdata->slots[0].switch_pin, "mmc_cd");
		if (ret)
			return ret;
		ret = gpio_direction_input(pdata->slots[0].switch_pin);
		if (ret)
			goto err_free_sp;
		ret = gpio_set_debounce(pdata->slots[0].switch_pin, 50000);
		if (ret)
			goto err_free_sp;
	} else
		pdata->slots[0].switch_pin = -EINVAL;

	if (gpio_is_valid(pdata->slots[0].gpio_wp)) {
		pdata->slots[0].get_ro = omap_hsmmc_get_wp;
		ret = gpio_request(pdata->slots[0].gpio_wp, "mmc_wp");
		if (ret)
			goto err_free_cd;
		ret = gpio_direction_input(pdata->slots[0].gpio_wp);
		if (ret)
			goto err_free_wp;
	} else
		pdata->slots[0].gpio_wp = -EINVAL;

	/* Make MMC wake up capable */
	enable_irq_wake(pdata->slots[0].card_detect_irq);

	return 0;

err_free_wp:
	gpio_free(pdata->slots[0].gpio_wp);
err_free_cd:
	if (gpio_is_valid(pdata->slots[0].switch_pin))
err_free_sp:
		gpio_free(pdata->slots[0].switch_pin);
	return ret;
}

static void omap_hsmmc_gpio_free(struct omap_mmc_platform_data *pdata)
{
	if (gpio_is_valid(pdata->slots[0].gpio_wp))
		gpio_free(pdata->slots[0].gpio_wp);
	if (gpio_is_valid(pdata->slots[0].switch_pin))
		gpio_free(pdata->slots[0].switch_pin);
}

/*
 * Start clock to the card
 */
static void omap_hsmmc_start_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);
}

/*
 * Stop clock to the card
 */
static void omap_hsmmc_stop_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC Clock is not stoped\n");
}

static void omap_hsmmc_enable_irq(struct omap_hsmmc_host *host,
				  struct mmc_command *cmd)
{
	unsigned int irq_mask;

	if (cmd->opcode == MMC_SEND_TUNING_BLOCK)
		irq_mask = INT_EN_MASK | BRR_ENABLE;
	else if (host->dma_type)
		irq_mask = INT_EN_MASK & ~(BRR_ENABLE | BWR_ENABLE);
	else
		irq_mask = INT_EN_MASK;

	/* Disable timeout for erases */
	if (cmd->opcode == MMC_ERASE
#ifdef CONFIG_LAB126_DIAG_MODE
		|| cmd->opcode == MMC_SWITCH
#endif
		)
		irq_mask &= ~DTO_ENABLE;

	OMAP_HSMMC_WRITE(host->base, ISE, irq_mask);
	OMAP_HSMMC_WRITE(host->base, IE, irq_mask);
}

static void omap_hsmmc_disable_irq(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, ISE, 0);
	OMAP_HSMMC_WRITE(host->base, IE, 0);
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
}

/* Calculate divisor for the given clock frequency */
static u16 calc_divisor(struct omap_hsmmc_host *host, struct mmc_ios *ios)
{
	u16 dsor = 0;

	if (ios->clock) {
		dsor = DIV_ROUND_UP(clk_get_rate(host->fclk), ios->clock);
		if (dsor > CLKD_MAX)
			dsor = CLKD_MAX;
	}

	return dsor;
}

static inline void omap_hsmmc_opp_scale(struct omap_hsmmc_host *host)
{
	int ret = 0;
	if (!host->pdata->opp_scale)
		return;

	ret = host->pdata->opp_scale(host->pdata, host->mmc->ios.clock);
	if (ret)
		dev_err(mmc_dev(host->mmc), "%s error: %d", __func__, ret);
}

static inline void omap_hsmmc_opp_relax(struct omap_hsmmc_host *host)
{
	int ret = 0;
	if (!host->pdata->opp_relax)
		return;
	ret = host->pdata->opp_relax(host->pdata);
	if (ret)
		dev_err(mmc_dev(host->mmc), "%s error: %d\n", __func__, ret);
}

static inline int omap_hsmmc_restore_dll(struct omap_hsmmc_host *host)
{
	u32 ac12;
	u32 dll;

	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 |= host->tuning_uhsmc;
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll |= DLL_FORCE_VALUE;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	if (omap_hsmmc_set_dll(host, host->tuning_fsrc))
		return -EIO;
	return 0;
}

static inline void omap_hsmmc_save_dll(struct omap_hsmmc_host *host)
{
	u32 ac12;

	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 &= ~AC12_UHSMC_MASK;
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);
}

static void omap_hsmmc_set_clock(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	unsigned long regval;
	unsigned long timeout;

	dev_dbg(mmc_dev(host->mmc), "Set clock to %uHz\n", ios->clock);

	omap_hsmmc_stop_clock(host);

	omap_hsmmc_opp_scale(host);

	regval = OMAP_HSMMC_READ(host->base, SYSCTL);
	regval = regval & ~(CLKD_MASK | DTO_MASK);
	regval = regval | (calc_divisor(host, ios) << 6) | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regval);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* Wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS
		&& time_before(jiffies, timeout))
		cpu_relax();

	omap_hsmmc_start_clock(host);
}

static void omap_hsmmc_set_bus_width(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 con;

	con = OMAP_HSMMC_READ(host->base, CON);
	/* configure in DDR mode */
	if (ios->timing == MMC_TIMING_UHS_DDR50)
		con |= DDR;
	else
		con &= ~DDR;
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON, con | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}
}

static void omap_hsmmc_set_bus_mode(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 con;

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);
}

#ifdef CONFIG_PM
/*
 * Restore the MMC host context, if it was lost as result of a
 * power state change.
 */
static int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	struct omap_mmc_platform_data *pdata = host->pdata;
	int context_loss = 0;
	u32 hctl, capa, value;
	unsigned long timeout;

	omap_hsmmc_opp_scale(host);

	if (host->tuning_done)
		omap_hsmmc_restore_dll(host);

	if (pdata->get_context_loss_count) {
		context_loss = pdata->get_context_loss_count(host->dev);
		if (context_loss < 0)
			return 1;
	}

	dev_dbg(mmc_dev(host->mmc), "context was %slost\n",
		context_loss == host->context_loss ? "not " : "");
	if (host->context_loss == context_loss)
		return 1;

	timeout = 100;
	while (!(OMAP_HSMMC_READ(host->base, SYSSTATUS) & RESETDONE)) {
		udelay(1);
		if (!timeout--)
			break;
	}

	if (!(OMAP_HSMMC_READ(host->base, SYSSTATUS) & RESETDONE)) {
		dev_err(mmc_dev(host->mmc), "Register MMC_SYSSTATUS is zero");
		return -EBUSY;
	}

	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		if (host->power_mode != MMC_POWER_OFF &&
		    (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180))
			hctl = SDVS18;
		else
			hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}
	if (host->dma_type == ADMA_XFER)
		hctl |= DMAS;

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | hctl);

	OMAP_HSMMC_WRITE(host->base, CAPA,
			OMAP_HSMMC_READ(host->base, CAPA) | capa);

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	if (host->dma_type == ADMA_XFER) {
		value = OMAP_HSMMC_READ(host->base, CON);
		OMAP_HSMMC_WRITE(host->base, CON, value | DMA_MNS_ADMA_MODE);
	}

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, HCTL) & SDBP) != SDBP
		&& time_before(jiffies, timeout))
		;

	omap_hsmmc_disable_irq(host);

	/* Do not initialize card-specific things if the power is off */
	if (host->power_mode == MMC_POWER_OFF)
		goto out;

	omap_hsmmc_set_bus_width(host);

	omap_hsmmc_set_clock(host);

	omap_hsmmc_set_bus_mode(host);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		value = OMAP_HSMMC_READ(host->base, HCTL);
		value &= ~SDVS_MASK;
		value |= SDVS18;
		OMAP_HSMMC_WRITE(host->base, HCTL, value);

		value = OMAP_HSMMC_READ(host->base, AC12);
		value |= OMAP_V1V8_SIGEN_V1V8;
		OMAP_HSMMC_WRITE(host->base, AC12, value);
	}

out:
	host->context_loss = context_loss;

	dev_dbg(mmc_dev(host->mmc), "context is restored\n");
	return 0;
}

/*
 * Save the MMC host context (store the number of power state changes so far).
 */
static void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
	struct omap_mmc_platform_data *pdata = host->pdata;
	int context_loss;

	if (pdata->get_context_loss_count) {
		context_loss = pdata->get_context_loss_count(host->dev);
		if (context_loss < 0)
			return;
		host->context_loss = context_loss;
	}
	if (host->tuning_done)
		omap_hsmmc_save_dll(host);
	omap_hsmmc_opp_relax(host);
}

#else

static int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	return 0;
}

static void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
}

#endif

/*
 * Send init stream sequence to card
 * before sending IDLE command
 */
static void send_init_stream(struct omap_hsmmc_host *host)
{
	int reg = 0;
	unsigned long timeout;

	if (host->protect_card)
		return;

	disable_irq(host->irq);

	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC) && time_before(jiffies, timeout))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_READ(host->base, STAT);

	enable_irq(host->irq);
}

static inline
int omap_hsmmc_cover_is_closed(struct omap_hsmmc_host *host)
{
	int r = 1;

	if (mmc_slot(host).get_cover_state)
		r = mmc_slot(host).get_cover_state(host->dev, host->slot_id);
	return r;
}

static ssize_t
omap_hsmmc_show_cover_switch(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n",
			omap_hsmmc_cover_is_closed(host) ? "closed" : "open");
}

static DEVICE_ATTR(cover_switch, S_IRUGO, omap_hsmmc_show_cover_switch, NULL);

static ssize_t
omap_hsmmc_show_slot_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n", mmc_slot(host).name);
}

static DEVICE_ATTR(slot_name, S_IRUGO, omap_hsmmc_show_slot_name, NULL);

/*
 * Configure the response type and send the cmd.
 */
static void
omap_hsmmc_start_command(struct omap_hsmmc_host *host, struct mmc_command *cmd,
	struct mmc_data *data, bool no_auto_cmd12)
{
	int cmdreg = 0, resptype = 0, cmdtype = 0;
	int blocks = 0;

	if (data)
		blocks = data->blocks;

	/* ACOS_MOD_BEGIN */
	TRAPZ_DESCRIBE(TRAPZ_KERN_MMC, omap_hsmmc_start_command,
			"MMC start device command");
	TRAPZ_LOG_PRINTF(TRAPZ_LOG_DEBUG, 0, TRAPZ_KERN_MMC,
			omap_hsmmc_start_command,
			"CMD%d; argument=0x%08x",
			cmd->opcode, cmd->arg, host->mmc->index, blocks);
	/* ACOS_MOD_END */

	dev_dbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
		mmc_hostname(host->mmc), cmd->opcode, cmd->arg);
	host->cmd = cmd;

	omap_hsmmc_enable_irq(host, cmd);

	host->response_busy = 0;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			resptype = 1;
		else if (cmd->flags & MMC_RSP_BUSY) {
			resptype = 3;
			host->response_busy = 1;
		} else
			resptype = 2;
	}

	/*
	 * Unlike OMAP1 controller, the cmdtype does not seem to be based on
	 * ac, bc, adtc, bcr. Only commands ending an open ended transfer need
	 * a val of 0x3, rest 0x0.
	 */
	if (cmd == host->mrq->stop)
		cmdtype = 0x3;

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);
	if ((host->flags & AUTO_CMD12) &&
			mmc_op_multi(cmd->opcode) && !no_auto_cmd12)
		cmdreg |= ACEN_ACMD12;

	if (data) {
		cmdreg |= DP_SELECT | MSBS | BCE;
		if (data->flags & MMC_DATA_READ)
			cmdreg |= DDIR;
		else
			cmdreg &= ~(DDIR);
	}

	if (host->dma_type)
		cmdreg |= DMA_EN;

	/* Tuning command is special. Data Present Select should be set */
	if ((cmd->opcode == MMC_SEND_TUNING_BLOCK) ||
		(cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200)) {
		cmdreg = (cmd->opcode << 24) | (resptype << 16) |
			(cmdtype << 22) | DP_SELECT | DDIR;
	}
	host->req_in_progress = 1;

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
	if (cmd->opcode <= 10)
		printk(KERN_DEBUG "MMC%d: cmd=%d arg=%08X\n",
				host->mmc->index, cmd->opcode, cmd->arg);
}

static int
omap_hsmmc_get_dma_dir(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

static void omap_hsmmc_request_done(struct omap_hsmmc_host *host, struct mmc_request *mrq)
{
	int dma_ch;
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);
	host->req_in_progress = 0;
	host->req_ts = jiffies;
	dma_ch = host->dma_ch;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	omap_hsmmc_disable_irq(host);
	/* Do not complete the request if DMA is still in progress */
	if (mrq->data && host->dma_type && dma_ch != -1)
		return;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

static int
omap_hsmmc_map_data(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	int dma_len = 0;
	unsigned long flags;
	spin_lock_irqsave(&host->irq_lock, flags);
	if (data)
		dma_len = data->dma_len;
	if (data && !dma_len) {
		dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg,
				     data->sg_len,
				     omap_hsmmc_get_dma_dir(host, data));
		data->dma_len = dma_len;
		if (dma_len)
			host->mapped_data_cnt++;
	}
	spin_unlock_irqrestore(&host->irq_lock, flags);
	WARN(host->mapped_data_cnt < 0 || host->mapped_data_cnt > 2,
		"MMC%d: Mapped Data count: %d",
		host->mmc->index, host->mapped_data_cnt);
	return dma_len  > 0 ? 0 : -EINVAL;
}

static void
omap_hsmmc_unmap_data(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	unsigned long flags;
	spin_lock_irqsave(&host->irq_lock, flags);
	if (data && data->dma_len) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				omap_hsmmc_get_dma_dir(host, data));
		data->dma_len = 0;
		host->mapped_data_cnt--;
	}
	spin_unlock_irqrestore(&host->irq_lock, flags);
	WARN(host->mapped_data_cnt < 0 || host->mapped_data_cnt > 2,
		"MMC%d: Mapped Data count: %d",
		host->mmc->index, host->mapped_data_cnt);
}

/*
 * Notify the transfer complete to MMC core
 */
static void
omap_hsmmc_xfer_done(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (!data) {
		struct mmc_request *mrq = host->mrq;

		/* some commands generate busy on DATx, but do not
		 * transfer data. Transition from on DATA bus from
		 * busy to ready generates TC interrupt, even if
		 * there was no data transfer. It may happen before
		 * or after CC interrupt. */
		if (host->cmd && host->response_busy) {
			host->response_busy = 0;
			return;
		}

		omap_hsmmc_request_done(host, mrq);
		return;
	}

	host->data = NULL;

	if (host->dma_type == ADMA_XFER)
		omap_hsmmc_unmap_data(host, data);
	if (!data->error)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;

	if (data->stop && ((!(host->flags & AUTO_CMD12)) || data->error)) {
		omap_hsmmc_start_command(host, data->stop, NULL, 0);
	} else {
		struct mmc_request *req = host->mrq;

		/* fill-in stop status only if CMD12 has actually been send */
		if (data->stop && !req->sbc)
			data->stop->resp[0] = OMAP_HSMMC_READ(host->base,
							RSP76);
		omap_hsmmc_request_done(host, data->mrq);
	}

	return;
}

static int
omap_hsmmc_errata_i761(struct omap_hsmmc_host *host, struct mmc_command *cmd)
{
	u32 rsp10, csre;

	if ((cmd->flags & MMC_RSP_R1) == MMC_RSP_R1
			|| (host->mmc->card && (mmc_card_sd(host->mmc->card)
			|| mmc_card_sdio(host->mmc->card))
			&& (cmd->flags & MMC_RSP_R5))) {
		rsp10 = OMAP_HSMMC_READ(host->base, RSP10);
		csre = OMAP_HSMMC_READ(host->base, CSRE);
		return rsp10 & csre;
	}

	return 0;
}

/*
 * Notify the core about command completion
 */
static void
omap_hsmmc_cmd_done(struct omap_hsmmc_host *host, struct mmc_command *cmd)
{
	struct mmc_request *req = host->mrq;

	if (host->cmd->opcode == MMC_SEND_TUNING_BLOCK)
		return;

	if (req->sbc && (host->cmd == req->sbc)) {
		int err;
		host->cmd = NULL;
		err = omap_hsmmc_prepare_data(host, req);
		if (err) {
			req->cmd->error = err;
			if (req->data)
				req->data->error = err;
			omap_hsmmc_request_done(host, req);
			return;
		}
		omap_hsmmc_start_command(host, host->mrq->cmd,
			host->mrq->data, 1);
		return;
	}

	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
			TRAPZ_DESCRIBE(TRAPZ_KERN_MMC, omap_hsmmc_cmd_done,
					"MMC HC status");
			TRAPZ_LOG_PRINTF(TRAPZ_LOG_VERBOSE, 0, TRAPZ_KERN_MMC,
					omap_hsmmc_cmd_done,
					"Status: %d %d %d %d",
					cmd->resp[3], cmd->resp[2], host->mmc->index, 0);
			TRAPZ_LOG_PRINTF(TRAPZ_LOG_VERBOSE, 0, TRAPZ_KERN_MMC,
					omap_hsmmc_cmd_done,
					"Status: %d %d %d %d",
					cmd->resp[1], cmd->resp[0], host->mmc->index, 1);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
			TRAPZ_LOG_PRINTF(TRAPZ_LOG_VERBOSE, 0, TRAPZ_KERN_MMC,
					omap_hsmmc_cmd_done,
					"Status: %d %d %d %d",
					cmd->resp[0], 0, host->mmc->index, 2);
		}
	}
	if ((host->data == NULL && !host->response_busy) || cmd->error)
		omap_hsmmc_request_done(host, cmd->mrq);
}

/*
 * DMA clean up for command errors
 */
static void omap_hsmmc_dma_cleanup(struct omap_hsmmc_host *host, int errno)
{
	int dma_ch;
	unsigned long flags;

	host->data->error = errno;

	spin_lock_irqsave(&host->irq_lock, flags);
	dma_ch = host->dma_ch;
	host->dma_ch = -1;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	if ((host->dma_type == SDMA_XFER) && (dma_ch != -1)) {
		omap_hsmmc_unmap_data(host, host->data);
		omap_free_dma(dma_ch);
	}
	host->data = NULL;
}

/*
 * Readable error output
 */
#ifdef CONFIG_MMC_DEBUG
static void omap_hsmmc_dbg_report_irq(struct omap_hsmmc_host *host, u32 status)
{
	/* --- means reserved bit without definition at documentation */
	static const char *omap_hsmmc_status_bits[] = {
		"CC"  , "TC"  , "BGE", "---", "BWR" , "BRR" , "---" , "---" ,
		"CIRQ", "OBI" , "---", "---", "---" , "---" , "---" , "ERRI",
		"CTO" , "CCRC", "CEB", "CIE", "DTO" , "DCRC", "DEB" , "---" ,
		"ACE" , "---" , "---", "---", "CERR", "BADA", "---" , "---" ,
		"---" , "CC"  , "TC" , "BGE", "DMA" , "BWR" , "BRR" , "CINS",
		"CREM", "CIRQ", "OBI", "BSR", "---" , "---" , "---" , "---" ,
		"ERRI", "CTO" , "CCRC", "CEB", "CIE" , "DTO" , "DCRC", "DEB" ,
		"CLE" , "ACE" , "ADMA", "---", "---" , "CERR", "BADA", "---" ,
		"---"
	};
	char res[256];
	char *buf = res;
	int len, i;

	len = sprintf(buf, "MMC IRQ 0x%x :", status);
	buf += len;

	for (i = 0; i < ARRAY_SIZE(omap_hsmmc_status_bits); i++)
		if (status & (1 << i)) {
			len = sprintf(buf, " %s", omap_hsmmc_status_bits[i]);
			buf += len;
		}

	dev_dbg(mmc_dev(host->mmc), "%s\n", res);
}
#else
static inline void omap_hsmmc_dbg_report_irq(struct omap_hsmmc_host *host,
					     u32 status)
{
}
#endif  /* CONFIG_MMC_DEBUG */

/*
 * MMC controller internal state machines reset
 *
 * Used to reset command or data internal state machines, using respectively
 *  SRC or SRD bit of SYSCTL register
 * Can be called from interrupt context
 */
static inline void omap_hsmmc_reset_controller_fsm(struct omap_hsmmc_host *host,
						   unsigned long bit)
{
	int i = 0;
	int limit = DIV_ROUND_UP(MMC_FSM_RESET_US, 10);

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			 OMAP_HSMMC_READ(host->base, SYSCTL) | bit);
	if (host->errata & OMAP_HSMMC_ERRATA_FSMR) {
		while ((!(OMAP_HSMMC_READ(host->base, SYSCTL) & bit))
					&& (i++ < limit))
			udelay(10);
	}
	i = 0;

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & bit) &&
		(i++ < limit)) {
		udelay(10);
	}

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & bit)
		dev_err(mmc_dev(host->mmc),
			"Timeout waiting on controller reset in %s\n",
			__func__);
}

static void omap_hsmmc_do_irq(struct omap_hsmmc_host *host, int status)
{
	struct mmc_data *data;
	int end_cmd = 0, end_trans = 0;
	int i = 0;

	if (!host->req_in_progress) {
		dev_dbg(host->dev, "Supirious IRQ 0x%08X\n", status);
		return;
	}

	data = host->data;
	dev_dbg(mmc_dev(host->mmc), "IRQ Status is %x\n", status);
	/* ACOS_MOD_BEGIN */
	TRAPZ_DESCRIBE(TRAPZ_KERN_MMC, omap_hsmmc_do_irq,
			"MMC interrupt");
	TRAPZ_LOG_PRINTF(TRAPZ_LOG_DEBUG, 0, TRAPZ_KERN_MMC,
			omap_hsmmc_do_irq,
			"Status %x", status, 0, host->mmc->index, 0);
	/* ACOS_MOD_END */

	if (status & ERR) {
		omap_hsmmc_dbg_report_irq(host, status);
		if ((status & CMD_TIMEOUT) ||
			(status & CMD_CRC)) {
			if (host->cmd) {
				if (status & CMD_TIMEOUT) {
					omap_hsmmc_reset_controller_fsm(host,
									SRC);
					host->cmd->error = -ETIMEDOUT;
				} else {
					host->cmd->error = -EILSEQ;
				}
				end_cmd = 1;
			}
			if (host->data || host->response_busy) {
				if (host->data)
					omap_hsmmc_dma_cleanup(host,
								-ETIMEDOUT);
				host->response_busy = 0;
				omap_hsmmc_reset_controller_fsm(host, SRD);
			}
		}
		if ((status & DATA_TIMEOUT) ||
			(status & DATA_CRC)) {
			if (host->data || host->response_busy) {
				int err = (status & DATA_TIMEOUT) ?
						-ETIMEDOUT : -EILSEQ;

				if (host->data)
					omap_hsmmc_dma_cleanup(host, err);
				else
					host->mrq->cmd->error = err;
				host->response_busy = 0;
				omap_hsmmc_reset_controller_fsm(host, SRD);
				end_trans = 1;
			}
		}
		if (status & CARD_ERR) {
			dev_dbg(mmc_dev(host->mmc),
				"Ignoring card err CMD%d\n", host->cmd->opcode);
			if (host->cmd)
				end_cmd = 1;
			if (host->data)
				end_trans = 1;
		}
		if (status & ADMA_ERR) {
			dev_dbg(mmc_dev(host->mmc),
				"ADMA err: ADMA_ES=%x, SAL=%x.\n",
					OMAP_HSMMC_READ(host->base, ADMA_ES),
					OMAP_HSMMC_READ(host->base, ADMA_SAL));
			if (host->cmd)
				end_cmd = 1;
			if (host->data)
				end_trans = 1;
		}
	}
	if (status & ADMA_XFER_INT) {
		dev_dbg(mmc_dev(host->mmc),
			"ADMA XFERINT: blk=%x at table=%x pstate=%x\n",
			OMAP_HSMMC_READ(host->base, BLK),
			OMAP_HSMMC_READ(host->base, ADMA_SAL),
			OMAP_HSMMC_READ(host->base, PSTATE));
	}

	/* Errata i761 */
	if ((host->errata & OMAP_HSMMC_ERRATA_I761) && host->cmd
			&& omap_hsmmc_errata_i761(host, host->cmd)) {
		/* Do the same as for CARD_ERR case */
		dev_dbg(mmc_dev(host->mmc),
			"Ignoring card err CMD%d\n", host->cmd->opcode);
		end_cmd = 1;
		if (host->data)
			end_trans = 1;
	}

	if (status & BRR) {
		for (i = 0; i < sizeof(tuning_data)/4; i++)
			tuning_data[i] = OMAP_HSMMC_READ(host->base, DATA);
		complete(&host->buf_ready);
		return;
	}

	if (end_cmd || ((status & CC) && host->cmd))
		omap_hsmmc_cmd_done(host, host->cmd);
	if ((end_trans || (status & TC)) && host->mrq)
		omap_hsmmc_xfer_done(host, data);
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t omap_hsmmc_irq(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	int status;

	status = OMAP_HSMMC_READ(host->base, STAT);
	do {
		OMAP_HSMMC_WRITE(host->base, STAT, status);
		omap_hsmmc_do_irq(host, status);
		/* Flush posted write */
		status = OMAP_HSMMC_READ(host->base, STAT);
	} while (status);

	return IRQ_HANDLED;
}

static void set_sd_bus_power(struct omap_hsmmc_host *host)
{
	unsigned long i;

	OMAP_HSMMC_WRITE(host->base, HCTL,
			 OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
	for (i = 0; i < loops_per_jiffy; i++) {
		if (OMAP_HSMMC_READ(host->base, HCTL) & SDBP)
			break;
		cpu_relax();
	}
}

/*
 * Switch MMC interface voltage ... only relevant for MMC1.
 *
 * MMC2 and MMC3 use fixed 1.8V levels, and maybe a transceiver.
 * The MMC2 transceiver controls are used instead of DAT4..DAT7.
 * Some chips, like eMMC ones, use internal transceivers.
 */
static int omap_hsmmc_switch_opcond(struct omap_hsmmc_host *host, int vdd)
{
	u32 reg_val = 0;
	int ret;

	/* Disable the clocks */
	pm_runtime_put_sync(host->dev);
	if (host->dbclk)
		clk_disable(host->dbclk);

	/* Turn the power off */
	ret = mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);

	/* Turn the power ON with given VDD 1.8 or 3.0v */
	if (!ret)
		ret = mmc_slot(host).set_power(host->dev, host->slot_id, 1,
					       vdd);
	pm_runtime_get_sync(host->dev);
	if (host->dbclk)
		clk_enable(host->dbclk);

	if (ret != 0)
		goto err;

	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
	reg_val = OMAP_HSMMC_READ(host->base, HCTL);

	/*
	 * If a MMC dual voltage card is detected, the set_ios fn calls
	 * this fn with VDD bit set for 1.8V. Upon card removal from the
	 * slot, omap_hsmmc_set_ios sets the VDD back to 3V on MMC_POWER_OFF.
	 *
	 * Cope with a bit of slop in the range ... per data sheets:
	 *  - "1.8V" for vdds_mmc1/vdds_mmc1a can be up to 2.45V max,
	 *    but recommended values are 1.71V to 1.89V
	 *  - "3.0V" for vdds_mmc1/vdds_mmc1a can be up to 3.5V max,
	 *    but recommended values are 2.7V to 3.3V
	 *
	 * Board setup code shouldn't permit anything very out-of-range.
	 * TWL4030-family VMMC1 and VSIM regulators are fine (avoiding the
	 * middle range) but VSIM can't power DAT4..DAT7 at more than 3V.
	 */
	if ((1 << vdd) <= MMC_VDD_23_24)
		reg_val |= SDVS18;
	else
		reg_val |= SDVS30;

	OMAP_HSMMC_WRITE(host->base, HCTL, reg_val);
	set_sd_bus_power(host);

	return 0;
err:
	dev_dbg(mmc_dev(host->mmc), "Unable to switch operating voltage\n");
	return ret;
}

/* Protect the card while the cover is open */
static void omap_hsmmc_protect_card(struct omap_hsmmc_host *host)
{
	if (!mmc_slot(host).get_cover_state)
		return;

	host->reqs_blocked = 0;
	if (mmc_slot(host).get_cover_state(host->dev, host->slot_id)) {
		if (host->protect_card) {
			dev_info(host->dev, "%s: cover is closed, "
					 "card is now accessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 0;
		}
	} else {
		if (!host->protect_card) {
			dev_info(host->dev, "%s: cover is open, "
					 "card is now inaccessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 1;
		}
	}
}

/*
 * irq handler to notify the core about card insertion/removal
 */
static irqreturn_t omap_hsmmc_detect(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	struct omap_mmc_slot_data *slot = &mmc_slot(host);
	int carddetect;

	if (host->suspended)
		return IRQ_HANDLED;

	sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");

	if (slot->card_detect)
		carddetect = slot->card_detect(host->dev, host->slot_id);
	else {
		omap_hsmmc_protect_card(host);
		carddetect = -ENOSYS;
	}

	if (carddetect) {
		mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	} else {
		host->tuning_done = 0;
	/*
	 * Because of OMAP4 Silicon errata (i705), we have to turn off the
	 * PBIAS and VMMC for SD card as soon as we get card disconnect
	 * interrupt. Because of this, we don't wait for all higher layer
	 * structures to be dismantled before turning off power
	 */
		mmc_claim_host(host->mmc);
		if ((MMC_POWER_OFF != host->power_mode) &&
				(mmc_slot(host).set_power != NULL)) {
			mmc_slot(host).set_power(host->dev, host->slot_id,
						0, 0);
			host->power_mode = MMC_POWER_OFF;
		}
		mmc_release_host(host->mmc);
		mmc_detect_change(host->mmc, 0);
	}
	return IRQ_HANDLED;
}

static int omap_hsmmc_get_dma_sync_dev(struct omap_hsmmc_host *host,
				     struct mmc_data *data)
{
	int sync_dev;

	if (data->flags & MMC_DATA_WRITE)
		sync_dev = host->dma_line_tx;
	else
		sync_dev = host->dma_line_rx;
	return sync_dev;
}

static void omap_hsmmc_config_dma_params(struct omap_hsmmc_host *host,
				       struct mmc_data *data,
				       struct scatterlist *sgl)
{
	int blksz, nblk, dma_ch;

	dma_ch = host->dma_ch;
	if (data->flags & MMC_DATA_WRITE) {
		omap_set_dma_dest_params(dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
			(host->mapbase + OMAP_HSMMC_DATA), 0, 0);
		omap_set_dma_src_params(dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(sgl), 0, 0);
	} else {
		omap_set_dma_src_params(dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
			(host->mapbase + OMAP_HSMMC_DATA), 0, 0);
		omap_set_dma_dest_params(dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(sgl), 0, 0);
	}

	blksz = host->data->blksz;
	nblk = sg_dma_len(sgl) / blksz;

	omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S32,
			blksz / 4, nblk, OMAP_DMA_SYNC_FRAME,
			omap_hsmmc_get_dma_sync_dev(host, data),
			!(data->flags & MMC_DATA_WRITE));

	omap_start_dma(dma_ch);
}

/*
 * DMA call back function
 */
static void omap_hsmmc_dma_cb(int lch, u16 ch_status, void *cb_data)
{
	struct omap_hsmmc_host *host = cb_data;
	struct mmc_data *data;
	int dma_ch, req_in_progress;
	unsigned long flags;

	if (!(ch_status & OMAP_DMA_BLOCK_IRQ)) {
		dev_warn(mmc_dev(host->mmc), "unexpected dma status %x\n",
			ch_status);
		return;
	}

	spin_lock_irqsave(&host->irq_lock, flags);
	if (host->dma_ch < 0) {
		spin_unlock_irqrestore(&host->irq_lock, flags);
		return;
	}

	data = host->mrq->data;
	host->dma_sg_idx++;
	if (host->dma_sg_idx < data->dma_len) {
		/* Fire up the next transfer. */
		omap_hsmmc_config_dma_params(host, data,
					   data->sg + host->dma_sg_idx);
		spin_unlock_irqrestore(&host->irq_lock, flags);
		return;
	}

	req_in_progress = host->req_in_progress;
	dma_ch = host->dma_ch;
	host->dma_ch = -1;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	omap_hsmmc_unmap_data(host, data);
	omap_free_dma(dma_ch);

	/* If DMA has finished after TC, complete the request */
	if (!req_in_progress) {
		struct mmc_request *mrq = host->mrq;

		omap_hsmmc_request_done(host, mrq);
	}
}

static int omap_hsmmc_pre_dma_transfer(struct omap_hsmmc_host *host,
				       struct mmc_data *data)
{
	return omap_hsmmc_map_data(host, data);
}

/*
 * Routine to configure and start DMA for the MMC card
 */
static int omap_hsmmc_start_sdma_transfer(struct omap_hsmmc_host *host,
					struct mmc_request *req)
{
	int dma_ch = 0, ret = 0, i;
	struct mmc_data *data = req->data;

	/* Sanity check: all the SG entries must be aligned by block size. */
	for (i = 0; i < data->sg_len; i++) {
		struct scatterlist *sgl;

		sgl = data->sg + i;
		if (sgl->length % data->blksz)
			return -EINVAL;
	}
	if ((data->blksz % 4) != 0)
		/* REVISIT: The MMC buffer increments only when MSB is written.
		 * Return error for blksz which is non multiple of four.
		 */
		return -EINVAL;

	BUG_ON(host->dma_ch != -1);

	ret = omap_request_dma(omap_hsmmc_get_dma_sync_dev(host, data),
			       "MMC/SD", omap_hsmmc_dma_cb, host, &dma_ch);
	if (ret != 0) {
		dev_err(mmc_dev(host->mmc),
			"%s: omap_request_dma() failed with %d\n",
			mmc_hostname(host->mmc), ret);
		return ret;
	}
	ret = omap_hsmmc_pre_dma_transfer(host, data);
	if (ret)
		return ret;

	host->dma_ch = dma_ch;
	host->dma_sg_idx = 0;

	omap_hsmmc_config_dma_params(host, data, data->sg);

	return 0;
}

static int mmc_populate_adma_desc_table(struct omap_hsmmc_host *host,
		struct mmc_request *req, struct adma_desc_table *pdesc)
{
	int i, j, dmalen;
	int splitseg, xferaddr;
	int numblocks = 0;
	dma_addr_t dmaaddr;
	struct mmc_data *data = req->data;
	int ret = 0;

	ret = omap_hsmmc_pre_dma_transfer(host, data);
	if (ret)
		return ret;
	for (i = 0, j = 0; i < data->dma_len; i++) {
		dmaaddr = sg_dma_address(data->sg + i);
		dmalen = sg_dma_len(data->sg + i);
		numblocks += dmalen / data->blksz;

		if (dmalen <= ADMA_MAX_XFER_PER_ROW) {
			pdesc[i + j].length = dmalen;
			pdesc[i + j].addr = dmaaddr;
			pdesc[i + j].attr = (ADMA_XFER_DESC |
				ADMA_XFER_VALID);
		} else {
			/* Each descriptor row can only support
			 * transfer upto ADMA_MAX_XFER_PER_ROW.
			 * If the current segment is bigger, it has to be
			 * split to multiple ADMA table entries.
			 */
			xferaddr = 0;
			do {
				splitseg = min(dmalen, ADMA_MAX_XFER_PER_ROW);
				dmalen -= splitseg;
				pdesc[i + j].length = splitseg;
				pdesc[i + j].addr =
					dmaaddr + xferaddr;
				xferaddr += splitseg;
				pdesc[i + j].attr = (ADMA_XFER_DESC |
					ADMA_XFER_VALID);
				j++;
			} while (dmalen);
			j--; /* Compensate for i++ */
		}
	}
	/* Setup last entry to terminate */
	pdesc[i + j - 1].attr |= ADMA_XFER_END;
	WARN_ON((i + j - 1) > DMA_TABLE_NUM_ENTRIES);
	dev_dbg(mmc_dev(host->mmc),
		"ADMA table has %d entries from %d sglist\n",
		i + j, data->dma_len);

	/* ACOS_MOD_BEGIN */
	TRAPZ_DESCRIBE(TRAPZ_KERN_MMC,  mmc_populate_adma_desc_table,
			"MMC Descriptor table populated");
	TRAPZ_LOG_PRINTF(TRAPZ_LOG_VERBOSE, 0, TRAPZ_KERN_MMC,
			mmc_populate_adma_desc_table,
			"ADMA table has %d entries from %d sglist\n",
			i + j, data->dma_len, host->mmc->index, 0);
	/* ACOS_MOD_END */
	return numblocks;
}

static void omap_hsmmc_start_adma_transfer(struct omap_hsmmc_host *host)
{
	wmb();
	OMAP_HSMMC_WRITE(host->base, ADMA_SAL, host->phy_adma_table);
}

static void set_data_timeout(struct omap_hsmmc_host *host,
			     unsigned int timeout_ns,
			     unsigned int timeout_clks)
{
	unsigned int timeout, cycle_ns;
	uint32_t reg, clkd, dto = 0;

	reg = OMAP_HSMMC_READ(host->base, SYSCTL);
	clkd = (reg & CLKD_MASK) >> CLKD_SHIFT;
	if (clkd == 0)
		clkd = 1;

	cycle_ns = 1000000000 / (clk_get_rate(host->fclk) / clkd);
	timeout = timeout_ns / cycle_ns;
	timeout += timeout_clks;
	if (timeout) {
		while ((timeout & 0x80000000) == 0) {
			dto += 1;
			timeout <<= 1;
		}
		dto = 31 - dto;
		timeout <<= 1;
		if (timeout && dto)
			dto += 1;
		if (dto >= 13)
			dto -= 13;
		else
			dto = 0;
		if (dto > 14)
			dto = 14;
	}

	reg &= ~DTO_MASK;
	reg |= dto << DTO_SHIFT;
	OMAP_HSMMC_WRITE(host->base, SYSCTL, reg);
}

/*
 * Configure block length for MMC/SD cards and initiate the transfer.
 */
static int
omap_hsmmc_prepare_data(struct omap_hsmmc_host *host, struct mmc_request *req)
{
	int ret;
	int numblks;

	host->data = req->data;

	if (req->data == NULL) {
		OMAP_HSMMC_WRITE(host->base, BLK, 0);
		/*
		 * Set an arbitrary 500ms data timeout for commands with
		 * busy signal.
		 */
		if (req->cmd->flags & MMC_RSP_BUSY)
			/* really advanced mathematics inside */
			set_data_timeout(host, 500 * 1000000U, 0);
		return 0;
	}

	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz)
					| (req->data->blocks << 16));
	set_data_timeout(host, req->data->timeout_ns, req->data->timeout_clks);

	if (host->dma_type == SDMA_XFER) {
		ret = omap_hsmmc_start_sdma_transfer(host, req);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc), "MMC start dma failure\n");
			return ret;
		}
	} else if (host->dma_type == ADMA_XFER) {
		numblks = mmc_populate_adma_desc_table(host,
				req, host->adma_table);
		WARN_ON(numblks != req->data->blocks);
		omap_hsmmc_start_adma_transfer(host);
	}
	return 0;
}

static void omap_hsmmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
				int err)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (data && host->dma_type)
		omap_hsmmc_unmap_data(host, data);
}

static void omap_hsmmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       bool is_first_req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	if (host->dma_type)
		omap_hsmmc_map_data(host, data);
}

/*
 * Request function. for read/write operation
 */
static void omap_hsmmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int err;

	BUG_ON(host->req_in_progress);
	BUG_ON(host->dma_ch != -1);
	if (host->protect_card) {
		if (host->reqs_blocked < 3) {
			/*
			 * Ensure the controller is left in a consistent
			 * state by resetting the command and data state
			 * machines.
			 */
			omap_hsmmc_reset_controller_fsm(host, SRD);
			omap_hsmmc_reset_controller_fsm(host, SRC);
			host->reqs_blocked += 1;
		}
		req->cmd->error = -EBADF;
		if (req->data)
			req->data->error = -EBADF;
		req->cmd->retries = 0;
		omap_hsmmc_request_done(host, req);
		return;
	} else if (host->reqs_blocked)
		host->reqs_blocked = 0;
	WARN_ON(host->mrq != NULL);

	/*
	 * Because of OMAP4 Silicon errata (i705), we have to turn off the
	 * PBIAS and VMMC for SD card as soon as we get card disconnect
	 * interrupt. Because of this, we don't wait for all higher layer
	 * structures to be dismantled before turning off power. Because
	 * of this, we might end up here even after SD card is removed
	 * and VMMC and PBIAS are turned off. In that case, just fail
	 * the commands immediately
	 */
	if (host->power_mode == MMC_POWER_OFF) {
		req->cmd->error = -EIO;
		if (req->data)
			req->data->error = -EIO;
		dev_warn(mmc_dev(host->mmc),
			"Card is no longer present\n");
		omap_hsmmc_request_done(host, req);
		return;
	}

	host->mrq = req;

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);

	if (req->sbc) {
		omap_hsmmc_start_command(host, req->sbc, NULL, 0);
		return;
	}

	err = omap_hsmmc_prepare_data(host, req);
	if (err) {
		req->cmd->error = err;
		if (req->data)
			req->data->error = err;
		omap_hsmmc_request_done(host, req);
		return;
	}

	omap_hsmmc_start_command(host, req->cmd, req->data, 0);
}

/* Routine to configure clock values. Exposed API to core */
static void omap_hsmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int do_send_init_stream = 0;

	pm_runtime_get_sync(host->dev);

	if (ios->power_mode != host->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			mmc_slot(host).set_power(host->dev, host->slot_id,
						 0, 0);
			host->vdd = 0;
			break;
		case MMC_POWER_UP:
			mmc_slot(host).set_power(host->dev, host->slot_id,
						 1, ios->vdd);
			host->vdd = ios->vdd;
			break;
		case MMC_POWER_ON:
			do_send_init_stream = 1;
			break;
		}
		host->power_mode = ios->power_mode;
	}

	/* FIXME: set registers based only on changes to ios */

	omap_hsmmc_set_bus_width(host);

	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		/* Only MMC1 can interface at 3V without some flavor
		 * of external transceiver; but they all handle 1.8V.
		 */
		if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
			(ios->vdd == DUAL_VOLT_OCR_BIT) &&
			/*
			 * With pbias cell programming missing, this
			 * can't be allowed when booting with device
			 * tree.
			 */
			!host->dev->of_node) {
				/*
				 * The mmc_select_voltage fn of the core does
				 * not seem to set the power_mode to
				 * MMC_POWER_UP upon recalculating the voltage.
				 * vdd 1.8v.
				 */
			if (omap_hsmmc_switch_opcond(host, ios->vdd) != 0)
				dev_dbg(mmc_dev(host->mmc),
						"Switch operation failed\n");
		}
	}

	omap_hsmmc_set_clock(host);

	if (do_send_init_stream)
		send_init_stream(host);

	omap_hsmmc_set_bus_mode(host);

	pm_runtime_put_autosuspend(host->dev);
}

static int omap_hsmmc_get_cd(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (!mmc_slot(host).card_detect)
		return -ENOSYS;
	return mmc_slot(host).card_detect(host->dev, host->slot_id);
}

static int omap_hsmmc_get_ro(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (!mmc_slot(host).get_ro)
		return -ENOSYS;
	return mmc_slot(host).get_ro(host->dev, 0);
}

static void omap_hsmmc_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (mmc_slot(host).init_card)
		mmc_slot(host).init_card(card);
}

static void omap_hsmmc_conf_bus_power(struct omap_hsmmc_host *host)
{
	u32 hctl, capa, value;

	/* Only MMC1 supports 3.0V */
	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	if (host->dma_type == ADMA_XFER)
		hctl |= DMAS;

	value = OMAP_HSMMC_READ(host->base, HCTL) & ~SDVS_MASK;
	OMAP_HSMMC_WRITE(host->base, HCTL, value | hctl);

	if (host->dma_type == ADMA_XFER) {
		value = OMAP_HSMMC_READ(host->base, CON);
		OMAP_HSMMC_WRITE(host->base, CON, value | DMA_MNS_ADMA_MODE);
	}

	value = OMAP_HSMMC_READ(host->base, CAPA);
	OMAP_HSMMC_WRITE(host->base, CAPA, value | capa);

	/* Set SD bus power bit */
	set_sd_bus_power(host);
}

static int omap_hsmmc_enable_fclk(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	pm_runtime_get_sync(host->dev);

	return 0;
}

static int omap_hsmmc_disable_fclk(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;
}

static int omap_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct omap_hsmmc_host *host;
	struct mmc_ios *ios = &mmc->ios;
	int phase_delay = 0;
	int err = 0;
	int count = 0;
	int length = 0;
	int note_index = 0xFF;
	int max_index = 0;
	int max_window = 0;
	bool previous_match = false;
	bool current_match;
	u32 ac12, capa2, dll;

	host  = mmc_priv(mmc);
	host->tuning_done = 0;
	/* clock tuning is not needed for upto 52MHz */
	if (ios->clock <= EMMC_HSDDR_SD_SDR25_MAX)
		return 0;

	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	capa2 = OMAP_HSMMC_READ(host->base, CAPA2);

	ac12 &= ~AC12_UHSMC_MASK;
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	/*
	 * Host Controller needs tuning only in case of SDR104 mode
	 * and for SDR50 mode when Use Tuning for SDR50 is set in
	 * Capabilities register.
	 */
	if (ios->clock <= SD_SDR50_MAX_FREQ) {
		if (!(capa2 & CAPA2_TSDR50))
			return 0;
		ac12 |= AC12_UHSMC_SDR50;
	} else
		ac12 |= AC12_UHSMC_SDR104;

	/* Enable SDR50/SDR104 mode */
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	/* Select the DLL clock */
	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll |= DLL_FORCE_VALUE;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	/* Start software tuning Procedure */
	dll |= DLL_SWT;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	/* HACK: Toshiba EMMC HS200 part is not responding
	 * to CMD21. Debug with Toshiba on-going. This issue
	 * is more of protocol handling issue than a silicon
	 * issue. This hack must be removed after debug with
	 * Toshiba.
	 * Temp Fix: We hard code the DLL delay for receive
	 * sampling clock and bypass tuning command CMD21
	 */
	if (opcode == MMC_SEND_TUNING_BLOCK_HS200)
		goto manual_tuning;
	/*
	 * As per the Host Controller spec v3.00, tuning command
	 * generates Buffer Read Ready interrupt, so enable that.
	 * Issue tuning command repeatedly till AC12.ET is cleared
	 * Tuning should maximum of 8 iterations
	 */
	do {
		struct mmc_command cmd = {0};
		struct mmc_request mrq = {0};

		if (phase_delay > MAX_PHASE_DELAY)
			break;

		omap_hsmmc_set_dll(host, phase_delay);

		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.retries = 0;
		cmd.data = NULL;
		cmd.error = 0;

		mrq.cmd = &cmd;
		host->mrq = &mrq;

		OMAP_HSMMC_WRITE(host->base, BLK, 64 | (1 << 16));
		set_data_timeout(host, 50000000, 0);
		OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
		omap_hsmmc_start_command(host, &cmd, NULL, 0);

		host->cmd = NULL;
		host->mrq = NULL;

		/* Wait for Buffer Read Ready interrupt */
		err = wait_for_completion_timeout(&host->buf_ready,
					msecs_to_jiffies(5000));
		omap_hsmmc_disable_irq(host);
		host->req_in_progress = 0;

		if (err == 0) {
			dev_err(mmc_dev(host->mmc),
				"Tuning BRR timeout. phase_delay=%x",
				phase_delay);
			err = -ETIMEDOUT;
			goto tuning_error;
		}

		current_match = true;
		if (memcmp(tuning_data, ref_tuning, sizeof(tuning_data)))
			current_match = false;
		else
			current_match = true;

		if (current_match == true) {
			if (previous_match == false) {
				/* new window */
				note_index = count;
				length = 1;
			} else {
				length++;
			}
			previous_match = true;
			if (length > max_window) {
				max_index = note_index;
				max_window = length;
			}
		} else {
			previous_match = false;
		}

		phase_delay += 4;
		ac12 = OMAP_HSMMC_READ(host->base, AC12);
	} while (ac12 & AC12_SCLK_SEL);

	if (note_index == 0xFF) {
		dev_err(mmc_dev(host->mmc), "Unable to find match\n");
		goto tuning_error;
	}

	if (ac12 & AC12_SCLK_SEL) {
manual_tuning:
		if (opcode == MMC_SEND_TUNING_BLOCK_HS200)
			count = 16;
		else
			count = 2 * (max_index + (max_window >> 1));
		dll = OMAP_HSMMC_READ(host->base, DLL);
		dll &= ~DLL_SWT;
		OMAP_HSMMC_WRITE(host->base, DLL, dll);
		count = 2 * (max_index + (max_window >> 1));
		if (omap_hsmmc_set_dll(host, count)) {
			err = -EIO;
			goto tuning_error;
		}
		host->tuning_fsrc = count;
		host->tuning_uhsmc = (OMAP_HSMMC_READ(host->base, AC12)
					& AC12_UHSMC_MASK);
		host->tuning_opcode = opcode;
		host->tuning_done = 1;
		omap_hsmmc_reset_controller_fsm(host, SRD);
		omap_hsmmc_reset_controller_fsm(host, SRC);
		return 0;
	} else {
		dev_err(mmc_dev(host->mmc),
			"Tuning failed. Using fixed sampling clock\n");
		err = -EIO;
		goto tuning_error;
	}
tuning_error:
	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 &= ~(AC12_UHSMC_MASK | AC12_SCLK_SEL);
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll &= ~(DLL_FORCE_VALUE | DLL_SWT);
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	omap_hsmmc_reset_controller_fsm(host, SRD);
	omap_hsmmc_reset_controller_fsm(host, SRC);
	return err;
}

static int omap_start_signal_voltage_switch(struct mmc_host *mmc,
			struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host;
	u32 value = 0;
	unsigned long timeout;
	unsigned long notimeout = 0;

	if (!(mmc->caps & (MMC_CAP_UHS_SDR12 |
			MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_DDR50)))
		return 0;

	host  = mmc_priv(mmc);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		omap_hsmmc_conf_bus_power(host);

		value = OMAP_HSMMC_READ(host->base, AC12);
		value &= ~OMAP_V1V8_SIGEN_V1V8;
		OMAP_HSMMC_WRITE(host->base, AC12, value);
		dev_dbg(mmc_dev(host->mmc), " i/o voltage switch to 3V\n");
		return 0;
	}

	if (mmc_slot(host).clk_pull_up)
		mmc_slot(host).clk_pull_up(host->dev, host->slot_id, false);

	value = OMAP_HSMMC_READ(host->base, CON);
	OMAP_HSMMC_WRITE(host->base, CON, (value | PADEN));

	value = OMAP_HSMMC_READ(host->base, PSTATE);
	timeout = jiffies + msecs_to_jiffies(50);
	do {
		if (!(value & CLEV)) {
			notimeout = 1;
			break;
		}
		usleep_range(100, 200);
		value = OMAP_HSMMC_READ(host->base, PSTATE);
	} while (!time_after(jiffies, timeout));
	if (!notimeout)
		dev_err(mmc_dev(host->mmc), "timeout wait for clev 0\n");

	notimeout = 0;
	timeout = jiffies + msecs_to_jiffies(50);
	do {
		if (!(value & DLEV)) {
			notimeout = 1;
			break;
		}
		usleep_range(100, 200);
		value = OMAP_HSMMC_READ(host->base, PSTATE);
	} while (!time_after(jiffies, timeout));
	if (!notimeout)
		dev_err(mmc_dev(host->mmc), "timeout wait for dlev 0\n");

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		value = OMAP_HSMMC_READ(host->base, HCTL);
		value &= ~SDVS_MASK;
		value |= SDVS18;
		OMAP_HSMMC_WRITE(host->base, HCTL, value);
		value |= SDBP;
		OMAP_HSMMC_WRITE(host->base, HCTL, value);

		mmc_slot(host).set_power(host->dev, host->slot_id,
						 1, VDD_165_195);
		dev_dbg(mmc_dev(host->mmc), "i/o voltage switch to 1.8v\n\n");
	}

	if (mmc_slot(host).clk_pull_up)
		mmc_slot(host).clk_pull_up(host->dev, host->slot_id, true);

	value = OMAP_HSMMC_READ(host->base, CON);
	OMAP_HSMMC_WRITE(host->base, CON, (value | CLKEXTFREE));

	value = OMAP_HSMMC_READ(host->base, PSTATE);

	notimeout = 0;
	timeout = jiffies + msecs_to_jiffies(50);
	do {
		if ((value & CLEV) == CLEV) {
			notimeout = 1;
			break;
		}
		usleep_range(100, 200);
		value = OMAP_HSMMC_READ(host->base, PSTATE);
	} while (!time_after(jiffies, timeout));
	if (!notimeout)
		dev_err(mmc_dev(host->mmc), "timeout wait for clev 1\n");

	notimeout = 0;
	timeout = jiffies + msecs_to_jiffies(50);
	do {
		if ((value & DLEV) == DLEV) {
			notimeout = 1;
			break;
		}
		usleep_range(100, 200);
		value = OMAP_HSMMC_READ(host->base, PSTATE);
	} while (!time_after(jiffies, timeout));
	if (!notimeout)
		dev_err(mmc_dev(host->mmc), "timeout wait for dlev 1\n");

	value = OMAP_HSMMC_READ(host->base, CON);
	OMAP_HSMMC_WRITE(host->base, CON, (value & ~(CLKEXTFREE | PADEN)));

	return 0;
}

static const struct mmc_host_ops omap_hsmmc_ops = {
	.enable = omap_hsmmc_enable_fclk,
	.disable = omap_hsmmc_disable_fclk,
	.post_req = omap_hsmmc_post_req,
	.pre_req = omap_hsmmc_pre_req,
	.request = omap_hsmmc_request,
	.set_ios = omap_hsmmc_set_ios,
	.get_cd = omap_hsmmc_get_cd,
	.get_ro = omap_hsmmc_get_ro,
	.init_card = omap_hsmmc_init_card,
	.start_signal_voltage_switch = omap_start_signal_voltage_switch,
	.execute_tuning = omap_execute_tuning,
	/* NYET -- enable_sdio_irq */
};

#ifdef CONFIG_DEBUG_FS

static int omap_hsmmc_regs_show(struct seq_file *s, void *data)
{
	struct mmc_host *mmc = s->private;
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int context_loss = 0;

	if (host->pdata->get_context_loss_count)
		context_loss = host->pdata->get_context_loss_count(host->dev);

	seq_printf(s, "mmc%d:\n ctx_loss:\t%d:%d\n fclk:\t\t%lu Hz\n\nregs:\n",
			mmc->index, host->context_loss, context_loss,
			clk_get_rate(host->fclk));

	if (host->suspended) {
		seq_printf(s, "host suspended, can't read registers\n");
		return 0;
	}

	pm_runtime_get_sync(host->dev);

	seq_printf(s, "SYSCONFIG:\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, SYSCONFIG));
	seq_printf(s, "CON:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CON));
	seq_printf(s, "HCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, HCTL));
	seq_printf(s, "SYSCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, SYSCTL));
	seq_printf(s, "IE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, IE));
	seq_printf(s, "ISE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, ISE));
	seq_printf(s, "CAPA:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CAPA));

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;
}

static int omap_hsmmc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_hsmmc_regs_show, inode->i_private);
}

static const struct file_operations mmc_regs_fops = {
	.open           = omap_hsmmc_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void omap_hsmmc_debugfs(struct mmc_host *mmc)
{
	if (mmc->debugfs_root)
		debugfs_create_file("regs", S_IRUSR, mmc->debugfs_root,
			mmc, &mmc_regs_fops);
}

#else

static void omap_hsmmc_debugfs(struct mmc_host *mmc)
{
}

#endif

#ifdef CONFIG_OF
static u16 omap4_reg_offset = 0x100;

static const struct of_device_id omap_mmc_of_match[] = {
	{
		.compatible = "ti,omap2-hsmmc",
	},
	{
		.compatible = "ti,omap3-hsmmc",
	},
	{
		.compatible = "ti,omap4-hsmmc",
		.data = &omap4_reg_offset,
	},
	{},
};
MODULE_DEVICE_TABLE(of, omap_mmc_of_match);

static struct omap_mmc_platform_data *of_get_hsmmc_pdata(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	u32 bus_width;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL; /* out of memory */

	if (of_find_property(np, "ti,dual-volt", NULL))
		pdata->controller_flags |= OMAP_HSMMC_SUPPORTS_DUAL_VOLT;

	/* This driver only supports 1 slot */
	pdata->nr_slots = 1;
	pdata->slots[0].switch_pin = of_get_named_gpio(np, "cd-gpios", 0);
	pdata->slots[0].gpio_wp = of_get_named_gpio(np, "wp-gpios", 0);

	if (of_find_property(np, "ti,non-removable", NULL)) {
		pdata->slots[0].nonremovable = true;
		pdata->slots[0].no_regulator_off_init = true;
	}
	of_property_read_u32(np, "ti,bus-width", &bus_width);
	if (bus_width == 4)
		pdata->slots[0].caps |= MMC_CAP_4_BIT_DATA;
	else if (bus_width == 8)
		pdata->slots[0].caps |= MMC_CAP_8_BIT_DATA;

	return pdata;
}
#else
static inline struct omap_mmc_platform_data
			*of_get_hsmmc_pdata(struct device *dev)
{
	return NULL;
}
#endif

static int __devinit omap_hsmmc_probe(struct platform_device *pdev)
{
	struct omap_mmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct omap_hsmmc_host *host = NULL;
	struct resource *res;
	int ret, irq;
	int ctrlr_caps = 0;
	const struct of_device_id *match;

	match = of_match_device(of_match_ptr(omap_mmc_of_match), &pdev->dev);
	if (match) {
		pdata = of_get_hsmmc_pdata(&pdev->dev);
		if (match->data) {
			u16 *offsetp = match->data;
			pdata->reg_offset = *offsetp;
		}
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev, "Platform Data is missing\n");
		return -ENXIO;
	}

	if (pdata->nr_slots == 0) {
		dev_err(&pdev->dev, "No Slots\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL)
		return -EBUSY;

	ret = omap_hsmmc_gpio_init(pdata);
	if (ret)
		goto err;

	mmc = mmc_alloc_host(sizeof(struct omap_hsmmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	host		= mmc_priv(mmc);
	host->mmc	= mmc;
	host->pdata	= pdata;
	host->dev	= &pdev->dev;
	host->dma_type	= SDMA_XFER;
	host->dev->dma_mask = &pdata->dma_mask;
	host->dma_ch	= -1;
	host->irq	= irq;
	host->slot_id	= 0;
	host->mapped_data_cnt = 0;
	host->mapbase	= res->start + pdata->reg_offset;
	host->base	= ioremap(host->mapbase, SZ_4K);
	if (!host->base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}
	host->power_mode = MMC_POWER_OFF;
	host->flags	= AUTO_CMD12;
	host->errata = 0;
	if (cpu_is_omap44xx())
		host->errata |= OMAP_HSMMC_ERRATA_I761;
	if (cpu_is_omap44xx() && (omap_rev() > OMAP4430_REV_ES1_0))
		host->errata |= OMAP_HSMMC_ERRATA_FSMR;

	host->regulator_enabled = 0;
	pdata->dev = host->dev;

	platform_set_drvdata(pdev, host);

	mmc->ops	= &omap_hsmmc_ops;

	/*
	 * If regulator_disable can only put vcc_aux to sleep then there is
	 * no off state.
	 */
	if (mmc_slot(host).vcc_aux_disable_is_sleep)
		mmc_slot(host).no_off = 1;

	mmc->f_min = pdata->min_freq;
	mmc->f_max = pdata->max_freq;

	spin_lock_init(&host->irq_lock);
	init_completion(&host->buf_ready);

	host->fclk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(host->fclk)) {
		ret = PTR_ERR(host->fclk);
		host->fclk = NULL;
		goto err1;
	}
	pdata->fclk = host->fclk;
	if (pdata->opp_scale_init)
		pdata->opp_scale_init(pdata);

	if (host->pdata->controller_flags & OMAP_HSMMC_BROKEN_MULTIBLOCK_READ) {
		dev_info(&pdev->dev, "multiblock reads disabled due to 35xx erratum 2.1.1.128; MMC read performance may suffer\n");
		mmc->caps2 |= MMC_CAP2_NO_MULTI_READ;
	}

	pm_runtime_enable(host->dev);
	pm_runtime_get_sync(host->dev);
	pm_runtime_set_autosuspend_delay(host->dev, MMC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(host->dev);

	omap_hsmmc_context_save(host);

	host->dbclk = clk_get(&pdev->dev, "mmchsdb_fck");
	/*
	 * MMC can still work without debounce clock.
	 */
	if (IS_ERR(host->dbclk)) {
		dev_warn(mmc_dev(host->mmc), "Failed to get debounce clk\n");
		host->dbclk = NULL;
	} else if (clk_enable(host->dbclk) != 0) {
		dev_warn(mmc_dev(host->mmc), "Failed to enable debounce clk\n");
		clk_put(host->dbclk);
		host->dbclk = NULL;
	}

	ctrlr_caps = OMAP_HSMMC_READ(host->base, CAPA);
	if (ctrlr_caps & CAPA_ADMA_SUPPORT) {
		/* FIXME: passing the device structure fails
		 * due to unset conherency mask
		 */
		host->adma_table = dma_alloc_coherent(NULL,
			ADMA_TABLE_SZ, &host->phy_adma_table, 0);
		if (host->adma_table != NULL)
			host->dma_type = ADMA_XFER;
	}
	dev_dbg(mmc_dev(host->mmc), "DMA Mode=%d\n", host->dma_type);

	/* Since we do only SG emulation, we can have as many segs
	 * as we want. */
	mmc->max_blk_size = HSMMC_BLK_SIZE;
	if (host->dma_type == ADMA_XFER) {
		/* Worst case is when above block layer gives us 512 segments,
		 * in which there are 511 single block entries, but one large
		 * block that is of size mmc->max_req_size - (511*512) bytes.
		 * In this case, we use the reserved 512 table entries to
		 * break up the large request. This is also the reason why we
		 * say we can only handle DMA_TABLE_NUM_ENTRIES/2
		 * segments instead of DMA_TABLE_NUM_ENTRIES.
		 */
		mmc->max_segs = DMA_TABLE_NUM_ENTRIES / 2;
		mmc->max_blk_count = ADMA_MAX_BLKS_PER_ROW *
					DMA_TABLE_NUM_ENTRIES / 2;
		mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	} else {
		mmc->max_segs = DMA_TABLE_NUM_ENTRIES;
		/* No. of Blocks is 16 bits */
		mmc->max_blk_count = 0xFFFF;
		mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	}
	mmc->max_seg_size = mmc->max_req_size;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
		     MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_ERASE | MMC_CAP_CMD23;

	mmc->caps |= mmc_slot(host).caps;
	if (mmc->caps & MMC_CAP_8_BIT_DATA)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (mmc_slot(host).nonremovable)
		mmc->caps |= MMC_CAP_NONREMOVABLE;

	mmc->caps2 |= mmc_slot(host).caps2;

	mmc->pm_caps = mmc_slot(host).pm_caps;

	mmc->caps |= MMC_CAP_DRIVER_TYPE_A | MMC_CAP_DRIVER_TYPE_C |
			MMC_CAP_DRIVER_TYPE_D | MMC_CAP_MAX_CURRENT_800 |
			MMC_CAP_MAX_CURRENT_600 | MMC_CAP_MAX_CURRENT_400 |
			MMC_CAP_MAX_CURRENT_200;

	omap_hsmmc_conf_bus_power(host);

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
	if (!res) {
		dev_err(mmc_dev(host->mmc), "cannot get DMA TX channel\n");
		goto err_irq;
	}
	host->dma_line_tx = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	if (!res) {
		dev_err(mmc_dev(host->mmc), "cannot get DMA RX channel\n");
		goto err_irq;
	}
	host->dma_line_rx = res->start;

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, omap_hsmmc_irq, 0,
			mmc_hostname(mmc), host);
	if (ret) {
		dev_dbg(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ\n");
		goto err_irq;
	}

	if (pdata->init != NULL) {
		if (pdata->init(&pdev->dev) != 0) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to configure MMC IRQs\n");
			goto err_irq_cd_init;
		}
	}

	if (omap_hsmmc_have_reg() && !mmc_slot(host).set_power) {
		ret = omap_hsmmc_reg_get(host);
		if (ret)
			goto err_reg;
		host->use_reg = 1;
	}

	mmc->ocr_avail = mmc_slot(host).ocr_mask;

	/* Request IRQ for card detect */
	if ((mmc_slot(host).card_detect_irq)) {
		ret = request_threaded_irq(mmc_slot(host).card_detect_irq,
					   NULL,
					   omap_hsmmc_detect,
					   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   mmc_hostname(mmc), host);
		if (ret) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to grab MMC CD IRQ\n");
			goto err_irq_cd;
		}
		pdata->suspend = omap_hsmmc_suspend_cdirq;
		pdata->resume = omap_hsmmc_resume_cdirq;
	} else if (mmc_slot(host).mmc_data.register_status_notify) {
		mmc_slot(host).mmc_data.register_status_notify(omap_hsmmc_status_notify_cb, host);
	}

	omap_hsmmc_disable_irq(host);

	omap_hsmmc_protect_card(host);

	mmc_add_host(mmc);

	if (mmc_slot(host).name != NULL) {
		ret = device_create_file(&mmc->class_dev, &dev_attr_slot_name);
		if (ret < 0)
			goto err_slot_name;
	}
	if (mmc_slot(host).card_detect_irq && mmc_slot(host).get_cover_state) {
		ret = device_create_file(&mmc->class_dev,
					&dev_attr_cover_switch);
		if (ret < 0)
			goto err_slot_name;
	}

	omap_hsmmc_debugfs(mmc);
	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;

err_slot_name:
	mmc_remove_host(mmc);
	free_irq(mmc_slot(host).card_detect_irq, host);
err_irq_cd:
	if (host->use_reg)
		omap_hsmmc_reg_put(host);
err_reg:
	if (host->pdata->cleanup)
		host->pdata->cleanup(&pdev->dev);
err_irq_cd_init:
	free_irq(host->irq, host);
err_irq:
	pm_runtime_put_sync(host->dev);
	pm_runtime_disable(host->dev);
	if (host->dbclk) {
		clk_disable(host->dbclk);
		clk_put(host->dbclk);
	}
	clk_put(host->fclk);
	host->fclk = NULL;
err1:
	if (host->adma_table != NULL)
		dma_free_coherent(NULL, ADMA_TABLE_SZ,
			host->adma_table, host->phy_adma_table);
	iounmap(host->base);
err_ioremap:
	platform_set_drvdata(pdev, NULL);
	mmc_free_host(mmc);
err_alloc:
	omap_hsmmc_gpio_free(pdata);
err:
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	return ret;
}

static int __devexit omap_hsmmc_remove(struct platform_device *pdev)
{
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);
	struct resource *res;

	pm_runtime_get_sync(host->dev);
	mmc_remove_host(host->mmc);
	if (host->use_reg)
		omap_hsmmc_reg_put(host);
	if (host->pdata->cleanup)
		host->pdata->cleanup(&pdev->dev);
	free_irq(host->irq, host);
	if (mmc_slot(host).card_detect_irq)
		free_irq(mmc_slot(host).card_detect_irq, host);
	if (host->adma_table != NULL)
		dma_free_coherent(NULL, ADMA_TABLE_SZ,
			host->adma_table, host->phy_adma_table);
	pm_runtime_put_sync(host->dev);
	pm_runtime_disable(host->dev);
	clk_put(host->fclk);
	if (host->dbclk) {
		clk_disable(host->dbclk);
		clk_put(host->dbclk);
	}

	mmc_free_host(host->mmc);
	iounmap(host->base);
	omap_hsmmc_gpio_free(pdev->dev.platform_data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void omap_hsmmc_shutdown(struct platform_device *pdev)
{
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);
	long delay;

	/* Protection from forgotten wakelock */
	wake_unlock(&host->mmc->detect_wake_lock);

	/* If require housekeeping and has minimum one request */
	delay = mmc_slot(host).housekeeping_ms -
			jiffies_to_msecs(jiffies - host->req_ts);
	if (delay > 0) {
		/* Block all requests */
		host->protect_card = 1;
		dev_info(&pdev->dev, "remain housekeeping %ld ms\n", delay);
		msleep(delay);
	}
}

#ifdef CONFIG_PM
static int omap_hsmmc_suspend(struct device *dev)
{
	int ret = 0;
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (!host)
		return 0;

	if (host && host->suspended)
		return 0;

	if (host->req_in_progress)
		return -EBUSY;

	pm_runtime_get_sync(host->dev);
	host->suspended = 1;
	if (host->pdata->suspend) {
		ret = host->pdata->suspend(dev, host->slot_id);
		if (ret) {
			dev_dbg(dev, "Unable to handle MMC board"
					" level suspend\n");
			host->suspended = 0;
			return ret;
		}
	}
	if (mmc_slot(host).built_in)
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
	ret = mmc_suspend_host(host->mmc);

	if (ret) {
		host->suspended = 0;
		if (host->pdata->resume) {
			if (host->pdata->resume(dev, host->slot_id))
				dev_dbg(dev, "Unmask interrupt failed\n");
		}
		goto err;
	}
	host->tuning_done = 0;

	if (!(host->mmc->pm_flags & MMC_PM_KEEP_POWER)) {
		omap_hsmmc_disable_irq(host);
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base, HCTL) & ~SDBP);
	}

	if (host->dbclk)
		clk_disable(host->dbclk);
err:
	pm_runtime_put_sync(host->dev);
	return ret;
}

/* Routine to resume the MMC device */
static int omap_hsmmc_resume(struct device *dev)
{
	int ret = 0;
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (!host)
		return 0;

	if (host && !host->suspended)
		return 0;

	pm_runtime_get_sync(host->dev);

	if (host->dbclk)
		clk_enable(host->dbclk);

	if (!(host->mmc->pm_flags & MMC_PM_KEEP_POWER))
		omap_hsmmc_conf_bus_power(host);

	if (host->pdata->resume) {
		ret = host->pdata->resume(dev, host->slot_id);
		if (ret)
			dev_dbg(dev, "Unmask interrupt failed\n");
	}

	omap_hsmmc_protect_card(host);

	/* Notify the core to resume the host */
	ret = mmc_resume_host(host->mmc);
	if (ret == 0)
		host->suspended = 0;

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return ret;

}

#else
#define omap_hsmmc_suspend	NULL
#define omap_hsmmc_resume		NULL
#endif

static int omap_hsmmc_runtime_suspend(struct device *dev)
{
	struct omap_hsmmc_host *host;

	host = platform_get_drvdata(to_platform_device(dev));
	omap_hsmmc_context_save(host);
	dev_dbg(dev, "disabled\n");

	return 0;
}

static int omap_hsmmc_runtime_resume(struct device *dev)
{
	struct omap_hsmmc_host *host;

	host = platform_get_drvdata(to_platform_device(dev));
	omap_hsmmc_context_restore(host);
	dev_dbg(dev, "enabled\n");

	return 0;
}

static struct dev_pm_ops omap_hsmmc_dev_pm_ops = {
	.suspend	= omap_hsmmc_suspend,
	.resume		= omap_hsmmc_resume,
	.runtime_suspend = omap_hsmmc_runtime_suspend,
	.runtime_resume = omap_hsmmc_runtime_resume,
};

static struct platform_driver omap_hsmmc_driver = {
	.probe		= omap_hsmmc_probe,
	.remove		= __devexit_p(omap_hsmmc_remove),
	.shutdown	= omap_hsmmc_shutdown,
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &omap_hsmmc_dev_pm_ops,
		.of_match_table = of_match_ptr(omap_mmc_of_match),
	},
};

module_platform_driver(omap_hsmmc_driver);
MODULE_DESCRIPTION("OMAP High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
