/**
 * Copyright (c) 2011 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/types.h>
#include <linux/compiler.h>
#include <iomap.h>

#include "tf_defs.h"
#include "tf_util.h"
#include "tf_zebra.h"
#include "tf_crypto.h"
#include "tf_dma.h"

#define IO_ADDRESS OMAP2_L4_IO_ADDRESS

#define PUBLIC_CRYPTO_TIMEOUT_CONST    0x000FFFFF

#define ENABLE_CLOCK	true
#define DISABLE_CLOCK	false

/*------------------------------------------------------------------------- */
/**
 *Initialize the public crypto DMA channels, global HWA semaphores and handles
 */
u32 tf_crypto_init(void)
{
	u32 error = 0;
#ifdef CONFIG_SMC_KERNEL_CRYPTO
	struct tf_device *dev = tf_get_device();

	/*initialize the DMA semaphores */
	mutex_init(&dev->sm.dma_mutex);

	/*allocate DMA buffer */
	dev->dma_buffer_length = PAGE_SIZE;
	dev->dma_buffer = dma_alloc_coherent(NULL,
		dev->dma_buffer_length,
		&(dev->dma_buffer_phys),
		GFP_KERNEL);
	if (dev->dma_buffer == NULL) {
		printk(KERN_ERR
			"tf_crypto_init: Out of memory for DMA buffer\n");
		error = S_ERROR_OUT_OF_MEMORY;
	}
#endif

	return error;
}

/*------------------------------------------------------------------------- */
/**
 *Terminate the public crypto (including DMA)
 */
void tf_crypto_terminate(void)
{
#ifdef CONFIG_SMC_KERNEL_CRYPTO
	struct tf_device *dev = tf_get_device();

	if (dev->dma_buffer != NULL) {
		dma_free_coherent(NULL, dev->dma_buffer_length,
			dev->dma_buffer,
			dev->dma_buffer_phys);
		dev->dma_buffer = NULL;
	}
#endif
}

/*------------------------------------------------------------------------- */

void tf_crypto_wait_for_ready_bit_infinitely(u32 *reg, u32 bit)
{
	while (!(INREG32(reg) & bit))
		;
}

/*------------------------------------------------------------------------- */

u32 tf_crypto_wait_for_ready_bit(u32 *reg, u32 bit)
{
	u32 timeoutCounter = PUBLIC_CRYPTO_TIMEOUT_CONST;

	while ((!(INREG32(reg) & bit)) && ((--timeoutCounter) != 0))
		;

	if (timeoutCounter == 0)
		return PUBLIC_CRYPTO_ERR_TIMEOUT;

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*------------------------------------------------------------------------- */

static DEFINE_SPINLOCK(clk_lock);

u32 tf_crypto_turn_off_clocks(void)
{
	unsigned long flags;
	u32 ret = 0xf;

	spin_lock_irqsave(&clk_lock, flags);

	ret = tf_try_disabling_secure_hwa_clocks(0xf) & 0xff;

	spin_unlock_irqrestore(&clk_lock, flags);

	if (ret == 0xff)
		panic("Error calling API_HAL_HWATURNOFF_INDEX");

	return ret;
}

void tf_crypto_disable_clock(uint32_t clock_paddr)
{
	dprintk(KERN_INFO "tf_crypto_disable_clock: " \
		"clock_paddr=0x%08X\n",
		clock_paddr);

	tf_clock_timer_stop();
}

/*------------------------------------------------------------------------- */

void tf_crypto_enable_clock(uint32_t clock_paddr)
{
	u32 *clock_reg;
	u32 val;
	unsigned long flags;

	dprintk(KERN_INFO "tf_crypto_enable_clock: " \
		"clock_paddr=0x%08X\n",
		clock_paddr);

	/* Ensure none concurrent access when changing clock registers */

	tf_clock_timer_start();

	spin_lock_irqsave(&clk_lock, flags);

	clock_reg = (u32 *)IO_ADDRESS(clock_paddr);

	val = __raw_readl(clock_reg);

	if ((val & 0x30000) == 0)
		goto end;

	val |= 0x2;
	__raw_writel(val, clock_reg);

	/* Wait for clock to be fully enabled */
	while ((__raw_readl(clock_reg) & 0x30000) != 0)
		;

end:
	spin_unlock_irqrestore(&clk_lock, flags);
}
