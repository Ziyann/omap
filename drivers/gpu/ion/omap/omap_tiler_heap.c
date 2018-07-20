/*
 * drivers/gpu/ion/omap_tiler_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
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
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/mm.h>
#include <linux/omap_ion.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "../../../drivers/staging/omapdrm/omap_dmm_tiler.h"
#include <asm/mach/map.h>
#include <asm/page.h>


#include "../ion_priv.h"
#include "omap_ion_priv.h"
#include <asm/cacheflush.h>

static int omap_tiler_heap_allocate(struct ion_heap *heap,
				    struct ion_buffer *buffer,
				    unsigned long size, unsigned long align,
				    unsigned long flags)
{
	if (size == 0)
		return 0;

	pr_err("%s: This should never be called directly -- use the "
			"OMAP_ION_TILER_ALLOC flag to the ION_IOC_CUSTOM "
			"instead\n", __func__);
	return -EINVAL;
}

struct omap_tiler_info {
	struct tiler_block *tiler_handle;	/* handle of the allocation
						   intiler */
	bool lump;			/* true for a single lump allocation */
	u32 n_phys_pages;		/* number of physical pages */
	u32 *phys_addrs;		/* array addrs of pages */
	u32 n_tiler_pages;		/* number of tiler pages */
	u32 *tiler_addrs;		/* array of addrs of tiler pages */
	int fmt;			/* tiler buffer format */
	u32 tiler_start;		/* start addr in tiler -- if not page
					   aligned this may not equal the
					   first entry onf tiler_addrs */
	u32 vsize;			/* virtual stride of buffer */
	u32 vstride;			/* virtual size of buffer */
};

int omap_tiler_alloc(struct ion_heap *heap,
		     struct ion_client *client,
		     struct omap_ion_tiler_alloc_data *data)
{
	struct ion_handle *handle;
	struct ion_buffer *buffer;
	struct omap_tiler_info *info = NULL;
	u32 n_phys_pages;
	u32 n_tiler_pages;
	ion_phys_addr_t addr = 0;
	int i = 0, ret;
	uint32_t phys_stride, remainder;
	dma_addr_t ssptr;

	if (data->fmt == TILFMT_PAGE && data->h != 1) {
		pr_err("%s: Page mode (1D) allocations must have a height of "
				"one\n", __func__);
		return -EINVAL;
	}

	if (data->fmt == TILFMT_PAGE) {
		/* calculate required pages the usual way */
		n_phys_pages = round_up(data->w, PAGE_SIZE) >> PAGE_SHIFT;
		n_tiler_pages = n_phys_pages;
	} else {
		/* call APIs to calculate 2D buffer page requirements */
		n_phys_pages = tiler_size(data->fmt, data->w, data->h) >>
				PAGE_SHIFT;
		n_tiler_pages = tiler_vsize(data->fmt, data->w, data->h) >>
					PAGE_SHIFT;
	}

	info = kzalloc(sizeof(struct omap_tiler_info) +
		       sizeof(u32) * n_phys_pages +
		       sizeof(u32) * n_tiler_pages, GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->n_phys_pages = n_phys_pages;
	info->n_tiler_pages = n_tiler_pages;
	info->phys_addrs = (u32 *)(info + 1);
	info->tiler_addrs = info->phys_addrs + n_phys_pages;
	info->fmt = data->fmt;

	/* Allocate tiler space
	   FIXME: we only support PAGE_SIZE alignment right now. */
	if (data->fmt == TILFMT_PAGE)
		info->tiler_handle = tiler_reserve_1d(data->w);
	else
		info->tiler_handle = tiler_reserve_2d(data->fmt, data->w,
				data->h, PAGE_SIZE);

	if (IS_ERR_OR_NULL(info->tiler_handle)) {
		ret = PTR_ERR(info->tiler_handle);
		pr_err("%s: failure to allocate address space from tiler\n",
		       __func__);
		goto err_got_mem;
	}

	/* get physical address of tiler buffer */
	info->tiler_start = tiler_ssptr(info->tiler_handle);

	/* fill in tiler pages by using ssptr and stride */
	info->vstride = info->tiler_handle->stride;
	info->vsize = n_tiler_pages << PAGE_SHIFT;
	phys_stride = (data->fmt == TILFMT_PAGE) ? info->vstride :
				tiler_stride(info->tiler_start);
	ssptr = info->tiler_start;
	remainder = info->vstride;

	for (i = 0; i < n_tiler_pages; i++) {
		info->tiler_addrs[i] = PAGE_ALIGN(ssptr);
		ssptr += PAGE_SIZE;
		remainder -= PAGE_SIZE;

		/* see if we are done with this line.  If so, go to the next
		   line */
		if (!remainder) {
			remainder = info->vstride;
			ssptr += phys_stride - info->vstride;
		}
	}

	addr = ion_carveout_allocate(heap, n_phys_pages*PAGE_SIZE, 0);
	if (addr == ION_CARVEOUT_ALLOCATE_FAIL) {
		for (i = 0; i < n_phys_pages; i++) {
			addr = ion_carveout_allocate(heap, PAGE_SIZE, 0);

			if (addr == ION_CARVEOUT_ALLOCATE_FAIL) {
				ret = -ENOMEM;
				pr_err("%s: failed to allocate pages to back "
					"tiler address space\n", __func__);
				goto err_got_tiler;
			}
			info->phys_addrs[i] = addr;
		}
	} else {
		info->lump = true;
		for (i = 0; i < n_phys_pages; i++)
			info->phys_addrs[i] = addr + i*PAGE_SIZE;
	}

	ret = tiler_pin_phys(info->tiler_handle, info->phys_addrs,
			info->n_phys_pages);
	if (ret) {
		pr_err("%s: failure to pin pages to tiler\n", __func__);
		goto err_got_tiler;
	}

	data->stride = info->vstride;

	/* create an ion handle  for the allocation */
	handle = ion_alloc(client, 0, 0, 1 << OMAP_ION_HEAP_TILER);
	if (IS_ERR_OR_NULL(handle)) {
		ret = PTR_ERR(handle);
		pr_err("%s: failure to allocate handle to manage "
				"tiler allocation\n", __func__);
		goto err;
	}

	buffer = ion_handle_buffer(handle);
	buffer->size = n_tiler_pages * PAGE_SIZE;
	buffer->priv_virt = info;
	data->handle = handle;
	data->offset = (size_t)(info->tiler_start & ~PAGE_MASK);

	return 0;

err:
	tiler_unpin(info->tiler_handle);
err_got_tiler:
	tiler_release(info->tiler_handle);
	if (info) {
		if (info->lump) {
			ion_carveout_free(heap, addr, n_phys_pages * PAGE_SIZE);
		} else {
			for (i -= 1; i >= 0; i--)
				ion_carveout_free(heap, info->phys_addrs[i],
						PAGE_SIZE);
		}
	}
err_got_mem:
	kfree(info);
	return ret;
}

static void omap_tiler_heap_free(struct ion_buffer *buffer)
{
	struct omap_tiler_info *info = buffer->priv_virt;

	tiler_unpin(info->tiler_handle);
	tiler_release(info->tiler_handle);

	if (info->lump) {
		ion_carveout_free(buffer->heap, info->phys_addrs[0],
				  info->n_phys_pages*PAGE_SIZE);
	} else {
		int i;
		for (i = 0; i < info->n_phys_pages; i++)
			ion_carveout_free(buffer->heap,
					  info->phys_addrs[i], PAGE_SIZE);
	}

	kfree(info);
}

static int omap_tiler_phys(struct ion_heap *heap,
			   struct ion_buffer *buffer,
			   ion_phys_addr_t *addr, size_t *len)
{
	struct omap_tiler_info *info = buffer->priv_virt;

	*addr = info->tiler_start;
	*len = buffer->size;
	return 0;
}

int omap_tiler_pages(struct ion_client *client, struct ion_handle *handle,
		     int *n, u32 **tiler_addrs)
{
	ion_phys_addr_t addr;
	size_t len;
	int ret;
	struct omap_tiler_info *info = ion_handle_buffer(handle)->priv_virt;

	/* validate that the handle exists in this client */
	ret = ion_phys(client, handle, &addr, &len);
	if (ret)
		return ret;

	*n = info->n_tiler_pages;
	*tiler_addrs = info->tiler_addrs;
	return 0;
}

int omap_tiler_vinfo(struct ion_client *client, struct ion_handle *handle,
			unsigned int *vstride, unsigned int *vsize)
{
	struct omap_tiler_info *info = ion_handle_buffer(handle)->priv_virt;

	*vstride = info->vstride;
	*vsize = info->vsize;

	return 0;
}

static int omap_tiler_heap_map_user(struct ion_heap *heap,
		struct ion_buffer *buffer, struct vm_area_struct *vma)
{
	struct omap_tiler_info *info = buffer->priv_virt;
	unsigned long addr = vma->vm_start;
	u32 vma_pages = (vma->vm_end - vma->vm_start) / PAGE_SIZE;
	int n_pages = min(vma_pages, info->n_tiler_pages);
	int i, ret = 0;
	pgprot_t vm_page_prot;

	/* Use writecombined mappings unless on OMAP5.  If OMAP5, use
	shared device due to h/w issue. */
	if (cpu_is_omap54xx())
		vm_page_prot = __pgprot_modify(vma->vm_page_prot, L_PTE_MT_MASK,
						L_PTE_MT_DEV_SHARED);
	else
		vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (TILER_PIXEL_FMT_PAGE == info->fmt) {
		/* Since 1D buffer is linear, map whole buffer in one shot */
		ret = remap_pfn_range(vma, addr,
				 __phys_to_pfn(info->tiler_addrs[0]),
				(vma->vm_end - vma->vm_start),
				(buffer->cached ?
				(vma->vm_page_prot)
				: vm_page_prot));
	} else {
		for (i = vma->vm_pgoff; i < n_pages; i++, addr += PAGE_SIZE) {
			ret = remap_pfn_range(vma, addr,
				 __phys_to_pfn(info->tiler_addrs[i]),
				PAGE_SIZE,
				vm_page_prot);
			if (ret)
				return ret;
		}
	}
	return ret;
}

static void per_cpu_cache_flush_arm(void *arg)
{
	   flush_cache_all();
}

static int omap_tiler_cache_operation(struct ion_buffer *buffer, size_t len,
			unsigned long vaddr, enum cache_operation cacheop)
{
	struct omap_tiler_info *info;
	int n_pages;

	if (!buffer) {
		pr_err("%s(): buffer is NULL\n", __func__);
		return -EINVAL;
	}
	if (!buffer->cached) {
		pr_err("%s(): buffer not mapped as cacheable\n", __func__);
		return -EINVAL;
	}

	info = buffer->priv_virt;
	if (!info) {
		pr_err("%s(): tiler info of buffer is NULL\n", __func__);
		return -EINVAL;
	}

	n_pages = info->n_tiler_pages;
	if (len > (n_pages * PAGE_SIZE)) {
		pr_err("%s(): size to flush is greater than allocated size\n",
				__func__);
		return -EINVAL;
	}

	if (TILER_PIXEL_FMT_PAGE != info->fmt) {
		pr_err("%s(): only TILER 1D buffers can be cached\n", __func__);
		return -EINVAL;
	}

	if (len > FULL_CACHE_FLUSH_THRESHOLD) {
		on_each_cpu(per_cpu_cache_flush_arm, NULL, 1);
		outer_flush_all();
		return 0;
	}

	__cpuc_coherent_user_range((vaddr) & PAGE_MASK, PAGE_ALIGN(vaddr+len));

	if (cacheop == CACHE_FLUSH)
		outer_flush_range(info->tiler_addrs[0],
				info->tiler_addrs[0] + len);
	else
		outer_inv_range(info->tiler_addrs[0],
				info->tiler_addrs[0] + len);
	return 0;
}

static int omap_tiler_heap_flush_user(struct ion_buffer *buffer, size_t len,
			unsigned long vaddr)
{
	return omap_tiler_cache_operation(buffer, len, vaddr, CACHE_FLUSH);
}

static int omap_tiler_heap_inval_user(struct ion_buffer *buffer, size_t len,
			unsigned long vaddr)
{
	return omap_tiler_cache_operation(buffer, len, vaddr, CACHE_INVALIDATE);
}

static struct ion_heap_ops omap_tiler_ops = {
	.allocate = omap_tiler_heap_allocate,
	.free = omap_tiler_heap_free,
	.phys = omap_tiler_phys,
	.map_user = omap_tiler_heap_map_user,
	.flush_user = omap_tiler_heap_flush_user,
	.inval_user = omap_tiler_heap_inval_user,
};

struct ion_heap *omap_tiler_heap_create(struct ion_platform_heap *data)
{
	struct ion_heap *heap;

	heap = ion_carveout_heap_create(data);
	if (!heap)
		return ERR_PTR(-ENOMEM);
	heap->ops = &omap_tiler_ops;
	heap->type = OMAP_ION_HEAP_TYPE_TILER;
	heap->name = data->name;
	heap->id = data->id;
	return heap;
}

void omap_tiler_heap_destroy(struct ion_heap *heap)
{
	kfree(heap);
}
