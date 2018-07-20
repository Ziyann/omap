/*
 * drivers/gpu/ion/ion_mem_pool.c
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

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/shrinker.h>
#include "ion_priv.h"

struct ion_page_pool_item {
	struct page *page;
	struct list_head list;
};

static void *ion_page_pool_alloc_pages(struct ion_page_pool *pool)
{
	struct page *page = alloc_pages(pool->gfp_mask, pool->order);

	if (!page)
		return NULL;
	/* this is only being used to flush the page for dma,
	   this api is not really suitable for calling from a driver
	   but no better way to flush a page for dma exist at this time */
	__dma_page_cpu_to_dev(page, 0, PAGE_SIZE << pool->order,
			      DMA_BIDIRECTIONAL);
	return page;
}

static void ion_page_pool_free_pages(struct ion_page_pool *pool,
				     struct page *page)
{
	__free_pages(page, pool->order);
}

static int ion_page_pool_add(struct ion_page_pool *pool, struct page *page)
{
	struct ion_page_pool_item *item;

	item = kmalloc(sizeof(struct ion_page_pool_item), GFP_KERNEL);
	if (!item)
		return -ENOMEM;
	item->page = page;
	if (PageHighMem(page)) {
		list_add_tail(&item->list, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&item->list, &pool->low_items);
		pool->low_count++;
	}
	return 0;
}

static struct page *ion_page_pool_remove(struct ion_page_pool *pool, bool high)
{
	struct ion_page_pool_item *item;
	struct page *page;

	if (high) {
		BUG_ON(!pool->high_count);
		item = list_first_entry(&pool->high_items,
					struct ion_page_pool_item, list);
		pool->high_count--;
	} else {
		BUG_ON(!pool->low_count);
		item = list_first_entry(&pool->low_items,
					struct ion_page_pool_item, list);
		pool->low_count--;
	}

	list_del(&item->list);
	page = item->page;
	kfree(item);
	return page;
}

void *ion_page_pool_alloc(struct ion_page_pool *pool)
{
	struct page *page = NULL;

	BUG_ON(!pool);

	mutex_lock(&pool->mutex);
	if (pool->high_count)
		page = ion_page_pool_remove(pool, true);
	else if (pool->low_count)
		page = ion_page_pool_remove(pool, false);
	mutex_unlock(&pool->mutex);

	if (!page)
		page = ion_page_pool_alloc_pages(pool);

	return page;
}

void ion_page_pool_free(struct ion_page_pool *pool, struct page* page)
{
	int ret;

	mutex_lock(&pool->mutex);
	ret = ion_page_pool_add(pool, page);
	mutex_unlock(&pool->mutex);
	if (ret)
		ion_page_pool_free_pages(pool, page);
}

static int ion_page_pool_shrink(struct shrinker *shrinker,
				 struct shrink_control *sc)
{
	struct ion_page_pool *pool = container_of(shrinker,
						 struct ion_page_pool,
						 shrinker);
	int nr_freed = 0;
	int i;
	bool high;

	if (sc->gfp_mask & __GFP_HIGHMEM)
		high = true;

	if (sc->nr_to_scan == 0)
		return high ? (pool->high_count + pool->low_count) *
			(1 << pool->order) :
			pool->low_count * (1 << pool->order);

	for (i = 0; i < sc->nr_to_scan; i++) {
		struct page *page;

		mutex_lock(&pool->mutex);
		if (high && pool->high_count) {
			page = ion_page_pool_remove(pool, true);
		} else if (pool->low_count) {
			page = ion_page_pool_remove(pool, false);
		} else {
			mutex_unlock(&pool->mutex);
			break;
		}
		mutex_unlock(&pool->mutex);
		ion_page_pool_free_pages(pool, page);
		nr_freed += (1 << pool->order);
	}
	pr_info("%s: shrunk page_pool of order %d by %d pages\n", __func__,
		pool->order, nr_freed);

	return high ? (pool->high_count + pool->low_count) *
		(1 << pool->order) :
		pool->low_count * (1 << pool->order);
}

struct ion_page_pool *ion_page_pool_create(gfp_t gfp_mask, unsigned int order)
{
	struct ion_page_pool *pool = kmalloc(sizeof(struct ion_page_pool),
					     GFP_KERNEL);
	if (!pool)
		return NULL;
	pool->high_count = 0;
	pool->low_count = 0;
	INIT_LIST_HEAD(&pool->low_items);
	INIT_LIST_HEAD(&pool->high_items);
	pool->shrinker.shrink = ion_page_pool_shrink;
	pool->shrinker.seeks = DEFAULT_SEEKS * 16;
	pool->shrinker.batch = 0;
	register_shrinker(&pool->shrinker);
	pool->gfp_mask = gfp_mask;
	pool->order = order;
	mutex_init(&pool->mutex);

	return pool;
}

void ion_page_pool_destroy(struct ion_page_pool *pool)
{
	unregister_shrinker(&pool->shrinker);
	kfree(pool);
}

