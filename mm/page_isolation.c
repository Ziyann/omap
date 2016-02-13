/*
 * linux/mm/page_isolation.c
 */

#include <linux/mm.h>
#include <linux/page-isolation.h>
#include <linux/pageblock-flags.h>
#include <linux/ksm.h>
#include "internal.h"

#ifdef CONFIG_CMA_DEBUG_VERBOSE
void print_cma_page_stats(struct page *page)
{
	pr_err("page: phys = 0x%x, count = %d, order = %lu, private = %ld\n", page_to_phys(page), page_count(page), page_order(page), page_private(page));
	pr_err("page: buddy = %d, LRU = %d, KSM = %d, mapping = 0x%p, map count = %d\n", PageBuddy(page), PageLRU(page), PageKsm(page), page->mapping, atomic_read(&page->_mapcount));
	pr_err("page: index = 0x%lx, anon = %d, writeback = %d, dirty = %d, swap cache = %d\n", page->index, PageAnon(page), PageWriteback(page), PageDirty(page), PageSwapCache(page));
	pr_err("pageblock: migratetype = 0x%x, flags = 0x%lx\n", get_pageblock_migratetype(page), get_pageblock_flags(page));
}
#endif

static inline struct page *
__first_valid_page(unsigned long pfn, unsigned long nr_pages)
{
	int i;
	for (i = 0; i < nr_pages; i++)
		if (pfn_valid_within(pfn + i))
			break;
	if (unlikely(i == nr_pages))
		return NULL;
	return pfn_to_page(pfn + i);
}

/*
 * start_isolate_page_range() -- make page-allocation-type of range of pages
 * to be MIGRATE_ISOLATE.
 * @start_pfn: The lower PFN of the range to be isolated.
 * @end_pfn: The upper PFN of the range to be isolated.
 * @migratetype: migrate type to set in error recovery.
 *
 * Making page-allocation-type to be MIGRATE_ISOLATE means free pages in
 * the range will never be allocated. Any free pages and pages freed in the
 * future will not be allocated again.
 *
 * start_pfn/end_pfn must be aligned to pageblock_order.
 * Returns 0 on success and -EBUSY if any part of range cannot be isolated.
 */
int start_isolate_page_range(unsigned long start_pfn, unsigned long end_pfn,
			     unsigned migratetype)
{
	unsigned long pfn;
	unsigned long undo_pfn;
	struct page *page;

	BUG_ON((start_pfn) & (pageblock_nr_pages - 1));
	BUG_ON((end_pfn) & (pageblock_nr_pages - 1));

	for (pfn = start_pfn;
	     pfn < end_pfn;
	     pfn += pageblock_nr_pages) {
		page = __first_valid_page(pfn, pageblock_nr_pages);
		if (page && set_migratetype_isolate(page)) {
			undo_pfn = pfn;
			goto undo;
		}
	}
	return 0;
undo:
	for (pfn = start_pfn;
	     pfn < undo_pfn;
	     pfn += pageblock_nr_pages)
		unset_migratetype_isolate(pfn_to_page(pfn), migratetype);

	return -EBUSY;
}

/*
 * Make isolated pages available again.
 */
int undo_isolate_page_range(unsigned long start_pfn, unsigned long end_pfn,
			    unsigned migratetype)
{
	unsigned long pfn;
	struct page *page;
	BUG_ON((start_pfn) & (pageblock_nr_pages - 1));
	BUG_ON((end_pfn) & (pageblock_nr_pages - 1));
	for (pfn = start_pfn;
	     pfn < end_pfn;
	     pfn += pageblock_nr_pages) {
		page = __first_valid_page(pfn, pageblock_nr_pages);
		if (!page || get_pageblock_migratetype(page) != MIGRATE_ISOLATE)
			continue;
		unset_migratetype_isolate(page, migratetype);
	}
	return 0;
}
/*
 * Test all pages in the range is free(means isolated) or not.
 * all pages in [start_pfn...end_pfn) must be in the same zone.
 * zone->lock must be held before call this.
 *
 * Returns 1 if all pages in the range are isolated.
 */
static int
__test_page_isolated_in_pageblock(unsigned long pfn, unsigned long end_pfn)
{
	struct page *page;

	while (pfn < end_pfn) {
		if (!pfn_valid_within(pfn)) {
			pfn++;
			continue;
		}
		page = pfn_to_page(pfn);
		if (PageBuddy(page))
			pfn += 1 << page_order(page);
		else if (page_count(page) == 0 &&
				page_private(page) == MIGRATE_ISOLATE)
			pfn += 1;
		else
			break;
	}
	if (pfn < end_pfn) {
#ifdef CONFIG_CMA_DEBUG_VERBOSE
		print_cma_page_stats(pfn_to_page(pfn));
#endif
		return 0;
	}
	return 1;
}

int test_pages_isolated(unsigned long start_pfn, unsigned long end_pfn)
{
	unsigned long pfn, flags;
	struct page *page;
	struct zone *zone;
	int ret;

	/*
	 * Note: pageblock_nr_page != MAX_ORDER. Then, chunks of free page
	 * is not aligned to pageblock_nr_pages.
	 * Then we just check pagetype fist.
	 */
	for (pfn = start_pfn; pfn < end_pfn; pfn += pageblock_nr_pages) {
		page = __first_valid_page(pfn, pageblock_nr_pages);
		if (page && get_pageblock_migratetype(page) != MIGRATE_ISOLATE)
			break;
	}
	page = __first_valid_page(start_pfn, end_pfn - start_pfn);
	if ((pfn < end_pfn) || !page)
		return -EBUSY;
	/* Check all pages are free or Marked as ISOLATED */
	zone = page_zone(page);
	spin_lock_irqsave(&zone->lock, flags);
	ret = __test_page_isolated_in_pageblock(start_pfn, end_pfn);
	spin_unlock_irqrestore(&zone->lock, flags);
	return ret ? 0 : -EBUSY;
}
