/*
 * bct_t21x.c
 *
 * BCT-related functions for t21x SoCs
 *
 * Copyright (c) 2020 Matthew Madison
 */
#include <stdint.h>

#define BOOT_BLOCK_SIZE_LOG_2 333
#define BOOT_PAGE_SIZE_LOG_2  334

/*
 * bct_update_valid_t21x
 */
int
bct_update_valid_t21x (void *cur_bct, void *cand_bct,
		       unsigned int *block_size,
		       unsigned int *page_size)
{
	uint32_t *cur = cur_bct;
	uint32_t *cand = cand_bct;

	if (cur[BOOT_BLOCK_SIZE_LOG_2] != cand[BOOT_BLOCK_SIZE_LOG_2] ||
	    cur[BOOT_PAGE_SIZE_LOG_2] != cand[BOOT_PAGE_SIZE_LOG_2])
		return 0;
	*block_size = (1U << cur[BOOT_BLOCK_SIZE_LOG_2]);
	*page_size = (1U << cur[BOOT_PAGE_SIZE_LOG_2]);

	return 1;

} /* bct_update_valid_t21x */
