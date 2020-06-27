/*
 * bct_t18x.c
 *
 * BCT-related functions for t18x SoCs
 *
 * Copyright (c) 2020 Matthew Madison
 */

#include "t18x/nvboot_bct.h"

/*
 * bct_update_valid_t18x
 */
int
bct_update_valid_t18x (void *cur_bct, void *cand_bct,
		       unsigned int *block_size,
		       unsigned int *page_size)
{
	NvBootConfigTable *cur = cur_bct;
	NvBootConfigTable *cand = cand_bct;

	if (cur->BctSize != cand->BctSize)
		return 0;
	if ((cur->BootDataVersion >> 24) != (cand->BootDataVersion >> 24))
		return 0;
	if (cur->BlockSizeLog2 != cand->BlockSizeLog2)
		return 0;
	*block_size = (1U << cur->BlockSizeLog2);
	if (cur->PageSizeLog2 != cand->PageSizeLog2)
		return 0;
	*page_size = (1U << cur->PageSizeLog2);

	return 1;

} /* bct_update_valid_t18x */
