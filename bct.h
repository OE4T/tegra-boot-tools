#ifndef bct_h__
#define bct_h__
/* Copyright (c) 2020 Matthew Madison */

int bct_update_valid_t18x(void *cur_bct, void *cand_bct, unsigned int *block_size, unsigned int *page_size);
int bct_update_valid_t19x(void *cur_bct, void *cand_bct, unsigned int *block_size, unsigned int *page_size);

#endif /* bct_h__ */
