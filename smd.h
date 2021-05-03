#ifndef smd_h_included
#define smd_h_included
/* Copyright (c) 2020, Matthew Madison */

#include <stdbool.h>
#include "gpt.h"

typedef enum {
	REDUNDANCY_OFF = 0,
	REDUNDANCY_BOOTLOADER_ONLY = 1,
	REDUNDANCY_FULL = 2,
} smd_redundancy_level_t;

struct smd_slot_s {
	unsigned int slot_prio;
	const char  *slot_suffix;
	unsigned int slot_retry_count;
	bool slot_successful;
};
typedef struct smd_slot_s smd_slot_t;

struct smd_context_s;
typedef struct smd_context_s smd_context_t;

smd_context_t *smd_init(gpt_context_t *boot_gpt, int bootfd);
smd_context_t *smd_new(smd_redundancy_level_t level);
void smd_finish(smd_context_t *ctx);
smd_redundancy_level_t smd_redundancy_level(smd_context_t *ctx);
int smd_set_redundancy_level(smd_context_t *ctx, smd_redundancy_level_t level);
int smd_slot_get(smd_context_t *ctx, unsigned int which, smd_slot_t *slot);
int smd_slot_mark_successful(smd_context_t *ctx, unsigned int which);
int smd_slot_mark_active(smd_context_t *ctx, unsigned int which);
int smd_get_current_slot(void);
int smd_update(smd_context_t *ctx, gpt_context_t *boot_gpt, int bootfd, bool force);

#endif /* smd_h_included */
