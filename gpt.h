#ifndef gpt_h__
#define gpt_h__
/* Copyright (c) 2019, Matthew Madison */

#include <stdint.h>

#define GPT_SIZE_IN_BLOCKS 32

struct gpt_context_s;
typedef struct gpt_context_s gpt_context_t;

struct gpt_entry_s {
	uint8_t type_guid[16];
	uint8_t part_guid[16];
	uint64_t first_lba;
	uint64_t last_lba;
	uint64_t flags;
	char part_name[36];
};
typedef struct gpt_entry_s gpt_entry_t;

#define GPT_INIT_FOR_WRITING	(1<<2)
gpt_context_t *gpt_init(const char *devname, unsigned int blocksize, unsigned int flags);
void gpt_finish(gpt_context_t *ctx);
int gpt_fd(gpt_context_t *ctx);

#define GPT_BACKUP_ONLY		(1<<0)
#define GPT_NVIDIA_SPECIAL	(1<<1)

int gpt_load(gpt_context_t *ctx, unsigned int flags);
int gpt_load_from_config(gpt_context_t *ctx);
int gpt_save(gpt_context_t *ctx, unsigned int flags);

gpt_entry_t *gpt_find_by_name(gpt_context_t *ctx, const char *name);
gpt_entry_t *gpt_enumerate_partitions(gpt_context_t *ctx, void **iterctx);

#endif /* gpt_h__ */
