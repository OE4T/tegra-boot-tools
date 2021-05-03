#ifndef bup_h_included
#define bup_h_included
/* Copyright (c) 2019-2020, Matthew Madison */

#include <sys/types.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

struct bup_context_s;
typedef struct bup_context_s bup_context_t;

bup_context_t *bup_init(const char *pathname);
void bup_finish(bup_context_t *ctx);
const char *bup_gpt_device(bup_context_t *ctx);
const char *bup_boot_device(bup_context_t *ctx);
const char *bup_tnspec(bup_context_t *ctx);
bool bup_enumerate_entries(bup_context_t *ctx, void **iterctx,
			   const char **partname, off_t *offset,
			   size_t *length, unsigned int *version);
int bup_find_missing_entries(bup_context_t *ctx, const char **missing_parts,
			     size_t max_missing);
off_t bup_setpos(bup_context_t *ctx, off_t offset);
ssize_t bup_read (bup_context_t *ctx, void *buf, size_t bufsize);

#endif /* bup_h_included */
