#ifndef bootinfo_h_included
#define bootinfo_h_included
/* Copyright (c) 2022, Matthew Madison */

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>

struct bootinfo_context_s;
typedef struct bootinfo_context_s bootinfo_context_t;
struct bootinfo_var_iter_context_s;
typedef struct bootinfo_var_iter_context_s bootinfo_var_iter_context_t;

#define BOOTINFO_O_RDONLY (0<<0)
#define BOOTINFO_O_RDWR	  (3<<0)
#define BOOTINFO_O_CREAT  (1<<2)

int bootinfo_open(unsigned int flags, bootinfo_context_t **ctxp);
int bootinfo_close(bootinfo_context_t *ctx);
int bootinfo_mark_boot_success(bootinfo_context_t *ctx, unsigned int *failcount);
int bootinfo_check_boot_status(bootinfo_context_t *ctx, unsigned int *failcount);
int bootinfo_get_info(bootinfo_context_t *ctx, unsigned int *version, bool *boot_in_progress,
		      unsigned int *failcount, unsigned int *ext_sector_count);
int bootinfo_var_get(bootinfo_context_t *ctx, const char *name, char *valuebuf, size_t valuebuf_size);
int bootinfo_var_set(bootinfo_context_t *ctx, const char *name, const char *value);
int bootinfo_var_iter_begin(bootinfo_context_t *ctx, bootinfo_var_iter_context_t **iterctx);
int bootinfo_var_iter_next(bootinfo_var_iter_context_t *iterctx,
			   char *namebuf, size_t namebuf_size,
			   char *valuebuf, size_t valuebuf_size);
void bootinfo_var_iter_end(bootinfo_var_iter_context_t *iterctx);

#endif /* bootinfo_h_included */
