/*
 * smd.c
 *
 * Functions for working with the Tegra boot slot metadata.
 *
 * Copyright (c) 2020, Matthew Madison
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <zlib.h>
#include "smd.h"

/*
 * Structures used in the SMD storage
 */
struct slot_info_s {
	uint8_t priority;
	char suffix[2];
	uint8_t retry_count;
	uint8_t successful;
} __attribute__((packed));

#define SMD_VER_MIN 1U
#define SMD_VER_MAX 3U

struct smd_s {
	char magic[4];
	uint8_t version;
	uint8_t flags;
	uint16_t maxslots;
	struct slot_info_s slot_info[2];
	uint32_t crc32;
} __attribute__((packed));

static const char smd_magic[4] = {'\0', 'N', 'B', 'C'};

struct smd_context_s {
	struct smd_s smd_ods;
	char suffixes[2][3];
	bool needs_update;
};

/*
 * smd_init
 *
 * Initialize an SMD context and load the SMD(s)
 * from the boot device, which must have an open
 * gpt context.
 *
 * Returns: smd_context_t pointer or NULL on error
 */
smd_context_t *
smd_init (gpt_context_t *boot_gpt, int bootfd)
{
	smd_context_t *ctx;
	gpt_entry_t *part;
	ssize_t n, total, remain;
	int i;

	ctx = calloc(1, sizeof(smd_context_t));
	if (ctx == NULL)
		return NULL;
	/*
	 * Check the primary and backup for a valid copy of the metadata
	 */
	for (i = 0; i < 2; i++) {
		part = gpt_find_by_name(boot_gpt, (i == 0 ? "SMD" : "SMD_b"));
		if (part == NULL)
			continue;
		if (lseek(bootfd, part->first_lba * 512, SEEK_SET) == (off_t) -1)
			continue;
		for (remain = sizeof(ctx->smd_ods), total = 0; remain > 0; total += n, remain -= n) {
			n = read(bootfd, (uint8_t *) &ctx->smd_ods + total, remain);
			if (n <= 0)
				continue;
		}

		if (memcmp(ctx->smd_ods.magic, smd_magic, sizeof(smd_magic)) != 0)
			continue;
		/*
		 * There were versions 1 and 2 of the metadata, but we only
		 * support version 3 (and later, should there be more) here.
		 */
		if (ctx->smd_ods.version >= 3) {
			uint32_t crc = crc32(0, (void *) &ctx->smd_ods, sizeof(ctx->smd_ods)-sizeof(uint32_t));
			if (crc == ctx->smd_ods.crc32)
				break;
		}
	}
	/*
	 * If no valid SMD after looking at both partitions, punt.
	 */
	if (i > 2) {
		free(ctx);
		errno = ENODEV;
		ctx = NULL;
	}

	for (i = 0; i < 2; i++) {
		struct slot_info_s *s = &ctx->smd_ods.slot_info[i];
		ctx->suffixes[i][0] = s->suffix[0];
		ctx->suffixes[i][1] = s->suffix[1];
		ctx->suffixes[i][2] = '\0';
	}

	return ctx;

} /* smd_init */

/*
 * smd_init
 *
 * Initialize a new, clear SMD context
 * for redundancy at the specified level.
 *
 * Returns: smd_context_t pointer or NULL on error
 */
smd_context_t *
smd_new (smd_redundancy_level_t level)
{
	smd_context_t *ctx;

	if (level < REDUNDANCY_OFF || level > REDUNDANCY_FULL) {
		errno = EINVAL;
		return NULL;
	}

	ctx = calloc(1, sizeof(smd_context_t));
	if (ctx == NULL)
		return NULL;
	strcpy((char *) ctx->suffixes[0], "_a");
	strcpy((char *) ctx->suffixes[1], "_b");
	ctx->needs_update = true;
	memcpy(ctx->smd_ods.magic, smd_magic, sizeof(ctx->smd_ods.magic));
	ctx->smd_ods.version = 3;
	ctx->smd_ods.maxslots = 1;
	ctx->smd_ods.slot_info[0].priority = 15;
	ctx->smd_ods.slot_info[0].suffix[0] = ctx->suffixes[0][0];
	ctx->smd_ods.slot_info[0].suffix[1] = ctx->suffixes[0][1];
	ctx->smd_ods.slot_info[0].retry_count = 7;
	ctx->smd_ods.slot_info[0].successful = 1;
	if (smd_set_redundancy_level(ctx, level) != 0) {
		free(ctx);
		return NULL;
	}
	return ctx;

} /* smd_new */

/*
 * smd_finish
 *
 * Frees the allocated context.
 * Caller *must* call smd_save() before smd_finish()
 * if there are any updates to be made; otherwise,
 * they will be lost.
 */
void
smd_finish (smd_context_t *ctx)
{
	if (ctx != NULL)
		free(ctx);

} /* smd_finish */

/*
 * smd_redundancy_level
 *
 * Reports the current redundancy level in the metadata.
 *
 * Returns: smd_redundancy_level_t
 */
smd_redundancy_level_t
smd_redundancy_level (smd_context_t *ctx)
{
	unsigned int flags;

	if (ctx == NULL) {
		errno = EINVAL;
		return -1;
	}

	flags = ctx->smd_ods.flags & 3;
	/*
	 * The lowest bit of the flags is enable/disable
	 * The next bit is "user" redundancy (which we call "full")
	 *
	 * Extra check on the number slots here; should always be 2,
	 * but if it's 1 (or 0?), then there's really no redundancy.
	 */
	if (flags == 0 || flags == 2 || ctx->smd_ods.maxslots < 2)
		return REDUNDANCY_OFF;
	if (flags == 1)
		return REDUNDANCY_BOOTLOADER_ONLY;
	return REDUNDANCY_FULL;

} /* smd_redundancy_level */

/*
 * smd_set_redundancy_level
 *
 * Sets the redunancy level - off, bl-only, or full
 *
 * Returns: zero on success, -1 on error
 *
 */
int
smd_set_redundancy_level (smd_context_t *ctx, smd_redundancy_level_t level)
{
	uint8_t old_flags;

	if (ctx == NULL) {
		errno = EINVAL;
		return -1;
	}

	old_flags = ctx->smd_ods.flags;

	switch (level) {
	case REDUNDANCY_OFF:
		ctx->smd_ods.flags &= ~3U;
		ctx->smd_ods.maxslots = 1;
		ctx->smd_ods.slot_info[0].priority = 15;
		ctx->smd_ods.slot_info[0].retry_count = 7;
		ctx->smd_ods.slot_info[0].successful = 1;
		memset(&ctx->smd_ods.slot_info[1], 0, sizeof(ctx->smd_ods.slot_info[1]));
		break;
	case REDUNDANCY_BOOTLOADER_ONLY:
		ctx->smd_ods.flags = (ctx->smd_ods.flags & ~3U) | 1U;
		/* Slot initialization happens below */
		break;
	case REDUNDANCY_FULL:
		ctx->smd_ods.flags |= 3;
		/* Slot initialization happens below */
		break;
	default:
		errno = EINVAL;
		return -1;
	}
	if (old_flags != ctx->smd_ods.flags) {
		ctx->needs_update = 1;
		if (old_flags == 0) {
			int i;
			ctx->smd_ods.maxslots = 2;
			strcpy((char *) ctx->suffixes[0], "_a");
			strcpy((char *) ctx->suffixes[1], "_b");
			for (i = 0; i < 2; i++) {
				struct slot_info_s *s = &ctx->smd_ods.slot_info[i];
				s->priority = 15 - i;
				s->suffix[0] = ctx->suffixes[i][0];
				s->suffix[1] = ctx->suffixes[i][1];
				s->retry_count = 7;
				s->successful = 1;
			}
		}
	}
	return 0;

} /* smd_set_redundancy_level */

/*
 * smd_slot_get
 *
 * Retrieve information for a boot slot.
 *
 * Returns: 0 on success, -1 on error
 */
int
smd_slot_get (smd_context_t *ctx, unsigned int which, smd_slot_t *slot)
{
	struct slot_info_s *s;

	if (ctx == NULL || slot == NULL || which >= ctx->smd_ods.maxslots) {
		errno = EINVAL;
		return -1;
	}
	s = &ctx->smd_ods.slot_info[which];
	slot->slot_prio = s->priority;
	slot->slot_suffix = (const char *) &ctx->suffixes[which];
	slot->slot_retry_count = s->retry_count;
	slot->slot_successful = !!s->successful;
	return 0;

} /* smd_slot_get */

/*
 * smd_get_current_slot
 *
 * Extracts the current boot slot from the kernel command line
 * and converts it into an index.
 *
 * Returns 0 or 1, or negative number on error.
 */
int
smd_get_current_slot (void)
{
	char cmdline[4096], *cp;
	static const char *bssarg = "boot.slot_suffix=";
	ssize_t n;
	int fd;

	fd = open("/proc/cmdline", O_RDONLY);
	if (fd < 0)
		return fd;
	n = read(fd, cmdline, sizeof(cmdline)-1);
	if (n < 0) {
		close(fd);
		return n;
	}
	close(fd);
	cmdline[n] = '\0';
	cp = strstr(cmdline, bssarg);
	/*
	 * If boot.slot_suffix arg is not present, redundancy
	 * is disabled, so current slot is zero.
	 */
	if (cp == NULL)
		return 0;
	/*
	 * Validate that the arg is followed by a valid
	 * slot suffix or the null string.
	 */
	cp += strlen(bssarg);
	if (*cp == '_') {
		if (*(cp+1) == 'b')
			return 1;
		if (*(cp+1) == 'a')
			return 0;
		errno = ESRCH;
		return -1;
	}
	if (*cp == '\0' || *cp == ' ')
		return 0;
	errno = ESRCH;
	return -1;

} /* smd_get_current_slot */

/*
 * smd_slot_mark_successful
 *
 * Marks a boot slot as successful.  If the slot is
 * also the current slot, it will be marked as active
 * as well (i.e., priority set to 15).
 *
 * Returns: 0 on success, -1 on failure
 */
int
smd_slot_mark_successful (smd_context_t *ctx, unsigned int which)
{
	int curslot;
	struct slot_info_s *s;

	if (which >= ctx->smd_ods.maxslots) {
		errno = EINVAL;
		return -1;
	}

	curslot = smd_get_current_slot();
	if (curslot < 0)
		return -1;

	s = &ctx->smd_ods.slot_info[which];
	ctx->needs_update = (s->successful != 1 || s->retry_count != 7 ||
			     ((unsigned int) curslot == which && s->priority != 15));
	s->successful = 1;
	s->retry_count = 7;
	if ((unsigned int) curslot == which)
		s->priority = 15;

	return 0;

} /* smd_slot_mark_successful */

/*
 * smd_slot_mark_active
 *
 * Marks a boot slot as active by setting its priority
 * to 15 and setting the other slot's priority to 14,
 * resetting its retry count to 7, and marking
 * it as unsuccessful (so success can be tested after
 * reboot).
 *
 * Returns: 0 on success, -1 on failure
 */
int
smd_slot_mark_active (smd_context_t *ctx, unsigned int which)
{
	struct slot_info_s *s;

	if (which >= ctx->smd_ods.maxslots) {
		errno = EINVAL;
		return -1;
	}

	s = &ctx->smd_ods.slot_info[which];
	ctx->needs_update = (s->priority != 15 ||
			     ctx->smd_ods.slot_info[1-which].priority != 14 ||
			     s->retry_count != 7 || s->successful != 0);
	s->priority = 15;
	ctx->smd_ods.slot_info[1-which].priority = 14;
	s->retry_count = 7;
	s->successful = 0;

	return 0;

} /* smd_slot_mark_active */

/*
 * smd_update
 *
 * Writes the slot metadata to the boot device if
 * it has changed or if the `force` argument is `true`.
 *
 * Returns: 0 on success, negative integer on failure.
 */
int
smd_update (smd_context_t *ctx, gpt_context_t *boot_gpt, int bootfd, bool force)
{
	gpt_entry_t *part;
	ssize_t n, total, remain;
	int i;

	if (!(force || ctx->needs_update))
		return 0;
	if (ctx->smd_ods.version != 3 || memcmp(ctx->smd_ods.magic, smd_magic, sizeof(ctx->smd_ods.magic) != 0)) {
		errno = EINVAL;
		return -1;
	}
	ctx->smd_ods.crc32 = crc32(0, (void *) &ctx->smd_ods, sizeof(ctx->smd_ods)-sizeof(uint32_t));
	/*
	 * Write both the primary and backup copies.
	 */
	for (i = 0; i < 2; i++) {
		part = gpt_find_by_name(boot_gpt, (i == 0 ? "SMD" : "SMD_b"));
		if (part == NULL)
			continue;
		if (lseek(bootfd, part->first_lba * 512, SEEK_SET) == (off_t) -1)
			continue;
		for (remain = sizeof(ctx->smd_ods), total = 0; remain > 0; total += n, remain -= n) {
			n = write(bootfd, (uint8_t *) &ctx->smd_ods + total, remain);
			if (n <= 0)
				continue;
		}
	}
	ctx->needs_update = false;
	return 0;

} /* smd_update */
