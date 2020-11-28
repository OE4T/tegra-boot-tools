/*
 * gpt.c
 *
 * Functions for parsing a GUID partition table.
 *
 * Copyright (c) 2019, Matthew Madison
 *
 */
#define _DEFAULT_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <zlib.h>
#include <fcntl.h>
#include "gpt.h"
#include "config.h"
#define QUOTE(m_) #m_
#define XQUOTE(m_) QUOTE(m_)
static const char bootpartconf[] = XQUOTE(CONFIGPATH) "/boot-partitions.conf";
#define MAX_CONFIG_ENTRIES 64U

struct gpt_header_s {
	unsigned char signature[8];
	uint32_t revision;
	uint32_t header_size;
	uint32_t header_crc32;
	uint32_t reserved1;
	uint64_t current_lba;
	uint64_t backup_lba;
	uint64_t first_usable_lba;
	uint64_t last_usable_lba;
	uint8_t disk_guid[16];
	uint64_t entries_start_lba;
	uint32_t entry_count;
	uint32_t entry_size;
	uint32_t entries_crc32;
} __attribute__((packed));

struct gpt_entry_ondisk_s {
	uint8_t type_guid[16];
	uint8_t part_guid[16];
	uint64_t first_lba;
	uint64_t last_lba;
	uint64_t flags;
	uint16_t part_name_utf16[36];
} __attribute__((packed));

struct gpt_context_s {
	int fd;
	unsigned int blocksize;
	off_t devsize;
	void *buffer;
	int is_mmcboot1;
	int primary_valid, backup_valid;
	struct gpt_header_s primary_header;
	struct gpt_header_s backup_header;
	unsigned int entry_count;
	struct gpt_entry_s *entries;
};

gpt_context_t *
gpt_init (const char *devname, unsigned int blocksize)
{
	int fd;
	int err;
	gpt_context_t *ctx;

	if (blocksize < 512) {
		errno = EINVAL;
		return NULL;
	}

	ctx = malloc(sizeof(*ctx));
	if (ctx == NULL)
		return NULL;
	memset(ctx, 0, sizeof(*ctx));
	err = posix_memalign(&ctx->buffer, sizeof(uint64_t), blocksize * 32);
	if (err) {
		errno = err;
		free(ctx);
		return NULL;
	}
	fd = open(devname, O_RDONLY);
	if (fd < 0) {
		free(ctx->buffer);
		free(ctx);
		return NULL;
	}
	ctx->is_mmcboot1 = strcmp(devname, "/dev/mmcblk0boot1") == 0;
	ctx->fd = fd;
	ctx->blocksize = blocksize;
	ctx->devsize = lseek(fd, 0, SEEK_END);
	if (ctx->devsize == (off_t) -1) {
		gpt_finish(ctx);
		return NULL;
	}
	return ctx;

} /* gpt_init */

void
gpt_finish (gpt_context_t *ctx)
{
	close(ctx->fd);
	if (ctx->entries)
		free(ctx->entries);
	free(ctx->buffer);
	free(ctx);

} /* gpt_finish */

static int
parse_header (gpt_context_t *ctx, size_t len, struct gpt_header_s *dest) {
	struct gpt_header_s *hdr = ctx->buffer;
	uint32_t hlen, crc;
	if (memcmp(hdr->signature, "EFI PART", 8) != 0)
		return -1;
	hlen = le32toh(hdr->header_size);
	if (hlen < sizeof(struct gpt_header_s) || hlen > len)
		return -1;
        crc = le32toh(hdr->header_crc32);
	hdr->header_crc32 = 0;
	if (crc32(0, ctx->buffer, hlen) != crc)
		return -1;
	if (dest != NULL) {
		memcpy(dest->signature, hdr->signature, sizeof(dest->signature));
		dest->revision = le32toh(hdr->revision);
		dest->header_size = hlen;
		dest->header_crc32 = crc;
		dest->current_lba = le64toh(hdr->current_lba);
		dest->backup_lba = le64toh(hdr->backup_lba);
		dest->first_usable_lba = le64toh(hdr->first_usable_lba);
		dest->last_usable_lba = le64toh(hdr->last_usable_lba);
		memcpy(dest->disk_guid, hdr->disk_guid, sizeof(dest->disk_guid));
		dest->entries_start_lba = le64toh(hdr->entries_start_lba);
		dest->entry_count = le32toh(hdr->entry_count);
		dest->entry_size = le32toh(hdr->entry_size);
		dest->entries_crc32 = le32toh(hdr->entries_crc32);
	}
	return 0;

} /* parse_header */

int
gpt_load (gpt_context_t *ctx, unsigned int flags)
{
	off_t startpos;
	ssize_t n;
	unsigned int i;
	int fd = ctx->fd;
	struct gpt_header_s *hdr;
	struct gpt_entry_ondisk_s *ent;
	struct gpt_entry_s *destent;

	ctx->primary_valid = ctx->backup_valid = 0;
	memset(&ctx->primary_header, 0, sizeof(ctx->primary_header));
	memset(&ctx->backup_header, 0, sizeof(ctx->backup_header));
	if (ctx->entries != NULL) {
		free(ctx->entries);
		ctx->entries = NULL;
	}
	if ((flags & GPT_LOAD_BACKUP_ONLY) == 0) {
		startpos = lseek(fd, ctx->blocksize, SEEK_SET);
		if (startpos != (off_t) -1) {
			n = read(fd, ctx->buffer, 512);
			if (n > 0 && parse_header(ctx, n, &ctx->primary_header) == 0)
				ctx->primary_valid = 1;
		}
	}
	startpos = lseek(fd, ctx->devsize-ctx->blocksize, SEEK_SET);
	if (startpos != (off_t) -1) {
		n = read(fd, ctx->buffer, 512);
		if (n > 0 && parse_header(ctx, n, &ctx->backup_header) == 0)
			ctx->backup_valid = 1;
	}
	if (!(ctx->primary_valid || ctx->backup_valid))
		return -1;
	if (ctx->primary_valid && ctx->backup_valid) {
		if (ctx->primary_header.first_usable_lba != ctx->backup_header.first_usable_lba ||
		    ctx->primary_header.last_usable_lba != ctx->backup_header.last_usable_lba ||
		    ctx->primary_header.entries_start_lba != ctx->backup_header.entries_start_lba ||
		    ctx->primary_header.entry_size != ctx->backup_header.entry_size ||
		    ctx->primary_header.entry_count != ctx->backup_header.entry_count ||
		    ctx->primary_header.entries_crc32 != ctx->backup_header.entries_crc32)
			return -1;
	}
	hdr = (ctx->primary_valid ? &ctx->primary_header : &ctx->backup_header);
	if (hdr->entry_size < sizeof(struct gpt_entry_ondisk_s))
		return -1;
	startpos = ctx->blocksize * hdr->entries_start_lba;
	/*
	 * In the NVIDIA-special pseudo-GPT in mmblk0boot1, it counts the LBAs in
	 * both boot blocks together as if they were a single device.
	 */
	if (ctx->is_mmcboot1 && (flags & GPT_NVIDIA_SPECIAL) != 0)
		startpos -= ctx->devsize;
	startpos = lseek(fd, startpos, SEEK_SET);
	if (startpos == (off_t) -1)
		return -1;
	n = read(fd, ctx->buffer, hdr->entry_size * hdr->entry_count);
	if (n <= 0)
		return -1;
	if (hdr->entries_crc32 != crc32(0, ctx->buffer, hdr->entry_size * hdr->entry_count))
		return -1;
	ctx->entry_count = hdr->entry_count;
	ctx->entries = malloc(sizeof(struct gpt_entry_s) * hdr->entry_count);
	if (ctx->entries == NULL)
		return -1;
	for (ent = ctx->buffer, destent = ctx->entries, i = 0; i < ctx->entry_count; ent += 1, destent += 1, i += 1) {
		int j;
		memcpy(destent->type_guid, ent->type_guid, sizeof(destent->type_guid));
		memcpy(destent->part_guid, ent->part_guid, sizeof(destent->part_guid));
		destent->first_lba = le64toh(ent->first_lba);
		destent->last_lba = le64toh(ent->last_lba);
		destent->flags = le64toh(ent->flags);
		/* XXX Took a short-cut here, we should really handle Unicode properly */
		for (j = 0; j < sizeof(destent->part_name); j++)
			destent->part_name[j] = le16toh(ent->part_name_utf16[j]) & 0xFF;
	}
	return 0;

} /* gpt_load */

gpt_entry_t *
gpt_find_by_name (gpt_context_t *ctx, const char *name)
{
	struct gpt_entry_s *ent;
	size_t nlen, elen;
	unsigned int i;

	if (ctx->entries == NULL)
		return NULL;

	nlen = strlen(name);
	for (ent = ctx->entries, i = 0; i < ctx->entry_count; ent += 1, i += 1) {
		elen = strlen(ent->part_name);
		if (nlen == elen && memcmp(name, ent->part_name, nlen) == 0)
			return ent;
	}
	return NULL;

} /* gpt_find_by_name */

gpt_entry_t *
gpt_enumerate_partitions (gpt_context_t *ctx, void **iterctx)
{
	uintptr_t idx;

	idx = (uintptr_t)(*iterctx);

	if (idx >= ctx->entry_count) {
		*iterctx = 0;
		return NULL;
	}

	*iterctx = (void *)(idx + 1);
	return &ctx->entries[idx];

} /* gpt_enumerate_partitions */

int
gpt_fd (gpt_context_t *ctx)
{
	return ctx->fd;
} /* gpt_fd */

int
gpt_load_from_config (gpt_context_t *ctx)
{
	FILE *fp;
	struct gpt_entry_s *destent;
	char linebuf[256], *cp, *anchor;
	unsigned long val;
	unsigned int i;

	fp = fopen(bootpartconf, "r");
	if (fp == NULL)
		return -1;
	ctx->entries = calloc(MAX_CONFIG_ENTRIES, sizeof(struct gpt_entry_s));
	if (ctx->entries == NULL) {
		fclose(fp);
		return -1;
	}
	for (i = 0; i < MAX_CONFIG_ENTRIES && fgets(linebuf, sizeof(linebuf), fp) != NULL; i++) {
		destent = &ctx->entries[i];
		anchor = linebuf;
		cp = strchr(anchor, ':');
		if (cp == NULL || cp - anchor >= sizeof(destent->part_name))
			goto parse_error;
		memcpy(destent->part_name, anchor, cp-anchor);
		anchor = cp + 1;
		cp = strchr(anchor, ':');
		if (cp == NULL)
			goto parse_error;
		*cp = '\0';
		val = strtoul(anchor, NULL, 10);
		if (val == ULONG_MAX || val % ctx->blocksize != 0)
			goto parse_error;
		destent->first_lba = val / ctx->blocksize;
		anchor = cp + 1;
		val = strtoul(anchor, NULL, 10);
		if (val == ULONG_MAX || val == 0 || val % ctx->blocksize != 0)
			goto parse_error;
		destent->last_lba = destent->first_lba + (val / ctx->blocksize) - 1U;
	}
	ctx->entry_count = i;
	fclose(fp);
	return 0;
parse_error:
	free(ctx->entries);
	ctx->entries = NULL;
	fclose(fp);
	errno = EINVAL;
	return -1;

} /* gpt_load_from_config */
