/*
 * gpt.c
 *
 * Functions for parsing a GUID partition table.
 * Also includes functions for locating partitions
 * based on name/offset/length entries in a configuration
 * file, for handling tegra210 boot devices that
 * do not include a GPT.
 *
 * Copyright (c) 2019-2020, Matthew Madison
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

/*
 * On-device structures for a GUID partition table.
 */
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

/*
 * Context structure for the API.
 */
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

/*
 * parse_header
 *
 * Parses a GPT header that was loaded into our
 * context's storage buffer, extracting the fields
 * into a gpt_header_s struct and putting them into
 * host order.
 *
 * ctx: context pointer
 * len: length of the data in ctx->buffer
 * dest: pointer to gpt_header_s structure to populate
 *
 * Returns: 0 on success, -1 on error.
 *          errno is not set.
 */
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

/*
 * gpt_init
 *
 * Allocates and initializes a context structure
 * for the externally-facing API.
 *
 * devname: device name where the GPT is stored
 * blocksize: block size of the device, in bytes
 *
 * Returns: context pointer
 */
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

/*
 * gpt_finish
 *
 * Cleans up an API context.
 *
 * ctx: context pointer
 *
 * Returns: nothing
 */
void
gpt_finish (gpt_context_t *ctx)
{
	if (ctx == NULL)
		return;
	close(ctx->fd);
	if (ctx->entries)
		free(ctx->entries);
	free(ctx->buffer);
	free(ctx);

} /* gpt_finish */

/*
 * gpt_load
 *
 * Loads the partition table from the device that was
 * specified in gpt_init().
 *
 * A device will have a primary GPT located at the
 * second block, and a backup GPT located one block
 * from the end of the device. By default, this function
 * will load and validate both copies. If both copies
 * are valid, they must match.
 *
 * The flags argument is used for handling the boot
 * partition GPT used on some of the Tegra platforms,
 * which place only one copy at the end of the storage,
 * device (flag GPT_LOAD_BACKUP_ONLY). On platforms that
 * boot from eMMC, the GPT entries treat the two eMMC boot
 * partitions as a single device, so LBA offsets must be
 * adjusted (flag GPT_NVIDIA_SPECIAL).
 *
 * ctx: context pointer
 * flags: see above
 *
 * Returns: 0 on success, -1 on error.
 *          errno is not set.
 */
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
	 * In the NVIDIA-special pseudo-GPT in mmcblk0boot1, it counts the LBAs in
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

/*
 * gpt_find_by_name
 *
 * Returns a GPT entry for a named partition.
 * Caller must first call gpt_load() to load the
 * partition table.
 *
 * ctx: context pointer
 * name: name of partition
 *
 * Returns: NULL if not found, otherwise
 *          a pointer to the GPT entry.
 */
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

/*
 * gpt_enumerate partitions
 *
 * Iterates through the loaded GPT entries.  Caller
 * should initialize the void * pointer pointed to
 * be iterctx with 0 before the first call, then leave
 * iterctx untouched until this function returns
 * NULL at the end of the list.
 *
 * ctx: context pointer
 * iterctx: iteration context
 *
 * Returns: GPT entry pointer or NULL when
 *          there are no more entries
 */
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

/*
 * gpt_fd
 *
 * Returns the file descriptor opened for the
 * device in gpt_init(), so the caller can use
 * it for other I/O operations.
 *
 * ctx: context pointer
 *
 * Returns: >= 0: file descriptor
 *          -1 on error (errno not set)
 */
int
gpt_fd (gpt_context_t *ctx)
{
	if (ctx == NULL)
		return -1;
	return ctx->fd;

} /* gpt_fd */

/*
 * gpt_load_from_config
 *
 * Alternative to gpt_load() that parses a partition
 * configuration file instead of an actual GPT, for use
 * on tegra210 platforms that do not embed a GPT in their
 * boot devices.
 *
 * The configuration file consists of lines of the form
 *    <name>:<offset>:<length>
 *
 * where <name> is the name of the partition, <offset>
 * is the starting location as a byte offset, and
 * <length> is the length in bytes.
 *
 * ctx: context pointer
 *
 * Returns: 0 on success, -1 on error (errno not set)
 */
int
gpt_load_from_config (gpt_context_t *ctx)
{
	FILE *fp;
	struct gpt_entry_s *destent;
	char linebuf[256], *cp, *anchor;
	unsigned long val;
	unsigned int i;

	if (ctx == NULL)
		return -1;

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
