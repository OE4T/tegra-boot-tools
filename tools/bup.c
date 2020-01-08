/*
 * bup.c
 *
 * Functions for parsing a Tegra bootloader update payload.
 *
 * Copyright (c) 2019, Matthew Madison
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "bup.h"

struct bup_header_s {
	char magic[16];
	uint32_t version;
	uint32_t blob_size;
	uint32_t header_size;
	uint32_t entry_count;
	uint32_t blob_type;
	uint32_t uncomp_size;
} __attribute__((packed));


#define OP_MODE_COMMON		0
#define OP_MODE_PREPRODUCTION	1
#define OP_MODE_PRODUCTION	2
struct bup_entry_s {
	char partition[40];
	uint32_t offset;
	uint32_t length;
	uint32_t version;
	uint32_t op_mode;
	char spec[64];
} __attribute__((packed));

#define BUFFERSIZE (1024 * 1024 * 1024)

struct bup_context_s {
	int fd;
	void *buffer;
	char *our_spec;
	char *bootdev;
	char *gptdev;
	unsigned int entry_count;
	struct bup_entry_s *entries;
};

static const uint8_t bup_magic[16] = "NVIDIA__BLOB__V2";
static const uint32_t bup_version = 0x00020000;

struct specinfo_s {
	const char *start;
	size_t len;
};

static int
load_configuration (bup_context_t *ctx)
{
	int fd;
	ssize_t n;
	struct stat st;
	char *start, *line, *lineend;

	fd = open("/etc/nv_boot_control.conf", O_RDONLY);
	if (fd < 0)
		return -1;
	if (fstat(fd, &st) != 0) {
		close(fd);
		return -1;
	}
	if (st.st_size >= BUFFERSIZE-1) {
		close(fd);
		errno = EFBIG;
		return -1;
	}
	n = read(fd, ctx->buffer, BUFFERSIZE-2);
	if (n <= 0) {
		close(fd);
		return -1;
	}
	close(fd);
	start = ctx->buffer;
	*(start + n) = '\n';
	*(start + n+1) = '\0';
	/*
	 * config file format is
	 *
	 * VARNAME<whitespace>VALUE
	 *
	 * but allow for leading and trailing whitespace,
	 * and trim it.
	 */
	for (line = start; *line != '\0'; line = lineend + 1) {
		char *var, *val, *cp;
		lineend = strchr(line, '\n');
		*lineend = '\0';
		for (cp = line; *cp == '\t' || *cp == ' '; cp++);
		if (*cp == '\0')
			continue;
		var = cp;
		for (cp += 1; *cp != '\t' && *cp != ' '; cp++);
		if (*cp == '\0')
			continue;
		*cp = '\0';
		for (cp += 1; *cp == '\t' || *cp == ' '; cp++);
		val = cp;
		for (cp = lineend-1; cp > val && (*cp == '\t' || *cp == ' '); cp--);
		*(cp + 1) = '\0';
		if (strcmp(var, "TEGRA_OTA_BOOT_DEVICE") == 0)
			ctx->bootdev = strdup(val);
		else if (strcmp(var, "TEGRA_OTA_GPT_DEVICE") == 0)
			ctx->gptdev = strdup(val);
		else if (strcmp(var, "TNSPEC") == 0)
			ctx->our_spec = strdup(val);
	}

	return 0;

} /* load_configuration */

static void
free_context (bup_context_t *ctx)
{
	if (ctx->bootdev)
		free(ctx->bootdev);
	if (ctx->gptdev)
		free(ctx->gptdev);
	if (ctx->our_spec)
		free(ctx->our_spec);
	if (ctx->fd >= 0)
		close(ctx->fd);
	if (ctx->entries)
		free(ctx->entries);
	free(ctx->buffer);
	free(ctx);
} /* free_context */

bup_context_t *
bup_init (const char *pathname)
{
	int fd;
	ssize_t n, totsize;
	bup_context_t *ctx;
	struct bup_header_s *hdr;
	struct stat st;
	off_t payload_size;
	int i;

	ctx = malloc(sizeof(*ctx));
	if (ctx == NULL)
		return NULL;
	memset(ctx, 0, sizeof(*ctx));
	ctx->fd = -1;
	ctx->buffer = malloc(BUFFERSIZE);
	if (ctx->buffer == NULL) {
		free(ctx);
		return NULL;
	}
	if (load_configuration(ctx) != 0) {
		free_context(ctx);
		return NULL;
	}
	if (ctx->bootdev == NULL || ctx->gptdev == NULL || ctx->our_spec == NULL) {
		free_context(ctx);
		return NULL;
	}

	fd = open(pathname, O_RDONLY);
	if (fd < 0) {
		free_context(ctx);
		return NULL;
	}
	ctx->fd = fd;
	if (fstat(fd, &st) != 0) {
		free_context(ctx);
		return NULL;
	}
	payload_size = st.st_size;
	n = read(fd, ctx->buffer, BUFFERSIZE);
	if (n < sizeof(struct bup_header_s)) {
		free_context(ctx);
		return NULL;
	}
	hdr = ctx->buffer;
	if (memcmp(hdr->magic, bup_magic, sizeof(hdr->magic)) != 0) {
		fprintf(stderr, "%s: bad header magic\n", pathname);
		free_context(ctx);
		return NULL;
	}
	if (hdr->version != bup_version) {
		fprintf(stderr, "%s: bad header version\n", pathname);
		free_context(ctx);
		return NULL;
	}
	if (hdr->blob_type != 0) {
		fprintf(stderr, "%s: bad blob type\n", pathname);
		free_context(ctx);
		return NULL;
	}
	if (hdr->header_size < sizeof(struct bup_header_s)) {
		fprintf(stderr, "%s: bad header length\n", pathname);
		free_context(ctx);
		return NULL;
	}
	totsize = hdr->header_size + hdr->entry_count * sizeof(struct bup_entry_s);
	if (totsize > BUFFERSIZE) {
		fprintf(stderr, "%s: cannot load all update entries\n", pathname);
		free_context(ctx);
		return NULL;
	}
	while (totsize > n) {
		ssize_t i = read(fd, (uint8_t *)ctx->buffer + n, totsize - n);
		if (i < 0) {
			free_context(ctx);
			return NULL;
		}
		if (i == 0) {
			fprintf(stderr, "%s: premature EOF\n", pathname);
			free_context(ctx);
			return NULL;
		}
		n += i;
	}
	ctx->entries = malloc(hdr->entry_count * sizeof(struct bup_entry_s));
	if (ctx->entries == NULL) {
		free_context(ctx);
		return NULL;
	}
	memcpy(ctx->entries, (uint8_t *)ctx->buffer + hdr->header_size,
	       hdr->entry_count * sizeof(struct bup_entry_s));
	ctx->entry_count = hdr->entry_count;

	for (i = 0; i < ctx->entry_count; i++) {
		if (ctx->entries[i].offset > payload_size ||
		    ctx->entries[i].offset + ctx->entries[i].length > payload_size) {
			fprintf(stderr, "%s: entry %d (%s) beyond end of file\n",
				pathname, i, ctx->entries[i].partition);
			free_context(ctx);
			return NULL;
		}
	}

	return ctx;

} /* bup_init */

void
bup_finish (bup_context_t *ctx)
{
	free_context(ctx);
}

const char *
bup_gpt_device (bup_context_t *ctx)
{
	return ctx->gptdev;
}
const char *
bup_boot_device (bup_context_t *ctx)
{
	return ctx->bootdev;
}

/*
 * Specs are of the form
 *  BOARDID-FAB-BOARDSKU-BOARDREV-FUSELEVEL-CHIPREV-MACHINE-BOOTDEV
 */
static int
spec_split (const char *spec, struct specinfo_s *elements, int max_elements)
{
	const char *cp, *hyp;
	int i;

	for (cp = spec, i = 0; i < max_elements; i += 1) {
		elements[i].start = cp;
		hyp = strchr(cp, '-');
		if (hyp == NULL) {
			elements[i].len = strlen(cp);
			return i + 1;
		}
		elements[i].len = hyp - cp;
		cp = hyp + 1;
	}

	return i;

}

int
bup_enumerate_entries (bup_context_t *ctx, void **iterctx, const char **partname,
		       off_t *offset, size_t *length, unsigned int *version)
{
	unsigned int i;
	uintptr_t start;
	struct bup_entry_s *ent;
	struct specinfo_s sysspecinfo[32], entspecinfo[32];
	int sysinfocount, entinfocount;

	sysinfocount = spec_split(ctx->our_spec, sysspecinfo, sizeof(sysspecinfo)/sizeof(sysspecinfo[0]));
	start = (uintptr_t)(*iterctx);
	for (i = start; i < ctx->entry_count; i++) {
		int j;
		ent = &ctx->entries[i];
		if (ent->op_mode == OP_MODE_PREPRODUCTION)
			continue;
		/*
		 * null spec is wildcard
		 */
		if (ent->spec[0] == '\0')
			break;
		entinfocount = spec_split(ent->spec, entspecinfo, sizeof(entspecinfo)/sizeof(entspecinfo[0]));
		/*
		 * element count must match
		 */
		if (sysinfocount != entinfocount)
			continue;
		for (j = 0; j < sysinfocount; j++) {
			struct specinfo_s *e = &entspecinfo[j];
			struct specinfo_s *s = &sysspecinfo[j];
			/*
			 * null element is wildcard, otherwise
			 * element must match
			 */
			if (e->len == 0 || s->len == 0)
				continue;
			/*
			 * 'internal' in sys spec matches mmcblk0p<n> in
			 * bup entry
			 */
			if (s->len == 8 && memcmp(s->start, "internal", 8) == 0 &&
			    e->len > 8 && memcmp(e->start, "mmcblk0p", 8) == 0)
				continue;
			/*
			 * otherwise text must match exactly
			 */
			if (e->len != s->len || memcmp(e->start, s->start, s->len) != 0)
				break;
		}
		if (j < sysinfocount)
			continue; // not everything matched
		break;
	}
	if (i >= ctx->entry_count) {
		*iterctx = 0;
		return 0;
	}

	*partname = ctx->entries[i].partition;
	*offset = (off_t)(ctx->entries[i].offset);
	*length = (size_t)(ctx->entries[i].length);
	*version = (unsigned int)(ctx->entries[i].version);
	start = i;
	*iterctx = (void *)(start + 1);
	return 1;
}

off_t
bup_setpos (bup_context_t *ctx, off_t offset)
{
	return lseek(ctx->fd, offset, SEEK_SET);
}

ssize_t
bup_read (bup_context_t *ctx, void *buf, size_t bufsize)
{
	return read(ctx->fd, buf, bufsize);
}
