/*
 * bup.c
 *
 * Functions for parsing a Tegra bootloader update payload.
 *
 * Copyright (c) 2019, 2020 Matthew Madison
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ctype.h>
#include <tegra-eeprom/boardspec.h>
#include "bup.h"
#include "config.h"

/*
 * Structures for parsing and matching
 * TNSPEC strings.
 */
struct specfield_s {
	const char *start;
	size_t len;
};

#define MAX_SPEC_FIELDS 32
struct tnspec_s {
	unsigned int field_count;
	struct specfield_s fields[MAX_SPEC_FIELDS];
};

/*
 * BUP payload header
 */
struct bup_header_s {
	char magic[16];
	uint32_t version;
	uint32_t blob_size;
	uint32_t header_size;
	uint32_t entry_count;
	uint32_t blob_type;
	uint32_t uncomp_size;
} __attribute__((packed));

/*
 * BUP entries (in-payload format)
 */
#define OP_MODE_COMMON		0
#define OP_MODE_PREPRODUCTION	1
#define OP_MODE_PRODUCTION	2
struct bup_ods_entry_s {
	char partition[40];
	uint32_t offset;
	uint32_t length;
	uint32_t version;
	uint32_t op_mode;
	char spec[64];
} __attribute__((packed));

/*
 * BUP entry - structure for use in context
 *
 * Mirrors the on-disk structure, but adds
 * room for null terminators on strings and
 * isn't packed.
 */
struct bup_entry_s {
	uint32_t offset;
	uint32_t length;
	uint32_t version;
	uint32_t op_mode;
	char partition[41];
	char spec[65];
};

#define BUFFERSIZE (1024 * 1024 * 1024)

/*
 * API context
 */
struct bup_context_s {
	int fd;
	void *buffer;
	char our_spec_str[128];
	struct tnspec_s our_tnspec;
	unsigned int entry_count;
	struct bup_entry_s *entries;
};

static const uint8_t bup_magic[16] = "NVIDIA__BLOB__V2";
static const uint32_t bup_version = 0x00020000;
static const char bootdev[] = OTABOOTDEV;
static const char gptdev[] = OTAGPTDEV;
#define QUOTE(m_) #m_
#define XQUOTE(m_) QUOTE(m_)
static const char machineconf[] = XQUOTE(CONFIGPATH) "/machine-name.conf";
static const char rootfsconf[] = XQUOTE(CONFIGPATH) "/rootfsdev.conf";

/*
 * Specs are of the form
 *  BOARDID-FAB-BOARDSKU-BOARDREV-FUSELEVEL-CHIPREV-MACHINE-BOOTDEV
 */
static void
spec_split (const char *specstr, struct tnspec_s *tnspec)
{
	const char *cp, *hyp;
	int i;

	memset(tnspec, 0, sizeof(*tnspec));

	if (*specstr == '\0')
		return;

	for (cp = specstr, i = 0; i < MAX_SPEC_FIELDS; i += 1) {
		tnspec->fields[i].start = cp;
		hyp = strchr(cp, '-');
		if (hyp == NULL) {
			tnspec->fields[i].len = strlen(cp);
			tnspec->field_count = i + 1;
			return;
		}
		tnspec->fields[i].len = hyp - cp;
		cp = hyp + 1;
	}

	tnspec->field_count = i;

} /* spec_split */

/*
 * specs_match
 *
 * e: spec for an entry to be matched
 * s: tnspec for current system
 *
 * Order of arguments is important for wildcard field
 * matching.
 *
 * Returns non-zero on match, 0 on no-match.
 */
static int
specs_match (const struct tnspec_s *e, const struct tnspec_s *s)
{
	int i;
	/*
	 * Null spec in entry in wildcard
	 */
	if (e->field_count == 0)
		return 1;
	/*
	 * Otherwise, number of fields in specs must match
	 */
	if (e->field_count != s->field_count)
		return 0;

	/*
	 * Now check each field
	 */
	for (i = 0; i < s->field_count; i++) {
		/*
		 * Null field -> wildcard
		 */
		if (e->fields[i].len == 0 || s->fields[i].len == 0)
			continue;
		/*
		 * 'internal' in sys spec matches 'mmcblk0p<n>' in entry
		 */
		if (s->fields[i].len == 8 && memcmp(s->fields[i].start, "internal", 8) == 0 &&
		    e->fields[i].len > 8 && memcmp(e->fields[i].start, "mmcblk0p", 8) == 0)
			continue;
		/*
		 * Otherwise, text must match exactly
		 */
		if (e->fields[i].len != s->fields[i].len ||
		    memcmp(e->fields[i].start, s->fields[i].start, s->fields[i].len) != 0)
			break;
	}

	return i >= s->field_count;

} /* specs_match */

/*
 * rstrip
 *
 * Strings in BUP payload entries can be right-padded with
 * blanks, tabs, or nulls. Copies the string from the entry
 * while stripping any padding characters off the right.
 *
 * out is assumed to be of size len+1 (or larger).
 */
static void
rstrip (char *out, const char *in, size_t len)
{	char *outp = out;
	const char *inp = in;

	while (len > 0 && !(*inp == '\0' || *inp == '\t' || *inp == ' ')) {
		*outp++ = *inp++;
		len--;
	}
	*outp = '\0';

} /* rstrip */

/*
 * construct_tnspec
 *
 * Constructs the TNSPEC for the current system from
 * the EEPROM boardspec, machine name from our config file,
 * and our boot device from our config file.
 */
static int
construct_tnspec (char *buf, size_t bufsiz)
{
	char machinename[128], rootfsdev[128];
	char boardspec[128];
	int fd;
	ssize_t n;

	n = tegra_boardspec(boardspec, sizeof(boardspec)-1);
	if (n < 0)
		return n;
	boardspec[n] = '\0';

	fd = open(machineconf, O_RDONLY);
	if (fd < 0)
		return fd;
	n = read(fd, machinename, sizeof(machinename)-1);
	close(fd);
	if (n < 0)
		return n;
	while (n > 0 && !isgraph(machinename[n-1]))
		n -= 1;
	machinename[n] = '\0';

	fd = open(rootfsconf, O_RDONLY);
	if (fd < 0)
		return fd;
	n = read(fd, rootfsdev, sizeof(rootfsdev)-1);
	close(fd);
	if (n < 0)
		return n;
	while (n > 0 && !isgraph(rootfsdev[n-1]))
		n -= 1;
	rootfsdev[n] = '\0';

	n = snprintf(buf, bufsiz-1, "%s-%s-%s", boardspec, machinename, rootfsdev);
	if (n >= 0)
		buf[n] = '\0';

	return n;

} /* construct_tnspec */

/*
 * free_context
 */
static void
free_context (bup_context_t *ctx)
{
	if (ctx->fd >= 0)
		close(ctx->fd);
	if (ctx->entries)
		free(ctx->entries);
	free(ctx->buffer);
	free(ctx);
} /* free_context */

/*
 * bup_init
 *
 * Initialize BUP payload context.
 *
 * Header and the payload's directory entries
 * are validated and stored in the context.
 */
bup_context_t *
bup_init (const char *pathname)
{
	int fd;
	ssize_t n, totsize;
	bup_context_t *ctx;
	struct bup_header_s *hdr;
	struct stat st;
	off_t payload_size;
	uint8_t *bufp;
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
	if (construct_tnspec(ctx->our_spec_str, sizeof(ctx->our_spec_str)) < 0) {
		free(ctx);
		return NULL;
	}
	spec_split(ctx->our_spec_str, &ctx->our_tnspec);

	if (pathname == NULL)
		return ctx;

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
	totsize = hdr->header_size + hdr->entry_count * sizeof(struct bup_ods_entry_s);
	if (totsize > BUFFERSIZE) {
		fprintf(stderr, "%s: cannot load all update entries\n", pathname);
		free_context(ctx);
		return NULL;
	}
	while (totsize > n) {
		ssize_t rlen = read(fd, (uint8_t *)ctx->buffer + n, totsize - n);
		if (rlen < 0) {
			free_context(ctx);
			return NULL;
		}
		if (rlen == 0) {
			fprintf(stderr, "%s: premature EOF\n", pathname);
			free_context(ctx);
			return NULL;
		}
		n += rlen;
	}
	ctx->entries = malloc(hdr->entry_count * sizeof(struct bup_entry_s));
	if (ctx->entries == NULL) {
		free_context(ctx);
		return NULL;
	}

	ctx->entry_count = hdr->entry_count;
	for (i = 0, bufp = (uint8_t *)ctx->buffer + hdr->header_size;
	     i < ctx->entry_count;
	     i++, bufp += sizeof(struct bup_ods_entry_s)) {
		struct bup_ods_entry_s *odsent = (struct bup_ods_entry_s *) bufp;		
		rstrip(ctx->entries[i].partition, odsent->partition, sizeof(odsent->partition));
		rstrip(ctx->entries[i].spec, odsent->spec, sizeof(odsent->spec));
		ctx->entries[i].offset = odsent->offset;
		ctx->entries[i].length = odsent->length;
		ctx->entries[i].version = odsent->version;
		ctx->entries[i].op_mode = odsent->op_mode;
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

/*
 * bup_finish
 */
void
bup_finish (bup_context_t *ctx)
{
	free_context(ctx);

} /* bup_finish */

/*
 * bup_gpt_device
 *
 * returns the GPT device name from the configuration file.
 */
const char *
bup_gpt_device (bup_context_t *ctx)
{
	return gptdev;

} /* bup_gpt_device */

/*
 * bup_boot_device
 *
 * returns the boot device name from the configuration file.
 */
const char *
bup_boot_device (bup_context_t *ctx)
{
	return bootdev;

} /* bup_boot_device */

/*
 * bup_tnspec
 *
 * returns the TNSPEC string from the configuration file.
 */
const char *
bup_tnspec (bup_context_t *ctx)
{
	return ctx->our_spec_str;

} /* bup_tnspec */

/*
 * bup_enumerate_entries
 *
 * Iterates through the entries in the payload, returning
 * those entries that match the system TNSPEC (or are non-spec entries).
 *
 * First call should be with iterctx pointing to NULL (0). Do not modify
 * iterctx between calls.
 *
 * Returns non-zero on success, 0 on failure.
 * 
 */
int
bup_enumerate_entries (bup_context_t *ctx, void **iterctx, const char **partname,
		       off_t *offset, size_t *length, unsigned int *version)
{
	unsigned int i;
	uintptr_t start;
	struct bup_entry_s *ent;
	struct tnspec_s entspec;

	start = (uintptr_t)(*iterctx);
	for (i = start; i < ctx->entry_count; i++) {
		ent = &ctx->entries[i];
		if (ent->op_mode != OP_MODE_PREPRODUCTION) {
			spec_split(ent->spec, &entspec);
			if (specs_match(&entspec, &ctx->our_tnspec))
				break;
		}
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

} /* bup_enumerate_entries */


#define MAX_PARTS 64
static int
in_list (const char **list, unsigned int listsize, const char *val)
{
	unsigned int i;
	for (i = 0; i < listsize; i++)
		if (strcmp(list[i], val) == 0)
			return 1;
	return 0;
} /* in_list */

/*
 * bup_find_missing_entries
 *
 * Returns a list of partitions for which no TNSPSEC-matching entries are
 * present in the update payload. Used for validating that the payload
 * is suitable for use on the current system before applying an update.
 *
 * Returns number of missing entries. If missing_parts is non-NULL,
 * pointers to the partition names missing will be listed, up to
 * max_missing (which should be the length of the array provided
 * by the caller). The returned number of missing entries can
 * exceed max_missing, if the provided array is too small.
 *
 * Note that there is no guarantee that a BUP payload will be ordered
 * by partition name, which makes the logic here a little more complicated.
 *
 * Returns -1 for any errors.
 * 
 */
int
bup_find_missing_entries (bup_context_t *ctx, const char **missing_parts,
			  size_t max_missing)
{
	unsigned int i, partcount, matchcount, missing_count;
	struct bup_entry_s *ent;
	struct tnspec_s entspec;
	const char *all_parts[MAX_PARTS], *matching_parts[MAX_PARTS];

	partcount = matchcount = 0;
	for (i = 0; i < ctx->entry_count; i++) {
		ent = &ctx->entries[i];
		/*
		 * Don't need to bother with preproduction or non-TNSPECed entries
		 */
		if (ent->op_mode == OP_MODE_PREPRODUCTION || ent->spec[0] == '\0')
			continue;
		/*
		 * If we've already matched this part, skip
		 */
		if (in_list(matching_parts, matchcount, ent->partition))
			continue;
		if (!in_list(all_parts, partcount, ent->partition)) {
			if (partcount >= MAX_PARTS)
				return -1;
			all_parts[partcount++] = ent->partition;
		}

		spec_split(ent->spec, &entspec);
		/*
		 * No need to check the array length here, covered
		 * by the check above since both arrays are the same
		 * size
		 */
		if (specs_match(&entspec, &ctx->our_tnspec))
			matching_parts[matchcount++] = ent->partition;
	}

	missing_count = 0;
	for (i = 0; i < partcount; i++) {
		if (!in_list(matching_parts, matchcount, all_parts[i])) {
			if (missing_parts != NULL && missing_count < max_missing)
				missing_parts[missing_count] = all_parts[i];
			missing_count += 1;
		}
	}

	return missing_count;

} /* bup_find_missing_entries */

/*
 * bup_setpos
 */
off_t
bup_setpos (bup_context_t *ctx, off_t offset)
{
	return lseek(ctx->fd, offset, SEEK_SET);

} /* bup_setpos */

/*
 * bup_read
 */
ssize_t
bup_read (bup_context_t *ctx, void *buf, size_t bufsize)
{
	return read(ctx->fd, buf, bufsize);

} /* bup_read */
