/*
 * bootinfo.c
 *
 * Functions for working with the boot information
 * block used by tegra-bootinfo.
 *
 * Copyright (c) 2019-2022, Matthew Madison
 */

// _GNU_SOURCE is needed for O_PATH definition in fcntl.h
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <zlib.h>
#include "bootinfo.h"
#include "util.h"
#include "config.h"

static const char DEVICE_MAGIC[8] = {'B', 'O', 'O', 'T', 'I', 'N', 'F', 'O'};
#define DEVICE_MAGIC_SIZE sizeof(DEVICE_MAGIC)

static const uint16_t DEVINFO_VERSION_OLDER = 1;
static const uint16_t DEVINFO_VERSION_OLD = 2;
static const uint16_t DEVINFO_VERSION_CURRENT = 3;

#ifndef EXTENSION_SECTOR_COUNT
#define EXTENSION_SECTOR_COUNT 1
#endif

#define MAX_EXTENSION_SECTORS 511

#if (EXTENSION_SECTOR_COUNT == 0) || (EXTENSION_SECTOR_COUNT > MAX_EXTENSION_SECTORS)
#error "EXTENSION_SECTOR_COUNT out of range"
#endif


/*
 * The device_info structure is the on-disk (or on-storage-device)
 * bootinfo block.
 */
#define DEVINFO_BLOCK_SIZE 512
#define EXTENSION_SIZE (EXTENSION_SECTOR_COUNT*512)

struct device_info {
	unsigned char magic[DEVICE_MAGIC_SIZE];
	uint16_t devinfo_version;
	uint8_t	 flags;
	uint8_t	 failed_boots;
	uint32_t crcsum;
	uint8_t	 sernum;
	uint8_t	 unused__;
	uint16_t ext_sectors;
} __attribute__((packed));
#define FLAG_BOOT_IN_PROGRESS	(1<<0)
#define DEVINFO_HDR_SIZE sizeof(struct device_info)
#define VARSPACE_SIZE (DEVINFO_BLOCK_SIZE+EXTENSION_SIZE-(DEVINFO_HDR_SIZE+sizeof(uint32_t)))
/*
 * Maximum size of a variable name was arbitrarily set to DEVINFO_BLOCK_SIZE
 * in earlier versions, so retain that here.
 *
 * Maximum size for a variable value is all of the variable space minus two bytes
 * for null terminators (for name and value) and one byte for a name, plus one
 * byte for the null character terminating the variable list.
 */
#define MAX_NAME_SIZE (DEVINFO_BLOCK_SIZE)
#define MAX_VALUE_SIZE (VARSPACE_SIZE-4)

struct info_var {
	struct info_var *next;
	char *name;
	char *value;
	bool free_name, free_value;
};

struct bootinfo_var_iter_context_s {
	struct info_var *last;
	bootinfo_context_t *ctx;
};

/*
 * flags passed to bootinfo_open() include
 * access bits and CREAT bit, like open().
 */
#define BOOTINFO_O_ACCMODE 0x03

#define MAX_OFFSET_COUNT 4
struct bootinfo_context_s {
	int fd;
	int lockfd;
	bool readonly;
	bool dirty;
	bool valid[2];
	bool reset_bootdev_status;
	int current;
	const char *devinfo_dev;
	int offset_count;
	// Only the first two entries in these arrays are used for
	// read-write access; additional pairs of entries are allowed
	// for read-only access, to handle upgrades that change offsets.
	off_t devinfo_offset[MAX_OFFSET_COUNT];
	off_t extension_offset[MAX_OFFSET_COUNT];
	struct device_info curinfo;
	struct info_var *vars;
	size_t varsize;
	uint8_t infobuf[MAX_OFFSET_COUNT][DEVINFO_BLOCK_SIZE+EXTENSION_SIZE];
};

struct devinfo_offset_s {
	unsigned long chipid;
	const char *devinfo_dev;
	int offset_count;
	off_t devinfo_offset[MAX_OFFSET_COUNT];
	off_t extension_offset[MAX_OFFSET_COUNT];
} devinfo_offset_table[] = {
	// No GPT block in SPI flash-based Nanos, so use same layout
	// for both eMMC and SPI flash devices
	{ .chipid = 0x21,
	  .offset_count = 2,
	  .devinfo_offset = {
		  [0] = -512,
		  [1] = -(65536 + 512),
	  },
	  .extension_offset = {
		  [0] = -((EXTENSION_SECTOR_COUNT + 2) * 512),
		  [1] = -(65536 + (EXTENSION_SECTOR_COUNT + 2) * 512),
	  },
	},
	// All T18x use eMMC
	{ .chipid = 0x18,
	  .offset_count = 2,
	  .devinfo_offset = {
		  [0] = -((36 + 1) * 512),
		  [1] = -((36 + 2) * 512),
	  },
	  .extension_offset = {
		  [0] = -((EXTENSION_SECTOR_COUNT + 36 + 2) * 512),
		  [1] = -((EXTENSION_SECTOR_COUNT * 2 + 36 + 2) * 512),
	  },
	},
	// AGX Xavier and Xavier NX with eMMC
	{ .chipid = 0x19,
	  .devinfo_dev = "/dev/mmcblk0boot1",
	  .offset_count = 2,
	  .devinfo_offset = {
		  [0] = -((36 + 1) * 512),
		  [1] = -((36 + 2) * 512),
	  },
	  .extension_offset = {
		  [0] = -((EXTENSION_SECTOR_COUNT + 36 + 2) * 512),
		  [1] = -((EXTENSION_SECTOR_COUNT * 2 + 36 + 2) * 512),
	  },
	},
	// Xavier NX with SDcard - bootinfo must not share the
	// same 64KiB erase block with the pseudo-GPT, to prevent
	// bricking if power fails during a bootinfo update
	// There is space between VER_b and the GPT block
	// that we can use for this. We also ensure that the
	// two areas are in separate erase blocks, so the base
	// and extension sectors are grouped together for each
	// copy.
	//
	// Original implementation used the same block as the GPT,
	// so make it possible to upgrade by adding the old offsets
	// to the list (for read access only).
	{ .chipid = 0x19,
	  .devinfo_dev = "/dev/mtdblock0",
	  .offset_count = 4,
	  .devinfo_offset = {
		  [0] = -((128 + 1) * 512),  // 128 sectors = 64KiB
		  [1] = -((256 + 1) * 512),  // 256 sectors = 2 * 64KiB
		  [2] = -((36 + 1) * 512),
		  [3] = -((36 + 2) * 512),
	  },
	  .extension_offset = {
		  [0] = -((EXTENSION_SECTOR_COUNT + 128 + 1) * 512),
		  [1] = -((EXTENSION_SECTOR_COUNT + 256 + 1) * 512),
		  [2] = -((EXTENSION_SECTOR_COUNT + 36 + 2) * 512),
		  [3] = -((EXTENSION_SECTOR_COUNT * 2 + 36 + 2) * 512),
	  },
	},
};
#define OFFSET_TABLE_COUNT (sizeof(devinfo_offset_table)/sizeof(devinfo_offset_table[0]))

/*
 * Order here is important. Some systems may have both
 * eMMC and a SPI flash, and we prefer the eMMC.
 *
 * TBD: allow for customization/configuration of this list.
 */
static const char *devinfo_devices[] = {
	[0] = "/dev/mmcblk0boot1",
	[1] = "/dev/mtdblock0",
};

/*
 * identify_chip
 *
 * Look up the tegra chip ID and adjust offsets based
 * on whether it's a t210 or not.
 *
 * Returns: 0 on success, -1 on error (errno not set)
 */
static unsigned long
identify_chip ()
{
	char buf[32];
	ssize_t n;
	unsigned long chipid;
	int fd = open("/sys/module/tegra_fuse/parameters/tegra_chip_id", O_RDONLY);

	if (fd < 0)
		return 0;
	n = read(fd, buf, sizeof(buf));
	close(fd);
	if (n <= 0)
		return 0;
	chipid = strtoul(buf, NULL, 0);
	return chipid;

} /* identify_chip */

/*
 * find_storage_dev
 *
 * Identifies the devinfo storage device
 * by iterating through devinfo_devices[]. First
 * successful access(F_OK) wins.
 *
 * returns 0 on success, non-0 on failure.
 */
static int
find_storage_dev (bootinfo_context_t *ctx)
{
	unsigned int i;

	for (i = 0; i < sizeof(devinfo_devices)/sizeof(devinfo_devices[0]); i++) {
		if (access(devinfo_devices[i], F_OK) == 0) {
			ctx->devinfo_dev = devinfo_devices[i];
			return 0;
		}
	}
	ctx->devinfo_dev = NULL;
	errno = ENODEV;
	return -1;

} /* find_storage_dev */

/*
 * parse_vars
 *
 * A variable consists of a null-terminated name followed
 * by a null-terminated value.	Names and values are simply
 * concatenated into the space after the header, up to the
 * block size. A null byte at the beginning of a variable
 * name indicates the end of the list.
 *
 * It's possible to have a null value, but in this implementation
 * null-valued variables are not written to the info block;
 * setting a value to the null string deletes the variable.
 */
static int
parse_vars (struct bootinfo_context_s *ctx)
{
	struct info_var *var, *last;
	char *cp, *endp, *valp;
	ssize_t remain, varbytes;

	ctx->vars = NULL;
	if (ctx->current < 0) {
		fprintf(stderr, "error: parse_vars called with no valid info block\n");
		return -1;
	}
	for (cp = (char *)(ctx->infobuf[ctx->current] + DEVINFO_HDR_SIZE),
		     remain = sizeof(ctx->infobuf[ctx->current]) - (DEVINFO_HDR_SIZE+sizeof(uint32_t)),
		     ctx->varsize = 0,
		     last = NULL;
	     remain > 0 && *cp != '\0';
	     cp += varbytes, remain -= varbytes) {
		for (endp = cp + 1, varbytes = 1;
		     varbytes < remain && *endp != '\0';
		     endp++, varbytes++);
		if (varbytes >= remain)
			break;
		for (valp = endp + 1, varbytes += 1;
		     varbytes < remain && *valp != '\0';
		     valp++, varbytes++);
		if (varbytes >= remain)
			break;
		var = calloc(1, sizeof(struct info_var));
		if (var == NULL) {
			perror("variable storage");
			return -1;
		}
		var->name = cp;
		var->value = endp + 1;
		if (last == NULL)
			ctx->vars = var;
		else
			last->next = var;
		last = var;
		varbytes += 1; /* for trailing null at end of value */
		ctx->varsize += varbytes;
	}

	return 0;

} /* parse_vars */

/*
 * pack_vars
 *
 * Pack the list of variables into the current devinfo block.
 */
static int
pack_vars (struct bootinfo_context_s *ctx, int idx)
{
	struct info_var *var;
	char *cp;
	size_t remain, nlen, vlen;

	if (idx != 0 && idx != 1)
		return -1;
	if (ctx->vars == NULL)
		return 0;
	for (var = ctx->vars, cp = (char *)(ctx->infobuf[idx] + DEVINFO_HDR_SIZE),
		     remain = sizeof(ctx->infobuf[idx]) - (DEVINFO_HDR_SIZE+sizeof(uint32_t)+1);
	     var != NULL && remain > 0;
	     var = var->next) {
		nlen = strlen(var->name) + 1;
		vlen = strlen(var->value) + 1;
		if (nlen + vlen > remain) {
			fprintf(stderr, "error: variables list too large\n");
			return -1;
		}
		memcpy(cp, var->name, nlen);
		cp += nlen; remain -= nlen;
		memcpy(cp, var->value, vlen);
		cp += vlen; remain -= vlen;
	}
	if (var != NULL || remain == 0) {
		fprintf(stderr, "error: variables list too large\n");
		return -1;
	}
	*cp = '\0';

	return 0;

} /* pack_vars */

/*
 * free_one_var
 *
 * Frees one variable.
 */
static void
free_one_var (struct info_var *var)
{
	if (var->free_name)
		free(var->name);
	if (var->free_value)
		free(var->value);
	free(var);

} /* free_one_var */

/*
 * free_vars
 *
 * Frees the memory for variable tracking.
 */
static void
free_vars (struct bootinfo_context_s *ctx)
{
	struct info_var *var, *vnext;
	for (var = ctx->vars; var != NULL; var = vnext) {
		vnext = var->next;
		free_one_var(var);
	}
	ctx->vars = NULL;
	ctx->varsize = 0;

} /* free_vars */

/*
 * update_bootinfo
 *
 * Write out a device info block based on the current context.
 */
static int
update_bootinfo (struct bootinfo_context_s *ctx)
{
	uint32_t *crcptr;
	struct device_info *info;
	ssize_t n, cnt;
	int idx;

	if (ctx->readonly) {
		errno = EROFS;
		return -1;
	}

	/*
	 * Invalid current index -> initialize
	 */
	if (ctx->current < 0 || ctx->current > 1)
		idx = 0;
	else
		idx = 1 - ctx->current;

	info = (struct device_info *) ctx->infobuf[idx];
	crcptr = (uint32_t *) &ctx->infobuf[idx][DEVINFO_BLOCK_SIZE + EXTENSION_SIZE - sizeof(uint32_t)];
	memset(info, 0, DEVINFO_BLOCK_SIZE);
	memcpy(info->magic, DEVICE_MAGIC, sizeof(info->magic));
	info->devinfo_version = DEVINFO_VERSION_CURRENT;
	info->flags = ctx->curinfo.flags;
	info->failed_boots = ctx->curinfo.failed_boots;
	info->sernum = ctx->curinfo.sernum + 1;
	info->ext_sectors = EXTENSION_SECTOR_COUNT;
	/* Don't pack vars if current index is invalid */
	if (ctx->current >= 0 && pack_vars(ctx, idx) < 0)
		return -1;
	info->crcsum = crc32(0, ctx->infobuf[idx], DEVINFO_BLOCK_SIZE);
	*crcptr = crc32(0, &ctx->infobuf[idx][DEVINFO_BLOCK_SIZE], EXTENSION_SIZE-sizeof(uint32_t));

	if (lseek(ctx->fd, ctx->devinfo_offset[idx], SEEK_END) < 0)
		return -1;
	for (n = 0; n < DEVINFO_BLOCK_SIZE; n += cnt) {
		cnt = write(ctx->fd, ctx->infobuf[idx] + n, DEVINFO_BLOCK_SIZE-n);
		if (cnt < 0)
			return -1;
	}
	if (lseek(ctx->fd, ctx->extension_offset[idx], SEEK_END) < 0)
		return -1;

	for (n = 0; n < EXTENSION_SIZE; n += cnt) {
		cnt = write(ctx->fd, ctx->infobuf[idx] + DEVINFO_BLOCK_SIZE + n, EXTENSION_SIZE-n);
		if (cnt < 0)
			return -1;
	}

	ctx->dirty = false;
	return 0;

} /* update_bootinfo */

/*
 * boot_devinfo_init
 *
 * Initializes the bootinfo storage blocks
 * by writing all zeroes.
 *
 */
static int
boot_devinfo_init (bootinfo_context_t *ctx)
{
	ssize_t n, cnt;
	uint8_t *buf;
	int i;

	buf = calloc(1, DEVINFO_BLOCK_SIZE + EXTENSION_SIZE);
	if (buf == NULL)
		return -1;

	/*
	 * Start by zeroing out the primary and backup blocks
	 */
	for (i = 0; i < 2; i++) {
		if (lseek(ctx->fd, ctx->devinfo_offset[i], SEEK_END) < 0)
			break;
		for (n = 0; n < DEVINFO_BLOCK_SIZE; n += cnt) {
			cnt = write(ctx->fd, buf+n, DEVINFO_BLOCK_SIZE-n);
			if (cnt < 0)
				break;
		}
		if (n < DEVINFO_BLOCK_SIZE)
			break;
		if (lseek(ctx->fd, ctx->extension_offset[i], SEEK_END) < 0)
			break;
		for (n = 0; n < EXTENSION_SIZE; n += cnt) {
			cnt = write(ctx->fd, buf+DEVINFO_BLOCK_SIZE+n, EXTENSION_SIZE-n);
			if (cnt < 0)
				break;
		}
		if (n < EXTENSION_SIZE)
			break;
	}
	free(buf);
	if (i < 2)
		return -1;

	ctx->current = -1;
	return update_bootinfo(ctx);

} /* boot_devinfo_init */

/*
 * bootinfo_open
 *
 * Tries to find a valid bootinfo block, and initializes a context
 * if one is found.
 *
 * Returns negative value on an underlying error or if neither block
 * is valid.
 *
 * Returns 0 on success, and ctxp will be set to point to a valid
 * context.  Caller MUST call bootinfo_close to clean up the context.
 *
 */
int
bootinfo_open (unsigned int flags, struct bootinfo_context_s **ctxp)
{
	struct bootinfo_context_s *ctx;
	struct device_info *dp;
	unsigned long chipid;
	ssize_t n, cnt;
	int i, dirfd;
	unsigned int offset_table_index;

	*ctxp = NULL;
	ctx = calloc(1, sizeof(struct bootinfo_context_s));
	if (ctx == NULL)
		return -1;

	chipid = identify_chip();
	if (chipid != 0x21 && chipid != 0x18 && chipid != 0x19) {
		errno = ENODEV;
		goto failure_exit;
	}
	if (find_storage_dev(ctx) < 0)
		goto failure_exit;
	for (offset_table_index = 0; offset_table_index < OFFSET_TABLE_COUNT; offset_table_index++) {
		struct devinfo_offset_s *entry = &devinfo_offset_table[offset_table_index];
		if (chipid == entry->chipid && (entry->devinfo_dev == NULL ||
						strcmp(ctx->devinfo_dev, entry->devinfo_dev) == 0)) {
			ctx->offset_count = entry->offset_count;
			for (i = 0; i < ctx->offset_count; i++) {
				ctx->devinfo_offset[i] = entry->devinfo_offset[i];
				ctx->extension_offset[i] = entry->extension_offset[i];
			}
			break;
		}
	}
	if (offset_table_index >= OFFSET_TABLE_COUNT) {
		errno = ENODEV;
		goto failure_exit;
	}

	ctx->fd = ctx->lockfd = -1;
	ctx->readonly = (flags & BOOTINFO_O_ACCMODE) == BOOTINFO_O_RDONLY;
	if (!ctx->readonly)
		ctx->reset_bootdev_status = set_bootdev_writeable_status(ctx->devinfo_dev, true);

	ctx->fd = open(ctx->devinfo_dev, (ctx->readonly ? O_RDONLY : O_RDWR|O_DSYNC));
	if (ctx->fd < 0)
		goto failure_exit;
	/*
	 * We use a lockfile to coordinate access to the bootinfo block
	 * from multiple processes
	 */
	dirfd = open("/run/tegra-bootinfo", O_PATH);
	if (dirfd < 0) {
		if (mkdir("/run/tegra-bootinfo", 02770) < 0)
			goto failure_exit;
		dirfd = open("/run/tegra-bootinfo", O_PATH);
		if (dirfd < 0)
			goto failure_exit;
	}
	ctx->lockfd = openat(dirfd, "lockfile", O_CREAT|O_RDWR, 0770);
	close(dirfd);
	if (ctx->lockfd < 0)
		goto failure_exit;
	if (flock(ctx->lockfd, (ctx->readonly ? LOCK_SH : LOCK_EX)) < 0)
		goto failure_exit;

	for (i = 0; i < ctx->offset_count; i++) {
		/*
		 * Read base block
		 */
		if (lseek(ctx->fd, ctx->devinfo_offset[i], SEEK_END) < 0)
			continue;
		for (n = 0; n < DEVINFO_BLOCK_SIZE; n += cnt) {
			cnt = read(ctx->fd, &ctx->infobuf[i][n], DEVINFO_BLOCK_SIZE-n);
			if (cnt < 0)
				break;
		}
		if (n < DEVINFO_BLOCK_SIZE)
			continue;

		dp = (struct device_info *)(ctx->infobuf[i]);

		if (memcmp(dp->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE) != 0)
			continue;
		/*
		 * Automatically convert older layouts to current:
		 *
		 * V1 was one sector with different structure and no variable space
		 * V2 was one sector with variables but no extension space
		 */
		if (dp->devinfo_version == DEVINFO_VERSION_OLDER) {
			uint8_t inprogress = dp->flags;
			dp->flags = (inprogress != 0 ? FLAG_BOOT_IN_PROGRESS : 0);
			ctx->valid[i] = true;
			memset(ctx->infobuf[i] + sizeof(struct device_info), 0, DEVINFO_BLOCK_SIZE+EXTENSION_SIZE-sizeof(struct device_info));
			dp->ext_sectors = EXTENSION_SECTOR_COUNT;
			dp->devinfo_version = DEVINFO_VERSION_CURRENT;
			continue;
		}
		if (dp->devinfo_version == DEVINFO_VERSION_OLD) {
			uint32_t crcsum = dp->crcsum;
			dp->crcsum = 0;
			if (crc32(0, ctx->infobuf[i], DEVINFO_BLOCK_SIZE) != crcsum)
				continue;
			ctx->valid[i] = true;
			memset(ctx->infobuf[i] + DEVINFO_BLOCK_SIZE, 0, EXTENSION_SIZE);
			dp->ext_sectors = EXTENSION_SECTOR_COUNT;
			dp->devinfo_version = DEVINFO_VERSION_CURRENT;
			continue;
		}
		if (dp->devinfo_version >= DEVINFO_VERSION_CURRENT) {
			uint32_t crcsum;
			if (dp->ext_sectors != EXTENSION_SECTOR_COUNT) {
				fprintf(stderr, "warning: extension size mismatch\n");
				continue;
			}
			/*
			 * Read extension block
			 */
			if (lseek(ctx->fd, ctx->extension_offset[i], SEEK_END) < 0)
				continue;
			for (n = 0; n < EXTENSION_SIZE; n += cnt) {
				cnt = read(ctx->fd, &ctx->infobuf[i][DEVINFO_BLOCK_SIZE+n], EXTENSION_SIZE-n);
				if (cnt < 0)
					break;
			}
			if (n < EXTENSION_SIZE)
				continue;
			crcsum = *(uint32_t *)(&ctx->infobuf[i][DEVINFO_BLOCK_SIZE+EXTENSION_SIZE-sizeof(uint32_t)]);
			if (crc32(0, &ctx->infobuf[i][DEVINFO_BLOCK_SIZE], EXTENSION_SIZE-sizeof(uint32_t)) != crcsum)
				continue;
		} else
			continue; /* unrecognized version */
		ctx->valid[i] = true;
	}
	/*
	 * Caller can pass the CREAT flag to tell us to try
	 * initializing if no valid data is found, and add the
	 * FORCE_INIT flag to tell us to initialize even if valid
	 * data is found.
	 *
	 * Otherwise, choose the current block based on
	 * which is valid, and if both, which has the
	 * higher serial number.
	 */
	for (i = 0; i < ctx->offset_count && !ctx->valid[i]; i++);
	if (i >= ctx->offset_count) {
		if (flags & BOOTINFO_O_CREAT) {
			if (boot_devinfo_init(ctx) < 0)
				goto failure_exit;
			ctx->current = 0;
			/* If successful, fall through */
		} else {
			errno = ENODATA;
			goto failure_exit;
		}
	} else if (flags & BOOTINFO_O_FORCE_INIT) {
		if (boot_devinfo_init(ctx) < 0)
			goto failure_exit;
		ctx->current = 0;
		/* If successful, fall through */
	} else if (i < 2 && ctx->valid[1-i]) {
		/* both of the first two are valid */
		struct device_info *dp1 = (struct device_info *) (ctx->infobuf[1]);
		dp = (struct device_info *) (ctx->infobuf[0]);
		if (dp->sernum == 255 && dp1->sernum == 0)
			ctx->current = 1;
		else if (dp1->sernum == 255 && dp->sernum == 0)
			ctx->current = 0;
		else if (dp1->sernum > dp->sernum)
			ctx->current = 1;
		else
			ctx->current = 0;
	} else
		ctx->current = i;
	memcpy(&ctx->curinfo, ctx->infobuf[ctx->current], sizeof(ctx->curinfo));
	if (parse_vars(ctx) < 0) {
		/* internal error ? */
		ctx->readonly = true;
	}
	*ctxp = ctx;
	return 0;

failure_exit:
	if (ctx) {
		if (ctx->lockfd >= 0)
			close(ctx->lockfd);
		if (ctx->fd >= 0)
			close(ctx->fd);
		if (ctx->reset_bootdev_status)
			set_bootdev_writeable_status(ctx->devinfo_dev, false);
		free(ctx);
	}
	return -1;

} /* bootinfo_open */

/*
 * bootinfo_close
 *
 * Cleans up a context, freeing memory and closing open channels.
 */
int
bootinfo_close (struct bootinfo_context_s *ctx)
{
	int ret = 0;
	if (ctx == NULL)
		return ret;
	if (ctx->dirty)
		ret = update_bootinfo(ctx);
	if (ctx->lockfd >= 0)
		close(ctx->lockfd);
	if (ctx->fd >= 0)
		close(ctx->fd);
	if (ctx->reset_bootdev_status)
		set_bootdev_writeable_status(ctx->devinfo_dev, false);
	ctx->fd = -1;
	ctx->lockfd = -1;
	free_vars(ctx);
	free(ctx);

	return ret;

} /* bootinfo_close */


/*
 * bootinfo_mark_boot_success
 *
 * Clears the boot-in-progress flag and resets the failed boot
 * count to zero, optionally returning what the count was.
 */
int
bootinfo_mark_boot_success (bootinfo_context_t *ctx, unsigned int *failcount)
{
	if (!ctx) {
		errno = EINVAL;
		return -1;
	}
	if (ctx->readonly) {
		errno = EROFS;
		return -1;
	}
	ctx->curinfo.flags &= ~FLAG_BOOT_IN_PROGRESS;
	if (failcount)
		*failcount = ctx->curinfo.failed_boots;
	ctx->curinfo.failed_boots = 0;
	ctx->dirty = true;
	return 0;

} /* bootinfo_mark_boot_success */

/*
 * bootinfo_check_boot_status
 *
 * Increments the failed boot count if the boot-in-progress flag is
 * already set; otherwise, just sets the flag to indicate the start
 * of the boot sequence, zeroing the failed boot count.
 *
 * Optionally returns the number of boot failures.
 */
int
bootinfo_check_boot_status (bootinfo_context_t *ctx, unsigned int *failcount)
{

	if (!ctx) {
		errno = EINVAL;
		return -1;
	}
	if (ctx->readonly) {
		errno = EROFS;
		return -1;
	}

	if (ctx->curinfo.flags & FLAG_BOOT_IN_PROGRESS) {
		ctx->curinfo.failed_boots += 1;
	} else {
		ctx->curinfo.flags |= FLAG_BOOT_IN_PROGRESS;
		ctx->curinfo.failed_boots = 0;
	}
	ctx->dirty = true;
	if (failcount)
		*failcount = ctx->curinfo.failed_boots;
	return 0;

} /* boot_check_status */

/*
 * bootinfo_get_info
 *
 * Retrieves information from the bootinfo block.
 */
int
bootinfo_get_info (bootinfo_context_t *ctx, unsigned int *version, bool *boot_in_progress,
		   unsigned int *failcount, unsigned int *ext_sector_count)
{
	if (!ctx) {
		errno = EINVAL;
		return -1;
	}
	if (version)
		*version = ctx->curinfo.devinfo_version;
	if (boot_in_progress)
		*boot_in_progress = (ctx->curinfo.flags & FLAG_BOOT_IN_PROGRESS) != 0;
	if (failcount)
		*failcount = ctx->curinfo.failed_boots;
	if (ext_sector_count)
		*ext_sector_count = ctx->curinfo.ext_sectors;

	return 0;

} /* bootinfo_get_info */

/*
 * bootinfo_var_iter_begin
 *
 * Initializes an iterator for enumerating all of
 * the boot variables.
 */
int
bootinfo_var_iter_begin (bootinfo_context_t *ctx,
			 bootinfo_var_iter_context_t **iterctx)
{
	bootinfo_var_iter_context_t *victx;

	if (!ctx || !iterctx) {
		errno = EINVAL;
		return -1;
	}

	victx = calloc(1, sizeof(*victx));
	if (!victx)
		return -1;
	victx->ctx = ctx;
	*iterctx = victx;

	return 0;

} /* bootinfo_var_iter_begin */

/*
 * bootinfo_var_iter_next
 *
 * Retrieves the next boot variable. namebuf must be provided and
 * have a size > 0; valuebuf is optional.
 *
 * Returns:
 *    < 0: error
 *	0: success
 *    > 0: end of list
 */
int
bootinfo_var_iter_next (bootinfo_var_iter_context_t *victx,
			char *namebuf, size_t namebuf_size,
			char *valuebuf, size_t valuebuf_size)
{
	struct info_var *var;
	size_t n;

	if (!victx|| !namebuf) {
		errno = EINVAL;
		return -1;
	}

	if (victx->last == NULL)
		var = victx->ctx->vars;
	else
		var = victx->last->next;
	if (!var)
		return 1; // end of list reached

	n = strlen(var->name);
	if (n >= namebuf_size)
		n = namebuf_size - 1;
	memcpy(namebuf, var->name, n);
	namebuf[n] = '\0';
	if (valuebuf) {
		n = strlen(var->value);
		if (n >= valuebuf_size)
			n = valuebuf_size - 1;
		memcpy(valuebuf, var->value, n);
		valuebuf[n] = '\0';
	}
	victx->last = var;
	return 0;

} /* bootinfo_var_iter_next */

/*
 * bootinfo_var_iter_end
 *
 * Cleans up a boot variable iteration context.
 */
void
bootinfo_var_iter_end (bootinfo_var_iter_context_t *victx)
{
	if (victx) {
		memset(victx, 0, sizeof(*victx));
		free(victx);
	}

} /* bootinfo_var_iter_end */

/*
 * bootinfo_var_get
 *
 * Retrieves the value of a boot variable by name.
 *
 * Returns -1 on error.	 If errno is ENOENT, the name
 * was not found.
 */
int
bootinfo_var_get (bootinfo_context_t *ctx, const char *name,
		  char *valuebuf, size_t valuebuf_size)
{
	struct info_var *var;

	if (!ctx || !name || !valuebuf) {
		errno = EINVAL;
		return -1;
	}

	for (var = ctx->vars; var != NULL; var = var->next) {
		if (strcmp(name, var->name) == 0) {
			size_t n = strlen(var->value);
			if (n >= valuebuf_size)
				n = valuebuf_size - 1;
			memcpy(valuebuf, var->value, n);
			valuebuf[n] = '\0';
			return 0;
		}
	}
	errno = ENOENT;
	return -1;

} /* bootinfo_var_get */

/*
 * bootinfo_var_set
 *
 * Sets or deletes (if value ptr is null or length of value is zero) a
 * boot variable.
 */
int
bootinfo_var_set (bootinfo_context_t *ctx, const char *name, const char *value) {
	struct info_var *var, *prev;
	const char *cp;
	size_t namelen, vallen;

	/*
	 * Variable names must begin with a letter
	 * and may contain only letters, digits, and underscores
	 */
	if (!ctx || !name || !isalpha(*name)) {
		errno = EINVAL;
		return -1;
	}
	for (cp = name + 1; *cp != '\0'; cp++) {
		if (!(*cp == '_' || isalnum(*cp))) {
			errno = EINVAL;
			return -1;
		}
	}
	namelen = cp - name;
	if (namelen >= MAX_NAME_SIZE) {
		errno = EINVAL;
		return -1;
	}

	/*
	 * Let zero-length value also indicate deletion
	 */
	if (value != NULL && *value == '\0')
		value = NULL;
	/*
	 * Values may only contain printable characters
	 */
	if (value != NULL) {
		for (cp = value; *cp != '\0'; cp++) {
			if (!isprint(*cp)) {
				errno = EINVAL;
				return -1;
			}
		}
		vallen = cp - value;
		if (vallen >= MAX_VALUE_SIZE ||
		    ctx->varsize + namelen + vallen + 2 > MAX_VALUE_SIZE) {
			errno = ENOSPC;
			return -1;
		}
	}

	for (var = ctx->vars, prev = NULL; var != NULL && strcmp(name, var->name) != 0; prev = var, var = var->next);
	if (var == NULL) {
		/*
		 * If deleting, indicate name not found
		 */
		if (value == NULL) {
			errno = ENOENT;
			return -1;
		}
		var = calloc(1, sizeof(struct info_var));
		if (var == NULL)
			return -1;
		var->name = strdup(name);
		var->value = strdup(value);
		var->free_name = var->free_value = true;
		/* Add to end of list */
		if (ctx->vars == NULL)
			ctx->vars = var;
		else {
			for (prev = ctx->vars; prev->next != NULL; prev = prev->next);
			prev->next = var;
		}
	} else if (value == NULL) {
		/* Deleting found variable */
		if (prev == NULL)
			ctx->vars = var->next;
		else
			prev->next = var->next;
		free_one_var(var);
	} else {
		/* Changing value of found variable */
		var->value = strdup(value);
		var->free_value = true;
	}

	ctx->dirty = true;
	return 0;

} /* bootinfo_var_set */
