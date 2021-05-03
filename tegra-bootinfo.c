/*
 * tegra-bootinfo.c
 *
 * Display and/or modify the boot status info for
 * the operating system, used as a supplement for
 * handling boot slot failovers that cannot be
 * covered through NVIDIA's redundant boot system.
 *
 * This program should be run *after* informing
 * the NVIDIA bootloader of a successful boot
 * uing nvbootctrl.  (Do not use nv_update_engine --verify,
 * since that will overwrite the previous boot chain
 * with the current one, which is not desirable.)
 *
 * Both the above and the initial invocation of this
 * program should happen either in the initrd/initramfs
 * or very early in the main system startup.
 *
 * Initial invocation should be with the --check-status
 * option to check if the boot-in-progress flag was left
 * set when we booted.  If so, that means the last boot did
 * not fully complete, so we increment the failed
 * boots counter. If we increment to or past the
 * maximum, we exit with a return code of 77 to
 * tell the calling process that it should use
 * the nvbootctrl commands to switch to the other
 * boot slot.
 *
 * At the very end of system startup, this program
 * should be invoked again, with the --boot-success
 * option to signal that we successfully completed
 * the full boot sequence.
 *
 * In addition to tracking boot status, this program
 * maintains some persistent storage for named
 * variables, similar to U-Boot's environment variable
 * store.
 *
 * WARNING: if you use this program in conjunction with
 * U-Boot, note that by default, U-Boot's environment
 * variable space on the Jetson platforms conflicts
 * with the space allocation here (as well as the
 * pseudo-GPT used by NVIDIA).  You must customize
 * your U-Boot build and this tool's build to ensure
 * there is no conflict.
 *
 * Copyright (c) 2019-2020, Matthew Madison
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <zlib.h>
#include <tegra-eeprom/cvm.h>
#include "smd.h"
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

#define MAX_BOOT_FAILURES 3
#define DEVINFO_BLOCK_SIZE 512
#define EXTENSION_SIZE (EXTENSION_SECTOR_COUNT*512)

struct device_info {
	unsigned char magic[DEVICE_MAGIC_SIZE];
	uint16_t devinfo_version;
	uint8_t  flags;
	uint8_t  failed_boots;
	uint32_t crcsum;
	uint8_t  sernum;
	uint8_t  unused__;
	uint16_t ext_sectors;
} __attribute__((packed));
#define FLAG_BOOT_IN_PROGRESS	(1<<0)
#define DEVINFO_HDR_SIZE sizeof(struct device_info)
#define VARSPACE_SIZE (DEVINFO_BLOCK_SIZE+EXTENSION_SIZE-(DEVINFO_HDR_SIZE+sizeof(uint32_t)))
/*
 * Maximum size for a variable value is all of the variable space minus two bytes
 * for null terminators (for name and value) and one byte for a name, plus one
 * byte for the null character terminating the variable list.
 */
#define MAX_VALUE_SIZE (VARSPACE_SIZE-4)

struct info_var {
	struct info_var *next;
	char *name;
	char *value;
};


struct devinfo_context {
	int fd;
	int lockfd;
	int readonly;
	int valid[2];
	int current;
	struct device_info curinfo;
	struct info_var *vars;
	size_t varsize;
	uint8_t infobuf[2][DEVINFO_BLOCK_SIZE+EXTENSION_SIZE];
	/* storage for setting variables */
	char namebuf[DEVINFO_BLOCK_SIZE];
	char valuebuf[MAX_VALUE_SIZE];
};

/*
 * We stash the info at the end of mmcblk0boot1.
 * On tegra186/194 platforms, it goes just
 * in front of the pseudo-GPT that NVIDIA puts at the
 * very end. On tegra210 platforms, it goes at the
 * very end. Two copies are stored, with one 512 byte
 * sector each for the base, plus extended storage
 * for variables of one or more sectors (configurable
 * at build time). On tegra186/194 platforms, the two
 * copies are located together, while on tegra210 platforms,
 * the second copy is located between VER_b and VER.
 *
 * We look for these using SEEK_END, so the offsets
 * must be negative. The pseudo-GPT occupies the
 * last 34 sectors of the partition, and we leave two
 * additional sectors as buffer (on t186/t194).
 *
 * Note that 4 bytes are reserved at the end of
 * each of the extended storage blocks for a CRC
 * checksum for the block, which is maintained
 * separately from the CRC checksum for the base
 * block for compatibility with older versions of
 * this tool that did not support the extensions.
 *
 * Example layout on t186/t194 with 256KiB (1 base sector
 * plus 511 extension sectors) per storage copy:
 *
 *  sector                                          offset (hex)
 *  7132         +---------------------------------+  37B800
 *               |       extended var store B      |
 *               +---------------------------------+
 *  7643         +---------------------------------+  3BB600
 *               |       extended var store A      |
 *               +---------------------------------+
 *  8154         =       base devinfo copy B       =  3FB400
 *  8155         =       base devinfo copy A       =  3FB600
 *               ~---------- buffer ---------------~  3FB800
 *  8159         +---------------------------------+
 *               |           pseudo-GPT            |
 *  8192         +---------------------------------+
 *
 * Example layout on t210 (eMMC) with 8Kib (1 base sector
 * plus 15 extension sectors) per storage copy:
 *
 *  sector                                          offset (hex)
 *  7936         +---------------------------------+  3E0000
 *               |          VER_b                  |
 *               +---------------------------------+
 *  8047         +---------------------------------+  3EDE00
 *               |       extended var store B      |
 *               +---------------------------------+
 *  8063         =       base devinfo copy B       =  3EFE00
 *  8064         +---------------------------------+  3F0000
 *               |          VER                    |
 *               +---------------------------------+
 *  8175         +---------------------------------+  3FDE00
 *               |       extended var store A      |
 *               +---------------------------------+
 *  8191         =       base devinfo copy A       =  3FFE00
 *
 */
#define OFFSET_COUNT 2
static const off_t devinfo_offset_non_t21x[OFFSET_COUNT] = {
	[0] = -((36 + 1) * 512),
	[1] = -((36 + 2) * 512),
};

static const off_t extension_offset_non_t21x[OFFSET_COUNT] = {
	[0] = -((EXTENSION_SECTOR_COUNT + 36 + 2) * 512),
	[1] = -((EXTENSION_SECTOR_COUNT * 2 + 36 + 2) * 512),
};

static const off_t devinfo_offset_t21x[OFFSET_COUNT] = {
	[0] = -512,
	[1] = -(65536 + 512),
};

static const off_t extension_offset_t21x[OFFSET_COUNT] = {
	[0] = -((EXTENSION_SECTOR_COUNT + 2) * 512),
	[1] = -(65536 + (EXTENSION_SECTOR_COUNT + 2) * 512),
};

static const off_t *devinfo_offset;
static const off_t *extension_offset;

static const char *devinfo_dev;
static const char bootdev[] = OTABOOTDEV;
static const char gptdev[] = OTAGPTDEV;
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

static struct option options[] = {
	{ "boot-success",	no_argument,		0, 'b' },
	{ "check-status",	no_argument,		0, 'c' },
	{ "initialize",		no_argument,		0, 'I' },
	{ "show",		no_argument,		0, 's' },
	{ "omit-name",		no_argument,		0, 'n' },
	{ "from-file",		required_argument,	0, 'f' },
	{ "force-initialize",	no_argument,		0, 'F' },
	{ "get-variable",	no_argument,		0, 'v' },
	{ "set-variable",	no_argument,		0, 'V' },
	{ "help",		no_argument,		0, 'h' },
	{ "version",		no_argument,		0, 0   },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":bcIsnf:FvVh";

static char *optarghelp[] = {
	"--boot-success       ",
	"--check-status       ",
	"--initialize         ",
	"--show               ",
	"--omit-name          ",
	"--from-file FILE     ",
	"--force-initialize   ",
	"--get-variable       ",
	"--set-variable       ",
	"--help               ",
	"--version            ",
};

static char *opthelp[] = {
	"update boot info to record successful boot",
	"increment boot counter and check it is under limit",
	"initialize the device info area",
	"show boot counter information",
	"omit variable name in output (for use with --get-variable)",
	"take variable value from FILE (for use with --set-variable)",
	"force initialization even if bootinfo already initialized (for use with --initialize)",
	"get the value of a stored variable by name, list all if no name specified",
	"set the value of a stored variable (delete if no value)",
	"display this help text",
	"display version information"
};

/*
 * identify_chip
 *
 * Look up the tegra chip ID and adjust offsets based
 * on whether it's a t210 or not.
 *
 * Returns: 0 on success, -1 on error (errno not set)
 */
static int
identify_chip (void)
{
	char buf[32];
	ssize_t n;
	unsigned long chipid;
	int fd = open("/sys/module/tegra_fuse/parameters/tegra_chip_id", O_RDONLY);

	if (fd < 0)
		return -1;
	n = read(fd, buf, sizeof(buf));
	close(fd);
	if (n <= 0)
		return -1;
	chipid = strtoul(buf, NULL, 0);
	if (chipid != 0x21 && chipid != 0x18 && chipid != 0x19)
		return -1;
	if (chipid == 0x21) {
		devinfo_offset = devinfo_offset_t21x;
		extension_offset = extension_offset_t21x;
	} else {
		devinfo_offset = devinfo_offset_non_t21x;
		extension_offset = extension_offset_non_t21x;
	}
	return 0;

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
find_storage_dev (void)
{
	unsigned int i;

	for (i = 0; i < sizeof(devinfo_devices)/sizeof(devinfo_devices[0]); i++) {
		if (access(devinfo_devices[i], F_OK) == 0) {
			devinfo_dev = devinfo_devices[i];
			return 0;
		}
	}
	devinfo_dev = NULL;
	return -1;

} /* find_storage_dev */

/*
 * parse_vars
 *
 * A variable consists of a null-terminated name followed
 * by a null-terminated value.  Names and values are simply
 * concatenated into the space after the header, up to the
 * block size. A null byte at the beginning of a variable
 * name indicates the end of the list.
 *
 * It's possible to have a null value, but in this implementation
 * null-valued variables are not written to the info block;
 * setting a value to the null string deletes the variable.
 */
static int
parse_vars (struct devinfo_context *ctx)
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
pack_vars (struct devinfo_context *ctx, int idx)
{
	struct info_var *var;
	char *cp;
	size_t remain, nlen, vlen;

	if (idx != 0 && idx != 1)
		return -1;
	if (ctx->vars == NULL)
		return 0;
	for (var = ctx->vars, cp = (char *)(ctx->infobuf[idx] + DEVINFO_HDR_SIZE),
		     remain = sizeof(ctx->infobuf[ctx->current]) - (DEVINFO_HDR_SIZE+sizeof(uint32_t)+1);
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
 * free_vars
 *
 * Frees the memory for variable tracking.
 */
static void
free_vars (struct devinfo_context *ctx)
{
	struct info_var *var, *vnext;
	for (var = ctx->vars; var != NULL; var = vnext) {
		vnext = var->next;
		free(var);
	}
	ctx->vars = NULL;
	ctx->varsize = 0;

} /* free_vars */

/*
 * find_bootinfo
 *
 * Tries to find a valid bootinfo block, and initializes a context
 * if one is found.
 *
 * Returns negative value on an underlying error or if neither block
 * is valid.
 *
 * ctxp is set to NULL if an internal error occurred, otherwise it is
 * set to point to a valid context, and (*ctxp)->fd is the fd of
 * the open channel to the MMC boot1 device. Device is opened readonly
 * and (*ctxp)->readonly is set to 1 if the readonly arg is non-zero;
 * otherwise, (*ctxp)->readonly is set to 1 if a valid block is found
 * but an internal error occurred parsing the variables stored in the block.
 */
static int
find_bootinfo (int readonly, struct devinfo_context **ctxp)
{
	struct devinfo_context *ctx;
	struct device_info *dp;
	ssize_t n, cnt;
	int i, dirfd;

	*ctxp = NULL;
	ctx = calloc(1, sizeof(struct devinfo_context));
	if (ctx == NULL) {
		perror("calloc");
		return -1;
	}
	ctx->readonly = readonly;

	ctx->fd = open(devinfo_dev, (readonly ? O_RDONLY : O_RDWR|O_DSYNC));
	if (ctx->fd < 0) {
		perror(devinfo_dev);
		free(ctx);
		return -1;
	}
	dirfd = open("/run/tegra-bootinfo", O_PATH);
	if (dirfd < 0) {
		if (mkdir("/run/tegra-bootinfo", 02770) < 0) {
			perror("lockfile directory");
			close(ctx->fd);
			free(ctx);
			return -1;
		}
		dirfd = open("/run/tegra-bootinfo", O_PATH);
	}
	ctx->lockfd = openat(dirfd, "lockfile", O_CREAT|O_RDWR, 0770);
	if (ctx->lockfd < 0) {
		perror("lockfile");
		close(dirfd);
		close(ctx->fd);
		free(ctx);
		return -1;
	}
	close(dirfd);
	if (flock(ctx->lockfd, (readonly ? LOCK_SH : LOCK_EX)) < 0) {
		perror("flock");
		close(ctx->lockfd);
		close(ctx->fd);
		free(ctx);
		return -1;
	}
	for (i = 0; i < OFFSET_COUNT; i++) {
		/*
		 * Read base block
		 */
		if (lseek(ctx->fd, devinfo_offset[i], SEEK_END) < 0)
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
			ctx->valid[i] = 1;
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
			ctx->valid[i] = 1;
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
			if (lseek(ctx->fd, extension_offset[i], SEEK_END) < 0)
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
		ctx->valid[i] = 1;
	}
	*ctxp = ctx;
	if (!(ctx->valid[0] || ctx->valid[1])) {
		ctx->current = -1;
		memset(&ctx->curinfo, 0, sizeof(ctx->curinfo));
		return -1;
	} else if (ctx->valid[0] && !ctx->valid[1])
		ctx->current = 0;
	else if (!ctx->valid[0] && ctx->valid[1])
		ctx->current = 1;
	else {
		/* both valid */
		struct device_info *dp1 = (struct device_info *)(ctx->infobuf[1]);
		dp = (struct device_info *)(ctx->infobuf[0]);
		if (dp->sernum == 255 && dp1->sernum == 0)
			ctx->current = 1;
		else if (dp1->sernum == 255 && dp->sernum == 0)
			ctx->current = 0;
		else if (dp1->sernum > dp->sernum)
			ctx->current = 1;
		else
			ctx->current = 0;
	}
	memcpy(&ctx->curinfo, ctx->infobuf[ctx->current], sizeof(ctx->curinfo));
	if (parse_vars(ctx) < 0) {
		/* internal error ? */
		ctx->readonly = 1;
	}
	return 0;

} /* find_bootinfo */

/*
 * update_bootinfo
 *
 * Write out a device info block based on the current context.
 */
static int
update_bootinfo (struct devinfo_context *ctx)
{
	uint32_t *crcptr;
	struct device_info *info;
	ssize_t n, cnt;
	int idx;

	if (ctx == NULL) {
		fprintf(stderr, "error: invalid context provided for update\n");
		return -1;
	}
	if (ctx->readonly) {
		fprintf(stderr, "error: update requested for read-only context\n");
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
	if (pack_vars(ctx, idx) < 0)
		return -1;
	info->crcsum = crc32(0, ctx->infobuf[idx], DEVINFO_BLOCK_SIZE);
	*crcptr = crc32(0, &ctx->infobuf[idx][DEVINFO_BLOCK_SIZE], EXTENSION_SIZE-sizeof(uint32_t));

	if (lseek(ctx->fd, devinfo_offset[idx], SEEK_END) < 0) {
		perror(devinfo_dev);
		set_bootdev_writeable_status(devinfo_dev, false);
		return -1;
	}
	for (n = 0; n < DEVINFO_BLOCK_SIZE; n += cnt) {
		cnt = write(ctx->fd, ctx->infobuf[idx] + n, DEVINFO_BLOCK_SIZE-n);
		if (cnt < 0) {
			perror(devinfo_dev);
			set_bootdev_writeable_status(devinfo_dev, false);
			return -1;
		}
	}
	if (lseek(ctx->fd, extension_offset[idx], SEEK_END) < 0) {
		perror(devinfo_dev);
		set_bootdev_writeable_status(devinfo_dev, false);
		return -1;
	}
	for (n = 0; n < EXTENSION_SIZE; n += cnt) {
		cnt = write(ctx->fd, ctx->infobuf[idx] + DEVINFO_BLOCK_SIZE + n, EXTENSION_SIZE-n);
		if (cnt < 0) {
			perror(devinfo_dev);
			set_bootdev_writeable_status(devinfo_dev, false);
			return -1;
		}
	}

	return 0;

} /* update_bootinfo */

/*
 * close_bootinfo
 *
 * Cleans up a context, freeing memory and closing open channels.
 */
static int
close_bootinfo (struct devinfo_context *ctx, int keeplock)
{
	int lockfd = -1;

	if (ctx == NULL)
		return lockfd;
	if (keeplock)
		lockfd = ctx->lockfd;
	else if (ctx->lockfd >= 0)
		close(ctx->lockfd);
	if (ctx->fd >= 0)
		close(ctx->fd);
	ctx->fd = -1;
	ctx->lockfd = -1;
	free_vars(ctx);
	free(ctx);

	return lockfd;

} /* close_bootinfo */

/*
 * print_usage
 */
static void
print_usage (void)
{
	int i;
	printf("\nUsage:\n");
	printf("\ttegra-bootinfo\n");
	printf("Options:\n");
	for (i = 0; i < sizeof(options)/sizeof(options[0]) && options[i].name != 0; i++) {
		printf(" %s\t%c%c\t%s\n",
		       optarghelp[i],
		       (options[i].val == 0 ? ' ' : '-'),
		       (options[i].val == 0 ? ' ' : options[i].val),
		       opthelp[i]);
	}

} /* print_usage */

static int
boot_devinfo_init(int force_init)
{
	int fd, i, lockfd;
	ssize_t n, cnt;
	struct devinfo_context *ctx;
	static uint8_t buf[DEVINFO_BLOCK_SIZE + EXTENSION_SIZE];

	if (find_bootinfo(0, &ctx) == 0 && ctx != NULL) {
		if (!ctx->readonly && !force_init) {
			fprintf(stderr, "Device info already initialized\n");
			close_bootinfo(ctx, 0);
			return 0;
		}
	}
	if (ctx != NULL)
		lockfd = close_bootinfo(ctx, 1);
	else
		lockfd = -1;

	fd = open(devinfo_dev, O_RDWR|O_DSYNC);
	if (fd < 0) {
		perror(devinfo_dev);
		set_bootdev_writeable_status(devinfo_dev, false);
		close(lockfd);
		return -1;
	}
	for (i = 0; i < 2; i++) {
		if (lseek(fd, devinfo_offset[i], SEEK_END) < 0)
			break;
		for (n = 0; n < DEVINFO_BLOCK_SIZE; n += cnt) {
			cnt = write(fd, buf+n, DEVINFO_BLOCK_SIZE-n);
			if (cnt < 0)
				break;
		}
		if (n < DEVINFO_BLOCK_SIZE)
			break;
		if (lseek(fd, extension_offset[i], SEEK_END) < 0)
			break;
		for (n = 0; n < EXTENSION_SIZE; n += cnt) {
			cnt = write(fd, buf+DEVINFO_BLOCK_SIZE+n, EXTENSION_SIZE-n);
			if (cnt < 0)
				break;
		}
		if (n < EXTENSION_SIZE)
			break;
	}
	if (i < 2) {
		fprintf(stderr, "could not initialize bootinfo areas\n");
		close(fd);
		close(lockfd);
		return -1;
	}

	ctx = calloc(1, sizeof(struct devinfo_context));
	if (ctx == NULL) {
		perror("calloc");
		close(fd);
		close(lockfd);
		return -1;
	}
	ctx->fd = fd;
	ctx->lockfd = lockfd;
	ctx->current = -1;
	if (update_bootinfo(ctx) < 0)
		return -1;
	close_bootinfo(ctx, 0);
	return 0;

} /* boot_devinfo_init */

static int
mark_nv_boot_successful (void)
{
	tegra_soctype_t soctype = cvm_soctype();
	gpt_context_t *gptctx;
	smd_context_t *smdctx = NULL;
	int curslot, fd;
	bool reset_bootdev;
	int ret = -1;

	if (soctype != TEGRA_SOCTYPE_186 && soctype != TEGRA_SOCTYPE_194)
		return 0;

	curslot = smd_get_current_slot();
	if (curslot < 0) {
		perror("smd_get_current_slot");
		return ret;
	}
	gptctx = gpt_init(gptdev, 512, 0);
	if (gptctx == NULL) {
		perror("gpt_init");
		return ret;
	}
	if (gpt_load(gptctx, GPT_BACKUP_ONLY|GPT_NVIDIA_SPECIAL)) {
		perror("gpt_load");
		gpt_finish(gptctx);
		return ret;
	}
	reset_bootdev = set_bootdev_writeable_status(bootdev, true);
	fd = open(bootdev, O_RDWR);
	if (fd < 0) {
		perror(bootdev);
		goto reset_and_depart;
	}
	smdctx = smd_init(gptctx, fd);
	if (smdctx == NULL) {
		perror("smd_init");
		goto reset_and_depart;
	}
	if (smd_slot_mark_successful(smdctx, curslot) == 0 && smd_update(smdctx, gptctx, fd, false) == 0) {
		ret = 0;
	} else
		perror("marking boot slot successful");

reset_and_depart:
	if (smdctx != NULL)
		smd_finish(smdctx);
	fsync(fd);
	close(fd);
	if (reset_bootdev)
		set_bootdev_writeable_status(bootdev, false);
	gpt_finish(gptctx);
	return ret;

} /* mark_nv_boot_successful */

static int
boot_successful(void)
{
	struct devinfo_context *ctx = NULL;

	if (mark_nv_boot_successful() < 0)
		return 1;

	set_bootdev_writeable_status(devinfo_dev, true);
	if (find_bootinfo(0, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info, initializing\n");
		close_bootinfo(ctx, 0);
		ctx = NULL;
		if (boot_devinfo_init(1) < 0) {
			set_bootdev_writeable_status(devinfo_dev, false);
			return -2;
		}
		if (find_bootinfo(0, &ctx) < 0) {
			close_bootinfo(ctx, 0);
			set_bootdev_writeable_status(devinfo_dev, false);
			return -2;
		}
	}

	ctx->curinfo.flags &= ~FLAG_BOOT_IN_PROGRESS;
	if (ctx->curinfo.failed_boots > 0)
		fprintf(stderr, "Failed boot count: %u\n", ctx->curinfo.failed_boots);
	ctx->curinfo.failed_boots = 0;
	if (update_bootinfo(ctx) < 0) {
		perror("writing boot info");
		close_bootinfo(ctx, 0);
		set_bootdev_writeable_status(devinfo_dev, false);
		return -2;
	}
	close_bootinfo(ctx, 0);
	set_bootdev_writeable_status(devinfo_dev, false);
	return 0;

} /* boot_successful */

static int
boot_check_status(void)
{
	struct devinfo_context *ctx = NULL;
	int rc = 0;

	if (mark_nv_boot_successful() < 0)
		return 1;
	set_bootdev_writeable_status(devinfo_dev, true);
	if (find_bootinfo(0, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info, initializing\n");
		close_bootinfo(ctx, 0);
		ctx = NULL;
		rc = boot_devinfo_init(1);
		if (rc == 0)
			rc = find_bootinfo(0, &ctx);
		if (rc < 0) {
			close_bootinfo(ctx, 0);
			set_bootdev_writeable_status(devinfo_dev, false);
			return rc;
		}
	}

	if (ctx->curinfo.flags & FLAG_BOOT_IN_PROGRESS) {
		ctx->curinfo.failed_boots += 1;
		fprintf(stderr, "Boot failures: %u\n", ctx->curinfo.failed_boots);
		if (ctx->curinfo.failed_boots >= MAX_BOOT_FAILURES) {
			fprintf(stderr, "Too many boot failures, exit with error to signal boot slot switch\n");
			rc = 77;
			ctx->curinfo.flags &= ~FLAG_BOOT_IN_PROGRESS;
			ctx->curinfo.failed_boots = 0;
		}
	} else
		ctx->curinfo.flags |= FLAG_BOOT_IN_PROGRESS;
	if (update_bootinfo(ctx) < 0) {
		perror("writing boot info");
		rc = -2;
	}
	close_bootinfo(ctx, 0);
	set_bootdev_writeable_status(devinfo_dev, false);
	return rc;

} /* boot_check_status */

/*
 * show_bootinfo
 *
 * Prints out the boot info header information.
 */
static int
show_bootinfo(void) {

	struct devinfo_context *ctx;

	if (find_bootinfo(1, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info\n");
		return -2;
	}
	printf("devinfo version:        %u\n"
	       "Boot in progress:       %s\n"
	       "Failed boots:           %d\n"
	       "Extension space:        %d sector%s\n",
	       ctx->curinfo.devinfo_version,
	       (ctx->curinfo.flags & FLAG_BOOT_IN_PROGRESS) ? "YES" : "NO",
	       ctx->curinfo.failed_boots,
	       ctx->curinfo.ext_sectors,
	       (ctx->curinfo.ext_sectors == 1 ? "" : "s"));
	close_bootinfo(ctx, 0);
	return 0;

} /* show_bootinfo */

/*
 * show_bootvar
 *
 * Prints out the value of a variable, or
 * all var=value settings if varname == NULL
 */
int
show_bootvar (const char *name, int omitname)
{
	struct devinfo_context *ctx;
	struct info_var *var;
	int found = (name == NULL) ? 1 : 0;

	if (find_bootinfo(1, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info\n");
		return -2;
	}
	for (var = ctx->vars; var != NULL; var = var->next) {
		if (name == NULL || strcmp(name, var->name) == 0) {
			found = 1;
			if (omitname)
				printf("%s\n", var->value);
			else
				printf("%s=%s\n", var->name, var->value);
			if (name != NULL)
				break;
		}
	}
	close_bootinfo(ctx, 0);
	if (!found) {
		fprintf(stderr, "not found: %s\n", name);
		return 1;
	}
	return 0;

} /* show_bootvar */

/*
 * set_bootvar
 *
 * Sets or deletes a variable.
 */
int
set_bootvar (const char *name, const char *value, char *inputfile)
{
	struct devinfo_context *ctx;
	struct info_var *var, *prev;
	static char valuebuf[MAX_VALUE_SIZE];

	if (inputfile != NULL) {
		FILE *fp;
		ssize_t n, cnt;

		if ((value != NULL) || strchr(name, '=') != NULL) {
			fprintf(stderr, "cannot specify both value and input file\n");
			return 1;
		}
		if (strcmp(inputfile, "-") == 0)
			fp = stdin;
		else {
			fp = fopen(inputfile, "r");
			if (fp == NULL) {
				perror(inputfile);
				return 1;
			}
		}
		for (n = 0; n < sizeof(valuebuf); n += cnt) {
			cnt = fread(valuebuf + n, sizeof(char), sizeof(valuebuf)-n, fp);
			if (cnt < sizeof(valuebuf)-n) {
				if (feof(fp)) {
					n += cnt;
					break;
				}
				fprintf(stderr, "error reading %s\n",
					(fp == stdin ? "input" : inputfile));
				if (fp != stdin)
					fclose(fp);
				return 1;
			}
		}
		if (fp != stdin)
			fclose(fp);
		if (n >= sizeof(valuebuf)-1) {
			fprintf(stderr, "input value too large\n");
			return 1;
		}
		valuebuf[n] = '\0';
		if (strlen(valuebuf) != n) {
			fprintf(stderr, "null character in input value not allowed\n");
			return 1;
		}
		value = valuebuf;
	}

	/*
	 * Allow 'name=value' as a single argument
	 * Or 'name=' to unset a variable
	 */
	if (value == NULL) {
		char *cp = strchr(name, '=');
		if (cp != NULL) {
			if (cp == name) {
				fprintf(stderr, "invalid variable name\n");
				return 1;
			}
			*cp = '\0';
			value = cp + 1;
		}
	}
	/*
	 * Now that the input/parsing of names and values is complete,
	 * check for a null (0-length) value and just set value to NULL
	 * to indicate that we want to delete the variable in that
	 * case.
	 */
	if (value != NULL && *value == '\0')
		value = NULL;
	/*
	 * Variable names must begin with a letter
	 * and can contain letters, digits, or underscores.
	 */
	if (!isalpha(*name)) {
		fprintf(stderr, "invalid variable name\n");
		return 1;
	} else {
		const char *cp;
		for (cp = name + 1; *cp != '\0'; cp++) {
			if (!(*cp == '_' || isalnum(*cp))) {
				fprintf(stderr, "invalid variable name\n");
				return 1;
			}
		}
	}
	/*
	 * Values may only contain printable characters
	 */
	if (value != NULL) {
		const char *cp;
		for (cp = value; *cp != '\0'; cp++) {
			if (!isprint(*cp)) {
				fprintf(stderr, "invalid value for variable\n");
				return 1;
			}
		}
	}
	set_bootdev_writeable_status(devinfo_dev, true);
	if (find_bootinfo(0, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info\n");
		set_bootdev_writeable_status(devinfo_dev, false);
		return -2;
	}

	if (strlen(name) >= sizeof(ctx->namebuf)) {
		fprintf(stderr, "error: variable name too long\n");
		close_bootinfo(ctx, 0);
		set_bootdev_writeable_status(devinfo_dev, false);
		return 1;
	}

	strcpy(ctx->namebuf, name);
	if (value != NULL) {
		size_t vallen = strlen(value);
		size_t s = strlen(name) + vallen + 2;
		if (vallen >= sizeof(ctx->valuebuf) ||
		    ctx->varsize + s > sizeof(ctx->valuebuf)) {
			fprintf(stderr, "error: insufficient space for variable storage\n");
			close_bootinfo(ctx, 0);
			set_bootdev_writeable_status(devinfo_dev, false);
			return 1;
		}
		strcpy(ctx->valuebuf, value);
	}
	for (var = ctx->vars, prev = NULL; var != NULL && strcmp(name, var->name) != 0; prev = var, var = var->next);
	if (var == NULL) {
		if (value == NULL) {
			fprintf(stderr, "not found: %s\n", name);
			close_bootinfo(ctx, 0);
			return 1;
		}
		var = calloc(1, sizeof(struct info_var));
		if (var == NULL) {
			perror("calloc");
			close_bootinfo(ctx, 0);
			set_bootdev_writeable_status(devinfo_dev, false);
			return 1;
		}
		var->name = ctx->namebuf;
		var->value = ctx->valuebuf;
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
		free(var);
	} else
		/* Changing value of found variable */
		var->value = ctx->valuebuf;

	if (update_bootinfo(ctx) < 0) {
		fprintf(stderr, "could not update variables\n");
		close_bootinfo(ctx, 0);
		set_bootdev_writeable_status(devinfo_dev, false);
		return 1;
	}
	close_bootinfo(ctx, 0);
	set_bootdev_writeable_status(devinfo_dev, false);
	return 0;

} /* set_bootvar */

/*
 * main program
 */
int
main (int argc, char * const argv[])
{

	int c, which, ret;
	int omitname = 0;
	int force_init = 0;
	char *inputfile = NULL;
	enum {
		nocmd,
		success,
		check,
		show,
		showvar,
		setvar,
		init,
	} cmd = nocmd;

	if (argc < 2) {
		print_usage();
		return 1;
	}


	while ((c = getopt_long_only(argc, argv, shortopts, options, &which)) != -1) {

		switch (c) {
		case 'h':
			print_usage();
			return 0;
		case 'b':
			cmd = success;
			break;
		case 'c':
			cmd = check;
			break;
		case 'I':
			cmd = init;
			break;
		case 's':
			cmd = show;
			break;
		case 'n':
			omitname = 1;
			break;
	        case 'f':
			inputfile = strdup(optarg);
			break;
		case 'F':
			force_init = 1;
			break;
		case 'v':
		case 'V':
			if (cmd != nocmd) {
				fprintf(stderr, "Error: only one of -v/-V permitted\n");
				print_usage();
				return 1;
			}
			cmd = (c == 'v' ? showvar : setvar);
			break;
		case 0:
			if (strcmp(options[which].name, "version") == 0) {
				printf("%s\n", VERSION);
				return 0;
			}
			/* fallthrough */
		default:
			fprintf(stderr, "Error: unrecognized option\n");
			print_usage();
			return 1;
		} /* switch (c) */

	} /* while getopt */

	if (identify_chip() != 0) {
		fprintf(stderr, "Error: cannot identify SoC type\n");
		return 1;
	}

	if (find_storage_dev() != 0) {
		fprintf(stderr, "Error: cannot locate storage device\n");
		return 1;
	}

	switch (cmd) {
	case success:
		return boot_successful();
	case check:
		return boot_check_status();
	case show:
		return show_bootinfo();
	case init:
		set_bootdev_writeable_status(devinfo_dev, true);
		ret = boot_devinfo_init(force_init);
		set_bootdev_writeable_status(devinfo_dev, false);
		return ret;
	case showvar:
		if (optind >= argc)
			return show_bootvar(NULL, 0);
		return show_bootvar(argv[optind], omitname);
	case setvar:
		if (optind >= argc) {
			fprintf(stderr, "Error: missing variable name\n");
			print_usage();
			return 1;
		}
		return set_bootvar(argv[optind], (optind < argc - 1 ? argv[optind+1] : NULL), inputfile);
	default:
		break;
	}


	print_usage();
	return 1;

} /* main */
