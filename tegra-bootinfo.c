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
 * Copyright (c) 2019, Matthew Madison
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <endian.h>
#include <getopt.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <zlib.h>

static const char DEVICE_MAGIC[8] = {'B', 'O', 'O', 'T', 'I', 'N', 'F', 'O'};
#define DEVICE_MAGIC_SIZE sizeof(DEVICE_MAGIC)

static const uint16_t DEVINFO_VERSION_OLD = 1;
static const uint16_t DEVINFO_VERSION_CURRENT = 2;

#define MAX_BOOT_FAILURES 3
#define DEVINFO_BLOCK_SIZE 512

struct device_info {
	unsigned char magic[DEVICE_MAGIC_SIZE];
	uint16_t devinfo_version;
	uint8_t  flags;
	uint8_t  failed_boots;
	uint32_t crcsum;
	uint8_t  sernum;
	uint8_t  unused__[3];
} __attribute__((packed));
#define FLAG_BOOT_IN_PROGRESS	(1<<0)
#define DEVINFO_HDR_SIZE sizeof(struct device_info)

struct info_var {
	struct info_var *next;
	char *name;
	char *value;
};

struct devinfo_context {
	int fd;
	int readonly;
	int valid[2];
	int current;
	struct device_info curinfo;
	struct info_var *vars;
	size_t varsize;
	uint8_t infobuf[2][DEVINFO_BLOCK_SIZE];
	/* storage for setting variables */
	char namebuf[DEVINFO_BLOCK_SIZE];
	char valuebuf[DEVINFO_BLOCK_SIZE];
};

/*
 * We stash the info at the end of mmcblk0boot1, just
 * in front of the pseudo-GPT that NVIDIA puts at the
 * very end. Two copies, 512 bytes each.
 *
 * We look for these using SEEK_END, so the offsets
 * must be negative.
 */
static const off_t devinfo_offset[2] = {
	[0] = -((36 + 1) * 512),
	[1] = -((36 + 2) * 512),
};

static const char devinfo_dev[] = "/dev/mmcblk0boot1";

static struct option options[] = {
	{ "boot-success",	no_argument,		0, 'b' },
	{ "check-status",	no_argument,		0, 'c' },
	{ "initialize",		no_argument,		0, 'I' },
	{ "show",		no_argument,		0, 's' },
	{ "get-variable",	no_argument,		0, 'v' },
	{ "set-variable",	no_argument,		0, 'V' },
	{ "help",		no_argument,		0, 'h' },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":bcIsvVh";

static char *optarghelp[] = {
	"--boot-success       ",
	"--check-status       ",
	"--initialize         ",
	"--show               ",
	"--get-variable       ",
	"--set-variable       ",
	"--help               ",
};

static char *opthelp[] = {
	"update boot info to record successful boot",
	"increment boot counter and check it is under limit",
	"initialize the device info area",
	"show boot counter information",
	"get the value of a stored variable by name, list all if no name specified",
	"set the value of a stored variable (delete if no value)",
	"display this help text"
};

/*
 * set_bootdev_writeable_status
 *
 * Sets the readonly/readwrite flag for an mmc boot block.
 */
static int
set_bootdev_writeable_status (int make_writeable)
{
	char pathname[64];
	char buf[1];
	int fd, is_writeable;

	sprintf(pathname, "/sys/block/%s/force_ro", devinfo_dev + 5);
	fd = open(pathname, O_RDWR);
	if (fd < 0)
		return 0;
	if (read(fd, buf, sizeof(buf)) != sizeof(buf)) {
		close(fd);
		return 0;
	}
	make_writeable = !!make_writeable;
	is_writeable = buf[0] == '0';
	if (make_writeable && !is_writeable)
		if (write(fd, "0", 1) != 1)
			fprintf(stderr, "error updating force_ro for %s\n", devinfo_dev);
	else if (!make_writeable && is_writeable)
		if (write(fd, "1", 1) != 1)
			fprintf(stderr, "error updating force_ro for %s\n", devinfo_dev);
	close(fd);

	return make_writeable != is_writeable;

} /* set_bootdev_writeable_status */

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
	size_t remain, varbytes;

	ctx->vars = NULL;
	if (ctx->current < 0) {
		fprintf(stderr, "error: parse_vars called with no valid info block\n");
		return -1;
	}
	for (cp = (char *)(ctx->infobuf[ctx->current] + DEVINFO_HDR_SIZE),
		     remain = DEVINFO_BLOCK_SIZE - DEVINFO_HDR_SIZE,
		     ctx->varsize = 0,
		     last = NULL;
	     remain > 0 && *cp != '\0';
	     remain -= varbytes) {
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
		     remain = DEVINFO_BLOCK_SIZE - DEVINFO_HDR_SIZE;
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
	if (var != NULL) {
		fprintf(stderr, "error: variables list too large\n");
		return -1;
	}

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
	int i;

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
	for (i = 0; i < sizeof(devinfo_offset)/sizeof(devinfo_offset[0]); i++) {
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
		if (dp->devinfo_version == DEVINFO_VERSION_OLD) {
			uint8_t inprogress = dp->flags;
			dp->flags = (inprogress != 0 ? FLAG_BOOT_IN_PROGRESS : 0);
			ctx->valid[i] = 1;
			memset(ctx->infobuf[i] + sizeof(struct device_info), 0, DEVINFO_BLOCK_SIZE-sizeof(struct device_info));
			dp->devinfo_version = DEVINFO_VERSION_CURRENT;
			continue;
		}
		if (dp->devinfo_version >= DEVINFO_VERSION_CURRENT) {
			uint32_t crcsum = dp->crcsum;
			dp->crcsum = 0;
			if (crc32(0, ctx->infobuf[i], DEVINFO_BLOCK_SIZE) != crcsum)
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
		memcpy(&ctx->curinfo, ctx->infobuf[ctx->current], sizeof(ctx->curinfo));
	}
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
	memset(info, 0, DEVINFO_BLOCK_SIZE);
	memcpy(info->magic, DEVICE_MAGIC, sizeof(info->magic));
	info->devinfo_version = DEVINFO_VERSION_CURRENT;
	info->flags = ctx->curinfo.flags;
	info->failed_boots = ctx->curinfo.failed_boots;
	info->sernum = ctx->curinfo.sernum + 1;
	pack_vars(ctx, idx);
	info->crcsum = crc32(0, ctx->infobuf[idx], DEVINFO_BLOCK_SIZE);

	if (lseek(ctx->fd, devinfo_offset[idx], SEEK_END) < 0) {
		perror(devinfo_dev);
		set_bootdev_writeable_status(0);
		return -1;
	}
	for (n = 0; n < DEVINFO_BLOCK_SIZE; n += cnt) {
		cnt = write(ctx->fd, ctx->infobuf[idx] + n, DEVINFO_BLOCK_SIZE-n);
		if (cnt < 0) {
			perror(devinfo_dev);
			set_bootdev_writeable_status(0);
			return -1;
		}
	}

} /* update_bootinfo */

/*
 * close_bootinfo
 *
 * Cleans up a context, freeing memory and closing open channels.
 */
static void
close_bootinfo (struct devinfo_context *ctx)
{
	if (ctx == NULL)
		return;
	if (ctx->fd >= 0)
		close(ctx->fd);
	ctx->fd = -1;
	free_vars(ctx);
	free(ctx);

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
	printf("Options (use only one per invocation):\n");
	for (i = 0; i < sizeof(options)/sizeof(options[0]) && options[i].name != 0; i++) {
		printf(" %s\t%c%c\t%s\n",
		       optarghelp[i],
		       (options[i].val == 0 ? ' ' : '-'),
		       (options[i].val == 0 ? ' ' : options[i].val),
		       opthelp[i]);
	}

} /* print_usage */

static int
boot_devinfo_init(void)
{
	struct devinfo_context *ctx;
	struct info_var *var, *prev;

	set_bootdev_writeable_status(1);
	if (find_bootinfo(0, &ctx) == 0 && ctx != NULL & !ctx->readonly) {
		fprintf(stderr, "Device info already initialized\n");
		close_bootinfo(ctx);
		set_bootdev_writeable_status(0);
		return 0;
	}
	if (ctx == NULL) {
		ctx = calloc(1, sizeof(struct devinfo_context));
		if (ctx == NULL) {
			perror("calloc");
			set_bootdev_writeable_status(0);
			return -1;
		}
		ctx->fd = open(devinfo_dev, O_RDWR|O_DSYNC);
		if (ctx->fd < 0) {
			perror(devinfo_dev);
			free(ctx);
			set_bootdev_writeable_status(0);
			return -1;
		}
		ctx->current = -1;
	}
	ctx->readonly = 0;
	if (update_bootinfo(ctx) < 0)
		return -1;
	close_bootinfo(ctx);
	set_bootdev_writeable_status(0);
	return 0;

} /* boot_devinfo_init */

static int
boot_successful(void)
{
	struct devinfo_context *ctx;

	set_bootdev_writeable_status(1);
	if (find_bootinfo(0, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info, initializing\n");
		if (boot_devinfo_init() < 0) {
			close_bootinfo(ctx);
			set_bootdev_writeable_status(0);
			return -2;
		}
		if (find_bootinfo(0, &ctx) < 0) {
			close_bootinfo(ctx);
			set_bootdev_writeable_status(0);
			return -2;
		}
	}

	ctx->curinfo.flags &= ~FLAG_BOOT_IN_PROGRESS;
	if (ctx->curinfo.failed_boots > 0)
		fprintf(stderr, "Failed boot count: %u\n", ctx->curinfo.failed_boots);
	ctx->curinfo.failed_boots = 0;
	if (update_bootinfo(ctx) < 0) {
		perror("writing boot info");
		close_bootinfo(ctx);
		set_bootdev_writeable_status(0);
		return -2;
	}
	close_bootinfo(ctx);
	set_bootdev_writeable_status(0);
	return 0;

} /* boot_successful */

static int
boot_check_status(void)
{
	struct devinfo_context *ctx = NULL;
	int rc = 0;

	set_bootdev_writeable_status(1);
	if (find_bootinfo(0, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info, initializing\n");
		close_bootinfo(ctx);
		ctx = NULL;
		rc = boot_devinfo_init();
		if (rc == 0)
			rc = find_bootinfo(0, &ctx);
		if (rc < 0) {
			close_bootinfo(ctx);
			set_bootdev_writeable_status(0);
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
	close_bootinfo(ctx);
	set_bootdev_writeable_status(0);
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
	       "Failed boots:           %d\n",
	       ctx->curinfo.devinfo_version,
	       (ctx->curinfo.flags & FLAG_BOOT_IN_PROGRESS) ? "YES" : "NO",
	       ctx->curinfo.failed_boots);
	close_bootinfo(ctx);
	return 0;

} /* show_bootinfo */

/*
 * show_bootvar
 *
 * Prints out the value of a variable, or
 * all var=value settings if varname == NULL
 */
int
show_bootvar (const char *name)
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
			printf("%s=%s\n", var->name, var->value);
			if (name != NULL)
				break;
		}
	}
	close_bootinfo(ctx);
	if (!found) {
		fprintf(stderr, "not found: %s\n", name);
		return -1;
	}
	return 0;

} /* show_bootvar */

/*
 * set_bootvar
 *
 * Sets or deletes a variable.
 */
int
set_bootvar (const char *name, const char *value)
{
	struct devinfo_context *ctx;
	struct info_var *var, *prev;

	if (find_bootinfo(0, &ctx) < 0) {
		fprintf(stderr, "Could not locate device info\n");
		return -2;
	}

	if (strlen(name) > DEVINFO_BLOCK_SIZE-DEVINFO_HDR_SIZE) {
		fprintf(stderr, "error: variable name too long\n");
		close_bootinfo(ctx);
		return -1;
		strcpy(ctx->namebuf, name);
	}

	if (value != NULL) {
		size_t s = strlen(name) + strlen(value) + 2;
		if (ctx->varsize + s > DEVINFO_BLOCK_SIZE-DEVINFO_HDR_SIZE) {
			fprintf(stderr, "error: insufficient space for variable storage\n");
			close_bootinfo(ctx);
			return -1;
		}
		strcpy(ctx->valuebuf, value);
	}
	for (var = ctx->vars, prev = NULL; var != NULL && strcmp(name, var->name) != 0; prev = var, var = var->next);
	if (var == NULL) {
		if (value == NULL) {
			/* Deleting variable that isn't there - no error */
			close_bootinfo(ctx);
			return 0;
		}
		var = calloc(1, sizeof(struct info_var));
		if (var == NULL) {
			perror("calloc");
			close_bootinfo(ctx);
			return -1;
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
		prev->next = var->next;
		free(var);
	} else
		/* Changing value of found variable */
		var->value = ctx->valuebuf;

	if (update_bootinfo(ctx) < 0) {
		fprintf(stderr, "could not update variables\n");
		close_bootinfo(ctx);
		return -1;
	}
	close_bootinfo(ctx);
	return 0;

} /* set_bootvar */

/*
 * main program
 */
int
main (int argc, char * const argv[])
{

	int c, which, ret;

	if (argc < 2) {
		print_usage();
		return 1;
	}

	c = getopt_long_only(argc, argv, shortopts, options, &which);
	if (c == -1) {
		perror("getopt");
		print_usage();
		return 1;
	}

	switch (c) {
	case 'h':
		print_usage();
		return 0;
	case 'b':
		return boot_successful();
	case 'c':
		return boot_check_status();
	case 'I':
		return boot_devinfo_init();
	case 's':
		return show_bootinfo();
	case 'v':
		if (optind >= argc)
			return show_bootvar(NULL);
		return show_bootvar(argv[optind]);
	case 'V':
		if (optind >= argc) {
			fprintf(stderr, "Error: missing variable name\n");
			print_usage();
			return 1;
		}
		return set_bootvar(argv[optind], (optind < argc - 1 ? argv[optind+1] : NULL));
	default:
		fprintf(stderr, "Error: unrecognized option\n");
		print_usage();
		return 1;
	}

} /* main */
