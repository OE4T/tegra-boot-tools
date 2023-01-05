/*
 * tegra-bootinfo.c
 *
 * Tools for working with the boot information
 * block.  See the documentation for more detail.
 *
 * Copyright (c) 2019-2022, Matthew Madison
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <tegra-eeprom/cvm.h>
#include "bootinfo.h"
#include "smd.h"
#include "util.h"
#include "config.h"

#define MAX_BOOT_FAILURES 3

static const char bootdev[] = OTABOOTDEV;
static const char gptdev[] = OTAGPTDEV;

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


/*
 * mark_nv_boot_successful
 *
 * For platforms with A/B bootloaders, marks the current boot
 * slot as successful.	This is used early on in the OS startup
 * to prevent the NVIDIA bootoader failover handling from kicking
 * in when we're dealing with an OS startup failure, because the
 * bl failover will give up after some number of tries and put
 * the device into forced-recovery mode.
 */
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

/*
 * boot_successful
 *
 * Resets the boot-in-progress flag and zeros the failed boot counter.
 * Typically used at the end of the system startup sequence.
 *
 */
static int
boot_successful(void)
{
	bootinfo_context_t *ctx;
	unsigned int failcount;
	int rc = 0;

	if (mark_nv_boot_successful() < 0)
		return 1;

	if (bootinfo_open(BOOTINFO_O_RDWR|BOOTINFO_O_CREAT, &ctx) < 0) {
		perror("bootinfo_open");
		return 1;
	}

	if (bootinfo_mark_boot_success(ctx, &failcount) < 0) {
		perror("bootinfo_mark_boot_success");
		rc = 1;
	} if (failcount > 0)
		  fprintf(stderr, "Failed boot count: %u\n", failcount);

	if (bootinfo_close(ctx) < 0)
		perror("bootinfo_close");

	return rc;

} /* boot_successful */

/*
 * boot_check_status
 *
 * Performs the early-boot (initrd stage) boot count check
 * sequence.
 *
 */
static int
boot_check_status (void)
{
	bootinfo_context_t *ctx;
	unsigned int failcount;
	int rc = 0;

	if (mark_nv_boot_successful() < 0)
		return 1;

	if (bootinfo_open(BOOTINFO_O_RDWR|BOOTINFO_O_CREAT, &ctx) < 0) {
		perror("bootinfo_open");
		return 1;
	}
	if (bootinfo_check_boot_status(ctx, &failcount) < 0) {
		perror("bootinfo_check_boot_status");
		return 1;
	}
	if (failcount != 0) {
		fprintf(stderr, "Boot failures: %u\n", failcount);
		if (failcount >= MAX_BOOT_FAILURES) {
			fprintf(stderr, "Too many boot failures, exit with error to signal boot slot switch\n");
			rc = 77;
			bootinfo_mark_boot_success(ctx, NULL);
		}
	}
	if (bootinfo_close(ctx) < 0)
		perror("bootinfo_close");

	return rc;

} /* boot_check_status */

/*
 * init_bootinfo
 *
 * Initialize the bootinfo block.
 */
static int
init_bootinfo (bool force_init)
{
	bootinfo_context_t *ctx;
	unsigned int flags = BOOTINFO_O_RDWR;

	if (force_init)
		flags |= BOOTINFO_O_CREAT|BOOTINFO_O_FORCE_INIT;

	if (bootinfo_open(flags, &ctx) < 0) {
		perror("bootinfo_open");
		return 1;
	}

	if (bootinfo_close(ctx) < 0) {
		perror("bootinfo_close");
		return 1;
	}

	return 0;

} /* init_bootinfo */

/*
 * show_bootinfo
 *
 * Prints out the boot info header information.
 */
static int
show_bootinfo (void) {

	bootinfo_context_t *ctx;
	unsigned int version, failcount, ext_sector_count;
	bool boot_in_progress;
	int rc = 0;

	if (bootinfo_open(BOOTINFO_O_RDONLY, &ctx) < 0) {
		perror("bootinfo_open");
		return 1;
	}

	if (bootinfo_get_info(ctx, &version, &boot_in_progress, &failcount,
			      &ext_sector_count) < 0) {
		perror("bootinfo_get_info");
		rc = 1;
	} else
		printf("devinfo version:        %u\n"
		       "Boot in progress:       %s\n"
		       "Failed boots:           %d\n"
		       "Extension space:        %d sector%s\n",
		       version,
		       (boot_in_progress ? "YES" : "NO"),
		       failcount,
		       ext_sector_count,
		       (ext_sector_count  == 1 ? "" : "s"));

	bootinfo_close(ctx);
	return rc;

} /* show_bootinfo */

/*
 * show_bootvar
 *
 * Prints out the value of a variable, or
 * all var=value settings if varname == NULL
 */
int
show_bootvar (const char *name, bool omitname)
{
	bootinfo_context_t *ctx;
	bootinfo_var_iter_context_t *iter;
	int rc = 0;
	char namebuf[256], valuebuf[1024];

	if (bootinfo_open(BOOTINFO_O_RDONLY, &ctx) < 0) {
		perror("bootinfo_open");
		return -2;
	}
	if (name != NULL) {
		if (bootinfo_var_get(ctx, name, valuebuf, sizeof(valuebuf)) < 0) {
			if (errno == ENOENT) {
				fprintf(stderr, "not found: %s\n", name);
				rc = 1;
			} else {
				perror("bootinfo_var_get");
				rc = -2;
			}
		} else {
			if (omitname)
				printf("%s\n", valuebuf);
			else
				printf("%s=%s\n", name, valuebuf);
		}
	} else {
		if (bootinfo_var_iter_begin(ctx, &iter) < 0) {
			rc = -2;
			perror("bootinfo_var_iter_begin");
		} else {
			while (bootinfo_var_iter_next(iter, namebuf, sizeof(namebuf),
						      valuebuf, sizeof(valuebuf)) == 0) {
				if (omitname)
					printf("%s\n", valuebuf);
				else
					printf("%s=%s\n", namebuf, valuebuf);
			}
			bootinfo_var_iter_end(iter);
		}
	}
	bootinfo_close(ctx);
	return rc;

} /* show_bootvar */

/*
 * set_bootvar
 *
 * Sets or deletes a variable.
 */
int
set_bootvar (const char *name, const char *value, char *inputfile)
{
	bootinfo_context_t *ctx;
	static char valuebuf[1024];
	int rc = 0;

	if (inputfile != NULL) {
		FILE *fp;
		size_t n, cnt;

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
		/*
		 * Trim off trailing CR/LFs from input file
		 */
		while (n > 0 && (valuebuf[n-1] == '\n' || valuebuf[n-1] == '\r'))
			n -= 1;
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

	if (bootinfo_open(BOOTINFO_O_RDWR, &ctx) < 0) {
		perror("bootinfo_open");
		return -2;
	}
	if (bootinfo_var_set(ctx, name, value) < 0) {
		perror("bootinfo_var_set");
		rc = 1;
	}
	if (bootinfo_close(ctx) < 0)
		perror("bootinfo_close");

	return rc;

} /* set_bootvar */

/*
 * main program
 */
int
main (int argc, char * const argv[])
{

	int c, which;
	bool omitname = false;
	bool force_init = false;
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
			omitname = true;
			break;
		case 'f':
			inputfile = strdup(optarg);
			break;
		case 'F':
			force_init = true;
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

	switch (cmd) {
	case success:
		return boot_successful();
	case check:
		return boot_check_status();
	case show:
		return show_bootinfo();
	case init:
		return init_bootinfo(force_init);
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
