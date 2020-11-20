/*
 * tegra-boot-control.c
 *
 * Tool for managing boot slot metadata
 * on tegra platforms that support A/B
 * redundancy.
 *
 * Copyright (c) 2020, Matthew Madison
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "bup.h"
#include "gpt.h"
#include "smd.h"
#include "soctype.h"
#include "config.h"

static struct option options[] = {
	{ "current-slot",	no_argument,		0, 'c' },
	{ "disable",		no_argument,		0, 'd' },
	{ "enable",		no_argument,		0, 'e' },
	{ "mark-successful",	no_argument,		0, 'm' },
	{ "set-active",		required_argument,	0, 'a' },
	{ "status",		no_argument,		0, 's' },
	{ "help",		no_argument,		0, 'h' },
	{ "version",		no_argument,		0, 0   },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":cdema:sh";

static char *optarghelp[] = {
	"--current-slot       ",
	"--disable            ",
	"--enable             ",
	"--mark-successful    ",
	"--set-active         ",
	"--status             ",
	"--help               ",
	"--version            ",
};

static char *opthelp[] = {
	"print current boot slot",
	"disable redundant boot",
	"enable redundant boot",
	"mark current boot slot successful",
	"set active slot for next reboot",
	"display redundancy and boot slot status",
	"display this help text",
	"display version information"
};

typedef enum {
	ACTION_CURRENT_SLOT,
	ACTION_DISABLE,
	ACTION_ENABLE,
	ACTION_MARK_SUCCESS,
	ACTION_SET_ACTIVE,
	ACTION_STATUS,
	ACTION_INVALID = 255,
} bootctrl_action_t;

static void
print_usage (void)
{
	int i;
	printf("\nUsage:\n");
	printf("\ttegra-boot-control <option>\n\n");
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
 * set_bootdev_writeable_status
 */
static int
set_bootdev_writeable_status (const char *bootdev, int make_writeable)
{
	char pathname[64];
	char buf[1];
	int fd, is_writeable, rc = 0;

	if (bootdev == NULL)
		return 0;
	if (strlen(bootdev) < 6 || strlen(bootdev) > 32)
		return 0;
	sprintf(pathname, "/sys/block/%s/force_ro", bootdev + 5);
	fd = open(pathname, O_RDWR);
	if (fd < 0)
		return 0;
	if (read(fd, buf, sizeof(buf)) != sizeof(buf)) {
		close(fd);
		return 0;
	}
	make_writeable = !!make_writeable;
	is_writeable = buf[0] == '0';
	if (make_writeable && !is_writeable) {
		if (write(fd, "0", 1) != 1)
			rc = 1;
	} else if (!make_writeable && is_writeable) {
		if (write(fd, "1", 1) != 1)
			rc = 1;
	}
	close(fd);

	if (rc)
		fprintf(stderr, "warning: could not change boot device write status\n");

	return make_writeable != is_writeable;

} /* set_bootdev_writeable_status */

/*
 * print_smd_info
 *
 * Displays the slot metadata.
 */
static int
print_smd_info (smd_context_t *smdctx, int curslot)
{
	unsigned int i;
	smd_redundancy_level_t level = smd_redundancy_level(smdctx);
	smd_slot_t slot_info;

	if (level == REDUNDANCY_OFF) {
		printf("Redundancy: disabled\n");
		return 0;
	}
	printf("Redundancy: enabled (%s)\n",
	      (level == REDUNDANCY_BOOTLOADER_ONLY ? "bootloader only" : "full"));
	printf("Current slot: %d\n", curslot);
	for (i = 0; i < 2; i++) {
		if (smd_slot_get(smdctx, i, &slot_info) < 0) {
			perror("smd_slot_get");
			return 1;
		}
		printf("Slot %u: priority: %d, suffix: %s, retry_count: %d, successful: %s\n", i,
		       slot_info.slot_prio, slot_info.slot_suffix, slot_info.slot_retry_count,
		       (slot_info.slot_successful ? "YES" : "NO"));
	}

	return 0;

} /* print_smd_info */


/*
 * main program
 */
int
main (int argc, char * const argv[])
{
	int c, which, fd;
	int reset_bootdev;
	int curslot;
	unsigned int selected_slot;
	int result = 1;
	tegra_soctype_t soctype;
	gpt_context_t *gptctx;
	bup_context_t *bupctx;
	smd_context_t *smdctx = NULL;
	bootctrl_action_t action = ACTION_INVALID;
	bool readonly = false;
	bool option_error = false;

	while ((c = getopt_long(argc, argv, shortopts, options, &which)) != -1) {
		switch (c) {
			case 'h':
				print_usage();
				return 0;
			case 'c':
				if (action != ACTION_INVALID)
					option_error = true;
				else
					action = ACTION_CURRENT_SLOT;
				break;
			case 'd':
				if (action != ACTION_INVALID)
					option_error = true;
				else
					action = ACTION_DISABLE;
				break;
			case 'e':
				if (action != ACTION_INVALID)
					option_error = true;
				else
					action = ACTION_ENABLE;
				break;
			case 'm':
				if (action != ACTION_INVALID)
					option_error = true;
				else
					action = ACTION_MARK_SUCCESS;
				break;
			case 'a':
				if (action != ACTION_INVALID) {
					option_error = true;
					break;
				}
				if (strcmp(optarg, "0") == 0)
					selected_slot = 0;
				else if (strcmp(optarg, "1") == 0)
					selected_slot = 1;
				else {
					fprintf(stderr, "Error: slot number must be '0' or '1'\n");
					print_usage();
					return 1;
				}
				action = ACTION_SET_ACTIVE;
				break;
			case 's':
				if (action != ACTION_INVALID)
					option_error = true;
				else {
					action = ACTION_STATUS;
					readonly = true;
				}
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
		}
		if (option_error) {
			fprintf(stderr, "Error: multiple actions specified\n");
			print_usage();
			return 1;
		}
	}

	if (action == ACTION_INVALID) {
		fprintf(stderr, "Error: no action specified\n");
		print_usage();
		return 1;
	}

	soctype = soctype_get();
	if (soctype == TEGRA_SOCTYPE_INVALID) {
		fprintf(stderr, "Error: could not determine SoC type\n");
		return 1;
	}

	if (soctype != TEGRA_SOCTYPE_186 &&
	    soctype != TEGRA_SOCTYPE_194) {
		fprintf(stderr, "Error: unsupported SoC type: %s\n", soctype_name(soctype));
		return 1;
	}

	if (action == ACTION_CURRENT_SLOT) {
		curslot = smd_get_current_slot();
		if (curslot < 0) {
			perror("retrieving current boot slot");
			return 1;
		}
		printf("%d\n", curslot);
		return 0;
	}

	bupctx = bup_init(NULL);
	if (bupctx == NULL) {
		perror("loading boot control config");
		return 1;
	}

	gptctx = gpt_init(bup_gpt_device(bupctx), 512);
	if (gptctx == NULL) {
		perror("boot sector GPT");
		bup_finish(bupctx);
		return 1;
	}
	if (gpt_load(gptctx, GPT_LOAD_BACKUP_ONLY|GPT_NVIDIA_SPECIAL)) {
		fprintf(stderr, "Error: cannot load boot sector partition table\n");
		gpt_finish(gptctx);
		bup_finish(bupctx);
		return 1;
	}

	if (readonly) {
		reset_bootdev = 0;
		fd = open(bup_boot_device(bupctx), O_RDONLY);
	} else {
		reset_bootdev = set_bootdev_writeable_status(bup_boot_device(bupctx), 1);
		fd = open(bup_boot_device(bupctx), O_RDWR);
	}
	if (fd < 0) {
		perror(bup_boot_device(bupctx));
		goto reset_and_depart;
	}

	smdctx = smd_init(gptctx, fd);
	if (smdctx == NULL) {
		if (action == ACTION_ENABLE || action == ACTION_DISABLE) {
			smdctx = smd_new((action == ACTION_ENABLE ? REDUNDANCY_FULL : REDUNDANCY_OFF));
			if (smdctx == NULL)
				perror("initializing slot metadata");
		} else
			perror("loading slot metadata");
		if (smdctx == NULL)
			goto reset_and_depart;
	}

	switch (action) {
		case ACTION_ENABLE:
			if (smd_set_redundancy_level(smdctx, REDUNDANCY_FULL) < 0)
				perror("setting redundancy level");
			else {
				printf("Enabled redundancy.\n");
				result = 0;
			}
			break;
		case ACTION_DISABLE:
			if (smd_set_redundancy_level(smdctx, REDUNDANCY_OFF) < 0)
				perror("setting redundancy level");
			else {
				printf("Disabled redundancy.\n");
				result = 0;
			}
			break;
		case ACTION_MARK_SUCCESS:
			curslot = smd_get_current_slot();
			if (curslot >= 0 && smd_slot_mark_successful(smdctx, curslot) == 0) {
				printf("Slot %d marked successful.\n", curslot);
				result = 0;
			} else
				perror("marking slot successful");
			break;
		case ACTION_SET_ACTIVE:
			if (smd_slot_mark_active(smdctx, selected_slot) < 0)
				perror("setting active slot");
			else {
				printf("Slot %u set active for next boot.\n", selected_slot);
				result = 0;
			}
			break;
		case ACTION_STATUS:
			curslot = smd_get_current_slot();
			if (curslot < 0)
				perror("getting current boot slot");
			else
				result = print_smd_info(smdctx, curslot);
			break;
		default:
			fprintf(stderr, "Internal error: unrecognized action\n");
			break;
	}

	if (!readonly && result == 0) {
		if (smd_update(smdctx, gptctx, fd, (action == ACTION_ENABLE || action == ACTION_DISABLE)) < 0) {
			perror("updating slot metadata");
			result = 1;
		}
	}

  reset_and_depart:
	if (smdctx)
		smd_finish(smdctx);
	if (fd >= 0) {
		if (!readonly)
			fsync(fd);
		close(fd);
	}
	if (reset_bootdev)
		set_bootdev_writeable_status(bup_boot_device(bupctx), 0);

	gpt_finish(gptctx);
	bup_finish(bupctx);

	return result;

} /* main */
