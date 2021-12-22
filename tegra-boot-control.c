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
#include <stdbool.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <tegra-eeprom/cvm.h>
#include <limits.h>
#include "gpt.h"
#include "smd.h"
#include "util.h"
#include "config.h"

static struct option options[] = {
	{ "current-slot",	no_argument,		0, 'c' },
	{ "disable",		no_argument,		0, 'd' },
	{ "enable",		no_argument,		0, 'e' },
	{ "mark-successful",	no_argument,		0, 'm' },
	{ "set-active",		required_argument,	0, 'a' },
	{ "status",		no_argument,		0, 's' },
	{ "load",		required_argument,	0, 'L' },
	{ "dump",		required_argument,	0, 'D' },
	{ "help",		no_argument,		0, 'h' },
	{ "version",		no_argument,		0, 0   },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":cdema:si:D:h";

static char *optarghelp[] = {
	"--current-slot       ",
	"--disable            ",
	"--enable             ",
	"--mark-successful    ",
	"--set-active         ",
	"--status             ",
	"--load               ",
	"--dump               ",
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
	"load slot metadata from file",
	"dump slot metadata to file",
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
	ACTION_LOAD,
	ACTION_DUMP,
	ACTION_INVALID = 255,
} bootctrl_action_t;

static const char bootdev[] = OTABOOTDEV;
static const char gptdev[] = OTAGPTDEV;
static char slot_metadata_bin_file[PATH_MAX];

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
	int c, which, fd, smd_fd;
	bool reset_bootdev;
	int curslot;
	unsigned int selected_slot = 0;
	int result = 1;
	tegra_soctype_t soctype;
	gpt_context_t *gptctx;
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
			case 'L':
				if (action != ACTION_INVALID)
					option_error = true;
				else
					action = ACTION_LOAD;
				strncpy(slot_metadata_bin_file, optarg, sizeof(slot_metadata_bin_file));
				if (access(slot_metadata_bin_file, R_OK) != 0) {
					fprintf(stderr, "Error: cannot access slot metadata file %s\n", slot_metadata_bin_file);
					return 1;
				}
				break;
			case 'D':
				if (action != ACTION_INVALID)
					option_error = true;
				else
					action = ACTION_DUMP;
				strncpy(slot_metadata_bin_file, optarg, sizeof(slot_metadata_bin_file));
				readonly = true;
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

	soctype = cvm_soctype();
	if (soctype == TEGRA_SOCTYPE_INVALID) {
		fprintf(stderr, "Error: could not determine SoC type\n");
		return 1;
	}

	if (soctype != TEGRA_SOCTYPE_186 &&
	    soctype != TEGRA_SOCTYPE_194) {
		fprintf(stderr, "Error: unsupported SoC type: %s\n", cvm_soctype_name(soctype));
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

	gptctx = gpt_init(gptdev, 512, 0);
	if (gptctx == NULL) {
		perror("boot sector GPT");
		return 1;
	}
	if (gpt_load(gptctx, GPT_BACKUP_ONLY|GPT_NVIDIA_SPECIAL)) {
		fprintf(stderr, "Error: cannot load boot sector partition table\n");
		gpt_finish(gptctx);
		return 1;
	}

	if (readonly) {
		reset_bootdev = false;
		fd = open(bootdev, O_RDONLY);
	} else {
		reset_bootdev = set_bootdev_writeable_status(bootdev, true);
		fd = open(bootdev, O_RDWR);
	}
	if (fd < 0) {
		perror(bootdev);
		goto reset_and_depart;
	}

	if (action == ACTION_DUMP) {
		smd_fd = creat(slot_metadata_bin_file, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
		if (smd_fd < 0) {
			perror(slot_metadata_bin_file);
			goto reset_and_depart;
		}
	}

	if (action == ACTION_LOAD) {
		smd_fd = open(slot_metadata_bin_file, O_RDONLY);
		if (smd_fd < 0) {
			perror(slot_metadata_bin_file);
			goto reset_and_depart;
		}
		smdctx = smd_new_from_file(smd_fd);
		if (!smdctx) {
			perror("initializing slot metadata");
			goto reset_and_depart;
		}
	} else {
		smdctx = smd_init(gptctx, fd);
	}

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
		case ACTION_LOAD:
			result = print_smd_info(smdctx, 0);
			break;
		case ACTION_DUMP:
			result = smd_write_to_file(smdctx, smd_fd);
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
	if (smd_fd >= 0) {
		close(smd_fd);
	}
	if (fd >= 0) {
		if (!readonly)
			fsync(fd);
		close(fd);
	}
	if (reset_bootdev)
		set_bootdev_writeable_status(bootdev, false);

	gpt_finish(gptctx);

	return result;

} /* main */
