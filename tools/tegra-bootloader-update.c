/*
 * tegra-bootloader-update.c
 *
 * Tool for updating/initializing Tegra boot partitions
 * using a BUP package.
 *
 * Copyright (c) 2019, Matthew Madison
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "bup.h"
#include "gpt.h"


static struct option options[] = {
	{ "initialize",		no_argument,		0, 'i' },
	{ "slot-suffix",	required_argument,	0, 's' },
	{ "dry-run",		no_argument,		0, 'n' },
	{ "help",		no_argument,		0, 'h' },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":ins:h";

static char *optarghelp[] = {
	"--initialize         ",
	"--slot-suffix        ",
	"--dry-run            ",
	"--help               ",
};

static char *opthelp[] = {
	"initialize the entire set of boot partitions",
	"update only the redundant boot partitions with the specified suffix",
	"do not perform any writes, just show what would be written",
	"display this help text"
};

struct update_entry_s {
	char partname[64];
	gpt_entry_t *part;
	char devname[PATH_MAX];
	off_t bup_offset;
	size_t length;
};

static struct update_entry_s redundant_entries[32];
static struct update_entry_s nonredundant_entries[32];
static unsigned int redundant_entry_count;
static unsigned int nonredundant_entry_count;
static size_t contentbuf_size;
static uint8_t *contentbuf;

static void
print_usage (void)
{
	int i;
	printf("\nUsage:\n");
	printf("\ttegra-bootloader-update <option> <bup-package-path>\n\n");
	printf("Options:\n");
	for (i = 0; i < sizeof(options)/sizeof(options[0]) && options[i].name != 0; i++) {
		printf(" %s\t%c%c\t%s\n",
		       optarghelp[i],
		       (options[i].val == 0 ? ' ' : '-'),
		       (options[i].val == 0 ? ' ' : options[i].val),
		       opthelp[i]);
	}
	printf("\nArguments:\n");
	printf(" <bup-package-path>\tpathname of bootloader update package\n");

} /* print_usage */

/*
 * set_bootdev_writeable_status
 */
static int
set_bootdev_writeable_status (const char *bootdev, int make_writeable)
{
	char pathname[64];
	char buf[1];
	int fd, is_writeable;

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
	if (make_writeable && !is_writeable)
		write(fd, "0", 1);
	else if (!make_writeable && is_writeable)
		write(fd, "1", 1);
	close(fd);

	return make_writeable != is_writeable;

} /* set_bootdev_writeable_status */

/*
 * process_entry
 */
static int
process_entry (bup_context_t *bupctx, int bootfd, struct update_entry_s *ent, int dryrun)
{
	ssize_t n, total, remain;
	int fd;

	printf("  Processing %s:...", ent->partname);
	if (bup_setpos(bupctx, ent->bup_offset) == (off_t) -1) {
		printf("[FAIL]\n");
		fprintf(stderr, "could not set position for %s\n", ent->partname);
		return -1;
	}
	for (total = 0; total < ent->length; total += n) {
		n = bup_read(bupctx, contentbuf + total, contentbuf_size - total);
		if (n <= 0) {
			printf("[FAIL]\n");
			fprintf(stderr, "error reading content for %s\n", ent->partname);
			return -1;
		}
	}

	if (dryrun) {
		printf("[OK] (dry run)\n");
		return 0;
	}
	if (ent->part == NULL) {
		fd = open(ent->devname, O_WRONLY);
		if (fd < 0) {
			printf("[FAIL]\n");
			perror(ent->devname);
			return -1;
		}
	} else {
		fd = bootfd;
		if (lseek(fd, ent->part->first_lba * 512, SEEK_SET) == (off_t) -1) {
			printf("[FAIL]\n");
			fprintf(stderr, "could not seek to start of %s in boot partition\n", ent->partname);
			close(fd);
			return -1;
		}
	}
	for (remain = ent->length, total = 0; remain > 0; total += n, remain -= n) {
		n = write(fd, contentbuf + total, remain);
		if (n <= 0) {
			printf("[FAIL]\n");
			perror(ent->partname);
			close(fd);
			return -1;
		}
	}

	if (ent->part == NULL) {
		fsync(fd);
		close(fd);
	}

	printf("[OK]\n");
	return 0;

} /* process_entry */

/*
 * main program
 */
int
main (int argc, char * const argv[])
{
	int c, which, fd;
	int reset_bootdev;
	gpt_context_t *gptctx;
	bup_context_t *bupctx;
	gpt_entry_t *part;
	struct update_entry_s updent;
	void *bupiter;
	const char *partname;
	const char *suffix = NULL;
	off_t offset;
	size_t length;
	size_t largest_length;
	unsigned int version;
	char pathname[PATH_MAX];
	int initialize = 0;
	int dryrun = 0;
	int ret = 0;
	unsigned int i;

	while ((c = getopt_long_only(argc, argv, shortopts, options, &which)) != -1) {
		switch (c) {
			case 'h':
				print_usage();
				return 0;
			case 'i':
				if (suffix != NULL) {
					fprintf(stderr, "Error: cannot use --initialize with --slot-suffix\n");
					print_usage();
					return 1;
				}
				initialize = 1;
				break;
			case 's':
				if (initialize) {
					fprintf(stderr, "Error: cannot specify --slot-suffix with --initialize\n");
					print_usage();
					return 1;
				}
				if (strcmp(optarg, "_a") == 0)
					suffix = "";
				else
					suffix = optarg;
				if (suffix[0] != '\0' && strcmp(suffix, "_b") != 0) {
					fprintf(stderr, "Error: slot suffix must be either _a or _b\n");
					print_usage();
					return 1;
				}
				break;
			case 'n':
				dryrun = 1;
				break;
			default:
				fprintf(stderr, "Error: unrecognized option\n");
				print_usage();
				return 1;
		}
	}

	if (optind >= argc) {
		fprintf(stderr, "Error: missing required argument\n");
		print_usage();
		return 1;
	}
	if (!initialize && suffix == NULL) {
		fprintf(stderr, "Error: must either specify --initialize or --slot-suffix\n");
		print_usage();
		return 1;
	}

	bupctx = bup_init(argv[optind]);
	if (bupctx == NULL) {
		perror(argv[optind]);
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

	/*
	 * Verify that all of the partitions we need to update are acutally present,
	 * and at the same time build the set of update tasks.
	 *
	 * For initialization, we separate the redundant entries from the non-redundant
	 * ones and write the non-redundant entries last (there's probably only one, the BCT).
	 * For updates, we never write the non-redundant entries.
	 */
	bupiter = 0;
	redundant_entry_count = nonredundant_entry_count = 0;
	largest_length = 0;
	while (bup_enumerate_entries(bupctx, &bupiter, &partname, &offset, &length, &version)) {
		gpt_entry_t *part, *part_b;
		char partname_b[64], pathname_b[PATH_MAX];

		sprintf(partname_b, "%s_b", partname);
		memset(&updent, 0, sizeof(updent));
		strcpy(updent.partname, partname);
		updent.bup_offset = offset;
		updent.length = length;
		if (length > largest_length)
			largest_length = length;

		part = gpt_find_by_name(gptctx, partname);
		if (part != NULL) {
			part_b = gpt_find_by_name(gptctx, partname_b);
			if (initialize) {
				if (part_b != NULL) {
					redundant_entries[redundant_entry_count] = updent;
					redundant_entries[redundant_entry_count].part = part;
					redundant_entry_count += 1;
					redundant_entries[redundant_entry_count] = updent;
					strcpy(redundant_entries[redundant_entry_count].partname, partname_b);
					redundant_entries[redundant_entry_count].part = part_b;
					redundant_entry_count += 1;
				} else {
					nonredundant_entries[nonredundant_entry_count] = updent;
					nonredundant_entries[nonredundant_entry_count].part = part;
					nonredundant_entry_count += 1;
				}
			} else if (part_b != NULL) {
				redundant_entries[redundant_entry_count] = updent;
				strcpy(redundant_entries[redundant_entry_count].partname, (*suffix == '\0' ? partname : partname_b));
				redundant_entries[redundant_entry_count].part = (*suffix == '\0' ? part : part_b);
				redundant_entry_count += 1;
			}
		} else {
			int redundant;
			sprintf(pathname, "/dev/disk/by-partlabel/%s", partname);
			if (access(pathname, F_OK|W_OK) != 0) {
				fprintf(stderr, "Error: cannot locate partition: %s\n", partname);
				ret = 1;
				goto depart;
			}
			sprintf(pathname_b, "/dev/disk/by-partlabel/%s_b", partname);
			redundant = access(pathname_b, F_OK|W_OK) == 0;
			if (initialize) {
				if (redundant) {
					redundant_entries[redundant_entry_count] = updent;
					strcpy(redundant_entries[redundant_entry_count].devname, pathname);
					redundant_entry_count += 1;
					redundant_entries[redundant_entry_count] = updent;
					strcpy(redundant_entries[redundant_entry_count].partname, partname_b);
					strcpy(redundant_entries[redundant_entry_count].devname, pathname_b);
					redundant_entry_count += 1;
				} else {
					nonredundant_entries[nonredundant_entry_count] = updent;
					strcpy(nonredundant_entries[nonredundant_entry_count].devname, pathname);
					nonredundant_entry_count += 1;
				}
			} else if (redundant) {
				redundant_entries[redundant_entry_count] = updent;
				strcpy(redundant_entries[redundant_entry_count].partname, (*suffix == '\0' ? partname : partname_b));
				strcpy(redundant_entries[redundant_entry_count].devname, (*suffix == '\0' ? pathname : pathname_b));
				redundant_entry_count += 1;
			}
		}
	}

	contentbuf = malloc(largest_length);
	if (contentbuf == NULL) {
		perror("allocating content buffer");
		goto depart;
	}
	contentbuf_size = largest_length;

	if (dryrun) {
		reset_bootdev = 0;
		fd = -1;
	} else {
		reset_bootdev = set_bootdev_writeable_status(bup_boot_device(bupctx), 1);
		fd = open(bup_boot_device(bupctx), O_RDWR);
		if (fd < 0) {
			perror(bup_boot_device(bupctx));
			goto reset_and_depart;
		}
	}

	for (i = 0; i < redundant_entry_count; i++)
		if (process_entry(bupctx, fd, &redundant_entries[i], dryrun) != 0)
			goto reset_and_depart;

	if (initialize) {
		for (i = 0; i < nonredundant_entry_count; i++)
			if (process_entry(bupctx, fd, &nonredundant_entries[i], dryrun) != 0)
				goto reset_and_depart;
	}

  reset_and_depart:
	if (fd >= 0) {
		fsync(fd);
		close(fd);
	}
	if (reset_bootdev)
		set_bootdev_writeable_status(bup_boot_device(bupctx), 0);

  depart:
	gpt_finish(gptctx);
	bup_finish(bupctx);

	return ret;

} /* main */
