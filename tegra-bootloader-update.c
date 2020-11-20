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
#include "soctype.h"
#include "bct.h"
#include "smd.h"
#include "config.h"

static struct option options[] = {
	{ "initialize",		no_argument,		0, 'i' },
	{ "slot-suffix",	required_argument,	0, 's' },
	{ "dry-run",		no_argument,		0, 'n' },
	{ "chipid",		required_argument,	0, 'c' },
	{ "help",		no_argument,		0, 'h' },
	{ "version",		no_argument,		0, 0   },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":ins:c:h";

static char *optarghelp[] = {
	"--initialize         ",
	"--slot-suffix        ",
	"--dry-run            ",
	"--chipid             ",
	"--help               ",
	"--version            ",
};

static char *opthelp[] = {
	"initialize the entire set of boot partitions",
	"update only the redundant boot partitions with the specified suffix (with no SMD update)",
	"do not perform any writes, just show what would be written",
	"specify Tegra chip ID (used only for testing)",
	"display this help text",
	"display version information"
};

struct update_entry_s {
	char partname[64];
	gpt_entry_t *part;
	char devname[PATH_MAX];
	off_t bup_offset;
	size_t length;
};

#define MAX_ENTRIES 64
static struct update_entry_s redundant_entries[MAX_ENTRIES];
static struct update_entry_s nonredundant_entries[MAX_ENTRIES];
static unsigned int redundant_entry_count;
static unsigned int nonredundant_entry_count;
static size_t contentbuf_size;
static uint8_t *contentbuf, *slotbuf;
static int bct_updated;
static tegra_soctype_t soctype = TEGRA_SOCTYPE_INVALID;

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
 * update_bct
 *
 * Special handling for writing the BCT. Content to be
 * written expected to be present in contentbuf.
 *
 * A BCT slot is 0xE00 = 3584 bytes.
 * A block is 16KiB and holds multiple slots.
 * The Tegra bootrom can handle up to 63 blocks, but
 * in practice, only block 0 slots 0 & 1, and block 1 slot 0
 * are used.
 *
 * Write sequence is block 0/slot 1, then block 1/slot 0,
 * then block 0/slot 0.
 *
 */
static int
update_bct (int bootfd, void *curbct, struct update_entry_s *ent)
{
	ssize_t n, total, remain;
	unsigned int block_size = 16384;
	unsigned int page_size = 512;
	int i;

	if ((soctype == TEGRA_SOCTYPE_186 && !bct_update_valid_t18x(slotbuf, curbct, &block_size, &page_size)) ||
	    (soctype == TEGRA_SOCTYPE_194 && !bct_update_valid_t19x(slotbuf, curbct, &block_size, &page_size))) {
		printf("[FAIL]\n");
		fprintf(stderr, "Error: validation check failed for BCT update\n");
		return -1;
	}

	for (i = 0; i < 3; i++) {
		off_t offset;
		switch (i) {
			case 0:
				offset = page_size * ((ent->length + (page_size-1)) / page_size);
				break;
			case 1:
				offset = block_size;
				break;
			case 2:
				offset = 0;
				break;
		}
		printf("[offset=%lu]...", (unsigned long) offset);
		if (lseek(bootfd, ent->part->first_lba * 512 + offset, SEEK_SET) == (off_t) -1) {
			printf("[FAIL]\n");
			perror("BCT");
			return -1;
		}

		for (remain = ent->length, total = 0; remain > 0; total += n, remain -= n) {
			n = write(bootfd, contentbuf + total, remain);
			if (n <= 0) {
				printf("[FAIL]\n");
				perror("writing BCT");
				return -1;
			}
		}
	}

	fsync(bootfd);
	bct_updated = 1;
	printf("[OK]\n");
	return 0;

} /* update_bct */

/*
 * maybe_update_bootpart
 *
 * Update a boot partition if its current contents
 * differ from the BUP content (which is in contentbuf).
 *
 */
static int
maybe_update_bootpart (int bootfd, struct update_entry_s *ent, int is_bct)
{
	ssize_t n, total, remain;

	if (ent->length > (ent->part->last_lba - ent->part->first_lba + 1) * 512) {
		printf("[FAIL]\n");
		fprintf(stderr, "Error: BUP contents too large for boot partition\n");
		return -1;
	}

	if (lseek(bootfd, ent->part->first_lba * 512, SEEK_SET) == (off_t) -1) {
		printf("[FAIL]\n");
		perror(ent->partname);
		return -1;
	}
	for (remain = ent->length, total = 0; remain > 0; total += n, remain -= n) {
		n = read(bootfd, slotbuf + total, remain);
		if (n <= 0) {
			printf("[FAIL]\n");
			perror("reading BCT");
			return -1;
		}
	}
	if (memcmp(contentbuf, slotbuf, ent->length) == 0) {
		printf("[no update needed]\n");
		return 0;
	}

	if (is_bct)
		return update_bct(bootfd, contentbuf, ent);
	 
	if (lseek(bootfd, ent->part->first_lba * 512, SEEK_SET) == (off_t) -1) {
		printf("[FAIL]\n");
		perror(ent->partname);
		return -1;
	}

	for (remain = ent->length, total = 0; remain > 0; total += n, remain -= n) {
		n = write(bootfd, contentbuf + total, remain);
		if (n <= 0) {
			printf("[FAIL]\n");
			perror(ent->partname);
			return -1;
		}
	}

	fsync(bootfd);
	printf("[OK]\n");
	return 0;

} /* maybe_update_bootpart */

/*
 * process_entry
 */
static int
process_entry (bup_context_t *bupctx, int bootfd, struct update_entry_s *ent, int dryrun)
{
	ssize_t n, total, remain;
	int fd;

	printf("  Processing %s... ", ent->partname);
	fflush(stdout);
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
	if (ent->part != NULL)
		return maybe_update_bootpart(bootfd, ent, strcmp(ent->partname, "BCT") == 0);

	fd = open(ent->devname, O_WRONLY);
	if (fd < 0) {
		printf("[FAIL]\n");
		perror(ent->devname);
		return -1;
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

	fsync(fd);
	close(fd);
	printf("[OK]\n");
	return 0;

} /* process_entry */

/*
 * order_entries
 *
 * Sort an entries list to ensure that we process
 * mb2/mb2_b before BCT before mb1/mb1_b.
 */
static void
order_entries (struct update_entry_s *orig, struct update_entry_s **ordered, int count)	       
{
	int i, j, mb1, mb1_b, bct, mb2, mb2_b;

	mb1 = mb1_b = bct = mb2 = mb2_b = -1;
	j = 0;
	for (i = 0; i < count; i++) {
		if (strcmp(orig[i].partname, "mb1") == 0)
			mb1 = i;
		else if (strcmp(orig[i].partname, "mb1_b") == 0)
			mb1_b = i;
		else if (strcmp(orig[i].partname, "mb2") == 0)
			mb2 = i;
		else if (strcmp(orig[i].partname, "mb2_b") == 0)
			mb2_b = i;
		else if (strcmp(orig[i].partname, "BCT") == 0)
			bct = i;
		else
			ordered[j++] = &orig[i];
	}

	if (mb2 >= 0)
		ordered[j++] = &orig[mb2];
	if (mb2_b >= 0)
		ordered[j++] = &orig[mb2_b];
	if (bct >= 0)
		ordered[j++] = &orig[bct];
	if (mb1 >= 0)
		ordered[j++] = &orig[mb1];
	if (mb1_b >= 0)
		ordered[j++] = &orig[mb1_b];

	if (j != count)
		fprintf(stderr, "Warning: ordered entry list mismatch\n");

} /* order_entries */

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
	smd_context_t *smdctx = NULL;
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
	int missing_count;
	int curslot = -1;
	int slot_specified = 0;
	const char *missing[32];
	struct update_entry_s *ordered_entries[MAX_ENTRIES], mb1_other;
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
				slot_specified = 1;
				break;
			case 'c':
				soctype = soctype_from_chipid(strtoul(optarg, NULL, 0));
				if (soctype == TEGRA_SOCTYPE_INVALID) {
					fprintf(stderr, "Error: invalid SoC type: %s\n", optarg);
					print_usage();
					return 1;
				}
				break;
			case 'n':
				dryrun = 1;
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
	}

	if (optind >= argc) {
		fprintf(stderr, "Error: missing required argument\n");
		print_usage();
		return 1;
	}

	if (soctype == TEGRA_SOCTYPE_INVALID) {
		soctype = soctype_get();
		if (soctype == TEGRA_SOCTYPE_INVALID) {
			fprintf(stderr, "Error: could not determine SoC type\n");
			return 1;
		}
	}

	if (soctype != TEGRA_SOCTYPE_186 &&
	    soctype != TEGRA_SOCTYPE_194) {
		fprintf(stderr, "Error: unsupported SoC type: %s\n", soctype_name(soctype));
		return 1;
	}

	if (!slot_specified && !initialize) {
		curslot = smd_get_current_slot();
		if (curslot < 0) {
			perror("retrieving current boot slot");
			return 1;
		}
		if (curslot == 0)
			suffix = "_b";
		else
			suffix = "";
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

	if (dryrun) {
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
		if (initialize) {
			smdctx = smd_new(REDUNDANCY_FULL);
			if (smdctx == NULL)
				perror("initializing slot metadata");
		} else
			perror("loading slot metadata");
		if (smdctx == NULL)
			goto reset_and_depart;
	}

	if (!slot_specified && smd_redundancy_level(smdctx) != REDUNDANCY_FULL) {
		if (dryrun)
			printf("[skip] enable redundancy in slot metadata\n");
		else if (smd_set_redundancy_level(smdctx, REDUNDANCY_FULL) < 0) {
			perror("enabling redundancy in slot metadata");
			goto reset_and_depart;
		}
	}

	missing_count = bup_find_missing_entries(bupctx, missing, sizeof(missing)/sizeof(missing[0]));
	if (missing_count < 0) {
		fprintf(stderr, "Error checking BUP payload for missing entries\n");
		goto reset_and_depart;
	} else if (missing_count > 0) {
		int i;
		fprintf(stderr, "Error: missing entries for partition%s: %s",
			(missing_count == 1 ? "" : "s"), missing[0]);
		for (i = 1; i < missing_count; i++)
			fprintf(stderr, ", %s", missing[i]);
		fprintf(stderr, "\n       for TNSPEC %s\n", bup_tnspec(bupctx));
		goto reset_and_depart;
	}
		
	/*
	 * Verify that all of the partitions we need to update are acutally present,
	 * and at the same time build the set of update tasks.
	 *
	 * For initialization, we separate the redundant entries from the non-redundant
	 * ones and write the non-redundant entries last. While the BCT appears to
	 * be non-redundant (only one BCT partition), it is internally redundant and
	 * requires special handling.
	 *
	 * For updates, we never write the non-redundant entries.
	 */
	bupiter = 0;
	redundant_entry_count = nonredundant_entry_count = 0;
	largest_length = 0;
	memset(&mb1_other, 0, sizeof(mb1_other));
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
				if (part_b != NULL || strcmp(partname, "BCT") == 0) {
					if (redundant_entry_count >= sizeof(redundant_entries)/sizeof(redundant_entries[0])) {
						fprintf(stderr, "too many partitions to initialize\n");
						ret = 1;
						goto reset_and_depart;
					}
					redundant_entries[redundant_entry_count] = updent;
					redundant_entries[redundant_entry_count].part = part;
					redundant_entry_count += 1;
					if (part_b != NULL) {
						redundant_entries[redundant_entry_count] = updent;
						strcpy(redundant_entries[redundant_entry_count].partname, partname_b);
						redundant_entries[redundant_entry_count].part = part_b;
						redundant_entry_count += 1;
					}
				} else {
					if (nonredundant_entry_count >= sizeof(nonredundant_entries)/sizeof(nonredundant_entries[0])) {
						fprintf(stderr, "too many (non-redundant) partitions to initialize\n");
						ret = 1;
						goto reset_and_depart;
					}
					nonredundant_entries[nonredundant_entry_count] = updent;
					nonredundant_entries[nonredundant_entry_count].part = part;
					nonredundant_entry_count += 1;
				}
			} else if (part_b != NULL || strcmp(partname, "BCT") == 0) {
				if (redundant_entry_count >= sizeof(redundant_entries)/sizeof(redundant_entries[0])) {
					fprintf(stderr, "too many partitions to update\n");
					ret = 1;
					goto reset_and_depart;
				}
				redundant_entries[redundant_entry_count] = updent;
				strcpy(redundant_entries[redundant_entry_count].partname,
				       (part_b == NULL || *suffix == '\0' ? partname : partname_b));
				redundant_entries[redundant_entry_count].part = (part_b == NULL || *suffix == '\0' ? part : part_b);
				/*
				 * Save the info for the other mb1 entry, in case the BCT
				 * was updated and we need to update both mb1's
				 */
				if (strcmp(partname, "mb1") == 0) {
					mb1_other = updent;
					strcpy(mb1_other.partname, (*suffix == '\0' ? partname_b : partname));
					mb1_other.part = (*suffix == '\0' ? part_b : part);
				}
				redundant_entry_count += 1;
			}
		} else {
			int redundant;
			sprintf(pathname, "/dev/disk/by-partlabel/%s", partname);
			if (access(pathname, F_OK|W_OK) != 0) {
				fprintf(stderr, "Error: cannot locate partition: %s\n", partname);
				ret = 1;
				goto reset_and_depart;
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
	slotbuf = malloc(largest_length);
	if (contentbuf == NULL || slotbuf == NULL) {
		perror("allocating content buffers");
		goto reset_and_depart;
	}
	contentbuf_size = largest_length;

	order_entries(redundant_entries, ordered_entries, redundant_entry_count);

	for (i = 0; i < redundant_entry_count; i++)
		if (process_entry(bupctx, fd, ordered_entries[i], dryrun) != 0)
			goto reset_and_depart;

	if (initialize) {
		for (i = 0; i < nonredundant_entry_count; i++)
			if (process_entry(bupctx, fd, &nonredundant_entries[i], dryrun) != 0)
				goto reset_and_depart;
	} else if (bct_updated) {
		/*
		 * If the BCT was updated, we must update both mb1 and mb1_b
		 */
		if (mb1_other.partname[0] == '\0') {
			fprintf(stderr, "Error: could not update alternate mb1 partition\n");
			goto reset_and_depart;
		}
		if (process_entry(bupctx, fd, &mb1_other, dryrun) != 0)
			goto reset_and_depart;
	}
	if (dryrun) {
		if (!slot_specified)
			printf("[skip] mark slot %d as active\n", (initialize ? 0 : 1 - curslot));
	} else if (!slot_specified) {
		unsigned int newslot = (initialize ? 0 : 1 - curslot);
		if (smd_slot_mark_active(smdctx, newslot) < 0) {
			perror("marking new boot slot active");
			goto reset_and_depart;
		}
		printf("Slot %u marked as active for next boot\n", newslot);
		if (smd_update(smdctx, gptctx, fd, initialize) < 0)
			perror("updating slot metadata");
	}

  reset_and_depart:
	if (smdctx)
		smd_finish(smdctx);
	if (fd >= 0) {
		if (!dryrun)
			fsync(fd);
		close(fd);
	}
	if (reset_bootdev)
		set_bootdev_writeable_status(bup_boot_device(bupctx), 0);
	if (slotbuf)
		free(slotbuf);
	if (contentbuf)
		free(contentbuf);

	gpt_finish(gptctx);
	bup_finish(bupctx);

	return ret;

} /* main */
