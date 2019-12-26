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

static const char DEVICE_MAGIC[8] = {'B', 'O', 'O', 'T', 'I', 'N', 'F', 'O'};
#define DEVICE_MAGIC_SIZE sizeof(DEVICE_MAGIC)

static const uint16_t DEVINFO_VERSION = 1;

#define MAX_BOOT_FAILURES 3

struct device_info {
	unsigned char magic[DEVICE_MAGIC_SIZE];
	uint16_t devinfo_version;
	uint8_t  boot_in_progress;
	uint8_t  failed_boots;
};

/*
 * We stash the info at the end of mmcblk0boot1, just
 * in front of the pseudo-GPT that NVIDIA puts at the
 * very end.
 */
static const off_t devinfo_offset = -((36 + 1) * 512);

static const char devinfo_dev[] = "/dev/mmcblk0boot1";

static struct option options[] = {
	{ "boot-success",	no_argument,		0, 'b' },
	{ "check-status",	no_argument,		0, 'c' },
	{ "initialize",		no_argument,		0, 'I' },
	{ "show",		no_argument,		0, 's' },
	{ "help",		no_argument,		0, 'h' },
	{ 0,			0,			0, 0   }
};
static const char *shortopts = ":bcIsh";

static char *optarghelp[] = {
	"--boot-success       ",
	"--check-status       ",
	"--initialize         ",
	"--show               ",
	"--help               ",
};

static char *opthelp[] = {
	"update boot info to record successful boot",
	"increment boot counter and check it is under limit",
	"initialize the device info area",
	"show boot counter information",
	"display this help text"
};

/*
 * set_bootdev_writeable_status
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
		write(fd, "0", 1);
	else if (!make_writeable && is_writeable)
		write(fd, "1", 1);
	close(fd);

	return make_writeable != is_writeable;

} /* set_bootdev_writeable_status */

static int
find_bootinfo (int *fdptr, struct device_info *devinfop, int readonly)
{
	struct device_info devinfo;
	ssize_t n;
	int fd, i;

	if (devinfop == NULL)
		devinfop = &devinfo;

	fd = open(devinfo_dev, (readonly ? O_RDONLY : O_RDWR|O_DSYNC));
	if (fd < 0) {
		perror(devinfo_dev);
		return -2;
	}
	if (lseek(fd, devinfo_offset, SEEK_END) < 0) {
		perror(devinfo_dev);
		close(fd);
		return -1;
	}
	n = read(fd, devinfop, sizeof(devinfo));
	if (n < sizeof(devinfo)) {
		close(fd);
		return -1;
	}
	if (memcmp(devinfop->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE) != 0) {
		close(fd);
		return -1;
	}
	if (fdptr == NULL) {
		close(fd);
		return 0;
	}
	/* rewind back to the start of the devinfo */
	if (lseek(fd, devinfo_offset, SEEK_END) < 0) {
		close(fd);
		return -1;
	}
	*fdptr = fd;
	return 0;
}

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
	struct device_info devinfo;
	unsigned long which = 0;
	int fd;

	set_bootdev_writeable_status(1);
	if (find_bootinfo(&fd, &devinfo, 0) == 0) {
		fprintf(stderr, "Device info already initialized\n");
		close(fd);
		set_bootdev_writeable_status(0);
		return 0;
	}
	fd = open(devinfo_dev, O_RDWR|O_DSYNC);
	if (fd < 0) {
		perror(devinfo_dev);
		set_bootdev_writeable_status(0);
		return -2;
	}
	if (lseek(fd, devinfo_offset, SEEK_END) < 0) {
		perror(devinfo_dev);
		close(fd);
		set_bootdev_writeable_status(0);
		return -1;
	}
	memset(&devinfo, 0, sizeof(devinfo));
	memcpy(devinfo.magic, DEVICE_MAGIC, sizeof(devinfo.magic));
	devinfo.devinfo_version = DEVINFO_VERSION;
	if (write(fd, &devinfo, sizeof(devinfo)) < 0) {
		perror("writing device info");
		close(fd);
		set_bootdev_writeable_status(0);
		return -2;
	}
	close(fd);
	set_bootdev_writeable_status(1);
	return 0;

} /* boot_devinfo_init */

static int
boot_successful(void)
{
	struct device_info devinfo;
	int fd;

	set_bootdev_writeable_status(1);
	if (find_bootinfo(&fd, &devinfo, 0) < 0) {
		fprintf(stderr, "Could not locate device info, initializing\n");
		if (boot_devinfo_init() < 0) {
			set_bootdev_writeable_status(0);
			return -2;
		}
		if (find_bootinfo(&fd, &devinfo, 0) < 0) {
			set_bootdev_writeable_status(0);
			return -2;
		}
	}

	devinfo.boot_in_progress = 0;
	if (devinfo.failed_boots > 0)
		fprintf(stderr, "Failed boot count: %u\n", devinfo.failed_boots);
	devinfo.failed_boots = 0;
	if (write(fd, &devinfo, sizeof(devinfo)) < 0) {
		perror("writing boot info");
		close(fd);
		set_bootdev_writeable_status(0);
		return -2;
	}
	close(fd);
	set_bootdev_writeable_status(0);
	return 0;

} /* boot_successful */

static int
boot_check_status(void)
{
	struct device_info devinfo;
	int fd, rc = 0;

	set_bootdev_writeable_status(1);
	if (find_bootinfo(&fd, &devinfo, 0) < 0) {
		fprintf(stderr, "Could not locate device info, initializing\n");
		rc = boot_devinfo_init();
		if (rc == 0)
			rc = find_bootinfo(&fd, &devinfo, 0);
		if (rc < 0) {
			set_bootdev_writeable_status(0);
			return rc;
		}
	}

	if (devinfo.boot_in_progress) {
		devinfo.failed_boots += 1;
		fprintf(stderr, "Boot failures: %u\n", devinfo.failed_boots);
		if (devinfo.failed_boots >= MAX_BOOT_FAILURES) {
			fprintf(stderr, "Too many boot failures, exit with error to signal boot slot switch\n");
			rc = 77;
			devinfo.boot_in_progress = 0;
			devinfo.failed_boots = 0;
		}
	} else
		devinfo.boot_in_progress = 1;
	if (write(fd, &devinfo, sizeof(devinfo)) < 0) {
		perror("writing boot info");
		rc = -2;
	}
	close(fd);
	set_bootdev_writeable_status(0);
	return rc;

} /* boot_check_status */

static int
show_bootinfo(void) {
	struct device_info devinfo;

	if (find_bootinfo(NULL, &devinfo, 1) < 0) {
		fprintf(stderr, "Could not locate device info\n");
		return -2;
	}
	printf("devinfo version:        %u\n"
	       "Boot in progress:       %s\n"
	       "Failed boots:           %d\n",
	       devinfo.devinfo_version,
	       devinfo.boot_in_progress ? "YES" : "NO",
	       devinfo.failed_boots);
	return 0;
}

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
	default:
		fprintf(stderr, "Error: unrecognized option\n");
		print_usage();
		return 1;
	}

} /* main */
