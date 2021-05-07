/*
 * util.c
 *
 * Utility functions used in multiple places.
 *
 * Copyright (c) 2021, Matthew Madison
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include "util.h"

#define MAX_PARTITIONS 256
static bool partconf_read;
static char partconf_buf[4096];
static char *partconf_list[MAX_PARTITIONS];
static unsigned int partcount;
#define QUOTE(m_) #m_
#define XQUOTE(m_) QUOTE(m_)
static const char allpartconf[] = XQUOTE(CONFIGPATH) "/all-partitions.conf";

/*
 * set_bootdev_writeable_status
 *
 * Toggles the read-only soft switch in sysfs for eMMC boot0/boot1
 * devices, if present.
 *
 * bootdev: device name
 * make_writeable: true for wrieable, false otherwise
 *
 * Returns: true if changed, false otherwise
 *
 */
bool
set_bootdev_writeable_status (const char *bootdev, bool make_writeable)
{
	char pathname[64];
	char buf[1];
	int fd, is_writeable, rc = 0;

	if (bootdev == NULL)
		return false;
	if (strlen(bootdev) < 6 || strlen(bootdev) > 32)
		return false;
	sprintf(pathname, "/sys/block/%s/force_ro", bootdev + 5);
	fd = open(pathname, O_RDWR);
	if (fd < 0)
		return false;
	if (read(fd, buf, sizeof(buf)) != sizeof(buf)) {
		close(fd);
		return false;
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
 * partition_should_be_present
 *
 * Given a partition name, checks the list of partitions in
 * /usr/share/tegra-boot-tools/all-partitions.conf to see
 * if it should be present. If that file does not exist,
 * this function will default to assuming all partitions
 * asked about should be present.
 *
 * partname: partition name
 *
 * Returns true if either no config file or if partname is
 * on the list, false otherwise.
 */
bool
partition_should_be_present (const char *partname)
{
	unsigned int i;

	if (!partconf_read) {
		FILE *fp = fopen(allpartconf, "r");
		if (fp != NULL) {
			char *name, *saveptr = NULL;
			size_t count = fread(partconf_buf, sizeof(partconf_buf[0]), sizeof(partconf_buf)/sizeof(partconf_buf[0])-1, fp);
			fclose(fp);
			while (count > 0 && isspace(partconf_buf[count-1]))
				count -= 1;
			partconf_buf[count] = '\0';
			for (name = strtok_r(partconf_buf, ",", &saveptr);
			     name != NULL && partcount < MAX_PARTITIONS;
			     name = strtok_r(NULL, ",", &saveptr))
				partconf_list[partcount++] = name;
		} else
			fprintf(stderr, "Warning: could not open %s\n", allpartconf);
		partconf_read = true;
	}

	if (partcount == 0)
		return true;
	for (i = 0; i < partcount; i++)
		if (strcmp(partname, partconf_list[i]) == 0)
			return true;
	return false;

} /* partition_should_be_present */
