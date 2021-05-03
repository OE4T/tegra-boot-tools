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
#include "util.h"

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
