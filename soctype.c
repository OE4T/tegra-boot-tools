/*
 * soctype.c
 *
 * SoC identification functions.
 *
 * Copyright (c) 2020 Matthew Madison
 */
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include "soctype.h"

/*
 * soctype_valid
 */
tegra_soctype_t
soctype_from_chipid (unsigned int chipid)
{
	switch (chipid) {
	case 0x18:
		return TEGRA_SOCTYPE_186;
	case 0x19:
		return TEGRA_SOCTYPE_194;
	case 0x21:
		return TEGRA_SOCTYPE_210;
	default:
		break;
	}
	return TEGRA_SOCTYPE_INVALID;

} /* return soctype_from_chipid */

/*
 * soctype_get
 */
tegra_soctype_t
soctype_get (void) {
	ssize_t typelen;
	int fd;
	char soctype[65];

	fd = open("/sys/module/tegra_fuse/parameters/tegra_chip_id", O_RDONLY);
	if (fd < 0)
		return TEGRA_SOCTYPE_INVALID;
	typelen = read(fd, soctype, sizeof(soctype)-1);
	close(fd);
	if (typelen < 0)
		return TEGRA_SOCTYPE_INVALID;
	while (typelen > 0 && soctype[typelen-1] == '\n') typelen--;

	soctype[typelen] = '\0';

	return soctype_from_chipid(strtoul(soctype, NULL, 10));

} /* soctype_get */

/*
 * soctype_name
 */
const char *
soctype_name (tegra_soctype_t soctype)
{
	switch (soctype) {
	case TEGRA_SOCTYPE_186:
		return "Tegra186";
	case TEGRA_SOCTYPE_194:
		return "Tegra194";
	case TEGRA_SOCTYPE_210:
		return "Tegra210";
	default:
		break;
	}
	return "INVALID";

} /* soctype_name */
