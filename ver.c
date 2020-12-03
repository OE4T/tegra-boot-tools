/*
 * ver.c
 *
 * Functions for parsing and validating the
 * version information in a VER partition or
 * BUP entry.
 *
 * Copyright (c) 2020, Matthew Madison
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <ctype.h>
#include "ver.h"
#include "posix-crc32.h"

/*
 * extract_line
 *
 * Extracts a linefeed-terminated string from `buf`,
 * copying the contents to `outbuf`, with the linefeed
 * removed and a null terminator added.
 *
 * Parameters:
 *
 *  buf: pointer to buffer to search
 *  bufsiz: address of length of buf (will be updated on success)
 *  outbuf: pointer to output buffer
 *  outsiz: size of output buffer
 *
 * Returns:
 *    non-NULL: pointer to next char after linefeed, bufsiz updated
 *              with remaining length of buffer
 *    NULL: No linefeed found (errno set to EINVAL), or
 *          Output buffer not large enough (errno set to EMSGSIZE)
 */
static char *
extract_line (char *buf, size_t *bufsiz, char *outbuf, size_t outsiz)
{
	char *cp;
	size_t len;

	cp = memchr(buf, '\n', *bufsiz);
	if (cp == NULL) {
		errno = EINVAL;
		return NULL;
	}
	len = cp - buf;
	if (len >= outsiz) {
		errno = EMSGSIZE;
		return NULL;
	}
	memcpy(outbuf, buf, len);
	outbuf[len] = '\0';
	*bufsiz -= len + 1; // include the linefeed
	return cp + 1;

} /* extract_line */

/*
 * parse_bsp_version
 *
 * Parses the BSP version line.
 *
 * Parameters:
 *   line: pointer to string to parse
 *   verp: address of unsigned int to hold the packed version number
 *
 * Returns:
 *   0: success
 *   1: error (errno set)
 *
 */
static int
parse_bsp_version (const char *line, unsigned int *verp)
{
	char *anchor;
	unsigned long major, minor, maint;
	size_t remain;

	if (*line != '#' || line[1] != ' ' || line[2] != 'R') {
		errno = EINVAL;
		return 1;
	}
	major = strtoul(&line[3], &anchor, 10);
	if (major == ULONG_MAX)
		return 1;
	remain = strlen(anchor);
	if (remain < 14 || memcmp(anchor, " , REVISION: ", 13) != 0) {
		errno = EINVAL;
		return 1;
	}
	minor = strtoul(anchor + 13, &anchor, 10);
	if (minor == ULONG_MAX)
		return 1;
	if (*anchor != '.') {
		errno = EINVAL;
		return 1;
	}
	maint = strtoul(anchor + 1, NULL, 10);
	if (maint == ULONG_MAX)
		return 1;
	*verp = make_bsp_version(major, minor, maint);

	return 0;

} /* parse_bsp_version */

/*
 * parse_datetime
 *
 * Parses the YYYMMDDHHMMSS date/time stamp
 * into a struct tm.
 *
 * Parameters:
 *   buf: input string buffer
 *   tm: pointer to struct tm
 *
 * Returns:
 *        0: success
 * non-zero: error
 */
static int
parse_datetime (const char *buf, struct tm *tm)
{
	const char *cp;

	if (strlen(buf) != 14) {
		errno = EINVAL;
		return -1;
	}
	
	for (cp = buf; *cp != '\0' && isdigit(*cp); cp++);
	if (*cp != '\0') {
		errno = EINVAL;
		return -1;
	}

	memset(tm, 0, sizeof(*tm));
	tm->tm_year = (buf[0]-'0') * 1000 + (buf[1]-'0') * 100 + (buf[2]-'0') * 10 + (buf[3]-'0');
	tm->tm_mon = (buf[4]-'0') * 10 + (buf[5]-'0');
	tm->tm_mday = (buf[6]-'0') * 10 + (buf[7]-'0');
	tm->tm_hour = (buf[8]-'0') * 10 + (buf[9]-'0');
	tm->tm_min = (buf[10]-'0') * 10 + (buf[11]-'0');
	tm->tm_sec = (buf[12]-'0') * 10 + (buf[12]-'0');

	return 0;

} /* parse_datetime */

/*
 * parse_sizecrc_and_validate
 *
 * Parses the line with the size and CRC of the stored version
 * information and validates it against the actual data read.
 *
 * Parameters:
 *   line: BYTES/CRC32 line contents
 *   buf:  pointer to start of buffer with VER info
 *   bufsiz: size of buffer in bytes
 *
 * Returns:
 *   0:    success, size and CRC match
 *   1:    error, errno set to
 *           EINVAL: could not parse the line
 *           ERANGE: from strtoul, error parsing a number
 *           EMSGSIZE: sizes did not match
 *           EPROTO: CRCs did not match
 *
 */
static int
parse_sizecrc_and_validate (const char *line, void *buf, size_t bufsiz, uint32_t *crcp)
{
	char *anchor;
	size_t len;
	unsigned long bytes, crc;
	uint32_t actual_crc;

	len = strlen(line);
	if (len <= 6 || memcmp(line, "BYTES:", 6) != 0) {
		errno = EINVAL;
		return 1;
	}
	bytes = strtoul(line + 6, &anchor, 10);
	if (bytes == ULONG_MAX)
		return 1;
	if (bufsiz != (size_t) bytes) {
		errno = EMSGSIZE;
		return 1;
	}
	len = strlen(anchor);
	if (len <= 7 || memcmp(anchor, " CRC32:", 7) != 0) {
		errno = EINVAL;
		return 1;
	}
	crc = strtoul(anchor + 7, NULL, 10);
	if (crc == ULONG_MAX)
		return 1;
	actual_crc = posix_crc32(buf, bufsiz);
	if (actual_crc != (uint32_t) crc) {
		errno = EPROTO;
		return 1;
	}
	*crcp = actual_crc;
	return 0;

} /* parse_sizecrc_and_validate */

/*
 * ver_extract_info
 *
 * Parses the version information in buf, validating
 * the contents along the way.
 *
 * A version block is a series of text lines:
 *
 * NVn                               <- version information revision tag
 * # Rxx , REVISION: y.z             <- BSP version information
 * BOARID=xxxx BOARDSKU=xxxx ...     <- product information
 * YYYYMMDDHHMMSS                    <- date/time stamp
 * BYTES: nn CRC32:cksum             <- length and checksum
 *
 * Revisions:
 *  NV1 - only includes the first two lines
 *  NV2 - only includes the first three lines
 *  NV3 - includes all of the lines, checksum is calculated on the text
 *        up to (but not including) the last line
 *
 */
int
ver_extract_info (void *buf, size_t bufsiz, ver_info_t *ver)
{
	char *anchor = buf;
	char linebuf[1024];
	size_t infosize;

	memset(ver, 0, sizeof(*ver));

	anchor = extract_line(buf, &bufsiz, linebuf, sizeof(linebuf));
	if (anchor == NULL)
		return -1;
	if (linebuf[0] != 'N' || linebuf[1] != 'V' || !isdigit(linebuf[2])) {
		errno = EINVAL;
		return -1;
	}

	ver->ver_info_revision = linebuf[2] - '0';

	anchor = extract_line(anchor, &bufsiz, linebuf, sizeof(linebuf));
	if (anchor == NULL)
		return -1;
	if (parse_bsp_version(linebuf, &ver->bsp_version) != 0)
		return -1;

	if (ver->ver_info_revision < 2)
		return 0;
	anchor = extract_line(anchor, &bufsiz, ver->product_spec, sizeof(ver->product_spec));
	if (anchor == NULL)
		return -1;

	if (ver->ver_info_revision < 3)
		return 0;
	anchor = extract_line(anchor, &bufsiz, linebuf, sizeof(linebuf));
	if (anchor == NULL)
		return -1;
	if (parse_datetime(linebuf, &ver->ver_dtstamp) != 0)
		return -1;

	infosize = anchor - (char *) buf;
	anchor = extract_line(anchor, &bufsiz, linebuf, sizeof(linebuf));
	if (anchor == NULL)
		return -1;
	if (parse_sizecrc_and_validate(linebuf, buf, infosize, &ver->crc) != 0)
		return -1;

	return 0;

} /* ver_extract_info */
