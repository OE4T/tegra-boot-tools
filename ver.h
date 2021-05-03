#ifndef ver_h_included
#define ver_h_included
/* Copyright (c) 2020, Matthew Madison */

#include <time.h>
#include <stdint.h>
#include <stddef.h>

#define VER_PRODUCT_MAXSIZE 256
struct ver_info_s {
	unsigned int ver_info_revision;
	unsigned int bsp_version;
	uint32_t crc;
	struct tm ver_dtstamp;
	char product_spec[VER_PRODUCT_MAXSIZE];
};
typedef struct ver_info_s ver_info_t;

static inline __attribute__((unused)) unsigned int bsp_version_major(unsigned int bv) {
	return (bv >> 16) & 0xffff;
}
static inline __attribute__((unused)) unsigned int bsp_version_minor(unsigned int bv) {
	return (bv >> 8) & 0xff;
}
static inline __attribute__((unused)) unsigned int bsp_version_maint(unsigned int bv) {
	return bv & 0xff;
}
static inline __attribute__((unused)) unsigned int make_bsp_version(unsigned int major, unsigned int minor, unsigned int maint) {
	return ((major & 0xffff) << 16) | ((minor & 0xff) << 8) | (maint & 0xff);
}

int ver_extract_info (void *buf, size_t bufsiz, ver_info_t *ver);

#endif /* ver_h_included */
