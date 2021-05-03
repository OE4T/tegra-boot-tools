#ifndef posix_crc32_h_included
#define posix_crc32_h_included
/* Copyright (c) 2020, Matthew Madison */

#include <stddef.h>
#include <stdint.h>

uint32_t posix_crc32(void *buf, size_t bufsiz);

#endif /* posix_crc32_h_included */
