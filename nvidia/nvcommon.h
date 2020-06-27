#ifndef nvcommon_h__
#define nvcommon_h__
/*
 * Copyright (c) 2020, Matthew Madison
 *
 * Definitions needed by NVIDIA headers.
 */
#include <inttypes.h>
#include <stdint.h>
#include <stddef.h>

typedef uint8_t		NvU8;
typedef uint16_t	NvU16;
typedef uint32_t	NvU32;
typedef uint64_t	NvU64;

typedef int8_t		NvS8;
typedef int16_t		NvS16;
typedef int32_t		NvS32;
typedef int64_t		NvS64;

typedef NvU8		NvBool;

typedef uintptr_t	NvUPtr;
typedef intptr_t	NvSPtr;

#endif /* nvcommon_h__ */
