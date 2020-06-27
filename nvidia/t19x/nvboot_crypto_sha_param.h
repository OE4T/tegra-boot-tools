/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H
#define INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H

#include "nvcommon.h"
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * SHA256 Hash of the empty string; Needed for T210_B01 SE WAR. See http://nvbugs/1788437.
 */
// Removed (unused)  If needed move this to a .c file so it doesn't get instantiated multiple times.
//static const NvU8 Sha256NullStringDigest[] =
//{
//    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
//    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
//};

typedef enum NvBootCryptoSha2AlgorithmRec
{
    // Algorithm = Digest Size in bits
    SHA2_256 = 256,

    SHA2_INVALID,
} NvBootCryptoSha2Algorithm;

typedef enum NvBootCryptoShaDigestSizeRec
{
    SHA_256 = 256,
    SHA_384 = 384,
    SHA_512 = 512,

    SHA_INVALID_DIGEST,
} NvBootCryptoShaDigestSize;

typedef enum NvBootCryptoShaFamilyRec
{
    SHA2 = 2,
    SHA3,

    SHA_INVALID_FAMILY,
} NvBootCryptoShaFamily;

typedef struct NvBootCryptoShaConfigRec
{
    NvBootCryptoShaFamily ShaFamily;
    NvBootCryptoShaDigestSize ShaDigestSize;
} NvBootCryptoShaConfig;


/**
 * Defines the length of a SHA256 hash in bytes
 *
 */
enum {NVBOOT_SHA256_LENGTH_BYTES = SHA_256 / 8};

/**
 * Defines the length of a SHA256 hash in words
 *
 */
enum {NVBOOT_SHA256_LENGTH_WORDS = NVBOOT_SHA256_LENGTH_BYTES / 4};

enum {NVBOOT_SHA2_MAX_BYTE_SIZE = SHA_512 / 8};

typedef struct NvBootSha256HashDigestRec
{
    NvU8 Hash[NVBOOT_SHA256_LENGTH_BYTES];
} DECLARE_ALIGNED(NvBootSha256HashDigest, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

#if defined(__cplusplus)
}
#endif

#endif /* INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H */
