/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
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

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined (_MSC_VER)
#define DECLARE_ALIGNED(var, n) __declspec(align(n)) var
#elif defined __GNUC__
#define DECLARE_ALIGNED(var, n) var __attribute__((aligned(n)))
#endif

/**
 * Defines the length of a SHA256 hash in bytes
 *
 */
enum {NVBOOT_SHA256_LENGTH_BYTES = 256 / 8};

/**
 * Defines the length of a SHA256 hash in words
 *
 */
enum {NVBOOT_SHA256_LENGTH_WORDS = NVBOOT_SHA256_LENGTH_BYTES / 4};

enum {NVBOOT_SHA2_MAX_BYTE_SIZE = 512 /8};

typedef struct NvBootSha256HashDigestRec
{
    NvU8 Hash[NVBOOT_SHA256_LENGTH_BYTES];
} DECLARE_ALIGNED(NvBootSha256HashDigest , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

#if defined(__cplusplus)
}
#endif

#endif /* INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H */
