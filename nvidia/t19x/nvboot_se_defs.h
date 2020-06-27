/*
 * Copyright (c) 2006-2011, NVIDIA CORPORATION. All rights reserved.
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

/**
 * @file nvboot_se_defs.h
 */

#ifndef INCLUDED_NVBOOT_SE_DEFS_H
#define INCLUDED_NVBOOT_SE_DEFS_H

#define ARSE_SHA1_HASH_SIZE     160
#define ARSE_SHA224_HASH_SIZE   224
#define ARSE_SHA256_HASH_SIZE   256
#define ARSE_SHA384_HASH_SIZE   384
#define ARSE_SHA512_HASH_SIZE   512
#define ARSE_SHA_HASH_SIZE      512
#define ARSE_AES_HASH_SIZE      128
#define ARSE_HASH_SIZE  512
// #define ARSE_TZRAM_BYTE_SIZE    16384
#define ARSE_SHA_MSG_LENGTH_SIZE        128
#define ARSE_RSA_MAX_EXPONENT_SIZE      2048
#define ARSE_RSA_MAX_MODULUS_SIZE       2048
#define ARSE_RSA_MAX_INPUT_SIZE 2048
#define ARSE_RSA_MAX_OUTPUT_SIZE        2048
#define ARSE_RSA_MIN_EXPONENT_SIZE      32
#define ARSE_RSA_MIN_MODULUS_SIZE       512
#define ARSE_RSA_MIN_INPUT_SIZE 512
#define ARSE_RSA_MIN_OUTPUT_SIZE        512

#define SE_RSA_KEY_PKT_KEY_SLOT_ONE                     _MK_ENUM_CONST(0)
#define SE_RSA_KEY_PKT_KEY_SLOT_TWO                     _MK_ENUM_CONST(1)
/*
 * Defines the maximum link list buffer size in log2 bytes
 */
enum {NVBOOT_SE_LL_MAX_BUFFER_LENGTH_LOG2 = 24};

/*
 * Defines the maximum link list buffer size in bytes
 *
 * An RTL bug exists (http://nvbugs/917474) in T114 which doesn't allow
 * the specifying a buffer byte size of 0 indicating the maximum
 * buffer size of 16MB (2^24 bits). To workaround this, specify the
 * maximum buffer byte size to be a multiple of the largest SHA block
 * size of 1024-bits by reducing the max buffer size by 1024-bits.
 */
enum {NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES = (1 << NVBOOT_SE_LL_MAX_BUFFER_LENGTH_LOG2) - 0x400};

/**
 * Defines the maximum link list buffer size in words
 */
enum {NVBOOT_SE_LL_MAX_BUFFER_SIZE_WORDS = NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES / 4};

/*
 * Defines the maximum number of buffers in a linked list
 * handled by the Boot ROM at one time. This limitation is artificial and
 * mostly is to keep the amount of IRAM space used to store the linked list
 * to a minimum. In the worst case, BR has to process a boot image of size
 * equal to the amount of DRAM. To wholly specify an input of 2GB (assuming the
 * maximum size of DRAM is 2GB), requires 128 linked list buffers (16MB
 * buffer size * 128 = 2GB). 128 linked list buffers would require 257 words
 * of storage (2 words for each buffer, plus the last buffer number), or 1028
 * bytes. By limiting the BR to only 16 linked list buffers (meaning that BR
 * can tell SE to process 256MB of data in one operation), we only need
 * reserve 132 bytes in IRAM. The BR will split any SE operations into multiple
 * chunks of 256MB. This number can be tuned if necessary up to the maximum
 * amount of contiguous IRAM space available.
 */
enum {NVBOOT_SE_LL_MAX_NUM_BUFFERS = 16};

/*
 * Defines the maximum number of bytes addressed by the largest possible
 * linked list handled by the Boot ROM.
 */
enum {NVBOOT_SE_LL_MAX_SIZE_BYTES = NVBOOT_SE_LL_MAX_NUM_BUFFERS * NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES};

/*
 * Security Engine operation modes.
 */
typedef enum
{
    SE_OP_MODE_AES_CMAC_HASH,
    SE_OP_MODE_AES_CBC,
    SE_OP_MODE_AES_ECB,

    // Specifies the number of modes.
    SE_OP_MODE_MAX,
    SE_OP_MODE_FORCE32 = 0x7FFFFFFF
} NvBootSeOperationMode;

#endif //INCLUDED_NVBOOT_SE_DEFS_H

