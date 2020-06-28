/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
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
 * \file nvboot_crypto_aes_param.h
 *
 * Defines the parameters and data structures for AES.
 *
 * AES is used for encryption, decryption, and signatures.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_AES_H
#define INCLUDED_NVBOOT_CRYPTO_AES_H

#include "nvcommon.h"
#include "nvboot_config.h"

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
 * NEW AES ENUMS HERE!!!!!!!!!!!!!
 */
typedef enum
{
    AesKey128,
    AesKey192,
    AesKey256,
} NvBootAesKeySize;

enum {NVBOOT_AES_BLOCK_LENGTH_BYTES = 16};

enum {NVBOOT_AES_BLOCK_LENGTH_WORDS = NVBOOT_AES_BLOCK_LENGTH_BYTES / 4};

/**
 * Defines the length of an AES block in units of log2 bytes.
 */
enum {NVBOOT_AES_BLOCK_LENGTH_LOG2 = 4};

enum {NVBOOT_AES_IV_LENGTH_BYTES = 16};

enum {NVBOOT_AES_IV_LENGTH_WORDS = NVBOOT_AES_IV_LENGTH_BYTES / 4};

enum {NVBOOT_AES_KEY_128_BYTES = 16};

enum {NVBOOT_AES_KEY_192_BYTES = 24};

enum {NVBOOT_AES_KEY_256_BYTES = 32};

enum {NVBOOT_AES_KEY_MAX_BYTES = NVBOOT_AES_KEY_256_BYTES};

enum {NVBOOT_SECURE_BOOT_KEY_LENGTH_BYTES = NVBOOT_AES_KEY_128_BYTES};

enum {NVBOOT_AES128_KEYSCHED_BYTES = 176};
enum {NVBOOT_AES192_KEYSCHED_BYTES = 208};
enum {NVBOOT_AES256_KEYSCHED_BYTES = 240};

enum {NVBOOT_AES_MAX_KEYSCHED_BYTES = NVBOOT_AES256_KEYSCHED_BYTES};

#define MAX_KEYSCHED_BYTES (AES256_KEYSCHED_BYTES)

/**
 * Defines the value of const_Rb, as specified in the AES-CMAC specification.
 * See RFC 4493: http://tools.ietf.org/html/rfc4493.
 */
enum {NVBOOT_AES_CMAC_CONST_RB = 0x87};

/**
 * NEW AES ENUMS END
 */

typedef enum
{
    UserSpecified,
    SBK,
    AesInputKeyType_Num,
    AesInputKeyType_Force32 = 0x7fffffff,
} NvBootAesInputKeyType;

typedef enum
{
    ECB,
    CBC,
    CTR,
    AesBlockMode_Num,
    AesBlockMode_Force32 = 0x7fffffff,
} NvBootAesBlockCipherMode;


/**
 * Defines an 128-bit AES key.
 */
typedef struct NvBootAesKey128Rec
{
    /// Specifies the key data.
    NvU8 Key128[NVBOOT_AES_KEY_128_BYTES];
} NvBootAesKey128;

/**
 * Defines an 192-bit AES key.
 */
typedef struct NvBootAesKey192Rec
{
    /// Specifies the key data.
    NvU8 Key192[NVBOOT_AES_KEY_192_BYTES];
} NvBootAesKey192;

/**
 * Defines an 256-bit AES key.
 */
typedef struct NvBootAesKey256Rec
{
    /// Specifies the key data.
    NvU8 Key256[NVBOOT_AES_KEY_256_BYTES];
} NvBootAesKey256;
/**
 * Defines an AES Initial Vector (128-bits).
 */
typedef struct NvBootAesIvRec
{
    /// Specifies the initial vector data.
    NvU8 Iv[NVBOOT_AES_IV_LENGTH_BYTES];
} NvBootAesIv;

/**
 * Defines an AES key schedule buffer.
 */
typedef struct NvBootAesKeyScheduleRec
{
    NvU8 KeySchedule[NVBOOT_AES_MAX_KEYSCHED_BYTES];
} NvBootAesKeySchedule;

typedef union
{
    NvU8 Key128[NVBOOT_AES_KEY_128_BYTES];
    NvU8 Key192[NVBOOT_AES_KEY_192_BYTES];
    NvU8 Key256[NVBOOT_AES_KEY_256_BYTES];
    DECLARE_ALIGNED(NvU8 KeyMax[NVBOOT_AES_KEY_MAX_BYTES] , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} NvBootAesKey;

typedef struct NvBootAesCmacHashParamsRec
{
    DECLARE_ALIGNED(NvU8 K1[NVBOOT_AES_BLOCK_LENGTH_BYTES], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(NvU8 K2[NVBOOT_AES_BLOCK_LENGTH_BYTES] , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootAesCmacHashParams , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef struct NvBootAesParamsRec
{
    //NvBootCryptoMgrAesCryptoStatus Status;
    NvBootAesBlockCipherMode AesBlockMode;
    DECLARE_ALIGNED(NvBootAesKey Key , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    NvBootAesKeySize KeySize;
    DECLARE_ALIGNED(NvBootAesIv Iv , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(NvBootAesKeySchedule KeySchedule , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    NvBootAesCmacHashParams AesCmacParams;
    // PayloadSizeBytes must be multiple of an AES block.
    NvU32 PayloadSizeBytes;
} DECLARE_ALIGNED(NvBootAesParams , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

#if 0
typedef struct NvBootAesParamsRec
{
    //NvBootCryptoMgrAesCryptoStatus Status;
    NvBootAesBlockCipherMode AesBlockMode;
    NvBootAesKey Key __attribute__((aligned(4)));
    NvBootAesKeySize KeySize;
    NvBootAesIv Iv __attribute__((aligned(4)));
    NvBootAesKeySchedule KeySchedule __attribute__((aligned(4)));
    NvBootAesCmacHashParams AesCmacParams;
    // PayloadSizeBytes must be multiple of an AES block.
    NvU32 PayloadSizeBytes;
} NvBootAesParams;
#endif


// Struct for storage of an AES-CMAC hash.
typedef struct NvBootAesCmacHashRec
{
    // Hash is the calculated Message Authentication Code (MAC).
    DECLARE_ALIGNED(NvU8 Hash[NVBOOT_AES_BLOCK_LENGTH_BYTES] , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootAesCmacHash , NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

/*typedef struct AesCmacContextRec
{
} AesCmacContext;
*/

/**
 * Define a macro to check if the MSB of the input is set.
 */
#define NVBOOT_AES_CMAC_IS_MSB_SET(x)  ( (x) >> (8*sizeof(x)-1) )

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_CRYPTO_AES_H
