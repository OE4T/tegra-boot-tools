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

#include "arse0.h"

#if defined (_MSC_VER)
#define DECLARE_ALIGNED(var, n) __declspec(align(n)) var
#elif defined __GNUC__
#define DECLARE_ALIGNED(var, n) var __attribute__((aligned(n)))
#endif

typedef enum
{
    RsaKey1024,
    RsaKey2048,
} NvBootRsaKeySize;

//TODO: change to NVBOOT_CRYPTO_*
enum {NVBOOT_RSA_1024_MODULUS_SIZE_BYTES = 1024 / 8};

enum {NVBOOT_RSA_1024_EXPONENT_SIZE_BYTES = 1024 / 8};

enum {NVBOOT_RSA_1024_MODULUS_SIZE_WORDS = 1024 / 8 / 4};

enum {NVBOOT_RSA_1024_EXPONENT_SIZE_WORDS = 1024 / 8 / 4};

enum {NVBOOT_RSA_2048_MODULUS_SIZE_BYTES = 2048 / 8};

enum {NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES = 2048 / 8};

enum {NVBOOT_RSA_2048_MODULUS_SIZE_WORDS = 2048 / 8 / 4};

enum {NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS = 2048 / 8 / 4};

enum {NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES = NVBOOT_RSA_2048_MODULUS_SIZE_BYTES};

enum {NVBOOT_RSA_MAX_EXPONENT_SIZE_BYTES = NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES};

enum {NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS = NVBOOT_RSA_2048_MODULUS_SIZE_WORDS};

enum {NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS = NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS};

enum {NVBOOT_RSA_MAX_MODULUS_SIZE_BITS = NVBOOT_RSA_2048_MODULUS_SIZE_WORDS * 8 * 4};

enum {NVBOOT_RSA_MAX_EXPONENT_SIZE_BITS = NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS * 8 * 4};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SIZE_BYTES = NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SIZE_WORDS = NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_HASH_ALGORITHM = SE_MODE_PKT_SHAMODE_SHA256};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SLEN = ARSE_SHA256_HASH_SIZE / 8};

enum {NVBOOT_RSA_DEFAULT_PUBLIC_EXPONENT = 0x10001};

/**
 * Defines the length of the RSASSA-PSS salt length (sLen) to use for RSASSA-PSS
 *  signature verifications
 */
enum {NVBOOT_RSA_PSS_SALT_LENGTH_BITS = ARSE_SHA256_HASH_SIZE};
enum {NVBOOT_RSA_PSS_SALT_LENGTH_BYTES = NVBOOT_RSA_PSS_SALT_LENGTH_BITS / 8};

typedef struct NvBootCryptoRsaKey1024Rec
{
    // The modulus size is 1024-bits.
    DECLARE_ALIGNED(NvU8 Modulus[NVBOOT_RSA_1024_MODULUS_SIZE_BYTES] , 4);
    // The exponent size is 1024-bits.
    DECLARE_ALIGNED(NvU8 Exponent[NVBOOT_RSA_1024_EXPONENT_SIZE_BYTES] , 4);
} NvBootCryptoRsaKey1024NvU8;

typedef struct NvBootCryptoRsaKey1024NvU32Rec
{
    // The modulus size is 1024-bits.
    DECLARE_ALIGNED(NvU32 Modulus[NVBOOT_RSA_1024_MODULUS_SIZE_WORDS] , 4);
    // The exponent size is 1024-bits.
    DECLARE_ALIGNED(NvU32 Exponent[NVBOOT_RSA_1024_EXPONENT_SIZE_WORDS] , 4);
} NvBootCryptoRsaKey1024NvU32;

/*
 *  Defines the storage for an 2048-bit RSA key to be used by the SE.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKey2048NvU8Rec
{
    // The modulus size is 2048-bits.
    DECLARE_ALIGNED(NvU8 Modulus[NVBOOT_RSA_2048_MODULUS_SIZE_BYTES] , 4);
    // The exponent size is 2048-bits.
    DECLARE_ALIGNED(NvU8 Exponent[NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES] , 4);
} NvBootCryptoRsaKey2048NvU8;

/*
 *  Defines the storage for an 2048-bit RSA key to be used by the BR.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKey2048NvU32Rec
{
    // The modulus size is 2048-bits.
    DECLARE_ALIGNED(NvU32 Modulus[NVBOOT_RSA_2048_MODULUS_SIZE_WORDS] , 4);
    // The exponent size is 2048-bits.
    DECLARE_ALIGNED(NvU32 Exponent[NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS] , 4);
} NvBootCryptoRsaKey2048NvU32;

/*
 *  Defines the storage for the largest RSA key to be used by the BR, currently 2048-bits.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeNvU8Rec
{
    // The modulus size is 2048-bits.
    DECLARE_ALIGNED(NvU8 Modulus[NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES] , 4);
    // The exponent size is 2048-bits.
    DECLARE_ALIGNED(NvU8 Exponent[NVBOOT_RSA_MAX_EXPONENT_SIZE_BYTES] , 4);
} NvBootCryptoRsaKeyMaxSizeNvU8;

/*
 *  Defines the storage for the largest RSA key to be used by the BR, currently 2048-bits.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeNvU32Rec
{
    // The modulus size is 2048-bits.
    DECLARE_ALIGNED(NvU32 Modulus[NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS] , 4);
    // The exponent size is 2048-bits.
    DECLARE_ALIGNED(NvU32 Exponent[NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS] , 4);
} NvBootCryptoRsaKeyMaxSizeNvU32;


typedef union NvBootCryptoRsaKey
{
    NvBootCryptoRsaKey1024NvU8 RsaKey1024NvU8;
    NvBootCryptoRsaKey1024NvU32 RsaKey1024NvU32;
    NvBootCryptoRsaKey2048NvU8 RsaKey2048NvU8;
    NvBootCryptoRsaKey2048NvU32 RsaKey2048NvU32;
    NvBootCryptoRsaKeyMaxSizeNvU8 RsaKeyMaxSizeNvU8;
    NvBootCryptoRsaKeyMaxSizeNvU32 RsaKeyMaxSizeNvU32;
} NvBootCryptoRsaKey;

/**
 *  Defines the storage for a full 2048-bit RSA key pair.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoFullRsa2048KeyPairRec
{
    DECLARE_ALIGNED(NvU8 Modulus[NVBOOT_RSA_2048_MODULUS_SIZE_BYTES] , 4);
    DECLARE_ALIGNED(NvU8 Private_exponent_d[NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES] , 4);
    DECLARE_ALIGNED(NvU8 Public_exponent_e[NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES] , 4);
} NvBootCryptoFullRsa2048KeyPair;

/**
 * RSA related parameters, not NVIDIA specific.
 */
typedef struct NvBootCryptoRsaPublicParamsRec
{
    // RSA Public Key Modulus and Exponent
    NvBootCryptoRsaKey RsaPublicKey;
} NvBootCryptoRsaPublicParams;

typedef struct NvBootRsaParams
{
    NvBootRsaKeySize RsaKeySize;
    NvBootCryptoRsaPublicParams RsaPublicParams;
} NvBootRsaParams;

typedef struct NvBootCryptoRsaSsaPssSigNvU8Rec
{
    // The length of the signature is the same as the length of the key used
    // in bytes.
    DECLARE_ALIGNED(NvU8 RsaSsaPssSig[NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES] , 4);
} NvBootCryptoRsaSsaPssSigNvU8;

typedef struct NvBootCryptoRsaSsaPssSigNvU32Rec
{
    // The length of the signature is the same as the length of the key used
    // in bytes.
    DECLARE_ALIGNED(NvU32 RsaSsaPssSig[NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS] , 4);
} NvBootCryptoRsaSsaPssSigNvU32;

typedef union NvBootCryptoRsaSsaPssSig
{
    NvBootCryptoRsaSsaPssSigNvU8 RsaSsaPssSigNvU8;
    NvBootCryptoRsaSsaPssSigNvU32 RsaSsaPssSigNvU32;
} NvBootCryptoRsaSsaPssSig;
