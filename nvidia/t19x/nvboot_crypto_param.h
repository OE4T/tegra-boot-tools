/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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
 * Definition of crypto parameter structures grouped by algorithm, HW engine,
 * and NVIDIA specific parameters.
 */
#ifndef INCLUDED_NVBOOT_CRYPTO_PARAM_H
#define INCLUDED_NVBOOT_CRYPTO_PARAM_H

#include "nvboot_crypto_se_param.h"
#include "nvboot_crypto_aes_param.h"
#include "nvboot_crypto_rsa_param.h"
#include "nvboot_crypto_ecc_param.h"
#include "nvboot_crypto_eddsa_param.h"
#include "nvboot_crypto_sha_param.h"
#include "nvboot_crypto_signatures.h"
#include "nvboot_crypto_public_keystore.h"

typedef enum
{
    CryptoAlgo_None,
    CryptoAlgo_AES,
    CryptoAlgo_AES_CMAC,
    CryptoAlgo_ECC_ECDSA,
    CryptoAlgo_ECC_EDDSA,
    CryptoAlgo_RSA,
    CryptoAlgo_RSA_RSASSA_PSS,
    CryptoAlgo_SHA2,

    CryptoAlgo_Num,
    CryptoAlgo_Force32 = 0x7fffffff,

} NvBootCryptoAlgo;

/**
 * The list of possible HW/SW crypto calculation engines.
 *
 * - See Figure 1 of SE IAS. for top level block diagram.
 * - SE0 is the APB access port for BR use. Other access ports like SE1-4
 * are Host1x ports.
 * - There are multiple engines in what we call the SE.
 * - AES0, AES1 are HW accelerated AES engines.
 * - PKA0 is the original RSA accelerator introducted in T114.
 * - PKA1 is the Elliptic public key accelerator BR will use for ECC operations.
 *
 */
typedef enum
{
    CryptoEngine_Se0_Aes0, // NV AES engine in the SE
    CryptoEngine_Se0_Pka0, // NV RSA engine from T114
    CryptoEngine_Se0_Pka1, // Elliptic PKA engine
    CryptoEngine_Se0_Sha2, // NV SE SHA2 engine
    CryptoEngine_Sw_AES_Engine, // SW implementation of AES inside BR

    CryptoEngine_Num,
    CryptoEngine_Force32 = 0x7fffffff,
} NvBootCryptoEngine;

typedef enum
{
    // OEM key is the RSA or ECC public key in the BCT.
    Verify_w_OEM_Key,
    // NV key is the NVIDIA ECC public key.
    Verify_w_NV_Key,

    Verify_Force32 = 0x7fffffff,
} NvBootVerifyOp;

typedef enum
{
    // OEM key is the SBK.
    Decrypt_w_OEM_Key,
    // NV key is the MB1 decryption key.
    Decrypt_w_NV_Key,

    Decrypt_w_Force32 = 0x7fffffff,
} NvBootDecryptOp;

typedef union
{
    NvBootAesParams AesParams;
    NvBootRsaParams RsaParams;
    NvBootEccParams EccParams;
} NvBootCryptoParams;

/**
 *
 * This struct will house all of the public, non-secret parameters
 * required for asymmmetric-key based authentication schemes.
 *
 * Notes:
 *
 * - Storage is allocated for an RSA exponent, but not used by the
 * Boot ROM. The Boot ROM will always use a RSA public exponent of 0x10001.
 * - Storage is allocated for 384-bit sized Prime field curve parameters. However,
 * user specified curves are not supported by Boot ROM for T186. Only the NIST P-256
 * curve is supported.
 * - The space reserved for the above parameters is for future usage should
 * the Boot ROM POR be changed in subsequent Tegra generations.
 *
 */
 
 
typedef union NvBootEccPublicParamsRec
{
    NvBootEdDsaPoint EdDsaPubKey; // Compressed EdDsa point 32 bytes
    NvBootEcPointT194 EccPublicKey;
} NvBootEccPublicParams;

typedef struct
{
    NvBootCryptoRsaPublicParams RsaPublicParams __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    NvBootEccPublicParams EccPublicParams __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootPublicCryptoParameters;

typedef union
{
    NvBootCryptoSeAes0AesParams SeAes0AesParams;
    NvBootCryptoSePka0RsaParams SePka0RsaParams;
    NvBootCryptoSePka1EccParams SePka1EccParams;
} NvBootNvCryptoEngineParams;

typedef struct
{
    NvBool UseSbk;
    NvBootNvCryptoEngineParams CryptoEngineParams;
} NvBootCryptoNvParams;
#endif


