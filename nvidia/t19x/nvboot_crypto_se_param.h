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

#ifndef INCLUDED_NVBOOT_CRYPTO_SE_H
#define INCLUDED_NVBOOT_CRYPTO_SE_H

#include "nvboot_crypto_aes_param.h"

/**
 * SE notes.
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

/**
 * NVIDIA specific AES parameters for the AES0 engine in the SE.
 */
typedef struct NvBootCryptoSeAes0AesParamsRec
{
    NvBootAesIv OriginalIv;
    NvBootAesIv UpdatedIv;
    NvU32 KeySlotNum;
} NvBootCryptoSeAes0AesParams;

/**
 * NVIDIA specific RSA parameters for the PKA0 engine in the SE.
 */
typedef struct NvBootCryptoSePka0RsaParamsRec
{
    NvU32 KeySlotNum;
} NvBootCryptoSePka0RsaParams;

/**
 * NVIDIA specific ECC parameters for the PKA1 engine in the SE.
 */
typedef struct NvBootCryptoSePka1EccParamsRec
{
    NvU32 KeySlotNum;
} NvBootCryptoSePka1EccParams;

#endif //INCLUDED_NVBOOT_CRYPTO_ECC_H

