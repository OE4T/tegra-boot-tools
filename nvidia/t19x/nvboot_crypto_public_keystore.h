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

#ifndef INCLUDED_NVBOOT_CRYPTO_PUBLIC_KEYSTORE_H
#define INCLUDED_NVBOOT_CRYPTO_PUBLIC_KEYSTORE_H

typedef struct NvSysRamPublicKeysRec
{
    NvU8 MTS[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 MTS_Debug[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 MB1[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES]; //this value is only valid for BR run time
    NvU8 MB1_Debug[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES]; //this value is only valid for BR run time
    NvU8 BPMP[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 APE[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 SPE[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 SCE[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 RCE[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 PVA[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 XUSB[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 IST[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
    NvU8 IST_Debug[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES];
} NvSysRamPublicKeys;

#endif //INCLUDED_NVBOOT_CRYPTO_PUBLIC_KEYSTORE_H
