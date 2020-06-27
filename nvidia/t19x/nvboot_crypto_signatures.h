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
 * \file nvboot_crypto_signature.h
 *
 * Defines the structure to hold the various possible types of
 * cryptographic signatures supported by the Boot ROM.
 *
 */
#ifndef NVBOOT_CRYPTO_SIGNATURE_H
#define NVBOOT_CRYPTO_SIGNATURE_H



/**
 *  union to store all ECC signatures
 */
 typedef union NvBootEccSignaturesRec
 {
     NvBootEcdsaSig EcdsaSig;
     NvBootEdDsaSignature EdDsaSig; // EdDsa signature is a compressed point 32 bytes
 } NvBootEccSignatures;
/**
 * \brief           Crypto signature structure.
 *
 * \note Force AES block alignment for all members. This matters
 *       where signatures are sometimes contained in signed sections
 *       (like in the case where BootLoader signatures are inside the
 *       BCT). Sha256 Digest and RSASSA-PSS signatures are inherently
 *       16-byte aligned so the attribute isn't necessary. However,
 *       making it explicit doesn't hurt.
 */
typedef struct NvBootCryptoSignaturesRec
{
    DECLARE_ALIGNED(uint8_t Digest[32], NVBOOT_CRYPTO_BUFFER_ALIGNMENT); //SHA-256 hash
    DECLARE_ALIGNED(NvBootCryptoRsaSsaPssSig RsaSsaPssSig, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(NvBootEccSignatures EccSig, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} NvBootCryptoSignatures;

#endif
