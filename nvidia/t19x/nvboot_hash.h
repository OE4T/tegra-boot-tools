/*
 * Copyright (c) 2006-2009, NVIDIA CORPORATION. All rights reserved.
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
 * @file nvboot_hash.h
 *
 * Defines the NvBootHash data structure for the Boot ROM.
 *
 * The Boot ROM uses the AES-CMAC keyed hash function for validating objects
 * read from the secondary boot device.
 *
 * The AES-CMAC algorithm is defined in RFC 4493 published by the Internet
 * Engineering Task Force (IETF).  The RFC is available at the following web
 * site, among others --
 *
 *    http://tools.ietf.org/html/rfc4493
 *
 * The 128-bit variant is implemented here, i.e., AES-CMAC-128.  This variant
 * employs an underlying AES-128 encryption computataion.
 *
 * This implementation only handles messages that are a multiple of 16 bytes in
 * length.  The intent is to compute the hash value for AES-encrypted messages
 * which (by definition) are always a multiple of 16 bytes in length.  Hence,
 * the K2 constant will never be used and therefore is not even computed.
 * Similarly, there's no provision for padding packets since they're already
 * required to be a multiple of 16 bytes in length.
 */

#ifndef INCLUDED_NVBOOT_HASH_H
#define INCLUDED_NVBOOT_HASH_H

#include "nvcommon.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the CMAC-AES-128 hash length in 32 bit words. (128 bits = 4 words)
 */
enum {NVBOOT_CMAC_AES_HASH_LENGTH = 4};

/**
 * Defines the storage for a hash value (128 bits).
 */
typedef struct NvBootHashRec
{
    NvU32 hash[NVBOOT_CMAC_AES_HASH_LENGTH];
} NvBootHash;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_HASH_H
