/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

#ifndef INCLUDED_NVBOOT_CRYPTO_EDDSA_H
#define INCLUDED_NVBOOT_CRYPTO_EDDSA_H

#include "nvboot_util_int.h"

/**
 *  Compressed EDDSA point.
 *  Encoded points can be represented by b bits where 2b is the size of sha512. So b=256
 *  Encoded point for (x,y) is sign(x)|| y. sign(x) goes to MSB of encoded point in Little Endian.
 *  
 */
#define ED25519_POINT_SIZE_BITS  (256)
#define ED25519_POINT_SIZE_BYTES  (ED25519_POINT_SIZE_BITS>>3)
typedef struct
{
    uint8_t Pxy[ED25519_POINT_SIZE_BYTES];
} NvBootEdDsaPoint;

/**
 *  EdDsa signature is (R,S) where
 *  R: compressed point representing rB
 *     where r = Hash(Hb...H2b-1 || M)
 *     where Hb...H2b-1 = Upper half of Hash(priv_key_scalar)
 *     where 2b = 512
 *     where   B = Base point
 *     where M= message
 *  S: r+ Hash(R,A,M)s(mod l) 
 *     where A =  Public key (sB)
 *     where s = H0...Hb-1 (lower half of priv_key_scalar)
 *     where l = order of B subgroup (cofactor 2^c, #E(F)=2^c * l)
 */
typedef struct
{
    NvBootEdDsaPoint R;
    NvBootEdDsaPoint S;
}DECLARE_ALIGNED(NvBootEdDsaSignature, 4);




#endif
