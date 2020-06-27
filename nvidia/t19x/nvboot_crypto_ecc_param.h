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

#ifndef INCLUDED_NVBOOT_CRYPTO_ECC_H
#define INCLUDED_NVBOOT_CRYPTO_ECC_H

#include "nvboot_util_int.h"

#if defined (_MSC_VER)
#define DECLARE_ALIGNED(var, n) __declspec(align(n)) var
#elif defined __GNUC__
#define DECLARE_ALIGNED(var, n) var __attribute__((aligned(n)))
#endif


//enum {NVBOOT_SECP256R1_KEY_SIZE_BYTES = NV_ICEIL(256, 8)};
//enum {NVBOOT_SECP521R1_KEY_SIZE_BYTES = NV_ICEIL(521, 8)};

//enum {NVBOOT_ECC_PRIME_FIELD_DEFAULT_KEY_SIZE_BYTES = NVBOOT_SECP256R1_KEY_SIZE_BYTES};
//enum {NVBOOT_ECC_PRIME_FIELD_MAX_KEY_SIZE_BYTES = NVBOOT_SECP521R1_KEY_SIZE_BYTES};

/**
 * The Boot ROM need only support the NIST P-256 EC curve.
 * No user specified curves required.
 * See: http://nvbugs/1630454/2
 */
typedef enum
{
    EccCurve_Nist_P256,

    EccCurve_Num,
    EccCurve_Default = EccCurve_Nist_P256,

} NvBootEccEllipticCurves;

/**
 * P-192 is not defined because it is unlikely it will ever be used.
 */
typedef enum
{
    EccPrimeFieldKeyBits192 = 192,
    EccPrimeFieldKeyBits224 = 224,
    EccPrimeFieldKeyBits256 = 256,
    EccPrimeFieldKeyBits384 = 384,
    EccPrimeFieldKeyBits521 = 521,

    EccPrimeFieldKeyDefaultKeySizeBits = EccPrimeFieldKeyBits256,
} NvBootEccPrimeFieldKeySizeBits;

/**
 * IMPORTANT NOTE. An valid EC "Key" is a point on the curve. Each point is
 * specified by two integers x and y in the interval [0, p-1] for prime field
 * curves. Each integer x and y can be up to the key size in bytes.
 * For example, a 256-bit "key" Q = (x, y) is specified in 512-bits.
 *
 */
typedef enum
{
    EccPrimeFieldKeySizeBytes192 =  NV_ICEIL((uint32_t)EccPrimeFieldKeyBits192, 8U),
    EccPrimeFieldKeySizeBytes224 =  NV_ICEIL((uint32_t)EccPrimeFieldKeyBits224, 8U),
    EccPrimeFieldKeySizeBytes256 =  NV_ICEIL((uint32_t)EccPrimeFieldKeyBits256, 8U),
    EccPrimeFieldKeySizeBytesT194 = EccPrimeFieldKeySizeBytes256,
    EccPrimeFieldKeySizeBytes384 =  NV_ICEIL((uint32_t)EccPrimeFieldKeyBits384, 8U),
    EccPrimeFieldKeySizeBytes521 =  NV_ICEIL((uint32_t)EccPrimeFieldKeyBits521, 8U),

    EccPrimeFieldKeyDefaultKeySizeBytes = EccPrimeFieldKeySizeBytes256,
    EccPrimeFieldKeyMaxKeySizeBytes = EccPrimeFieldKeySizeBytes384,
} NvBootEccPrimeFieldKeySizeBytes;

typedef struct NvBootEcPoint256Rec
{
    DECLARE_ALIGNED(uint8_t x[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(uint8_t y[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEcPoint256, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef struct NvBootEcPoint384Rec
{
    DECLARE_ALIGNED(uint8_t x[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(uint8_t y[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEcPoint384, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef struct NvBootEcPoint521Rec
{
    DECLARE_ALIGNED(uint8_t x[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(uint8_t y[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEcPoint521, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef struct NvBootEcPointMaxRec
{
    DECLARE_ALIGNED(uint8_t x[EccPrimeFieldKeyMaxKeySizeBytes], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    DECLARE_ALIGNED(uint8_t y[EccPrimeFieldKeyMaxKeySizeBytes], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEcPointMax, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef NvBootEcPoint256 NvBootEcPointT194;

/**
 * \brief           A struct to store the x and y coordinate of a point on the
 *                  elliptic curve over GF(p).
 *
 */
typedef union NvBootEcPointRec
{
    NvBootEcPoint256 EcPoint256;
    NvBootEcPoint384 EcPoint384;
    NvBootEcPointMax EcPointMax;
} NvBootEcPoint;

/**
typedef struct NvBootEccPrimeFieldParamsRec
{

    uint8_t *p;
    uint8_t *n;
    uint8_t *SEED;
    uint8_t *a; // NIST curves have a=-3
    uint8_t *b;
    NvBootEcPoint *G;
    uint8_t *h;
} NvBootEccPrimeFieldParams;
*/

/**
 * \brief           Parameters for P-521 NIST elliptic Curve over Fp
 *
 * \note            Let p > 3 be an odd prime. An elliptic curve E
 *                  over Fp is defined by an equation of the form
 *                  y^2 = x^3 + ax +b.
 *
 *                  For efficiency reasons, a = -3 is always used (see
 *                  IEEE Std 1363-2000), so it is not a configurable
 *                  parameter in this struct.
 *
 * \note            See fips-4 table D-1 on the bit length of n.
 *                  Note that the bit length of n can be 256 to 383 bits
 *                  for a Prime field p = 256.
 *                  The bit length of prime field p = 521 is >=512.
 *                  Therefore, let's declare each parameter with a size
 *                  of Ceiling(521/8).
 */
typedef struct NvBootEccPrimeFieldParamsP256Rec
{
    /*!< p = Prime modulus of the base field */
    DECLARE_ALIGNED(uint8_t p[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Coefficient a                   */
    DECLARE_ALIGNED(uint8_t a[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Coefficient b                   */
    DECLARE_ALIGNED(uint8_t b[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Order n of the curve            */
    DECLARE_ALIGNED(uint8_t n[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  X-coordinate of the base point on the curve */
    DECLARE_ALIGNED(uint8_t Gx[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Y-coordinate of the base point on the curve */
    DECLARE_ALIGNED(uint8_t Gy[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEccPrimeFieldParamsP256, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

/**
 * \brief           Parameters for P-384 NIST elliptic Curve over Fp
 *
 * \note            Let p > 3 be an odd prime. An elliptic curve E
 *                  over Fp is defined by an equation of the form
 *                  y^2 = x^3 + ax +b.
 *
 *                  For efficiency reasons, a = -3 is always used (see
 *                  IEEE Std 1363-2000), so it is not a configurable
 *                  parameter in this struct.
 *
 * \note            See fips-4 table D-1 on the bit length of n.
 *                  Note that the bit length of n can be 256 to 383 bits
 *                  for a Prime field p = 256.
 *                  The bit length of prime field p = 521 is >=512.
 *                  Therefore, let's declare each parameter with a size
 *                  of Ceiling(521/8).
 */
typedef struct NvBootEccPrimeFieldParamsP384Rec
{
    /*!< p = Prime modulus of the base field */
    DECLARE_ALIGNED(uint8_t p[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Coefficient a                   */
    DECLARE_ALIGNED(uint8_t a[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Coefficient b                   */
    DECLARE_ALIGNED(uint8_t b[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Order n of the curve            */
    DECLARE_ALIGNED(uint8_t n[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  X-coordinate of the base point on the curve */
    DECLARE_ALIGNED(uint8_t Gx[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Y-coordinate of the base point on the curve */
    DECLARE_ALIGNED(uint8_t Gy[EccPrimeFieldKeySizeBytes384], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEccPrimeFieldParamsP384, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

/**
 * \brief           Parameters for P-521 NIST elliptic Curve over Fp
 *
 * \note            Let p > 3 be an odd prime. An elliptic curve E
 *                  over Fp is defined by an equation of the form
 *                  y^2 = x^3 + ax +b.
 *
 *                  For efficiency reasons, a = -3 is always used (see
 *                  IEEE Std 1363-2000), so it is not a configurable
 *                  parameter in this struct.
 *
 * \note            See fips-4 table D-1 on the bit length of n.
 *                  Note that the bit length of n can be 256 to 383 bits
 *                  for a Prime field p = 256.
 *                  The bit length of prime field p = 521 is >=512.
 *                  Therefore, let's declare each parameter with a size
 *                  of Ceiling(521/8).
 */
typedef struct NvBootEccPrimeFieldParamsP521Rec
{
    /*!< p = Prime modulus of the base field */
    DECLARE_ALIGNED(uint8_t p[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Coefficient a                   */
    DECLARE_ALIGNED(uint8_t a[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Coefficient b                   */
    DECLARE_ALIGNED(uint8_t b[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Order n of the curve            */
    DECLARE_ALIGNED(uint8_t n[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  X-coordinate of the base point on the curve */
    DECLARE_ALIGNED(uint8_t Gx[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    /*!<  Y-coordinate of the base point on the curve */
    DECLARE_ALIGNED(uint8_t Gy[EccPrimeFieldKeySizeBytes521], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEccPrimeFieldParamsP521, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef union NvBootEccPrimeFieldParamsRec
{
    NvBootEccPrimeFieldParamsP256 EccPrimeFieldParamsP256;
    NvBootEccPrimeFieldParamsP384 EccPrimeFieldParamsP384;
} DECLARE_ALIGNED(NvBootEccPrimeFieldParams, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);


typedef struct NvBootEccCalcBufferP256Rec
{
    // w = (s')^-1 nod n
    DECLARE_ALIGNED(uint8_t w[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    // u1 = (e' * w) mod n
    DECLARE_ALIGNED(uint8_t u1[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    // u2 = (r' * w) mod n
    DECLARE_ALIGNED(uint8_t u2[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    // R = (Xr, Yr) = u1 * G + u2 * Q
    DECLARE_ALIGNED(NvBootEcPoint256 R, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    // v = Xr mod n
    DECLARE_ALIGNED(uint8_t v[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
    // e', the modular reduced version of the hash of the input message.
    DECLARE_ALIGNED(uint8_t e_prime[EccPrimeFieldKeySizeBytes256], NVBOOT_CRYPTO_BUFFER_ALIGNMENT);
} DECLARE_ALIGNED(NvBootEcdsaCalcBufferP256, NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

typedef union NvBootEccCalcBufferRec
{
    NvBootEcdsaCalcBufferP256 EcdsaCalcBufferP256;
} NvBootEcdsaCalcBuffer;

typedef struct NvBootEccParamsRec
{
    NvBootEcPointT194 EccPublicParams;
} NvBootEccParams;

// ECDSA Signature S = (r,s), where r and s are integers in the interval of
// [1, n-1], specified as bytes/octets.
typedef struct NvBootEcdsaSigRec
{
    uint8_t r[EccPrimeFieldKeySizeBytesT194];
    uint8_t s[EccPrimeFieldKeySizeBytesT194];
} NvBootEcdsaSig;

/**
* \brief   NIST P-256 curve parameters (aka secp256r1).
*
*/
//extern const NvBootEccPrimeFieldParamsP256 EccPrimeFieldParamsP256_NIST_P256;

#endif
