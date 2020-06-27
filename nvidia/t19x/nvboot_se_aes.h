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
 * @file nvboot_se_aes.h
 *
 * Defines the parameters and data structure for SE's AES engine.
 *
 * AES is used for encryption, decryption, and signatures.
 */

#ifndef INCLUDED_NVBOOT_SE_AES_H
#define INCLUDED_NVBOOT_SE_AES_H

#include <stdbool.h>
#include "nvcommon.h"
#include "nvboot_config.h"
#include "nvboot_crypto_aes_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines SE AES Key Slots
 */
typedef enum
{
    //Each key-slot can be configured as secure or non-secure as controlled by
    //the corresponding bit in register SE_CRYPTO_SECURITY_PERKEY.
    //Furthermore, it is possible to enable or disable direct register read/write
    //permission for keys and IV in each key-slot and this is controlled via
    //registers SE_CRYPTO_KEYTABLE_ACCESS[keyIndex].
    //See SE IAS section 3.4.1.4 Key-slots Access Control for more info.

    /// Specifies SE AES Key Slot "0"
    NvBootSeAesKeySlot_0,
    /// Specifies SE AES Key Slot "1"
    NvBootSeAesKeySlot_1,
    /// Specifies SE AES Key Slot "2"
    NvBootSeAesKeySlot_2,
    /// Specifies SE AES Key Slot "3"
    NvBootSeAesKeySlot_3,
    /// Specifies SE AES Key Slot "4"
    NvBootSeAesKeySlot_4,
    /// Specifies SE AES Key Slot "5"
    NvBootSeAesKeySlot_5,
    /// Specifies SE AES Key Slot "6"
    NvBootSeAesKeySlot_6,
    /// Specifies SE AES Key Slot "7"
    NvBootSeAesKeySlot_7,
    /// Specifies SE AES Key Slot "8"
    NvBootSeAesKeySlot_8,
    /// Specifies SE AES Key Slot "9"
    NvBootSeAesKeySlot_9,
    /// Specifies SE AES Key Slot "10"
    NvBootSeAesKeySlot_10,
    /// Specifies SE AES Key Slot "11"
    NvBootSeAesKeySlot_11,
    /// Specifies SE AES Key Slot "12"
    NvBootSeAesKeySlot_12,
    /// Specifies SE AES Key Slot "13"
    NvBootSeAesKeySlot_13,
    /// Specifies SE AES Key Slot "14"
    NvBootSeAesKeySlot_14,
    /// Specifies SE AES Key Slot "15"
    NvBootSeAesKeySlot_15,

    // Specifies max number of SE AES Key Slots
    NvBootSeAesKeySlot_Num,

    // Key slot to load the SRK into for SE LP0 context restore.
    // Only used in SC7 path.
    NvBootSeAesKeySlot_SRK = NvBootSeAesKeySlot_2,

    // Specifies the SE key slot for the Decryption Key that might need to be passed on.
    NvBootSeAesKeySlot_KEK1_256b = NvBootSeAesKeySlot_13,
    NvBootSeAesKeySlot_KEK1e_128b = NvBootSeAesKeySlot_13,
    NvBootSeAesKeySlot_KEK1g_128b = NvBootSeAesKeySlot_12,
    NvBootSeAesKeySlot_KEK2_128b = NvBootSeAesKeySlot_11,
    NvBootSeAesKeySlot_MTS_Decryption_Key = NvBootSeAesKeySlot_10,
    NvBootSeAesKeySlot_BPMP_Decryption_Key = NvBootSeAesKeySlot_9,
    NvBootSeAesKeySlot_APE_Decryption_Key = NvBootSeAesKeySlot_8,
    NvBootSeAesKeySlot_SPE_Decryption_Key = NvBootSeAesKeySlot_7,
    NvBootSeAesKeySlot_SCE_Decryption_Key = NvBootSeAesKeySlot_6,
    NvBootSeAesKeySlot_RCE_Decryption_Key = NvBootSeAesKeySlot_5,
    NvBootSeAesKeySlot_PVA_Decryption_Key = NvBootSeAesKeySlot_4,
    NvBootSeAesKeySlot_XUSB_Decryption_Key = NvBootSeAesKeySlot_3,
    NvBootSeAesKeySlot_Default_Key2_Key = NvBootSeAesKeySlot_2,
    NvBootSeAesKeySlot_Default_Key1_Key = NvBootSeAesKeySlot_1,
    NvBootSeAesKeySlot_MB1_Decryption_Key = NvBootSeAesKeySlot_0, // Needs to be zeroed out before exit, since MB1 DK is not passed on to MB1.

    //temp keys used in BR runtime
    NvBootSeAesKeySlot_FEK_NV_Temp = NvBootSeAesKeySlot_1,
    NvBootSeAesKeySlot_FEK_OEM_Temp = NvBootSeAesKeySlot_2,


    // Define Key-wrapping key keyslot, which will
    // then decrypt a user specified secure provisioning Key contained
    // in the BCT.
    // NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt = NvBootSeAesKeySlot_0,

    // TEMPORARY storage of the factory secure
    // provisioning key. This key must be cleared before BR exit.
    NvBootSeAesKeySlot_Secure_Provisioning_Key = NvBootSeAesKeySlot_4,
    NvBootSeAesKeySlot_Secure_Provisioning_Key_Auth_Key = NvBootSeAesKeySlot_3,
    NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt_Key = NvBootSeAesKeySlot_2,

    // Reserve Key slot 14 for exclusive storage of the SBK
    NvBootSeAesKeySlot_SBK = NvBootSeAesKeySlot_14,

    // Reserve Key slot 13 for TEMPORARY storage of the SBK for use by BR only.
    // To facilitate AES-CMAC hash and AES-Decryption of the BCT and Bootloader
    // as data chunks are read in from secondary storage, we use two key slots
    // loaded with the SBK. This reduces the overhead of having to switch
    // between AES-CMAC and AES decrypt operations if we use one keyslot.
    // The keyslot NvBootSeAesKeySlot_SBK_AES_CMAC_Hash MUST BE
    // cleared before Boot ROM exit!!
    //NvBootSeAesKeySlot_SBK_AES_Decrypt = NvBootSeAesKeySlot_SBK, // DEPRECATE
    //NvBootSeAesKeySlot_SBK_AES_CMAC_Hash = NvBootSeAesKeySlot_13, //DEPRECATE

    // Reserve Key slot 15 for exclusive storage of the SSK
    NvBootSeAesKeySlot_SSK = NvBootSeAesKeySlot_15,
} NvBootSeAesKeySlot;

typedef enum
{
    NvBootSeAesSecProvisioningKey_0, // 0 means Secure Provisioning is Disabled
    NvBootSeAesSecProvisioningKey_1,
    NvBootSeAesSecProvisioningKey_2,
    NvBootSeAesSecProvisioningKey_3,
    NvBootSeAesSecProvisioningKey_4,
    NvBootSeAesSecProvisioningKey_5,
    NvBootSeAesSecProvisioningKey_6,
    NvBootSeAesSecProvisioningKey_7,
    NvBootSeAesSecProvisioningKey_8,
    NvBootSeAesSecProvisioningKey_9,
    NvBootSeAesSecProvisioningKey_10,
    NvBootSeAesSecProvisioningKey_11,
    NvBootSeAesSecProvisioningKey_12,
    NvBootSeAesSecProvisioningKey_13,
    NvBootSeAesSecProvisioningKey_14,
    NvBootSeAesSecProvisioningKey_15,
    NvBootSeAesSecProvisioningKey_16,
    NvBootSeAesSecProvisioningKey_17,
    NvBootSeAesSecProvisioningKey_18,
    NvBootSeAesSecProvisioningKey_19,
    NvBootSeAesSecProvisioningKey_20,
    NvBootSeAesSecProvisioningKey_21,
    NvBootSeAesSecProvisioningKey_22,
    NvBootSeAesSecProvisioningKey_23,
    NvBootSeAesSecProvisioningKey_24,
    NvBootSeAesSecProvisioningKey_25,
    NvBootSeAesSecProvisioningKey_26,
    NvBootSeAesSecProvisioningKey_27,
    NvBootSeAesSecProvisioningKey_28,
    NvBootSeAesSecProvisioningKey_29,
    NvBootSeAesSecProvisioningKey_30,
    NvBootSeAesSecProvisioningKey_31,
    NvBootSeAesSecProvisioningKey_32,
    NvBootSeAesSecProvisioningKey_33,
    NvBootSeAesSecProvisioningKey_34,
    NvBootSeAesSecProvisioningKey_35,
    NvBootSeAesSecProvisioningKey_36,
    NvBootSeAesSecProvisioningKey_37,
    NvBootSeAesSecProvisioningKey_38,
    NvBootSeAesSecProvisioningKey_39,
    NvBootSeAesSecProvisioningKey_40,
    NvBootSeAesSecProvisioningKey_41,
    NvBootSeAesSecProvisioningKey_42,
    NvBootSeAesSecProvisioningKey_43,
    NvBootSeAesSecProvisioningKey_44,
    NvBootSeAesSecProvisioningKey_45,
    NvBootSeAesSecProvisioningKey_46,
    NvBootSeAesSecProvisioningKey_47,
    NvBootSeAesSecProvisioningKey_48,
    NvBootSeAesSecProvisioningKey_49,
    NvBootSeAesSecProvisioningKey_50,
    NvBootSeAesSecProvisioningKey_51,
    NvBootSeAesSecProvisioningKey_52,
    NvBootSeAesSecProvisioningKey_53,
    NvBootSeAesSecProvisioningKey_54,
    NvBootSeAesSecProvisioningKey_55,
    NvBootSeAesSecProvisioningKey_56,
    NvBootSeAesSecProvisioningKey_57,
    NvBootSeAesSecProvisioningKey_58,
    NvBootSeAesSecProvisioningKey_59,
    NvBootSeAesSecProvisioningKey_60,
    NvBootSeAesSecProvisioningKey_61,
    NvBootSeAesSecProvisioningKey_62,
    NvBootSeAesSecProvisioningKey_63,

    NvBootSeAesSecProvisioningKey_Num,

    NvBootSeAesSecProvisioningKey_SecProvisioningDisabled = NvBootSeAesSecProvisioningKey_0,
    NvBootSeAesSecProvisioningKey_AntiCloningKeyStart = NvBootSeAesSecProvisioningKey_1,
    NvBootSeAesSecProvisioningKey_AntiCloningKeyEnd = NvBootSeAesSecProvisioningKey_15,
    NvBootSeAesSecProvisioningKey_RegularKeyStart = NvBootSeAesSecProvisioningKey_16,
    NvBootSeAesSecProvisioningKey_RegularKeyEnd = NvBootSeAesSecProvisioningKey_63,
    NvBootSeAesSecProvisioningKey_KeyWrapKey = NvBootSeAesSecProvisioningKey_62,
    NvBootSeAesSecProvisioningKey_NvDebugKey = NvBootSeAesSecProvisioningKey_63,
} NvBootSeAesSecProvisioningKeySlot;


typedef enum
{
    Kek_128b,
    Kek_256b,

    Kek_Select_Num,
} KekKeySelect;

/**
 * Defines the maximum length of an Initial Vector (IV) in units of
 * 32 bit words.
 */
enum {NVBOOT_SE_AES_MAX_IV_LENGTH = 8}; //todo: needed?

/**
 * Defines the length of an Initial Vector (IV) as used by the Boot ROM
 * in units of 32 bit words.
 */
enum {NVBOOT_SE_AES_IV_LENGTH = 4};

/**
 * Defines the length of an original Initial Vector (IV) as used by the Boot ROM
 * in units of 32 bit words.
 */
enum {NVBOOT_SE_AES_ORIG_IV_LENGTH = 4};

/**
 * Defines the length of an original Initial Vector (IV) as used by the Boot ROM
 * in units of bytes.
 */
enum {NVBOOT_SE_AES_ORIG_IV_LENGTH_BYTES = 16};

/**
 * Defines the length of an updated Initial Vector (IV) as used by the Boot ROM
 * in units of 32 bit words.
 */
enum {NVBOOT_SE_AES_UPDATED_IV_LENGTH = 4};

/**
 * Defines the length of an updated Initial Vector (IV) as used by the Boot ROM
 * in units of bytes.
 */
enum {NVBOOT_SE_AES_UPDATED_IV_LENGTH_BYTES = 16};

/**
 * Defines the max length of a key in units of 32 bit words.
 */
enum {NVBOOT_SE_AES_MAX_KEY_LENGTH = 8};    // 256-bits

/**
 * Defines the max length of a key in units of bytes
 */
enum {NVBOOT_SE_AES_MAX_KEY_LENGTH_BYTES = 32}; // 256-bits

/**
 * Defines the length of a 128-bit key in units 32-bit words.
 */
enum {NVBOOT_SE_AES_KEY128_LENGTH = 4};

/**
 * Defines the length of a 128-bit key in units of bytes
 */
enum {NVBOOT_SE_AES_KEY128_LENGTH_BYTES = 16};

/**
 * Defines the length of a 192-bit key in units 32-bit words
 */
enum {NVBOOT_SE_AES_KEY192_LENGTH = 6};

/**
 * Defines the length of a 192-bit key in units of bytes
 */
enum {NVBOOT_SE_AES_KEY192_LENGTH_BYTES = 24};

/**
 * Defines the length of a 256-bit key in units 32-bit words.
 */
enum {NVBOOT_SE_AES_KEY256_LENGTH = 8};

/**
 * Defines the length of a 256-bit key in units of byte
 */
enum {NVBOOT_SE_AES_KEY256_LENGTH_BYTES = 32};


/**
 * Defines the length of an AES block in units of 32 bit words.
 */
enum {NVBOOT_SE_AES_BLOCK_LENGTH = 4};

/**
 * Defines the length of an AES block in units of log2 bytes.
 */
enum {NVBOOT_SE_AES_BLOCK_LENGTH_LOG2 = 4};

/**
 * Defines the length of an AES block in units of bytes.
 * 128-bits = 16 bytes.
 */
enum {NVBOOT_SE_AES_BLOCK_LENGTH_BYTES = 16};

/**
 * Defines a struct for a 128-bit AES Key.
 */
typedef struct NvBootAes128KeyRec
{
    /// Specifies the key data.
    NvU32 Key[NVBOOT_SE_AES_KEY128_LENGTH];
} NvBootAes128Key;

/**
 * Defines a struct for a 128-bit AES Iv.
 */
typedef struct NvBootAes128IvRec
{
    /// Specifies the key data.
    NvU32 Iv[NVBOOT_SE_AES_KEY128_LENGTH];
} NvBootAes128Iv;

/**
 * Defines a struct for a 256-bit AES Key.
 */
typedef struct NvBootAes256KeyRec
{
    /// Specifies the key data.
    NvU32 Key[NVBOOT_SE_AES_KEY256_LENGTH];
} NvBootAes256Key;

/**
 * Defines a struct for a 128-bit AES Key.
 */
typedef struct NvBootAes128KeyNvU8Rec
{
    /// Specifies the key data.
    uint8_t Key[NVBOOT_SE_AES_KEY128_LENGTH_BYTES];
} NvBootAes128KeyNvU8;

/**
 * Defines a struct for a 192-bit AES Key.
 */
typedef struct NvBootAes192KeyNvU8Rec
{
    /// Specifies the key data.
    uint8_t Key[NVBOOT_SE_AES_KEY192_LENGTH_BYTES];
} NvBootAes192KeyNvU8;

/**
 * Defines a struct for a 256-bit AES Key.
 */
typedef struct NvBootAes256KeyNvU8Rec
{
    /// Specifies the key data.
    uint8_t Key[NVBOOT_SE_AES_KEY256_LENGTH_BYTES];
} NvBootAes256KeyNvU8;

/**
 * Defines the value of const_Rb, as specified in the AES-CMAC specification.
 * See RFC 4493: http://tools.ietf.org/html/rfc4493.
 */
enum {NVBOOT_SE_AES_CMAC_CONST_RB = 0x87};

/**
 * Define a struct for AES operstaions on SE.
 * Handles addresses > 32-bits.
 */
typedef struct NvBootSeAesContextRec
{
    uint8_t *Src;
    uint8_t *Src_Hi;
    uint8_t *Dst;
    uint8_t *Dst_Hi;
    // Only handle 32-bit sizes.
    uint32_t Size;
    uint8_t KeySlot;
    NvBootAesKeySize KeySize;
    // Clear IVs to 0 if first operation.
    bool First;
} NvBootSeOpContext;

/**
 * Define a macro to check if the MSB of the input is set.
 */
#define NVBOOT_SE_AES_CMAC_IS_MSB_SET(x)  ( (x) >> (8*sizeof(x)-1) )

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SE_AES_H

