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
 * @file nvboot_fuse.h
 *
 * Defines the parameters and data structures related to fuses.
 */

#ifndef INCLUDED_NVBOOT_FUSE_H
#define INCLUDED_NVBOOT_FUSE_H

#include "nvcommon.h"
#if defined(__cplusplus)
extern "C"
{
#endif

#define NVBOOT_DEVICE_KEY_BYTES (16)

typedef struct NvBootECIDRec
{
    NvU32   ECID_0;
    NvU32   ECID_1;
    NvU32   ECID_2;
    NvU32   ECID_3;
} NvBootECID;

//DRF defines for ECID fields.

//Bootrom is defining 128 bits i.e. 4 x 32 bits, out of which
// a)100 bits make ECID for non RCM modes
// b)128 bits make ECID for RCM mode

//<Lot1:2><Wafer:6><X:9><Y:9><Reserved:6>
#define                 ECID_ECID0_0_RSVD1_RANGE        5:0
#define                 ECID_ECID0_0_Y_RANGE            14:6
#define                 ECID_ECID0_0_X_RANGE            23:15
#define                 ECID_ECID0_0_WAFER_RANGE        29:24
#define                 ECID_ECID0_0_LOT1_RANGE         31:30
//<Lot0:6><Lot1:26>
#define                 ECID_ECID1_0_LOT1_RANGE         25:0
#define                 ECID_ECID1_0_LOT0_RANGE         31:26
//<Fab:6><Lot0:26>
#define                 ECID_ECID2_0_LOT0_RANGE         25:0
#define                 ECID_ECID2_0_FAB_RANGE          31:26
//<operating mode:4><rcm version:16><Reserved:8><Vendor:4>
#define                 ECID_ECID3_0_VENDOR_RANGE       3:0

/**
From ISS section 6.2.4.5.
[103:100]
T210_B01: APB_MISC_GP_HIDREV_0_MAJORREV
T194: MISCREG_HIDREV_0_MAJORREV
[111:104]
T210_B01: APB_MISC_GP_HIDREV_0_CHIPID
T194: MISCREG_HIDREV_0_CHIPID
[115:112]
T210_B01: APB_MISC_GP_MINORREV
T194: MISCREG_HIDREV_0_MINORREV
[121:116]
Reserved
[122]
BOOT_SECURITY_INFO[7]
[123]
BPMP_DUMMY_KEY_READ_ENABLE
[126:124]
BOOT_SECURITY_INFO[2:0]
[127]
FUSE_PRODUCTION_MODE
*/
#define                 ECID_ECID3_0_MAJORREV_RANGE 7:4
#define                 ECID_ECID3_0_CHIPID_RANGE 15:8
#define                 ECID_ECID3_0_MINORREV_RANGE 19:16
#define                 ECID_ECID3_0_BOOT_SECURITY_INFO_ECC_RANGE 26:26
#define                 ECID_ECID3_0_BPMP_DUMMY_KEY_READ_ENABLE_RANGE 27:27
#define                 ECID_ECID3_0_BOOT_SECURITY_INFO_AUTH_RANGE 29:28
#define                 ECID_ECID3_0_BOOT_SECURITY_INFO_ENC_RANGE 30:30
#define                 ECID_ECID3_0_PRODUCTION_MODE_RANGE 31:31

/*
 * NvBootFuseBootDevice -- Peripheral device where Boot Loader is stored
 *
 * This enum *MUST* match the equivalent list in nvboot_devmgr.h for
 * all valid values and None, Undefined not present in nvboot_devmgr.h
 */
typedef enum
{
    NvBootFuseBootDevice_Sdmmc,
    NvBootFuseBootDevice_SpiFlash,
    NvBootFuseBootDevice_Sata,
    NvBootFuseBootDevice_resvd_4,
    NvBootFuseBootDevice_Foos = NvBootFuseBootDevice_resvd_4,
    NvBootFuseBootDevice_Usb3,
    NvBootFuseBootDevice_Ufs,
    NvBootFuseBootDevice_Max, /* Must appear after the last legal item */
    NvBootFuseBootDevice_Force32 = 0x7fffffff
} NvBootFuseBootDevice;

/*
 * Definitions of device fuse fields
 */

/**
 * SDMMC configuration fuses.
 */

/**
 * Fuse Bit 2,1,0: Represents the config option supported by the driver.
 */
#define SDMMC_DEVICE_CONFIG_0_DEV_CONFIG_RANGE 2:0

/**
 * Fuse Bit 2,1,0: Represents the config option supported by the driver.
 */
#define SPI_DEVICE_CONFIG_0_DEV_CONFIG_RANGE 2:0

/**
 * Fuse Bit 1,0: Represents the config option supported by the driver.
 */
#define SATA_DEVICE_CONFIG_0_DEV_CONFIG_RANGE 1:0

/**
 * USBH configuration fuses.
 */

/**
 * Fuse Bit 0: Represents the config option supported by the driver.
 */
#define USBH_DEVICE_CONFIG_0_DEV_CONFIG_RANGE 0:0

/**
 * Ufs configuration fuses.
 */

/**
 * Fuse Bit 1,0: Represents the config option supported by the driver.
 */
#define UFS_FUSE_PARAMS_0_DEV_CONFIG_RANGE       2:0


/*
 * Secure Provisioning Fuses
 */
#define FUSE_SECURE_PROVISION_INFO_0_KEY_HIDE_RANGE 0:0
#define FUSE_SECURE_PROVISION_INFO_0_TEST_PART_RANGE 1:1

/*
 * Boot info security fuses, for information on authentication and confidentiality
 * schemes.
 */
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHIFT    _MK_SHIFT_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_FIELD    _MK_FIELD_CONST(0x3, FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_RANGE    1:0
// DEFAULT in this case is SHA2.
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_DEFAULT  _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHA2 _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA  _MK_ENUM_CONST(1)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA_3K  _MK_ENUM_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_ECC  _MK_ENUM_CONST(3)

#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_SHIFT        _MK_SHIFT_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_FIELD        _MK_FIELD_CONST(0x1, FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_RANGE        2:2
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_DEFAULT      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_DISABLE      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_ENABLE       _MK_ENUM_CONST(1)

#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_SHIFT   _MK_SHIFT_CONST(3)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_FIELD   _MK_FIELD_CONST(0x1, FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_RANGE   3:3
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_DEFAULT _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_DISABLE _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_ENABLE  _MK_ENUM_CONST(1)

#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_SHIFT _MK_SHIFT_CONST(4)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_FIELD _MK_FIELD_CONST(0x7, FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_RANGE      6:4
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_DEFAULT    _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_TEST_KEY   _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_NVIDIA_KEY _MK_ENUM_CONST(1)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_1  _MK_ENUM_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_2  _MK_ENUM_CONST(3)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_3  _MK_ENUM_CONST(4)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_4  _MK_ENUM_CONST(5)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_5  _MK_ENUM_CONST(6)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_6  _MK_ENUM_CONST(7)

#define FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_SHIFT        _MK_SHIFT_CONST(7)
#define FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_FIELD        _MK_FIELD_CONST(0x1, FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_RANGE        7:7
#define FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_DEFAULT      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_ECDSA      _MK_ENUM_CONST(0) //(NIST P256 curve)
#define FUSE_BOOT_SECURITY_INFO_0_ECC_ALGORITHM_EDDSA       _MK_ENUM_CONST(1) //(Ed25519 curve)

#define FUSE_RESERVED_PRODUCTION_0_RANDOMIZATION_FEATURE_RANGE 3:3
#define FUSE_RESERVED_PRODUCTION_0_RANDOMIZATION_FEATURE_DEFAULT _MK_ENUM_CONST(0)
// Note reversed polarity for ENABLE & DISABLED value
#define FUSE_RESERVED_PRODUCTION_0_RANDOMIZATION_FEATURE_DISABLED _MK_ENUM_CONST(1)
#define FUSE_RESERVED_PRODUCTION_0_RANDOMIZATION_FEATURE_ENABLED _MK_ENUM_CONST(0)

#define FUSE_RESERVED_PRODUCTION_0_RANDOMIZATION_TUNING_RANGE 9:4 

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_FUSE_H
