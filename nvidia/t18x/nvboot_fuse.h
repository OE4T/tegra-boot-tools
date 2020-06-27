/*
 * Copyright (c) 2014-2018 NVIDIA Corporation.  All rights reserved.
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
#define                 ECID_ECID3_0_RSVD2_RANGE        11:4
#define                 ECID_ECID3_0_RCM_VERSION_RANGE  27:12
#define                 ECID_ECID3_0_OPMODE_RANGE       31:28   // Valid for RCM mode only

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
    NvBootFuseBootDevice_ProdUart,
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
 * Fuse Bit 0: indicates the ddr mode to select.
 *      = 0 means, Normal mode.
 *      = 1 means, DDR mode.
 */
#define SDMMC_DEVICE_CONFIG_0_DDR_MODE_RANGE 0:0
/**
 * Fuse Bits 1: indicates the voltage range to use.
 *      = 00 means query voltage range from the card.
 *      = 01 means use low voltage range.
 */
#define SDMMC_DEVICE_CONFIG_0_VOLTAGE_RANGE_RANGE 1:1
/**
 * Fuse Bit 2: indicates the Boot mode support.
 *      = 0 means the Boot mode is disabled.
 *      = 1 means the Boot mode is enabled.
 */
#define SDMMC_DEVICE_CONFIG_0_ENABLE_BOOT_MODE_RANGE 2:2
/**
 * Fuse Bit 3,4: SDMMC4 Clock divider
 *  Option to control the clock divider by subtracting the value from fuses.
 *      = 0, means default clock divider 16
 *      = 1, means default clock divider 8
 *      = 2, means default clock divider 4
 *      = 3, Reserved
 */
#define SDMMC_DEVICE_CONFIG_0_SDMMC4_CLOCK_DIVIDER_RANGE 4:3
/**
 * Fuse Bit 5: SDMMC4 read MultiPage support
 *      = 0, MultiPage Read Support
 *      = 1, Single page, 512 bytes page size
 */
#define SDMMC_DEVICE_CONFIG_0_SDMMC4_MULTI_PAGE_READ_RANGE 5:5


/**
 * SPI configuration fuses.
 */

/**
 * Bit 0 --> 0 = 2k page size, 1 = 16k page size
 */
#define SPI_DEVICE_CONFIG_0_PAGE_SIZE_2KOR16K_RANGE 0:0
/**
 * Bit 1 --> 0 = 1 bit, 1 = 4 bit
 */
#define SPI_DEVICE_CONFIG_0_DATA_BUS_WIDTH_RANGE 1:1
/**
 * Bit 2 --> 0 = PIO, 1 = DMA
 */
#define SPI_DEVICE_CONFIG_0_MODE_RANGE 2:2
/**
 * Bit 3 --> 0 = 20MHz, 1 = 50MHz
 */
#define SPI_DEVICE_CONFIG_0_SPI_CLOCK_RANGE 3:3
/**
 * Bit 4 --> 0 = Active Low, 1 = Active High
 */
#define SPI_DEVICE_CONFIG_0_CS_POLARITY_RANGE 4:4

/**
 * USBH configuration fuses.
 */

/**
 * Bit 0,1,2,3 -->  used for representing the root port number device is attached to the
 * usb host controller.
 */
#define USBH_DEVICE_CONFIG_0_ROOT_PORT_RANGE 3:0

// /**
 // * Bit 4 --> 0 = 2k page size, 1 = 16k page size
 // */
// #define USBH_DEVICE_CONFIG_0_PAGE_SIZE_2KOR16K_RANGE 4:4

/**
 * Bit 4,5,6,7--->  used to represent to program the OC pin or OC group this port.
 */
#define USBH_DEVICE_CONFIG_0_OC_PIN_RANGE 7:4

/**
 * Bit 8 --> 0 = pad 0, 1 = pad 1, used to represent tri-state of the associated vbus pad.
 */
#define USBH_DEVICE_CONFIG_0_VBUS_ENABLE_RANGE 8:8


/*
 * Boot info security fuses, for information on authentication and confidentiality
 * schemes.
 */
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHIFT    _MK_SHIFT_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_FIELD    _MK_FIELD_CONST(0x3, FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_RANGE    1:0
// DEFAULT in this case is also AES-CMAC.
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_DEFAULT  _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_AES_CMAC _MK_ENUM_CONST(1)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA  _MK_ENUM_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_ECC  _MK_ENUM_CONST(3)

#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_SHIFT        _MK_SHIFT_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_FIELD        _MK_FIELD_CONST(0x1, FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_RANGE        2:2
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_DEFAULT      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_DISABLE      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_ENABLE       _MK_ENUM_CONST(1)

#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_SHIFT _MK_SHIFT_CONST(3)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_FIELD _MK_FIELD_CONST(0x7, FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_RANGE      5:3
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_DEFAULT    _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_TEST_KEY   _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_NVIDIA_KEY _MK_ENUM_CONST(1)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_1  _MK_ENUM_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_2  _MK_ENUM_CONST(3)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_3  _MK_ENUM_CONST(4)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_4  _MK_ENUM_CONST(5)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_5  _MK_ENUM_CONST(6)
#define FUSE_BOOT_SECURITY_INFO_0_ODM_FUSE_ENCRYPTION_SELECT_OEM_KEY_6  _MK_ENUM_CONST(7)

#define FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_SHIFT   _MK_SHIFT_CONST(15)
#define FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_FIELD   _MK_FIELD_CONST(0x1, FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_SHIFT)
#define FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_RANGE   15:15
#define FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_DEFAULT _MK_ENUM_CONST(0)
#define FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_DISABLE _MK_ENUM_CONST(0)
#define FUSE_ODM_INFO_0_ODM_FUSE_ENCRYPTION_ENABLE_ENABLE  _MK_ENUM_CONST(1)

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_FUSE_H
