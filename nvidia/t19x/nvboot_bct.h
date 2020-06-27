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
 * @file
 * <b>NVIDIA Tegra ODM Kit:
 *         Boot Configuration Table (Tegra APX)</b>
 *
 * @b Description: NvBootConfigTable (BCT) contains the information
 *  needed to load boot loaders (BLs).
 */

/**
 * @defgroup nvbl_bct_ap15 BCT (Tegra APX)
 * @ingroup nvbl_bct_group
 * @{
 *
 * @par Boot Sequence
 *
 * The following is an overview of the boot sequence.
 * -# The Boot ROM (BR) uses information contained in fuses and straps to
 *    determine its operating mode and the secondary boot device from which
 *    to boot. If the recovery mode strap is enabled or the appropriate
 *    AO bit is set, it heads straight to recovery mode.
 *    The BR also intializes the subset of the hardware needed to boot the
 *    device.
 * -# The BR configures the secondary boot device and searches for a valid
 *    Boot Configuration Table (BCT). If it fails to locate one, it enters
 *    recovery mode.
 * -# If the BCT contains device parameters, the BR reconfigures the
 *    appropriate controller.
 * -# The BCT attempts to load a boot loader (BL), using redundant copies
 *    and failover as needed. The BR enters recovery mode if it cannot load
 *    a valid BL.
 * -# The BR cleans up after itself and hands control over to the BL.
 *
 * <!-- Note: Recovery mode is described in nvboot_rcm.h. -->
 * <!-- During the boot process, the BR records data in the Boot Information
 * Table (BIT). This table provides information to the BL about what
 * transpired during booting, along with a pointer to where a copy of
 * the BCT can be found in memory. Details about the BIT can be found
 * in nvboot_bit.h.
 * -->
 *
 * @par Boot ROM Operating Modes
 *
 * The operating modes of the BR include:
 * - @b NvProduction: This is the mode in which chips are provided to customers
 *   from NVIDIA. In this mode, fuses can still be programmed via recovery
 *    mode. BCTs and BLs are signed with a key of all 0's, but not encrypted.
 * - @b OdmNonSecure: This is the mode in which customers ship products if they
 *   choose not to enable the more stringent security mechanisms. In
 *   this mode, fuses can no longer be programmed. As in NvProduction mode,
 *   BCTs and BLs have SHA256 integrity check and not encrypted.
 *   This mode is sometimes called OdmProduction.
 * - @b OdmSecure: This is the mode in which customers ship products with the
 *   stricter security measures in force. Fuses cannot be programmed, and
 *   all BCTs and BLs must be signed and encrypted with the secure boot key
 *   (SBK).
 *
 * @par Cryptographic Notes
 *
 * - If a BCT is encrypted, it is encrypted starting from the
 *   NvBootConfigTableRec::RandomAesBlock2 field and ends at the end of the BCT
 *   (the end of the NvBootConfigTable::Reserved area).
 * - If a BL is encrypted, the entire BL image, including any padding, is
 *   encrypted.
 * - Signatures are computed as a CMAC hash over the encrypted data.
 * - All cryptographic operations use 128-bit AES in CBC mode w/an IV of 0's.
 *
 * @par Requirements for a Good BCT
 *
 * To be used by the BR, the BCT's CryptoHash must match the hash value
 * computed while reading the BCT from the secondary boot device.
 *
 * For secondary boot devices that do not naturally divide storage into pages
 * and blocks, suitable values have been chosen to provide a consistent model
 * for BCT and BL loading. For eMMC devices, the page size is fixed at 512
 * bytes and the block size is 4096 bytes.
 *
 * <!-- Additional requirements for BCTs created from scratch are:
 *  - The BootDataVersion must match the BR's data structure version number.
 *  - The block and page sizes must lie within the allowed range.
 *  - The block and page sizes must match the sizes used by the device manager
 *    to talk to the device.
 *  - The partition size must be a multiple of the block size.
 *  - The number of device parameter sets must be within range.
 *  - The block size used by the bad block table must be the BCT's block size.
 *  - The block size must be <= the virtual block size used by the
 *    bad block table.
 *  - The number of entries used within the bad block table must fit within
 *    the space available in the table.
 *  - The number of entries used must be equal to the number of virtual blocks
 *    that fit within the partition size.
 *  - The number of BLs present must fit within the available table space.
 *  - For each BL in the table:
 *     -# The starting page must fit within a block.
 *     -# The length of the BL > 0.
 *     -# The BL must fit within the partition.
 *     -# The entry point must lie within the BL.
 *  - The \c Reserved field must contain the padding pattern, which is one byte
 *    of 0x80 followed by bytes of 0x00.
 * -->
 * @par Boot ROM Search for a Good BCT
 *
 * After configuring the hardware to read from the secondary boot device,
 * the BR commences a search for a valid BCT. In the descriptions that
 * follow, the term "slot" refers to a potential location of a BCT in a block.
 * A slot is the smallest integral number of pages that can hold a BCT.
 * Thus, every BCT begins at the start of a page and may span multiple pages.
 *
 * The search sequence is:
 * <pre>
 *    Block 0, Slot 0
 *    Block 0, Slot 1
 *    Block 1, Slot 0
 *    Block 1, Slot 1
 *    Block 1, Slot 2
 *    . . .
 *    Block 1, Slot N
 *    Block 2, Slot 0
 *    . . .
 *    Block 2, Slot N
 *    . . .
 *    Block 63, Slot N
 * </pre>
 *
 * A few points worthy of note:
 * - Block 0 is treated differently from the rest. In some storage devices,
 *   this block has special properties, such as being guaranteed to be good
 *   from the factory.
 * - The remaining blocks that are searched are journal blocks. These are
 *   backups which provide a means to boot the system in the presence of
 *   unexpected failures or interrupted device updates.
 * - The search within a journal block ends as soon as a bad BCT or a read
 *   error is found.
 * - Not all of the journal blocks need to contain BCTs. If the BR reads
 *   non-BCT data, it should fail to validate.
 * - The search terminates when:
 *    -# A good BCT is found in either of the slots in Block 0.
 *    -# A good BCT is found in a journal block and either the end of the
 *       block is reached or an error (validation failure or read error)
 *       occurs. The last good BCT in the journal block is used.
 *
 * Once a good BCT has been located, the BR proceeds with the boot sequence.
 *
 * <!-- Details of the device parameters are contained within their
 * respective header files.
 * -->
 *
 * The BR attempts to load each BL in the order they appear in the BootLoader
 * table (which is an array of \c NvBootLoaderInfo structures) and also within
 * the selected boot chain until locating a good one. A BL is good if it fits 
 * within the destination memory area and passes the signature check.
 *
 * The BR begins reading a BCH from NvBootLoaderInfoRec::StartPage within
 * NvBootLoaderInfoRec::StartBlock. In the BCH, the location of the StartPage
 * and StartBlock of the BL is given.  BR will continue to read pages sequentially
 * from this point, skipping over known bad blocks. Read failures cause the
 * BR to use data from the redundant copies in an effort to assemble a
 * complete, good BL.
 *
 * By default, the BR will only load BLs from the first generation it finds.
 * A generation is a set of BLs with the same version number. 
 */

#ifndef INCLUDED_NVBOOT_BCT_H
#define INCLUDED_NVBOOT_BCT_H

#include "nvcommon.h"
#include "nvboot_config.h"
#include "nvboot_devparams.h"
#include "nvboot_fuse.h"
#include "nvboot_hash.h"
#include "nvboot_sdram_param.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_rsa.h"
#include "nvboot_crypto_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 *  Based on Secure Boot ISS.
 */
typedef struct NvBootLoaderInfoVersionRec{
       NvU8 VersionMajor;
       NvU8 VersionMinor;
       NvU8 RatchetLevel; //to support loader enforced ratchet mechanism
       NvU8 Reserved;
} NvBootLoaderInfoVersion; // 4 (0x4) bytes

/**
 * Stores information needed to locate and verify a boot loader.
 *
 * There is one \c NvBootLoaderInfo structure for each copy of a BL stored on
 * the device.
 */
typedef struct NvBootLoaderInfoRec
{
    /// Specifies the first physical block on the secondary boot device
    /// that contains the start of the BCH. The first block can never be
    /// a known bad block.
    NvU32      StartBlock;

    /// Specifies the page within the first block that contains the start
    /// of the BCH.
    NvU32      StartPage;

    /// Specifies a version number for the BL. The assignment of numbers is
    /// arbitrary; the numbers are only used to identify redundant copies
    /// (which have the same version number) and to distinguish between
    /// different versions of the BL (which have different numbers).
    NvBootLoaderInfoVersion Version;

    /// Reserved to make this struct AES block aligned.
    NvU32      Reserved[1];
} NvBootLoaderInfo;

/**
 * Stores information needed to locate MB1 Bct on the boot media.
 *
 *  MB1 Bct information.
 *
 */
typedef struct NvBootMb1BcttInfoRec
{
    /// Specifies the first physical block on the secondary boot device
    /// that contains the start of the Mb1 Bct image.
    NvU32        SectorInfo;

    /// Specifies the partitionsize of the block holding the mb1-bct copies
    NvU32        PartitionSize;

} NvBootMb1BcttInfo;

/**
 * Identifies the types of devices from which the system booted.
 * Used to identify primary and secondary boot devices.
 * Note that these no longer match the fuse API device values (for
 * backward compatibility with AP15).
 */
typedef enum
{
    /// Specifies a default (unset) value
    NvBootDevType_None = 0,

    /// Specifies SPI NOR
    NvBootDevType_Spi,

    /// Specifies SPI NOR
    NvBootDevType_Qspi = NvBootDevType_Spi,

    /// Specifies SDMMC (either eMMC or eSD)
    NvBootDevType_Sdmmc,

    /// Specifies internal ROM (i.e., the BR)
    NvBootDevType_Irom,

    /// Specifies UART (only available internal to NVIDIA)
    NvBootDevType_Uart,

    /// Specifies USB (i.e., Xusb RCM)
    NvBootDevType_Usb,

    /// Specifies USB3 boot interface
    NvBootDevType_Usb3,

    /// Specifies SATA boot interface
    NvBootDevType_Sata,

    /// Specifies UFS boot interface
    NvBootDevType_Ufs,

    NvBootDevType_Foos,

    NvBootDevType_Max,

    NvBootDevType_Force32 = 0x7FFFFFFF
 } NvBootDevType;


 /**
  *  MSS Encryption generate/enable/distribute flags
  */
  typedef struct NvBootBctMssFlagsRec {
  /// Specifies MSS Mts Region Generate key0
    /// Regardless of region setting, for production platform BR should
    /// ignore this setting and enabled by BR by default.
    NvBool        MssMtsRegionGenKey0;

    /// Specifies MSS Tz Region generatke key1
    NvBool        MssTzRegionGenKey1;

    /// Specifies MSS Vpr Region generate key2
    NvBool        MssVprRegionGenKey2;

    /// Specifies MSS Gsc Region generate key3
    NvBool        MssGscRegionGenKey3;

    /// Specifies MSS region encrypt clock source selection
    NvBool        MssRegionskipEncryptClkSrc;
	
    /// Specifies MSS MTS Carveout distribute key0
    NvBool        MssMtsCoDisKey0;

    /// Specifies MSS TZ Carveout distribute key1
    NvBool        MssTzCoDisKey1;

    /// Specifies MSS VPR Carveout distribute key2
    NvBool        MssVprCoDisKey2;

    /// Specifies MSS GSC Carveout distribute key3
    NvBool        MssGscCoDisKey3;

    /// Specifies MSS MTS Carveout Enable key0
    NvBool        MssMtsCoEnKey0;

    /// Specifies MSS TZ Carveout Enable key1
    NvBool        MssTzCoEnKey1;

    /// Specifies MSS VPR Carveout Enable key2
    NvBool        MssVprCoEnKey2;

    /// Specifies MSS GSC Carveout Enable key3
    NvBool        MssGscCoEnKey3;

    /// Specifies EncryptDistribute lock
    NvBool        MssSkipEncryptLock;
} NvBootBctMssFlags;
/**
 * Contains the information needed to load BLs from the secondary boot device.
 *
 * - Supplying NumParamSets = 0 indicates not to load any of them.
 * - The \c RandomAesBlock member exists to increase the difficulty of
 *   key attacks based on knowledge of this structure.
 */
typedef struct NvBootConfigTableRec
{
    /// *** UNSIGNED SECTION OF THE BCT *** ///
    ///
    /// IMPORTANT NOTE: If the start of the unsigned section changes from
    ///                 RandomAesBlock to some other starting point,
    ///                 other parts of Boot ROM must be updated!
    ///                 See SignatureOffset in function ReadOneBct
    ///                 in nvboot_bct.c, as well as the compile time
    ///                 assert at around line 59 nvboot_bct.c.
    ///                 (This is NOT a comprehensive list).
    ///
    /// IMPORTANT NOTE 2: The size of the unsigned section must be a multiple
    ///                   of the AES block size, to maintain compatibility
    ///                   with the nvboot_reader function LaunchCryptoOps!
    ///

    /// Specifies a size/length information of the BCT.
    NvU32       BctSize;

    /// Specify Preproduction Debug features in BootROM
    NvU32       BootROMPreproductionDebugFeatures;

    /// Reserved field.
    NvU32       NvUnsignedReserved;

    /// The BCT will house public, non-secret cryptographic parameters necessary
    /// for the authentication of the BCT and Boot Images. These parameters are
    /// collectively known as Public Cryptographic Parameters (PCP) and they will
    /// be stored in the unsigned section of the BCT.
    NvBootPublicCryptoParameters    Pcp;

    /// All cryptographic signatures supported will be stored here. The BCT can be
    /// simultaneously signed by all cryptographic signature types.
    NvBootCryptoSignatures          Signatures;

    /// Specifies a region of data available to customers of the BR.
    /// This data region is primarily used by a manufacturing utility
    /// or BL to store useful information that needs to be
    /// shared among manufacturing utility, BL, and OS image.
    /// BR only provides framework and does not use this data
    /// @note Some of this space has already been allocated for use
    /// by NVIDIA.
    /// Information currently stored in the \c CustomerData[] buffer is
    /// defined below.
    /// @note Some of the information mentioned shall be deprecated
    /// or replaced by something else in future releases
    ///
    /// -# Start location of OS image (physical blocks). Size:- NvU32
    ///    OS image is written from block boundary.
    /// -# Length of OS image. Size:- NvU32
    /// -# OS Flavor: wince or winwm (windows mobile). Size:-NvU32
    ///    wince type image is a raw binary
    ///    winwm has different image layout (".dio" format)
    /// -# Information about how many columns (banks) are used for
    ///    NAND interleave operations. Size:- uint8_t
    /// -# Pointer to DRM device certificate location. Size:-NvU32
    /// -# Pointer to secure clock information. Size:- NvU32
    /// -# \a custopt data filed. Size: NvU32
    ///    RM allows ODM adaptations and ODM query implementations
    ///    to read this value at runtime and use it for various useful
    ///    features.
    ///    For example: use of single BSP image that supports multiple product
    ///    SKUs.
    /// @note The storage space is much larger for AP20 than AP15 or AP16.
    uint8_t     CustomerData[NVBOOT_BCT_CUSTOMER_DATA_SIZE];

    /// *** START OF SIGNED SECTION OF THE BCT *** ///
    ///
    /// Specifies a chunk of random data.
    NvBootHash  RandomAesBlock;

    /// Specifies the device parameters with which to reinitialize the
    /// secondary boot device controller. The device straps index into this
    /// table. The definition of \c NvBootDevParams is contained within
    /// nvboot_devparams.h and the specific device nvboot_*_param.h files.
    NvBootDevParams DevParams[NVBOOT_BCT_MAX_PARAM_SETS];

    /// Specifies the boot chain to use by default when GPIOSelectBootChain
    /// is *DISABLED*.  GPIOSelectBootChain has priority when enabled.
    /// 0 = primary, 1 = secondary
    uint8_t     NonGPIOSelectBootChain;

    /// Specifies the number of BLs described in the BootLoader table.
    NvU32       BootLoadersUsed;

    /// Specifies the information needed to locate and validate each BL.
    /// The BR uses entries 0 through BootLoadersUsed-1 for each chain.
    /// T19x supports 2 chains where each chain can support upto MAX of 4 BLs.
    /// Primary chain will use BootLoader[0-3].
    /// Secondary chain will use BootLoader[4-7].
    NvBootLoaderInfo    BootLoader[NVBOOT_MAX_BOOTLOADERS*NVBOOT_MAX_NUM_OF_CHAINS];

    /// Specify Mb1 Bct info for each chain
    /// Primary chain would be Mb1Bct[0] 
    NvBootMb1BcttInfo   Mb1Bct[NVBOOT_MAX_NUM_OF_CHAINS];

    /// Unused space allocated for customer usage.
    NvU32       Signed_CustomerData[NVBOOT_BCT_SIGNED_CUSTOMER_DATA_WORDS];
    /// *** END OF UN-ENCRYPTED & SIGNED SECTION OF THE BCT *** ///

    /// *** START OF SIGNED & (OPTIONALLY) ENCRYPTED SECTION OF THE BCT *** ///
    /// Specifies a chunk of random data.
    NvBootHash  RandomAesBlock2;
    /// Specifies the Unique ID / ECID of the chip that this BCT is specifically
    /// generated for. This field is required if SecureJtagControl == NV_TRUE.
    /// It is optional otherwise. This is to prevent a signed BCT with
    /// SecureJtagControl == NV_TRUE being leaked into the field that would
    /// enable JTAG debug for all devices signed with the same private RSA key.
    NvBootECID  UniqueChipId;

    /// Specifies the version of the BR data structures used to build this BCT.
    /// \c BootDataVersion must match the version number in the BR.
    NvU32       BootDataVersion;

    /// Specifies the size of a physical block on the secondary boot device
    /// in log2(bytes).
    NvU32       BlockSizeLog2;

    /// Specifies the size of a page on the secondary boot device
    /// in log2(bytes).
    NvU32       PageSizeLog2;

    /// Specifies the size of the boot partition in bytes.
    /// Used for internal error checking; BLs must fit within this region.
    NvU32       PartitionSize;

    /// Specifies the number of valid device parameter sets provided within
    /// this BCT. If the device straps are left floating, the same parameters
    /// should be replicated to all NVBOOT_BCT_MAX_PARAM_SETS sets.
    NvU32       NumParamSets;

    /// Specifies the type of device for parameter set DevParams[i]
    NvBootDevType   DevType[NVBOOT_BCT_MAX_PARAM_SETS];

    /// Specifies if GPIO is used to select the chain to boot.  If enabled, the
    /// next two addresses are used to determine the boot chain selection.
    NvBool      GPIOSelectBootChain;
    
    /// Specify the GPIO config address and the Pad control register address 
    /// for the GPIO to use for boot chain selection.
    /// If the GPIOSelectBootChain is enabled, these will be used to configure
    /// the GPIO, select the pad control, and read the GPIO value at 
    /// GPIOConfigAddressBootChain+0x08 to determine the boot chain to use. 
    /// These addresses will be range checked for security purpsoes.
    /// 0 = primary, 1 = secondary
    NvU32       GPIOConfigAddressBootChain;
    NvU32       GPIOPadctlAddressBootChain;

    /// SoftFuses reserved for MB1 Usage
    /// Request 64 32b fuses in signed and optionally encrypted section
    NvU32       Mb1SoftFuses[NVBOOT_MB1_SOFT_FUSES];

    /// Speicfy the desire to debug MB1 post NV_PRODUCTION and
    /// before SECURITY_MODE is fused (OEM production).
    NvBool      MB1DebugProduction;

    /// Speicfy the desire to debug MTS post NV_PRODUCTION and before
    /// SECURITY_MODE is fused (OEM production);
    /// MTSDebugProduction support in MB1 is NOT POR, but MTS Debug Keys
    /// are still implemented in IROM secure region and BR will still load
    /// the keys according to this bit set.
    NvBool      MTSDebugProduction;

    /// Speicfy the desire to debug IST-FW post NV_PRODUCTION and before
    /// SECURITY_MODE is fused (OEM production);
    /// ISTFWDebugProduction also allows for debug when SECURITY_MODE is fused
    /// if the ECID in MB1 matches the ECID of the chip
    NvBool      ISTFWDebugProduction;

    /// Specify the desire to monitor VDD_RTC rail for security violation in SC7 exit 
    NvBool      RtcRailViolationDetect;

    /// Specifies MSS encryption flags.
    NvBootBctMssFlags MssFlags;

    /// Specifies to enable I/D cachec for R5 post bct.
    NvBool      EnableR5Cache;

    /// Specifies override Bpmp Cpu Clk dividers
    NvU32       BootClientBpmpCpu;

    /// Specifies override Bpmp Apb Clk dividers
    NvU32       BootClientBpmpApb;

    /// Specifies override Bpmp Axi Cbb dividers
    NvU32       BootClientAxiCbb;

    /// Specifies override Se Clk dividers
    NvU32       BootClientSe;

    /// Specifies override Emc Clk roc dividers
    NvU32       BootClientEmcRoc;

    /// Specify KEK size select
    /// 0 - sets of 128 bit keys.
    /// 1 - 256 bit key
    uint8_t     BctKEKKeySelect;

    /// Specify whether or not to enable NvCPU Cluster dfd access
    /// CustNvCcplexDfdEn = NV_FALSE (0) = Disable Dfd access.
    /// CustNvCcplexDfdEn = NV_TRUE (1) = Enable Dfd access.
    NvBool      CustNvCcplexDfdEn;

    /// Specifies which debug features to be enabled or disabled.
    /// Maps directly to APBDEV_PMC_DEBUG_AUTHENTICATION_0. These bits
    /// are not tied to a specific chip ECID, and UniqueChipId in the BCT
    /// does not need to match the actual chip ECID for them to take effect.
    /// 0x1 = ON, 0x0 = OFF
    /// PVA1_Secure_Debug - bit 13
    /// PVA0_Secure_Debug - bit 12
    /// RCE_Secure_Debug - bit 11
    /// SCE_Secure_Debug - bit 10
    /// SPE_Secure_Debug - bit 9
    /// BPMP_Secure_Debug - bit 8
    /// Reserved bits [7:6]
    /// 0x1 = ENABLED. 0x0 = DISABLED.
    /// NIDEN - bit 4
    NvU32       SecureDebugControl_Not_ECID_Checked;

    /// Specifies which debug features to be enabled or disabled.
    /// Maps directly to APBDEV_PMC_DEBUG_AUTHENTICATION_0.
    /// The ECID of the chip must match the ECID specified in UniqueChipId
    /// for the bits in this field to take effect.
    /// ECID check is mandatory for bits 0, 1, 2, 3, 5, 31.
    /// 0x1 = ENABLED. 0x0 = DISABLED.
    /// Ramdump Enable - bit 31.
    /// DBGEN - bit 5
    /// SPIDEN - bit 3
    /// SPNIDEN - bit 2
    /// DEVICEEN - bit 1
    /// JTAG_ENABLE - bit 0
    NvU32       SecureDebugControl_ECID_Checked;

    /// Specifies the factory secure provisioning key number to use.
    /// There are 64 such 256-bit AES keys.
    /// Specifying a key number of 0 will cause Boot ROM to default to
    /// NvProduction mode boot (i.e. Factory Secure Provisioning mode disabled).
    /// Specifying a key number of 1 to 15 is invalid. These are anti-cloning keys
    /// numbers and BR will ignore these values.
    /// BR will ignore this field if the secure_provision_index fuse is burned.
    /// Key number 64 (index [63]) is reserved for NVIDIA debug use.
    /// So, this field will only be used if the chip is in NvProductionMode,
    /// and when secure_provision_index is zero, and when SecProvisioningKeyNum
    /// is not 0 to 15.
    NvU32       SecProvisioningKeyNum_Secure;

    /// For FSKP, derivation strings are used to generate new authentication
    /// key and decryption key.  The original FSKP key is used to encrypt these
    /// derivation strings to generate the new keys.
    uint8_t     SecProvisionDerivationString1[NVBOOT_BCT_DERIVATION_STRING_SIZE];
    uint8_t     SecProvisionDerivationString2[NVBOOT_BCT_DERIVATION_STRING_SIZE];
    
    /// A way to turn on dev keys for the KEK
    /// Also requires SPNIDEN or SPIDEN to be set to work
    bool        DebugWithTestKeys;

    /// Specifies a reserved area at the end of the BCT that must be filled
    /// with the padding pattern.
    uint8_t     Reserved[NVBOOT_BCT_RESERVED_SIZE];
} NvBootConfigTable;

#define NVBOOT_BCT_SOFT_FUSES_SIZE     (sizeof(((NvBootConfigTable*)0)->Mb1SoftFuses))

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BCT_H */
