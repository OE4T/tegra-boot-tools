/*
 * Copyright (c) 2007-2014, NVIDIA CORPORATION. All rights reserved.
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
 * nvboot_config.h provides constants that parameterize operations in the
 * Boot ROM.
 *
 * IMPORTANT: These constants can ONLY be changed during Boot ROM
 *            development.  For a particular Tegra product, their
 *            values are frozen and provided here for reference by
 *            humans and bootloader code alike.
 */

#ifndef INCLUDED_NVBOOT_CONFIG_H
#define INCLUDED_NVBOOT_CONFIG_H

#if defined(__cplusplus)
extern "C"
{
#endif


/**
 * Defines the maximum number of device parameter sets in the BCT.
 * The value must be equal to (1 << # of device straps)
 */
#define NVBOOT_BCT_MAX_PARAM_SETS      1

/**
 * Defines the maximum number of SDRAM parameter sets in the BCT.
 * The value must be equal to (1 << # of SDRAM straps)
 */
#define NVBOOT_BCT_MAX_SDRAM_SETS      4

/**
 * Defines the number of bytes defined for Secondary Device Status in the BIT
 */
#define NVBOOT_DEV_STATUS_SIZE_BYTES   256

/**
 *  Defines the maximum page size of supported devices.
 *  Xavier maximum is 4K for UFS
 */
#define NVBOOT_DEV_MAX_PAGE_SIZE    4096
/**
 * Defines the number of bytes provided for the CustomerData field in
 * the BCT.  In this context, "Customer" means a customer of the Boot ROM,
 * namely bootloaders.
 *
 * Note that some of this data has been allocated by other NVIDIA tools
 * and components for their own use.  Please see TBD for further details.
 *
 * The CUSTOMER_DATA_SIZE is set to maximize the use of space within the BCT.
 * The actual BCT size, excluding the Customer data & reserved fields,
 * is 2885 bytes.  This leaves 1192 bytes for customer data (298 32bit words),
 * and the Reserved bytes required are 3 bytes. This brings the total BCT
 * size to 4080 (0xFF0) bytes as required by NVBOOT_BCT_REQUIRED_SIZE below.
 */

/**
 * Defines the number of 32-bit words in the CustomerData area of the BCT.
 */
#define NVBOOT_BCT_CUSTOMER_DATA_WORDS 83

/**
 * Defines the number of bytes in the CustomerData area of the BCT.
 */
#define NVBOOT_BCT_CUSTOMER_DATA_SIZE \
                (NVBOOT_BCT_CUSTOMER_DATA_WORDS * 4)

/**
 * Defines the number of 32-bit words in the CustomerData area of the BCT.
 */
#define NVBOOT_BCT_SIGNED_CUSTOMER_DATA_WORDS 14

/**
 * Defines the number of bytes in the CustomerData area of the BCT.
 */
#define NVBOOT_BCT_SIGNED_CUSTOMER_DATA_SIZE \
                (NVBOOT_BCT_SIGNED_CUSTOMER_DATA_WORDS * 4)

/**
 * Defines the number of bytes in the Reserved area of the BCT.
 */
#define NVBOOT_BCT_RESERVED_SIZE       115

/**
 * Defines the number of bytes in the Reserved area of the BIT.
 */
#define NVBOOT_BIT_RESERVED_SIZE       88

/**
 * Defines the required size of the BCT.  NVBOOT_BCT_REQUIRED_SIZE is set to
 * ensure that the BCT uses the entirety of the pages in which it resides,
 * minus the 16 bytes needed to work around a bug in the reader code.
 */
/**
 * Defines the number of maximum-sized pages needed by the BCT.
 */
#define NVBOOT_BCT_REQUIRED_NUM_PAGES 1
/**
 * Defines the maximum page size needed by the BCT.
 */
#define NVBOOT_BCT_REQUIRED_PAGE_SIZE NVBOOT_DEV_MAX_PAGE_SIZE

/**
 * Defines the required BCT size, in bytes.
 */
#define NVBOOT_BCT_REQUIRED_SIZE \
          ((NVBOOT_BCT_REQUIRED_NUM_PAGES) * (NVBOOT_BCT_REQUIRED_PAGE_SIZE))

/**
 * Defines the maximum page size of any secondary boot device
 * This year's award goes to UFS.
 */
#define NVBOOT_MAX_SECONDARY_BOOT_DEVICE_PAGE_SIZE 4096

/**
 * Defines the required size of the BIT.  NVBOOT_BIT_REQUIRED_SIZE is fixed.
 */
/**
 * Defines the maximum size needed by the BIT.
 */
#define NVBOOT_BIT_REQUIRED_SIZE 4096

/**
 * Defines the maximum number of bootloader descriptions in the BCT.
 */
#define NVBOOT_MAX_BOOTLOADERS         4

/**
 * Defines the maximum number of chains supported in BootROM.
 */
#define NVBOOT_MAX_NUM_OF_CHAINS       2

/**
 * Defines the number of soft fuses for MB1 usage.
 */
#define NVBOOT_MB1_SOFT_FUSES       64

/**
 * Defines the minimum size of a block of storage in the secondary boot
 * device in log2(bytes) units.  Thus, a value of 8 == 256 bytes.
 */
#define NVBOOT_MIN_BLOCK_SIZE_LOG2     8

/**
 * Defines the maximum size of a block of storage in the secondary boot
 * device in log2(bytes) units.  Thus, a value of 23 == 8 mebibytes.
 */
#define NVBOOT_MAX_BLOCK_SIZE_LOG2    23

/**
 * Defines the minimum size of a page of storage in the secondary boot
 * device in log2(bytes) units.  Thus, a value of 8 == 256 bytes.
 */
#define NVBOOT_MIN_PAGE_SIZE_LOG2      8

/**
 * Defines the maximum size of a page of storage in the secondary boot
 * device in log2(bytes) units.  Thus, a value of 14 == 16384 bytes.
 */
#define NVBOOT_MAX_PAGE_SIZE_LOG2     14

/**
 * Defines the maximum page size in bytes (for convenience).
 */
#define NVBOOT_MAX_PAGE_SIZE  (1 << (NVBOOT_MAX_PAGE_SIZE_LOG2))

/**
 * Defines the minimum page size in bytes (for convenience).
 */
#define NVBOOT_MIN_PAGE_SIZE  (1 << (NVBOOT_MIN_PAGE_SIZE_LOG2))

/**
 * Defines the Boot flow log depth
 */
#define NVBOOT_FLOW_LOG_DEPTH  44

/**
 * Defines the maximum number of blocks to search for BCTs.
 *
 * This value covers the initial block and a set of journal blocks.
 *
 * Ideally, this number will span several erase units for reliable updates
 * and tolerance for blocks to become bad with use.  Safe updates require
 * a minimum of 2 erase units in which BCTs can appear.
 *
 * To ensure that the BCT search spans a sufficient range of configurations,
 * the search block count has been set to 64. This allows for redundancy with
 * a wide range of parts and provides room for greater problems in this
 * region of the device.
 */
#define NVBOOT_MAX_BCT_SEARCH_BLOCKS   64

/**
 * Defines the length of the FSKP derivation string lenght
 */
#define NVBOOT_BCT_DERIVATION_STRING_SIZE   32

/**
 * Memory Layout for SYSRAM
 *
 ***************************** --(NV_ADDRESS_MAP_SYSRAM_0_IMPL_LIMIT)
 *							*	
 *    		       BCT				*	
 *			(6K)				*	
 *							*	
 *****************************--( NVBOOT_BCT_START ) =
 *						  	*   (NV_ADDRESS_MAP_SYSRAM_0_IMPL_LIMIT - NVBOOT_BCT_SIZE+1)
 *    		   Public Key store		*	
 *			(4K)				*	
 *							*	
 *****************************--( NVBOOT_PUBLICKEY_START ) =
 *						  	*   (NV_ADDRESS_MAP_SYSRAM_0_IMPL_LIMIT - NVBOOT_BCT_SIZE - NVBOOT_PUBLICKEY_SIZE+1)
 *							*	
 *			(278K)			* 
 **			MB1/RCM Applet	*
 *							*	
 *							*	
 *****************************-- ( NVBOOT_BL_SYSRAM_START ) =
 *							*	(NV_ADDRESS_MAP_SYSRAM_0_IMPL_BASE + NVBOOT_RCM_MB1_SYSRAM_START_OFFSET )
 *			(128K)		*	
 *	BootROM reserved aread  *	
 *							*	
 *****************************--( NV_ADDRESS_MAP_SYSRAM_0_BASE ) == (NV_ADDRESS_MAP_SYSRAM_0_IMPL_BASE)
*/

#define ALIGN_ADDR(ADDR, BYTES) ((((ADDR)+(BYTES)-1)/(BYTES)) * (BYTES))

/// TODO. define BTCM and SYSRAM address/size data from AMAP defined macros!!
/**
 * Defines the starting physical address of IRAM
 */
#define NVBOOT_BL_BTCM_START  (NV_ADDRESS_MAP_BPMP_BTCM_BASE)

/**
 * Defines the ending physical address of IRAM
 */
#define NVBOOT_BL_BTCM_END    (NV_ADDRESS_MAP_BPMP_BTCM_BASE + NV_ADDRESS_MAP_BPMP_BTCM_SIZE)

/**
 * Defines the starting physical address of SDRAM
 */
#define NVBOOT_BL_SDRAM_START 	(NV_ADDRESS_MAP_EMEM_BASE)

/**
 * Defines the size of IRAM in bytes.
 */
#define NVBOOT_IRAM_SIZE   		(NVBOOT_BL_BTCM_END - NVBOOT_BL_BTCM_START)

/**
 * Defines the max allocated size of RCM Data Structures in sysram in bytes.
 */
#define NVBOOT_DEV_DS_SIZE	(8192) // 8K space shared by controllers for hw data structures


/**
 * Defines the RCM/MB1 layout in sysram for BR.
 */
#define NVBOOT_RCM_MB1_SYSRAM_START_OFFSET (131072) //128 K

#define NVBOOT_BL_SYSRAM_START	(NV_ADDRESS_MAP_SYSRAM_0_IMPL_BASE + NVBOOT_RCM_MB1_SYSRAM_START_OFFSET )

#define NVBOOT_RCM_MB1_SIZE (284672) // 278K, 0x45800

#define NVBOOT_BL_SYSRAM_END    (NV_ADDRESS_MAP_SYSRAM_0_IMPL_BASE + NVBOOT_RCM_MB1_SIZE-1)//0x657FF


#define NVBOOT_RCM_SYSRAM_START	(NVBOOT_BL_SYSRAM_START - sizeof(NvBootRcmMsg) - sizeof(NvBootComponentHeader))

#define NVBOOT_MB1_SYSRAM_START (NVBOOT_BL_SYSRAM_START) // 0x40020000

/**
 * Defines the max allocated size of BCT in sysram in bytes.
 */
#define NVBOOT_BCT_SIZE			(6144) // BRBCT 6K

/**
 * Defines the start address of the Public key offset
 * This is in sync with https://p4viewer.nvidia.com/get/hw/ar/doc/t18x/boot/SysRAM buffers.xlsx
 */
#define NVBOOT_PUBLICKEY_SIZE	(4096)
#define NVBOOT_PUBLICKEY_START	(NV_ADDRESS_MAP_SYSRAM_0_IMPL_LIMIT \
                                - NVBOOT_BCT_SIZE - NVBOOT_PUBLICKEY_SIZE + 1)

/**
  * Defines the start address of the BIT in BTCM
  */
#define NVBOOT_BIT_START NVBOOT_BL_BTCM_START


/*
 * Stack for Preproduction UART is to be moved to top of BTCM before
 * beginning uart download in order to avoid stack corruption.
 * This is because BootROM continues to have a stack at an offset of
 * around 20K(this will shift based on changes to BootROM)
 */
#define UART_BL_BTCM_STACK_TOP NV_ADDRESS_MAP_BPMP_BTCM_LIMIT+1

#define NVBOOT_PROD_UART_DOWNLOAD_BINARY_LIMIT_FROM_TOP     (0x10000)

/**
 * Production UART can be used to safely download data till end of BTCM.
 * but it's better to use BL_BTCM_END for safe download so that end address
 * will automatically get adjusted upon changes to bootloader safe end address in this file.
 */
#define NVBOOT_PROD_UART_DATA_END (NVBOOT_BL_BTCM_END)//NVBOOT_BCT_START//(NVBOOT_BL_BTCM_END)

/**
 * Production UART will allow only total 64K download of raw binary with BCT and Bootloader
 * This was an agreement in previous chip.
 * Production UART will use the last 64K in BTCM to read the entire data layout.
 */
#define NVBOOT_PROD_UART_DATA_START (NVBOOT_PROD_UART_DATA_END - NVBOOT_PROD_UART_DOWNLOAD_BINARY_LIMIT_FROM_TOP)


/**
 * Selection of engines & key slots for AES operations.
 *
 * The SBK key tables are stored in key slots for which read access can
 * be disabled, but write access cannot be disabled.  Key slots 0 and 1
 * have these characteristics.
 *
 * The SBK key slots are configured for write-only access by the Boot ROM.
 *
 * The bootloader is required to over-write the SBK key slots before
 * passing control to any other code.
 *
 * Either engine can be used for each operation.
 */

/**
 * Defines the engine to use for SBK engine A.  The value is an enumerated
 * constant.
 */
#define NVBOOT_SBK_ENGINEA NvBootAesEngine_A

/**
 * Defines the engine to use for SBK engine B.  The value is an enumerated
 * constant.
 */
#define NVBOOT_SBK_ENGINEB NvBootAesEngine_B

/**
 * Defines the key slot used for encryption with the SBK
 */
#define NVBOOT_SBK_ENCRYPT_SLOT   NvBootAesKeySlot_0
/**
 * Defines the key slot used for decryption with the SBK
 */
#define NVBOOT_SBK_DECRYPT_SLOT   NVBOOT_SBK_ENCRYPT_SLOT

/**
 * The SSK key tables are stored in key slots for which read and/or
 * write access can be disabled.  Key slots 2 and 3 have these
 * characteristics.
 *
 * The SSK key slots are configured for write-only access by the Boot ROM.
 *
 * The SSK key slots can optionally be over-written by the bootloader with
 * any user-defined values.  Regardless, though, the bootloader must ensure
 * that write-access is disabled for these slots (at which time both read-
 * and write-access will be disabled).
 */

/**
 * Defines the engine to use for SSK engine A.  The value is an enumerated
 * constant.
 */
#define NVBOOT_SSK_ENGINEA NvBootAesEngine_A

/**
 * Defines the engine to use for SSK engine B.  The value is an enumerated
 * constant.
 */
#define NVBOOT_SSK_ENGINEB NvBootAesEngine_B

/**
 * Defines the key slot used for encryption with the SSK
 */
#define NVBOOT_SSK_ENCRYPT_SLOT   NvBootAesKeySlot_4
/**
 * Defines the key slot used for decryption with the SSK
 */
#define NVBOOT_SSK_DECRYPT_SLOT   NVBOOT_SSK_ENCRYPT_SLOT


/**
 * Defines the default AES-CMAC hash key size.
 */
#define NVBOOT_AES_CMAC_DEFAULT_KEY_SIZE AesKey128

/**
 * Defines the minimum byte alignment for crypto buffers.
 * Use with __attribute__((align(...)))
 */
#define NVBOOT_CRYPTO_BUFFER_ALIGNMENT 4

/**
 * Defines the maximum number of fuse words that can be programmed.
 */
#define NVBOOT_FUSE_ARRAY_MAX_WORDS 64

/**
 * Defines the maximum number of commands that can be allowed to be queued.
 * This is as per SATA AHCI spec ver 1.3. Also subject to the fact that queueing
 * is implemented in T30 SATA controller and is taken advantage of in the
 * T30-bootrom SATA driver.
  */
#define NVBOOT_SATA_MAX_COMMANDS_IN_Q    32
#define NVBOOT_SATA_MAX_SUPPORTED_COMMANDS_IN_Q    1

/**
 * Defines the maximum number of MTS boot components descriptions in the BCT.
 */
#define NVBOOT_MAX_MTS_COMPONENTS         6

/**
 * Defines the index to MTS Preboot components descriptions in the BCT.
 */
#define NVBOOT_MTS_PREBOOT_SLOT         0

/**
 * Define virtual address for accessing SDRAM to be after real SYSRAM implementation
 * Only used in SC7 when we use BPMP AST to map the location of SC7-RF to this virtual address.
 * This is to allow SC7-RF to possibly be placed above the 32-bit address limit.
 */
//#define NVBOOT_SDRAM_VIRTUAL_ADDRESS (NV_ADDRESS_MAP_SYSRAM_0_IMPL_LIMIT+1)

/**
 * GSC 15 is used to store the SE context, size is 16KB.
 * See //hw/ar/doc/t19x/clusters/mss/arch/T19x_gsc_programming.xlsx.
 */
#define GSC15_BOM 0x84000000
#define GSC15_BOM_HI 0
#define GSC15_SIZE 16*1024
// SE IAS recommends to place SE context at GSC15_BOM + 1kb.
#define SE_CONTEXT_OFFSET 1024
#define SE_CONTEXT_LOCATION_PHYSICAL (GSC15_BOM + SE_CONTEXT_OFFSET)

/**
 * GSC 23 is used to store the SC7-RF, size is 16KB.
 * See //hw/ar/doc/t19x/clusters/mss/arch/T19x_gsc_programming.xlsx.
 */
#define GSC23_SIZE 192*1024

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CONFIG_H */
