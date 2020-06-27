/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
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
#define NVBOOT_BCT_CUSTOMER_DATA_WORDS 88

/**
 * Defines the number of bytes in the CustomerData area of the BCT.
 */
#define NVBOOT_BCT_CUSTOMER_DATA_SIZE \
                (NVBOOT_BCT_CUSTOMER_DATA_WORDS * 4)

/**
 * Defines the number of bytes in the Reserved area of the BCT.
 */
#define NVBOOT_BCT_RESERVED_SIZE       116

/**
 * Defines the number of bytes in the Reserved area of the BCT.
 */
#define NVBOOT_BIT_RESERVED_SIZE       90



#define NVBOOT_BCT_DRAM_RESERVED_SIZE     18

/**
 * Defines the number of 32-bit words provided in each set of SDRAM parameters
 * for arbitration configuration data.  These values are passed to the
 * bootloader - the Boot ROM does not use the values itself.  Note that this
 * data is part of the SDRAM parameter structure, so there are four sets
 * of this data.
 */
#define NVBOOT_BCT_SDRAM_ARB_CONFIG_WORDS 27

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
#define NVBOOT_BCT_REQUIRED_PAGE_SIZE 3584

/**
 * Defines the required BCT size, in bytes.
 */
#define NVBOOT_BCT_REQUIRED_SIZE \
          ((NVBOOT_BCT_REQUIRED_NUM_PAGES) * (NVBOOT_BCT_REQUIRED_PAGE_SIZE))


/**
 * Defines the required size of the BIT.  NVBOOT_BIT_REQUIRED_SIZE is fixed.
 */
/**
 * Defines the maximum size needed by the BIT.
 */
#define NVBOOT_BIT_REQUIRED_SIZE 1024


/**
 * Defines the maximum number of bootloader descriptions in the BCT.
 */
#define NVBOOT_MAX_BOOTLOADERS         4

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

/*
 * Defines the Boot flow log depth
 */
#define NVBOOT_FLOW_LOG_DEPTH  40

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
 * Memory range constants.
 * The range is defined as [Start, End)
 */
/*
 * Note: The following symbolic definitions 
 *    #define NVBOOT_BL_SDRAM_START (NV_ADDRESS_MAP_EMEM_BASE)
 */

/* 
 * Memory Layout for SYSRAM
 *
 ***************************** --(NV_ADDRESS_MAP_SYSRAM_0_LIMIT)
 *							*	
 *    		       BCT				*	
 *			(6K)				*	
 *							*	
 *****************************--( NVBOOT_BCT_START ) =
 *						  	*   (NVBOOT_BL_SYSRAM_END - NVBOOT_BCT_SIZE)
 *    		   Public Key store		*	
 *			(4K)				*	
 *							*	
 *****************************--( NVBOOT_PUBLICKEY_START ) =
 *						  	*   (NVBOOT_BL_SYSRAM_END - NVBOOT_BCT_SIZE - NVBOOT_PUBLICKEY_SIZE)
 *							*	
 *    		   MB1/RCM Applet    	*	
 *							*	
 *							*	
 *****************************-- ( NVBOOT_RCM_SYSRAM_START ) =
 *							*	(NVBOOT_BPMP_R5_SYSRAM + NVBOOT_RCM_MB1_SYSRAM_START_OFFSET )
 *    		  Not Assigned		    	*	
 *							*	
 *							*	
 *****************************
 *           PKA Buffers 			*
 *		(1K)					*	
 *                          			*
 *****************************-- ( NVBOOT_PKA_BUF_START ) =
 *							*	(NVBOOT_SE_BUF_START + NVBOOT_SE_BUF_SIZE)
 *           SE Buffers 			*
 *		(1K)					*	
 *                          			*
 *****************************-- ( NVBOOT_SE_BUF_START ) =
 *							*	(NVBOOT_CRYPTO_MGR_BUF_START + NVBOOT_CRYPTO_MGR_BUF_SIZE)
 *           Crypto Buffers 		*
 *		(8K)					*	
 *                          			*
 *****************************-- ( NVBOOT_CRYPTO_MGR_BUF_START ) =
 *							*	(NVBOOT_BR_BUF_START + NVBOOT_BR_BUF_SIZE)
 *    		BR Buffers   		 	*	
 *		(12K)				*	
 *							*	
 *****************************-- ( NVBOOT_BR_BUF_START ) =
 *							*	( NVBOOT_RCM_DS_BUF_START + NVBOOT_RCM_DS_SIZE )
 *    		RCM Data Structures	*		
 *		(8K)					*	
 *							*	
 *****************************-- ( NVBOOT_RCM_DS_BUF_START ) =
 *							*	( NV_ADDRESS_MAP_SYSRAM_0_BASE + NVBOOT_DEV_DS_SIZE )
 *    	Device Data Structures		*		
 *		 (8K)				*	
 *							*	
 *****************************--( NV_ADDRESS_MAP_SYSRAM_0_BASE ) == (NVBOOT_BPMP_R5_SYSRAM)
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

// Sysram is remapped for R5 virtually @ 0x40000000 
// Enabled via MPU and AST in startup code.
#define NVBOOT_BPMP_R5_SYSRAM (0x40000000)

#define NVBOOT_SYSRAM_DIFFERENCE (0x10000000)

#define NVBOOT_SYSRAM_DIFFERENCE_MASK (0x10000000)

//Sysram access from boot interface client is 
#define NVBOOT_BPMP_CLIENT_SYSRAM (NV_ADDRESS_MAP_SYSRAM_0_IMPL_BASE)


/**
 * Defines the start of RCM Data Structures in sysram.
 */
#define NVBOOT_DEV_DS_START	(NVBOOT_BPMP_R5_SYSRAM)

/**
 * Defines the max allocated size of RCM Data Structures in sysram in bytes.
 */
#define NVBOOT_DEV_DS_SIZE	(8192) // 8K space shared by controllers for hw data structures

/**
 * Defines the start of Device Data Structures in sysram.
 */
#define NVBOOT_RCM_DS_START	(NVBOOT_DEV_DS_START + NVBOOT_DEV_DS_SIZE)

/**
 * Defines the max allocated size of Device Data Structures in sysram in bytes.
 */
#define NVBOOT_RCM_DS_SIZE  (8192) // 8K space for RCM hw data structures

/**
 * Defines the start of BR Buffer in sysram.
 */
#define NVBOOT_BR_BUF_START	(NVBOOT_RCM_DS_START + NVBOOT_RCM_DS_SIZE)

/**
 * Defines the size of BR Buffer in sysram in bytes.
 */
#define NVBOOT_BR_BUF_SIZE	(12288) // 12K allocated for BR buffer .

/**
 * Defines the start address of the crypto manager context in client
 * SysRam address.
 */
//TODO: finalize the buffer address
#define NVBOOT_CRYPTO_MGR_BUF_START		(NVBOOT_BR_BUF_START + NVBOOT_BR_BUF_SIZE)
#define NVBOOT_CRYPTO_MGR_BUF_SIZE		(8192)

#define NVBOOT_CRYPTO_MGR_BUF_START_R5	(NVBOOT_CRYPTO_MGR_BUF_START)
#define NVBOOT_CRYPTO_MGR_BUF_START_CLIENT (NVBOOT_CRYPTO_MGR_BUF_START - 0x10000000)

#define NVBOOT_CRYPTO_MGR_BUF_KEY_DECRYPTION_REGION_R5 (NVBOOT_CRYPTO_MGR_BUF_START_R5 + 7168)
#define NVBOOT_CRYPTO_MGR_BUF_KEY_DECRYPTION_REGION_CLIENT (NVBOOT_CRYPTO_MGR_BUF_KEY_DECRYPTION_REGION_R5 - 0x10000000)

/**
 * Defines the start address of the SE internal buffers
 */
//TODO: finialize the buffer address
#define NVBOOT_SE_BUF_START				(NVBOOT_CRYPTO_MGR_BUF_START + NVBOOT_CRYPTO_MGR_BUF_SIZE)
#define NVBOOT_SE_BUF_SIZE				(1024)

#define NVBOOT_SE_BUF_START_R5			(NVBOOT_SE_BUF_START)
#define NVBOOT_SE_BUF_START_CLIENT		(NVBOOT_SE_BUF_START - 0x10000000)

/**
 * PKA Internal Buffer
 */
#define NVBOOT_PKA_BUF_START			(NVBOOT_SE_BUF_START + NVBOOT_SE_BUF_SIZE)
#define NVBOOT_PKA_BUF_SIZE				(1024)

#define NVBOOT_PKA_BUF_START_R5			(NVBOOT_PKA_BUF_START)
#define NVBOOT_PKA_BUF_START_CLIENT		(NVBOOT_PKA_BUF_START - 0x10000000)

#define NVBOOT_BR_BUF_OFFSET			(NVBOOT_PKA_BUF_START + NVBOOT_PKA_BUF_SIZE)
/**
 *  Define SYSRAM upper nibble mask. Use for adjusting sysram addresses for access
 *  from the BPMP or client access.
 */
#define NVBOOT_SYSRAM_UPPER_NIBBLE_MASK (0x70000000)

/// FIXME RCM applet download address
/**
 * Defines the start of RCM applet download address in sysram.
 */
//#define NVBOOT_BL_SYSRAM_START (NVBOOT_BR_BUF_START + NVBOOT_BR_BUF_SIZE)
/** 
 * BL Address in Sysram. No real requirement to have it aligned to 0x1000.
 * FIXME 
 */
//#define NVBOOT_BL_SYSRAM_START (ALIGN_ADDR((NVBOOT_PKA_BUF_START + NVBOOT_PKA_BUF_SIZE+ sizeof(NvBootRcmMsg)),0x1000))


/**
 * Defines the RCM/MB1 layout in sysram for BR.
 * This is in sync with https://p4viewer.nvidia.com/get/hw/ar/doc/t18x/boot/SysRAM buffers.xlsx 
 */
#define NVBOOT_RCM_MB1_SYSRAM_START_OFFSET (131072) //128 K

#define NVBOOT_BL_SYSRAM_START	(NVBOOT_BPMP_R5_SYSRAM + NVBOOT_RCM_MB1_SYSRAM_START_OFFSET )

#define NVBOOT_RCM_SYSRAM_START	(NVBOOT_BL_SYSRAM_START - sizeof(NvBootRcmMsg) )

/**
 * Defines the max allocated size of BCT in sysram in bytes.
 */
#define NVBOOT_BCT_SIZE			(6144) // BRBCT 6K

/**
 * Defines the start address of the Public key offset
 * This is in sync with https://p4viewer.nvidia.com/get/hw/ar/doc/t18x/boot/SysRAM buffers.xlsx 
 */
#define NVBOOT_PUBLICKEY_SIZE	(4096)
#define NVBOOT_PUBLICKEY_START	(NVBOOT_BL_SYSRAM_END - NVBOOT_BCT_SIZE - NVBOOT_PUBLICKEY_SIZE)


/**
 * Defines the start of BCT in sysram.
 */
#define NVBOOT_BCT_START		(NVBOOT_BL_SYSRAM_END - NVBOOT_BCT_SIZE)

/**
 * SDMMC supports DMA. Needs boot mode buffer and Ext CSD buffers to be in sysram.
 * This buffer has to be word aligned.
 */
#define NVBOOT_SDMMC_INT_SYSRAM_BUF_SIZE    (512)
#define NVBOOT_SDMMC_BOOT_MODE_SYSRAM_BUF_SIZE    (512)
#define NVBOOT_SDMMC_INT_SYSRAM_BUF_START    (NVBOOT_BL_SYSRAM_END - \
                                                    NVBOOT_BCT_SIZE - \
                                                    NVBOOT_PUBLICKEY_SIZE - \
                                                    NVBOOT_SDMMC_INT_SYSRAM_BUF_SIZE)

#define NVBOOT_SDMMC_BOOT_MODE_SYSRAM_BUF_START    (NVBOOT_BL_SYSRAM_END - \
                                                    NVBOOT_BCT_SIZE - \
                                                    NVBOOT_PUBLICKEY_SIZE - \
                                                    NVBOOT_SDMMC_INT_SYSRAM_BUF_SIZE - \
                                                    NVBOOT_SDMMC_BOOT_MODE_SYSRAM_BUF_SIZE)

/**
 * Defines the End of in sysram.
 */
#define NVBOOT_BL_SYSRAM_END    (NVBOOT_BPMP_R5_SYSRAM + NV_ADDRESS_MAP_SYSRAM_0_IMPL_SIZE)//(NV_ADDRESS_MAP_SYSRAM_0_LIMIT + 1)

/**
 * Defines the size of IRAM in bytes.
 */
#define NVBOOT_BL_SYSRAM_SIZE   (NVBOOT_BL_SYSRAM_END - NVBOOT_BL_SYSRAM_START)
//#define NVBOOT_BL_SYSRAM_SIZE   (NVBOOT_BL_SYSRAM_START - NVBOOT_BCT_SIZE - NVBOOT_PUBLICKEY_SIZE )



/**
 * Defines the IROM address where secret assets are stored. For Parker,
 * refer to the T186 Security GFD or the T186 Fuse Options spreadsheet for information
 * on what keys are stored in this region of IROM.
 */
#define NVBOOT_IROM_SECRET_STORAGE_SIZE 0x1000
#define NVBOOT_IROM_SECRET_STORAGE_START (NV_ADDRESS_MAP_BPMP_BOOTROM_BASE + NV_ADDRESS_MAP_BPMP_BOOTROM_SIZE - NVBOOT_IROM_SECRET_STORAGE_SIZE)

/**
  * Defines the start address of the BIT in BTCM
  */
#define NVBOOT_BIT_START NVBOOT_BL_BTCM_START

/**
  * Defines the start address of the BCT in BTCM
  */
#define NVBOOT_BCT_START_IN_BTCM (NVBOOT_BL_BTCM_START + NVBOOT_BIT_REQUIRED_SIZE)

/**
 * Defines the Target Address for binary sent during Preproduction/FA
 * UART boot. UART boot will allow downloading binary to BTCM safe start
 * address. Only BIT is valid.
 */
#define NVBOOT_UART_DOWNLOAD_TARGET_ADDRESS NVBOOT_BCT_START_IN_BTCM

/**
 * Defines the Target Address for Mts Sc7 exit binary in BTCM
 * address. valid only in Sc7 exit. 
 */
#define NVBOOT_MTS_SC7_TARGET_ADDRESS NVBOOT_BCT_START_IN_BTCM

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

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CONFIG_H */
