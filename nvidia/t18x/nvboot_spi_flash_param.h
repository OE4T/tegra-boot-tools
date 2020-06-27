/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
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
 * Defines the parameters and data structure for SPI FLASH devices.
 */

#ifndef INCLUDED_NVBOOT_SPI_FLASH_PARAM_H
#define INCLUDED_NVBOOT_SPI_FLASH_PARAM_H

#if defined(__cplusplus)
extern "C"
{
#endif
//Clock sources supported by spi driver in bootrom are pllp and clkm only
typedef enum 
{
    /// Specifies SPI clock source to be PLLP_Out0.
    NvBootSpiClockSource_PllPOut0 = 0,

    /// Specifies SPI clock source to be PLLC4_Out2.
    NvBootSpiClockSource_PllC4_Muxed = 4	,

    /// Specifies SPI clock source to be ClockM.
    NvBootSpiClockSource_ClockM = 6,

    NvBootSpiClockSource_Num,
    NvBootSpiClockSource_Force32 = 0x7FFFFFF
} NvBootSpiClockSource;

typedef enum
{
    NvBootSpiDataWidth_x1 = 0,

    NvBootSpiDataWidth_x2,

    NvBootSpiDataWidth_x4,

    NvBootSpiDataWidth_Num,
    NvBootSpiDataWidth_Force32 = 0x7FFFFFF

} NvBootSpiDataWidth;

typedef enum
{
    NvBootSpiXferMode_Pio = 0,

    NvBootSpiXferMode_Dma,

    NvBootSpiXferMode_Num,
    NvBootSpiXferMode_Force32 = 0x7FFFFFF

} NvBootSpiXferMode;

typedef enum {
	NvBootSpiFlashCSActivePolarity_ActiveLow = 0,
	NvBootSpiFlashCSActivePolarity_ActiveHigh = 1,
	NvBootSpiFlashCSActivePolarity_Num,
	NvBootSpiFlashCSActivePolarity_Force32 = 0x7fffffff
} NvBootSpiFlashCSActivePolarity;
/**
 * Defines the parameters for SPI FLASH devices.
 */
typedef struct NvBootSpiFlashParamsRec
{
    /**
     * Chip select polarity - will be selected through BOOT_DEVICE_INFO fuse only
     * Can't be set again through BCT because it doesn't make any sense to set it htrough BCT
     */
     NvBootSpiFlashCSActivePolarity ChipSelectPolarity;

    /**
     * Spi Clock selected will be 20MHz or 50MHz through the BOOT_DEVICE_INFO fuse.
     * Thereafter, spi clock and divider specified in Spi flash params in BCT will be used.
     * 
     */
     NvBool SpiClock;

    /**
     * Specifies the clock source to use.
     */
    
    NvBootSpiClockSource ClockSource;

    /**
     * Specifes the clock divider to use.
     * The value is a 7-bit value based on an input clock of 432Mhz.
     * Divider = (432+ DesiredFrequency-1)/DesiredFrequency;
     * Typical values:
     *     NORMAL_READ at 20MHz: 22
     *     FAST_READ   at 33MHz: 14
     *     FAST_READ   at 40MHz: 11
     *     FAST_READ   at 50MHz:  9
     */
    NvU8 ClockDivider;
    
    /**
     * Specifies the type of command for read operations.
     * NV_FALSE specifies a NORMAL_READ Command
     * NV_TRUE  specifies a FAST_READ   Command
     */
    NvBool ReadCommandTypeFast;
    //0 = 2k page size, 1 = 16K page size
    NvU8   PageSize2kor32k;

    /*
     * Data Width - 0:x1 , 1:x4 (in line with the BOOT_DEVICE_INFO fuse
     * definition.
     * Note: It's highly unlike to change from the fuse definition but
     * Data width as part of the BCT allows us to have a fallback method
     * and not lose performance.
     */
    NvBootSpiDataWidth DataWidth;

    /*
     * XferMode - 0: PIO, 1: DMA
     */
    NvBootSpiXferMode XferMode;

    /*
     * Erase sector size - This will be same as physical sector size of
     * spi flash. Reads can span across pages for some spi flash devices
     * but need to check if some spi flash devices really limit reads to
     * a certain page size
     */
    NvU32 EraseSectorSizeLog2;

    /*
     * Erase time may vary from with flash devices and also based on the
     * the type of erase command. This driver uses only sector erase.
     * This time is to be specified in milliseconds. Most spi flash part specifications
     * provide erase time for one sector of the order of a hundreds of milliseconds.
     */
    NvU32 EraseTime;

    /*
     * Write page size for Page Program in most cases, may not be the same
     * as read/erase sector size.
     */
    NvU32 WritePageSizeLog2;

    /* PLLC4 parameters */
    NvU8 PllcDivM;
	
    NvU8 PllcDivN;

    NvU8 PllcDivP;

    NvU8 PllcKVCO;

    NvU8 PllcKCP;

    NvU8 PllcSetup;

    NvU8 TxFifoDepth;

    NvU8 RxFifoDepth;	

    NvU8 TxFifoTriggerLevel;

    NvU8 RxFifoTriggerLevel;	

    NvU32 DataXferTimeout;

    NvU8 QuadReadDummyCycles;

    NvU8 QuadReadModeCycles;
} NvBootSpiFlashParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SPI_FLASH_PARAM_H */

