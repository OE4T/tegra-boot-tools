/*
 * Copyright (c) 2007-2009, NVIDIA CORPORATION. All rights reserved.
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
     * Specifies the Config supported by driver.
     * Spi Config (0-5).
     */
    int8_t SpiConfig;

    /* PLLC4 parameters */
    uint8_t PllcDivM;

    uint8_t PllcDivN;

    uint8_t PllcDivP;

    uint8_t PllcClkSelKVCO;

    uint8_t PllcKCP;

    uint8_t PllcSetup;
} NvBootSpiFlashParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SPI_FLASH_PARAM_H */

