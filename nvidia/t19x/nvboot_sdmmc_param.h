/*
 * Copyright (c) 2008-2011, NVIDIA CORPORATION. All rights reserved.
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
 * Defines the parameters and data structure for eMMC and eSD devices, which
 * attach to the same SDMMC controller.
 */

#ifndef INCLUDED_NVBOOT_SDMMC_PARAM_H
#define INCLUDED_NVBOOT_SDMMC_PARAM_H

#if defined(__cplusplus)
extern "C"
{
#endif

/// Defines various data widths supported.
typedef enum
{
    /**
     * Specifies a 1 bit interface to eMMC.
     * Note that 1-bit data width is only for the driver's internal use.
     * Fuses doesn't provide option to select 1-bit data width.
     * The driver selects 1-bit internally based on need.
     * It is used for reading Extended CSD and when the power class
     * requirements of a card for 4-bit or 8-bit transfers are not
     * supported by the target board.
     */
    NvBootSdmmcDataWidth_1Bit = 0,

    /// Specifies a 4 bit interface to eMMC.
    NvBootSdmmcDataWidth_4Bit = 1,

    /// Specifies a 8 bit interface to eMMC.
    NvBootSdmmcDataWidth_8Bit = 2,

    /// Specifies a 4 bit Ddr interface to eMMC.
    NvBootSdmmcDataWidth_Ddr_4Bit = 5,

    /// Specifies a 8 bit Ddr interface to eMMC.
    NvBootSdmmcDataWidth_Ddr_8Bit = 6,

    NvBootSdmmcDataWidth_Num,
    NvBootSdmmcDataWidth_Force32 = 0x7FFFFFFF
} NvBootSdmmcDataWidth;

/// Defines the parameters that can be changed after BCT is read.
typedef struct NvBootSdmmcParamsRec
{
    /// Specifies the Config supported by driver.
    /// Sdmmc Config (0-4)
    int8_t SdmmcConfig;

    ///  only Reinit option.
    /**
     * Max Power class supported by the target board.
     * The driver determines the best data width and clock frequency
     * supported within the power class range (0 to Max) if the selected
     * data width cannot be used at the chosen clock frequency.
     */
    uint8_t MaxPowerClassSupported;
} NvBootSdmmcParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SDMMC_PARAM_H */

