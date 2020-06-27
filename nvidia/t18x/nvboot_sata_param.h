/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
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
 * Defines the parameters and data structure for SATA devices.
 */

#ifndef INCLUDED_NVBOOT_SATA_PARAM_H
#define INCLUDED_NVBOOT_SATA_PARAM_H

#include "nvcommon.h"

#if defined(__cplusplus)
extern "C"
{
#endif


typedef enum
{
    /// Specifies Legacy mode
    NvBootSataMode_LegacyPio = 1,

    /// Specifies AHCI mode compliant with AHCI 1.3 spec
    NvBootSataMode_AhciDma,

    NvBootSataMode_Num,
    NvBootSataMode_Force32 = 0x7FFFFFF
} NvBootSataMode;


typedef enum
{
    /// Specifies SATA clock source to be PLLP.
    NvBootSataClockSource_PllPOut0 = 0,

    /// Specifies SATA clock source to be CLK M.
    NvBootSataClockSource_ClkM = 6,

    NvBootSataClockSource_Num,
    NvBootSataClockSource_Force32 = 0x7FFFFFF
} NvBootSataClockSource;


/**
 * Defines the parameters SATA devices.
 */
typedef struct NvBootSataParamsRec
{
    /// Specifies the clock source for SATA controller.
    NvBootSataClockSource SataClockSource;

    /// Specifes the clock divider to be used.
    NvU32 SataClockDivider;

    /// Specifies the clock source for SATA controller.
    NvBootSataClockSource SataOobClockSource;

    /// Specifes the clock divider to be used.
    NvU32 SataOobClockDivider;

    /// Specifies the SATA mode to be used.
    NvBootSataMode SataMode;

   /// Start address(in SYSRAM) for command and FIS structures.
    NvU32 SataBuffersBase;

    /// Plle ref clk divisors
    NvU8 PllRefeDivM;

    NvU8 PllRefeDivN;

    NvU8 PllRefeDivP;

    NvU32 PllRefeMisc;

    /// Plle ref clk divisors
    NvU8 PlleDivM;

    NvU8 PlleDivN;

    NvU8 PlleDivPlCml;

    NvU32 PlleMisc;

    NvBool PlleUsePllRefe;

} NvBootSataParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SATA_PARAM_H */
