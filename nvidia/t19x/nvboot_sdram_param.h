/*
 * Copyright (c) 2007-2010, NVIDIA CORPORATION. All rights reserved.
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
 * Defines the SDRAM parameter structure.
 *
 * Note that PLLM is used by EMC.
 */

#ifndef INCLUDED_NVBOOT_SDRAM_PARAM_H
#define INCLUDED_NVBOOT_SDRAM_PARAM_H

// Don't integrate these to SW folder
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum
{
    /// Specifies the memory type to be undefined
    NvBootMemoryType_None = 0,

    #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
    /// Specifies the memory type to be DDR SDRAM
    NvBootMemoryType_Ddr,

    /// Specifies the memory type to be LPDDR SDRAM
    NvBootMemoryType_LpDdr,

    /// Specifies the memory type to be DDR2 SDRAM
    NvBootMemoryType_Ddr2,
    #endif

    /// Specifies the memory type to be LPDDR2 SDRAM
    NvBootMemoryType_LpDdr2,

    /// Specifies the memory type to be DDR3 SDRAM
    NvBootMemoryType_Ddr3,

    /// Specifies the memory type to be LPDDR4 SDRAM
    NvBootMemoryType_LpDdr4,

    NvBootMemoryType_Num,
    NvBootMemoryType_Force32 = 0x7FFFFFF
} NvBootMemoryType;


/**
 * Defines the SDRAM parameter structure
 */
#include "nvboot_sdram_param_generated.h"

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SDRAM_PARAM_H */

