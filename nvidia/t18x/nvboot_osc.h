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

#ifndef INCLUDED_NVBOOT_OSC_H
#define INCLUDED_NVBOOT_OSC_H

#include "arclk_rst.h"

/**
 * Defines the oscillator frequencies supported by the hardware.
 */

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Set of oscillator frequencies supported by the hardware.
 */
/*
 * Set of oscillator frequencies supproted in the internal API
 * + invalid (measured but not in any valid band)
 * + unknown (not measured at all)
 * Locally define NvBootClocksOscFreq here to have the correct collection of
 * oscillator frequencies.
 */
typedef enum
{
    /// Specifies an oscillator frequency of 13MHz.
    NvBootClocksOscFreq_13 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC13,

    /// Specifies an oscillator frequency of 19.2MHz.
    NvBootClocksOscFreq_19_2 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC19P2,

    /// Specifies an oscillator frequency of 12MHz.
    NvBootClocksOscFreq_12 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC12,

    /// Specifies an oscillator frequency of 26MHz.
    NvBootClocksOscFreq_26 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC26,

    /// Specifies an oscillator frequency of 16.8MHz.
    NvBootClocksOscFreq_16_8 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC16P8,

    /// Specifies an oscillator frequency of 38.4MHz.
    NvBootClocksOscFreq_38_4 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC38P4,

    /// Specifies an oscillator frequency of 48MHz.
    NvBootClocksOscFreq_48 = CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC48,

    NvBootClocksOscFreq_Num = 7,            // dummy to get number of frequencies
    NvBootClocksOscFreq_MaxVal = 13,      // dummy to get the max enum value
    NvBootClocksOscFreq_Unknown = 15,   // unused value to use a illegal/undefined frequency
    NvBootClocksOscFreq_Default_If_Unknown = NvBootClocksOscFreq_38_4, // 38.4Mhz is the primary OSC expected
                                                                       // for T210.
    NvBootClocksOscFreq_Force32 = 0x7fffffff
} NvBootClocksOscFreq;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_OSC_H */
