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
 * Definition of the device manager parameters structure.
 *
 * The device parameter structure is a union of the parameter structures
 * of all the classes of secondary boot devices.
 *
 * The values in the BCT's device parameter structures are used to
 * reinitialize the driver prior to reading BLs.  This mechanism gives the
 * system designer an opportunity to store values tuned for maximum performance
 * with the specific devices in the system.  In contrast, the values initially
 * used by the Boot ROM to read BCTs are geared toward universality across
 * a wide set of potential parts, not tuned to the characteristics of a
 * specific device.
 *
 * The array of device parameter structures in the BCT permits the storage
 * of sets of optimized parameters for different devices, which supports
 * different board stuffing options, multiple sourcing of parts, etc.
 */

#ifndef INCLUDED_NVBOOT_DEVPARAMS_H
#define INCLUDED_NVBOOT_DEVPARAMS_H

#include "nvcommon.h"
#include "nvboot_error.h"

/*
 * Include the declaration of parameter structures for all supported devices.
 */
#include "nvboot_sdmmc_param.h"
#include "nvboot_spi_flash_param.h"
#include "nvboot_sata_param.h"
#include "nvboot_ufs_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the union of the parameters required by each device.
 */
typedef union
{
    /// Dummy member to fix the size of NvBootDevParams
    uint8_t Size[64];

    /// Specifies optimized parameters for eMMC and eSD
    NvBootSdmmcParams		SdmmcParams;

    /// Specifies optimized parameters for SPI NOR
    NvBootSpiFlashParams	SpiFlashParams;

    /// Specifies optimized parameters for SATA
    NvBootSataParams		SataParams;

	/// Specifies optimized parameters for UFS
	NvBootUfsParams			UfsParams;
} NvBootDevParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_DEVPARAMS_H */
