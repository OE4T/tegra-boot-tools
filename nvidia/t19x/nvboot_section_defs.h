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

#ifndef INCLUDED_NVBOOT_SECTION_DEFS_H
#define INCLUDED_NVBOOT_SECTION_DEFS_H

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined (_MSC_VER)
#define FT_NONSECURE
#else
#define FT_NONSECURE __attribute__((section(".text.nonsecure")))
#define VT_NONSECURE __attribute__((section(".rodata.nonsecure")))
#endif

/* Make function stay close to reset vector
 */
#define FT_BOOTVEC __attribute__((section(".text.boot")))

/* Used by UART */
#define FT_UART __attribute__((section(".text.uart")))

/* Used by NvBootInfoTable */
#define VT_MAINBIT __attribute__((section(".MainBIT")))

/* Used by NvBootConfigTable*/
#define VT_MAINBCT __attribute__((section(".MainBCT")))


/**
 *   Use for all non zero initialized buffers in BTCM. 
 *   1. This saves space by not being part of bootrom
 *   2. Saves time during c runtime init.
 */
#define VT_NOZI __attribute__((section(".nozi.data")))

/**
 *   Use for uninitialized data in SysRAM 
 *   No guarantees that this data in this section will be
 *   preserved after BR exit.
 */
#define VT_SYSRAM __attribute__((section(".noinit.sysram")))

/*  Use for uninitialized data in SysRAM that must be
 *  preserved after BR exit.
 */
#define VT_SYSRAM_PRESERVED __attribute__((section(".noinit.sysram_preserved")))

/**
 *  Used for forcing irom keys to 184K in ROM space
 */
#define VT_IROM_KEYS __attribute__((section(".keys")))

/**
 *  Reserving space in BOOTROM code region for precomputed values.
 */
#define VT_MONT_PRECOMP __attribute__((section(".mont_precomp")))

/**
 *   Use this tag for getting buffers in unimplemented / reserved
 *   SYSRAM address space. Useful for mapping virtual / slave
 *   addresses with AST to master / physical addresses.
 */
#define VT_VIRTUAL_ADDRESS __attribute__((section(".noinit.sysram_c")))

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SECTION_DEFS_H

