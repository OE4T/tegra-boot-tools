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

/*
 * nvboot_error.h - error codes.
 */

#ifndef INCLUDED_NVBOOT_ERROR_H
#define INCLUDED_NVBOOT_ERROR_H

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootError: Enumerated error codes
 */
typedef enum
{
    NvBootError_None = 0,
    NvBootError_Success = 0,
    NvBootError_SE_Signature_Valid = 0, //TODO: remove this in favor of NvBootError_Success?
    NvBootError_ValidAntiCloningFuse = 0,
    NvBootError_CryptoMgr_Idle = 0,

    NvBootError_InvalidParameter,
    NvBootError_IllegalParameter,
    NvBootError_HwTimeOut,
    NvBootError_NotInitialized,
    NvBootError_DeviceNotResponding,
    NvBootError_DataCorrupted,
    NvBootError_DataUnderflow,
    NvBootError_DeviceError,
    NvBootError_DeviceReadError,
    NvBootError_DeviceUnsupported,
    NvBootError_DeviceResponseError,
    NvBootError_Unimplemented,
    NvBootError_ValidationFailure,
    NvBootError_EccDiscoveryFailed,
    NvBootError_EccFailureCorrected,
    NvBootError_EccFailureUncorrected,
    NvBootError_Busy,
    NvBootError_Idle,
    NvBootError_MemoryNotAllocated,
    NvBootError_MemoryNotAligned,
    NvBootError_BctNotFound,
    NvBootError_BctSizeError,
    NvBootError_BootLoaderLoadFailure,
    NvBootError_BctBlockInfoMismatch,
    NvBootError_IdentificationFailed,
    NvBootError_HashMismatch,
    NvBootError_TxferFailed,
    NvBootError_WriteFailed,
    NvBootError_EpNotConfigured,
    NvBootError_WarmBoot0_Failure,
    NvBootError_AccessDenied,
    NvBootError_InvalidOscFrequency,
    NvBootError_PllNotLocked,
    NvBootError_InvalidDevParams,
    NvBootError_InvalidBootDeviceEncoding,
    NvBootError_CableNotConnected,
    NvBootError_InvalidBlDst,
    NvBootError_SE_ModExp_OOR,   // Message or Signature representative 
                                 // out of range 
    NvBootError_SE_RsaPssVerify_Inconsistent, //TODO: add more granulatiry on the errors
    NvBootError_FuseHashMismatch, // Mismatch of hash of public key modulus read from
                                  // secondary storage and fuses
    
    //Xusb related new error codes
    NvBootError_XusbDeviceNotAttached,
    NvBootError_XusbPortResetFailed,
    NvBootError_XusbInvalidBmRequest,
    NvBootError_XusbParseConfigDescFail,
    NvBootError_XusbMscInvalidCmd,
    NvBootError_XusbCswStatusCmdFail,
    NvBootError_XusbMscResetRecovery,
    NvBootError_XusbEpStalled,
    NvBootError_XusbEpError,
    NvBootError_XusbEpRetry,
    NvBootError_XusbEpNotReady,
    //end Xusb
    // Xusb device error codes
    NvBootError_XusbControlSeqNumError,
    NvBootError_XusbControlDirError,
    NvBootError_XusbOutofSync,
    NvBootError_XusbPortError,
    NvBootError_XusbDisconnected,
    NvBootError_XusbReset,
    // UFS error codes
    NvBootError_UFSResourceMax,
    NvBootError_UFSBootNotEnabled,
    NvBootError_UFSFatalError,
    NvBootError_UFSTRDTimeout,
    NvBootError_UFSTRDInProgress,
    NvBootError_UFSReadError,
    NvBootError_UFSLUNNotEnabled,
    NvBootError_UFSLUNBusy,
    NvBootError_UFSLUNCheckCondition,
    NvBootError_UFSBootLUNNotEnabled,
    NvBootError_UFSFlagSet,
    NvBootError_UFSBootLUNNotFound,
    NvBootError_UFSBootDMECmdError,
    NvBootError_UFSUnknownSCSIStatus,
    // end UFS error codes
    NvBootError_SE_Context_Restore_Failure,
    NvBootError_SecProvisioningBctKeyMismatch, // Indicates a mismatch between SecProvisioningKeyNum_Secure
    									   // SecProvisioningKeyNum_Insecure
    NvBootError_SecProvisioningRcmKeyMismatch, // Indicates a mismatch between SecProvisioningKeyNum_Secure
    									   // SecProvisioningKeyNum_Insecure
    NvBootError_SecProvisioningDisabled,
    NvBootError_SecProvisioningEnabled,
    NvBootError_SecProvisioningInvalidKeyInput, // Indicates an invalid/out of bounds value
    										// in SecProvisioningKeyNum_Insecure
    NvBootError_SecProvisioningInvalidAntiCloningKey,
    NvBootError_SecProvisioningAntiCloningKeyDisabled,


    NvBootError_CryptoMgr_Busy,
    NvBootError_CryptoMgr_VerifyFailure,
    NvBootError_CryptoMgr_InvalidAuthScheme,
    NvBootError_CryptoMgr_InvalidVerifyOp,
    NvBootError_CryptoMgr_InvalidEllipticCurve,
    NvBootError_CryptoMgr_Ecdsa_R_S_Out_Of_Range,
    NvBootError_CryptoMgr_Ecdsa_Invalid_R_is_O, // R = u1G + u2Q = O = point at infinity
    NvBootError_CryptoMgr_Ecdsa_Invalid_Sig,
    NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode, // Pcp not loaded because not PKC or ECC mode.
    NvBootError_CryptoMgr_Pcp_Invalid, // Pcp has not been validated.

    // Rcm Force DEBUG Rcm. Not a real error but to force dispatcher to quit RCM.
    NvBootError_RcmDebugRcm,

    // Mss error codes
    NvBootError_MssDistributeEnable,

    // If the Signature (an integer) is outside the range of 0 to n-1 inclusive, this error
    // is returned.
    NvBootError_SE_RSA_Signature_Out_Of_Range,

    NvBootError_CryptoMgr_InvalidEcPoint, // Point is not on the specified EC curve.
    NvBootError_CryptoMgr_InvalidNvAuthScheme, // Invalid NV authentication scheme.
    NvBootError_Pka_PointMult_Hw_Error, // PKA_RETURN_CODE != NORMAL, hw issue.
    NvBootError_Pka_PointAdd_Hw_Error,
    NvBootError_Pka_PointVerif_Hw_Error,
    NvBootError_Pka_PointVerif_Invalid_Ec_Point,
    NvBootError_Pka_PointShamir_Hw_Error,
    NvBootError_Pka_ModRed_Hw_Error,
    NvBootError_Pka_ModInv_Hw_Error,
    NvBootError_Pka_ModMult_Hw_Error,
    NvBootError_Pka_ModAdd_Hw_Error,
    NvBootError_Pka_MontRInv_Hw_Error,
    NvBootError_Pka_MontMP_Hw_Error,
    NvBootError_Pka_MontRSqr_Hw_Error,

    NvBootError_Force32 = 0x7fffffff
} NvBootError;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_ERROR_H */

