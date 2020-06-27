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

/*
 * nvboot_error.h - error codes.
 */

#ifndef INCLUDED_NVBOOT_ERROR_H
#define INCLUDED_NVBOOT_ERROR_H

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined (_MSC_VER)
#define __attribute__(A)
#endif

/**
 * NvBootError: Enumerated error codes.
 *
 * NumErrors should be the last error. Add new errors in between Success and Force32.
 */
#define NVBOOT_ERRORS \
    X(Success/*No Synonyms for Success should be used.*/) \
    X(InvalidParameter) \
    X(IllegalParameter) \
    X(HwTimeOut) \
    X(NotInitialized) \
    X(DeviceNotResponding) \
    X(DataCorrupted) \
    X(DataUnderflow) \
    X(DeviceError) \
    X(DeviceReadError) \
    X(DeviceUnsupported) \
    X(DeviceResponseError) \
    X(Unimplemented) \
    X(ValidationFailure) \
    X(EccDiscoveryFailed) \
    X(EccFailureCorrected) \
    X(EccFailureUncorrected) \
    X(Busy) \
    X(Idle) \
    X(MemoryNotAllocated) \
    X(MemoryNotAligned) \
    X(BctNotFound) \
    X(BctSizeError) \
    X(BootLoaderLoadFailure) \
    X(BctBlockInfoMismatch) \
    X(BctBootloaderVersionMismatch) \
    X(IdentificationFailed) \
    X(HashMismatch) \
    X(TxferFailed) \
    X(WriteFailed) \
    X(EpNotConfigured) \
    X(WarmBoot0_Failure) \
    X(AccessDenied) \
    X(InvalidOscFrequency) \
    X(PllNotLocked) \
    X(InvalidDevParams) \
    X(InvalidBootDeviceEncoding) \
    X(CableNotConnected) \
    X(InvalidBlDst) \
    X(SE_ModExp_OOR /*Message or Signature representative out of range*/) \
    X(SE_RsaPssVerify_Inconsistent) \
    X(FuseHashMismatch /*Mismatch of hash of public key modulus*/) \
    X(XusbDeviceNotAttached/*Xusb related new error codes*/) \
    X(XusbPortResetFailed) \
    X(XusbInvalidBmRequest) \
    X(XusbParseConfigDescFail) \
    X(XusbMscInvalidCmd) \
    X(XusbCswStatusCmdFail) \
    X(XusbMscResetRecovery) \
    X(XusbEpStalled) \
    X(XusbEpError) \
    X(XusbEpRetry) \
    X(XusbEpNotReady/* End Xusb*/) \
    X(XusbControlSeqNumError) \
    X(XusbControlDirError) \
    X(XusbOutofSync) \
    X(XusbPortError) \
    X(XusbDisconnected) \
    X(XusbReset) \
    X(UFSResourceMax/*UFS error codes*/) \
    X(UFSBootNotEnabled) \
    X(UFSFatalError) \
    X(UFSTRDTimeout) \
    X(UFSTRDInProgress) \
    X(UFSReadError) \
    X(UFSLUNNotEnabled) \
    X(UFSLUNBusy) \
    X(UFSLUNCheckCondition) \
    X(UFSBootLUNNotEnabled) \
    X(UFSFlagSet) \
    X(UFSAttributeWrite) \
    X(UFSBootLUNNotFound) \
    X(UFSBootDMECmdError) \
    X(UFSUnknownSCSIStatus/*end UFS error codes*/) \
    X(SE_Context_Restore_Failure) \
    X(SecProvisioningBctKeyMismatch/*Indicates a mismatch between _Secure and _Insecure*/) \
    X(SecProvisioningRcmKeyMismatch/*Indicates a mismatch between _Secure and _Insecure*/) \
    X(SecProvisioningDisabled) \
    X(SecProvisioningEnabled) \
    X(SecProvisioningInvalidKeyInput/*Indicates an invalid/out of bounds value in SecProvisioningKeyNum_Insecure*/) \
    X(SecProvisioningInvalidAntiCloningKey) \
    X(SecProvisioningAntiCloningKeyDisabled) \
    X(CryptoMgr_Busy) \
    X(CryptoMgr_InitFailure) \
    X(CryptoMgr_VerifyFailure) \
    X(CryptoMgr_InvalidAuthScheme) \
    X(CryptoMgr_InvalidVerifyOp) \
    X(CryptoMgr_InvalidEllipticCurve) \
    X(CryptoMgr_Ecdsa_R_S_Out_Of_Range) \
    X(CryptoMgr_Ecdsa_Invalid_R_is_O/*R = u1G + u2Q = O = point at infinity*/) \
    X(CryptoMgr_Ecdsa_Invalid_Sig) \
    X(CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode/*Pcp not loaded because not PKC or ECC mode*/) \
    X(CryptoMgr_Pcp_Invalid/*Pcp has not been validated*/) \
    X(CryptoMgr_Sha_SetupError/*Selected wrong hash function*/) \
    X(CryptoMgr_Sha2_Hash_Mismatch/*Sha2 Hash did not match*/) \
    X(CryptoMgr_BchStage1HeaderNotAuth/*BCH Stage1 not authenticated*/) \
    X(CryptoMgr_BchStage1AuthFail/*BCH Stage1 Authentication failure*/) \
    X(CryptoMgr_BchMb1Stage1HashMismatch/*Stage1 Mb1 hash mismatch*/) \
    X(CryptoMgr_BchStage2HeaderNotAuth/*BCH Stage2 not authenticated*/) \
    X(CryptoMgr_BchStage2AuthFail/*BCH Stage2 Authentication failure*/) \
    X(CryptoMgr_BchMb1Stage2HashMismatch/*Stage2 Mb1 hash mismatch*/) \
    X(CryptoMgr_RcmPayloadAuthFail/*Rcm payload hash mismatch*/) \
    X(CryptoMgr_RcmHeaderAuthFail/*Rcm header hash mismatch*/) \
    X(RcmDebugRcm/*Rcm Force DEBUG Rcm. Not a real error but to force dispatcher to quit RCM.*/) \
    X(MssGenKeyFail/*Mss error codes*/) \
    X(MssDistributeEnable/*Mss error codes*/) \
    X(SE_RSA_Signature_Out_Of_Range/*If the Signature (an integer) is outside the range of 0 to n-1 inclusive, this error is returned.*/) \
    X(CryptoMgr_InvalidEcPoint/*Point is not on the specified EC curve.*/) \
    X(CryptoMgr_InvalidNvAuthScheme/*Invalid NV authentication scheme.*/) \
    X(CryptoMgr_DecryptionNotEnabled/*Decryption not enabled by scheme.*/) \
    X(Pka_PointMult_Hw_Error/*PKA_RETURN_CODE != NORMAL, hw issue.*/) \
    X(Pka_PointAdd_Hw_Error) \
    X(Pka_PointVerif_Hw_Error) \
    X(Pka_PointVerif_Invalid_Ec_Point) \
    X(Pka_PointShamir_Hw_Error) \
    X(Pka_ModRed_Hw_Error) \
    X(Pka_ModInv_Hw_Error) \
    X(Pka_ModMult_Hw_Error) \
    X(Pka_ModAdd_Hw_Error) \
    X(Pka_MontRInv_Hw_Error) \
    X(Pka_MontMP_Hw_Error) \
    X(Pka_MontRSqr_Hw_Error) \
    X(Unsupported_SHA_Family/*Could be unsupported by the driver, not the device*/) \
    X(Unsupported_SHA_DigestSize/*Could be unsupported by the driver, not the device*/) \
    X(SeShaDevInit_CryptoContextSetupError) \
    X(SHA_Digest_Calculation_Error/*Error during SHA operation*/) \
    X(Test_AesWriteReadKey_Mismatch) \
    X(Test_AesEncryptTestFailure) \
    X(Test_AesCmacTestFailure/*Add new errors after this error, but before Force32*/) \
    X(BCHSanityCheckError/*Boot component header sanity check failed */) \
    X(InvalidSeKeySlotNum) \
    X(InvalidSeKeySize) \
    X(Unsupported_RSA_Key_Size) \
    X(Sha2_IntegrityCheck_Fail) \
    X(RsaSsaPss_SignatureVerify_Fail) \
    X(RsaSsaPss_InitFail) \
    X(AesDecrypt_InitFail) \
    X(AesDecrypt_OpFail) \
    X(AesEncrypt_OpFail) \
    X(AesEncrypt_InitFail) \
    X(LoadDebugProdKeys_InvalidMode) \
    X(LoadDebugProdKeys_Fail) \
    X(ProdDebugAllowed_ECID_Mismatch) \
    X(ProdDebugAllowed_InvalidMode) \
    X(ProdDebugAllowed_KeysNotLoaded) \
    X(CryptoMgr_Sc7RfHeaderAuth_InitFail) \
    X(Sc7_InitFail) \
    X(ECID_Mismatch) \
    X(Sc7_DetectRtcRailViolation) \
    X(CB_DetectRtcRailViolationEnable) \
    X(Rng_InitFail) \
    X(Rng_RandomNumberGeneration_Fail) \
    X(SeContext_InitFail) \
    X(SeContextHashMismatch) \
    X(Ecdsa_InitFail) \
    X(Ecdsa_Verification_Fail) \
    X(EdDsa_InitFail) \
    X(EdDsa_VerifyFail) \
    X(RatchetingFail) \
    X(Fault_Injection_Detection) \
    X(Force32/*This is no longer needed because of the mode attribute used in the declaration of NvBootError. Kept in to so the code which uses Force32 can still build.*/) \
    X(NumErrors/*Should be last error in case we need to know the number of errors defined.*/)

#define X(a) NvBootError_##a,
typedef enum
{
    NVBOOT_ERRORS
} NvBootError __attribute__((mode(SI))); // mode(SI) Forces 4-byte size (signed int) for NvBootError.
                                         // If not forced, gcc will reduce the enum size to the smallest
                                         // unit that can fit all the enum values (tested via experimentation).
#undef X

/**
 * Notes on the mode attribute:
 *
 * https://gcc.gnu.org/onlinedocs/gcc-3.2/gcc/Variable-Attributes.html
 * mode (mode)
 * This attribute specifies the data type for the declaration--whichever type corresponds to the mode mode.
 * This in effect lets you request an integer or floating point type according to its width.
 *
 * https://gcc.gnu.org/onlinedocs/gccint/Machine-Modes.html
 * and http://www.delorie.com/gnu/docs/gcc/gcc_80.html.
 * SI means "SImode" or "Single Integer". This usually represents four times the size of the smallest addressable unit, byte.
 * gcc uses "BITS_PER_UNIT" to define the byte size.
 */

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_ERROR_H */

