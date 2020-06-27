/*
 * Copyright (c) 2019 NVIDIA Corporation.  All rights reserved.
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

#ifndef NVBOOT_SDRAM_PARAM_GENERATED_H
#define NVBOOT_SDRAM_PARAM_GENERATED_H
typedef struct NvBootSdramParamsRec

{

    /// Specifies the type of memory device
    NvBootMemoryType MemoryType;
     /// Specifies MemIoVolatage
    NvU32 MemIoVoltage;
    /// Specifies the M value for PllM, PLLM_BASE
    NvU32 PllMInputDivider;
    /// Specifies the N value for PllM, PLLM_BASE
    NvU32 PllMFeedbackDivider;
    /// Specifies the time to wait for PLLM to lock (in microseconds)
    NvU32 PllMStableTime;
    /// Specifies misc. control bits, PLLM_MISC1
    NvU32 PllMSetupControl;
    /// Specifies the P value for PLLM, PLLM_BASE
    NvU32 PllMPostDivider;
    /// Specifies value for Charge Pump Gain Control, PLLM_MISC2
    NvU32 PllMKCP;
    /// Specirfic VCO gain, PLLM_MISC2
    NvU32 PllMKVCO;
    /// Specifies extra init sequence specifically for FPGA
    NvU32 InitExtraForFpga;
    /// Spare BCT param
    NvU32 EmcBctSpare0;
    /// Spare BCT param
    NvU32 EmcBctSpare1;
    /// Spare BCT param
    NvU32 EmcBctSpare2;
    /// Spare BCT param
    NvU32 EmcBctSpare3;
    /// Spare BCT param
    NvU32 EmcBctSpare4;
    /// Spare BCT param
    NvU32 EmcBctSpare5;
    /// Spare BCT param
    NvU32 EmcBctSpare6;
    /// Spare BCT param
    NvU32 EmcBctSpare7;
    /// Spare BCT param
    NvU32 EmcBctSpare8;
    /// Spare BCT param
    NvU32 EmcBctSpare9;
    /// Spare BCT param
    NvU32 EmcBctSpare10;
    /// Spare BCT param
    NvU32 EmcBctSpare11;
    /// Spare BCT param
    NvU32 EmcBctSpare12;
    /// Spare BCT param
    NvU32 EmcBctSpare13;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure0;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure1;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure2;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure3;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure4;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure5;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure6;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure7;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure8;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure9;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure10;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure11;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure12;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure13;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure14;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure15;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure16;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure17;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure18;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure19;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure20;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure21;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure22;
    /// Spare BCT param
    NvU32 EmcBctSpareSecure23;
    /// Defines EMC_2X_CLK_SRC, EMC_2X_CLK_DIVISOR, EMC_INVERT_DCD
    NvU32 EmcClockSource;
    /// Defines EMC_2X_CLK_SRC, EMC_2X_CLK_DIVISOR, EMC_INVERT_DCD
    NvU32 EmcClockSourceDll;
    /// Defines possible override for PLLLM_MISC2
    NvU32 ClkRstControllerPllmMisc2Override;
    /// enables override for PLLLM_MISC2
    NvU32 ClkRstControllerPllmMisc2OverrideEnable;
    /// Disables MSS clocks = register CLK_RST_CONTROLLER_CLK_OUT_ENB_EMC_CLR
    NvU32 ClkRstClkEnbClrEmc;
    /// Disables MSS clocks = register CLK_RST_CONTROLLER_CLK_OUT_ENB_EMCSB_CLR
    NvU32 ClkRstClkEnbClrEmcSb;
    /// Disables MSS clocks = register CLK_RST_CONTROLLER_CLK_OUT_ENB_EMCSB_CLR
    NvU32 ClkRstClkEnbClrEmcSc;
    /// Disables MSS clocks = register CLK_RST_CONTROLLER_CLK_OUT_ENB_EMCSB_CLR
    NvU32 ClkRstClkEnbClrEmcSd;
    /// Disables MSS clocks = register CLK_RST_CONTROLLER_CLK_OUT_ENB_EMCHUB_CLR
    NvU32 ClkRstClkEnbClrEmchub;
    /// Hub clock settings register CLK_RST_CONTROLLER_EMC_MISC
    NvU32 ClkRstEmcMisc;

    /// Auto-calibration of EMC pads
    ///
    /// Specifies the value for EMC_AUTO_CAL_INTERVAL
    NvU32 EmcAutoCalInterval;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG
    /// Note: Trigger bits are set by the SDRAM code.
    NvU32 EmcAutoCalConfig;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG2
    NvU32 EmcAutoCalConfig2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG3
    NvU32 EmcAutoCalConfig3;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG4
    NvU32 EmcAutoCalConfig4;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG5
    NvU32 EmcAutoCalConfig5;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG6
    NvU32 EmcAutoCalConfig6;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG7
    NvU32 EmcAutoCalConfig7;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG8
    NvU32 EmcAutoCalConfig8;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG9
    NvU32 EmcAutoCalConfig9;
    /// Specifies the value for EMC_AUTO_CAL_VREF_SEL_0
    NvU32 EmcAutoCalVrefSel0;
    /// Specifies the value for EMC_AUTO_CAL_VREF_SEL_1
    NvU32 EmcAutoCalVrefSel1;
    /// Specifies the value for EMC_AUTO_CAL_CHANNEL
    NvU32 EmcAutoCalChannel;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_0_CH0
    NvU32 EmcPmacroAutocalCfg0_0;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2_CH0
    NvU32 EmcPmacroAutocalCfg2_0;
    /// Specifies the value for EMC_PMACRO_RX_TERM
    NvU32 EmcPmacroRxTerm;
    /// Specifies the value for EMC_PMACRO_DQ_TX_DRV
    NvU32 EmcPmacroDqTxDrv;
    /// Specifies the value for EMC_PMACRO_CA_TX_DRV
    NvU32 EmcPmacroCaTxDrv;
    /// Specifies the value for EMC_PMACRO_CMD_TX_DRV
    NvU32 EmcPmacroCmdTxDrv;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_COMMON
    NvU32 EmcPmacroAutocalCfgCommon;
    /// Specifies the value for EMC_PMACRO_ZCTRL
    NvU32 EmcPmacroZctrl;
    /// Specifies the time for the calibration to stabilize (in microseconds)
    NvU32 EmcAutoCalWait;
    /// Specifies the value for EMC_XM2COMPPADCTRL
    NvU32 EmcXm2CompPadCtrl;
    /// Specifies the value for EMC_XM2COMPPADCTRL2
    NvU32 EmcXm2CompPadCtrl2;
    /// Specifies the value for EMC_XM2COMPPADCTRL3
    NvU32 EmcXm2CompPadCtrl3;

    /// DRAM size information
    /// Specifies the value for EMC_ADR_CFG
    NvU32 EmcAdrCfg;

    /// Specifies the time to wait after asserting pin CKE (in microseconds)
    NvU32 EmcPinProgramWait;
    /// Specifies the extra delay before/after pin RESET/CKE command
    NvU32 EmcPinExtraWait;
    /// Specifies the value for GPIO_EN in EMC_PIN
    NvU32 EmcPinGpioEn;
    /// Specifies the value for GPIO in  EMC_PIN during ramdump lp4 case
    NvU32 EmcPinGpioRamdump;
    /// Specifies the value for GPIO in  EMC_PIN
    NvU32 EmcPinGpio;
    /// Specifies the extra delay after the first writing of EMC_TIMING_CONTROL
    NvU32 EmcTimingControlWait;

    /// Timing parameters required for the SDRAM
    ///
    /// Specifies the value for EMC_RC
    NvU32 EmcRc;
    /// Specifies the value for EMC_RFC
    NvU32 EmcRfc;
    /// Specifies the value for EMC_RFCPB
    NvU32 EmcRfcPb;
    /// Specifies the value for EMC_PBR2PBR
    NvU32 EmcPbr2Pbr;
    /// Specifies the value for EMC_REFCTRL2
    NvU32 EmcRefctrl2;
    /// Specifies the value for EMC_RFC_SLR
    NvU32 EmcRfcSlr;
    /// Specifies the value for EMC_RAS
    NvU32 EmcRas;
    /// Specifies the value for EMC_RP
    NvU32 EmcRp;
    /// Specifies the value for EMC_R2R
    NvU32 EmcR2r;
    /// Specifies the value for EMC_W2W
    NvU32 EmcW2w;
    /// Specifies the value for EMC_TR2NONTR
    NvU32 EmcTr2nontr;
    /// Specifies the value for EMC_NONRW2TRRW
    NvU32 EmcNonrw2trrw;
    /// Specifies the value for EMC_R2W
    NvU32 EmcR2w;
    /// Specifies the value for EMC_W2R
    NvU32 EmcW2r;
    /// Specifies the value for EMC_R2P
    NvU32 EmcR2p;
    /// Specifies the value for EMC_W2P
    NvU32 EmcW2p;
    /// Specifies the value for EMC_TPPD
    NvU32 EmcTppd;
    /// Specifies the value for EMC_TRTM
    NvU32 EmcTrtm;
    /// Specifies the value for EMC_TWTM
    NvU32 EmcTwtm;
    /// Specifies the value for EMC_TRATM
    NvU32 EmcTratm;
    /// Specifies the value for EMC_TWATM
    NvU32 EmcTwatm;
    /// Specifies the value for EMC_TR2REF
    NvU32 EmcTr2ref;
    /// Specifies the value for EMC_CCDMW
    NvU32 EmcCcdmw;
    /// Specifies the value for EMC_RD_RCD
    NvU32 EmcRdRcd;
    /// Specifies the value for EMC_WR_RCD
    NvU32 EmcWrRcd;
    /// Specifies the value for EMC_RRD
    NvU32 EmcRrd;
    /// Specifies the value for EMC_REXT
    NvU32 EmcRext;
    /// Specifies the value for EMC_WEXT
    NvU32 EmcWext;
    /// Specifies the value for EMC_WDV
    NvU32 EmcWdv;
    /// Specifies the value for EMC_WDV_CHK
    NvU32 EmcWdvChk;
    /// Specifies the value for EMC_WSV
    NvU32 EmcWsv;
    /// Specifies the value for EMC_WSV
    NvU32 EmcWev;
    /// Specifies the value for EMC_WDV_MASK
    NvU32 EmcWdvMask;
    /// Specifies the value for EMC_WS_DURATION
    NvU32 EmcWsDuration;
    /// Specifies the value for EMC_WS_DURATION
    NvU32 EmcWeDuration;
    /// Specifies the value for EMC_QUSE
    NvU32 EmcQUse;
    /// Specifies the value for EMC_QUSE_WIDTH
    NvU32 EmcQuseWidth;
    /// Specifies the value for EMC_IBDLY
    NvU32 EmcIbdly;
    /// Specifies the value for EMC_OBDLY
    NvU32 EmcObdly;
    /// Specifies the value for EMC_EINPUT
    NvU32 EmcEInput;
    /// Specifies the value for EMC_EINPUT_DURATION
    NvU32 EmcEInputDuration;
    /// Specifies the value for EMC_PUTERM_EXTRA
    NvU32 EmcPutermExtra;
    /// Specifies the value for EMC_PUTERM_WIDTH
    NvU32 EmcPutermWidth;
    /// Specifies the value for EMC_QRST
    NvU32 EmcQRst;
    /// Specifies the value for EMC_QSAFE
    NvU32 EmcQSafe;
    /// Specifies the value for EMC_RDV
    NvU32 EmcRdv;
    /// Specifies the value for EMC_RDV_MASK
    NvU32 EmcRdvMask;
    /// Specifies the value for EMC_RDV_EARLY
    NvU32 EmcRdvEarly;
    /// Specifies the value for EMC_RDV_EARLY_MASK
    NvU32 EmcRdvEarlyMask;
    /// Specifies the value for EMC_QPOP
    NvU32 EmcQpop;
    /// Specifies the value for EMC_REFRESH
    NvU32 EmcRefresh;
    /// Specifies the value for EMC_BURST_REFRESH_NUM
    NvU32 EmcBurstRefreshNum;
    /// Specifies the value for EMC_PRE_REFRESH_REQ_CNT
    NvU32 EmcPreRefreshReqCnt;
    /// Specifies the value for EMC_PDEX2WR
    NvU32 EmcPdEx2Wr;
    /// Specifies the value for EMC_PDEX2RD
    NvU32 EmcPdEx2Rd;
    /// Specifies the value for EMC_PCHG2PDEN
    NvU32 EmcPChg2Pden;
    /// Specifies the value for EMC_ACT2PDEN
    NvU32 EmcAct2Pden;
    /// Specifies the value for EMC_AR2PDEN
    NvU32 EmcAr2Pden;
    /// Specifies the value for EMC_RW2PDEN
    NvU32 EmcRw2Pden;
    /// Specifies the value for EMC_CKE2PDEN
    NvU32 EmcCke2Pden;
    /// Specifies the value for EMC_PDEX2CKE
    NvU32 EmcPdex2Cke;
    /// Specifies the value for EMC_PDEX2MRR
    NvU32 EmcPdex2Mrr;
    /// Specifies the value for EMC_TXSR
    NvU32 EmcTxsr;
    /// Specifies the value for EMC_TXSRDLL
    NvU32 EmcTxsrDll;
    /// Specifies the value for EMC_TCKE
    NvU32 EmcTcke;
    /// Specifies the value for EMC_TCKESR
    NvU32 EmcTckesr;
    /// Specifies the value for EMC_TPD
    NvU32 EmcTpd;
    /// Specifies the value for EMC_TFAW
    NvU32 EmcTfaw;
    /// Specifies the value for EMC_TRPAB
    NvU32 EmcTrpab;
    /// Specifies the value for EMC_TCLKSTABLE
    NvU32 EmcTClkStable;
    /// Specifies the value for EMC_TCLKSTOP
    NvU32 EmcTClkStop;
    /// Specifies the value for EMC_TREFBW
    NvU32 EmcTRefBw;

    /// FBIO configuration values
    ///
    /// Specifies the value for EMC_FBIO_CFG5
    NvU32 EmcFbioCfg5;
    /// Specifies the value for EMC_FBIO_CFG7
    NvU32 EmcFbioCfg7;
    /// Specify the value for EMC_FBIO_CFG9
    NvU32 EmcFbioCfg8;
    /// Specify the value for EMC_FBIO_CFG9
    NvU32 EmcFbioCfg9;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_0;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_2;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_0;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_2;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_0;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_2;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_0;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_2;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_0;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_2;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_0;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_2;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_0;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_2;
    /// Specifies the value for EMC_FBIO_SPARE
    NvU32 EmcFbioSpare;

    /// Specifies the value for EMC_CFG_RSV
    NvU32 EmcCfgRsv;

    /// MRS command values
    ///
    /// Specifies the value for EMC_MRS
    NvU32 EmcMrs;
    /// Specifies the MP0 command to initialize mode registers
    NvU32 EmcEmrs;
    /// Specifies the MR2 command to initialize mode registers
    NvU32 EmcEmrs2;
    /// Specifies the MR3 command to initialize mode registers
    NvU32 EmcEmrs3;
    /// Specifies the programming to LPDDR2 Mode Register 1 at cold boot
    NvU32 EmcMrw1;
    /// Specifies the programming to LPDDR2 Mode Register 2 at cold boot
    NvU32 EmcMrw2;
    /// Specifies the programming to LPDDR2/4 Mode Register 3/13 at cold boot
    NvU32 EmcMrw3;
    /// Specifies the programming to LPDDR2 Mode Register 11 at cold boot
    NvU32 EmcMrw4;
    /// Specifies the programming to LPDDR4 Mode Register 3 at cold boot
    NvU32 EmcMrw6;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot
    NvU32 EmcMrw8;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH0
    NvU32 EmcMrw9_0;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_2;
    /// Specifies the programming to LPDDR4 Mode Register 12 at cold boot
    NvU32 EmcMrw10;
    /// Specifies the programming to LPDDR4 Mode Register 14 at cold boot
    NvU32 EmcMrw12;
    /// Specifies the programming to LPDDR4 Mode Register 14 at cold boot
    NvU32 EmcMrw13;
    /// Specifies the programming to LPDDR4 Mode Register 22 at cold boot
    NvU32 EmcMrw14;
    /// Specifies the programming to extra LPDDR2 Mode Register at cold boot
    NvU32 EmcMrwExtra;
    /// Specifies the programming to extra LPDDR2 Mode Register at warm boot
    NvU32 EmcWarmBootMrwExtra;
    /// Specify the enable of extra Mode Register programming at warm boot
    NvU32 EmcWarmBootExtraModeRegWriteEnable;
    /// Specify the enable of extra Mode Register programming at cold boot
    NvU32 EmcExtraModeRegWriteEnable;

    /// Specifies the EMC_MRW reset command value
    NvU32 EmcMrwResetCommand;
    /// Specifies the EMC Reset wait time (in microseconds)
    NvU32 EmcMrwResetNInitWait;
    /// Specifies the value for EMC_MRS_WAIT_CNT
    NvU32 EmcMrsWaitCnt;
    /// Specifies the value for EMC_MRS_WAIT_CNT2
    NvU32 EmcMrsWaitCnt2;

    /// EMC miscellaneous configurations
    ///
    /// Specifies the value for EMC_CFG
    NvU32 EmcCfg;
    /// Specifies value for EMC_RESET_PAD_CTRL
    NvU32 EmcResetPadCtrl;
    /// Specifies the value for EMC_CFG_2
    NvU32 EmcCfg2;
    /// Specifies the Clock Enable Override for Pipe/Barrelshifters
    NvU32 EmcCfgPipeClk;
    /// Specifies the value for EMC_FDPD_CTRL_CMD_NO_RAMP
    NvU32 EmcFdpdCtrlCmdNoRamp;
    /// specify the value for EMC_CFG_UPDATE
    NvU32 EmcCfgUpdate;
    /// Specifies the value for EMC_DBG
    NvU32 EmcDbg;
    /// Specifies the value for EMC_DBG at initialization
    NvU32 EmcDbgWriteMux;
    /// Specifies the value for EMC_CMDQ
    NvU32 EmcCmdQ;
    /// Specifies the value for EMC_MC2EMCQ
    NvU32 EmcMc2EmcQ;
    /// Specifies the value for EMC_DYN_SELF_REF_CONTROL
    NvU32 EmcDynSelfRefControl;
    /// Specifies the value for EMC_ASR_CONTROL
    NvU32 EmcAsrControl;

    /// Specifies the value for EMC_CFG_DIG_DLL
    NvU32 EmcCfgDigDll;

    /// Specifies the value for EMC_CFG_DIG_DLL_1
    NvU32 EmcCfgDigDll_1;
    /// Specifies the value for EMC_CFG_DIG_DLL_PERIOD
    NvU32 EmcCfgDigDllPeriod;
    /// Specifies the vlaue of *DEV_SELECTN of various EMC registers
    NvU32 EmcDevSelect;

    /// Specifies the value for EMC_SEL_DPD_CTRL
    NvU32 EmcSelDpdCtrl;
    /// Specifies the value for fdpd ctrl delays on dq
    NvU32 EmcFdpdCtrlDq;
    /// Specifies the value for fdpd ctrl delays on cmd
    NvU32 EmcFdpdCtrlCmd;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0
    NvU32 EmcPmacroIbVrefDq_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1
    NvU32 EmcPmacroIbVrefDq_1;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0
    NvU32 EmcPmacroIbVrefDqs_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1
    NvU32 EmcPmacroIbVrefDqs_1;
    /// Specifies the value for EMC_PMACRO_IB_RXRT
    NvU32 EmcPmacroIbRxrt;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_5_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_5_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_5_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_5_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_2;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH0
    NvU32 EmcPmacroDdllLongCmd_0_0;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_2;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH0
    NvU32 EmcPmacroDdllLongCmd_1_0;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_2;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH0
    NvU32 EmcPmacroDdllLongCmd_2_0;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_2;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH0
    NvU32 EmcPmacroDdllLongCmd_3_0;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_2;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH0
    NvU32 EmcPmacroDdllLongCmd_4_0;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_2;
    /// Specifies the value for EMC_PMACRO_DDLL_SHORT_CMD_0
    NvU32 EmcPmacroDdllShortCmd_0;
    /// Specifies the value for EMC_PMACRO_DDLL_SHORT_CMD_1
    NvU32 EmcPmacroDdllShortCmd_1;
    /// Specifies the value for EMC_PMACRO_DDLL_SHORT_CMD_2
    NvU32 EmcPmacroDdllShortCmd_2;
    /// Specifies the value for EMC_PMACRO_DDLL_PERIODIC_OFFSET
    NvU32 EmcPmacroDdllPeriodicOffset;

    /// Specifies the delay after asserting CKE pin during a WarmBoot0
    /// sequence (in microseconds)
    NvU32 WarmBootWait;

    /// Specifies the value for EMC_ODT_WRITE
    NvU32 EmcOdtWrite;

    /// Periodic ZQ calibration
    ///
    /// Specifies the value for EMC_ZCAL_INTERVAL
    /// Value 0 disables ZQ calibration
    NvU32 EmcZcalInterval;
    /// Specifies the value for EMC_ZCAL_WAIT_CNT
    NvU32 EmcZcalWaitCnt;
    /// Specifies the value for EMC_ZCAL_MRW_CMD
    NvU32 EmcZcalMrwCmd;

    /// DRAM initialization sequence flow control
    ///
    /// Specifies the MRS command value for resetting DLL
    NvU32 EmcMrsResetDll;
    /// Specifies the command for ZQ initialization of device 0
    NvU32 EmcZcalInitDev0;
    /// Specifies the command for ZQ initialization of device 1
    NvU32 EmcZcalInitDev1;
    /// Specifies the wait time after programming a ZQ initialization command
    /// (in microseconds)
    NvU32 EmcZcalInitWait;
    /// Specifies the enable for ZQ calibration at cold boot [bit 0] and warm boot [bit 1]
    NvU32 EmcZcalWarmColdBootEnables;
    /// Specifies the MRW command to LPDDR2 for ZQ calibration on warmboot
    /// Is issued to both devices separately
    NvU32 EmcMrwLpddr2ZcalWarmBoot;
    /// Specifies the ZQ command to DDR3 for ZQ calibration on warmboot
    /// Is issued to both devices separately
    NvU32 EmcZqCalDdr3WarmBoot;
    /// Specifies the ZQ command to LPDDR4 for ZQ calibration on warmboot
    /// Is issued to both devices separately
    NvU32 EmcZqCalLpDdr4WarmBoot;
    /// Specifies the wait time for ZQ calibration on warmboot
    /// (in microseconds)
    NvU32 EmcZcalWarmBootWait;
    /// Specifies the enable for DRAM Mode Register programming at warm boot
    NvU32 EmcMrsWarmBootEnable;
    /// Specifies the wait time after sending an MRS DLL reset command
    /// in microseconds)
    NvU32 EmcMrsResetDllWait;
    /// Specifies the extra MRS command to initialize mode registers
    NvU32 EmcMrsExtra;
    /// Specifies the extra MRS command at warm boot
    NvU32 EmcWarmBootMrsExtra;
    /// Specifies the EMRS command to enable the DDR2 DLL
    NvU32 EmcEmrsDdr2DllEnable;
    /// Specifies the MRS command to reset the DDR2 DLL
    NvU32 EmcMrsDdr2DllReset;
    /// Specifies the EMRS command to set OCD calibration
    NvU32 EmcEmrsDdr2OcdCalib;
    /// Specifies the wait between initializing DDR and setting OCD
    /// calibration (in microseconds)
    NvU32 EmcDdr2Wait;
    /// Specifies the value for EMC_CLKEN_OVERRIDE
    NvU32 EmcClkenOverride;
    /// Specifies LOG2 of the extra refresh numbers after booting
    /// Program 0 to disable
    NvU32 EmcExtraRefreshNum;
    /// Specifies the master override for all EMC clocks
    NvU32 EmcClkenOverrideAllWarmBoot;
    /// Specifies the master override for MC interface clkens
    NvU32 McClkenA1OverrideAllWarmBoot;
    /// Specifies the master override for all MC clocks
    NvU32 McClkenOverrideAllWarmBoot;
    /// Specifies digital dll period, choosing between 4 to 64 ms
    NvU32 EmcCfgDigDllPeriodWarmBoot;

    /// Pad controls
    ///
    /// Specifies the value for PMC_VDDP_SEL
    NvU32 MssAonVddpSel;
    /// Specifies the wait time after programming PMC_VDDP_SEL
    NvU32 MssAonVddpSelWait;
    /// No longer used in MSS INIT
    NvU32 PmcDdrPwr;
    /// Specifies the value for MSS_AON_CFG_XM0_PAD_DPD_IO_CTRL
    NvU32 MssAonXm0DpdIo;
    /// Specifies the value for MSS_AON_CFG_XM1_PAD_DPD_IO_CTRL
    NvU32 MssAonXm1DpdIo;
    /// Specifies the value for MSS_AON_CFG_XM2_PAD_DPD_IO_CTRL
    NvU32 MssAonXm2DpdIo;
    /// Specifies the value for MSS_AON_CFG_XM3_PAD_DPD_IO_CTRL
    NvU32 MssAonXm3DpdIo;
    /// Specifies the value for MSS_AON_CFG_XM0_PAD_CMD_CTRL
    NvU32 MssAonXm0CmdCtrl;
    /// Specifies the value for MSS_AON_CFG_XM1_PAD_CMD_CTRL
    NvU32 MssAonXm1CmdCtrl;
    /// Specifies the value for MSS_AON_CFG_XM2_PAD_CMD_CTRL
    NvU32 MssAonXm2CmdCtrl;
    /// Specifies the value for MSS_AON_CFG_XM3_PAD_CMD_CTRL
    NvU32 MssAonXm3CmdCtrl;
    /// Specifies the wait time after programming PMC_IO_DPD3_REQ
    NvU32 PmcIoDpd3ReqWait;
    /// Specifies the wait time after programming PMC_IO_DPD4_REQ
    NvU32 PmcIoDpd4ReqWait;
    /// Not used in MSS INIT
    NvU32 PmcBlinkTimer;
    /// Specifies the value for MSS_AON_CFG_XM0_PAD_MISC_CTRL
    NvU32 MssAonNoIoPower;
    /// Specifies the wait time after programing PMC_DDR_CNTRL
    NvU32 MssAonHoldLowWait;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ1_CTRL_REG
    NvU32 RamdumpSeq1CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ1_DATA_REG
    NvU32 RamdumpSeq1DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ2_CTRL_REG
    NvU32 RamdumpSeq2CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ2_DATA_REG
    NvU32 RamdumpSeq2DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ3_CTRL_REG
    NvU32 RamdumpSeq3CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ3_DATA_REG
    NvU32 RamdumpSeq3DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ4_CTRL_REG
    NvU32 RamdumpSeq4CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ4_DATA_REG
    NvU32 RamdumpSeq4DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ5_CTRL_REG
    NvU32 RamdumpSeq5CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ5_DATA_REG
    NvU32 RamdumpSeq5DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ6_CTRL_REG
    NvU32 RamdumpSeq6CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ6_DATA_REG
    NvU32 RamdumpSeq6DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ7_CTRL_REG
    NvU32 RamdumpSeq7CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ7_DATA_REG
    NvU32 RamdumpSeq7DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ8_CTRL_REG
    NvU32 RamdumpSeq8CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ8_DATA_REG
    NvU32 RamdumpSeq8DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ9_CTRL_REG
    NvU32 RamdumpSeq9CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ9_DATA_REG
    NvU32 RamdumpSeq9DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ10_CTRL_REG
    NvU32 RamdumpSeq10CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ10_DATA_REG
    NvU32 RamdumpSeq10DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ11_CTRL_REG
    NvU32 RamdumpSeq11CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ11_DATA_REG
    NvU32 RamdumpSeq11DataReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ12_CTRL_REG
    NvU32 RamdumpSeq12CtrlReg;
    /// Specifies the value for MSS_AON_CFG_RAMDUMP_SEQ12_DATA_REG
    NvU32 RamdumpSeq12DataReg;
    /// Specifies the value for EMC_ACPD_CONTROL
    NvU32 EmcAcpdControl;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH0
    NvU32 EmcSwizzleRank0Byte0_0;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_2;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH0
    NvU32 EmcSwizzleRank0Byte1_0;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_2;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH0
    NvU32 EmcSwizzleRank0Byte2_0;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_2;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH0
    NvU32 EmcSwizzleRank0Byte3_0;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_2;

    /// Specifies the value for EMC_TXDSRVTTGEN
    NvU32 EmcTxdsrvttgen;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH0
    NvU32 EmcDataBrlshft0_0;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_2;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH0
    NvU32 EmcDataBrlshft1_0;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_2;
    /// Specifies the value for EMC_DQS_BRLSHFT_0_CH0
    NvU32 EmcDqsBrlshft0;
    /// Specifies the value for EMC_DQS_BRLSHFT_1_CH0
    NvU32 EmcDqsBrlshft1;
    /// Specifies the value for EMC_CMD_BRLSHFT_0_CH0
    NvU32 EmcCmdBrlshft0;
    /// Specifies the value for EMC_CMD_BRLSHFT_1_CH0
    NvU32 EmcCmdBrlshft1;
    /// Specifies the value for EMC_CMD_BRLSHFT_2_CH0
    NvU32 EmcCmdBrlshft2;
    /// Specifies the value for EMC_CMD_BRLSHFT_3_CH0
    NvU32 EmcCmdBrlshft3;
    /// Specifies the value for EMC_QUSE_BRLSHFT_0
    NvU32 EmcQuseBrlshft0;
    /// Specifies the value for EMC_QUSE_BRLSHFT_2
    NvU32 EmcQuseBrlshft2;
    /// Specifies the value for EMC_PMACRO_DLL_CFG_0
    NvU32 EmcPmacroDllCfg0;
    /// Specifies the value for EMC_PMACRO_DLL_CFG_1
    NvU32 EmcPmacroDllCfg1;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1_0;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2_0;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3_0;
    /// Specifies the value for EMC_PMACRO_PAD_CFG_CTRL
    NvU32 EmcPmacroPadCfgCtrl;
    /// Specifies the value for EMC_PMACRO_VTTGEN_CTRL_0
    NvU32 EmcPmacroVttgenCtrl0;
    /// Specifies the value for EMC_PMACRO_VTTGEN_CTRL_1
    NvU32 EmcPmacroVttgenCtrl1;
    /// Specifies the value for EMC_PMACRO_VTTGEN_CTRL_2
    NvU32 EmcPmacroVttgenCtrl2;
    /// Specifies the value for EMC_PMACRO_DSR_VTTGEN_CTRL_0
    NvU32 EmcPmacroDsrVttgenCtrl0;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL
    NvU32 EmcPmacroBrickCtrlRfu1;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL_FDPD
    NvU32 EmcPmacroCmdBrickCtrlFdpd;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL
    NvU32 EmcPmacroBrickCtrlRfu2;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL_FDPD
    NvU32 EmcPmacroDataBrickCtrlFdpd;
    /// Specifies the value for EMC_PMACRO_BG_BIAS_CTRL_0
    NvU32 EmcPmacroBgBiasCtrl0;
    /// Specifies the value for EMC_PMACRO_DATA_PAD_RX_CTRL
    NvU32 EmcPmacroDataPadRxCtrl;
    /// Specifies the value for EMC_PMACRO_CMD_PAD_RX_CTRL
    NvU32 EmcPmacroCmdPadRxCtrl;
    /// Specifies the value for EMC_PMACRO_DATA_RX_TERM_MODE
    NvU32 EmcPmacroDataRxTermMode;
    /// Specifies the value for EMC_PMACRO_CMD_RX_TERM_MODE
    NvU32 EmcPmacroCmdRxTermMode;
    /// Specifies the value for EMC_PMACRO_DATA_PAD_TX_CTRL
    NvU32 EmcPmacroDataPadTxCtrl;
    /// Specifies the value for EMC_PMACRO_CMD_PAD_TX_CTRL
    NvU32 EmcPmacroCmdPadTxCtrl;
    /// Specifies the value for EMC_CFG_3
    NvU32 EmcCfg3;
    /// Specifies the value for EMC_CONFIG_SAMPLE_DELAY
    NvU32 EmcConfigSampleDelay;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH0
    NvU32 EmcPmacroBrickMapping0_0;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_2;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH0
    NvU32 EmcPmacroBrickMapping1_0;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_2;
    /// Specifies the value for EMC_PMACRO_PERBIT_FGCG_CTRL_0
    NvU32 EmcPmacroPerbitFgcgCtrl0;
    /// Specifies the value for EMC_PMACRO_PERBIT_FGCG_CTRL_1
    NvU32 EmcPmacroPerbitFgcgCtrl1;
    /// Specifies the value for EMC_PMACRO_PERBIT_FGCG_CTRL_4
    NvU32 EmcPmacroPerbitFgcgCtrl4;
    /// Specifies the value for EMC_PMACRO_PERBIT_RFU_CTRL_0
    NvU32 EmcPmacroPerbitRfuCtrl0;
    /// Specifies the value for EMC_PMACRO_PERBIT_RFU_CTRL_1
    NvU32 EmcPmacroPerbitRfuCtrl1;
    /// Specifies the value for EMC_PMACRO_PERBIT_RFU_CTRL_4
    NvU32 EmcPmacroPerbitRfuCtrl4;
    /// Specifies the value for EMC_PMACRO_PERBIT_RFU1_CTRL_0
    NvU32 EmcPmacroPerbitRfu1Ctrl0;
    /// Specifies the value for EMC_PMACRO_PERBIT_RFU1_CTRL_1
    NvU32 EmcPmacroPerbitRfu1Ctrl1;
    /// Specifies the value for EMC_PMACRO_PERBIT_RFU1_CTRL_4
    NvU32 EmcPmacroPerbitRfu1Ctrl4;
    /// Specifies the value for EMC_PMACRO_DATA_PI_CTRL
    NvU32 EmcPmacroDataPiCtrl;
    /// Specifies the value for EMC_PMACRO_CMD_PI_CTRL
    NvU32 EmcPmacroCmdPiCtrl;
    /// Specifies the value for EMC_PMACRO_DDLL_BYPASS
    NvU32 EmcPmacroDdllBypass;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_0
    NvU32 EmcPmacroDdllPwrd0;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_2
    NvU32 EmcPmacroDdllPwrd2;
    /// Specifies the value for EMC_PMACRO_CMD_CTRL_0
    NvU32 EmcPmacroCmdCtrl0;
    /// Specifies the value for EMC_PMACRO_CMD_CTRL_1
    NvU32 EmcPmacroCmdCtrl1;
    /// Specifies the value for EMC_PMACRO_CMD_CTRL_2
    NvU32 EmcPmacroCmdCtrl2;
    /// Specifies the value for MC_REGIF_CONFIG
    NvU32 McRegifConfig;
    /// Specifies the value for MC_REGIF_CONFIG_1
    NvU32 McRegifConfig1;
    /// Specifies the value for MC_REGIF_CONFIG_2
    NvU32 McRegifConfig2;
    /// Specifies the value for MC_REGIF_BROADCAST
    NvU32 McRegifBroadcast;
    /// Specifies the value for MC_REGIF_BROADCAST_1
    NvU32 McRegifBroadcast1;
    /// Specifies the value for MC_REGIF_BROADCAST_2
    NvU32 McRegifBroadcast2;

    /// DRAM size information
    ///
    /// Specifies the value for MC_EMEM_ADR_CFG
    NvU32 McEmemAdrCfg;
    /// Specifies the value for MC_EMEM_ADR_CFG_DEV0
    NvU32 McEmemAdrCfgDev0;
    /// Specifies the value for MC_EMEM_ADR_CFG_DEV1
    NvU32 McEmemAdrCfgDev1;
    /// Specifies the value for MC_EMEM_ADR_CFG_CHANNEL_ENABLE
    NvU32 McEmemAdrCfgChannelEnable;
    /// Specifies the value for MC_EMEM_ADR_CFG_CHANNEL_MASK
    NvU32 McEmemAdrCfgChannelMask0;
    /// Specifies the value for MC_EMEM_ADR_CFG_CHANNEL_MASK_1
    NvU32 McEmemAdrCfgChannelMask1;
    /// Specifies the value for MC_EMEM_ADR_CFG_CHANNEL_MASK_2
    NvU32 McEmemAdrCfgChannelMask2;
    /// Specifies the value for MC_EMEM_ADR_CFG_CHANNEL_MASK_3
    NvU32 McEmemAdrCfgChannelMask3;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_0
    NvU32 McEmemAdrCfgBankMask0;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_1
    NvU32 McEmemAdrCfgBankMask1;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_2
    NvU32 McEmemAdrCfgBankMask2;

    /// Specifies the value for MC_EMEM_CFG which holds the external memory
    /// size (in KBytes)
    NvU32 McEmemCfg;
    /// Specifies the value for MC_CIFLL_MISC0
    NvU32 McCifllMisc0;
    /// Specifies the value for MC_CIFLL_WRDAT_MT_FIFO_CREDITS
    NvU32 McCifllWrdatWrlimit;
    /// Specifies the value for MC_CIFLL_REQ_MT_FIFO_CREDITS
    NvU32 McCifllReqWrlimit;

    /// MC arbitration configuration
    ///
    /// Specifies the value for MC_EMEM_ARB_CFG
    NvU32 McEmemArbCfg;
    /// Specifies the value for MC_EMEM_ARB_OUTSTANDING_REQ
    NvU32 McEmemArbOutstandingReq;
    /// Specifies the value for MC_EMEM_ARB_OUTSTANDING_REQ_NISO
    NvU32 McEmemArbOutstandingReqNiso;
    /// Specifies the value for MC_EMEM_ARB_REFPB_HP_CTRL
    NvU32 McEmemArbRefpbHpCtrl;
    /// Specifies the value for MC_EMEM_ARB_REFPB_BANK_CTRL
    NvU32 McEmemArbRefpbBankCtrl;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RCD
    NvU32 McEmemArbTimingRcd;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RP
    NvU32 McEmemArbTimingRp;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RC
    NvU32 McEmemArbTimingRc;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RAS
    NvU32 McEmemArbTimingRas;
    /// Specifies the value for MC_EMEM_ARB_TIMING_FAW
    NvU32 McEmemArbTimingFaw;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RRD
    NvU32 McEmemArbTimingRrd;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RAP2PRE
    NvU32 McEmemArbTimingRap2Pre;
    /// Specifies the value for MC_EMEM_ARB_TIMING_WAP2PRE
    NvU32 McEmemArbTimingWap2Pre;
    /// Specifies the value for MC_EMEM_ARB_TIMING_R2R
    NvU32 McEmemArbTimingR2R;
    /// Specifies the value for MC_EMEM_ARB_TIMING_W2W
    NvU32 McEmemArbTimingW2W;
    /// Specifies the value for MC_EMEM_ARB_TIMING_R2W
    NvU32 McEmemArbTimingR2W;
    /// Specifies the value for MC_EMEM_ARB_TIMING_W2R
    NvU32 McEmemArbTimingW2R;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RFCPB
    NvU32 McEmemArbTimingRFCPB;
    /// Specifies the value for MC_EMEM_ARB_TIMING_PBR2PBR
    NvU32 McEmemArbTimingPBR2PBR;
    /// Specifies the value for EMC_PERIODIC_TR_CTRL_1
    NvU32 EmcPeriodicTrCtrl1;
    /// Specifies the value for MC_EMEM_ARB_TIMING_PDEX
    NvU32 McEmemArbTimingPDEX;
    /// Specifies the value for MC_EMEM_ARB_TIMING_SREX
    NvU32 McEmemArbTimingSREX;
    /// Specifies the value for MC_EMEM_ARB_DA_TURNS
    NvU32 McEmemArbDaTurns;
    /// Specifies the value for MC_EMEM_ARB_DA_COVERS
    NvU32 McEmemArbDaCovers;
    /// Specifies the value for MC_EMEM_ARB_DA_HYSTERESIS
    NvU32 McEmemArbDaHysteresis;
    /// Specifies the value for MC_EMEM_ARB_MISC0
    NvU32 McEmemArbMisc0;
    /// Specifies the value for MC_EMEM_ARB_MISC1
    NvU32 McEmemArbMisc1;
    /// Specifies the value for MC_EMEM_ARB_MISC2
    NvU32 McEmemArbMisc2;
    /// Specifies the value for MC_EMEM_ARB_MISC3
    NvU32 McEmemArbMisc3;
    /// Specifies the value for MC_EMEM_ARB_MISC4
    NvU32 McEmemArbMisc4;
    /// Specifies the value for MC_EMEM_ARB_RING1_THROTTLE
    NvU32 McEmemArbRing1Throttle;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE
    NvU32 McEmemArbNisoThrottle;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE_MASK
    NvU32 McEmemArbNisoThrottleMask;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE_MASK_1
    NvU32 McEmemArbNisoThrottleMask1;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE_MASK_2
    NvU32 McEmemArbNisoThrottleMask2;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE_MASK_3
    NvU32 McEmemArbNisoThrottleMask3;
    /// Specifies the value for MC_EMEM_ARB_OVERRIDE
    NvU32 McEmemArbOverride;
    /// Specifies the value for MC_EMEM_ARB_OVERRIDE_1
    NvU32 McEmemArbOverride1;
    /// Specifies the value for MC_EMEM_ARB_RSV
    NvU32 McEmemArbRsv;
    /// Specifies the value for MC_DA_CONFIG0
    NvU32 McDaCfg0;
    /// specifies the DRAM CAS to CAS delay timing for masked writes
    NvU32 McEmemArbTimingCcdmw;

    /// Specifies the value for MC_A1_CLKEN_OVERRIDE
    NvU32 McClkenA1Override;

    /// Specifies the value for MC_CLKEN_OVERRIDE
    NvU32 McClkenOverride;

    /// Specifies the value for MC_HUB_CLKEN_OVERRIDE
    NvU32 McHubClkenOverride;

    /// Specifies the value for MC_STAT_CONTROL
    NvU32 McStatControl;

    /// Specifies the value for MC_ECC_CFG
    NvU32 McEccCfg;

    /// Specifies the value for MC_ECC_CONTROL
    NvU32 McEccControl;

    /// Specifies the value for MC_CFG_WCAM_GOB_REMAP
    NvU32 McCfgWcamGobRemap;

    /// Specifies the value for MC_ECC_RAW_MODE_CONTROL
    NvU32 McEccRawModeControl;

    /// Specifies the value for MC_CFG_WCAM
    NvU32 McCfgWcam;

    /// Specifies the value for MC_RING0_MT_FIFO_CREDITS
    NvU32 McRing0MtFifoCredits;
    /// Specifies the value for MC_VIDEO_PROTECT_BOM
    NvU32 McVideoProtectBom;
    /// Specifies the value for MC_VIDEO_PROTECT_BOM_ADR_HI
    NvU32 McVideoProtectBomAdrHi;
    /// Specifies the value for MC_VIDEO_PROTECT_SIZE_MB
    NvU32 McVideoProtectSizeMb;
    /// Specifies the value for MC_VIDEO_PROTECT_VPR_OVERRIDE
    NvU32 McVideoProtectVprOverride;
    /// Specifies the value for MC_VIDEO_PROTECT_VPR_OVERRIDE1
    NvU32 McVideoProtectVprOverride1;
    /// Specifies the value for MC_VIDEO_PROTECT_VPR_OVERRIDE2
    NvU32 McVideoProtectVprOverride2;
    /// Specifies the value for MC_VIDEO_PROTECT_GPU_OVERRIDE_0
    NvU32 McVideoProtectGpuOverride0;
    /// Specifies the value for MC_VIDEO_PROTECT_GPU_OVERRIDE_1
    NvU32 McVideoProtectGpuOverride1;
    /// Specifies the value for MC_SEC_CARVEOUT_BOM
    NvU32 McSecCarveoutBom;
    /// Specifies the value for MC_SEC_CARVEOUT_ADR_HI
    NvU32 McSecCarveoutAdrHi;
    /// Specifies the value for MC_SEC_CARVEOUT_SIZE_MB
    NvU32 McSecCarveoutSizeMb;
    /// Specifies the value for MC_VIDEO_PROTECT_REG_CTRL.VIDEO_PROTECT_WRITE_ACCESS
    NvU32 McVideoProtectWriteAccess;
    /// Specifies the value for MC_SEC_CARVEOUT_REG_CTRL.SEC_CARVEOUT_WRITE_ACCESS
    NvU32 McSecCarveoutProtectWriteAccess;
    /// Specifies which carveouts to program during MSS init
    NvU32 McGSCInitMask;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_BOM
    NvU32 McGeneralizedCarveout1Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_BOM_HI
    NvU32 McGeneralizedCarveout1BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_SIZE_128KB
    NvU32 McGeneralizedCarveout1Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout1Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout1Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout1Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout1Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout1Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout1Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout1Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout1Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout1ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout1ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout1ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout1ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout1ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout1ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout1ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout1ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CFG0
    NvU32 McGeneralizedCarveout1Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_BOM
    NvU32 McGeneralizedCarveout2Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_BOM_HI
    NvU32 McGeneralizedCarveout2BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_SIZE_128KB
    NvU32 McGeneralizedCarveout2Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout2Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout2Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout2Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout2Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout2Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout2Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout2Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout2Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout2ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout2ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout2ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout2ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout2ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout2ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout2ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout2ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CFG0
    NvU32 McGeneralizedCarveout2Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_BOM
    NvU32 McGeneralizedCarveout3Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_BOM_HI
    NvU32 McGeneralizedCarveout3BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_SIZE_128KB
    NvU32 McGeneralizedCarveout3Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout3Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout3Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout3Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout3Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout3Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout3Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout3Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout3Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout3ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout3ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout3ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout3ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout3ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout3ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout3ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout3ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CFG0
    NvU32 McGeneralizedCarveout3Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_BOM
    NvU32 McGeneralizedCarveout4Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_BOM_HI
    NvU32 McGeneralizedCarveout4BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_SIZE_128KB
    NvU32 McGeneralizedCarveout4Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout4Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout4Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout4Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout4Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout4Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout4Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout4Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout4Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout4ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout4ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout4ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout4ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout4ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout4ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout4ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout4ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CFG0
    NvU32 McGeneralizedCarveout4Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_BOM
    NvU32 McGeneralizedCarveout5Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_BOM_HI
    NvU32 McGeneralizedCarveout5BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_SIZE_128KB
    NvU32 McGeneralizedCarveout5Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout5Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout5Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout5Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout5Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout5Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout5Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout5Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout5Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout5ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout5ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout5ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout5ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout5ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout5ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout5ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout5ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CFG0
    NvU32 McGeneralizedCarveout5Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_BOM
    NvU32 McGeneralizedCarveout6Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_BOM_HI
    NvU32 McGeneralizedCarveout6BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_SIZE_128KB
    NvU32 McGeneralizedCarveout6Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout6Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout6Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout6Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout6Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout6Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout6Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout6Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout6Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout6ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout6ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout6ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout6ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout6ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout6ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout6ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout6ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT6_CFG0
    NvU32 McGeneralizedCarveout6Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_BOM
    NvU32 McGeneralizedCarveout7Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_BOM_HI
    NvU32 McGeneralizedCarveout7BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_SIZE_128KB
    NvU32 McGeneralizedCarveout7Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout7Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout7Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout7Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout7Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout7Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout7Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout7Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout7Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout7ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout7ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout7ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout7ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout7ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout7ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout7ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout7ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT7_CFG0
    NvU32 McGeneralizedCarveout7Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_BOM
    NvU32 McGeneralizedCarveout8Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_BOM_HI
    NvU32 McGeneralizedCarveout8BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_SIZE_128KB
    NvU32 McGeneralizedCarveout8Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout8Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout8Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout8Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout8Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout8Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout8Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout8Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout8Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout8ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout8ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout8ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout8ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout8ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout8ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout8ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout8ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT8_CFG0
    NvU32 McGeneralizedCarveout8Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_BOM
    NvU32 McGeneralizedCarveout9Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_BOM_HI
    NvU32 McGeneralizedCarveout9BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_SIZE_128KB
    NvU32 McGeneralizedCarveout9Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout9Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout9Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout9Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout9Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout9Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout9Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout9Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout9Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout9ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout9ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout9ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout9ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout9ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout9ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout9ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout9ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT9_CFG0
    NvU32 McGeneralizedCarveout9Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_BOM
    NvU32 McGeneralizedCarveout10Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_BOM_HI
    NvU32 McGeneralizedCarveout10BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_SIZE_128KB
    NvU32 McGeneralizedCarveout10Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout10Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout10Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout10Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout10Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout10Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout10Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout10Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout10Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout10ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout10ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout10ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout10ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout10ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout10ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout10ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout10ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT10_CFG0
    NvU32 McGeneralizedCarveout10Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_BOM
    NvU32 McGeneralizedCarveout11Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_BOM_HI
    NvU32 McGeneralizedCarveout11BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_SIZE_128KB
    NvU32 McGeneralizedCarveout11Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout11Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout11Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout11Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout11Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout11Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout11Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout11Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout11Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout11ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout11ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout11ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout11ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout11ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout11ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout11ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout11ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT11_CFG0
    NvU32 McGeneralizedCarveout11Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_BOM
    NvU32 McGeneralizedCarveout12Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_BOM_HI
    NvU32 McGeneralizedCarveout12BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_SIZE_128KB
    NvU32 McGeneralizedCarveout12Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout12Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout12Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout12Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout12Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout12Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout12Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout12Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout12Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout12ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout12ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout12ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout12ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout12ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout12ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout12ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout12ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT12_CFG0
    NvU32 McGeneralizedCarveout12Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_BOM
    NvU32 McGeneralizedCarveout13Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_BOM_HI
    NvU32 McGeneralizedCarveout13BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_SIZE_128KB
    NvU32 McGeneralizedCarveout13Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout13Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout13Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout13Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout13Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout13Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout13Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout13Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout13Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout13ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout13ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout13ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout13ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout13ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout13ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout13ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout13ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT13_CFG0
    NvU32 McGeneralizedCarveout13Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_BOM
    NvU32 McGeneralizedCarveout14Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_BOM_HI
    NvU32 McGeneralizedCarveout14BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_SIZE_128KB
    NvU32 McGeneralizedCarveout14Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout14Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout14Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout14Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout14Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout14Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout14Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout14Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout14Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout14ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout14ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout14ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout14ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout14ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout14ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout14ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout14ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT14_CFG0
    NvU32 McGeneralizedCarveout14Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_BOM
    NvU32 McGeneralizedCarveout15Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_BOM_HI
    NvU32 McGeneralizedCarveout15BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_SIZE_128KB
    NvU32 McGeneralizedCarveout15Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout15Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout15Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout15Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout15Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout15Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout15Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout15Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout15Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout15ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout15ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout15ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout15ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout15ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout15ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout15ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout15ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT15_CFG0
    NvU32 McGeneralizedCarveout15Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_BOM
    NvU32 McGeneralizedCarveout16Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_BOM_HI
    NvU32 McGeneralizedCarveout16BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_SIZE_128KB
    NvU32 McGeneralizedCarveout16Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout16Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout16Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout16Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout16Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout16Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout16Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout16Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout16Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout16ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout16ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout16ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout16ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout16ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout16ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout16ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout16ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT16_CFG0
    NvU32 McGeneralizedCarveout16Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_BOM
    NvU32 McGeneralizedCarveout17Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_BOM_HI
    NvU32 McGeneralizedCarveout17BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_SIZE_128KB
    NvU32 McGeneralizedCarveout17Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout17Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout17Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout17Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout17Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout17Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout17Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout17Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout17Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout17ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout17ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout17ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout17ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout17ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout17ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout17ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout17ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT17_CFG0
    NvU32 McGeneralizedCarveout17Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_BOM
    NvU32 McGeneralizedCarveout18Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_BOM_HI
    NvU32 McGeneralizedCarveout18BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_SIZE_128KB
    NvU32 McGeneralizedCarveout18Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout18Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout18Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout18Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout18Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout18Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout18Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout18Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout18Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout18ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout18ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout18ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout18ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout18ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout18ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout18ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout18ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT18_CFG0
    NvU32 McGeneralizedCarveout18Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_BOM
    NvU32 McGeneralizedCarveout19Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_BOM_HI
    NvU32 McGeneralizedCarveout19BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_SIZE_128KB
    NvU32 McGeneralizedCarveout19Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout19Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout19Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout19Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout19Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout19Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout19Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout19Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout19Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout19ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout19ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout19ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout19ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout19ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout19ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout19ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout19ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT19_CFG0
    NvU32 McGeneralizedCarveout19Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_BOM
    NvU32 McGeneralizedCarveout20Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_BOM_HI
    NvU32 McGeneralizedCarveout20BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_SIZE_128KB
    NvU32 McGeneralizedCarveout20Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout20Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout20Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout20Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout20Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout20Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout20Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout20Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout20Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout20ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout20ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout20ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout20ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout20ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout20ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout20ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout20ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT20_CFG0
    NvU32 McGeneralizedCarveout20Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_BOM
    NvU32 McGeneralizedCarveout21Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_BOM_HI
    NvU32 McGeneralizedCarveout21BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_SIZE_128KB
    NvU32 McGeneralizedCarveout21Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout21Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout21Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout21Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout21Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout21Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout21Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout21Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout21Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout21ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout21ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout21ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout21ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout21ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout21ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout21ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout21ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT21_CFG0
    NvU32 McGeneralizedCarveout21Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_BOM
    NvU32 McGeneralizedCarveout22Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_BOM_HI
    NvU32 McGeneralizedCarveout22BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_SIZE_128KB
    NvU32 McGeneralizedCarveout22Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout22Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout22Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout22Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout22Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout22Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout22Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout22Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout22Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout22ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout22ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout22ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout22ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout22ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout22ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout22ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout22ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT22_CFG0
    NvU32 McGeneralizedCarveout22Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_BOM
    NvU32 McGeneralizedCarveout23Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_BOM_HI
    NvU32 McGeneralizedCarveout23BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_SIZE_128KB
    NvU32 McGeneralizedCarveout23Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout23Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout23Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout23Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout23Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout23Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout23Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout23Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout23Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout23ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout23ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout23ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout23ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout23ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout23ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout23ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout23ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT23_CFG0
    NvU32 McGeneralizedCarveout23Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_BOM
    NvU32 McGeneralizedCarveout24Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_BOM_HI
    NvU32 McGeneralizedCarveout24BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_SIZE_128KB
    NvU32 McGeneralizedCarveout24Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout24Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout24Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout24Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout24Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout24Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout24Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout24Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout24Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout24ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout24ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout24ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout24ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout24ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout24ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout24ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout24ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT24_CFG0
    NvU32 McGeneralizedCarveout24Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_BOM
    NvU32 McGeneralizedCarveout25Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_BOM_HI
    NvU32 McGeneralizedCarveout25BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_SIZE_128KB
    NvU32 McGeneralizedCarveout25Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout25Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout25Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout25Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout25Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout25Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout25Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout25Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout25Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout25ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout25ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout25ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout25ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout25ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout25ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout25ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout25ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT25_CFG0
    NvU32 McGeneralizedCarveout25Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_BOM
    NvU32 McGeneralizedCarveout26Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_BOM_HI
    NvU32 McGeneralizedCarveout26BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_SIZE_128KB
    NvU32 McGeneralizedCarveout26Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout26Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout26Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout26Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout26Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout26Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout26Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout26Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout26Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout26ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout26ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout26ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout26ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout26ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout26ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout26ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout26ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT26_CFG0
    NvU32 McGeneralizedCarveout26Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_BOM
    NvU32 McGeneralizedCarveout27Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_BOM_HI
    NvU32 McGeneralizedCarveout27BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_SIZE_128KB
    NvU32 McGeneralizedCarveout27Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout27Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout27Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout27Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout27Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout27Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout27Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout27Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout27Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout27ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout27ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout27ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout27ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout27ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout27ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout27ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout27ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT27_CFG0
    NvU32 McGeneralizedCarveout27Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_BOM
    NvU32 McGeneralizedCarveout28Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_BOM_HI
    NvU32 McGeneralizedCarveout28BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_SIZE_128KB
    NvU32 McGeneralizedCarveout28Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout28Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout28Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout28Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout28Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout28Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout28Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout28Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout28Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout28ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout28ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout28ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout28ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout28ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout28ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout28ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout28ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT28_CFG0
    NvU32 McGeneralizedCarveout28Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_BOM
    NvU32 McGeneralizedCarveout29Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_BOM_HI
    NvU32 McGeneralizedCarveout29BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_SIZE_128KB
    NvU32 McGeneralizedCarveout29Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout29Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout29Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout29Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout29Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout29Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout29Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout29Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout29Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout29ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout29ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout29ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout29ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout29ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout29ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout29ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout29ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CFG0
    NvU32 McGeneralizedCarveout29Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_BOM
    NvU32 McGeneralizedCarveout30Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_BOM_HI
    NvU32 McGeneralizedCarveout30BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_SIZE_128KB
    NvU32 McGeneralizedCarveout30Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout30Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout30Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout30Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout30Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout30Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout30Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout30Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout30Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout30ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout30ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout30ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout30ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout30ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout30ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout30ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout30ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT30_CFG0
    NvU32 McGeneralizedCarveout30Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_BOM
    NvU32 McGeneralizedCarveout31Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_BOM_HI
    NvU32 McGeneralizedCarveout31BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_SIZE_128KB
    NvU32 McGeneralizedCarveout31Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout31Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout31Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout31Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout31Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout31Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS5
    NvU32 McGeneralizedCarveout31Access5;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS6
    NvU32 McGeneralizedCarveout31Access6;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_ACCESS7
    NvU32 McGeneralizedCarveout31Access7;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout31ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout31ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout31ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout31ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout31ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS5
    NvU32 McGeneralizedCarveout31ForceInternalAccess5;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS6
    NvU32 McGeneralizedCarveout31ForceInternalAccess6;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CLIENT_FORCE_INTERNAL_ACCESS7
    NvU32 McGeneralizedCarveout31ForceInternalAccess7;
    /// Specifies the value for MC_SECURITY_CARVEOUT31_CFG0
    NvU32 McGeneralizedCarveout31Cfg0;
    /// Specifies the value for MC_LATENCY_ALLOWANCE_CIFLL_WR_0
    NvU32 McLatencyAllowanceCifllWr0;
    /// Specifies the value for MC_ECC_REGION0_CFG0
    NvU32 McEccRegion0Cfg0;
    /// Specifies the value for MC_ECC_REGION0_BOM
    NvU32 McEccRegion0Bom;
    /// Specifies the value for MC_ECC_REGION0_BOM_HI
    NvU32 McEccRegion0BomHi;
    /// Specifies the value for MC_ECC_REGION0_SIZE
    NvU32 McEccRegion0Size;
    /// Specifies the value for MC_ECC_REGION1_CFG0
    NvU32 McEccRegion1Cfg0;
    /// Specifies the value for MC_ECC_REGION1_BOM
    NvU32 McEccRegion1Bom;
    /// Specifies the value for MC_ECC_REGION1_BOM_HI
    NvU32 McEccRegion1BomHi;
    /// Specifies the value for MC_ECC_REGion1_SIZE
    NvU32 McEccRegion1Size;
    /// Specifies the value for MC_ECC_REGION2_CFG0
    NvU32 McEccRegion2Cfg0;
    /// Specifies the value for MC_ECC_REGION2_BOM
    NvU32 McEccRegion2Bom;
    /// Specifies the value for MC_ECC_REGION2_BOM_HI
    NvU32 McEccRegion2BomHi;
    /// Specifies the value for MC_ECC_REGion2_SIZE
    NvU32 McEccRegion2Size;
    /// Specifies the value for MC_ECC_REGION3_CFG0
    NvU32 McEccRegion3Cfg0;
    /// Specifies the value for MC_ECC_REGION3_BOM
    NvU32 McEccRegion3Bom;
    /// Specifies the value for MC_ECC_REGION3_BOM_HI
    NvU32 McEccRegion3BomHi;
    /// Specifies the value for MC_ECC_REGION3_SIZE
    NvU32 McEccRegion3Size;
    /// Specifies enable and offset for patched boot rom write
    NvU32 BootRomPatchControl;
    /// Specifies data for patched boot rom write
    NvU32 BootRomPatchData;
    /// Specifies the value for MC_MTS_CARVEOUT_BOM
    NvU32 McMtsCarveoutBom;
    /// Specifies the value for MC_MTS_CARVEOUT_ADR_HI
    NvU32 McMtsCarveoutAdrHi;
    /// Specifies the value for MC_MTS_CARVEOUT_SIZE_MB
    NvU32 McMtsCarveoutSizeMb;
    /// Specifies the value for MC_MTS_CARVEOUT_REG_CTRL
    NvU32 McMtsCarveoutRegCtrl;
    /// Specifies the value for 1MB aligned value of Syncpoint BOM
    NvU32 McSyncpointBom;
    /// Specifies the value for 1MB aligned value of Syncpoint TOM
    NvU32 McSyncpointTom;
    /// Specifies the value for MC_SYNCPOINT_REG_CTRL_0
    NvU32 McSyncpointRegCtrl;
    /// Specifies flags for generating keys for encryption regions
    NvU32 MssEncryptGenKeys;
    /// Specifies flags for distributing encryption keys
    NvU32 MssEncryptDistKeys;
    /// Specifies IREQX VC Manager Arbiter Configuration
    NvU32 McMcfIreqxVcarbConfig;
    /// Specifies OREQX VC Manager Arbiter Configuration
    NvU32 McMcfOreqxVcarbConfig;
    /// Specifies OREQX LLARB Bypass Configuration
    NvU32 McMcfOreqxLlarbConfig;
    /// Specifies OREQX STAT Control Configuration
    NvU32 McMcfOreqxStatControl;
    /// Specifies IREQX source weights
    NvU32 McMcfIreqxSrcWeight0;
    /// Specifies IREQX source weights
    NvU32 McMcfIreqxSrcWeight1;
    /// Specifies IREQX Second-level Clock Enable Overrides
    NvU32 McMcfIreqxClkenOverride;
    /// Specifies OREQX Second-level Clock Enable Overrides
    NvU32 McMcfOreqxClkenOverride;
    /// Specifies SLICE Second-level Clock Enable Overrides
    NvU32 McMcfSliceClkenOverride;
    /// Specifies ORSPX Second-level Clock Enable Overrides
    NvU32 McMcfOrspxClkenOverride;
    /// Specifies IRSPX Second-level Clock Enable Overrides
    NvU32 McMcfIrspxClkenOverride;
    /// Specifies IRSPX Arb optimizations
    NvU32 McMcfIrspxRdrspOpt;
    /// Specifies ORSPX Arb optimizations
    NvU32 McMcfOrspxArbConfig;
    /// Specifies SYSRAM Second-level Clock Enable Overrides
    NvU32 McMssSysramClkenOverride;
    /// Specifies SBS aAsync Interface Configuration
    NvU32 McMssSbsAsync;
    /// Specifies SBS Arbiter Configuration
    NvU32 McMssSbsArb;
    /// Specifies SBS Second-level Clock Enable Overrides
    NvU32 McMssSbsClkenOverride;
    /// Specifies SBS VC limits
    NvU32 McMssSbsVcLimit;
    /// Specifies if ISO / NISO_REMOTE requests are allowed to be io-coherent
    NvU32 McMcfSliceCfg;
    /// Specifies the value for MC_MCF_SLICE_FL_NISO_LIMIT
    NvU32 McMcfSliceFlNisoLimit;
    /// Specifies the value for MC_MCF_SLICE_FL_SISO_LIMIT
    NvU32 McMcfSliceFlSisoLimit;
    /// Specifies the value for MC_MCF_SLICE_FL_ISO_LIMIT
    NvU32 McMcfSliceFlIsoLimit;
    /// Specifies the value for MC_MCF_SLICE_FL_TRANSDONE_LIMIT
    NvU32 McMcfSliceFlTransdoneLimit;
    /// Specifies the value for MC_MCF_SLICE_FL_NISO_REMOTE_LIMIT
    NvU32 McMcfSliceFlNisoRemoteLimit;
    /// Specifies the value for MC_MCF_SLICE_FL_ORD1_LIMIT
    NvU32 McMcfSliceFlOrd1Limit;
    /// Specifies the value for MC_MCF_SLICE_FL_ORD2_LIMIT
    NvU32 McMcfSliceFlOrd2Limit;
    /// Specifies the value for MC_MCF_SLICE_FL_ORD3_LIMIT
    NvU32 McMcfSliceFlOrd3Limit;
    /// Specifies the value for MC_SMMU_BYPASS_CONFIG
    NvU32 McSmmuBypassConfig;
    /// Specifies the value for MC_CLIENT_ORDER_ID_0
    NvU32 McClientOrderId0;
    /// Specifies the value for MC_CLIENT_ORDER_ID_2
    NvU32 McClientOrderId2;
    /// Specifies the value for MC_CLIENT_ORDER_ID_3
    NvU32 McClientOrderId3;
    /// Specifies the value for MC_CLIENT_ORDER_ID_4
    NvU32 McClientOrderId4;
    /// Specifies the value for MC_CLIENT_ORDER_ID_5
    NvU32 McClientOrderId5;
    /// Specifies the value for MC_CLIENT_ORDER_ID_6
    NvU32 McClientOrderId6;
    /// Specifies the value for MC_CLIENT_ORDER_ID_7
    NvU32 McClientOrderId7;
    /// Specifies the value for MC_CLIENT_ORDER_ID_8
    NvU32 McClientOrderId8;
    /// Specifies the value for MC_CLIENT_ORDER_ID_9
    NvU32 McClientOrderId9;
    /// Specifies the value for MC_CLIENT_ORDER_ID_10
    NvU32 McClientOrderId10;
    /// Specifies the value for MC_CLIENT_ORDER_ID_12
    NvU32 McClientOrderId12;
    /// Specifies the value for MC_CLIENT_ORDER_ID_13
    NvU32 McClientOrderId13;
    /// Specifies the value for MC_CLIENT_ORDER_ID_14
    NvU32 McClientOrderId14;
    /// Specifies the value for MC_CLIENT_ORDER_ID_15
    NvU32 McClientOrderId15;
    /// Specifies the value for MC_CLIENT_ORDER_ID_16
    NvU32 McClientOrderId16;
    /// Specifies the value for MC_CLIENT_ORDER_ID_17
    NvU32 McClientOrderId17;
    /// Specifies the value for MC_CLIENT_ORDER_ID_18
    NvU32 McClientOrderId18;
    /// Specifies the value for MC_CLIENT_ORDER_ID_19
    NvU32 McClientOrderId19;
    /// Specifies the value for MC_CLIENT_ORDER_ID_20
    NvU32 McClientOrderId20;
    /// Specifies the value for MC_CLIENT_ORDER_ID_21
    NvU32 McClientOrderId21;
    /// Specifies the value for MC_CLIENT_ORDER_ID_22
    NvU32 McClientOrderId22;
    /// Specifies the value for MC_CLIENT_ORDER_ID_23
    NvU32 McClientOrderId23;
    /// Specifies the value for MC_CLIENT_ORDER_ID_24
    NvU32 McClientOrderId24;
    /// Specifies the value for MC_CLIENT_ORDER_ID_25
    NvU32 McClientOrderId25;
    /// Specifies the value for MC_CLIENT_ORDER_ID_26
    NvU32 McClientOrderId26;
    /// Specifies the value for MC_CLIENT_ORDER_ID_27
    NvU32 McClientOrderId27;
    /// Specifies the value for MC_CLIENT_ORDER_ID_28
    NvU32 McClientOrderId28;
    /// Specifies the value for MC_CLIENT_ORDER_ID_29
    NvU32 McClientOrderId29;
    /// Specifies the value for MC_CLIENT_ORDER_ID_30
    NvU32 McClientOrderId30;
    /// Specifies the value for MC_CLIENT_ORDER_ID_31
    NvU32 McClientOrderId31;
    /// Specifies the value for MC_CONFIG_TSA_SINGLE_ARB_ENABLE
    NvU32 McConfigTsaSingleArbEnable;
    /// Specifies the value for MC_HUB_PC_VC_ID_0
    NvU32 McHubPcVcId0;
    /// Specifies the value for MC_HUB_PC_VC_ID_1
    NvU32 McHubPcVcId1;
    /// Specifies the value for MC_HUB_PC_VC_ID_2
    NvU32 McHubPcVcId2;
    /// Specifies the value for MC_HUB_PC_VC_ID_3
    NvU32 McHubPcVcId3;
    /// Specifies the value for MC_HUB_PC_VC_ID_4
    NvU32 McHubPcVcId4;
    /// Specifies the value for MC_HUB_PC_VC_ID_5
    NvU32 McHubPcVcId5;
    /// Specifies the value for MC_HUB_PC_VC_ID_6
    NvU32 McHubPcVcId6;
    /// Specifies the value for MC_HUB_PC_VC_ID_7
    NvU32 McHubPcVcId7;
    /// Specifies the value for MC_HUB_PC_VC_ID_8
    NvU32 McHubPcVcId8;
    /// Specifies the value for MC_HUB_PC_VC_ID_9
    NvU32 McHubPcVcId9;
    /// Specifies the value for MC_HUB_PC_VC_ID_10
    NvU32 McHubPcVcId10;
    /// Specifies the value for MC_HUB_PC_VC_ID_11
    NvU32 McHubPcVcId11;
    /// Specifies the value for MC_HUB_PC_VC_ID_12
    NvU32 McHubPcVcId12;
    /// Specifies the value for MC_HUB_PC_VC_ID_13
    NvU32 McHubPcVcId13;
    /// Specifies the value for MC_HUB_PC_VC_ID_14
    NvU32 McHubPcVcId14;
    /// Specifies if Sid programming should be bypassed at init
    NvU32 McBypassSidInit;
    /// Specifies the value for TRAINING_WRITE_FINE_CTRL
    NvU32 EmcTrainingWriteFineCtrl;
    /// Specifies the value for TRAINING_READ_FINE_CTRL
    NvU32 EmcTrainingReadFineCtrl;
    /// Specifies the value for TRAINING_WRITE_VREF_CTRL
    NvU32 EmcTrainingWriteVrefCtrl;
    /// Specifies the value for TRAINING_READ_VREF_CTRL
    NvU32 EmcTrainingReadVrefCtrl;
    /// Specifies the value for TRAINING_WRITE_CTRL_MISC
    NvU32 EmcTrainingWriteCtrlMisc;
    /// Specifies the value for TRAINING_READ_CTRL_MISC
    NvU32 EmcTrainingReadCtrlMisc;
    /// Specifies the value for TRAINING_MPC
    NvU32 EmcTrainingMpc;
    /// Specifies the value for TRAINING_CTRL
    NvU32 EmcTrainingCtrl;
    /// Specifies the value for TRAINING_CMD
    NvU32 EmcTrainingCmd;
    /// Specifies the value for TRAINING_PATRAM_DQ0
    NvU32 EmcTrainingPatramDq0;
    /// Specifies the value for TRAINING_PATRAM_DQ1
    NvU32 EmcTrainingPatramDq1;
    /// Specifies the value for TRAINING_PATRAM_DQ2
    NvU32 EmcTrainingPatramDq2;
    /// Specifies the value for TRAINING_PATRAM_DQ3
    NvU32 EmcTrainingPatramDq3;
    /// Specifies the value for TRAINING_PATRAM_DQ4
    NvU32 EmcTrainingPatramDq4;
    /// Specifies the value for TRAINING_PATRAM_DQ5
    NvU32 EmcTrainingPatramDq5;
    /// Specifies the value for TRAINING_PATRAM_DQ6
    NvU32 EmcTrainingPatramDq6;
    /// Specifies the value for TRAINING_PATRAM_DQ7
    NvU32 EmcTrainingPatramDq7;
    /// Specifies the value for TRAINING_PATRAM_DQ8
    NvU32 EmcTrainingPatramDq8;
    /// Specifies the value for TRAINING_PATRAM_DQ9
    NvU32 EmcTrainingPatramDq9;
    /// Specifies the value for TRAINING_PATRAM_DQ10
    NvU32 EmcTrainingPatramDq10;
    /// Specifies the value for TRAINING_PATRAM_DQ11
    NvU32 EmcTrainingPatramDq11;
    /// Specifies the value for TRAINING_PATRAM_DQ12
    NvU32 EmcTrainingPatramDq12;
    /// Specifies the value for TRAINING_PATRAM_DQ13
    NvU32 EmcTrainingPatramDq13;
    /// Specifies the value for TRAINING_PATRAM_DQ14
    NvU32 EmcTrainingPatramDq14;
    /// Specifies the value for TRAINING_PATRAM_DQ15
    NvU32 EmcTrainingPatramDq15;
    /// Specifies the value for TRAINING_PATRAM_DQ16
    NvU32 EmcTrainingPatramDq16;
    /// Specifies the value for TRAINING_PATRAM_DQ17
    NvU32 EmcTrainingPatramDq17;
    /// Specifies the value for TRAINING_PATRAM_DQ18
    NvU32 EmcTrainingPatramDq18;
    /// Specifies the value for TRAINING_PATRAM_DQ19
    NvU32 EmcTrainingPatramDq19;
    /// Specifies the value for TRAINING_PATRAM_DQ20
    NvU32 EmcTrainingPatramDq20;
    /// Specifies the value for TRAINING_PATRAM_DQ21
    NvU32 EmcTrainingPatramDq21;
    /// Specifies the value for TRAINING_PATRAM_DQ22
    NvU32 EmcTrainingPatramDq22;
    /// Specifies the value for TRAINING_PATRAM_DQ23
    NvU32 EmcTrainingPatramDq23;
    /// Specifies the value for TRAINING_PATRAM_DQ24
    NvU32 EmcTrainingPatramDq24;
    /// Specifies the value for TRAINING_PATRAM_DQ25
    NvU32 EmcTrainingPatramDq25;
    /// Specifies the value for TRAINING_PATRAM_DQ26
    NvU32 EmcTrainingPatramDq26;
    /// Specifies the value for TRAINING_PATRAM_DQ27
    NvU32 EmcTrainingPatramDq27;
    /// Specifies the value for TRAINING_PATRAM_DQ28
    NvU32 EmcTrainingPatramDq28;
    /// Specifies the value for TRAINING_PATRAM_DQ29
    NvU32 EmcTrainingPatramDq29;
    /// Specifies the value for TRAINING_PATRAM_DQ30
    NvU32 EmcTrainingPatramDq30;
    /// Specifies the value for TRAINING_PATRAM_DQ31
    NvU32 EmcTrainingPatramDq31;
    /// Specifies the value for TRAINING_PATRAM_DQ32
    NvU32 EmcTrainingPatramDq32;
    /// Specifies the value for TRAINING_PATRAM_DQ33
    NvU32 EmcTrainingPatramDq33;
    /// Specifies the value for TRAINING_PATRAM_DQ34
    NvU32 EmcTrainingPatramDq34;
    /// Specifies the value for TRAINING_PATRAM_DQ35
    NvU32 EmcTrainingPatramDq35;
    /// Specifies the value for TRAINING_PATRAM_DQ36
    NvU32 EmcTrainingPatramDq36;
    /// Specifies the value for TRAINING_PATRAM_DQ37
    NvU32 EmcTrainingPatramDq37;
    /// Specifies the value for TRAINING_PATRAM_DQ38
    NvU32 EmcTrainingPatramDq38;
    /// Specifies the value for TRAINING_PATRAM_DQ39
    NvU32 EmcTrainingPatramDq39;
    /// Specifies the value for TRAINING_PATRAM_DQ40
    NvU32 EmcTrainingPatramDq40;
    /// Specifies the value for TRAINING_PATRAM_DQ41
    NvU32 EmcTrainingPatramDq41;
    /// Specifies the value for TRAINING_PATRAM_DQ42
    NvU32 EmcTrainingPatramDq42;
    /// Specifies the value for TRAINING_PATRAM_DQ43
    NvU32 EmcTrainingPatramDq43;
    /// Specifies the value for TRAINING_PATRAM_DQ44
    NvU32 EmcTrainingPatramDq44;
    /// Specifies the value for TRAINING_PATRAM_DQ45
    NvU32 EmcTrainingPatramDq45;
    /// Specifies the value for TRAINING_PATRAM_DQ46
    NvU32 EmcTrainingPatramDq46;
    /// Specifies the value for TRAINING_PATRAM_DQ47
    NvU32 EmcTrainingPatramDq47;
    /// Specifies the value for TRAINING_PATRAM_DQ48
    NvU32 EmcTrainingPatramDq48;
    /// Specifies the value for TRAINING_PATRAM_DQ49
    NvU32 EmcTrainingPatramDq49;
    /// Specifies the value for TRAINING_PATRAM_DQ50
    NvU32 EmcTrainingPatramDq50;
    /// Specifies the value for TRAINING_PATRAM_DQ51
    NvU32 EmcTrainingPatramDq51;
    /// Specifies the value for TRAINING_PATRAM_DQ52
    NvU32 EmcTrainingPatramDq52;
    /// Specifies the value for TRAINING_PATRAM_DQ53
    NvU32 EmcTrainingPatramDq53;
    /// Specifies the value for TRAINING_PATRAM_DQ54
    NvU32 EmcTrainingPatramDq54;
    /// Specifies the value for TRAINING_PATRAM_DQ55
    NvU32 EmcTrainingPatramDq55;
    /// Specifies the value for TRAINING_PATRAM_DQ56
    NvU32 EmcTrainingPatramDq56;
    /// Specifies the value for TRAINING_PATRAM_DQ57
    NvU32 EmcTrainingPatramDq57;
    /// Specifies the value for TRAINING_PATRAM_DQ58
    NvU32 EmcTrainingPatramDq58;
    /// Specifies the value for TRAINING_PATRAM_DQ59
    NvU32 EmcTrainingPatramDq59;
    /// Specifies the value for TRAINING_PATRAM_DQ60
    NvU32 EmcTrainingPatramDq60;
    /// Specifies the value for TRAINING_PATRAM_DQ61
    NvU32 EmcTrainingPatramDq61;
    /// Specifies the value for TRAINING_PATRAM_DQ62
    NvU32 EmcTrainingPatramDq62;
    /// Specifies the value for TRAINING_PATRAM_DQ63
    NvU32 EmcTrainingPatramDq63;
    /// Specifies the value for TRAINING_PATRAM_DQ64
    NvU32 EmcTrainingPatramDq64;
    /// Specifies the value for TRAINING_PATRAM_DQ65
    NvU32 EmcTrainingPatramDq65;
    /// Specifies the value for TRAINING_PATRAM_DQ66
    NvU32 EmcTrainingPatramDq66;
    /// Specifies the value for TRAINING_PATRAM_DQ67
    NvU32 EmcTrainingPatramDq67;
    /// Specifies the value for TRAINING_PATRAM_DQ68
    NvU32 EmcTrainingPatramDq68;
    /// Specifies the value for TRAINING_PATRAM_DQ69
    NvU32 EmcTrainingPatramDq69;
    /// Specifies the value for TRAINING_PATRAM_DQ70
    NvU32 EmcTrainingPatramDq70;
    /// Specifies the value for TRAINING_PATRAM_DQ71
    NvU32 EmcTrainingPatramDq71;
    /// Specifies the value for TRAINING_PATRAM_DQ72
    NvU32 EmcTrainingPatramDq72;
    /// Specifies the value for TRAINING_PATRAM_DQ73
    NvU32 EmcTrainingPatramDq73;
    /// Specifies the value for TRAINING_PATRAM_DQ74
    NvU32 EmcTrainingPatramDq74;
    /// Specifies the value for TRAINING_PATRAM_DQ75
    NvU32 EmcTrainingPatramDq75;
    /// Specifies the value for TRAINING_PATRAM_DQ76
    NvU32 EmcTrainingPatramDq76;
    /// Specifies the value for TRAINING_PATRAM_DQ77
    NvU32 EmcTrainingPatramDq77;
    /// Specifies the value for TRAINING_PATRAM_DQ78
    NvU32 EmcTrainingPatramDq78;
    /// Specifies the value for TRAINING_PATRAM_DQ79
    NvU32 EmcTrainingPatramDq79;
    /// Specifies the value for TRAINING_PATRAM_DQ80
    NvU32 EmcTrainingPatramDq80;
    /// Specifies the value for TRAINING_PATRAM_DQ81
    NvU32 EmcTrainingPatramDq81;
    /// Specifies the value for TRAINING_PATRAM_DQ82
    NvU32 EmcTrainingPatramDq82;
    /// Specifies the value for TRAINING_PATRAM_DQ83
    NvU32 EmcTrainingPatramDq83;
    /// Specifies the value for TRAINING_PATRAM_DQ84
    NvU32 EmcTrainingPatramDq84;
    /// Specifies the value for TRAINING_PATRAM_DQ85
    NvU32 EmcTrainingPatramDq85;
    /// Specifies the value for TRAINING_PATRAM_DQ86
    NvU32 EmcTrainingPatramDq86;
    /// Specifies the value for TRAINING_PATRAM_DQ87
    NvU32 EmcTrainingPatramDq87;
    /// Specifies the value for TRAINING_PATRAM_DQ88
    NvU32 EmcTrainingPatramDq88;
    /// Specifies the value for TRAINING_PATRAM_DQ89
    NvU32 EmcTrainingPatramDq89;
    /// Specifies the value for TRAINING_PATRAM_DQ90
    NvU32 EmcTrainingPatramDq90;
    /// Specifies the value for TRAINING_PATRAM_DQ91
    NvU32 EmcTrainingPatramDq91;
    /// Specifies the value for TRAINING_PATRAM_DQ92
    NvU32 EmcTrainingPatramDq92;
    /// Specifies the value for TRAINING_PATRAM_DQ93
    NvU32 EmcTrainingPatramDq93;
    /// Specifies the value for TRAINING_PATRAM_DQ94
    NvU32 EmcTrainingPatramDq94;
    /// Specifies the value for TRAINING_PATRAM_DQ95
    NvU32 EmcTrainingPatramDq95;
    /// Specifies the value for TRAINING_PATRAM_DQ96
    NvU32 EmcTrainingPatramDq96;
    /// Specifies the value for TRAINING_PATRAM_DQ97
    NvU32 EmcTrainingPatramDq97;
    /// Specifies the value for TRAINING_PATRAM_DQ98
    NvU32 EmcTrainingPatramDq98;
    /// Specifies the value for TRAINING_PATRAM_DQ99
    NvU32 EmcTrainingPatramDq99;
    /// Specifies the value for TRAINING_PATRAM_DQ100
    NvU32 EmcTrainingPatramDq100;
    /// Specifies the value for TRAINING_PATRAM_DQ101
    NvU32 EmcTrainingPatramDq101;
    /// Specifies the value for TRAINING_PATRAM_DQ102
    NvU32 EmcTrainingPatramDq102;
    /// Specifies the value for TRAINING_PATRAM_DQ103
    NvU32 EmcTrainingPatramDq103;
    /// Specifies the value for TRAINING_PATRAM_DQ104
    NvU32 EmcTrainingPatramDq104;
    /// Specifies the value for TRAINING_PATRAM_DQ105
    NvU32 EmcTrainingPatramDq105;
    /// Specifies the value for TRAINING_PATRAM_DQ106
    NvU32 EmcTrainingPatramDq106;
    /// Specifies the value for TRAINING_PATRAM_DQ107
    NvU32 EmcTrainingPatramDq107;
    /// Specifies the value for TRAINING_PATRAM_DQ108
    NvU32 EmcTrainingPatramDq108;
    /// Specifies the value for TRAINING_PATRAM_DQ109
    NvU32 EmcTrainingPatramDq109;
    /// Specifies the value for TRAINING_PATRAM_DQ110
    NvU32 EmcTrainingPatramDq110;
    /// Specifies the value for TRAINING_PATRAM_DQ111
    NvU32 EmcTrainingPatramDq111;
    /// Specifies the value for TRAINING_PATRAM_DQ112
    NvU32 EmcTrainingPatramDq112;
    /// Specifies the value for TRAINING_PATRAM_DQ113
    NvU32 EmcTrainingPatramDq113;
    /// Specifies the value for TRAINING_PATRAM_DQ114
    NvU32 EmcTrainingPatramDq114;
    /// Specifies the value for TRAINING_PATRAM_DQ115
    NvU32 EmcTrainingPatramDq115;
    /// Specifies the value for TRAINING_PATRAM_DQ116
    NvU32 EmcTrainingPatramDq116;
    /// Specifies the value for TRAINING_PATRAM_DQ117
    NvU32 EmcTrainingPatramDq117;
    /// Specifies the value for TRAINING_PATRAM_DQ118
    NvU32 EmcTrainingPatramDq118;
    /// Specifies the value for TRAINING_PATRAM_DQ119
    NvU32 EmcTrainingPatramDq119;
    /// Specifies the value for TRAINING_PATRAM_DQ120
    NvU32 EmcTrainingPatramDq120;
    /// Specifies the value for TRAINING_PATRAM_DQ121
    NvU32 EmcTrainingPatramDq121;
    /// Specifies the value for TRAINING_PATRAM_DQ122
    NvU32 EmcTrainingPatramDq122;
    /// Specifies the value for TRAINING_PATRAM_DQ123
    NvU32 EmcTrainingPatramDq123;
    /// Specifies the value for TRAINING_PATRAM_DQ124
    NvU32 EmcTrainingPatramDq124;
    /// Specifies the value for TRAINING_PATRAM_DQ125
    NvU32 EmcTrainingPatramDq125;
    /// Specifies the value for TRAINING_PATRAM_DQ126
    NvU32 EmcTrainingPatramDq126;
    /// Specifies the value for TRAINING_PATRAM_DQ127
    NvU32 EmcTrainingPatramDq127;
    /// Specifies the value for TRAINING_PATRAM_DQ128
    NvU32 EmcTrainingPatramDq128;
    /// Specifies the value for TRAINING_PATRAM_DQ129
    NvU32 EmcTrainingPatramDq129;
    /// Specifies the value for TRAINING_PATRAM_DQ130
    NvU32 EmcTrainingPatramDq130;
    /// Specifies the value for TRAINING_PATRAM_DQ131
    NvU32 EmcTrainingPatramDq131;
    /// Specifies the value for TRAINING_PATRAM_DQ132
    NvU32 EmcTrainingPatramDq132;
    /// Specifies the value for TRAINING_PATRAM_DQ133
    NvU32 EmcTrainingPatramDq133;
    /// Specifies the value for TRAINING_PATRAM_DQ134
    NvU32 EmcTrainingPatramDq134;
    /// Specifies the value for TRAINING_PATRAM_DQ135
    NvU32 EmcTrainingPatramDq135;
    /// Specifies the value for TRAINING_PATRAM_DQ136
    NvU32 EmcTrainingPatramDq136;
    /// Specifies the value for TRAINING_PATRAM_DQ137
    NvU32 EmcTrainingPatramDq137;
    /// Specifies the value for TRAINING_PATRAM_DQ138
    NvU32 EmcTrainingPatramDq138;
    /// Specifies the value for TRAINING_PATRAM_DQ139
    NvU32 EmcTrainingPatramDq139;
    /// Specifies the value for TRAINING_PATRAM_DQ140
    NvU32 EmcTrainingPatramDq140;
    /// Specifies the value for TRAINING_PATRAM_DQ141
    NvU32 EmcTrainingPatramDq141;
    /// Specifies the value for TRAINING_PATRAM_DQ142
    NvU32 EmcTrainingPatramDq142;
    /// Specifies the value for TRAINING_PATRAM_DQ143
    NvU32 EmcTrainingPatramDq143;
    /// Specifies the value for TRAINING_PATRAM_DQ144
    NvU32 EmcTrainingPatramDq144;
    /// Specifies the value for TRAINING_PATRAM_DQ145
    NvU32 EmcTrainingPatramDq145;
    /// Specifies the value for TRAINING_PATRAM_DQ146
    NvU32 EmcTrainingPatramDq146;
    /// Specifies the value for TRAINING_PATRAM_DQ147
    NvU32 EmcTrainingPatramDq147;
    /// Specifies the value for TRAINING_PATRAM_DQ148
    NvU32 EmcTrainingPatramDq148;
    /// Specifies the value for TRAINING_PATRAM_DQ149
    NvU32 EmcTrainingPatramDq149;
    /// Specifies the value for TRAINING_PATRAM_DQ150
    NvU32 EmcTrainingPatramDq150;
    /// Specifies the value for TRAINING_PATRAM_DQ151
    NvU32 EmcTrainingPatramDq151;
    /// Specifies the value for TRAINING_PATRAM_DQ152
    NvU32 EmcTrainingPatramDq152;
    /// Specifies the value for TRAINING_PATRAM_DQ153
    NvU32 EmcTrainingPatramDq153;
    /// Specifies the value for TRAINING_PATRAM_DQ154
    NvU32 EmcTrainingPatramDq154;
    /// Specifies the value for TRAINING_PATRAM_DQ155
    NvU32 EmcTrainingPatramDq155;
    /// Specifies the value for TRAINING_PATRAM_DQ156
    NvU32 EmcTrainingPatramDq156;
    /// Specifies the value for TRAINING_PATRAM_DQ157
    NvU32 EmcTrainingPatramDq157;
    /// Specifies the value for TRAINING_PATRAM_DQ158
    NvU32 EmcTrainingPatramDq158;
    /// Specifies the value for TRAINING_PATRAM_DQ159
    NvU32 EmcTrainingPatramDq159;
    /// Specifies the value for TRAINING_PATRAM_DQ160
    NvU32 EmcTrainingPatramDq160;
    /// Specifies the value for TRAINING_PATRAM_DQ161
    NvU32 EmcTrainingPatramDq161;
    /// Specifies the value for TRAINING_PATRAM_DQ162
    NvU32 EmcTrainingPatramDq162;
    /// Specifies the value for TRAINING_PATRAM_DQ163
    NvU32 EmcTrainingPatramDq163;
    /// Specifies the value for TRAINING_PATRAM_DQ164
    NvU32 EmcTrainingPatramDq164;
    /// Specifies the value for TRAINING_PATRAM_DQ165
    NvU32 EmcTrainingPatramDq165;
    /// Specifies the value for TRAINING_PATRAM_DQ166
    NvU32 EmcTrainingPatramDq166;
    /// Specifies the value for TRAINING_PATRAM_DQ167
    NvU32 EmcTrainingPatramDq167;
    /// Specifies the value for TRAINING_PATRAM_DQ168
    NvU32 EmcTrainingPatramDq168;
    /// Specifies the value for TRAINING_PATRAM_DQ169
    NvU32 EmcTrainingPatramDq169;
    /// Specifies the value for TRAINING_PATRAM_DQ170
    NvU32 EmcTrainingPatramDq170;
    /// Specifies the value for TRAINING_PATRAM_DQ171
    NvU32 EmcTrainingPatramDq171;
    /// Specifies the value for TRAINING_PATRAM_DQ172
    NvU32 EmcTrainingPatramDq172;
    /// Specifies the value for TRAINING_PATRAM_DQ173
    NvU32 EmcTrainingPatramDq173;
    /// Specifies the value for TRAINING_PATRAM_DQ174
    NvU32 EmcTrainingPatramDq174;
    /// Specifies the value for TRAINING_PATRAM_DQ175
    NvU32 EmcTrainingPatramDq175;
    /// Specifies the value for TRAINING_PATRAM_DQ176
    NvU32 EmcTrainingPatramDq176;
    /// Specifies the value for TRAINING_PATRAM_DQ177
    NvU32 EmcTrainingPatramDq177;
    /// Specifies the value for TRAINING_PATRAM_DQ178
    NvU32 EmcTrainingPatramDq178;
    /// Specifies the value for TRAINING_PATRAM_DQ179
    NvU32 EmcTrainingPatramDq179;
    /// Specifies the value for TRAINING_PATRAM_DQ180
    NvU32 EmcTrainingPatramDq180;
    /// Specifies the value for TRAINING_PATRAM_DQ181
    NvU32 EmcTrainingPatramDq181;
    /// Specifies the value for TRAINING_PATRAM_DQ182
    NvU32 EmcTrainingPatramDq182;
    /// Specifies the value for TRAINING_PATRAM_DQ183
    NvU32 EmcTrainingPatramDq183;
    /// Specifies the value for TRAINING_PATRAM_DQ184
    NvU32 EmcTrainingPatramDq184;
    /// Specifies the value for TRAINING_PATRAM_DQ185
    NvU32 EmcTrainingPatramDq185;
    /// Specifies the value for TRAINING_PATRAM_DQ186
    NvU32 EmcTrainingPatramDq186;
    /// Specifies the value for TRAINING_PATRAM_DQ187
    NvU32 EmcTrainingPatramDq187;
    /// Specifies the value for TRAINING_PATRAM_DQ188
    NvU32 EmcTrainingPatramDq188;
    /// Specifies the value for TRAINING_PATRAM_DQ189
    NvU32 EmcTrainingPatramDq189;
    /// Specifies the value for TRAINING_PATRAM_DQ190
    NvU32 EmcTrainingPatramDq190;
    /// Specifies the value for TRAINING_PATRAM_DQ191
    NvU32 EmcTrainingPatramDq191;
    /// Specifies the value for TRAINING_PATRAM_DQ192
    NvU32 EmcTrainingPatramDq192;
    /// Specifies the value for TRAINING_PATRAM_DQ193
    NvU32 EmcTrainingPatramDq193;
    /// Specifies the value for TRAINING_PATRAM_DQ194
    NvU32 EmcTrainingPatramDq194;
    /// Specifies the value for TRAINING_PATRAM_DQ195
    NvU32 EmcTrainingPatramDq195;
    /// Specifies the value for TRAINING_PATRAM_DQ196
    NvU32 EmcTrainingPatramDq196;
    /// Specifies the value for TRAINING_PATRAM_DQ197
    NvU32 EmcTrainingPatramDq197;
    /// Specifies the value for TRAINING_PATRAM_DQ198
    NvU32 EmcTrainingPatramDq198;
    /// Specifies the value for TRAINING_PATRAM_DQ199
    NvU32 EmcTrainingPatramDq199;
    /// Specifies the value for TRAINING_PATRAM_DQ200
    NvU32 EmcTrainingPatramDq200;
    /// Specifies the value for TRAINING_PATRAM_DQ201
    NvU32 EmcTrainingPatramDq201;
    /// Specifies the value for TRAINING_PATRAM_DQ202
    NvU32 EmcTrainingPatramDq202;
    /// Specifies the value for TRAINING_PATRAM_DQ203
    NvU32 EmcTrainingPatramDq203;
    /// Specifies the value for TRAINING_PATRAM_DQ204
    NvU32 EmcTrainingPatramDq204;
    /// Specifies the value for TRAINING_PATRAM_DQ205
    NvU32 EmcTrainingPatramDq205;
    /// Specifies the value for TRAINING_PATRAM_DQ206
    NvU32 EmcTrainingPatramDq206;
    /// Specifies the value for TRAINING_PATRAM_DQ207
    NvU32 EmcTrainingPatramDq207;
    /// Specifies the value for TRAINING_PATRAM_DQ208
    NvU32 EmcTrainingPatramDq208;
    /// Specifies the value for TRAINING_PATRAM_DQ209
    NvU32 EmcTrainingPatramDq209;
    /// Specifies the value for TRAINING_PATRAM_DQ210
    NvU32 EmcTrainingPatramDq210;
    /// Specifies the value for TRAINING_PATRAM_DQ211
    NvU32 EmcTrainingPatramDq211;
    /// Specifies the value for TRAINING_PATRAM_DQ212
    NvU32 EmcTrainingPatramDq212;
    /// Specifies the value for TRAINING_PATRAM_DQ213
    NvU32 EmcTrainingPatramDq213;
    /// Specifies the value for TRAINING_PATRAM_DQ214
    NvU32 EmcTrainingPatramDq214;
    /// Specifies the value for TRAINING_PATRAM_DQ215
    NvU32 EmcTrainingPatramDq215;
    /// Specifies the value for TRAINING_PATRAM_DQ216
    NvU32 EmcTrainingPatramDq216;
    /// Specifies the value for TRAINING_PATRAM_DQ217
    NvU32 EmcTrainingPatramDq217;
    /// Specifies the value for TRAINING_PATRAM_DQ218
    NvU32 EmcTrainingPatramDq218;
    /// Specifies the value for TRAINING_PATRAM_DQ219
    NvU32 EmcTrainingPatramDq219;
    /// Specifies the value for TRAINING_PATRAM_DQ220
    NvU32 EmcTrainingPatramDq220;
    /// Specifies the value for TRAINING_PATRAM_DQ221
    NvU32 EmcTrainingPatramDq221;
    /// Specifies the value for TRAINING_PATRAM_DQ222
    NvU32 EmcTrainingPatramDq222;
    /// Specifies the value for TRAINING_PATRAM_DQ223
    NvU32 EmcTrainingPatramDq223;
    /// Specifies the value for TRAINING_PATRAM_DQ224
    NvU32 EmcTrainingPatramDq224;
    /// Specifies the value for TRAINING_PATRAM_DQ225
    NvU32 EmcTrainingPatramDq225;
    /// Specifies the value for TRAINING_PATRAM_DQ226
    NvU32 EmcTrainingPatramDq226;
    /// Specifies the value for TRAINING_PATRAM_DQ227
    NvU32 EmcTrainingPatramDq227;
    /// Specifies the value for TRAINING_PATRAM_DQ228
    NvU32 EmcTrainingPatramDq228;
    /// Specifies the value for TRAINING_PATRAM_DQ229
    NvU32 EmcTrainingPatramDq229;
    /// Specifies the value for TRAINING_PATRAM_DQ230
    NvU32 EmcTrainingPatramDq230;
    /// Specifies the value for TRAINING_PATRAM_DQ231
    NvU32 EmcTrainingPatramDq231;
    /// Specifies the value for TRAINING_PATRAM_DQ232
    NvU32 EmcTrainingPatramDq232;
    /// Specifies the value for TRAINING_PATRAM_DQ233
    NvU32 EmcTrainingPatramDq233;
    /// Specifies the value for TRAINING_PATRAM_DQ234
    NvU32 EmcTrainingPatramDq234;
    /// Specifies the value for TRAINING_PATRAM_DQ235
    NvU32 EmcTrainingPatramDq235;
    /// Specifies the value for TRAINING_PATRAM_DQ236
    NvU32 EmcTrainingPatramDq236;
    /// Specifies the value for TRAINING_PATRAM_DQ237
    NvU32 EmcTrainingPatramDq237;
    /// Specifies the value for TRAINING_PATRAM_DQ238
    NvU32 EmcTrainingPatramDq238;
    /// Specifies the value for TRAINING_PATRAM_DQ239
    NvU32 EmcTrainingPatramDq239;
    /// Specifies the value for TRAINING_PATRAM_DQ240
    NvU32 EmcTrainingPatramDq240;
    /// Specifies the value for TRAINING_PATRAM_DQ241
    NvU32 EmcTrainingPatramDq241;
    /// Specifies the value for TRAINING_PATRAM_DQ242
    NvU32 EmcTrainingPatramDq242;
    /// Specifies the value for TRAINING_PATRAM_DQ243
    NvU32 EmcTrainingPatramDq243;
    /// Specifies the value for TRAINING_PATRAM_DQ244
    NvU32 EmcTrainingPatramDq244;
    /// Specifies the value for TRAINING_PATRAM_DQ245
    NvU32 EmcTrainingPatramDq245;
    /// Specifies the value for TRAINING_PATRAM_DQ246
    NvU32 EmcTrainingPatramDq246;
    /// Specifies the value for TRAINING_PATRAM_DQ247
    NvU32 EmcTrainingPatramDq247;
    /// Specifies the value for TRAINING_PATRAM_DQ248
    NvU32 EmcTrainingPatramDq248;
    /// Specifies the value for TRAINING_PATRAM_DQ249
    NvU32 EmcTrainingPatramDq249;
    /// Specifies the value for TRAINING_PATRAM_DQ250
    NvU32 EmcTrainingPatramDq250;
    /// Specifies the value for TRAINING_PATRAM_DQ251
    NvU32 EmcTrainingPatramDq251;
    /// Specifies the value for TRAINING_PATRAM_DQ252
    NvU32 EmcTrainingPatramDq252;
    /// Specifies the value for TRAINING_PATRAM_DQ253
    NvU32 EmcTrainingPatramDq253;
    /// Specifies the value for TRAINING_PATRAM_DQ254
    NvU32 EmcTrainingPatramDq254;
    /// Specifies the value for TRAINING_PATRAM_DQ255
    NvU32 EmcTrainingPatramDq255;
    /// Specifies the value for TRAINING_PATRAM_DMI0
    NvU32 EmcTrainingPatramDmi0;
    /// Specifies the value for TRAINING_PATRAM_DMI1
    NvU32 EmcTrainingPatramDmi1;
    /// Specifies the value for TRAINING_PATRAM_DMI2
    NvU32 EmcTrainingPatramDmi2;
    /// Specifies the value for TRAINING_PATRAM_DMI3
    NvU32 EmcTrainingPatramDmi3;
    /// Specifies the value for TRAINING_PATRAM_DMI4
    NvU32 EmcTrainingPatramDmi4;
    /// Specifies the value for TRAINING_PATRAM_DMI5
    NvU32 EmcTrainingPatramDmi5;
    /// Specifies the value for TRAINING_PATRAM_DMI6
    NvU32 EmcTrainingPatramDmi6;
    /// Specifies the value for TRAINING_PATRAM_DMI7
    NvU32 EmcTrainingPatramDmi7;
    /// Specifies the value for TRAINING_PATRAM_DMI8
    NvU32 EmcTrainingPatramDmi8;
    /// Specifies the value for TRAINING_PATRAM_DMI9
    NvU32 EmcTrainingPatramDmi9;
    /// Specifies the value for TRAINING_PATRAM_DMI10
    NvU32 EmcTrainingPatramDmi10;
    /// Specifies the value for TRAINING_PATRAM_DMI11
    NvU32 EmcTrainingPatramDmi11;
    /// Specifies the value for TRAINING_PATRAM_DMI12
    NvU32 EmcTrainingPatramDmi12;
    /// Specifies the value for TRAINING_PATRAM_DMI13
    NvU32 EmcTrainingPatramDmi13;
    /// Specifies the value for TRAINING_PATRAM_DMI14
    NvU32 EmcTrainingPatramDmi14;
    /// Specifies the value for TRAINING_PATRAM_DMI15
    NvU32 EmcTrainingPatramDmi15;
    /// Specifies the value for TRAINING_PATRAM_DMI16
    NvU32 EmcTrainingPatramDmi16;
    /// Specifies the value for TRAINING_PATRAM_DMI17
    NvU32 EmcTrainingPatramDmi17;
    /// Specifies the value for TRAINING_PATRAM_DMI18
    NvU32 EmcTrainingPatramDmi18;
    /// Specifies the value for TRAINING_PATRAM_DMI19
    NvU32 EmcTrainingPatramDmi19;
    /// Specifies the value for TRAINING_PATRAM_DMI20
    NvU32 EmcTrainingPatramDmi20;
    /// Specifies the value for TRAINING_PATRAM_DMI21
    NvU32 EmcTrainingPatramDmi21;
    /// Specifies the value for TRAINING_PATRAM_DMI22
    NvU32 EmcTrainingPatramDmi22;
    /// Specifies the value for TRAINING_PATRAM_DMI23
    NvU32 EmcTrainingPatramDmi23;
    /// Specifies the value for TRAINING_PATRAM_DMI24
    NvU32 EmcTrainingPatramDmi24;
    /// Specifies the value for TRAINING_PATRAM_DMI25
    NvU32 EmcTrainingPatramDmi25;
    /// Specifies the value for TRAINING_PATRAM_DMI26
    NvU32 EmcTrainingPatramDmi26;
    /// Specifies the value for TRAINING_PATRAM_DMI27
    NvU32 EmcTrainingPatramDmi27;
    /// Specifies the value for TRAINING_PATRAM_DMI28
    NvU32 EmcTrainingPatramDmi28;
    /// Specifies the value for TRAINING_PATRAM_DMI29
    NvU32 EmcTrainingPatramDmi29;
    /// Specifies the value for TRAINING_PATRAM_DMI30
    NvU32 EmcTrainingPatramDmi30;
    /// Specifies the value for TRAINING_PATRAM_DMI31
    NvU32 EmcTrainingPatramDmi31;
    /// USED for MICRON ZQ WAR
    NvU32 EmcTrainingSpare0;
    /// Specifies the value for TRAINING_SPARE1
    NvU32 EmcTrainingSpare1;
    /// Specifies the value for TRAINING_SPARE2
    NvU32 EmcTrainingSpare2;
    /// Specifies the value for TRAINING_SPARE3
    NvU32 EmcTrainingSpare3;
    /// Specifies the value for TRAINING_SPARE4
    NvU32 EmcTrainingSpare4;
    /// Specifies the value for TRAINING_SPARE5
    NvU32 EmcTrainingSpare5;
    /// Specifies the value for TRAINING_SPARE6
    NvU32 EmcTrainingSpare6;
    /// Specifies the value for TRAINING_SPARE7
    NvU32 EmcTrainingSpare7;
    /// Specifies the value for TRAINING_SPARE8
    NvU32 EmcTrainingSpare8;
    /// Specifies the value for TRAINING_SPARE9
    NvU32 EmcTrainingSpare9;
    /// Specifies the value for TRAINING_SPARE10
    NvU32 EmcTrainingSpare10;
    /// Specifies the value for TRAINING_SPARE11
    NvU32 EmcTrainingSpare11;
    /// Specifies the value for TRAINING_SPARE12
    NvU32 EmcTrainingSpare12;
    /// Specifies the value for TRAINING_SPARE13
    NvU32 EmcTrainingSpare13;
    /// Specifies the value for TRAINING_SPARE14
    NvU32 EmcTrainingSpare14;
    /// Specifies the value for TRAINING_SPARE15
    NvU32 EmcTrainingSpare15;
    /// Specifies the value for TRAINING_SPARE16
    NvU32 EmcTrainingSpare16;
    /// Specifies the value for TRAINING_SPARE17
    NvU32 EmcTrainingSpare17;
    /// Specifies the value for TRAINING_SPARE18
    NvU32 EmcTrainingSpare18;
    /// Specifies the value for TRAINING_SPARE19
    NvU32 EmcTrainingSpare19;
    /// Specifies the value for TRAINING_SPARE20
    NvU32 EmcTrainingSpare20;
    /// Specifies the value for TRAINING_SPARE21
    NvU32 EmcTrainingSpare21;
    /// Specifies the value for TRAINING_SPARE22
    NvU32 EmcTrainingSpare22;
    /// Specifies the value for TRAINING_SPARE23
    NvU32 EmcTrainingSpare23;
    /// Specifies the value for TRAINING_SPARE24
    NvU32 EmcTrainingSpare24;
    /// Specifies the value for TRAINING_SPARE25
    NvU32 EmcTrainingSpare25;
    /// Specifies the value for TRAINING_SPARE26
    NvU32 EmcTrainingSpare26;
    /// Specifies the value for TRAINING_SPARE27
    NvU32 EmcTrainingSpare27;
    /// Specifies the value for TRAINING_SPARE28
    NvU32 EmcTrainingSpare28;
    /// Specifies the value for TRAINING_SPARE29
    NvU32 EmcTrainingSpare29;
    /// Specifies the value for TRAINING_SPARE30
    NvU32 EmcTrainingSpare30;
    /// Specifies the value for TRAINING_SPARE31
    NvU32 EmcTrainingSpare31;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR
    NvU32 McSidStreamidOverrideConfigPtcr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PTCR
    NvU32 McSidStreamidSecurityConfigPtcr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_HDAR
    NvU32 McSidStreamidOverrideConfigHdar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_HDAR
    NvU32 McSidStreamidSecurityConfigHdar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_HOST1XDMAR
    NvU32 McSidStreamidOverrideConfigHost1xdmar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_HOST1XDMAR
    NvU32 McSidStreamidSecurityConfigHost1xdmar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD
    NvU32 McSidStreamidOverrideConfigNvencsrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD
    NvU32 McSidStreamidSecurityConfigNvencsrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SATAR
    NvU32 McSidStreamidOverrideConfigSatar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SATAR
    NvU32 McSidStreamidSecurityConfigSatar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MPCORER
    NvU32 McSidStreamidOverrideConfigMpcorer;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MPCORER
    NvU32 McSidStreamidSecurityConfigMpcorer;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSWR
    NvU32 McSidStreamidOverrideConfigNvencswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENCSWR
    NvU32 McSidStreamidSecurityConfigNvencswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_HDAW
    NvU32 McSidStreamidOverrideConfigHdaw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_HDAW
    NvU32 McSidStreamidSecurityConfigHdaw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MPCOREW
    NvU32 McSidStreamidOverrideConfigMpcorew;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MPCOREW
    NvU32 McSidStreamidSecurityConfigMpcorew;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SATAW
    NvU32 McSidStreamidOverrideConfigSataw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SATAW
    NvU32 McSidStreamidSecurityConfigSataw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA
    NvU32 McSidStreamidOverrideConfigIspra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPRA
    NvU32 McSidStreamidSecurityConfigIspra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPFALR
    NvU32 McSidStreamidOverrideConfigIspfalr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPFALR
    NvU32 McSidStreamidSecurityConfigIspfalr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWA
    NvU32 McSidStreamidOverrideConfigIspwa;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPWA
    NvU32 McSidStreamidSecurityConfigIspwa;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWB
    NvU32 McSidStreamidOverrideConfigIspwb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPWB
    NvU32 McSidStreamidSecurityConfigIspwb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR
    NvU32 McSidStreamidOverrideConfigXusb_hostr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTR
    NvU32 McSidStreamidSecurityConfigXusb_hostr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW
    NvU32 McSidStreamidOverrideConfigXusb_hostw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTW
    NvU32 McSidStreamidSecurityConfigXusb_hostw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVR
    NvU32 McSidStreamidOverrideConfigXusb_devr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVR
    NvU32 McSidStreamidSecurityConfigXusb_devr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVW
    NvU32 McSidStreamidOverrideConfigXusb_devw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVW
    NvU32 McSidStreamidSecurityConfigXusb_devw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRD
    NvU32 McSidStreamidOverrideConfigTsecsrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSRD
    NvU32 McSidStreamidSecurityConfigTsecsrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWR
    NvU32 McSidStreamidOverrideConfigTsecswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSWR
    NvU32 McSidStreamidSecurityConfigTsecswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA
    NvU32 McSidStreamidOverrideConfigSdmmcra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA
    NvU32 McSidStreamidSecurityConfigSdmmcra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCR
    NvU32 McSidStreamidOverrideConfigSdmmcr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCR
    NvU32 McSidStreamidSecurityConfigSdmmcr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAB
    NvU32 McSidStreamidOverrideConfigSdmmcrab;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAB
    NvU32 McSidStreamidSecurityConfigSdmmcrab;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWA
    NvU32 McSidStreamidOverrideConfigSdmmcwa;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWA
    NvU32 McSidStreamidSecurityConfigSdmmcwa;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCW
    NvU32 McSidStreamidOverrideConfigSdmmcw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCW
    NvU32 McSidStreamidSecurityConfigSdmmcw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAB
    NvU32 McSidStreamidOverrideConfigSdmmcwab;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAB
    NvU32 McSidStreamidSecurityConfigSdmmcwab;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD
    NvU32 McSidStreamidOverrideConfigVicsrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VICSRD
    NvU32 McSidStreamidSecurityConfigVicsrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VICSWR
    NvU32 McSidStreamidOverrideConfigVicswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VICSWR
    NvU32 McSidStreamidSecurityConfigVicswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VIW
    NvU32 McSidStreamidOverrideConfigViw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VIW
    NvU32 McSidStreamidSecurityConfigViw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD
    NvU32 McSidStreamidOverrideConfigNvdecsrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD
    NvU32 McSidStreamidSecurityConfigNvdecsrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSWR
    NvU32 McSidStreamidOverrideConfigNvdecswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDECSWR
    NvU32 McSidStreamidSecurityConfigNvdecswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APER
    NvU32 McSidStreamidOverrideConfigAper;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APER
    NvU32 McSidStreamidSecurityConfigAper;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APEW
    NvU32 McSidStreamidOverrideConfigApew;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APEW
    NvU32 McSidStreamidSecurityConfigApew;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSRD
    NvU32 McSidStreamidOverrideConfigNvjpgsrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSRD
    NvU32 McSidStreamidSecurityConfigNvjpgsrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSWR
    NvU32 McSidStreamidOverrideConfigNvjpgswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSWR
    NvU32 McSidStreamidSecurityConfigNvjpgswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SESRD
    NvU32 McSidStreamidOverrideConfigSesrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SESRD
    NvU32 McSidStreamidSecurityConfigSesrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SESWR
    NvU32 McSidStreamidOverrideConfigSeswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SESWR
    NvU32 McSidStreamidSecurityConfigSeswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AXIAPR
    NvU32 McSidStreamidOverrideConfigAxiapr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AXIAPR
    NvU32 McSidStreamidSecurityConfigAxiapr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AXIAPW
    NvU32 McSidStreamidOverrideConfigAxiapw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AXIAPW
    NvU32 McSidStreamidSecurityConfigAxiapw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ETRR
    NvU32 McSidStreamidOverrideConfigEtrr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ETRR
    NvU32 McSidStreamidSecurityConfigEtrr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ETRW
    NvU32 McSidStreamidOverrideConfigEtrw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ETRW
    NvU32 McSidStreamidSecurityConfigEtrw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRDB
    NvU32 McSidStreamidOverrideConfigTsecsrdb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSRDB
    NvU32 McSidStreamidSecurityConfigTsecsrdb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWRB
    NvU32 McSidStreamidOverrideConfigTsecswrb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSWRB
    NvU32 McSidStreamidSecurityConfigTsecswrb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AXISR
    NvU32 McSidStreamidOverrideConfigAxisr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AXISR
    NvU32 McSidStreamidSecurityConfigAxisr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AXISW
    NvU32 McSidStreamidOverrideConfigAxisw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AXISW
    NvU32 McSidStreamidSecurityConfigAxisw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSR
    NvU32 McSidStreamidOverrideConfigEqosr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_EQOSR
    NvU32 McSidStreamidSecurityConfigEqosr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSW
    NvU32 McSidStreamidOverrideConfigEqosw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_EQOSW
    NvU32 McSidStreamidSecurityConfigEqosw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCR
    NvU32 McSidStreamidOverrideConfigUfshcr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_UFSHCR
    NvU32 McSidStreamidSecurityConfigUfshcr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCW
    NvU32 McSidStreamidOverrideConfigUfshcw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_UFSHCW
    NvU32 McSidStreamidSecurityConfigUfshcw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR
    NvU32 McSidStreamidOverrideConfigNvdisplayr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR
    NvU32 McSidStreamidSecurityConfigNvdisplayr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPR
    NvU32 McSidStreamidOverrideConfigBpmpr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPR
    NvU32 McSidStreamidSecurityConfigBpmpr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPW
    NvU32 McSidStreamidOverrideConfigBpmpw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPW
    NvU32 McSidStreamidSecurityConfigBpmpw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAR
    NvU32 McSidStreamidOverrideConfigBpmpdmar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAR
    NvU32 McSidStreamidSecurityConfigBpmpdmar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAW
    NvU32 McSidStreamidOverrideConfigBpmpdmaw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAW
    NvU32 McSidStreamidSecurityConfigBpmpdmaw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONR
    NvU32 McSidStreamidOverrideConfigAonr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONR
    NvU32 McSidStreamidSecurityConfigAonr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONW
    NvU32 McSidStreamidOverrideConfigAonw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONW
    NvU32 McSidStreamidSecurityConfigAonw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAR
    NvU32 McSidStreamidOverrideConfigAondmar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONDMAR
    NvU32 McSidStreamidSecurityConfigAondmar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAW
    NvU32 McSidStreamidOverrideConfigAondmaw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONDMAW
    NvU32 McSidStreamidSecurityConfigAondmaw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCER
    NvU32 McSidStreamidOverrideConfigScer;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCER
    NvU32 McSidStreamidSecurityConfigScer;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCEW
    NvU32 McSidStreamidOverrideConfigScew;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCEW
    NvU32 McSidStreamidSecurityConfigScew;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAR
    NvU32 McSidStreamidOverrideConfigScedmar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAR
    NvU32 McSidStreamidSecurityConfigScedmar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAW
    NvU32 McSidStreamidOverrideConfigScedmaw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAW
    NvU32 McSidStreamidSecurityConfigScedmaw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAR
    NvU32 McSidStreamidOverrideConfigApedmar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APEDMAR
    NvU32 McSidStreamidSecurityConfigApedmar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAW
    NvU32 McSidStreamidOverrideConfigApedmaw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APEDMAW
    NvU32 McSidStreamidSecurityConfigApedmaw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1
    NvU32 McSidStreamidOverrideConfigNvdisplayr1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR1
    NvU32 McSidStreamidSecurityConfigNvdisplayr1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD1
    NvU32 McSidStreamidOverrideConfigVicsrd1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VICSRD1
    NvU32 McSidStreamidSecurityConfigVicsrd1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD1
    NvU32 McSidStreamidOverrideConfigNvdecsrd1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD1
    NvU32 McSidStreamidSecurityConfigNvdecsrd1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VIFALR
    NvU32 McSidStreamidOverrideConfigVifalr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VIFALR
    NvU32 McSidStreamidSecurityConfigVifalr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VIFALW
    NvU32 McSidStreamidOverrideConfigVifalw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VIFALW
    NvU32 McSidStreamidSecurityConfigVifalw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA0RDA
    NvU32 McSidStreamidOverrideConfigDla0rda;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA0RDA
    NvU32 McSidStreamidSecurityConfigDla0rda;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA0FALRDB
    NvU32 McSidStreamidOverrideConfigDla0falrdb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA0FALRDB
    NvU32 McSidStreamidSecurityConfigDla0falrdb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA0WRA
    NvU32 McSidStreamidOverrideConfigDla0wra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA0WRA
    NvU32 McSidStreamidSecurityConfigDla0wra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA0FALWRB
    NvU32 McSidStreamidOverrideConfigDla0falwrb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA0FALWRB
    NvU32 McSidStreamidSecurityConfigDla0falwrb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA1RDA
    NvU32 McSidStreamidOverrideConfigDla1rda;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA1RDA
    NvU32 McSidStreamidSecurityConfigDla1rda;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA1FALRDB
    NvU32 McSidStreamidOverrideConfigDla1falrdb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA1FALRDB
    NvU32 McSidStreamidSecurityConfigDla1falrdb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA1WRA
    NvU32 McSidStreamidOverrideConfigDla1wra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA1WRA
    NvU32 McSidStreamidSecurityConfigDla1wra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA1FALWRB
    NvU32 McSidStreamidOverrideConfigDla1falwrb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA1FALWRB
    NvU32 McSidStreamidSecurityConfigDla1falwrb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0RDA
    NvU32 McSidStreamidOverrideConfigPva0rda;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0RDA
    NvU32 McSidStreamidSecurityConfigPva0rda;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0RDB
    NvU32 McSidStreamidOverrideConfigPva0rdb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0RDB
    NvU32 McSidStreamidSecurityConfigPva0rdb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0RDC
    NvU32 McSidStreamidOverrideConfigPva0rdc;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0RDC
    NvU32 McSidStreamidSecurityConfigPva0rdc;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0WRA
    NvU32 McSidStreamidOverrideConfigPva0wra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0WRA
    NvU32 McSidStreamidSecurityConfigPva0wra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0WRB
    NvU32 McSidStreamidOverrideConfigPva0wrb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0WRB
    NvU32 McSidStreamidSecurityConfigPva0wrb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0WRC
    NvU32 McSidStreamidOverrideConfigPva0wrc;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0WRC
    NvU32 McSidStreamidSecurityConfigPva0wrc;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1RDA
    NvU32 McSidStreamidOverrideConfigPva1rda;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1RDA
    NvU32 McSidStreamidSecurityConfigPva1rda;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1RDB
    NvU32 McSidStreamidOverrideConfigPva1rdb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1RDB
    NvU32 McSidStreamidSecurityConfigPva1rdb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1RDC
    NvU32 McSidStreamidOverrideConfigPva1rdc;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1RDC
    NvU32 McSidStreamidSecurityConfigPva1rdc;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1WRA
    NvU32 McSidStreamidOverrideConfigPva1wra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1WRA
    NvU32 McSidStreamidSecurityConfigPva1wra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1WRB
    NvU32 McSidStreamidOverrideConfigPva1wrb;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1WRB
    NvU32 McSidStreamidSecurityConfigPva1wrb;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1WRC
    NvU32 McSidStreamidOverrideConfigPva1wrc;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1WRC
    NvU32 McSidStreamidSecurityConfigPva1wrc;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_RCER
    NvU32 McSidStreamidOverrideConfigRcer;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_RCER
    NvU32 McSidStreamidSecurityConfigRcer;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_RCEW
    NvU32 McSidStreamidOverrideConfigRcew;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_RCEW
    NvU32 McSidStreamidSecurityConfigRcew;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_RCEDMAR
    NvU32 McSidStreamidOverrideConfigRcedmar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_RCEDMAR
    NvU32 McSidStreamidSecurityConfigRcedmar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_RCEDMAW
    NvU32 McSidStreamidOverrideConfigRcedmaw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_RCEDMAW
    NvU32 McSidStreamidSecurityConfigRcedmaw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENC1SRD
    NvU32 McSidStreamidOverrideConfigNvenc1srd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENC1SRD
    NvU32 McSidStreamidSecurityConfigNvenc1srd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENC1SWR
    NvU32 McSidStreamidOverrideConfigNvenc1swr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENC1SWR
    NvU32 McSidStreamidSecurityConfigNvenc1swr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE0R
    NvU32 McSidStreamidOverrideConfigPcie0r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE0R
    NvU32 McSidStreamidSecurityConfigPcie0r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE0W
    NvU32 McSidStreamidOverrideConfigPcie0w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE0W
    NvU32 McSidStreamidSecurityConfigPcie0w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE1R
    NvU32 McSidStreamidOverrideConfigPcie1r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE1R
    NvU32 McSidStreamidSecurityConfigPcie1r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE1W
    NvU32 McSidStreamidOverrideConfigPcie1w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE1W
    NvU32 McSidStreamidSecurityConfigPcie1w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE2AR
    NvU32 McSidStreamidOverrideConfigPcie2ar;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE2AR
    NvU32 McSidStreamidSecurityConfigPcie2ar;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE2AW
    NvU32 McSidStreamidOverrideConfigPcie2aw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE2AW
    NvU32 McSidStreamidSecurityConfigPcie2aw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE3R
    NvU32 McSidStreamidOverrideConfigPcie3r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE3R
    NvU32 McSidStreamidSecurityConfigPcie3r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE3W
    NvU32 McSidStreamidOverrideConfigPcie3w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE3W
    NvU32 McSidStreamidSecurityConfigPcie3w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE4R
    NvU32 McSidStreamidOverrideConfigPcie4r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE4R
    NvU32 McSidStreamidSecurityConfigPcie4r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE4W
    NvU32 McSidStreamidOverrideConfigPcie4w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE4W
    NvU32 McSidStreamidSecurityConfigPcie4w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE5R
    NvU32 McSidStreamidOverrideConfigPcie5r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE5R
    NvU32 McSidStreamidSecurityConfigPcie5r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE5W
    NvU32 McSidStreamidOverrideConfigPcie5w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE5W
    NvU32 McSidStreamidSecurityConfigPcie5w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPFALW
    NvU32 McSidStreamidOverrideConfigIspfalw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPFALW
    NvU32 McSidStreamidSecurityConfigIspfalw;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA0RDA1
    NvU32 McSidStreamidOverrideConfigDla0rda1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA0RDA1
    NvU32 McSidStreamidSecurityConfigDla0rda1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_DLA1RDA1
    NvU32 McSidStreamidOverrideConfigDla1rda1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_DLA1RDA1
    NvU32 McSidStreamidSecurityConfigDla1rda1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0RDA1
    NvU32 McSidStreamidOverrideConfigPva0rda1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0RDA1
    NvU32 McSidStreamidSecurityConfigPva0rda1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA0RDB1
    NvU32 McSidStreamidOverrideConfigPva0rdb1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA0RDB1
    NvU32 McSidStreamidSecurityConfigPva0rdb1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1RDA1
    NvU32 McSidStreamidOverrideConfigPva1rda1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1RDA1
    NvU32 McSidStreamidSecurityConfigPva1rda1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PVA1RDB1
    NvU32 McSidStreamidOverrideConfigPva1rdb1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PVA1RDB1
    NvU32 McSidStreamidSecurityConfigPva1rdb1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE5R1
    NvU32 McSidStreamidOverrideConfigPcie5r1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE5R1
    NvU32 McSidStreamidSecurityConfigPcie5r1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD1
    NvU32 McSidStreamidOverrideConfigNvencsrd1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD1
    NvU32 McSidStreamidSecurityConfigNvencsrd1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENC1SRD1
    NvU32 McSidStreamidOverrideConfigNvenc1srd1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENC1SRD1
    NvU32 McSidStreamidSecurityConfigNvenc1srd1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA1
    NvU32 McSidStreamidOverrideConfigIspra1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPRA1
    NvU32 McSidStreamidSecurityConfigIspra1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PCIE0R1
    NvU32 McSidStreamidOverrideConfigPcie0r1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PCIE0R1
    NvU32 McSidStreamidSecurityConfigPcie0r1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDEC1SRD
    NvU32 McSidStreamidOverrideConfigNvdec1srd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDEC1SRD
    NvU32 McSidStreamidSecurityConfigNvdec1srd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDEC1SRD1
    NvU32 McSidStreamidOverrideConfigNvdec1srd1;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDEC1SRD1
    NvU32 McSidStreamidSecurityConfigNvdec1srd1;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDEC1SWR
    NvU32 McSidStreamidOverrideConfigNvdec1swr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDEC1SWR
    NvU32 McSidStreamidSecurityConfigNvdec1swr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU0R
    NvU32 McSidStreamidOverrideConfigMiu0r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU0R
    NvU32 McSidStreamidSecurityConfigMiu0r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU0W
    NvU32 McSidStreamidOverrideConfigMiu0w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU0W
    NvU32 McSidStreamidSecurityConfigMiu0w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU1R
    NvU32 McSidStreamidOverrideConfigMiu1r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU1R
    NvU32 McSidStreamidSecurityConfigMiu1r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU1W
    NvU32 McSidStreamidOverrideConfigMiu1w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU1W
    NvU32 McSidStreamidSecurityConfigMiu1w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU2R
    NvU32 McSidStreamidOverrideConfigMiu2r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU2R
    NvU32 McSidStreamidSecurityConfigMiu2r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU2W
    NvU32 McSidStreamidOverrideConfigMiu2w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU2W
    NvU32 McSidStreamidSecurityConfigMiu2w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU3R
    NvU32 McSidStreamidOverrideConfigMiu3r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU3R
    NvU32 McSidStreamidSecurityConfigMiu3r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU3W
    NvU32 McSidStreamidOverrideConfigMiu3w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU3W
    NvU32 McSidStreamidSecurityConfigMiu3w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU4R
    NvU32 McSidStreamidOverrideConfigMiu4r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU4R
    NvU32 McSidStreamidSecurityConfigMiu4r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU4W
    NvU32 McSidStreamidOverrideConfigMiu4w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU4W
    NvU32 McSidStreamidSecurityConfigMiu4w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU5R
    NvU32 McSidStreamidOverrideConfigMiu5r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU5R
    NvU32 McSidStreamidSecurityConfigMiu5r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU5W
    NvU32 McSidStreamidOverrideConfigMiu5w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU5W
    NvU32 McSidStreamidSecurityConfigMiu5w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU6R
    NvU32 McSidStreamidOverrideConfigMiu6r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU6R
    NvU32 McSidStreamidSecurityConfigMiu6r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU6W
    NvU32 McSidStreamidOverrideConfigMiu6w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU6W
    NvU32 McSidStreamidSecurityConfigMiu6w;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU7R
    NvU32 McSidStreamidOverrideConfigMiu7r;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU7R
    NvU32 McSidStreamidSecurityConfigMiu7r;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MIU7W
    NvU32 McSidStreamidOverrideConfigMiu7w;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MIU7W
    NvU32 McSidStreamidSecurityConfigMiu7w;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_0_CH2
    NvU32 EmcPmacroAutocalCfg0_4;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2_CH2
    NvU32 EmcPmacroAutocalCfg2_4;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_4;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_6;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_4;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_6;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_4;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_6;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_4;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_6;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_4;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_6;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_4;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_6;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_4;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_6;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_4;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_6;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_6;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_4;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_6;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_4;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_6;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_4;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_6;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_4;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_6;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_4;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_6;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_4;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_6;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_4;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_6;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_4;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_6;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_4;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_6;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_4;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_6;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_4;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_6;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_4;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_6;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_4;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_6;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1_4;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2_4;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3_4;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_4;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_6;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_4;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_6;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_4;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_6;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_4;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_6;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_0_CH2
    NvU32 EmcPmacroAutocalCfg0_8;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_0_CH2
    NvU32 EmcPmacroAutocalCfg0_12;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2_CH2
    NvU32 EmcPmacroAutocalCfg2_8;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2_CH2
    NvU32 EmcPmacroAutocalCfg2_12;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_8;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_10;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_12;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot CH2
    NvU32 EmcMrw9_14;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_8;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_10;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_12;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0_14;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_8;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_10;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_12;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1_14;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_8;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_10;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_12;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2_14;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_8;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_10;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_12;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0_14;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_8;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_10;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_12;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1_14;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_8;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_10;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_12;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2_14;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_8;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_10;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_12;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_4_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_5_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_0_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_1_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_14;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_8;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_10;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_12;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_14;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_8;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_10;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_12;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_14;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_8;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_10;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_12;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_14;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_8;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_10;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_12;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_14;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_8;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_10;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_12;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_14;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_8;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_10;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_12;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0_CH2
    NvU32 EmcPmacroDdllLongCmd_0_14;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_8;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_10;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_12;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1_CH2
    NvU32 EmcPmacroDdllLongCmd_1_14;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_8;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_10;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_12;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2_CH2
    NvU32 EmcPmacroDdllLongCmd_2_14;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_8;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_10;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_12;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3_CH2
    NvU32 EmcPmacroDdllLongCmd_3_14;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_8;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_10;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_12;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4_CH2
    NvU32 EmcPmacroDdllLongCmd_4_14;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_8;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_10;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_12;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0_CH2
    NvU32 EmcSwizzleRank0Byte0_14;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_8;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_10;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_12;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1_CH2
    NvU32 EmcSwizzleRank0Byte1_14;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_8;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_10;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_12;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2_CH2
    NvU32 EmcSwizzleRank0Byte2_14;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_8;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_10;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_12;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3_CH2
    NvU32 EmcSwizzleRank0Byte3_14;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_8;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_10;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_12;
    /// Specifies the value for EMC_DATA_BRLSHFT_0_CH2
    NvU32 EmcDataBrlshft0_14;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_8;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_10;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_12;
    /// Specifies the value for EMC_DATA_BRLSHFT_1_CH2
    NvU32 EmcDataBrlshft1_14;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1_8;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1_12;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2_8;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2_12;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3_8;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3_12;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_8;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_10;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_12;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0_CH2
    NvU32 EmcPmacroBrickMapping0_14;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_8;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_10;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_12;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1_CH2
    NvU32 EmcPmacroBrickMapping1_14;
    /// just a place holder for special usage when there is no BCT for certain registers
    NvU32 BCT_NA;

} NvBootSdramParams
;
#endif //NVBOOT_SDRAM_PARAM_GENERATED_H

    // End of generated code by warmboot_code_gen
