/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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
    /// Specifies the value for EMC_AUTO_CAL_CONFIG3_CH0
    NvU32 EmcAutoCalConfig3_0;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG3_CH2
    NvU32 EmcAutoCalConfig3_2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG4_CH0
    NvU32 EmcAutoCalConfig4_0;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG4_CH2
    NvU32 EmcAutoCalConfig4_2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG5_CH0
    NvU32 EmcAutoCalConfig5_0;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG5_CH2
    NvU32 EmcAutoCalConfig5_2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG6_CH0
    NvU32 EmcAutoCalConfig6_0;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG6_CH2
    NvU32 EmcAutoCalConfig6_2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG7_CH0
    NvU32 EmcAutoCalConfig7_0;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG7_CH2
    NvU32 EmcAutoCalConfig7_2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG8_CH0
    NvU32 EmcAutoCalConfig8_0;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG8_CH2
    NvU32 EmcAutoCalConfig8_2;
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
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_0_CH2
    NvU32 EmcPmacroAutocalCfg0_2;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_1_CH0
    NvU32 EmcPmacroAutocalCfg1_0;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_1_CH2
    NvU32 EmcPmacroAutocalCfg1_2;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2_CH0
    NvU32 EmcPmacroAutocalCfg2_0;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2_CH2
    NvU32 EmcPmacroAutocalCfg2_2;
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

    /// FBIO configuration values
    /// 
    /// Specifies the value for EMC_FBIO_CFG8_CH0
    NvU32 EmcFbioCfg8_0;

    /// FBIO configuration values
    /// 
    /// Specifies the value for EMC_FBIO_CFG8_CH2
    NvU32 EmcFbioCfg8_2;
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
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_0_0;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_0_2;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_1_0;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_1_2;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_2_0;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_2_2;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_0_0;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_0_2;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_1_0;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_1_2;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_2_0;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_2_2;
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
    /// Specifies the pipe bypass controls
    NvU32 EmcCfgPipe_0;
    /// Specifies the pipe bypass controls
    NvU32 EmcCfgPipe_2;
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
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0_CH0
    NvU32 EmcPmacroIbVrefDq_0_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0_CH2
    NvU32 EmcPmacroIbVrefDq_0_2;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1_CH0
    NvU32 EmcPmacroIbVrefDq_1_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1_CH2
    NvU32 EmcPmacroIbVrefDq_1_2;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0_CH0
    NvU32 EmcPmacroIbVrefDqs_0_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0_CH2
    NvU32 EmcPmacroIbVrefDqs_0_2;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1_CH0
    NvU32 EmcPmacroIbVrefDqs_1_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1_CH2
    NvU32 EmcPmacroIbVrefDqs_1_2;
    /// Specifies the value for EMC_PMACRO_IB_RXRT
    NvU32 EmcPmacroIbRxrt;
    /// Specifies the value for EMC_CFG_PIPE_1_CH0
    NvU32 EmcCfgPipe1_0;
    /// Specifies the value for EMC_CFG_PIPE_1_CH2
    NvU32 EmcCfgPipe1_2;
    /// Specifies the value for EMC_CFG_PIPE_2_CH0
    NvU32 EmcCfgPipe2_0;
    /// Specifies the value for EMC_CFG_PIPE_2_CH2
    NvU32 EmcCfgPipe2_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_0_CH0
    NvU32 EmcPmacroQuseDdllRank0_0_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_0_CH2
    NvU32 EmcPmacroQuseDdllRank0_0_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_1_CH0
    NvU32 EmcPmacroQuseDdllRank0_1_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_1_CH2
    NvU32 EmcPmacroQuseDdllRank0_1_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_2_CH0
    NvU32 EmcPmacroQuseDdllRank0_2_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_2_CH2
    NvU32 EmcPmacroQuseDdllRank0_2_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_3_CH0
    NvU32 EmcPmacroQuseDdllRank0_3_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_3_CH2
    NvU32 EmcPmacroQuseDdllRank0_3_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_4_CH0
    NvU32 EmcPmacroQuseDdllRank0_4_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_4_CH2
    NvU32 EmcPmacroQuseDdllRank0_4_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_5_CH0
    NvU32 EmcPmacroQuseDdllRank0_5_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_5_CH2
    NvU32 EmcPmacroQuseDdllRank0_5_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_0_CH0
    NvU32 EmcPmacroQuseDdllRank1_0_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_0_CH2
    NvU32 EmcPmacroQuseDdllRank1_0_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_1_CH0
    NvU32 EmcPmacroQuseDdllRank1_1_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_1_CH2
    NvU32 EmcPmacroQuseDdllRank1_1_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_2_CH0
    NvU32 EmcPmacroQuseDdllRank1_2_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_2_CH2
    NvU32 EmcPmacroQuseDdllRank1_2_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_3_CH0
    NvU32 EmcPmacroQuseDdllRank1_3_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_3_CH2
    NvU32 EmcPmacroQuseDdllRank1_3_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_4_CH0
    NvU32 EmcPmacroQuseDdllRank1_4_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_4_CH2
    NvU32 EmcPmacroQuseDdllRank1_4_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_5_CH0
    NvU32 EmcPmacroQuseDdllRank1_5_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_5_CH2
    NvU32 EmcPmacroQuseDdllRank1_5_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_2_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_2_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_3_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_3_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_4_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_5_CH0
    NvU32 EmcPmacroObDdllLongDqRank0_5_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_5_CH2
    NvU32 EmcPmacroObDdllLongDqRank0_5_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_2_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_2_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3_CH0
    NvU32 EmcPmacroObDdllLongDqRank1_3_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3_CH2
    NvU32 EmcPmacroObDdllLongDqRank1_3_2;
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
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_2_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_2_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_2_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_2_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_3_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_3_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_3_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_3_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_4_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_5_CH0
    NvU32 EmcPmacroObDdllLongDqsRank0_5_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_5_CH2
    NvU32 EmcPmacroObDdllLongDqsRank0_5_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_2_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_2_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_2_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_2_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_3_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_3_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_3_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_3_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_4_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_4_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_5_CH0
    NvU32 EmcPmacroObDdllLongDqsRank1_5_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_5_CH2
    NvU32 EmcPmacroObDdllLongDqsRank1_5_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_0_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_1_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_2_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank0_2_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_2_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_2_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_3_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank0_3_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_3_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank0_3_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_0_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_0_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_0_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_1_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_1_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_1_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_2_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank1_2_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_2_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_2_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_3_CH0
    NvU32 EmcPmacroIbDdllLongDqsRank1_3_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_3_CH2
    NvU32 EmcPmacroIbDdllLongDqsRank1_3_2;
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
    /// Specifies the master override for all MC clocks
    NvU32 McClkenOverrideAllWarmBoot;
    /// Specifies digital dll period, choosing between 4 to 64 ms
    NvU32 EmcCfgDigDllPeriodWarmBoot;

    /// Pad controls
    /// 
    /// Specifies the value for PMC_VDDP_SEL
    NvU32 PmcVddpSel;
    /// Specifies the wait time after programming PMC_VDDP_SEL
    NvU32 PmcVddpSelWait;
    /// No longer used in MSS INIT
    NvU32 PmcDdrPwr;
    /// Specifies the value for PMC_DDR_CFG
    NvU32 PmcDdrCfg;
    /// Specifies the value for PMC_DDR_CFG_MEM1
    NvU32 PmcDdrCfgMem1;
    /// Specifies the value for PMC_IO_WEAK_BIAS
    NvU32 PmcIoWb;
    /// Specifies the value for PMC_IO_WEAK_BIAS_MEM1
    NvU32 PmcIoWbMem1;
    /// Specifies the value for PMC_IO_DPD3_REQ
    NvU32 PmcIoDpd3Req;
    /// Specifies the wait time after programming PMC_IO_DPD3_REQ
    NvU32 PmcIoDpd3ReqWait;
    /// Specifies the wait time after programming PMC_IO_DPD4_REQ
    NvU32 PmcIoDpd4ReqWait;
    /// Not used in MSS INIT
    NvU32 PmcBlinkTimer;
    /// Specifies the value for PMC_NO_IOPOWER
    NvU32 PmcNoIoPower;
    /// Specifies the wait time after programing PMC_DDR_CNTRL
    NvU32 PmcDdrCntrlWait;
    /// Specifies the value for PMC_DDR_CNTRL
    NvU32 PmcDdrCntrl;
    /// Specifies the value for PMC_DDR_CNTRL_MEM1
    NvU32 PmcDdrCntrlMem1;
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
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE0_CH0
    NvU32 EmcSwizzleRank1Byte0_0;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE0_CH2
    NvU32 EmcSwizzleRank1Byte0_2;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE1_CH0
    NvU32 EmcSwizzleRank1Byte1_0;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE1_CH2
    NvU32 EmcSwizzleRank1Byte1_2;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE2_CH0
    NvU32 EmcSwizzleRank1Byte2_0;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE2_CH2
    NvU32 EmcSwizzleRank1Byte2_2;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE3_CH0
    NvU32 EmcSwizzleRank1Byte3_0;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE3_CH2
    NvU32 EmcSwizzleRank1Byte3_2;

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
    NvU32 EmcDqsBrlshft0_0;
    /// Specifies the value for EMC_DQS_BRLSHFT_0_CH2
    NvU32 EmcDqsBrlshft0_2;
    /// Specifies the value for EMC_DQS_BRLSHFT_1_CH0
    NvU32 EmcDqsBrlshft1_0;
    /// Specifies the value for EMC_DQS_BRLSHFT_1_CH2
    NvU32 EmcDqsBrlshft1_2;
    /// Specifies the value for EMC_CMD_BRLSHFT_0_CH0
    NvU32 EmcCmdBrlshft0_0;
    /// Specifies the value for EMC_CMD_BRLSHFT_0_CH2
    NvU32 EmcCmdBrlshft0_2;
    /// Specifies the value for EMC_CMD_BRLSHFT_1_CH0
    NvU32 EmcCmdBrlshft1_0;
    /// Specifies the value for EMC_CMD_BRLSHFT_1_CH2
    NvU32 EmcCmdBrlshft1_2;
    /// Specifies the value for EMC_CMD_BRLSHFT_2_CH0
    NvU32 EmcCmdBrlshft2_0;
    /// Specifies the value for EMC_CMD_BRLSHFT_2_CH2
    NvU32 EmcCmdBrlshft2_2;
    /// Specifies the value for EMC_CMD_BRLSHFT_3_CH0
    NvU32 EmcCmdBrlshft3_0;
    /// Specifies the value for EMC_CMD_BRLSHFT_3_CH2
    NvU32 EmcCmdBrlshft3_2;
    /// Specifies the value for EMC_QUSE_BRLSHFT_0_CH0
    NvU32 EmcQuseBrlshft0_0;
    /// Specifies the value for EMC_QUSE_BRLSHFT_0_CH2
    NvU32 EmcQuseBrlshft0_2;
    /// Specifies the value for EMC_QUSE_BRLSHFT_1_CH0
    NvU32 EmcQuseBrlshft1_0;
    /// Specifies the value for EMC_QUSE_BRLSHFT_1_CH2
    NvU32 EmcQuseBrlshft1_2;
    /// Specifies the value for EMC_QUSE_BRLSHFT_2_CH0
    NvU32 EmcQuseBrlshft2_0;
    /// Specifies the value for EMC_QUSE_BRLSHFT_2_CH2
    NvU32 EmcQuseBrlshft2_2;
    /// Specifies the value for EMC_QUSE_BRLSHFT_3_CH0
    NvU32 EmcQuseBrlshft3_0;
    /// Specifies the value for EMC_QUSE_BRLSHFT_3_CH2
    NvU32 EmcQuseBrlshft3_2;
    /// Specifies the value for EMC_DLL_CFG_0
    NvU32 EmcDllCfg0;
    /// Specifies the value for EMC_DLL_CFG_1
    NvU32 EmcDllCfg1;
    /// Specifies the value for EMC_DLL_CFG_2
    NvU32 EmcDllCfg2;
    /// Specifies the value for EMC_DLL_CFG_3
    NvU32 EmcDllCfg3;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1_0;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1_2;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2_0;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2_2;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3_0;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3_2;
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
    /// Specifies the value for EMC_PMACRO_COMMON_PAD_TX_CTRL
    NvU32 EmcPmacroCommonPadTxCtrl;
    /// Specifies the value for EMC_PMACRO_CMD_PAD_TX_CTRL
    NvU32 EmcPmacroCmdPadTxCtrl;
    /// Specifies the value for EMC_CFG_3
    NvU32 EmcCfg3;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_0_CH0
    NvU32 EmcPmacroTxPwrd0_0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_0_CH2
    NvU32 EmcPmacroTxPwrd0_2;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_1_CH0
    NvU32 EmcPmacroTxPwrd1_0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_1_CH2
    NvU32 EmcPmacroTxPwrd1_2;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_2_CH0
    NvU32 EmcPmacroTxPwrd2_0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_2_CH2
    NvU32 EmcPmacroTxPwrd2_2;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_3_CH0
    NvU32 EmcPmacroTxPwrd3_0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_3_CH2
    NvU32 EmcPmacroTxPwrd3_2;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_4_CH0
    NvU32 EmcPmacroTxPwrd4_0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_4_CH2
    NvU32 EmcPmacroTxPwrd4_2;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_5_CH0
    NvU32 EmcPmacroTxPwrd5_0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_5_CH2
    NvU32 EmcPmacroTxPwrd5_2;
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
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_2_CH0
    NvU32 EmcPmacroBrickMapping2_0;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_2_CH2
    NvU32 EmcPmacroBrickMapping2_2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_0_CH0
    NvU32 EmcPmacroTxSelClkSrc0_0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_0_CH2
    NvU32 EmcPmacroTxSelClkSrc0_2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_1_CH0
    NvU32 EmcPmacroTxSelClkSrc1_0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_1_CH2
    NvU32 EmcPmacroTxSelClkSrc1_2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_2_CH0
    NvU32 EmcPmacroTxSelClkSrc2_0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_2_CH2
    NvU32 EmcPmacroTxSelClkSrc2_2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_3_CH0
    NvU32 EmcPmacroTxSelClkSrc3_0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_3_CH2
    NvU32 EmcPmacroTxSelClkSrc3_2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_4_CH0
    NvU32 EmcPmacroTxSelClkSrc4_0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_4_CH2
    NvU32 EmcPmacroTxSelClkSrc4_2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_5_CH0
    NvU32 EmcPmacroTxSelClkSrc5_0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_5_CH2
    NvU32 EmcPmacroTxSelClkSrc5_2;
    /// Specifies the value for EMC_PMACRO_DDLL_BYPASS
    NvU32 EmcPmacroDdllBypass;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_0
    NvU32 EmcPmacroDdllPwrd0;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_1
    NvU32 EmcPmacroDdllPwrd1;
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
    /// Specifies the value for MC_REGIF_BROADCAST
    NvU32 McRegifBroadcast;
    /// Specifies the value for MC_REGIF_UNICAST0
    NvU32 McRegifUnicast0;
    /// Specifies the value for MC_REGIF_UNICAST1
    NvU32 McRegifUnicast1;
    /// Specifies the value for MC_REGIF_UNICAST2
    NvU32 McRegifUnicast2;
    /// Specifies the value for MC_REGIF_UNICAST3
    NvU32 McRegifUnicast3;

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
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_0
    NvU32 McEmemAdrCfgBankMask0;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_1
    NvU32 McEmemAdrCfgBankMask1;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_2
    NvU32 McEmemAdrCfgBankMask2;

    /// Specifies the value for MC_EMEM_CFG which holds the external memory
    /// size (in KBytes)
    NvU32 McEmemCfg;

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
    /// Specifies the value for MC_EMEM_ARB_DA_TURNS
    NvU32 McEmemArbDaTurns;
    /// Specifies the value for MC_EMEM_ARB_DA_COVERS
    NvU32 McEmemArbDaCovers;
    /// Specifies the value for MC_EMEM_ARB_MISC0
    NvU32 McEmemArbMisc0;
    /// Specifies the value for MC_EMEM_ARB_MISC1
    NvU32 McEmemArbMisc1;
    /// Specifies the value for MC_EMEM_ARB_MISC2
    NvU32 McEmemArbMisc2;
    /// Specifies the value for MC_EMEM_ARB_RING1_THROTTLE
    NvU32 McEmemArbRing1Throttle;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE
    NvU32 McEmemArbNisoThrottle;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE_MASK
    NvU32 McEmemArbNisoThrottleMask;
    /// Specifies the value for MC_EMEM_ARB_NISO_THROTTLE_MASK_1
    NvU32 McEmemArbNisoThrottleMask1;
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

    /// Specifies the value for MC_CLKEN_OVERRIDE
    NvU32 McClkenOverride;

    /// Specifies the value for MC_STAT_CONTROL
    NvU32 McStatControl;

    /// Specifies the value for MC_ECC_CFG
    NvU32 McEccCfg;

    /// Specifies the value for MC_ECC_CONTROL
    NvU32 McEccControl;

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
    /// Specifies the value for MC_SECURITY_CARVEOUT29_CFG0
    NvU32 McGeneralizedCarveout29Cfg0;

    /// Specifies enable for CA training
    NvU32 EmcCaTrainingEnable;
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
    /// Specifies flags for generating keys for encryption regions
    NvU32 MssEncryptGenKeys;
    /// Specifies flags for distributing encryption keys
    NvU32 MssEncryptDistKeys;
    /// Specifies the value for MC_GK_EXTRA_SNAP_LEVELS
    NvU32 McGkExtraSnapLevels;
    /// Specifies the value for MC_SMMU_BYPASS_CONFIG
    NvU32 McSmmuBypassConfig;
    /// Specifies if Sid programming should be bypassed at init
    NvU32 McBypassSidInit;
    /// just a place holder for special usage when there is no BCT for certain registers
    NvU32 BCT_NA;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR
    NvU32 McSidStreamidOverrideConfigPtcr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PTCR
    NvU32 McSidStreamidSecurityConfigPtcr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AFIR
    NvU32 McSidStreamidOverrideConfigAfir;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AFIR
    NvU32 McSidStreamidSecurityConfigAfir;
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
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AFIW
    NvU32 McSidStreamidOverrideConfigAfiw;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AFIW
    NvU32 McSidStreamidSecurityConfigAfiw;
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
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD
    NvU32 McSidStreamidOverrideConfigGpusrd;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD
    NvU32 McSidStreamidSecurityConfigGpusrd;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR
    NvU32 McSidStreamidOverrideConfigGpuswr;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR
    NvU32 McSidStreamidSecurityConfigGpuswr;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA
    NvU32 McSidStreamidOverrideConfigSdmmcra;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA
    NvU32 McSidStreamidSecurityConfigSdmmcra;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAA
    NvU32 McSidStreamidOverrideConfigSdmmcraa;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAA
    NvU32 McSidStreamidSecurityConfigSdmmcraa;
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
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAA
    NvU32 McSidStreamidOverrideConfigSdmmcwaa;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAA
    NvU32 McSidStreamidSecurityConfigSdmmcwaa;
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
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD2
    NvU32 McSidStreamidOverrideConfigGpusrd2;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD2
    NvU32 McSidStreamidSecurityConfigGpusrd2;
    /// Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR2
    NvU32 McSidStreamidOverrideConfigGpuswr2;
    /// Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR2
    NvU32 McSidStreamidSecurityConfigGpuswr2;
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

} NvBootSdramParams
;

    // End of generated code by warmboot_code_gen
