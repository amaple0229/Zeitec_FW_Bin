/**
 * @file VarDef.h
 *
 *  VarDef.h define the global variables
 *
 *
 * @version $Revision: 31 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __VARDEF_H__
#define __VARDEF_H__

#include "ZetDEF.h"

///-------------------------------------------------------------------------------///
///  Register Declaration
///-------------------------------------------------------------------------------///
#define EXTERN extern

EXTERN REGISTERS RegTrnumType  __attribute__((section(".RegTrumA")))   RegTrumA;   ///< _at_ 0x194004
#ifdef FEATURE_7101_DEVELOPMENT
#else
EXTERN REGISTERS RegTrnumType  __attribute__((section(".RegTrumB")))   RegTrumB;   ///< _at_ 0x194024
#endif

#ifndef FEATURE_MCU_LIB_ENABLE
EXTERN REGISTERS GpioType RegGpio  __attribute__((section(".REG_GPIO")));        ///< _at_ 0x18C040
#endif

///-------------------------------------------------------------------------------///
///  Data flash SubClass #7 => MutualDriPtr
///-------------------------------------------------------------------------------///
EXTERN GlobalVarType volatile xdata ZetVar ;
EXTERN GlobalVarType2 xdata ZetVar2 ;
#ifdef FEATURE_SELF_SCAN
EXTERN SelfScanValue volatile sSelfScan;
#endif
#ifdef FEATURE_TX_RX_IR
EXTERN IRScanValue volatile sIRScan;
#endif

#ifdef FEATURE_CUSTOMER_PROTOCOL
EXTERN CustomerVarType xdata CustomerVar; 
#endif

EXTERN GlobalVarParaType volatile xdata  __attribute__((section(".MEM_ZETVARPARA"))) ZetVarPara;

EXTERN GlobalDFVarType const code  ZetDF;
EXTERN BYTE volatile xdata *pGenBuf;
EXTERN BYTE volatile xdata *pGenBuf1;
//EXTERN BYTE volatile xdata ZetAlgoPage[DF_PAGE_SIZE];

EXTERN MutualScanDevDataType volatile xdata  *AlgorithmDataPtr;
EXTERN MutualScanDevDataType volatile xdata  *ScanLatchDataPtr;

#ifdef FEATURE_FORCE_TOUCH_SCAN
EXTERN ForceScanDevDataType volatile xdata  *ForceAlgorithmDataPtr;
EXTERN ForceScanDevDataType volatile xdata  *ForceScanLatchDataPtr;
#endif

///---------------------------------------------------------------------------///
/// Mutual ADC Data Pointer
///---------------------------------------------------------------------------///
EXTERN RegTrnumType volatile xdata *Trum[2];

///---------------------------------------------------------------------------///
/// Main Loop Control
///---------------------------------------------------------------------------///
EXTERN BYTE data LoopIdx;

///---------------------------------------------------------------------------///
/// Project Code
///---------------------------------------------------------------------------///
EXTERN BYTE const code RomProject[4];
#ifdef FEATURE_7101_DEVELOPMENT
EXTERN BYTE const code RomCS[4];
EXTERN BYTE const code CodeOptionROM[40];

EXTERN BYTE bI2CRxBuf[MAX_RX_BUF_LEN]; ///< Current Received TP command

#ifdef FEATURE_ZET7101_GPIO_ISR
EXTERN BYTE gbTestP2Pin5;
EXTERN BYTE gbTestP2Pin6;
#endif

#endif

#ifdef FEATURE_INIT_REGION_SCAN  
EXTERN int data RegionAdcData[TABLE_DEFINE_REGION_ROW][MAX_TRACE_NUM];
#endif

///---------------------------------------------------------------------------///
/// Noise Rejection Function
///---------------------------------------------------------------------------///
#ifdef FEATURE_NOISE_REJECTION
EXTERN WORD data wNoiseMax;
#endif

/// ******** MAIN CODE  VARIABLE DEFINITION  ******* ////
#endif ///< __VARDEF_H__


