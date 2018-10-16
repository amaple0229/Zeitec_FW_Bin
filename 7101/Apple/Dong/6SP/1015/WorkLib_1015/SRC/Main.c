/**
 * @file Main.c
 *
 *  main code and main loop
 *
 *
 * @version $Revision: 75 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/
#include"ZetDEF.h"

///---------------------------------------------------------------------------///
/// Registers
///---------------------------------------------------------------------------///
REGISTERS RegTrnumType RegTrumA __attribute__((section(".REG_TRUMA0")));     ///< _at_ 0x194004
#ifdef FEATURE_7101_DEVELOPMENT
#else
REGISTERS RegTrnumType RegTrumB __attribute__((section(".REG_TRUMB0")));     ///< _at_ 0x194024
#endif

#ifndef FEATURE_MCU_LIB_ENABLE
REGISTERS GpioType RegGpio             __attribute__((section(".REG_GPIO")));        ///< _at_ 0x18C040
#endif

///---------------------------------------------------------------------------///
/// ZetVar
///---------------------------------------------------------------------------///
//lobalVarType  volatile   xdata  ZetVar __attribute__((section(".MEM_ZETVAR"))) ;
GlobalVarType volatile xdata ZetVar ;
GlobalVarType2 xdata ZetVar2 ;
#ifdef FEATURE_SELF_SCAN
SelfScanValue volatile sSelfScan;
#endif
#ifdef FEATURE_TX_RX_IR
IRScanValue volatile sIRScan;
#endif
GlobalVarParaType volatile xdata ZetVarPara __attribute__((section(".MEM_ZETVARPARA"))) ;

///---------------------------------------------------------------------------///
/// Main Loop Control
///---------------------------------------------------------------------------///
BYTE xdata LoopIdx;

///---------------------------------------------------------------------------///
/// General purpose buffer
///---------------------------------------------------------------------------///
BYTE volatile xdata *pGenBuf;    ///< General Purpose Buffer Pointer , it point to the AlgorithmDataPtr->bDevAlloc
BYTE volatile xdata *pGenBuf1;    ///< General Purpose Buffer Pointer1 , it point to the cluster buffer

///---------------------------------------------------------------------------///
/// Mutual Dev Buffer
///---------------------------------------------------------------------------///
MutualScanDevDataType volatile xdata  *AlgorithmDataPtr;
MutualScanDevDataType volatile xdata  *ScanLatchDataPtr;

#ifdef FEATURE_FORCE_TOUCH_SCAN
ForceScanDevDataType volatile xdata  *ForceAlgorithmDataPtr;
ForceScanDevDataType volatile xdata  *ForceScanLatchDataPtr;
#endif

RegTrnumType volatile xdata *Trum[2];

#ifdef FEATURE_51V
BYTE  CustomRes4Cmdinit[] = {0x1f,0x01,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79};
#endif
///---------------------------------------------------------------------------///
/// NOISE_REJECTION
///---------------------------------------------------------------------------///
#ifdef FEATURE_NOISE_REJECTION
WORD xdata wNoiseMax;
#endif

/**
 * @brief CommandVarInit
 *
 *  Command-Changeable Variable initialization
 *
 * @return NULL
 *
 */
void CommandVarInit(void)
{
	SYSSetVarNormal();

	//initial value decided by data flash parameter
	#ifdef FEATURE_CUSTOMER_PROTOCOL
	ZetVar.bWorkingState=ZetDF.cFormatCtrl.scDataCtrl.bDefaultProtocolVendor;
	#else
	ZetVar.bWorkingState=1;
  #endif

  ZetVar.wTestMode = TP_TEST_NONE;
	ZetVar.wSysMode	 = 0;
	ZetVar.wSysMode2 = 0;
  ZetVar.bTranType = ZetDF.cFormatCtrl.scDataCtrl.bTranType;	
  ZetVar2.bDriveMax = ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax;
  ZetVar2.bSenseMax = ZetDF.cGlobalData.scPanelInformation.bSenseAxisMax;

	#ifdef FEATURE_NINE_SQUARE_ENABLE
	ZetVar2.wFingerSchmitWeight=	ZetDF.cFinger.scNormal.wFingerWeightSchmit;
	#endif

	#ifdef FEATURE_1T2R_ARCHITECTURE
	if((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)  == TP_TYPE_1T2R	)
	{
		ZetVar2.bDriveMaxAlg = ZetVar2.bDriveMax<<1; ///< Drive Axis Num x 2		
		ZetVar2.bSenseMaxAlg = ZetVar2.bSenseMax>>1; ///< Sense Axis Num / 2			
	}
	#ifdef FEATURE_1T2R_NO_EDGE
	else if((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE) == TP_TYPE_1T2R_NO_EDGE)
	{
		ZetVar2.bDriveMaxAlg = (ZetVar2.bDriveMax<<1) - 2; ///< Drive Axis Num x 2	 without header/last row
		ZetVar2.bSenseMaxAlg = ZetVar2.bSenseMax>>1; 		 ///< Sense Axis Num / 2		
	}
	#endif
	#ifdef FEATURE_NINE_SQUARE_ENABLE
	else if((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)  == TP_TYPE_1T2R_REVERSE) 
	{
		ZetVar2.bDriveMaxAlg = ZetVar2.bDriveMax<<1; ///< Drive Axis Num x 2		
		ZetVar2.bSenseMaxAlg = ZetVar2.bSenseMax>>1; ///< Sense Axis Num / 2	
	}
	#endif
	else
	#endif	
	{
		ZetVar2.bDriveMaxAlg = ZetVar2.bDriveMax;
		ZetVar2.bSenseMaxAlg = ZetVar2.bSenseMax;
	}	

	#ifdef FEATURE_FORCE_TOUCH_SCAN
  ZetVar2.bForceSenseMax = ZetDF.cGlobalData.scForceTouchInformation.bFSenseAxisMax;
  ZetVar2.bForceDriveMax = ZetDF.cGlobalData.scForceTouchInformation.bFDriveAxisMax;
	#endif
	ZetVar2.CompileDeviceName = ICDEVICENAME ;
}

#ifdef FEATURE_HW_FREQUENCY_HOP
void HopFrequecnyToneSetup(void)
{
	int data i;
	BYTE data bFreqHopNum;
	BYTE data bFreqHopStart;
	BYTE data bFreqHopDelta;

	bFreqHopStart = ZetDF.cSYSFreqHop.scFreqHop.bFreqMulND1;
	bFreqHopDelta = ZetDF.cSYSFreqHop.scFreqHop.bFreqMulDiff1;
	bFreqHopNum 	= ZetDF.cSYSFreqHop.scFreqHop.bFreqHopSetNum;	

	ZetVar2.bNowToneID	 = ZetDF.cSYSFreqHop.scFreqHop.bFreqHopCurrentID;
	ZetVar2.bGoodToneID = ZetVar2.bNowToneID;
	for(i=0;i<bFreqHopNum;i++)
	{
		ZetVar.HopData[i].bToneA = bFreqHopStart + bFreqHopDelta*i;
	}
}
#endif

/**
 * @brief TaskDataFormat
 *
 *  Task of Data Format
 *
 * @return NULL
 *
 */
#ifdef	FEATURE_CUSTOMER_PROTOCOL_REDUCECODE
void TaskDataFormat(void)
{
	BOOL data bI2CReadySend = TRUE;
#ifdef FEATURE_CUSTOMER_PROTOCOL   
	if(ZetVar.bWorkingState != WORKING_STATE_ZET_CMD_STATE)
	{
		return;
	}
#endif
	if(ZetVar.wSysMode & (SYS_MODE_DEEP_SLEEP|SYS_MODE_IDLE))
	{
		return;
	}

	///-----------------------------------------------------------------------------------///
	/// 1. I2C Timeout Protection
	///-----------------------------------------------------------------------------------///
	if(I2C_INT() == FALSE)
	{
		///---------------------------------------------------///
		/// Check the timeout
		///---------------------------------------------------///
		ZetVar2.bIntLowTimeout--;
		if(ZetVar2.bIntLowTimeout ==0)
		{
			/// When timeout, reset the I2C buffer to the dynamic buffer
			I2CDataFormatReset();
			ZetVar.bI2cStatus &= ~(I2C_STA_TX_START);
			/// Reset the timeout buffer
			ZetVar2.bIntLowTimeout = ZetDF.cFormatCtrl.scDataCtrl.bI2CIntLowTimeOutCnt;
		}
		else
		{
			if(ZetVar.bI2cStatus & I2C_STA_TX_START)
			{
				bI2CReadySend = FALSE;
			}
		}
	}

	/// If the test TP command in, return
	if(ZetVar.wSysMode & SYS_MODE_TP_TEST_EN)
	{
		return;
	}
	
	///-----------------------------------------------------------------------------------///
	/// 2. Check if any finger found
	///-----------------------------------------------------------------------------------///
#ifdef FEATURE_REPORT_COORDINATE_FINGERS
	if( (ZetVar.bTranType == TRAN_TYPE_DYNAMIC) || 
			(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV) ||
			(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_FRH))
	{  
		I2CDataFormatDataInCheck();

		///---------------------------------------------------------------------------------------///
		/// Set first frame low
		///---------------------------------------------------------------------------------------///
		if(ZetVar.wAlgoStatus & ALGO_INT_FIRST_FRAME_LOW)
		{
			if(ZetVar2.bDataIn == FALSE)
			{
				if((ZetVar.bTranType == TRAN_TYPE_DYNAMIC) || 
					 (ZetVar.bTranType == TRAN_TYPE_FOCAL_TYPE))
				{
					return;
				}
			}
		}
		else
		{
			ZetVar.wAlgoStatus		= (ZetVar.wAlgoStatus | ALGO_INT_FIRST_FRAME_LOW);
			if(ZetDF.cAlgoPage.scResetSpecialCtrl.bResetReportZeroCnt >0)
			{
				ZetVar2.bFingerUpRpCnt = ZetDF.cAlgoPage.scResetSpecialCtrl.bResetReportZeroCnt-1;
				ZetVar.wSysMode 			|= SYS_MODE_FINGER_REPORT;
			}
			else
			{
				return;
			}
		}
		
	#endif
		///-----------------------------------------------------------------------------------///
		/// 3. Transfer type dispatch
		///-----------------------------------------------------------------------------------///

		///---------------------------------------------------///
		/// (1)  Check Dynamic Coordinate Mode
		///---------------------------------------------------/// 

		///---------------------------------------------///
		/// [JL] :	Fix the bug that the slow Main chip 
		/// 				failed to read the fast INT LOW
		///---------------------------------------------///
		if(bI2CReadySend == FALSE)
		{
			return;
		}
		
		if(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV)
		{
		#ifdef FEATURE_REPORT_COORDINATE_FINGERS
			DFDynamicCoordinate();
		#endif
			I2CWaitTransferDone();
		
			ResetDevBuffer(ALGO_DEV_DATA);	/// Clear buffer				
		}
		else
		{
		#ifdef FEATURE_REPORT_COORDINATE_FINGERS
			DFDynamicCoordinate();			
		#endif
		} 	 	
		return;
	}  
	///---------------------------------------------------///
	/// (3) Check Mutual-Scan ADBASE Mode
	///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_MUTUAL_SCAN_ADBASE  
	else if(ZetVar.bTranType == TRAN_TYPE_MUTUAL_SCAN_BASE)
	{
		DFMutualScanBase();
	}  
#endif ///< for FEATURE_DATA_FORMAT_MUTUAL_SCAN_ADBASE 

	///---------------------------------------------------///
	/// (7)  Check Key Mutual-Scan ADBASE Mode
	///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE    
	else if(ZetVar.bTranType == TRAN_TYPE_KEY_MUTUAL_SCAN_BASE)
	{
		DFKeyMutualScanBase();
	}  
#endif ///< for FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE

	///---------------------------------------------------///
	/// (8)  Check Key Mutual-Scan Dev Mode
	///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_DEV   
	else if(ZetVar.bTranType == TRAN_TYPE_KEY_MUTUAL_SCAN_DEV)
	{
		DFKeyMutualScanDev();
	}  
#endif ///< for FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE
	///---------------------------------------------------///
	/// (9)  Check Key Data
	///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_KEY_DATA    
	else if(ZetVar.bTranType == TRAN_TYPE_KEY_DATA)
	{
		DFKeyData();
	}  
#endif ///< for FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE
	///---------------------------------------------------///
	/// (10) CheckSum Read
	///---------------------------------------------------///
#ifdef FEATURE_CHECK_SUM
	else if(ZetVar.bTranType == TRAN_TYPE_CHECKSUM_READ)
	{
		DFCheckSumData();
	}
#endif ///< for FEATURE_CHECK_SUM

	///---------------------------------------------------///
	/// 4. Force lock the firmware while host not read back the item
	///---------------------------------------------------///
	I2CWaitTransferDone();
}

#else
void TaskDataFormat(void)
{
  BOOL data bI2CReadySend = TRUE;
	#ifdef FEATURE_CUSTOMER_PROTOCOL   
	if(ZetVar.bWorkingState != WORKING_STATE_ZET_CMD_STATE)
	{
		return;
	}
	#endif
  if(ZetVar.wSysMode & (SYS_MODE_DEEP_SLEEP|SYS_MODE_IDLE))
  {
  	return;
  }

  ///-----------------------------------------------------------------------------------///
  /// 1. I2C Timeout Protection
  ///-----------------------------------------------------------------------------------///
	if(I2C_INT() == FALSE)
  {
    ///---------------------------------------------------///
    /// Check the timeout
    ///---------------------------------------------------///
    ZetVar2.bIntLowTimeout--;
    if(ZetVar2.bIntLowTimeout ==0)
    {
      /// When timeout, reset the I2C buffer to the dynamic buffer
      I2CDataFormatReset();
      ZetVar.bI2cStatus &= ~(I2C_STA_TX_START);
      /// Reset the timeout buffer
      ZetVar2.bIntLowTimeout = ZetDF.cFormatCtrl.scDataCtrl.bI2CIntLowTimeOutCnt;
    }
    else
    {
      if(ZetVar.bI2cStatus & I2C_STA_TX_START)
      {
        bI2CReadySend = FALSE;
      }
    }
  }

  /// If the test TP command in, return
	if(ZetVar.wSysMode & SYS_MODE_TP_TEST_EN)
	{
		return;
	}
	
  ///-----------------------------------------------------------------------------------///
  /// 2. Check if any finger found
  ///-----------------------------------------------------------------------------------///
	#ifdef FEATURE_REPORT_COORDINATE_FINGERS

  if(	(ZetVar.bTranType == TRAN_TYPE_DYNAMIC) || 
     	(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV) ||
     	(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_FRH))
	{  
		I2CDataFormatDataInCheck();

		///---------------------------------------------------------------------------------------///
	  /// Set first frame low
	  ///---------------------------------------------------------------------------------------///
	  if(ZetVar.wAlgoStatus & ALGO_INT_FIRST_FRAME_LOW)
		{
		  if(ZetVar2.bDataIn == FALSE)
		  {
		    if((ZetVar.bTranType == TRAN_TYPE_DYNAMIC) || 
		       (ZetVar.bTranType == TRAN_TYPE_FOCAL_TYPE))
		    {
		      return;
		    }
		  }
	  }
		else
		{
	    ZetVar.wAlgoStatus 		= (ZetVar.wAlgoStatus | ALGO_INT_FIRST_FRAME_LOW);
			if(ZetDF.cAlgoPage.scResetSpecialCtrl.bResetReportZeroCnt >0)
			{
				ZetVar2.bFingerUpRpCnt = ZetDF.cAlgoPage.scResetSpecialCtrl.bResetReportZeroCnt-1;
				ZetVar.wSysMode 			|= SYS_MODE_FINGER_REPORT;
			}
			else
			{
				return;
			}
		}
		
		#endif
		///-----------------------------------------------------------------------------------///
	  /// 3. Transfer type dispatch
	  ///-----------------------------------------------------------------------------------///

	  ///---------------------------------------------------///
	  /// (1)  Check Dynamic Coordinate Mode
	  ///---------------------------------------------------/// 

    ///---------------------------------------------///
    /// [JL] :  Fix the bug that the slow Main chip 
    ///         failed to read the fast INT LOW
    ///---------------------------------------------///
  	if(bI2CReadySend == FALSE)
	  {
	    return;
	  }

		#ifdef FEATURE_WIN8_DATAFORMAT 
		DFWindows8Data();
		#else
		if(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV)
		{
			#ifdef FEATURE_MUTUAL_DEV_SHOW_INIT_DEV
	  	CopyInitDevToMutDev();
			#endif
			#ifdef FEATURE_REPORT_COORDINATE_FINGERS
			DFDynamicCoordinate();
			#endif
			I2CWaitTransferDone();
    	ResetDevBuffer(ALGO_DEV_DATA);	/// Clear buffer  			
		}
		else
		{
			#ifdef FEATURE_REPORT_COORDINATE_FINGERS
			DFDynamicCoordinate();			
			#endif
		}    
		#endif		
		
		#ifdef FEATURE_DYNAMIC_FREQ_TRAN_MODE_ON
		if(ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_FRH)
		{
		  FHDMDFHopDataFormatSend();
			I2CWaitTransferDone();
		}
		#endif ///< FEATURE_FREQ_HOP_TRAN_MODE_ON

    return;
  }  
  ///---------------------------------------------------///
  /// (3) Check Mutual-Scan ADBASE Mode
  ///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_MUTUAL_SCAN_ADBASE  
  else if(ZetVar.bTranType == TRAN_TYPE_MUTUAL_SCAN_BASE)
  {
    DFMutualScanBase();
  }  
#endif ///< for FEATURE_DATA_FORMAT_MUTUAL_SCAN_ADBASE 

  ///---------------------------------------------------///
  /// (4) Check Mutual-Scan Dev Mode
  ///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_MUTUAL_SCAN_DEV  
  else if((ZetVar.bTranType == TRAN_TYPE_MUTUAL_SCAN_DEV) || (ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV))
  {
    DFMutualScanDev();
  }
#endif  ///< for FEATURE_DATA_FORMAT_MUTUAL_SCAN_DEV  

  ///---------------------------------------------------///
  ///(5)  Check Init-Scan ADBASE Mode
  ///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_INIT_SCAN_ADBASE    
  else if(ZetVar.bTranType == TRAN_TYPE_INIT_SCAN_BASE)
  {
		DFInitScanBase();
  }  
#endif  ///< for FEATURE_DATA_FORMAT_INIT_SCAN_ADBASE    

  ///---------------------------------------------------///
  /// (6) Check Init-Scan Dev Mode
  ///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_INIT_SCAN_DEV    
  else if(ZetVar.bTranType == TRAN_TYPE_INIT_SCAN_DEV)
  {
  	DFInitScanDev();
  }  
#endif ///< for FEATURE_DATA_FORMAT_INIT_SCAN_DEV

  ///---------------------------------------------------///
  /// (7)  Check Key Mutual-Scan ADBASE Mode
  ///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE    
  else if(ZetVar.bTranType == TRAN_TYPE_KEY_MUTUAL_SCAN_BASE)
  {
    DFKeyMutualScanBase();
  }  
#endif ///< for FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE

  ///---------------------------------------------------///
  /// (8)  Check Key Mutual-Scan Dev Mode
  ///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_DEV   
  else if(ZetVar.bTranType == TRAN_TYPE_KEY_MUTUAL_SCAN_DEV)
  {
    DFKeyMutualScanDev();
  }  
#endif ///< for FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE
	///---------------------------------------------------///
	/// (9)  Check Key Data
	///---------------------------------------------------/// 
#ifdef FEATURE_DATA_FORMAT_KEY_DATA    
	else if(ZetVar.bTranType == TRAN_TYPE_KEY_DATA)
	{
		DFKeyData();
	}  
#endif ///< for FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE
  ///---------------------------------------------------///
  /// (10) CheckSum Read
  ///---------------------------------------------------///
#ifdef FEATURE_CHECK_SUM
	else if(ZetVar.bTranType == TRAN_TYPE_CHECKSUM_READ)
  {
    DFCheckSumData();
  }
#endif ///< for FEATURE_CHECK_SUM
	///---------------------------------------------------///
	/// (11) Force AD
	///---------------------------------------------------///
#ifdef FEATURE_FORCE_TOUCH_SCAN 
	else if(ZetVar.bTranType == TRAN_TYPE_FORCE_SCAN_BASE)
	{
    DFForceScanBase();
	}
#endif
	///---------------------------------------------------///
	/// (11) Force Dev
	///---------------------------------------------------///
#ifdef FEATURE_FORCE_TOUCH_SCAN  
	else if(ZetVar.bTranType == TRAN_TYPE_FORCE_SCAN_DEV)
	{
		DFForceScanDev();
	}
#endif

  ///---------------------------------------------------///
  /// 4. Force lock the firmware while host not read back the item
  ///---------------------------------------------------///
  I2CWaitTransferDone();
}

#endif

/**
 * @brief MainInit
 *
 *  Main function initialization
 *
 * @return NULL
 *
 */
void VarInit(void)
{
  BYTE	data i;

  ///-------------------------------------------------------------------------------///
  /// 1. Set the allocation buffer
  ///-------------------------------------------------------------------------------///  
  pGenBuf  = (BYTE xdata *)&ZetVar.MutualScanDevData[0].sDevAlloc[0];	
  pGenBuf1 = (BYTE xdata *)&ZetVar.bGenBuf1[0];		

	for(i=0;i<TRUM_OUT_NUM;i++)
	{
		Trum[0]	=	(RegTrnumType xdata *)&RegTrumA;
			#ifdef FEATURE_7101_DEVELOPMENT
			#else
		Trum[1]	=	(RegTrnumType xdata *)&RegTrumB;
			#endif
	}
  ///-------------------------------------------------------------------------------///
	/// 2. System Mode setup Init
  ///-------------------------------------------------------------------------------///
  ZetVar.wAlgoStatus = 0;
  ZetVar.wAlgoStatus2= 0;
	ZetVar.bLcdOn = 1; // LCD on:1 LCD off:0
	ZetVar.bCalibrationCtrl = 0;
  /// Check the Key support
  #ifdef FEATURE_REAL_KEY
  if(ZetDF.cKeyCtrl.scKeyDriveTrace.bKeySupport == TRUE)
  {
    ZetVar.wSysMode |= SYS_MODE_KEY_SUPPORTED;    
  }
	#endif

	ZetVar.wAlgoStatus 	|= ALG_INIT_ABNORMAL_MUT_ALL_SCAN;		
 	
	#ifdef FEATURE_SNE_AXIS_INIT_SCAN
	if(ZetDF.cGlobalData.scPanelInformation.bEnableInitScan & SENSE_AXIS_INIT_SCAN)
	{
		ZetVar.wSysMode |=  SYS_MODE_INIT_SCAN_EN_SEN_AXIS; 
	}
	#endif

	#ifdef FEATURE_INIT_REGION_SCAN
	if(ZetDF.cGlobalData.scPanelInformation.bEnableInitScan & DRIVE_AXIS_INIT_PN_SCAN)
	{
		ZetVar.wSysMode2 |=  SYS_MODE2_INIT_PN_SCAN_EN; 
	}
	#endif 

	if(ZetDF.cAlgoPage.scAlgoData.bAlgFuncCtrl & FUN_RAW_DATA_VIEW)
	{
		ZetVar.wSysMode2 |= SYS_MODE2_RAW_MUTUAL_DEV; 
	}

	///-----------------------------------------------------------------------------------------///
	/// 4. PAD Register Initial
	///-----------------------------------------------------------------------------------------///
	PAD_SEL = (volatile unsigned char *)REG32_TR_SEL_OUT_OUT0;

  ///-------------------------------------------------------------------------------///
	/// 6.Load the Drive and Sense Max 
  ///-------------------------------------------------------------------------------/// 
	#ifdef FEATURE_MERGE_DEV_ARRAY
	if(ZetDF.cAlgoPage.scAlgoData.bAlgFuncCtrl & FUN_MERGE_DEV)
	{
		ZetVar2.bDriveMaxAlg = ZetVar2.bDriveMaxAlg + 1;
	}
	#endif

  #ifdef 	FEATURE_DIFFERENT_LENGTH_PITCH
  {
      BYTE const code *bptr;
			if(ZetDF.cCoordRemap.scCoordRemapSetup.bCoordRemapEnable!=0)
			{
					bptr=&ZetDF.cCoordRemap.scCoordRemapSetup.bCoordRemapDriveAxis00;
					for(i=0;i<31;i++)
					{
				     ZetVar2.bCoordRemapDriveAxis[i]=bptr[i];
					}
					bptr=&ZetDF.cCoordRemap.scCoordRemapSetup.bCoordRemapSenseAxis00;
					for(i=0;i<31;i++)
					{
				     ZetVar2.bCoordRemapSenseAxis[i]=bptr[i];
					}
			}
			else
			{
          for(i=0;i<31;i++)
					{
				     ZetVar2.bCoordRemapDriveAxis[i]=i;
					}

					for(i=0;i<31;i++)
					{
				     ZetVar2.bCoordRemapSenseAxis[i]=i;
					}
			}
  }
  #endif

	
	#ifdef FEATURE_HTYPE_DEV_RE_MAPPING_REV_153 
	if((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)  == TP_TYPE_H)
	{
		if(ZetDF.cFormatCtrl.scDataCtrl.bTPTypeSub & TP_TYPE_H_AXIS)
		{	
			ZetVar2.wDriveAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wDriAxisRes<<4))/(ZetVar2.bDriveMaxAlg);
			ZetVar2.wSenseAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wSenAxisRes<<4))/(ZetVar2.bSenseMaxAlg-1);
		}
		else
		{
			ZetVar2.wDriveAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wDriAxisRes<<4))/(ZetVar2.bDriveMaxAlg-1);
			ZetVar2.wSenseAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wSenAxisRes<<4))/(ZetVar2.bSenseMaxAlg);				
		}
	}
	else
	#endif
	{
	  #ifdef 	FEATURE_DIFFERENT_LENGTH_PITCH
    if(ZetDF.cCoordRemap.scCoordRemapSetup.bCoordRemapEnable!=0)
		{
			ZetVar2.wDriveAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wDriAxisRes<<4))/(ZetVar2.bCoordRemapDriveAxis[ZetVar2.bDriveMaxAlg-1]+1);
			ZetVar2.wSenseAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wSenAxisRes<<4))/(ZetVar2.bCoordRemapSenseAxis[ZetVar2.bSenseMaxAlg-1]+1);
    }
		else		
		#endif
		{
		  ZetVar2.wDriveAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wDriAxisRes<<4))/(ZetVar2.bDriveMaxAlg);
		  ZetVar2.wSenseAxisInterpolation16 = ((WORD)(ZetDF.cGlobalData.scPanelInformation.wSenAxisRes<<4))/ZetVar2.bSenseMaxAlg;
	  }
	}
	//ZetDF.cGlobalData.scPanelInformation.wDriAxisInterp
	//ZetDF.cGlobalData.scPanelInformation.wSenAxisInterp

  ///-----------------------------------------------------------------------------------------///
  /// 9. Mutual Scan DEV Buffer pointer init
  ///-----------------------------------------------------------------------------------------///
  MSMutualScanDevDataInit();  
	#ifdef FEATURE_FORCE_TOUCH_SCAN
	MSForceScanDevDataInit();
	#endif
  ///-----------------------------------------------------------------------------------------///
  /// 10. Virtual Key Init
  ///-----------------------------------------------------------------------------------------///
#ifdef FEATURE_VIRTUAL_KEY  
  ZetVar.pVirtualKey[0] = (VirtualKeyType const code *)&ZetDF.cKeyCtrl.scVirtualKey.wKey0X;
  ZetVar.pVirtualKey[1] = (VirtualKeyType const code *)&ZetDF.cKeyCtrl.scVirtualKey.wKey1X;
  ZetVar.pVirtualKey[2] = (VirtualKeyType const code *)&ZetDF.cKeyCtrl.scVirtualKey.wKey2X;
  ZetVar.pVirtualKey[3] = (VirtualKeyType const code *)&ZetDF.cKeyCtrl.scVirtualKey.wKey3X;
  ZetVar.bVirtualKeyValidByte = 0x00;
  ZetVar.bVirtualKeyEnable =  ZetDF.cKeyCtrl.scVirtualKey.bVKeyEnable;
#endif ///< for FEATURE_VIRTUAL_KEY  

  ///-------------------------------------------------------------------------------///
	/// 11.  ADBASE data pointer init
  ///-------------------------------------------------------------------------------///
  #ifdef FEATURE_SWITCH_DRIVE_SENSE
	ADBasePtrInit();
	#else ///< for FEATURE_SWITCH_DRIVE_SENSE
  //j = ZetVar2.bSenseMaxAlg;
	for(i=0;i<ZetVar2.bDriveMaxAlg;i++)
	{	
		//ZetVar.AdBaseRound[i] = (WORD *)(&ZetVar.MutualScanAdBase.AdBaseAlloc[i*j]);
		ZetVar.AdBaseRound[i] = (WORD *)(&ZetVar.MutualScanAdBase.AdBaseAlloc[i*ZetVar2.bSenseMaxAlg]);
	}
	#endif ///< for FEATURE_SWITCH_DRIVE_SENSE

  ///-------------------------------------------------------------------------------///
  /// 12. Key Control
  ///-------------------------------------------------------------------------------///
  /// Make all keys invalid
  #if defined(FEATURE_REAL_KEY) ||defined(FEATURE_VIRTUAL_KEY)
  ZetVar.KeyCtrl.bKeyValidByte = 0x00;
	ZetVar.bKeyReleaseCnt = ZetDF.cKeyCtrl.scAlgoKeyTh.bKeyReleaseCnt;
	ZetVar.bKeyLastStatus = 0;
	ZetVar.bKeyLockStatus = 0;
	ZetVar.bKeyLockRelease = TRUE;
	#endif ///< for FEATURE_VIRTUAL_KEY 
  ///-------------------------------------------------------------------------------///
  /// 13. Algorithm  : Mutual-Scan / Init-Scan Cut DEV Init ==> For normal and charger mode
  ///-------------------------------------------------------------------------------///
  //ZetVar.wCutDevRegionScan = (WORD)((WORD)ZetDF.cDevCut.scDevCut.bMutDevCut  <<( ZetDF.cMutualAnalog.scMutualNormal.bRegDGDCTRL4 & (0x0F)));

	///-------------------------------------------------------------------------------///
  /// 14. Frequency Trim Hop
  ///-------------------------------------------------------------------------------///  
  #ifdef FEATURE_TRIM_FREQ_HOP
	MSHopDataInitialization();
	#endif ///< for FEATURE_FREQ_HOP_ALG
	
  ///-----------------------------------------------------------------------------------------///
  /// 15. Algorithm  : Palm State Init
  ///-----------------------------------------------------------------------------------------///
  #ifdef FEATURE_PALM_DETECT
  ZetVar.bPalmState = PALM_STATE_LEVEL_0;
	#endif	
  ///-----------------------------------------------------------------------------------------///
  /// 16. Algorithm Status Init
  ///-----------------------------------------------------------------------------------------///
  #ifdef FEATURE_ALLPOINT_HEAVY_PRESS_CUT_AVG_ALG
	ZetVar.wAllPointDevAvg = 0;
	#endif
  ZetVar2.bNegFingerMutRekDebounce = 0;
  ZetVar2.NFingerCnts      = 0;
  ZetVar2.PFingerCnts      = 0;
	#ifdef FEATURE_FINGER_UP_DEBOUNCE_ALG
  ZetVar2.bFingerReportCounts = 0;	
	#endif
	ZetVar2.bFingerCountsFound	=	0;
  ZetVar2.bFingerCountsFoundLast =	0;
	
	for(i=0;i<REPORT_BUFFER_NUM;i++)
	{
		ZetVar2.dynamicData[i].wValid	= 0;
	}

	#ifdef FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS	
	ZetVar2.bHeavyPressDetectInOutCnt = 0;
	#endif ///< for FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS	 

	///-----------------------------------------------------------------------------------------///
  /// 17. Data Format Init
  ///-----------------------------------------------------------------------------------------///
  ZetVar2.pCurReportIdx = 0;
  ZetVar.pCurReportData = (BYTE xdata *)&ZetVar2.dynamicData[ZetVar2.pCurReportIdx]; 
  ZetVar2.bDataIn  = FALSE;
  ZetVar2.bIntLowTimeout = ZetDF.cFormatCtrl.scDataCtrl.bI2CIntLowTimeOutCnt;

  /// Transfer Type

	#ifdef FEATURE_RING_BUFFER
	ZetVar.bAlgRingBufCnt = 0;
	ZetVar.bI2CRingBufCnt = 0;
	ZetVar.bRingBufFlag		=	0;
	#endif ///< for FEATURE_RING_BUFFER

  ///-----------------------------------------------------------------------------------------///
  /// 19. For Init-Scan Dev mode init
  ///-----------------------------------------------------------------------------------------///
  for(i = 0 ; i < (MAX_TRACE_NUM + 2) ; i++)
  {
    ZetVar.InitScanSenseDevData.sDev[i] = 0x00; 
		ZetVar.InitScanDriveDevData.sDev[i] = 0x00; 
  }

  ///-------------------------------------------------------------------------------///
  /// 22. Debounce Count
  ///-------------------------------------------------------------------------------///
  /// Finger down debounce count
  SYSSetDebounceNormal();

  /// IIR add one point
  /// Finger up debounce count
  #ifdef FEATURE_IIR_FINGER_UP
  if(ZetDF.cFinger.scNormal.bFingerUpEnable & FINGER_UP_MV_BIT1)
  {
    ZetVar2.bFingerUpEn = 3;
    ZetVar2.bFingerUpOneDebounceCnt = 0;
    ZetVar2.bFingerUpMultiDebounceCnt = 0;
  }
  else
  {
    ZetVar2.bFingerUpEn = ZetDF.cFinger.scNormal.bFingerUpEnable;
    ZetVar2.bFingerUpOneDebounceCnt = ZetDF.cFinger.scNormal.bDebounceUpCnt;
    ZetVar2.bFingerUpMultiDebounceCnt = ZetDF.cFinger.scNormal.bDebounceUpCntMulti;
  }
  #else
  ZetVar2.bFingerUpMultiDebounceCnt = ZetDF.cFinger.scNormal.bDebounceUpCntMulti;
  #endif 

  ///-------------------------------------------------------------------------------///
  /// 25. Re-calibration detect count
  ///-------------------------------------------------------------------------------/// 
  ZetVar.bReCalDetNum = ZetDF.cAlgoPage.scAlgoData.bReCalDetCnt;
	ZetVar.bReCalDetCnt	= ZetDF.cAlgoPage.scAlgoData.bReCalDetCnt;		  	

  ///-------------------------------------------------------------------------------///
  /// 26. No finger round count
  ///-------------------------------------------------------------------------------///    
  #ifdef FEATURE_POWER_SAVE_MODE  
  ZetVar.bNoFingerRoundCount = 0;
 	#endif

	///----------------------------------------------------------------------------------------------------------------------------///
  /// 28. ADC CTRL0 Value Change
  ///----------------------------------------------------------------------------------------------------------------------------///  

	///----------------------------------------------------------------------------------------------------------------------------///
  /// 29. PHSEL for ZETA FPC open / short test
  ///----------------------------------------------------------------------------------------------------------------------------///  

	///----------------------------------------------------------------------------------------------------------------------------///
  /// 30. Sense Val Change 
  ///----------------------------------------------------------------------------------------------------------------------------///  	

	///-----------------------------------------------------------------------------------------///
	/// 31. Re-Calibration / Base-Tracking Total enable/disable Control
  ///-----------------------------------------------------------------------------------------///	
  if(ZetDF.cAlgoPage.scAlgoData.bEnableReCalibration == TRUE)	
	{
		ZetVar.bCalibrationCtrl |= BASE_TRACK_RECALIBRAION_TOTAL_CTRL;
	}
	#ifdef FEATURE_SYNC_REK
  else //0x02, not 1
  {
     ZetVar.bCalibrationCtrl |= (BASE_SYNC_REK_MODE|BASE_EN_TRACKINGBASE);
  }
  #endif
  ///----------------------------------------------------------------------------------------------------------------------------///
  /// 32.Initial Coordinate Data
  ///----------------------------------------------------------------------------------------------------------------------------///  	
	for (i=0; i<FINGER_MAX;i++)
  {
    ///-----------------------------------------------------///
    ///  Search Finger Init
    ///-----------------------------------------------------///
  #ifdef FEATURE_NINE_SQUARE_ENABLE
		#ifdef FEATURE_SEARCH_FINGER_TOUCH_FLAG
		ZetVar.SearchFingerRecord[i].bTouch 								= FALSE;
		#endif
				
		#ifdef FEATURE_FINGER_SCHMIT_TRIGGER
		ZetVar.SearchFingerRecord[i].wFingerFoundWeight 		=	0;
		#endif ///< for FEATURE_FINGER_SCHMIT_TRIGGER 		
	#endif ///< for FEATURE_NINE_SQUARE_ENABLE

		if(i<FINGER_MAX_REPORT)
		{
			ZetVar2.FingerCoordinateRecord[i].wFingerStatus = 0;
			ZetVar2.FingerCoordinateRecord[i].bFingerDown 				= FINGER_UP;
			ZetVar2.FingerCoordinateRecord[i].bIIRWeight 				= 0;
			ZetVar2.FingerCoordinateRecord[i].bFingerDebounce = 0;
			ZetVar2.FingerCoordinateRecord[i].bFingerUpState = 0;
			#ifdef FEATURE_MERGE_CLOSE_NINESQU_FINGER
			ZetVar2.TrackCoordinateRecord[i].bStatus 				= 0;
			ZetVar2.TrackCoordinateRecord[i].bMergeDebounce	= 0;
      #ifdef FEATURE_NEW_MERGE_CLOSE_CLUSTER_FINGER	
			ZetVar2.TrackCoordinateRecord[i].bExistDebounce	= 0;	
      #endif
			#endif
			#ifdef FEATURE_MERGE_CLOSE_CLUSTER_FINGER
			ZetVar2.TrackCoordinateRecord[i].bStatus 				= 0;	
			ZetVar2.TrackCoordinateRecord[i].bRootClusterID  = 0;	
			#endif	
			#ifdef FEATURE_NEW_MERGE_CLOSE_CLUSTER_FINGER
			ZetVar2.TrackCoordinateRecord[i].bMergeDebounce	= 0;
			ZetVar2.TrackCoordinateRecord[i].bFakeDivideDebounce = 0;
			//ZetVar2.TrackCoordinateRecord[i].bPeakCnts				= 0;
			#endif			
		}
	}	

	for( i = 0 ; i < (MAX_TRACE_NUM+2); i++)
  {
    ZetVar.MutualScanDevData[0].bFound[i]    = MUTUAL_SCAN_NONE;
    ZetVar.MutualScanDevData[0].bSenFound[i] = MUTUAL_SCAN_NONE;
    ZetVar.MutualScanDevData[1].bFound[i]    = MUTUAL_SCAN_NONE;
    ZetVar.MutualScanDevData[1].bSenFound[i] = MUTUAL_SCAN_NONE;    
  } 

	///----------------------------------------------------------------------------------------------------------------------------///
	/// 	 27. Base Track Period
	///----------------------------------------------------------------------------------------------------------------------------/// 
	#ifdef FEATURE_POWER_BASE_TRACK
	ZetVar2.bBaseTrackPeriod = ZetDF.cAlgoPage.scAlgoData.bBaseTrackPeriodWhenRst;
	ZetVar2.wFrameCntUnStableState = 0;
	#else ///< for FEATURE_POWER_BASE_TRACK
	ZetVar2.bBaseTrackPeriod = ZetDF.cAlgoPage.scAlgoData.bBaseTrackPeriod;
	#endif ///< for FEATURE_POWER_BASE_TRACK

	#ifdef FEATURE_REK_POWER_BASE_TRACK
	ZetVar2.wFrameCntReKUnStableState = 0xFFFF;
	#endif ///< for FEATURE_REK_POWER_BASE_TRACK

  ///-------------------------------------------///
	/// Dev data init
  ///-------------------------------------------///	
  MEMSET((void *)&ZetVar.MutualScanDevData[0].sDevAlloc[0], 0x00, (MAX_DEV_ARRAY_SIZE)*sizeof(short));
  MEMSET((void *)&ZetVar.MutualScanDevData[1].sDevAlloc[0], 0x00, (MAX_DEV_ARRAY_SIZE)*sizeof(short));
	#ifdef FEATURE_PROCESS_COORDINATE_WITH_RAW_DATA
  MEMSET((void *)&ZetVar.MutualScanRawDevData[0], 0x00, (MAX_BASE_ARRAY_SIZE)*sizeof(short));
	#endif ///< for FEATURE_PROCESS_COORDINATE_WITH_RAW_DATA

	#ifdef FEATURE_FORCE_TOUCH_SCAN	
  MEMSET((void *)&ZetVar.ForceScanDevData[0].sDevAlloc[0], 0x00, (FORCE_BASE_ARRAY_SIZE)*sizeof(short));
  MEMSET((void *)&ZetVar.ForceScanDevData[1].sDevAlloc[0], 0x00, (FORCE_BASE_ARRAY_SIZE)*sizeof(short));
	#endif

  ///-------------------------------------------///
	/// Cluster Search initial
  ///-------------------------------------------///	
	#ifdef FEATURE_CLUSTER_ENABLE
	for(i = 0 ; i < (ZetVar2.bDriveMaxAlg); i++)
	{
		ZetVar.ClusterSchmitSwitch[i] = 0;
		#ifdef FEATURE_MUT_PARTIAL_REK
		ZetVar.BaseUpdateControl[i] = 0;
		#endif
		#ifdef FEATURE_MUT_PARTIAL_CROSS_SET
		ZetVar.BaseUpdateCross[i] = 0;
		#endif
	}

	#endif ///< for FEATURE_CLUSTER_ENABLE

	///-------------------------------------------///
	/// Merge Close Finger initialization 
	///-------------------------------------------/// 

	#ifdef FEATURE_MERGE_CLOSE_NINESQU_FINGER
	for(i = 0 ; i < FINGER_10_COMBINATION_NUMBER; i++)
	{
		ZetVar2.MergeIDStatus[i].bStstus = MERGE_STATE_INITIALIZATION;
		ZetVar2.MergeIDStatus[i].bDebounceCnt = MERGE_DEBOUNCE_INIT_VALUE;
	}
	ALMergeParamInitialization();
	#endif ///< for FEATURE_MERGE_CLOSE_NINESQU_FINGER

	#ifdef FEATURE_ADV_NINE_SQAURE
	ZetVar.NineSquClusterIDArray[0] = (BYTE *)(&ZetVar.NineSquClusterIDData[0]);
	ZetVar.NineSquClusterIDArray[1] = (BYTE *)(&ZetVar.NineSquClusterIDData[(MAX_DRIVE_TRACE_NUM +2)*1]);		
	#endif ///< for FEATURE_ADV_NINE_SQAURE

	///-------------------------------------------///
	/// Border Coordinate Compensation
	///-------------------------------------------///
	#ifdef FEATURE_BORDER_COOR
	ZetVar.bSenseAxisInterpolation = (BYTE)(0x00FF & (ZetVar2.wSenseAxisInterpolation16 >>4));	///< divided by 16
	ZetVar.bDriveAxisInterpolation = (BYTE)(0x00FF & (ZetVar2.wDriveAxisInterpolation16 >>4));	///< divided by 16	
	ZetVar.wSenseAxisBorderCompNearNum = ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderSenNearCoorPercent	, ZetVar.bSenseAxisInterpolation);
	ZetVar.wSenseAxisBorderCompFarNum  = ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderSenFarCoorPercent		,	ZetVar.bSenseAxisInterpolation);
	ZetVar.wDriveAxisBorderCompNearNum = ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderDriNearCoorPercent	, ZetVar.bDriveAxisInterpolation);
	ZetVar.wDriveAxisBorderCompFarNum  = ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderDriFarCoorPercent		,	ZetVar.bDriveAxisInterpolation);
	
	ZetVar.wSenseAxisBorderCompNearFarDen = ZetVar.bSenseAxisInterpolation >> 1;
	ZetVar.wDriveAxisBorderCompNearFarDen = ZetVar.bDriveAxisInterpolation >> 1;
	#endif ///< for FEATURE_BORDER_COOR
	ZetVar.wSenseAxisBorderHitCompNearNum = ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderHitSenNearCoorPercent	, ZetVar.bSenseAxisInterpolation);
	ZetVar.wSenseAxisBorderHitCompFarNum	= ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderHitSenFarCoorPercent 	, ZetVar.bSenseAxisInterpolation);
	ZetVar.wDriveAxisBorderHitCompNearNum = ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderHitDriNearCoorPercent	, ZetVar.bDriveAxisInterpolation);
	ZetVar.wDriveAxisBorderHitCompFarNum	= ALBorderCompStartEnd(ZetDF.cBorderComp.scAlgoBorderCoorComp.bBorderHitDriFarCoorPercent 	, ZetVar.bDriveAxisInterpolation);

	///-------------------------------------------///
	/// Initial Dev is Neg After ReK
	///-------------------------------------------///
	#ifdef FEATURE_MUT_ABNORMAL_REK
	ZetVar2.bNegFingerFoundFrameCnt = 0;
	ZetVar2.wAfterReKNumMax 					=(ZetDF.cAlgoPage.scAlgoData.bReCalDetAccCnt * ZetDF.cAlgoPage.scAlgoData.bReCalDetCnt )<<2;
	#endif
	
	///-------------------------------------------///
	/// Interpolatin Average
	///-------------------------------------------///
	#ifdef FEATURE_GG_FILTER_FIX
	ZetVar.bInterpolationAvg = (ZetVar2.wDriveAxisInterpolation16/16 + ZetVar2.wSenseAxisInterpolation16/16)/2;
	#endif

	///-------------------------------------------///
	///  ReK reference
	///-------------------------------------------///
	ZetVar2.wContinueReKStopCnt = 0;
	ZetVar2.bReKWithFingerCnt = 0;

	#ifdef FEATURE_ADV_NINE_SQAURE
	ZetVar2.bFingerVellySlop = ZetVar2.bFingerSlop;
	#endif

	///-------------------------------------------///
	///  1T2R Dev Compensation Algorithm
	///-------------------------------------------///
	#ifdef FEATURE_1T2R_ARCHITECTURE_FARNEAR_REDUCTION
	Alg1T2RCompPtrInit();
	for(i=0;i<=ZetVar2.bSenseMaxAlg+1;i++)
  {
    ZetVar.sSumArrEven[i] = 0;
    ZetVar.sSumArrOdd[i]  = 0;
  }
	#endif

  #ifdef FEATURE_1T2R_NEAR_FAR_COMPEN_1ST_SHOW_AVG
  for(i=1;i<=ZetVar2.bSenseMaxAlg;i++)
  {
    ZetVar.wSumArrEven1st[i] = 0;
    ZetVar.wSumArrOdd1st[i]  = 0;
  }
  #endif

	///-------------------------------------------///
	///  Frequency Hop 
	///-------------------------------------------///
	#ifdef FEATURE_HW_FREQUENCY_HOP
	HopFrequecnyToneSetup();	
	#endif
	SYSSetFreqToneReg(ZetVar2.bNowToneID);

	ZetVar2.bDynamicFreqHopScanCnt = 0;
	ZetVar2.bDynamicFreqStatus			= 0;
	ZetVar2.bDynamicFreqHopRoundCnt= 0;
	ZetVar2.bDynamicCurrentIDBadCnt= 0;
  ZetVar2.bDynamicHopDataOutSel  = 0;

#ifdef FEATURE_7101_DEVELOPMENT
  //trivial code, need to make sure ZetVarPara.bCmdPara[] re-set right value
	ZetVarPara.bCmdPara[SPARA_TONEA]=CodeOptionROM[0]; //don't remove, or compiler won't include it
	ZetVarPara.bCmdPara[SPARA_TONEB]=RomCS[0]; //don't remove, or compiler won't include it
#endif

	///-------------------------------------------///
	///  Command parameter SRAM Initial
	///-------------------------------------------///
	SYSScriptRegInitial();

	#ifdef FEATURE_NOISE_REJECTION
	if( ((RG_TX_DUAL_TONE & ZetVarPara.bCmdPara[SPARA_PNCHIPNUM])==0x00) && (ZetDF.cMutualAnalog.scMutualNormal.bNoiseRejectEn != 0x00))
	{
		ZetVar2.bNoiseRejEn = TRUE;
	}
	else
	{
		ZetVar2.bNoiseRejEn = FALSE;
	}
	#endif

	#ifdef FEATURE_WATER_INIT_SCAN
	ZetVar.bWaterStatus = 0;
	ZetVar.bWaterDetectCnt = 0;
	ZetVar.bWaterNoInitCnt = 0;
	ZetVar.wWaterBaseTrackDebounce = 0;
	#endif

	#ifdef FEATURE_CLUSTER_ENABLE
	ALFingerSearchParam(NORMAL_MODE_PARA);
	#endif

	#ifdef FEATURE_SENSE_DEV_PROCESS_MA_FILTER
	#ifdef FEATURE_SENSE_DEV_PROCESS_MA_FILTER_FINGER_PROTECT_INTEN_CTRL
	ZetVar.bSenDevProcIntenCtrlAvgLcl = 8-ZetDF.cAlgoPage.scAlgoData.bSenDevProcIntenCtrl;
	ZetVar.bSenDevProcIntenCtrlAvgTtl = ZetDF.cAlgoPage.scAlgoData.bSenDevProcIntenCtrl;
	#endif
	#endif
	
}
/**
 * @brief MainInit
 *
 *  Main function initialization
 *
 * @return NULL
 *
 */
void MainInit()
{
  ///-------------------------------------------------------------------------------///
  /// Set the IO Voltage 1.8V or 3.3V
  ///-------------------------------------------------------------------------------///
  WRITE_REG16(REG32_P3_SEL18, ZetDF.cMutualAnalog.scMutualNormal.wRegSel18);

  ///-------------------------------------------------------------------------------///
  /// 1. Enable GPIO
  ///-------------------------------------------------------------------------------///
  
#ifdef	FEATURE_7101_DEVELOPMENT
	//WRITE_REG(SYSREG, 0x00);
  
	if((ZetDF.cMiscRegister.scInterface.bRegSPIMode&0x40)==0x40)
	{
	  SET_BIT(SYSREG, (1<<6));
	}
	else
	{		
	  CLR_BIT(SYSREG, (1<<6));
	}
	
	if((ZetDF.cMiscRegister.scInterface.bRegSPIMode&0x80)==0x00)
	{
	  //If SPI is not in use, let SPI Pin pull-high to avoid trigger isr wrongly
		WRITE_REG(REG32_PORT1_PH,0x0f);	  
	}
	else
	{
    //If SPI is in use, let SPI SO Pin pull-high to speed-up SPI rate
		//WRITE_REG(REG32_PORT1_PH,0x04);
	}

	 SET_BIT(REG32_CK_CTR3,CK_CO_FSEL);

	 CLR_BIT(SYSREG, IO_CTRL_BIT);
	 WRITE_REG(REG32_STR_PN_CTRL , 0x00);
	 CLR_BIT(REG32_WDCON, WDT_RESET_TYPE);
	 WRITE_REG32(REG32_IODIRP2,0x00000000);	
#endif
	
	#ifdef FEATURE_GIO_ENABLE
  SYSGpioInit();
	#endif ///< for FEATURE_GIO_ENABLE
	#ifdef FEATURE_ZET7101_ANA2GPIO
  ANA2GpioInit();  
  ANA2GpioDataOut(0xA5);	
	#endif
	///-------------------------------------------------------------------------------///  
  /// 2. Command Changeable Variable init 
  ///-------------------------------------------------------------------------------///    
  CommandVarInit();
	
  ///-------------------------------------------------------------------------------///
	/// 3. I2C Init
	///
	///   [JL] : Make the TP able to send the B2 command back, just after system reset
  ///          Do not modify the  session!!!
  ///-------------------------------------------------------------------------------///   
  I2CInit(); 
  TPCmdB2Set();
  #ifdef FEATURE_SPI_TRANSFER
	SPIInit();
	#endif
	
	
	#ifdef  FEATURE_CUSTOMER_PROTOCOL
	CustomerInit();
	#endif  
  ///-------------------------------------------------------------------------------///
  /// 4. Enable All Interupts
  ///-------------------------------------------------------------------------------///
  IntInit();
  InterruptEnable();
	
  #ifdef FEATURE_ZET7101_GPIO_ISR
  GpioISRInit();
  #endif

  ///-------------------------------------------------------------------------------///
  /// 5. Trim 512K OSC With High Frequence OSC
  ///-------------------------------------------------------------------------------///
	SYSRC512KTrim();

  ///-------------------------------------------------------------------------------///
	/// 6. Dummy Report
  ///-------------------------------------------------------------------------------///
  DFResetDummyPointSend();
#ifdef FEATURE_51V
	// Workaround for 6SP issue
	CustomerResponseArray((BYTE *)CustomRes4Cmdinit,16, 0);
#endif
  ///-------------------------------------------------------------------------------///
  /// 7. Variable Init
  ///-------------------------------------------------------------------------------///
  VarInit();

  ///-------------------------------------------------------------------------------///
  /// 8.  SFR  init
  ///-------------------------------------------------------------------------------///  
  SYSInit();
  #ifdef	FEATURE_7101_DEVELOPMENT
	/// modem auto rst
	CLR_BIT(SYSREG, 1<<2);
	SET_BIT(REG32_PN_CTRL+1, 1<<2);   /// cic1
	SET_BIT(REG32_PN_CTRL+2, 1<<4);  ///str_pn_d

	// divider clk gate
	SET_BIT(REG32_PN_CTRL+2, 1<<1);  ///

	///modem self test
	// SET_BIT(REG32_PN_CTRL+1, 1<<5);   ///

	///by pass decorr
	//  SET_BIT(REG32_PN_CTRL+1, 1<<1);   ///

	/// square-wave mode
	//       CLR_BIT(REG32_AD_CTRL13, 1<<1);
	//        CLR_BIT(REG32_AD_CTRL13, 1<<4);  
	//	  CLR_BIT(REG_STR_PN_CTRL_1, 1<<0);
	// 	SET_BIT(REG32_DAC_CTRL0,RG_DAC0_PWD_15V);

	///   clk gate for reg
	SET_BIT(REG32_CKCON, 1<<3); 
	SET_BIT(REG32_CKCON, 1<<4); 

	/// off  FLASFTOP clk
	CLR_BIT(SYSREG, 1<<5);
	  	  
#endif
  ///-------------------------------------------------------------------------------///
  /// 9. Mutual Scan Trace Setup
  ///-------------------------------------------------------------------------------/// 

	#ifdef FEATURE_SCAN_REDUCED
	TSTraceInit(CAP_MODE);
	#else
  TSTraceInit();
  #endif
	#ifdef FEATURE_FORCE_TOUCH_SCAN
	#ifdef FEATURE_SCAN_REDUCED
	TSTraceInit(FORCE_MODE);
	#else
  TSForceTouchTraceInit();
	#endif
  #endif
		
  TSTraceAllSet(PAD_MODE_SEL_GND);

	#ifdef FEATURE_FORCE_TOUCH_SCAN
  TSForceTraceAllSet(PAD_MODE_SEL_GND);
  #endif

  ADCPowerOff(CAP_MODE);
  ///-------------------------------------------------------------------------------///
  /// 10. Region Scan Init
  ///-------------------------------------------------------------------------------///
#ifdef FEATURE_REGION_SCAN
  RegionScanInit();
#ifdef FEATURE_7101_DEVELOPMENT
#else
 PnSramTableInit();
#endif
#endif ///< for FEATURE_REGION_SCAN  
}

/**
 * @brief MainInit
 *
 *  Main function initialization
 *
 * @return NULL
 *
 */
int main()
{
  ///-------------------------------------------------------------------------------///  
  /// 1. init 
  ///-------------------------------------------------------------------------------///
  MainInit(); 

	///-------------------------------------------------------------------------------///  
	/// 2. Frequecny Hopping at Boot-Up
	///-------------------------------------------------------------------------------///	
	#ifdef FEATURE_TRIM_FREQ_HOP 
	TaskBootUpFrequencyHopping();
 	#endif ///< for FEATURE_TRIM_FREQ_HOP

	#ifdef FEATURE_BOOT_FREQUENCY_HOP 
	TaskBootUpFrequencyHop();
	MSHopDataInit();
	#endif
	
	#ifdef FEATURE_SYNC_REK	
  /// mainloop  
  LoopIdx = 0;
	#endif
	///-------------------------------------------------------------------------------///  
  /// 3. Do the Calibration
  ///-------------------------------------------------------------------------------///   
	CBTouchPanelCalibration();  	
  ALRegulateDataReset();
	
	///-------------------------------------------------------------------------------///
  /// 4. Watch Dog 
  ///-------------------------------------------------------------------------------///
	#ifdef FEATURE_WATCH_DOG  
  SYSWatchDogInit();  
	#endif ///< for FEATURE_WATCH_DOG

  ///-------------------------------------------------------------------------------///
  /// 5. Timer
  ///-------------------------------------------------------------------------------///
	#ifdef FEATURE_TIMER  
	CounterInit(ZetDF.cFrameRate.scAlgoFrameRate.wTimerPeriod);
  TimerInit(ZetDF.cFrameRate.scAlgoFrameRate.wTimerPeriod);
	#endif ///< for FEATURE_TIMER
  #ifdef FEATURE_TX_RX_IR	
	IRInit();
  #endif
	
	#ifdef FEATURE_ZET7101_CUSTOMER_TIMER2_ISR
	//CustomerTimer2Init(4000);
	CustomerTimer2Init(227); //about 1ms
  #endif
	///-------------------------------------------------------------------------------///  
  /// 6. Preparation to enter mainloop
  ///-------------------------------------------------------------------------------///
  #ifdef FEATURE_SYNC_REK
	LoopIdx=(LoopIdx+1)%2;   
	#else
  /// mainloop  
  LoopIdx = 0;
	#endif

 #ifdef FEATURE_DEEP_SLEEP_ESD_TEST				  
  {
		//ZetVar.wI2CnSPIRxLen = TP_CMD_B1_CMD_LEN;
		//ZetVar.wI2CnSPITxLen = TP_CMD_B1_TX_DATA_LEN;
		    
		/// Test Mode Enabled
		ZetVar.wSysMode |= (SYS_MODE_DEEP_SLEEP);
		/// Test Mode Disabled
		// ZetVar.wSysMode &= ~SYS_MODE_TP_TEST_EN;
		/// Cancel the previous interrupt, if any
		I2C_INT_HIGH();

		/// Link the buffer
		//LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
  }
#endif

  while(1)
  {
    ///-------------------------------------------------------------------------------///      
    /// 1. TASK#1 : Round Init 
    ///-------------------------------------------------------------------------------///
    TaskRoundInit();
#ifdef FEATURE_ZET7101_ANA2GPIO
    //sample code			
		ANA2GpioDataOut((READ_REG(P2)&0xFF));	//P2_0~P2_12	  		
    #ifdef FEATURE_ZET7101_GPIO_ISR
		//sample code
		ANA2GpioDataOut(gbTestP2Pin5); 
		ANA2GpioDataOut(gbTestP2Pin6); 
		#endif
#endif

    ///-------------------------------------------------------------------------------///  
    /// 2. TASK#2 : Re-Calibration
    ///-------------------------------------------------------------------------------///
    TaskCalibation();

		///-------------------------------------------------------------------------------///  
    /// 3. TASK#3 : Key Scan
    ///-------------------------------------------------------------------------------///
    #ifdef FEATURE_REAL_KEY   
    TaskKeyScan();
		#endif ///<for FEATURE_REAL_KEY   
		
		///-------------------------------------------------------------------------------///  
    /// 5. TASK#5 : Init-Scan :1.32ms
    ///-------------------------------------------------------------------------------///	
		#ifndef FEATURE_INIT_ISR_SCAN // ndef
		TaskInitScan();
		#endif	///< for FEATURE_INIT_ISR_SCAN		

		#ifdef FEATURE_NOISE_DETECTION_MODE
		TaskNoiseDetection();
		#endif

		///-------------------------------------------------------------------------------///  
    /// 6. Dynamic frequency Hopping
    ///-------------------------------------------------------------------------------///		
		#ifdef FEATURE_DYNAMIC_FREQUENCY_HOP 		
		TaskDynamicUpFrequencyHop();
		#endif

		///-------------------------------------------------------------------------------///  
    /// 7. Initial Scan for water issue
    ///-------------------------------------------------------------------------------///
		#ifdef FEATURE_WATER_INIT_SCAN
		TaskWaterInitScan();
		#endif
	
    ///-------------------------------------------------------------------------------///  
    /// 7. TASK#7 : Mutual Scan
    ///    TASK#7 : Region Scan    
    ///-------------------------------------------------------------------------------/// 
		#ifdef FEATURE_INIT_ISR_SCAN
		TaskIsrScanStart();		
		#endif ///< for FEATURE_INIT_ISR_SCAN
		
		#ifdef FEATURE_ALLPOINT_SCAN
    TaskRegionScan();
		#endif

    ///-------------------------------------------------------------------------------///  
    /// 8. TASK#8 : Algorithm
    ///-------------------------------------------------------------------------------///      
    TaskAlgorithm();

    ///-------------------------------------------------------------------------------///  
    /// 8.1 Gesture
    ///-------------------------------------------------------------------------------/// 
		#ifdef FEATURE_GESTURE
		if(ZetDF.cGesture.scGesture.bGestureEn != 0)
		{
			ZetVar2.bReportGestureId = TaskGesture(); 
		}
		#endif ///< for FEATURE_GESTURE	

 		///-------------------------------------------------------------------------------///  
    /// 9. TASK#9 : Data Transfer1
    ///-------------------------------------------------------------------------------/// 
		TaskDataFormat();
		#ifdef FEATURE_CUSTOMER_PROTOCOL	 
	  CustomerDataFormat();
		#endif

		///-------------------------------------------------------------------------------///
    /// 9. Wait untile the scan procedure done (PipeLine with Algorithm End )
    ///-------------------------------------------------------------------------------/// 
    WaitScanDone();

		///-------------------------------------------------------------------------------///  
    /// 10. TASK#9 :	TPTest  
    ///-------------------------------------------------------------------------------///
    TaskTPTest();    
    ///-------------------------------------------------------------------------------///  
    /// 11. TASK#10 : Power Control
    ///-------------------------------------------------------------------------------///
    #if defined(FEATURE_DEEP_SLEEP) || defined(FEATURE_IDLE)
    TaskPower();	
		#endif	

	#ifdef FEATURE_TX_RX_IR
		if(CustomerVar.bIREnable == 1)
		{
			MSMutualScanAnalogSetup();
			ADCPowerOn(CAP_MODE);
			IRSetup(40,0,100);
			ADCPowerOff(CAP_MODE);
		}
	#endif	

#ifdef FEATURE_SELF_SCAN
		CLR_BIT(REG32_PN_DEMOD_IF,1<<0);
		IntAdcEnable();
		MSMutualScanAnalogSetup();
		ADCPowerOn(CAP_MODE);
		MSSelfScanStart();
		ADCPowerOff(CAP_MODE);
		IntAdcDisable();
#endif

		///-------------------------------------------------------------------------------///  
		/// 12. TASK#10 : Period Control
		///-------------------------------------------------------------------------------///	   
		#ifdef FEATURE_TIMER
		TaskPeriodControl();
		#endif ///< for FEATURE_TIMER

		// Workaround for 6SP issue
		if((ZetVar.bWorkingState==WORKING_STATE_CUSTOMER_NORMAL)&&(I2C_INT()==FALSE))
		{
			I2C_INT_HIGH();
		}
	
    ///-------------------------------------------------------------------------------///
    ///  Mainloop Index ++
    ///-------------------------------------------------------------------------------///
    LoopIdx=(LoopIdx+1)%2;   
  }
  return 0;
}

