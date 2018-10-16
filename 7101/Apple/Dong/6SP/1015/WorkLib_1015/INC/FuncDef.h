/**
 * @file FuncDef.h
 *
 * Define global functions
 *
 * @version $Revision: 70 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/
#ifndef __FUNC_DEF__
#define __FUNC_DEF__
/// =============================================================///
///      START OF BOOT  FUNCTION DEFINITION                      ///
/// =============================================================///

///-------------------------------------------------------------------------------///
/// 1. Main Init
///-------------------------------------------------------------------------------///
EXTERN void MainInit();
EXTERN void VarInit(void);
#ifdef FEATURE_SWITCH_DRIVE_SENSE
EXTERN void ADBasePtrInit();
#endif ///< for FEATURE_SWITCH_DRIVE_SENSE

///-------------------------------------------------------------------------------///
/// 2. Task
///-------------------------------------------------------------------------------///
EXTERN void TaskRoundInit(void);
EXTERN void TaskCalibation(void);

#ifdef FEATURE_INIT_ISR_SCAN
EXTERN void TaskIsrScanStart(void);
#else ///< for FEATURE_INIT_ISR_SCAN
EXTERN void TaskInitScan(void);
#endif ///< for FEATURE_INIT_ISR_SCAN

EXTERN void TaskRegionScan(void);

EXTERN void TaskMutualScan(void);
EXTERN void TaskAlgorithm(void);
EXTERN void TaskKeyScan(void);
EXTERN void TaskDataFormat(void);
EXTERN void TaskTPTest(void);
#if defined(FEATURE_DEEP_SLEEP) || defined(FEATURE_IDLE)
EXTERN void TaskPower(void);
#endif
EXTERN void WaitScanDone();
#ifdef FEATURE_TIMER
EXTERN void TaskPeriodControl(void);
#endif ///< for FEATURE_TIMER

EXTERN void TaskWaterInitScan(void);

///-------------------------------------------------------------------------------///
/// 3. Debug Function
///-------------------------------------------------------------------------------///
EXTERN void DebugShow(BYTE bData);

///-------------------------------------------------------------------------------///
/// 3. Interrupts
///-------------------------------------------------------------------------------///
EXTERN void I2CTxRxReset(void);
EXTERN void IntInit(void);
EXTERN void IntAdcEnable(void);
EXTERN void IntAdcDisable(void);
EXTERN void IntTimerEnable(void);
EXTERN void IntTimerDisable(void);


///-------------------------------------------------------------------------------///
/// 4. SPI Function
///-------------------------------------------------------------------------------///

#ifdef FEATURE_SPI_TRANSFER
//EXTERN void SPISetNonIntTypeCmd();
EXTERN void SPISetWakeupTypeCmd();
EXTERN inline void SPIBufferWriteEn();
EXTERN void SPIInit(void);
EXTERN void SPIRxIsr(void);
EXTERN void SPI_INT_HIGH(void);
EXTERN void SPI_INT_LOW(void);
EXTERN BYTE SPI_INT_STATUS(void);
//EXTERN void  SPIWaitTransferDone(void);
#if 1 //def FEATURE_DEVELOP1 
EXTERN void SPISramInit(void);
EXTERN void SPISramReConfigure(void);
#endif
#ifdef FEATURE_CUSTOMER_PROTOCOL
EXTERN void SPIDispatchCustomerCmd(void);
#endif

#endif

#ifdef FEATURE_TX_RX_IR
EXTERN void IRInit(void);
EXTERN void IRSetup( unsigned char freq_multi, unsigned char sqrt_mode, unsigned char rx_sine_len);
#endif


#ifdef FEATURE_ZET7101_CUSTOMER_TIMER2_ISR
EXTERN	void CustomerTimer2Init(WORD wTimerPeriod);
EXTERN	void CustomerTimer2Isr();
#endif

///-------------------------------------------------------------------------------///
/// 4. I2C Function
///-------------------------------------------------------------------------------///
EXTERN void I2CInit(void);
EXTERN void I2CTxIsr(void);
EXTERN void I2CRxIsr(void);
EXTERN void I2CnSPIDispatchCmd();
EXTERN void I2CnSPIDispatchZetCmd();
EXTERN void I2CnSPIDispatchZetVenCmd();
EXTERN void I2CDataFormatReset(void);
EXTERN void I2CDataFormatResetInt(void);
EXTERN void I2CDataFormatIntLow(void);
EXTERN void I2CDataFormatDataInCheck(void);
#ifdef FEATURE_FINGER_UP_DEBOUNCE_ALG 
EXTERN void I2CDataFormatDataInCheckSub(void);
#endif ///< for FEATURE_FINGER_UP_DEBOUNCE_ALG
EXTERN void  I2CWaitTransferDone(void);
EXTERN void LinkBuffer(BYTE* Buffer);

#ifdef FEATURE_MCU_LIB_ENABLE
EXTERN BOOL I2C_INT(void);
//#endif

EXTERN void I2C_INT_LOW(void);
EXTERN void I2C_INT_HIGH(void);

#endif

///-------------------------------------------------------------------------------///
/// 5. System Control
///-------------------------------------------------------------------------------///
#ifdef FEATURE_7101_DEVELOPMENT
#ifdef FEATURE_ZET7101_ANA2GPIO
EXTERN void ANA2GpioInit(void);  
EXTERN void ANA2GpioDataOut(BYTE bData);
#endif
#ifdef FEATURE_ZET7101_GPIO_ISR
EXTERN void GpioISRInit(void);
EXTERN void GpioIsr(void);
#endif

EXTERN void SYSSetMcuIdle(void);
#endif


EXTERN void SYSSetFreqToneReg(BYTE bID);
EXTERN void SYSInit(void);
EXTERN void SYSGpioInit(void);
EXTERN void SYSAlgoPageInit(void);

EXTERN void SYSSoftReset(void);
EXTERN void SYSSoftResetInt(void);

EXTERN void SYSDeepSleep(void);
EXTERN void SYSIdle(void);
EXTERN void SYSWakeUp(void);

EXTERN void SYSWatchDogInit(void);

EXTERN void SYSRC512KTrim(void);

EXTERN void SYSCircuitPowerDown(void);
EXTERN void SYSTraceInit(void);

EXTERN void SYSChargerModeEnable();
EXTERN void SYSChargerModeDisable();
EXTERN void SYSSetDebounceCharger();
EXTERN void SYSSetDebounceNormal();

EXTERN void SYSPsuedoInit();
EXTERN void SYSPsuedoSetup(BOOL bChargerEnable);

#ifdef FEATURE_POWER_SAVE_MODE
EXTERN void SYSPowerSaveMode();
#endif ///< for FEATURE_POWER_SAVE_MODE

EXTERN void SYSSetVarNormal() reentrant;
EXTERN void SYSSetVarCharger() reentrant;

#ifdef FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS
EXTERN void SYSSetVarNormalHeavy();
EXTERN void SYSSetVarChargerHeavy();
#endif 

//#ifdef FEATURE_FIX_FRAME_RATE
EXTERN void SYSTimer0Set(void);
EXTERN void SYSTimer0SetInt(void);
//#endif
EXTERN void SYSSetInitialScanReg(void);
EXTERN void SYSKeyScanReg(void);
EXTERN void SYSSetMutualScanReg(void);
#ifdef FEATURE_FORCE_TOUCH_SCAN
EXTERN void SYSSetForceScanReg(void);
#endif
EXTERN void SYSSetPnChipDuration(BYTE bPnChipNumber , BYTE bPnControl);

EXTERN void SYSScriptRegInitial(void);



///-------------------------------------------------------------------------------///
/// 6. TP Command
///-------------------------------------------------------------------------------///
EXTERN void TPCmdB2Set(void);

EXTERN void TPCmdA0(void);
EXTERN void TPCmdA1(void);
EXTERN void TPCmdA3(void);
EXTERN void TPCmdAB(void);

///-------------------------------------------------------------------------------///
///  7. Calibration
///-------------------------------------------------------------------------------///
EXTERN void CBTouchPanelCalibration(void);
EXTERN void CBMutualScanCalibration(BYTE bCalibrationTimes);
EXTERN void CBMutualRevScanCalibration(void);
EXTERN void CBBaseArrayAlloc(void);
EXTERN void CBMutualScanTrackBaseCutDev(BOOL bADDDev);
EXTERN void CBMutualResumeTraceSetting(void);

#ifdef FEATURE_RECALIBRATION
EXTERN void CBInitSingedDevCheck(void);
EXTERN void CBResetReKCheck(void);
#endif ///< for FEATURE_RECALIBRATION
#ifdef FEATURE_BASE_TRACKING
EXTERN void CBBaseTrackCheck(void);
#endif ///< for FEATURE_BASE_TRACKING
#ifdef FEATURE_MUT_ABNORMAL_REK
EXTERN void CBMutReKCheck(void);
#endif ///< for FEATURE_MUT_ABNORMAL_REK

///-------------------------------------------------------------------------------///
///  8. Region Scan
///-------------------------------------------------------------------------------///
EXTERN void RegionScanDataInit(BYTE bApplyMode);
EXTERN void RegionDataStructSetup(BYTE bApplyMode);
EXTERN void RegionScanInit(void);
#ifdef FEATURE_7101_DEVELOPMENT
#else
EXTERN void PnSramTableInit(void);
#endif
EXTERN void RegionScanBaseTrack(void);

EXTERN void RegionScanProcess(void);

EXTERN void RegionScanCollectIsr(void);

#ifdef FEATURE_SCAN_REDUCED
EXTERN void RegionScanReadData(BYTE bScanType, BYTE bApplyMode, BYTE bDriveRegionId ,BYTE bSenseGroupId, BYTE bCalibrationTimes);
EXTERN void RegionScanIsr(BYTE bApplyMode);
EXTERN void RegionScanTriggerStart(BYTE bApplyMode);
EXTERN void RegionScanCalibration(BYTE bApplyMode, BYTE bCalibrationTimes);
#else
EXTERN void RegionScanIsr(void);
EXTERN void RegionScanTriggerStart(void);
EXTERN void RegionScanCalibration(BYTE bCalibrationTimes);

EXTERN void RegionScanReadAdcData(BYTE bDriveRegionId ,BYTE bSenseGroupId, BYTE bCalibrationTimes);
EXTERN void RegionScanReadDevData(BYTE bDriveRegionId ,BYTE bSenseGroupId);
#endif
EXTERN void RegionScanSetPNCodeTable(BYTE bPnChipID);

#ifdef FEATURE_1T2R_ARCHITECTURE
EXTERN int RegionDevArrayGet1T2RRow(int Row,int Col);
EXTERN int RegionDevArrayGetRevese1T2RCol(int i,int j);

#endif

#ifdef FEATURE_FORCE_TOUCH_SCAN

#ifdef FEATURE_SCAN_REDUCED
#else
EXTERN void ForceScanReadDevData(BYTE bDriveRegionId ,BYTE bSenseGroupId);
EXTERN void ForceScanReadAdcData(BYTE bDriveRegionId ,BYTE bSenseGroupId, BYTE bCalibrationTimes);
EXTERN void ForceScanTriggerStart(void);
EXTERN void ForceScanIsr(void);
EXTERN void ForceScanCalibration(BYTE bCalibrationTimes);
#endif


#endif

///-------------------------------------------------------------------------------///
///  8. Mutual Scan
///-------------------------------------------------------------------------------///
EXTERN void AdcIsr();

EXTERN void ResetADConverter(void);
EXTERN void StartADConverter(void);
EXTERN void WaitADCConverterDone(void);
EXTERN void ADCPowerOn(BYTE bApplyMode);
#ifdef FEATRUE_TURNOFF_UNUSED_ADC
EXTERN void ADCPartialPowerOn(BYTE bApplyMode);
#endif
#ifdef FEATRUE_USED_LESS_ADC
EXTERN void ADCLessPowerOn(BYTE bApplyMode);
#endif
EXTERN void ADCPowerOff(BYTE bApplyMode);

EXTERN void MSMutualScanDevDataInit(void);

EXTERN void MSMutualScanAnalogSetup(void);
EXTERN void MSMutualScanKeyAnalogSetup(void);
EXTERN void MSMutualScanKey(void);
EXTERN void MSMutualTraceMappingSetup(void) ;

EXTERN void MSMutualScanTriggerStart(void);
EXTERN void MSMutualScanAdcIsr(void);

EXTERN void ResetDevBuffer(BYTE bDataType);

#ifdef FEATURE_FORCE_TOUCH_SCAN
EXTERN void MSForceScanAnalogSetup(void);
EXTERN void MSForceScanDevDataInit(void);
#endif


///-------------------------------------------------------------------------------///
///  9. Key Scan
///-------------------------------------------------------------------------------///
EXTERN void KeyScanProcess(void);
EXTERN void KeyScanCalibration(void);
EXTERN void KeyScanBaseInit(void);
EXTERN void KeyScan(BOOL bDev, BYTE bScanTimes);

///-------------------------------------------------------------------------------///
///  9. Init Scan
///-------------------------------------------------------------------------------///
EXTERN void ISInitScanPrepareSetup(void);
EXTERN void ISInitScanEndSetup(void);
EXTERN void ISInitScanDriveAxisEvenTraceSetup(void);
EXTERN void ISInitScanDriveAxisOddTraceSetup(void);
EXTERN void ISInitScanSenseAxisEvenTraceSetup(void);
EXTERN void ISInitScanSenseAxisOddTraceSetup(void);

EXTERN void ISInitScanAnalogSetup(void);
#ifdef FEATURE_SCAN_REDUCED
void ISInitScanCalibration(BYTE bScanAxis, BYTE bADMode);
#else
EXTERN void ISInitScanSenseAxisCalibration(BYTE bADMode);
EXTERN void ISInitScanDriveAxisCalibration(BYTE bADMode); 
#endif

#ifdef FEATURE_INIT_ISR_SCAN
EXTERN void ISInitScanTriggerStart(void) ;
#endif ///<for FEATURE_INIT_ISR_SCAN



///-------------------------------------------------------------------------------///
/// 10. Algorithm
///-------------------------------------------------------------------------------///
#ifdef FEATURE_CO_AXIS_PROCESS
EXTERN void ALCoAxisStateCheck();
EXTERN void ALCoAxisProcess();
EXTERN void ALCoAxisDetect();
EXTERN BOOL ALCoAxisCheck();
#endif ///< for FEATURE_CO_AXIS_PROCESS

#ifdef FEATURE_HEAVY_PRESS
EXTERN void ALHeavyPress();
#endif ///<for FEATURE_HEAVY_PRESS


#ifdef FEATURE_NINE_SQUARE_ENABLE
EXTERN void ALTabletNineSquareSearch();
EXTERN void ALTabletNineSquareFingerSearch();
#ifdef FEATURE_ADV_NINE_SQAURE
EXTERN void ALTabletAdvNineSquareFingerSearch();
#endif ///< for FEATURE_ADV_NINE_SQAURE

EXTERN void ALTabletCoordinateProcess();
#ifdef FEATURE_BORDER_DEV
EXTERN void ALBorderDevComp(BYTE row, BYTE col, BYTE id);
#endif ///< for FEATURE_BORDER_DEV
#endif  ///< for FEATURE_NINE_SQUARE_ENABLE

#ifdef FEATURE_BORDER_COOR
#ifdef FEATURE_REV_131
EXTERN int ALBorderXComp(int iOrgCoorX,BYTE idx);
EXTERN int ALBorderYComp(int iOrgCoorY,BYTE idx);
#else
EXTERN int ALBorderXComp(int iOrgCoorX);
EXTERN int ALBorderYComp(int iOrgCoorY);
#endif
EXTERN WORD ALBorderCompStartEnd(BYTE BorderCoorCompenPercent, BYTE bAxisInterpolationLength);
#endif ///< for FEATURE_BORDER_COOR



#ifdef FEATURE_SENSE_DEV_PROCESS
#ifdef FEATURE_SENSE_DEV_PROCESS_MA_FILTER
EXTERN void ALSenDevProcessMA();
EXTERN void ALSenDevProcessEvenOdd(BYTE LoopSrt, BYTE LoopEnd, BYTE DrvAxisNo);
#else
EXTERN void ALSenDevProcess();
#endif
#endif ///< for FEATURE_SENSE_DEV_PROCESS


#ifdef FEATURE_DRIVE_DEV_PROCESS
#ifdef FEATURE_DRIVE_DEV_PROCESS_MA_FILTER
#ifdef FEATURE_1T2R_ARCHITECTURE_DEVPRORCESS
EXTERN void ALDriDevProcessMA1T2R();
#endif
EXTERN void ALDriDevProcessMA();
#else
EXTERN void ALDriDevProcess();
#endif  ///< FEATURE_MUTUAL_DRV_DEV_PROC_USING_MV_FILT

#ifdef FEATURE_1T2R_ARCHITECTURE_CROSSTALK_REDUCTION
EXTERN void	AL1T2RDevCompensation();
#endif

#ifdef FEATURE_1T2R_ARCHITECTURE_FARNEAR_REDUCTION
#ifdef FEATURE_1T2R_NEAR_FAR_DEV_COMPEN
EXTERN void	AL1T2RFarNearDevCompensation(void);
#endif
#ifdef FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND
EXTERN void	AL1T2RFarNearDevCompensation2nd(void);
#ifdef FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND_ACCURACY_MID_AREA_CORRECT_SUBROUTINE
EXTERN short ALDevReduce(short sDevRef1, short sDevRef2, short sDevTarget, BYTE bReduceRatio);
#endif
#endif
EXTERN void	Alg1T2RCompPtrInit(void);
#ifdef FEATURE_1T2R_NEAR_FAR_MA
EXTERN void AL1T2RFarNearDevCompenMA(void);
#endif
#endif

#endif ///< for FEATURE_DRIVE_DEV_PROCESS

EXTERN BOOL AL1T2REvenOddCheck(int Row , int Col);


EXTERN void ALSpecifyCutDev();

#ifdef FEATURE_INITIAL_DEV_PROCESS
EXTERN void ALInitialDevProcess();
#endif ///< for FEATURE_INITIAL_DEV_PROCESS

#ifdef FEATURE_VIRTUAL_KEY 
EXTERN void  ALVirtualKey();
#endif ///< for FEATURE_VIRTUAL_KEY

#ifdef FEATURE_ONEWAY_COORD_TRACK
EXTERN void ALFingerTracking();
#endif 

#ifdef FEATURE_ADVANCE_COORD_TRACK
EXTERN void ALFingerTrackingAdvance();
#endif 


EXTERN WORD ALCalculateCoordDiff();
#ifdef FEATURE_FINGER_UP_DEBOUNCE_ALG
EXTERN void ALDebouncingSystem(BYTE bFingerId,BYTE bDebounceCntDown,BYTE bDebounceCntUp);
EXTERN void ALReportZeroFingerCounting();
#else ///< for FEATURE_FINGER_UP_DEBOUNCE_ALG
EXTERN void ALDebouncing(BYTE bFingerId);
#endif ///< for FEATURE_FINGER_UP_DEBOUNCE_ALG

EXTERN void ALFingerDebounce();
EXTERN BYTE ALSteadyRange(BYTE bFingerId);

EXTERN void ALCoordinateSmoothing();
EXTERN void ALRegulateDataReset();

EXTERN void ALPalmDetect(void);

EXTERN void ALKeyDetect(void);

#ifdef FEATURE_SHARP_DEV_CUT
EXTERN void ALSharpDevCut(BYTE bMI, BYTE bMJ);
#endif ///< for FEATURE_SHARP_DEV_CUT

#ifdef FEATURE_ONE_FINGER_CLUSTER
EXTERN void ALTabletOneFingerCluster(void);
EXTERN void ALTabletOneFingerClusterSub(BYTE bSlot,BYTE bDriStart,BYTE bDriEnd,BYTE bSenStart,BYTE bSenEnd);
#endif ///< for FEATURE_ONE_FINGER_CLUSTER

#ifdef FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS
void ALInitHeavyPressDetectChangeMutualGain(void);
#endif ///< FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS

#ifdef FEATURE_COMP_MUT_WITH_INIT
EXTERN void ALInitDevCompMutDevCtrl();
EXTERN void ALInitDevCompMutDevProcess();
#endif ///< for FEATURE_COMP_MUT_WITH_INIT


#ifdef FEATURE_COORD_EDGE_POSITION_LOCK
#ifndef FEATURE_MERGE_CLOSE_CLUSTER_FINGER
EXTERN BOOL ALFingerInBorderCheck(BYTE idx);
#endif
#endif ///< for FEATURE_COORD_EDGE_POSITION_LOCK


EXTERN void ALCheckSumProcess(void);

#ifdef FEATURE_FLOATING_FINGER_ENHANCE
EXTERN void ALFingerEnhance(void);
#endif


#ifdef FEATURE_CLUSTER_ENABLE
EXTERN void ALClusterClearBuf();
EXTERN void ALPClusterSearch(void);
EXTERN void ALPClusterSearchSubReSumWeight(BYTE bRadius , BYTE bType);
EXTERN void ALPClusterSearchSubAlg(void);
EXTERN void ALPClusterSearchSubCoordProc();
EXTERN BOOL ALPClusterMaxDevWeightCheck(BYTE bID);

#ifdef FEATURE_MERGE_CLOSE_CLUSTER_FINGER
EXTERN void	ALPSearchTracking(void);
EXTERN void	ALPReportTracking(void);
#endif ///< for FEATURE_MERGE_CLOSE_CLUSTER_FINGER


#ifdef FEATURE_NEW_MERGE_CLOSE_CLUSTER_FINGER
EXTERN void	ALPhoneMergeCheck(void);
EXTERN int ALPClusterSearchSubCoordCalculate(int dwWeightUpper , int dwWeightLower);

#ifdef FEATURE_CLUSTER_MERGE_V61
EXTERN BYTE ALProjectDevPeakCnt(BYTE bID , BYTE bTHRatio);
EXTERN BYTE ALProjectDevPeakSearch(BYTE bTHRatio);
#endif
#endif

#endif  ///< for FEATURE_CLUSTER_ENABLE

#ifdef FEATURE_MERGE_CLOSE_NINESQU_FINGER
EXTERN void ALTabletMergeCloseFinger(void);
EXTERN void ALTabletSearchTracking(void);
EXTERN void ALTabletReportFingerTracking(void);
#endif ///< for FEATURE_MERGE_CLOSE_NINESQU_FINGER 

#ifdef FEATURE_MERGE_CLOSE_NINESQU_FINGER
EXTERN void ALMergeParamInitialization(void);
#endif

#ifdef FEATURE_MUT_PARTIAL_REK
EXTERN void ALClearPartialMutReKControl(void);
#endif

#ifdef FEATURE_LTYPE_DEV_RE_MAPPING
EXTERN void ALLTypeDevReMapping(void);
#endif

#ifdef FEATURE_HTYPE_DEV_RE_MAPPING
EXTERN void ALHTypeDevReMapping(void);
#endif

#ifdef FEATURE_RAW_DATA_COPY
EXTERN void ALDevRawDataCopy(void);
#endif

#ifdef FEATURE_MERGE_DEV_ARRAY
EXTERN void ALMergeDevMapping(void);
#endif

#ifdef FEATURE_BORDER_DEV_INTERNAL
EXTERN void ALBorderDevInternalComp();
#endif

EXTERN void ResetBaseBuffer(void);
#ifdef FEATURE_FORCE_TOUCH_SCAN
EXTERN void ResetForceBaseBuffer(void);
#endif

#ifdef FEATURE_FORCE_TOUCH  
EXTERN BYTE FingerMapForceDev(int bi , int bj);
EXTERN LocationType ClusterCenter(BYTE bID);
EXTERN WORD ALForceTouch(int bi , int bj);
EXTERN WORD UpdateSumValue(WORD NowSum, WORD LastSum, WORD LimitedValue);
#endif

 #ifdef FEATURE_GESTURE
EXTERN BYTE GetFreemancode(I2 xi, I2 yi);
EXTERN BYTE editDistance(BYTE lengthTarget, BYTE lengthSource, BYTE *bufTarget, BYTE *bufSource);

#endif

EXTERN void ALFingerSearchParam(BYTE bMode);
#ifdef FEATURE_WATER_INIT_SCAN
EXTERN void ALReseveMaxClusterInWaterMode(void);
#endif
EXTERN void ALInitDevRatio(void);


///-------------------------------------------------------------------------------///
///  11. Data Format Function
///-------------------------------------------------------------------------------///
EXTERN void DFResetDummyPointSend(void);
EXTERN void DFMutualAdBaseSend(BYTE bSenseNumber, BYTE bDriveNumber, BYTE bApplyMode);

#ifdef FEATURE_WIN8_DATAFORMAT 
EXTERN void DFWindows8Data(void);
#else
EXTERN void DFDynamicCoordinate(void);
#endif ///< for FEATURE_WIN8_DATAFORMAT

#ifdef FEATURE_DYNAMIC_FREQ_TRAN_MODE_ON
EXTERN void FHDMDFHopDataFormatSend(void);
#endif ///< FEATURE_DYNAMIC_FREQ_TRAN_MODE_ON

#ifdef FEATURE_DATA_FORMAT_MUTUAL_SCAN_ADBASE
EXTERN void DFMutualScanBase(void);
#endif
#ifdef FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE
EXTERN void DFKeyMutualScanBase(void);
#endif
#ifdef FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_DEV
EXTERN void DFKeyMutualScanDev(void);
#endif
#ifdef FEATURE_DATA_FORMAT_KEY_DATA
EXTERN void DFKeyData(void);
#endif
#ifndef	FEATURE_CUSTOMER_PROTOCOL_REDUCECODE // ndef
EXTERN void DFMutualScanDev(void);
EXTERN void DFInitScanBase(void);
EXTERN void DFInitScanDev(void);
#ifdef FEATURE_FORCE_TOUCH_SCAN 
EXTERN void DFForceScanBase(void);
EXTERN void DFForceScanDev(void);
#endif

#endif
EXTERN void DFCheckSumData(void);
#ifdef FEATURE_MUTUAL_DEV_SHOW_INIT_DEV
EXTERN void CopyInitDevToMutDev(void);
#endif

///-------------------------------------------------------------------------------///
/// 12. Timer
///-------------------------------------------------------------------------------///
EXTERN void TimerIsr();
EXTERN void TimerInit(WORD wTimerPeriod);
EXTERN void TimerSetup(BYTE bImmediately, WORD wTimerPeriod);
EXTERN void CounterInit(WORD wTimerPeriod);
///-------------------------------------------------------------------------------///
/// 13. Trace Control Function
///-------------------------------------------------------------------------------///
EXTERN void TSTraceToGnd(BYTE bTableIdx);
EXTERN void TSTraceToAdc(BYTE bTableIdx);
EXTERN BYTE TSTraceTableIdxGet(BYTE bTraceId);
EXTERN void TSTraceToPosWave(BYTE bTableIdx);
EXTERN inline void TSTraceToDrive(BYTE bTableIdx, BYTE bWave);

EXTERN void TSTraceAllSet(BYTE bMode);
#ifdef FEATURE_SCAN_REDUCED

void  TSTraceInit(BYTE bApplyMode);
#else

EXTERN void TSTraceInit(void);
#endif
#ifdef FEATURE_SCAN_REDUCED
EXTERN void TSAllTraceModeSet(BYTE bScanAxis, BYTE bMode , BYTE bApplyMode);

#else
EXTERN void TSSenseAllTraceModeSet(BYTE bMode ,BYTE bApplyMode);
#endif
EXTERN void TSSenseGroupTraceModeSet(BYTE bSenseGroupId, BYTE bMode , BYTE bApplyMode);
EXTERN void TSSenseGroupTraceToGnd(BYTE bSenseGroupId);
EXTERN void TSSenseGroupTraceToAdc(BYTE bGroupId ,BYTE bApplyMode);
EXTERN void TSKeySenseGroupTraceToAdc(void);
EXTERN void TSTraceModeSet(BYTE iTableIdx, BYTE bMode);

#ifdef FEATURE_INIT_REGION_SCAN
EXTERN void TSSenseAllTracePNModeSet(BYTE bRound);
EXTERN SBYTE const code RegionDriveTable[TABLE_DEFINE_REGION_ROW][MAX_REGION_ROUND];
#endif
#ifdef FEATURE_FORCE_TOUCH_SCAN
#ifdef FEATURE_SCAN_REDUCED
#else
EXTERN void TSForceTouchTraceInit();	
#endif
EXTERN void TSForceTraceAllSet(BYTE bMode);
#endif

EXTERN void TSDriveGroupTraceToGnd(BYTE bDriveGroupId , BYTE bApplyMode);
EXTERN void TSDriveGroupTraceToAdc(BYTE bDriveGroupId);
#ifdef FEATURE_SCAN_REDUCED

#else
EXTERN void TSDriveAllTraceModeSet(BYTE bMode);
#endif
EXTERN void TSDriveGroupTraceModeSet(BYTE bGroupId, BYTE bMode);
EXTERN void TSDriveGroupTraceModeSet(BYTE bDriveGroupId, BYTE bMode);

EXTERN void TSAllTraceToGnd(void);
EXTERN void TSDriveGroupTraceToDrive(BYTE bDriveRegionId , BYTE bApplyMode);
#ifdef FEATURE_NOISE_DETECTION_MODE
EXTERN void TaskNoiseDetection(void);
#endif

///-------------------------------------------------------------------------------///
/// 14. Coordinate Related
///-------------------------------------------------------------------------------///
EXTERN void CoorRawPointAdd(BYTE id, I2 x, I2 y);
EXTERN void CoorRawPointGet(BYTE id, int idx, PointType *point);
EXTERN void CoorExePointAdd(BYTE id, I2 x, I2 y);
EXTERN void CoorExePointGet(BYTE id, int idx, PointType *point);
EXTERN void CoorExePointSet(BYTE id, int idx, I2 x, I2 y);

#ifdef FEATURE_IIR_FILTER_SMOOTHING_SCALING
EXTERN void CoorExePointRemainderGet(BYTE id, Point16Type *point);
EXTERN void CoorExePointRemainderAdd(BYTE id, U2 x, U2 y);
#endif

EXTERN void CoorPointInit(BYTE id, I2 x, I2 y);

EXTERN void CoorSmoothing(BYTE id);
EXTERN void TimeCoorSmoothing(BYTE id);

#ifdef FEATURE_COORD_EDGE_POSITION_LOCK
EXTERN void CoordEdgeLimiter(WORD wCoord , BYTE idx , BYTE bAxis);
#endif ///< for FEATURE_COORD_EDGE_POSITION_LOCK

EXTERN void DeltaDistanceGet(BYTE id, int idx, WORD *wDist);
EXTERN void DeltaDistanceAdd(BYTE id, WORD wDist);


///-------------------------------------------------------------------------------///	
/// 2. Frequecny Hopping at Boot-Up
///-------------------------------------------------------------------------------///  
#ifdef FEATURE_TRIM_FREQ_HOP 
EXTERN void MSHopDataInitialization(void);
EXTERN void TaskBootUpFrequencyHopping(void);
//EXTERN SBYTE const code FrequencyTrimSet[];
#endif ///< for FEATURE_TRIM_FREQ_HOP

#ifdef FEATURE_HW_FREQUENCY_HOP
EXTERN void MSHopDataInit(void);

#ifdef FEATURE_BOOT_FREQUENCY_HOP
EXTERN void TaskBootUpFrequencyHop(void);
#endif
#ifdef FEATURE_DYNAMIC_FREQUENCY_HOP 
EXTERN void TaskDynamicUpFrequencyHop(void);
EXTERN void DynamicRecentScanDataRecovery(void);
EXTERN void MSHopSetMinNoiseToneID(void);
#endif
#endif

#ifdef FEATURE_GIO_ENABLE
EXTERN void GpioDataOut(BYTE bData);
EXTERN void GpioDataOut16(WORD dData);
EXTERN void GpioDataOut32(DWORD dData);
EXTERN void GpioDataOut64(QWORD qData);
#endif

#ifdef FEATURE_GESTURE
EXTERN BYTE TaskGesture(void);
#endif ///< for FEATURE_GESTURE

#ifdef FEATURE_CUSTOMER_PROTOCOL
EXTERN void CustomerDataFormat(void);
EXTERN void I2CDispatchCustomerCmd(void);
EXTERN void CustomerResponseArray(BYTE bTmpBuf[], WORD wI2CnSPITxLen, BYTE bResetRxTxIdx);
EXTERN void CustomerInit(void);
EXTERN void CustomerTxIsrINTControl(void);
#endif

///-------------------------------------------------------------------------------///
///  16. Self Scan
///-------------------------------------------------------------------------------///

#ifdef FEATURE_SELF_SCAN
EXTERN void MSSelfScanStart(void);
#endif

#endif ///< __FUNC_DEF__

