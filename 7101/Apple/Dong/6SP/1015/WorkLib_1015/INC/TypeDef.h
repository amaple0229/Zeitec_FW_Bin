/**
 * @file TypeDef.h
 *
 *  TypeDef.h define variable type
 *
 *
 * @version $Revision: 72 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__
/// =============================================================///
///  0. IC Setup 
/// =============================================================///
#ifdef FEATURE_PROJECT_ZET7100
#include "ZET7100_Region.h"
#endif

#ifdef FEATURE_PROJECT_ZET7101
#include "ZET7101_Region.h"
#endif


#ifdef FEATURE_CUSTOMER_PROTOCOL 
#ifdef FEATURE_7101_DEVELOPMENT
#define PAD_MAX_NUM 				  (52)
#else
#define PAD_MAX_NUM 				  (80)
#endif
#define CHANNEL_MAX_NUM 			(32) 	// the Max LCM Ratio = 16:10 => 52*16/(10+16)=32;
#define MAX_DEV_ARRAY_SIZE  	748 //34*22
#define MAX_BASE_ARRAY_SIZE 	640 //32*20
#else
#ifdef FEATURE_7101_DEVELOPMENT
#define PAD_MAX_NUM 				  (52)
#else
#define PAD_MAX_NUM 				  (80)
#endif 
#define CHANNEL_MAX_NUM 			(52)  // the Max LCM Ratio = 16:10 => 80*16/(10+16)=50; + 2 dummy
#define MAX_DEV_ARRAY_SIZE  	((PAD_MAX_NUM/2)+2	)*((PAD_MAX_NUM/2)+2	)
#define MAX_BASE_ARRAY_SIZE 	((PAD_MAX_NUM/2)	  )*((PAD_MAX_NUM/2)	  )	
#endif

#define MAX_DRIVE_TRACE_NUM      (CHANNEL_MAX_NUM)
#define MAX_SENSE_TRACE_NUM      (CHANNEL_MAX_NUM)
#define MAX_PN_DRIVE_ROUND_NUM   (CHANNEL_MAX_NUM)

#ifdef FEATURE_FORCE_TOUCH_SCAN
#define FORCE_BASE_ARRAY_SIZE     12   // (1+2)*(2+2)
#else // FEATURE_FORCE_TOUCH_SCAN
#define FORCE_BASE_ARRAY_SIZE     0
#endif // FEATURE_FORCE_TOUCH_SCAN

#ifdef FEATURE_7101_DEVELOPMENT
#define AD_MAX_NUM               (10)
#else
#define AD_MAX_NUM               (16)
#endif
#define MAX_TRACE_NUM            (MAX_DRIVE_TRACE_NUM)
#define MAX_FORCE_TRACE_NUM      (2)

#ifdef  FEATURE_PNSCANDATA_REDUCED
#ifdef FEATURE_7101_DEVELOPMENT
#define PN_MAX_NUM             (31)
#define PN_TONE_NUM            (1)
#else
#define PN_MAX_NUM             21 // (31)
#define PN_TONE_NUM             2 // (1)
#endif
#else
#define PN_MAX_NUM               (21)
#define PN_TONE_NUM              (2)
#endif

#define TRUM_OUT_NUM 	           (6)

#define KEY_MAX_NUM              (8)
#define KEY_AD_ROUND             (1)
#ifdef FEATURE_CLUSTER_ENABLE	// cell phone
#define FINGER_MAX							 (6)       ///< Max Finger Number for algorithm
#define FINGER_MAX_REPORT				 (5)       ///< Max Finger Number for algorithm
#else
#define FINGER_MAX							 (11)       ///< Max Finger Number for algorithm
#define FINGER_MAX_REPORT				 (10)       ///< Max Finger Number for algorithm
#endif

#define LOCAL_CLUSTER_FINGER_MAX (4)        ///< Max Finger Number for 1 cluster

#define MAX_RXSINE_TABLE_NUM		 (224)
#define MAX_RXCOEF_TABLE_NUM		 (416)
#define MAX_TXSINE_TABLE_NUM		 (1632)

#define ABS(x) (((x)>=0)?(x):-(x))
#define ABSDIFF(a,b) ((a>b)?(a-b):(b-a))


#define MAX_GESTURE_MAX							(100)

///-------------------------------------------------------------------------------///
/// Freq Hopping
///-------------------------------------------------------------------------------///
#ifdef FEATURE_7101_DEVELOPMENT
#define FREQ_HOP_SET_NUM  (10)
#else
#define FREQ_HOP_SET_NUM  (32)
#endif
/// =============================================================///
///  1. General type definition
/// =============================================================///
/// DWORD
typedef unsigned long        DWORD;
/// WORD
typedef unsigned short       WORD;
/// BYTE
typedef unsigned char        BOOL;
/// BYTE
typedef unsigned char        BYTE;
/// Signed byte
typedef char                 SBYTE;
/// U4
typedef unsigned long        U4;
/// U2
typedef unsigned short       U2;
/// U1
typedef unsigned char        U1;
/// I2
typedef short                I2;

typedef long                 I4;

typedef unsigned long long   QWORD;

/// =============================================================///
/// 2. Define the data flash class and sub-class
/// =============================================================///
#include "DataFlash.h"

/// Region setup 

//#define HALF_MAX_SENSE_TRACE_NUM      (MAX_SENSE_TRACE_NUM>>1)
#define HALF_MAX_DRIVE_TRACE_NUM      (MAX_DRIVE_TRACE_NUM>>1)
/// =============================================================///
/// Dev register
/// =============================================================///
typedef struct PACK RegTrnumSt
{
	int Channel[TRUM_OUT_NUM];
}RegTrnumType;
/// =============================================================///
/// Dev register
/// =============================================================///
typedef struct PACK RegDevSt
{
  BYTE bSignedDev;  
  BYTE bDev;
}RegDevType;

/// =============================================================///
/// Finger Point Type
/// =============================================================///
typedef struct PointStruct
{
  I2  x;
  I2  y;
}PointType;

/// =============================================================///
/// Finger Point Type (32 bits)
/// =============================================================///
#ifdef FEATURE_IIR_FILTER_SMOOTHING_SCALING
typedef struct Point32Struct
{
  U4  x;
  U4  y;
}Point32Type;

typedef struct Point16Struct
{
  U2  x;
  U2  y;
}Point16Type;
#endif

/// =============================================================///
/// Force Finger Point Type
/// =============================================================///
typedef struct AdvPointStruct
{
  I2  x;
  I2  y;
	#ifdef FEATURE_FORCE_TOUCH
	BYTE  z;
	#endif
}AdvPointType;

/// =============================================================///
/// Finger Location Type
/// =============================================================///
typedef struct LocationStruct
{
  BYTE i;
  BYTE j;
}LocationType;

typedef struct LocalPeakInfoStruct
{
  BYTE i;
  BYTE j;
	#ifdef FEATURE_CLUSTER_MERGE_LOCAL_PEAK_SORT
	short sPeakDev;
	#endif
}LocalPeakInfoType;

#ifdef FEATURE_NEW_MERGE_CLOSE_CLUSTER_FINGER
/// =============================================================///
/// Location Limit Type
/// =============================================================///
typedef struct ClusterRangeStruct
{
  SBYTE LEdge;
  SBYTE HEdge;
}ClusterRangeType;

/// =============================================================///
/// Point Relative Location Shift
/// =============================================================///
typedef struct PointShiftStruct
{
  short sDeltaX;
  short sDeltaY;
}PointShiftType;

/// =============================================================///
/// Eight Way
/// =============================================================///
typedef struct EightWayDataStruct
{
  SBYTE  sbi;	
  SBYTE  sbj;		
}EightWayDataType;
#endif

/// =============================================================///
/// AD BASE Data for Mutual Scan
/// =============================================================///
typedef struct ADBaseDataSt
{
  WORD AdBaseAlloc[MAX_BASE_ARRAY_SIZE + FORCE_BASE_ARRAY_SIZE];
}ADBaseDataType;

/// =============================================================///
/// AD BASE Data for Init Scan
/// =============================================================///
#define INIT_SCAN_ADBASE_TRACE_NUM ((MAX_DRIVE_TRACE_NUM + 1))
typedef struct InitScanDriveAdBaseSt
{
  WORD AdBaseInit[INIT_SCAN_ADBASE_TRACE_NUM];  
}InitScanDriveAdBaseType;
  
/// =============================================================///
/// For DEV BASE Data
/// =============================================================///
typedef struct InitScanDriveDevDataSt
{
  short sDev[MAX_TRACE_NUM + 2];       ///< Unsigned dev Raw data  
  short sDevAlg[MAX_TRACE_NUM + 2];
}InitScanDriveDevDataType;

/// =============================================================///
/// For Init Scan Drive Axis Drive and Sense Axise Sense 
/// =============================================================///
typedef struct InitScanSenseAdBaseSt
{
  WORD AdBaseInit[INIT_SCAN_ADBASE_TRACE_NUM];
}InitScanSenseAdBaseType;

typedef struct InitScanSenseDevDataSt
{
	short sDev[MAX_TRACE_NUM + 2];
	short sDevAlg[MAX_TRACE_NUM + 2];
}InitScanSenseDevDataType;	

/// =============================================================///
/// For Mutual Scan Dev Data
/// =============================================================///
typedef struct MutualScanDevDataSt
{
  BYTE bFound[MAX_DRIVE_TRACE_NUM + 2];
  short sDevAlloc[MAX_DEV_ARRAY_SIZE];       ///< Raw data
  short *sDev[MAX_DRIVE_TRACE_NUM + 2];
  short *sRawDev[MAX_DRIVE_TRACE_NUM + 2];  
#ifdef FEATURE_MUTUAL_SCAN_DEV_DETECT_HEAVY_PRESS
	BYTE bHeavyPressIndexByMutual;
#endif ///< for FEATURE_MUTUAL_SCAN_DEV_DETECT_HEAVY_PRESS
	BYTE bSenFound[MAX_TRACE_NUM+2];
}MutualScanDevDataType;

/// =============================================================///
/// For Force Scan Dev Data
/// =============================================================///
#ifdef FEATURE_FORCE_TOUCH_SCAN
typedef struct ForceScanDevDataSt
{
  short sDevAlloc[FORCE_BASE_ARRAY_SIZE];       ///< Raw data
  short *sDev[4]; // 2+2
}ForceScanDevDataType;
#endif

/// =============================================================///
/// For Key Data
/// =============================================================///
#ifdef FEATURE_REAL_KEY 
typedef struct KeyCtrlSt
{
  WORD wKeyAdBase[KEY_MAX_NUM]; 
	BYTE bKeyDevData[KEY_MAX_NUM];
	BYTE bKeyValidByte; 
}KeyCtrlType;
#endif

#ifdef FEATURE_VIRTUAL_KEY 
typedef struct KeyCtrlSt
{
  WORD wKeyAdBase[KEY_MAX_NUM]; 
	BYTE bKeyDevData[KEY_MAX_NUM];
	BYTE bKeyValidByte; 
}KeyCtrlType;
#endif
/// =============================================================///
/// For Finger Search Data
/// =============================================================///
#ifdef FEATURE_NINE_SQUARE_ENABLE
typedef struct SearchFingerRecordStruct
{
  BYTE bFingerI;	
	BYTE bFingerJ;
  BYTE bFingerRightDownI;	
	BYTE bFingerRightDownJ;
	#ifdef FEATURE_ADV_NINE_SQAURE
	BYTE bSize;
	#endif
	#ifdef FEATURE_SEARCH_FINGER_TOUCH_FLAG
	BYTE bTouch;
	#endif
	I2	 iCoordinateX;
	I2	 iCoordinateY;
	#ifdef FEATURE_FINGER_SCHMIT_TRIGGER
	WORD wFingerFoundWeight;
	//WORD wFingerWeightSchmit;
	#endif	
}SearchFingerRecordType;
#endif ///< for FEATURE_NINE_SQUARE_ENABLE

#ifdef FEATURE_CLUSTER_ENABLE   
typedef struct ClusterStructrue
{
	#ifdef FEATURE_MERGE_CLOSE_CLUSTER_FINGER
	PointType ptRawCoordinate;
	#endif ///< for FEATURE_MERGE_CLOSE_CLUSTER_FINGER
	BYTE bClusterStatus;
	BYTE bClusterID;
	BYTE bClusterSize;
	BYTE bClusterLeft;
	BYTE bClusterRight;
	BYTE bClusterDown;
	BYTE bClusterUp;
	WORD wClusterWeight;
	#ifdef FEATURE_CLUSTER_LIMIT_LOCAL_PEAKTH
	short wClusterMaxPeakDev;
	#endif		
	U4   dwClusterWeightXU;
	U4   dwClusterWeightYU;
} ClusterStructrueType;

#ifdef FEATURE_MERGE_CLOSE_CLUSTER_FINGER
typedef struct ClusterTrackCoordinateStruct
{
	PointType ptSearchTrackCoord;	
	
	BYTE bStatus;
	BYTE bRootClusterID;			///< the source of cluster data structure ID

	///BYTE bStatus1;

	#ifdef FEATURE_NEW_MERGE_CLOSE_CLUSTER_FINGER
	BYTE bClusterSize;
	
	BYTE bInitClusterSize;
	BYTE bMergeDebounce;
	BYTE bExistDebounce;
	//BYTE bPeakCnts;
	BYTE bFakeDivideDebounce;
	PointShiftType SubPointShift[4];
	#endif

	#ifdef FEATURE_CLUSTER_MERGE_V61
	BYTE bStateMode;
	BYTE bInitMulPeakCnt; ///< initial 3 round Multi-Peak cnt
	#endif
}ClusterTrackCoordinateType;

#endif
#endif


/// =============================================================///
/// For Coordinate Point data
/// =============================================================///
#ifdef FEATURE_KALMAN_FILTER_SMOOTHING
typedef struct KFilterDataSt
{
  long mp;
  I2 preDataX;
  I2 preDataY;    
} KFilterDataType;
#endif ///< for FEATURE_KALMAN_FILTER_SMOOTHING

#if defined(FEATURE_MERGE_CLOSE_CLUSTER_FINGER) || defined(FEATURE_MERGE_CLOSE_NINESQU_FINGER)
typedef struct MergeIDCombinationStruct
{
	BYTE bStstus;	
	BYTE bDebounceCnt;	
}MergeIDCombinationType;
#endif

#ifdef FEATURE_MERGE_CLOSE_NINESQU_FINGER
typedef struct TrackCoordinateStruct
{
	BYTE bStatus;
	BYTE bMergeDebounce;
	PointType ptSearchTrackCoord;
	AdvPointType ptReportTrackCoord;
	LocationType XYStart;
	LocationType XYEnd;
}TrackCoordinateType;
#endif 

typedef struct FingerCoordinateRecordStruct
{
		#ifdef FEATURE_TRACK_PREDICT_COORD
		WORD	wPredictedCoordinateX;
		WORD	wPredictedCoordinateY;
		#endif ///< for FEATURE_TRACK_PREDICT_COORD

    #ifdef FEATURE_IIR_FILTER_SMOOTHING_SCALING
    WORD  wLsb16LastPtX;
    WORD  wLsb16LastPtY;
    #endif
		
		WORD	wLastDistance[REC_DISTANECE_LEN];
		BYTE	bDistRecIdx;
		BYTE	bFingerDebounce;				 ///< The finger shall keep visible 	for #N	frames then it would be treated as a	finger click (Finger down )
		BYTE	bFingerUpState; 				 ///< The finger shall keep invisible for #N	frame then it would be treated as a  finger leave (Finger up )	
		BYTE	bFingerDown;
	
		BYTE	bIIRWeight;
		
		WORD wFingerStatus;
	
		///---------------------------------------------///
		///  Points queue
		///---------------------------------------------/// 
		BYTE cRawPointIdx;
		BYTE cExePointIdx;
		BYTE cPointCnt;  
		AdvPointType ptCoordinate;  
		PointType ptExePoints[EXE_POINT_LEN]; 	 
		PointType ptRawPoints[RAW_POINT_LEN];	
	#ifdef FEATURE_COORD_EDGE_POSITION_LOCK
		PointType ptBorderLockedCoord;  
	#endif ///< for FEATURE_COORD_EDGE_POSITION_LOCK		
	
	#ifdef FEATURE_KALMAN_FILTER_SMOOTHING
		KFilterDataType kData;
	#endif ///< for FEATURE_KALMAN_FILTER_SMOOTHING
}FingerCoordinateRecordType;

/// =============================================================///
/// Dynamic Mode
/// =============================================================///
typedef struct PACK PointDataStruct
{
  BYTE bXYHighByte;
  BYTE bXLowByte;
  BYTE bYLowByte;    
  BYTE bZLowByte;  
}PointDataType;

typedef struct PACK DynamicDataStruct
{
  BYTE bPacketId;
  WORD wValid;
  PointDataType point[I2C_FINGER_BUFFER_NUM];
  BYTE bKeyStatus;
}DynamicDataType;

/// =============================================================///
/// Focal Data Type Mode
/// =============================================================///
#define MAX_FOCAL_FINGER_NUM        (5)

typedef struct FocalPointDataStruct
{
  BYTE bXHighByte;
  BYTE bXLowByte;
  BYTE bYHighByte;
  BYTE bYLowByte;    
  WORD wDummy;    
}FocalPointDataType;

typedef struct FocalDataStruct
{
  BYTE bDeviceMode;
  BYTE bGestId;
  BYTE bTdStatus;
  FocalPointDataType point[MAX_FOCAL_FINGER_NUM];
}FocalDataType;

/// =============================================================///
/// Virtual Key
/// =============================================================///
typedef struct VirtualKeyStruct
{
  WORD x; ///< Left of key
  WORD y; ///< Top of key
  BYTE w; ///< width of key;  
  BYTE h; ///< height of key;  
}VirtualKeyType;

/// =============================================================///
/// Collect Dev
/// =============================================================///
typedef struct PACK DevCollectSetupStruct
{
  BYTE bTotal;
  BYTE bPow;  
  //BYTE bNoCut;
}DevCollectSetupType;

/// =============================================================///
/// Hopping R/C Trim setup
/// =============================================================///
#ifdef FEATURE_HW_FREQUENCY_HOP
typedef struct HopDataStruct
{
	WORD wRawADBase;
	WORD wADDeltaAccumulator;
	WORD wADDeltaSumation;
	WORD wADDeltaHistory[HISTORY_SAVE_NUM];
	BYTE bToneA;	
} HopDataType;

typedef struct HopSortDataStruct
{
	BYTE bHopID;
	WORD wHopData;
} HopSortDataType;
#endif
#ifdef FEATURE_TRIM_FREQ_HOP
typedef struct HopDataStruct
{
	WORD wRawAD[AD_MAX_NUM];
	WORD wADDeltaAccumulator;
	WORD wADDeltaSumation;	
	#ifdef FEATURE_DEBUG_TRIM_FREQ_HOP
	WORD wDebugAD[AD_MAX_NUM];
	#endif
} HopDataType;

typedef struct HopSortDataStruct
{
	BYTE bHopID;
	WORD wHopData;

} HopSortDataType;
#endif ///< for FEATURE_TRIM_FREQ_HOP

/// =============================================================///
/// Region PN Scan Data strucurt for ZET7100
/// =============================================================///
typedef struct PACK SubRegionDataStruct
{
  BYTE bRegionTraceNum; 					///< Trace numbers in a region
  #ifdef FEATURE_PNSCANDATA_REDUCED
	BYTE *pbRegionTrace;
	BYTE *pbRow;
	#else
  BYTE bRegionTrace[PN_MAX_NUM]; 	///< Trace IDs
  BYTE bRow[PN_MAX_NUM]; 					///< Trace row position 
  #endif
  #ifdef FEATURE_PN_VIOLATE_FIX 
  DWORD dwEnPad; 									///< Trace row is 1:enable/0:disable
  #endif
  DWORD dwTrumReg[TRUM_OUT_NUM]; 						///< TRNUM_OUT0 ~ TRNUM_OUT5

}SubRegionDataType;

typedef struct PACK RegionDataStruct
{
  BYTE bToneNum; ///< Tone numbers in this Round
  SubRegionDataType ToneData[PN_TONE_NUM];
  
}RegionDataType;

/// =============================================================///
/// GPIO Register define for ZET6275
/// =============================================================///
typedef struct GpioSt
{
  unsigned bit0:1;
  unsigned bit1:1;
  unsigned bit2:1;
  unsigned bit3:1;
  unsigned bit4:1;
  unsigned bit5:1;
  unsigned bit6:1;
  unsigned bit7:1;
  unsigned unsued:24;
}GpioType;

/// ============================================================================================================================///
/// ZetVar
/// ============================================================================================================================///
typedef struct GlobalVarSt
{
  ///----------------------------------------------------------------------------------------------------------------------------///
  /// I:  Data Buffers
  ///----------------------------------------------------------------------------------------------------------------------------///
  ADBaseDataType  MutualScanAdBase;                               /// XDATA[500]
  InitScanDriveAdBaseType  InitScanDriveAdBase;                   /// EVEN: XDATA[980], ODD: XDATA[9B0] 
  InitScanDriveDevDataType   InitScanDriveDevData;                /// XDATA[9E0]  

  MutualScanDevDataType MutualScanDevData[2];
	WORD *AdBaseRound[MAX_TRACE_NUM]; 
	#ifdef FEATURE_PROCESS_COORDINATE_WITH_RAW_DATA
  short MutualScanRawDevData[MAX_DEV_ARRAY_SIZE];
	#endif ///< for FEATURE_PROCESS_COORDINATE_WITH_RAW_DATA

	#ifdef FEATURE_FORCE_TOUCH_SCAN
	ForceScanDevDataType ForceScanDevData[2];
  WORD *ForceAdBaseRound[MAX_FORCE_TRACE_NUM]; 
	#endif

	#ifdef FEATURE_NINE_SQUARE_ENABLE
	SearchFingerRecordType SearchFingerRecord[FINGER_MAX];          ///< XDATA[0x1118] 
	#endif
	
	#ifdef FEATURE_CLUSTER_ENABLE  
	BYTE ClusterIDArray[CLUSTER_ID_ARRAY_NUM][(MAX_TRACE_NUM +2)];	
	DWORD ClusterSchmitSwitch[MAX_TRACE_NUM];
	
	#ifdef FEATURE_MUT_PARTIAL_REK
	DWORD BaseUpdateControl[MAX_TRACE_NUM];
	#endif
	#ifdef FEATURE_MUT_PARTIAL_CROSS_SET
	DWORD BaseUpdateCross[MAX_TRACE_NUM];
	#endif

	#endif ///< for FEATURE_CLUSTER_ENABLE

	#ifdef FEATURE_ADV_NINE_SQAURE
	BYTE NineSquClusterIDData[(MAX_TRACE_NUM +2)*(CLUSTER_ID_ARRAY_NUM)];
	BYTE *NineSquClusterIDArray[CLUSTER_ID_ARRAY_NUM];
	#endif ///< for FEATURE_ADV_NINE_SQAURE
	
	//----------------------------//
	// HW Frequency Hop data stucture
	//----------------------------//	
	#if defined(FEATURE_TRIM_FREQ_HOP) || defined(FEATURE_HW_FREQUENCY_HOP)
	HopDataType	HopData[FREQ_HOP_SET_NUM];
	HopSortDataType	HopSortData[FREQ_HOP_SORT_NUM];	
	#endif

	#ifdef FEATURE_CLUSTER_MERGE_V61
	short ProjectDev[MAX_TRACE_NUM];
	#endif
  ///----------------------------------------------------------------------------------------------------------------------------///
  ///   Init Scan Test Buffer
  ///----------------------------------------------------------------------------------------------------------------------------///
  InitScanSenseAdBaseType   InitScanSenseAdBase;
  InitScanSenseDevDataType  InitScanSenseDevData; 

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///   B2 Command Buffer
  ///----------------------------------------------------------------------------------------------------------------------------///
  BYTE bB2Buf[B2_BUF_LEN];  
	
  ///----------------------------------------------------------------------------------------------------------------------------///
  ///   f9 Command Buffer
  ///----------------------------------------------------------------------------------------------------------------------------///
  BYTE bF9Buf[F9_BUF_LEN];  

	///----------------------------------------------------------------------------------------------------------------------------///
  ///   fb Command Buffer
  ///----------------------------------------------------------------------------------------------------------------------------///
  #ifdef FEATURE_I2C_CMD_FB_DEVICE_NAME
	BYTE bFBBuf[FB_BUF_LEN];  
	#endif

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    3. System  Mode
  ///----------------------------------------------------------------------------------------------------------------------------///
  WORD wSysMode;
  WORD wSysMode2;  
  WORD wTestMode;
  WORD wAlgoStatus;
	WORD wAlgoStatus2;
	BYTE bCalibrationCtrl;
	BYTE bLcdOn;											///< LCD on:1 LCD off:0

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    5:  Cut DEV
  ///----------------------------------------------------------------------------------------------------------------------------///
  //WORD wCutDevRegionScan;
  //WORD wCutDevInitScan;
  
  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    6. Scan Control Methods 
  ///----------------------------------------------------------------------------------------------------------------------------///
  BYTE bScanMethod;
  BYTE bScanADCtrl;

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    7. Mutual Scan Control Queue
  ///----------------------------------------------------------------------------------------------------------------------------///

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    8: SRAM Access Control
  ///----------------------------------------------------------------------------------------------------------------------------///
  U4 dwSramAddr;

  
  BYTE bInterfaceType; //SPI or I2C	

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    9: I2C 
  ///----------------------------------------------------------------------------------------------------------------------------///

  BYTE *pbI2CnSPIRxData; 
	BYTE *pbI2CnSPITxData; 
	WORD wI2CnSPITxIdx; 										///< I2C TX Buffer Idx
	WORD wI2CnSPIRxIdx;										 ///< I2C RX Buffer Idx
	
	
  BYTE bI2CCmd;

  WORD wI2CnSPIRxLen;                     ///< Data length which is needed to be transferred
  WORD wI2CnSPITxLen;                     ///< Data length which is needed to be transferred

 
  BYTE bI2cStatus;                    ///< I2C Status

	BYTE bVenCmd;                       ///< Command Mode 
	BYTE bI2cData;                   	 ///< I2C Data

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    10. Key Control
  ///----------------------------------------------------------------------------------------------------------------------------///
  #if defined(FEATURE_REAL_KEY) ||defined(FEATURE_VIRTUAL_KEY) 
	KeyCtrlType KeyCtrl;
	BYTE bKeyMax;
	BYTE bKeyDebounce[KEY_MAX_NUM];
	BYTE bKeyReleaseCnt;
	BYTE bKeyLastStatus;
	BYTE bKeyLockStatus;
	BYTE bKeyLockRelease; 
	BYTE bKeyRek;
	#endif ///< for FEATURE_VIRTUAL_KEY

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    13. Data Transfer
  ///----------------------------------------------------------------------------------------------------------------------------///
  BYTE *pCurReportData;

  BYTE bTranType;                     ///< Data Format Type      

	#ifdef FEATURE_RING_BUFFER
	BYTE bAlgRingBufCnt;              ///< Ring buffer
	BYTE bRingBufFlag;	              ///< Ring buffer
	BYTE bI2CRingBufCnt;              ///< Ring buffer	
	#endif

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    14. Algorithm Data
  ///----------------------------------------------------------------------------------------------------------------------------///
	#ifdef FEATURE_PALM_DETECT
	BYTE bPalmState;                    ///< 0: No Palm  >0: Palm on TP
	#endif
	#ifdef FEATURE_ADV_NINE_SQAURE
	BYTE bSearchIDFound;  
	#endif

	BYTE bADC0Gain;
	#ifdef FEATURE_FORCE_TOUCH_SCAN
	BYTE bForceADC0Gain;	
	#endif
	
  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    15. Virtual Key
  ///----------------------------------------------------------------------------------------------------------------------------///
#ifdef FEATURE_VIRTUAL_KEY
  VirtualKeyType const code *pVirtualKey[VIRTUAL_KEY_MAX_NUM];
  ///-------------------------------------------------------------///
  /// BIT#0 : 1 --> Key#0 (Search) pressed, 0 --> Key#0 released 
  /// BIT#1 : 1 --> Key#1 (Back)   pressed, 0 --> Key#1 released
  /// BIT#2 : 1 --> Key#2 (Home)   pressed, 0 --> Key#2 released
  /// BIT#3 : 1 --> Key#3 (Value)  pressed, 0 --> Key#3 released  
  ///-------------------------------------------------------------///  
  BYTE bVirtualKeyValidByte; 
	BYTE bVirtualKeyDeboucneCnt; 
  BYTE bVirtualKeyEnable;
#endif ///< for FEATURE_VIRTUAL_KEY

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    17. Re-Calibration
  ///----------------------------------------------------------------------------------------------------------------------------///
  BYTE bReCalDetNum;
	BYTE bReCalDetCnt;

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    18. Power Saving mode
  ///----------------------------------------------------------------------------------------------------------------------------///  
	#ifdef FEATURE_POWER_SAVE_MODE  
  BYTE bNoFingerRoundCount;
	#endif

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    23. Region Scan
  ///----------------------------------------------------------------------------------------------------------------------------/// 
#ifdef FEATURE_REGION_SCAN
	BYTE bADStrAmpIntCnt;                   ///<
#endif ///< for FEATURE_REGION_SCAN

	///----------------------------------------------------------------------------------------------------------------------------///
	/// 	 24. Border Dynamic Comp Dev
	///----------------------------------------------------------------------------------------------------------------------------///	
#ifdef FEATURE_BORDER_DEV
	WORD wBorderDriFarWeightUpper;
	WORD wBorderDriFarWeightLower;	
	WORD wBorderSenFarWeightUpper;
	WORD wBorderSenFarWeightLower;		
#endif ///< for FEATURE_BORDER_DYNAMIC_DEV

 	//BYTE bGenBuf1[MAX_GENBUF1_NUM];
	BYTE bGenBuf1[(HALF_MAX_DRIVE_TRACE_NUM+2)*4*2];
	
	///----------------------------------------------------------------------------------------------------------------------------///
 	///		30. Check Sum
	///----------------------------------------------------------------------------------------------------------------------------/// 
	#ifdef FEATURE_CHECK_SUM
	BYTE bROMCheckSum ;
	#endif 

	///----------------------------------------------------------------------------------------------------------------------------///
 	///	Finger Tracking DataBuffer
	///----------------------------------------------------------------------------------------------------------------------------/// 
	#ifdef FEATURE_ADVANCE_COORD_TRACK
	WORD wMDistData[MAX_TRACK_NUM][MAX_TRACK_NUM];
	BYTE bRowCover[MAX_TRACK_NUM];
	BYTE bColCover[MAX_TRACK_NUM];
	BYTE bMask[MAX_TRACK_NUM][MAX_TRACK_NUM];
	BYTE bRowArrayStep5[MAX_TRACK_NUM];	
	BYTE bColArrayStep5[MAX_TRACK_NUM];
	BYTE bReportFingerIdx[MAX_TRACK_NUM];
	BYTE bRow;
	BYTE bCol;
	BYTE bRowStar;
	BYTE bColStar;	
	BYTE bRowTrackNum;
	BYTE bColFoundNum;		
	BYTE bPathCnt;
	WORD wSearchFingerIsTracked;
	#endif

	///----------------------------------------------------------------------------------------------------------------------------///
	/// 	33. Merge Close Finger
	///----------------------------------------------------------------------------------------------------------------------------/// 
	#if defined(FEATURE_OLD_MERGE_CLOSE_CLUSTER_FINGER) || defined(FEATURE_MERGE_CLOSE_NINESQU_FINGER)
	WORD wMergeDist;
	WORD wDividDist;	
	#endif ///< for FEATURE_MERGE_CLOSE_NINESQU_FINGER
	
	#ifdef FEATURE_GG_FILTER_FIX
	WORD wRaw2RawDist;
	BYTE bInterpolationAvg ;
	#endif

	#ifdef FEATURE_ALLPOINT_HEAVY_PRESS_CUT_AVG_ALG
	WORD wAllPointDevAvg;
	#endif

	///----------------------------------------------------------------------------------------------------------------------------///
	///  1T2R Dev Compensation Algorithm
	///----------------------------------------------------------------------------------------------------------------------------///
	#ifdef FEATURE_1T2R_ARCHITECTURE_FARNEAR_REDUCTION
	WORD *p1T2RRawDevSum[2];
	WORD *p1T2RAlgDevSum[2];
	BYTE b1T2RMAEn;

  ///#ifdef FEATURE_1T2R_NEAR_FAR_MA_SHOW_AVG
  short sSumArrEven[MAX_TRACE_NUM];
  short sSumArrOdd[MAX_TRACE_NUM];
  short sAvgEvenTtl;
  short sAvgOddTtl;
  ///#endif

  #if defined(FEATURE_1T2R_NEAR_FAR_COMPEN_1ST_SHOW_AVG) || defined(FEATURE_1T2R_NEAR_FAR_DEV_COMPEN)
  WORD wSumArrEven1st[MAX_TRACE_NUM];
  WORD wSumArrOdd1st[MAX_TRACE_NUM];
  #endif

  #ifdef FEATURE_1T2R_NEAR_FAR_MA_SHOW_MA
  short sMAArrEven[MAX_TRACE_NUM];
  short sMAArrOdd[MAX_TRACE_NUM];
  #endif

	#ifdef FEATURE_1T2R_NEAR_FAR_MA_SHOW_SLOPE
	short sSlopeArr[MAX_TRACE_NUM];
	#endif
  
	#endif

	WORD wTXCtrlBackup[4];

	#ifdef FEATURE_SENSE_DEV_PROCESS_MA_FILTER
	#ifdef FEATURE_SENSE_DEV_PROCESS_MA_FILTER_FINGER_PROTECT_INTEN_CTRL
	BYTE bSenDevProcIntenCtrlAvgTtl;
	BYTE bSenDevProcIntenCtrlAvgLcl;
	#endif
	#endif

  //add for Zetouch mode & Customer mode swtching
	BYTE bWorkingState;

	///----------------------------------------------------------------------------------------------------------------------------///
	///  Border Coordinate Compensation
	///----------------------------------------------------------------------------------------------------------------------------///
#ifdef FEATURE_BORDER_COOR
	WORD wSenseAxisBorderCompNearNum;
	WORD wSenseAxisBorderCompFarNum;
	WORD wSenseAxisBorderCompNearFarDen;
	WORD wDriveAxisBorderCompNearNum;
	WORD wDriveAxisBorderCompFarNum;
	WORD wDriveAxisBorderCompNearFarDen;
	BYTE bSenseAxisInterpolation;
	BYTE bDriveAxisInterpolation;
#endif ///< for FEATURE_BORDER_COOR

	WORD wSenseAxisBorderHitCompNearNum;
	WORD wSenseAxisBorderHitCompFarNum;
	WORD wDriveAxisBorderHitCompNearNum;
	WORD wDriveAxisBorderHitCompFarNum;

	#ifdef FEATURE_WATER_INIT_SCAN
	BYTE bWaterStatus;
	BYTE bWaterDetectCnt;
	BYTE bWaterNoInitCnt;
	WORD wWaterBaseTrackDebounce;
	#endif	
///// Timer status///////////
	BYTE bTimerStatus;
///////////////////////////
}GlobalVarType;

typedef struct GlobalVarSt2
{
#ifdef FEATURE_MERGE_CLOSE_CLUSTER_FINGER
	ClusterTrackCoordinateType  TrackCoordinateRecord[FINGER_MAX_REPORT];	
#endif // FEATURE_MERGE_CLOSE_CLUSTER_FINGER 

#ifdef FEATURE_MERGE_CLOSE_NINESQU_FINGER
	TrackCoordinateType  TrackCoordinateRecord[FINGER_MAX_REPORT];  
	MergeIDCombinationType MergeIDStatus[FINGER_10_COMBINATION_NUMBER];  ///< 45 = C(10,2) = 10!/8!*2!
#endif ///< for FEATURE_MERGE_CLOSE_NINESQU_FINGER

	FingerCoordinateRecordType FingerCoordinateRecord[FINGER_MAX_REPORT];  ///< XDATA[0x119C]

	char *CompileDeviceName;

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    4. Maximum Trace Numbers
  ///----------------------------------------------------------------------------------------------------------------------------///
  BYTE bSenseMax;
  BYTE bDriveMax;
	BYTE bDriveMaxAlg;
	BYTE bSenseMaxAlg;
	
	WORD wDriveAxisInterpolation16;
	WORD wSenseAxisInterpolation16;
#ifdef 	FEATURE_DIFFERENT_LENGTH_PITCH
  BYTE bCoordRemapDriveAxis[32];
  BYTE bCoordRemapSenseAxis[32];
#endif

#ifdef FEATURE_FORCE_TOUCH_SCAN
  BYTE bForceSenseMax;
  BYTE bForceDriveMax;
#endif

  BYTE bIntLowTimeout;                ///< for the int low timeout protection

	BYTE bI2cByteNumber;                ///< I2C Dynamic Byte Nuumber

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    11. Dynamic Data
  ///----------------------------------------------------------------------------------------------------------------------------///
  DynamicDataType dynamicData[REPORT_BUFFER_NUM];

  BYTE pCurReportIdx;

	BYTE bDataIn;                       ///< 1 : threre is at least one finger or one key down ; 0 : No key or finger done ;  2 : One Finger Up Reporting
  BYTE bFingerUpRpCnt;                ///< Finger up report count   

	BYTE bFingerCountsFound;
	BYTE bFingerCountsFoundLast; 
#ifdef FEATURE_FINGER_UP_DEBOUNCE_ALG
	BYTE bFingerReportCounts;
#endif ///< for FEATURE_FINGER_UP_DEBOUNCE_ALG

  WORD wFingerTH;                     ///< Finger search threshold for normal and charger mode
  BYTE bFingerSlop;
#ifdef FEATURE_ADV_NINE_SQAURE
	BYTE bFingerVellySlop;
#endif

  /// Debounce count
  BYTE bDebounceCnt;            ///< For single finger down debounce 
  BYTE bDebounceCntTwo;         ///< For 2 fingers down debounce   
  BYTE bDebounceCntMulti;       ///< For >3 fingers down debounce 

  /// Finger up debounce count
  BYTE bFingerUpMultiDebounceCnt;            ///< For multi finger up debounce count

	/// IIR add one point
#ifdef FEATURE_IIR_FINGER_UP
	BYTE bFingerUpEn;
	BYTE bFingerUpOneDebounceCnt;
#endif

	/// Finger weight
	WORD wFingerWeight;
	WORD wFingerSchmitWeight;
#ifdef FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS
	BYTE bHeavyPressDetectInOutCnt; 
#endif  ///< for FEATURE_INIT_SCAN_DEVDETECT_HEAVY_PRESS

  ///----------------------------------------------------------------------------------------------------------------------------///
  ///    28. Power Base Tracking 
  ///----------------------------------------------------------------------------------------------------------------------------/// 
	BYTE bBaseTrackPeriod;
#ifdef FEATURE_POWER_BASE_TRACK
	WORD wFrameCntUnStableState;
#endif	///< for FEATURE_POWER_BASE_TRACK
#ifdef FEATURE_REK_POWER_BASE_TRACK
	WORD wFrameCntReKUnStableState;
#endif ///< for FEATURE_POWER_BASE_TRACK
#ifdef FEATURE_INIT_ABNORMAL_REK
	WORD wAfterReKCnts;
	WORD wAfterReKNumMax;
	BYTE bAfterReKInitNegTimes;
#endif ///< for FEATURE_INIT_ABNORMAL_REK
#ifdef FEATURE_MUT_ABNORMAL_REK
	BYTE bNegFingerFoundFrameCnt;
	BYTE bNegFingerMutRekDebounce;
#endif

	///----------------------------------------------------------------------------------------------------------------------------///
	///  ReK reference
	///----------------------------------------------------------------------------------------------------------------------------///
	BYTE NFingerCnts ;
	BYTE PFingerCnts ;
	short NFingerMaxDevSum;
	short PFingerMaxDevSum;	
	WORD wContinueReKStopCnt;
	BYTE bReKWithFingerCnt;

	//FrequecnyHopToneType HopTone[MAX_HOP_NUM];
	BYTE bNowToneID;	///< Tone B setup
	BYTE bGoodToneID;	///< Tone B setup
	BYTE bDynamicFreqHopScanCnt;
	BYTE bDynamicFreqHopRoundCnt;
	BYTE bDynamicFreqStatus;
	BYTE bDynamicHistoryCnt;
	BYTE bDynamicCurrentIDBadCnt;
  BYTE bDynamicHopDataOutSel;

#ifdef FEATURE_GESTURE
	BYTE bDirection[8];
	BYTE bGestureId;
	BYTE bGestureStatus;
	BYTE bGestureCount;
	BYTE bMultiFinger;
	BYTE bTimeToDoubleclick;
	BYTE bUpKey;
	BYTE bReportGestureId;
	BYTE bVector[MAX_GESTURE_MAX];
	WORD wGestureCoordX[2];
	WORD wGestureCoordY[2];
	BYTE bXCoordTurn;	
	BYTE bYCoordTurn;
	BYTE bXIncrease;
	BYTE bYIncrease;	
	WORD wXCoordMax;
	WORD wXCoordMin;
	WORD wYCoordMax;
	WORD wYCoordMin;	
#endif

#ifdef FEATURE_NOISE_REJECTION
	BYTE bNoiseRejEn;
#endif

	///----------------------------------------------------------------------------------------------------------------------------///
	/// 	34. Green Mode Counter
	///----------------------------------------------------------------------------------------------------------------------------/// 
	WORD wGreenModeCountVaule;
	WORD wYellowModeCountVaule;
	WORD wGreenModeCounter;
	
	BYTE bTimerTrimCounter;			///< Every IC-Chip different, Low frequence-OSC count 2^12 => X-period , High frequence-OSC count 3000 => Y-period , bTimerTrimCounter = X-period / Y-period
	BYTE bTimerCounter;
}GlobalVarType2;

#ifdef FEATURE_SELF_SCAN
typedef struct PACK SelfScanStructure
{
	BYTE bSelfModeINT;
	BYTE bSelfMode;
	U2   wSelfValue[4];
} SelfScanValue;
#endif

#ifdef FEATURE_TX_RX_IR
typedef struct PACK IRScanStructure
{
  BYTE bIR_ADmode;
  BYTE bIR_ADmode_INT;
	#ifdef FEATURE_IR_BASE
  BYTE bIRBaseRound;
  U2 wIRBase;
 	#endif
} IRScanValue;
#endif

#ifdef FEATURE_CUSTOMER_PROTOCOL

typedef struct PACK I2CCmdStructure
{
  BYTE const code *Cmdkey;
	//BYTE CmdSize;
  void (*CmdHF)(); 
} I2CCmdType;

typedef struct CustomerVarSt
{
	BYTE bINTtriggerCnt;
  WORD wCustomerLastValidPoint;
	BYTE bCustomerLastReportPoint;
	BYTE bCustomerBuf[CUSTOMER_BUF_LEN]; 
#ifdef FEATURE_ZET7101_CUSTOMER_TIMER2_ISR
	BYTE bToggleCnt;
#endif

#ifdef FEATURE_6SP_PROTOCOL
	BYTE bCustomerLastReportFingerCNT;
  BYTE bDoubleBuffer[CUSTOMER_BUF_LEN];
	BYTE bCmdStatus;
	WORD wCustomFramePeriod;	
	BYTE bCustom1ACnt;	
	BYTE bTimeToReport;
	BYTE bReportTouchCnt;
	BYTE bRepeatPoint;

	#ifdef FEATURE_TX_RX_IR	
	BYTE bIRValid;	
	BYTE bIREnable;
	BYTE bIRKeepStatus;
	BYTE bIRFirstDetect;
	BYTE bLcdoff;
	#endif
#else
	BYTE bCustomReportCnt; 
	BYTE bCustomerReSendCNT;	
	BYTE bCustomerLastReportFingerCNT;

	BYTE bDoubleBufferStoreIdx;
	BYTE bDoubleBufferSendIdx;
	BYTE bDoubleBuffer[2][40];
#endif
}CustomerVarType;
#endif // FEATURE_CUSTOMER_PROTOCOL

typedef struct GlobalVarParaSt
{
	BYTE bCmdPara[64];
} GlobalVarParaType;

#endif ///< __TYPEDEF_H__

