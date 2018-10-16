/**
 * @file Vender.h
 *
 *  Vender.h controls the features enabled or disabled
 *
 *
 * @version $Revision: 75 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __VENDER_H__
#define __VENDER_H__

/// =============================================================///
/// Feature area
/// =============================================================///

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 1. INT Pin
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_INT_PIN	  FEATURE_INT_P35

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 2. GPIO
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_GIO_ENABLE                 ///< (DEFAULT : OFF), This feature enables and disable the GPIO control

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 3. Init-Scan 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_INIT_ISR_SCAN
//#define FEATURE_INIT_REGION_SCAN           ///< (DEFAULT : OFF), PN Code 
//#define FEATURE_SNE_AXIS_INIT_SCAN

#if defined(FEATURE_CLUSTER_ENABLE) && defined(FEATURE_1T2R_ARCHITECTURE)
#define FEATURE_WATER_INIT_SCAN
#define FEATURE_WATER_INIT_COORD
#endif

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 4. Mutual-Scan
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_ALLPOINT_SCAN

///#define FEATURE_INITIAL_DEV_PROCESS
#define FEATURE_DRIVE_DEV_PROCESS             ///< (DEFAULT : ON), Drive Dev process 
#define FEATURE_SENSE_DEV_PROCESS             ///< (DEFAULT : ON), Sense Dev process

#ifdef FEATURE_DRIVE_DEV_PROCESS
#define FEATURE_DRIVE_DEV_PROC_SCHMIT_TRIGGER
#define FEATURE_DRIVE_DEV_PROCESS_MA_FILTER
  #ifdef FEATURE_DRIVE_DEV_PROCESS_MA_FILTER
  #define FEATURE_DRIVE_DEV_PROCESS_MA_FILTER_LIMIT_TH
  #define FEATURE_DRIVE_DEV_PROCESS_MA_FILTER_FINGER_PROTECT
  #endif
#endif

#ifdef FEATURE_SENSE_DEV_PROCESS
#define FEATURE_SENSE_DEV_PROCESS_MA_FILTER
  #ifdef FEATURE_SENSE_DEV_PROCESS_MA_FILTER
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_AVG
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_NEW_AVG
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_DIV_BY_3
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_FINGER_PROTECT_ALWAYS_ON
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_NO_DEV_CUT
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_PROC_PRIORITY_CTRL
  #define FEATURE_SENSE_DEV_PROCESS_MA_FILTER_FINGER_PROTECT_INTEN_CTRL
  #endif
#endif

#ifdef FEATURE_INITIAL_DEV_PROCESS
#define FEATURE_INITIAL_DEV_PROCESS_MA_FILTER
#define FEATURE_INITIAL_DEV_PROCESS_BOTH
#define FEATURE_INITIAL_DEV_PROCESS_SCALING
#endif

#define FEATURE_DEVPROC_INCLUDE_NEG
#define FEATURE_DEVCUT_NEG_VAL_NOT_PROC

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 5. Report finger to host
///----------------------------------------------------------------------------------------------- ------------------------------------------------------------///
#define FEATURE_FINGER_DEBOUNCE
#define FEATURE_FINGER_UP_DEBOUNCE_ALG					///< (DEFAULT : ON),  Finger Up Debouncing Alg New
		
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 6. Base Tracking and ReCalibration
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_RECALIBRATION                   ///< (DEFAULT : ON),  This feature enables re-calibration function 
#define FEATURE_BASE_TRACKING                   ///< (DEFAULT : ON),  This feature enables base tracking function 

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 7. Watch Dog
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_WATCH_DOG                                         ///< (DEFAULT : ON),  This feature enables watch dog protection in the system

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 8.  Deep Sleep
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_DEEP_SLEEP                                         ///< (DEFAULT : OFF),  This feature enables DEEP SLEEP function

#ifdef FEATURE_DEEP_SLEEP
  #ifndef FEATURE_CIRCUIT_POWER_DOWN
    #define FEATURE_CIRCUIT_POWER_DOWN 
  #endif ///< for FEATURE_CIRCUIT_POWER_DOWN
  #ifndef FEATURE_WAKE_UP  
    #define FEATURE_WAKE_UP
  #endif ///< for FEATURE_WAKE_UP  
#endif ///< for FEATURE_IDLE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 9.  IDLE
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_IDLE                                               ///< (DEFAULT : OFF),  This feature enables IDLE function

#ifdef FEATURE_IDLE
  #ifndef FEATURE_CIRCUIT_POWER_DOWN
    #define FEATURE_CIRCUIT_POWER_DOWN 
  #endif ///< for FEATURE_CIRCUIT_POWER_DOWN
  #ifndef FEATURE_WAKE_UP  
    #define FEATURE_WAKE_UP
  #endif ///< for FEATURE_WAKE_UP
#endif ///< for FEATURE_IDLE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 10.  Report COORDINATE
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_REPORT_COORDINATE_FINGERS                      ///< (DEFAULT : ON), This feature reports the coordinate finger record
//#define FEATURE_WIN8_DATAFORMAT
#define FEATURE_PROCESS_COORDINATE_WITH_RAW_DATA                   ///< (DEFAULT : ON), This feature enable the Process Coordinate with Raw Data

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 11. Data Format
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_DATA_FORMAT_MUTUAL_SCAN_ADBASE                ///< (DEFAULT : ON), This feature enables the data format Mutual-Scan ADBASE Data
#define FEATURE_DATA_FORMAT_MUTUAL_SCAN_DEV                   ///< (DEFAULT : ON), This feature enables the data format Mutual-Scan Dev Data
//#define FEATURE_MUTUAL_DEV_SHOW_INIT_DEV									  /// show init dev in mutual dev

#define FEATURE_DATA_FORMAT_INIT_SCAN_ADBASE                  ///< (DEFAULT : ON), This feature enables the data format Init-Scan ADBASE Data
#define FEATURE_DATA_FORMAT_INIT_SCAN_DEV                     ///< (DEFAULT : ON), This feature enables the data format Init-Scan Dev Data

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 12.  Finger Search algorithm 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_CLUSTER_ENABLE
#ifdef FEATURE_CLUSTER_ENABLE
#define FEATURE_MERGE_CLOSE_CLUSTER_FINGER
#define FEATURE_CLUSTER_LIMIT_LOCAL_PEAKTH
#define FEATURE_NEW_MERGE_CLOSE_CLUSTER_FINGER

#define FEATURE_CLUSTER_MERGE_V61

#define FEATURE_CLUSTER_MAXPEAK_CHECK
#define FEATURE_CLUSTER_MERGE_LOCAL_PEAK_SORT
#endif

//#define FEATURE_NINE_SQUARE_ENABLE                          ///< (DEFAULT : ON), This feature enables the nine-square algorithm
#ifdef FEATURE_NINE_SQUARE_ENABLE
#define FEATURE_ONE_FINGER_CLUSTER                            ///< (DEFAULT : ON), This feature Enhance One Finger Algorithm
#define FEATURE_SHARP_DEV_CUT                                 ///< (DEFAULT : ON), This feature Enhance sharp dev cut algorithm
#endif

//#define FEATURE_PALM_DETECT                                 ///< (DEFAULT : OFF), This feature enables the palm detection with init-scan

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 13.  Key Function (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_REAL_KEY                                    ///< (DEFAULT : ON), This feature enables real key function
//#define FEATURE_VIRTUAL_KEY                                 ///< (DEFAULT : ON), This feature enables virtual key function

#if defined(FEATURE_REAL_KEY) ||defined(FEATURE_VIRTUAL_KEY)
#define FEATURE_KEY_DATAFORMAT_REPORT_NEW_VER  								///< Control the Report dataformat
#endif

#ifdef FEATURE_REAL_KEY
#define FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_ADBASE            ///< (DEFAULT : ON), This feature enables the data format Key Mutual-Scan ADBASE Data
//#define FEATURE_DATA_FORMAT_KEY_MUTUAL_SCAN_DEV             ///< (DEFAULT : OFF), This feature enables the data format Key Mutual-Scan Dev Data
#define FEATURE_DATA_FORMAT_KEY_DATA                          ///< (DEFAULT : ON), This feature enables the data format Key Mutual-Scan Dev + one valid byte Data
#endif 

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 14.  Heavy press average all
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_HEAVY_PRESS                                 ///< (DEFAULT : OFF), This feature enables heavy press function
//#define FEATURE_MUTUAL_SCAN_DEV_DETECT_HEAVY_PRESS 					///< (DEFAULT : OFF), This feature enables detect heavy press during mutual scan

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 15.  CO-Axis Process
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_CO_AXIS_PROCESS                               ///< (DEFAULT : ON), This feature enables the moving average function at the mutual dev data

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 16.  Power Saving mode
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_POWER_SAVE_MODE                             ///< (DEFAULT : OFF), This feature enable the power save mode to save the normal power from 17mA to 7 mA

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 17.  ADC
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATRUE_USED_LESS_ADC																/// This feature use less ADC (power save but frame rate down)
#define FEATRUE_TURNOFF_UNUSED_ADC                            //Don't turn on the ADCs which are not used in run time scan

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 18.  Finger Schmitter Trigger
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_FINGER_SCHMIT_TRIGGER

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 19  Frequency Hopping
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_DYNAMIC_FREQ_TRAN_MODE_ON
//#define FEATURE_NOISE_DETECTION_MODE
#define FEATURE_HW_FREQUENCY_HOP
#define FEATURE_BOOT_FREQUENCY_HOP
#define FEATURE_DYNAMIC_FREQUENCY_HOP
//#define FEATURE_HW_FREQUENCY_HOP_DEBUG
//#define FEATURE_FREQUENCY_HOP_NOW_TONE_DEBUG
#define FEATURE_PN_VIOLATE_FIX

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 20.  Border compensation algorithm
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#ifdef FEATURE_NINE_SQUARE_ENABLE  
//#define FEATURE_BORDER_DEV                               ///< (DEFAULT : OFF), Border compensation with Dev compensate
//#define FEATURE_BORDER_DYNAMIC_DEV
#endif 

#define FEATURE_BORDER_COOR
#define FEATURE_REV_131
#define FEATURE_BORDER_DEV_INTERNAL 	
//#define FEATURE_QUADRATIC_COMPENSATION

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 21.  Force Finger Up when distance too long
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_FORCE_FINGER_UP_DIST                       ///< (DEFAULT : ON), fix the smooth bug

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 22.  Region Scan
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_REGION_SCAN 															 ///< (DEFAULT : ON), PN Code

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 23. Timer
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_TIMER                                      ///< (DEFAULT : ON), timer

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 24. Dev data Type
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_16BIT_DEV                                     
#ifdef FEATURE_16BIT_DEV
	#define MUTUAL_DATA_TYPE 2
#else ///< for FEATURE_16BIT_DEV
	#define MUTUAL_DATA_TYPE 1
#endif ///< for FEATURE_16BIT_DEV

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 25.  Smoothing Algorithm
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_SMOOTHING_ALGORITHM
#define FEATURE_IIR_FILTER_SMOOTHING                        ///< (DEFAULT : ON), This feature enabled IIR Filter
#define FEATURE_IIR_FILTER_SMOOTHING_RUN_1ST_3PTS
#define FEATURE_IIR_FILTER_SMOOTHING_SCALING                ///< (DEFAULT : ON), This feature enabled IIR Filter scaling for higher SNR
//#define FEATURE_KALMAN_FILTER_SMOOTHING 									///< (DEFAULT : OFF), This feature enabled Kalman Filter
//#define FEATURE_GG_FILTER_SMOOTHING                       ///< (DEFAULT : OFF), This feature enabled GG Filter
//#define FEATURE_GG_FILTER_ENHANCE
//#define FEATURE_GG_FILTER_FIX 
//#define FEATURE_SMOOTHING_DEBUG

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 26. Compensate Mutual Dev with Iitial Dev 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_COMP_MUT_WITH_INIT
//#define FEATURE_COMP_MUT_WITH_INIT_OPTION_EN

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 27.  Power base Track after Reset
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_POWER_BASE_TRACK
#define FEATURE_REK_POWER_BASE_TRACK

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 28.  Coord Edge Limit
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_COORD_EDGE_POSITION_LOCK                                ///< Limit the Report-Coord Max/Min value

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 29.  Gesture
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_GESTURE
#ifdef FEATURE_GESTURE
#define FEATURE_GESTURE_PRO_DBG
#endif

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 30  Merge Close Finger
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#ifdef FEATURE_NINE_SQUARE_ENABLE 
#define FEATURE_MERGE_CLOSE_NINESQU_FINGER
//#define FEATURE_ADV_NINE_SQAURE

#endif
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 31  Switch Drive Sense
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_SWITCH_DRIVE_SENSE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 32  Check Sum 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_CHECK_SUM

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 33  Finger Tracking
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_ONEWAY_COORD_TRACK
#define FEATURE_SEARCH_FINGER_TOUCH_FLAG	
#define FEATURE_TRACK_PREDICT_COORD   ///< if(true) track ID use predic-coord, else use last-report-coord
//#define FEATURE_ADVANCE_COORD_TRACK
//#define FEATURE_ADVANCE_COORD_TRACK_DEBUG

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 34  Drive Axis Initial Large Neg Rek flow
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#ifdef FEATURE_CLUSTER_ENABLE 
#define FEATURE_INIT_ABNORMAL_REK
#define FEATURE_MUT_ABNORMAL_REK	 
//#define FEATURE_MUT_PARTIAL_REK	
//#endif
//#define FEATURE_MUT_PARTIAL_CROSS_SET	 

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 35.  For Axis dev ratio
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_AXIS_DEV_RATIO_ALG

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 36.  Ring Buffer For Dynamic Coordinate Data
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_RING_BUFFER
//#define FEATURE_DEBUG_RING_BUFFER

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 37.  TAP issue
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_AVOID_TAP_LINE
#define FEATURE_AVOID_TAP_LINE_WHEN_FINGER_UP_DEB

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 38. enhance decimation when move < SteadyRange
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_ENHANCE_DECIMA_STADY_RANGE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 39. H type Dev Array Remapping (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_HTYPE_DEV_RE_MAPPING
#ifdef FEATURE_HTYPE_DEV_RE_MAPPING
#define FEATURE_HTYPE_DEV_RE_MAPPING_REV_153 ///< for Modification back : X*(Y-2) back to X*Y
#define FEATURE_HTYPE_DEV_RE_MAPPING_REV_167 ///< for Modification back : X*(Y-2) back to X*Y
#endif
//#define DEBUG_HTYPE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 40. Fixed IIR in border area
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_FIX_IIR_IN_BORDER

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 41. Raw Data Copy At TaskRoundInit
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_RAW_DATA_COPY

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 42. Extend DriveMax to (DriveMax+1)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_MERGE_DEV_ARRAY
//#define FEATURE_MERGE_DEV_ARRAY_DEBUG

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 43. Allpoint Scan Heavy Press Alg : Cut Average Dev
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_ALLPOINT_HEAVY_PRESS_CUT_AVG_ALG ///<  

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 44. Lighting type Dev Array Remapping (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_LTYPE_DEV_RE_MAPPING

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 45.  1T2R TP (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_1T2R_ARCHITECTURE
#ifdef FEATURE_1T2R_ARCHITECTURE
#define FEATURE_1T2R_ARCHITECTURE_DEVPRORCESS
#define FEATURE_1T2R_ARCHITECTURE_CROSSTALK_REDUCTION
#define FEATURE_1T2R_ARCHITECTURE_FARNEAR_REDUCTION
#define FEATURE_1T2R_NEAR_FAR_NEW_EQU
#define FEATURE_1T2R_NEAR_FAR_USING_TTL_AVG
///#define FEATURE_1T2R_NEAR_FAR_LNR_EQU
#define FEATURE_1T2R_NEAR_FAR_MA
#define FEATURE_1T2R_NEAR_FAR_MA_NEW_AVG
#define FEATURE_1T2R_NEAR_FAR_MA_RATIODEV
///#define FEATURE_1T2R_NEAR_FAR_MA_SHOW_AVG
///#define FEATURE_1T2R_NEAR_FAR_MA_SHOW_MA
///#define FEATURE_1T2R_NEAR_FAR_MA_SHOW_SLOPE
///#define FEATURE_1T2R_NEAR_FAR_COMPEN_1ST_SHOW_AVG
#define FEATURE_MA_BORDER_4PTS_3PTS_OVERLAP
///#define FEATURE_1T2R_NEAR_FAR_DEV_COMPEN
#define FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND
#define FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND_BACK_COMPEN_1ST
#define FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND_REV_DIR
#define FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND_ACCURACY_MID_AREA_CORRECT
#define FEATURE_1T2R_NEAR_FAR_DEV_COMPEN_2ND_ACCURACY_MID_AREA_CORRECT_SUBROUTINE

#define FEATURE_1T2R_NO_EDGE
#endif

//#define FEATURE_DEBUG_1T2R_ARCHITECTURE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 46. SRAM READ WRITE
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_SRAM_READ_WRITE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 47. I2C command : F7 Calibration
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_I2C_CMD_F7_ENABLE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 48. WDT enable command : F8 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_I2C_CMD_F8_ENABLE

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 49. Device Name command : FB 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_I2C_CMD_FB_DEVICE_NAME

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 50. Finger Up Report IIR
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_IIR_FINGER_UP
#define FEATURE_IIR_FINGER_UP_FIX_WEIGHTING
#define FEATURE_BORDER_AREA_DOUBLE            ///for IIR debounce up

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 51. ZET7100 Debug Mode
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 52. Sine Table from Dataflash ROM Table or from Firmware Dynamic Gen.
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_SINEGEN_WITH_ROM

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 53. Noise Rejection Fuction with ToneB Dummy Scan, Function actived when single tone work only.
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_NOISE_REJECTION

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 54. PN AD equalizer
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_PN_AD_EQUALIZER

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 55. Finger enhancement for floating
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_FLOATING_FINGER_ENHANCE
//#define FEATURE_FLOATING_FINGER_ENHANCE_MASK_BY_WATER

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 56. Linearity Compensation
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_COORD_OFFSET_SHIFT_16TIMES
#define FEATURE_COORD_GAIN_SHIFT
#define FEATURE_COORD_ROUND_BEFORE_DIV16

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 57. Advance TP information Command
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_ADV_TP_INFOMATION

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 58. INT Pull High Control
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_INT_PULL_HIGH_CONTROL

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 59. CUSTOMER PROTOCOL (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_CUSTOMER_PROTOCOL 
#ifdef FEATURE_CUSTOMER_PROTOCOL
//#define FEATURE_BM_NEW
#define SPI_FLASH_WR_ENABLE //enable flash read write function
//#define SPI_API_TEST
#endif

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 60. Reduce code (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_REDUCE_CODESIZE (Compile Batch Option)
#ifdef FEATURE_REDUCE_CODESIZE
#undef FEATURE_WATER_INIT_SCAN
#undef FEATURE_WATER_INIT_COORD
#undef FEATURE_SRAM_READ_WRITE
#define FEATURE_CUSTOMER_PROTOCOL_REDUCECODE
#undef FEATURE_DATA_FORMAT_INIT_SCAN_ADBASE                  ///< (DEFAULT : ON), This feature enables the data format Init-Scan ADBASE Data
#undef FEATURE_DATA_FORMAT_INIT_SCAN_DEV                     ///< (DEFAULT : ON), This feature enables the data format Init-Scan Dev Data
#endif

#define FEATURE_SCAN_REDUCED
#define FEATURE_PNSCANDATA_REDUCED

//#ifdef FEATURE_PROJECT_7101
//#endif

#ifdef FEATURE_PROJECT_ZET7101
#define FEATURE_7101_DEVELOPMENT
#define FEATURE_SPI_TRANSFER
#ifdef FEATURE_SPI_TRANSFER
//if defined, I2C can't coexist with SPI!!!
//if undef, using Cmd buffer(sharing with I2C cmd buffer)
//#define FEATURE_SPI_CMD_REG_TYPE 
#endif
//////#define FEATURE_DEEP_SLEEP_ESD_TEST

//#define FEATURE_ZET7101_ANA2GPIO
//#define FEATURE_ZET7101_GPIO_ISR
//#define FEATURE_ZET7101_CUSTOMER_TIMER2_ISR

#define FEATURE_DIFFERENT_LENGTH_PITCH

//#define FEATURE_IN_CELL_SCAN
//#define FEATURE_SYNC_REK

#define FEATURE_SELF_SCAN
#ifdef FEATURE_SELF_SCAN
#define FEATURE_SELF_SCAN_DEBUG
#endif

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 61. Force Touch Application (Compile Batch Option)
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#ifdef FEATURE_FORCE_TOUCH
//#define FEATURE_REAL_FORCE_TOUCH
#define FEATURE_6SP_PROTOCOL
#ifdef FEATURE_REAL_FORCE_TOUCH
#define FEATURE_FORCE_TOUCH_SCAN
#endif
#define FEATURE_REPORT_RAW_FORCE
//#define FEATURE_FORCE_TOUCH_DEBUG
//#define FEATURE_FORCE_TOUCH_SCAN_DEBUG
#ifdef FEATURE_6SP_PROTOCOL
#define FEATURE_51V
//#define FEATURE_TX_RX_IR
//#define FEATURE_IR_BASE
#endif
#undef FEATURE_GESTURE
#undef FEATURE_GESTURE_PRO_DBG
#undef FEATURE_DIFFERENT_LENGTH_PITCH
#endif

#ifdef FEATURE_7101_DEVELOPMENT
#undef FEATURE_PN_VIOLATE_FIX
#undef FEATURE_NOISE_DETECTION_MODE
#endif
#endif

#endif ///< __VENDER_H__
