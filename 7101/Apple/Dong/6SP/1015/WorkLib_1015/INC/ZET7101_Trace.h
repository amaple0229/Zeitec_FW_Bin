/**
 * @file Trace.h
 *
 *  Trace.h define the global variables
 *
 *
 * @version $Revision: 32 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __TRACE_H__
#define __TRACE_H__

/// =============================================================///
/// MACRO DEFINITION
/// =============================================================///
#define TW0    									(0x00)
#define TX0    									(0x10)
#define TY0    									(0x20)
#define TS0    									(0x30)

#define TW1    						      (0x00)+(0x40) ///<(0x40)
#define TX1         						(0x10)+(0x40) ///<(0x50)
#define TY1    									(0x20)+(0x40) ///<(0x60)
#define TS1         						(0x30)+(0x40) ///<(0x70)

#define TW2    						      (0x00)+(0x40)+(0x40) ///<(0x80)
#define TX2         						(0x10)+(0x40)+(0x40) ///<(0x90)
#define TY2    									(0x20)+(0x40)+(0x40) ///<(0xA0)
#define TS2         						(0x30)+(0x40)+(0x40) ///<(0xB0)

#define TW3    						      (0x00)+(0x40)+(0x40)+(0x40) ///<(0xC0)
#define TX3         						(0x10)+(0x40)+(0x40)+(0x40) ///<(0xD0)
#define TY3    									(0x20)+(0x40)+(0x40)+(0x40) ///<(0xE0)
#define TS3         						(0x30)+(0x40)+(0x40)+(0x40) ///<(0xF0)


#define PAD00                   (0x00)+(TW2)   // num + 0x80
#define PAD01                   (0x01)+(TW2)   // num + 0x80
#define PAD02                   (0x02)+(TW2)   // num + 0x80
#define PAD03                   (0x03)+(TW2)   // num + 0x80
#define PAD04                   (0x04)+(TW2)   // num + 0x80
#define PAD05                   (0x05)+(TW2)   // num + 0x80
#define PAD06                   (0x06)+(TW2)   // num + 0x80
#define PAD07                   (0x07)+(TW2)   // num + 0x80
#define PAD08                   (0x08)+(TW2)   // num + 0x80
#define PAD09                   (0x09)+(TW2)   // num + 0x80
#define PAD10                   (0x0A)+(TW2)   // num + 0x80
#define PAD11                   (0x0B)+(TW2)   // num + 0x80
#define PAD12                   (0x0C)+(TW2)   // num + 0x80
#define PAD13                   (0x0D)+(TW2)   // num + 0x80
#define PAD14                   (0x0E)+(TW2)   // num + 0x80
#define PAD15                   (0x0F)+(TW2)   // num + 0x80
#define PAD16                   (0x10)+(TW2)   // num + 0x80
#define PAD17                   (0x11)+(TW2)   // num + 0x80
#define PAD18                   (0x12)+(TW2)   // num + 0x80
#define PAD19                   (0x13)+(TW2)   // num + 0x80
#define PAD20                   (0x14)+(TW2)   // num + 0x80
#define PAD21                   (0x15)+(TW2)   // num + 0x80
#define PAD22                   (0x16)+(TW2)   // num + 0x80
#define PAD23                   (0x17)+(TW2)   // num + 0x80
#define PAD24                   (0x18)+(TW2)   // num + 0x80
#define PAD25                   (0x19)+(TW2)   // num + 0x80
#define PAD26                   (0x1A)+(TW2)   // num + 0x80
#define PAD27                   (0x1B)+(TW2)   // num + 0x80
#define PAD28                   (0x1C)+(TW2)   // num + 0x80
#define PAD29                   (0x1D)+(TW2)   // num + 0x80
#define PAD30                   (0x1E)+(TW2)   // num + 0x80
#define PAD31                   (0x1F)+(TW2)   // num + 0x80
#define PAD32                   (0x20)+(TW2)   // num + 0x80
#define PAD33                   (0x21)+(TW2)   // num + 0x80
#define PAD34                   (0x22)+(TW2)   // num + 0x80
#define PAD35                   (0x23)+(TW2)   // num + 0x80
#define PAD36                   (0x24)+(TW2)   // num + 0x80
#define PAD37                   (0x25)+(TW2)   // num + 0x80
#define PAD38                   (0x26)+(TW2)   // num + 0x80
#define PAD39                   (0x27)+(TW2)   // num + 0x80
#define PAD40                   (0x28)+(TW2)   // num + 0x80
#define PAD41                   (0x29)+(TW2)   // num + 0x80
#define PAD42                   (0x2A)+(TW2)   // num + 0x80
#define PAD43                   (0x2B)+(TW2)   // num + 0x80
#define PAD44                   (0x2C)+(TW2)   // num + 0x80
#define PAD45                   (0x2D)+(TW2)   // num + 0x80
#define PAD46                   (0x2E)+(TW2)   // num + 0x80
#define PAD47                   (0x2F)+(TW2)   // num + 0x80
#define PAD48                   (0x30)+(TW2)   // num + 0x80
#define PAD49                   (0x31)+(TW2)   // num + 0x80
#define PAD50                   (0x32)+(TW2)   // num + 0x80
#define PAD51                   (0x33)+(TW2)   // num + 0x80
#define PAD52                   (0x34)+(TW2)   // num + 0x80
#define PAD53                   (0x35)+(TW2)   // num + 0x80
#define PAD54                   (0x36)+(TW2)   // num + 0x80
#define PAD55                   (0x37)+(TW2)   // num + 0x80
#define PAD56                   (0x38)+(TW2)   // num + 0x80
#define PAD57                   (0x39)+(TW2)   // num + 0x80
#define PAD58                   (0x3A)+(TW2)   // num + 0x80
#define PAD59                   (0x3B)+(TW2)   // num + 0x80
#define PAD60                   (0x3C)+(TW2)   // num + 0x80
#define PAD61                   (0x3D)+(TW2)   // num + 0x80
#define PAD62                   (0x3E)+(TW2)   // num + 0x80
#define PAD63                   (0x3F)+(TW2)   // num + 0x80
#define PAD64                   (0x40)+(TW2)   // num + 0x80
#define PAD65                   (0x41)+(TW2)   // num + 0x80
#define PAD66                   (0x42)+(TW2)   // num + 0x80
#define PAD67                   (0x43)+(TW2)   // num + 0x80
#define PAD68                   (0x44)+(TW2)   // num + 0x80
#define PAD69                   (0x45)+(TW2)   // num + 0x80
#define PAD70                   (0x46)+(TW2)   // num + 0x80
#define PAD71                   (0x47)+(TW2)   // num + 0x80
#define PAD72                   (0x48)+(TW2)   // num + 0x80
#define PAD73                   (0x49)+(TW2)   // num + 0x80
#define PAD74                   (0x4A)+(TW2)   // num + 0x80
#define PAD75                   (0x4B)+(TW2)   // num + 0x80
#define PAD76                   (0x4C)+(TW2)   // num + 0x80
#define PAD77                   (0x4D)+(TW2)   // num + 0x80
#define PAD78                   (0x4E)+(TW2)   // num + 0x80
#define PAD79                   (0x4F)+(TW2)   // num + 0x80

///----------------------------------------------------///
/// TRACE MODE
///----------------------------------------------------///
#define ADC15             (15)
#define ADC14             (14)
#define ADC13             (13)
#define ADC12             (12)
#define ADC11             (11)
#define ADC10             (10)
#define ADC9              (9)
#define ADC8              (8)
#define ADC7              (7)
#define ADC6              (6)
#define ADC5              (5)
#define ADC4              (4)
#define ADC3              (3)
#define ADC2              (2)
#define ADC1              (1)
#define ADC0              (0)

#define TRACE_INVALID_ID (0xFF)

///----------------------------------------------------///
/// PN Code Table
///----------------------------------------------------///
#define PN_NUM_1            (1)
#define PN_NUM_3            (3)
#define PN_NUM_4            (4)
#define PN_NUM_5            (5)
#define PN_NUM_7            (7)
#define PN_NUM_9            (9)
#define PN_NUM_11           (11)
#define PN_NUM_13           (13)
#define PN_NUM_15           (15)
#define PN_NUM_17           (17)
#define PN_NUM_19           (19)
#define PN_NUM_21           (21)
#define PN_NUM_31           (31)
#define PN_NUM_ERR          (255)


///-------------------------------------------------------------------------------///
/// Sense TRACE Group
///-------------------------------------------------------------------------------///
#define MAX_ADC_ROUND     (4)  ///<  (CHANNEL_MAX_NUM/AD_MAX_NUM)=3.25 =~ 4 
#define GROUP_MAX_SIZE    (MAX_ADC_ROUND) //((MAX_TRACE_NUM + 1)>> GROUP_NUM) 
#define GROUP_SIZE_FORCE    (1) //((MAX_TRACE_NUM + 1)>> GROUP_NUM) 

#define GROUP_SIZE_KEY    (1) //((MAX_TRACE_NUM + 1)>> GROUP_NUM) 
#define GROUP_INIT_KEY    (0) //((MAX_TRACE_NUM + 1)>> GROUP_NUM) 

#define SENSE_GROUP_INIT	(0) //((MAX_TRACE_NUM + 1)>> GROUP_NUM) 

///-------------------------------------------------------------------------------///
/// Drive TRACE 
///-------------------------------------------------------------------------------///
#define DRIVE_REGION_INIT	(0) //((MAX_TRACE_NUM + 1)>> GROUP_NUM) 

///-------------------------------------------------------------------------------///
/// Region Scan DataType TYPE DEFINITION
///-------------------------------------------------------------------------------///
typedef struct PACK TraceSetupStruct
{
  BYTE bTraceId;  
} TraceSetupType;

typedef struct TraceGroupStruct
{
  BYTE bGroupSize;
  BYTE bPos[MAX_TRACE_NUM];
	BYTE bPad[MAX_TRACE_NUM];
}TraceGroupType;

typedef struct KeyTraceGroupStruct
{
  BYTE bGroupSize;
  BYTE bPos[KEY_MAX_NUM];
	BYTE bPad[KEY_MAX_NUM];
}KeyTraceGroupType;

typedef struct PACK PnDeCodeStruct
{
	BYTE  bPnChipNum;
	DWORD TxPnCode; 
	DWORD PnDecode[4]; 
	BYTE  bPnEqualizer; ///< eqal different PN to same AD/Base : (AD*n)>>3
} PnDeCodeType;




/// =============================================================///
/// VARIABLE DEFINITION
/// =============================================================///
EXTERN BYTE volatile xdata SenseTraceAdcGroupNum;
EXTERN BYTE volatile xdata DriveTraceAdcGroupNum;
EXTERN BYTE xdata PnDriveRoundNum;
EXTERN BYTE volatile xdata PnChipNum;
EXTERN BYTE volatile xdata PnToneNum;
EXTERN BYTE volatile xdata PnChipID;
EXTERN BYTE volatile xdata PnEqualizer;

EXTERN BYTE xdata DriveTraceTableIdx[MAX_TRACE_NUM];
EXTERN BYTE xdata SenseTraceTableIdx[MAX_TRACE_NUM];

#ifdef FEATURE_REAL_KEY
EXTERN BYTE xdata KeyDriveTraceTableIdx[KEY_DRIVE_TRACE_NUM];
EXTERN BYTE xdata KeySenseTraceTableIdx[KEY_MAX_NUM];
EXTERN KeyTraceGroupType xdata KeySenseTraceAdcGroup[1];
#endif

EXTERN TraceGroupType xdata SenseTraceAdcGroup[GROUP_MAX_SIZE];
EXTERN TraceGroupType xdata DriveTraceAdcGroup[GROUP_MAX_SIZE];

#ifdef FEATRUE_TURNOFF_UNUSED_ADC
EXTERN BYTE xdata MaxUsedADCNum;
EXTERN U2 xdata UsedADCBit;
EXTERN BYTE xdata TotalMaxUsedADCNum;
EXTERN U2 xdata TotalUsedADCBit;
#endif

EXTERN unsigned char volatile  * PAD_SEL;

EXTERN unsigned int volatile * SramTableRxSine0;
EXTERN unsigned int volatile * SramTableRxSine1;
EXTERN unsigned int volatile * SramTableTxSine;
EXTERN unsigned int volatile * SramTablePnDemod[2];
EXTERN unsigned int volatile * SramTableRxCoeff;

EXTERN RegionDataType RegionDriveTrace[MAX_PN_DRIVE_ROUND_NUM];
EXTERN PnDeCodeType const code PnDeCodeTable[];

#ifdef FEATURE_FORCE_TOUCH_SCAN
EXTERN BYTE xdata ForcePnDriveRoundNum;
EXTERN BYTE volatile xdata ForceSenseTraceAdcGroupNum;
EXTERN BYTE volatile xdata ForceDriveTraceAdcGroupNum;
EXTERN BYTE volatile xdata ForcePnChipNum;
EXTERN BYTE volatile xdata ForcePnToneNum;
EXTERN BYTE volatile xdata ForcePnChipID;
EXTERN BYTE volatile xdata ForcePnEqualizer;

EXTERN BYTE xdata ForceDriveTraceTableIdx[MAX_FORCE_TRACE_NUM];
EXTERN BYTE xdata ForceSenseTraceTableIdx[MAX_FORCE_TRACE_NUM];

EXTERN TraceGroupType xdata ForceSenseTraceAdcGroup[GROUP_SIZE_FORCE];
EXTERN TraceGroupType xdata ForceDriveTraceAdcGroup[GROUP_SIZE_FORCE];
EXTERN RegionDataType  ForceRegionDriveTrace[1];
#endif



#endif ///< __TRACE_H__


