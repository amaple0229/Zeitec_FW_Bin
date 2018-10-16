/**
 * @file define.h
 *
 *  Define and macro for all the source and header file
 *
 * @version $Revision: 74 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __DEFINE_H__
#define __DEFINE_H__

#include "Revision.h"
#include "SubRevision.h"

///-------------------------------------------------------------------------------///
/// Project Code
///-------------------------------------------------------------------------------///
//#define ROM_PCODE0  (0x11)  ///< Project Code -> 0x11 : ZET7100
//#define ROM_PCODE1  (0x01)  ///< Version Code -> 0x00
//#define ROM_PCODE2  (0x02)  ///< Sub-Version Code
//#define ROM_PCODE3  (B2_CONFIG_IDEV_16_BIT|B2_CONFIG_MDEV_16_BIT)  /// (0x08)  ///< Code Base -> 0x08 : Dynamic Mode

///-------------------------------------------------------------------------------///
/// Architecutre Definition
///-------------------------------------------------------------------------------///
#ifndef ARCH_8051
#define xdata
#define pdata
#define code
#define data
#define PACK  __attribute__ ((__packed__))
#define reentrant 
#define REGISTERS volatile

#else  ///< for ARCH_8051
#define PACK 

#endif ///< for ARCH_8051

///-------------------------------------------------------------------------------///
/// Boolean define
///-------------------------------------------------------------------------------///
#define TRUE        (1)
#define FALSE       (0)
#define NULLSTATE   (2)

///-------------------------------------------------------------------------------///
/// Interrupt define
///-------------------------------------------------------------------------------///
#define INT_RX       (1L<<0)
#define INT_TX       (1L<<1)
#define INT_ADC      (1L<<2)
#define INT_WDT      (1L<<3)
#define INT_TM0      (1L<<4)
#define INT_LVD      (1L<<5)
#ifdef FEATURE_SPI_TRANSFER
#define INT_SPI      (1L<<7)
#define INT_SPI2     (1L<<8)
#else
#define INT_SPI      (0L<<7)
#define INT_SPI2     (0L<<8)
#endif

#ifdef FEATURE_ZET7101_CUSTOMER_TIMER2_ISR
#define INT_TM2      (1L<<9)
#else
#define INT_TM2      (0L<<9)

#endif

#define INT_IR      (0L<<10)

#define INT_SPI_INT_PIN      (1L<<11)
#define INT_TSHD     (1L<<12)
#define INT_TSVD     (1L<<13)
#ifdef FEATURE_ZET7101_GPIO_ISR
#define INT_P2IO     (1L<<14)
#else
#define INT_P2IO     (0L<<14)
#endif
#define INT_P3IO     (1L<<15)



//LM, test
//#define INT_ALL      (INT_RX|INT_TX|INT_ADC|INT_WDT|INT_TM0|INT_IR)

#define INT_ALL      (INT_RX|INT_TX|INT_ADC|INT_WDT|INT_TM0|INT_SPI|INT_SPI2|INT_IR|INT_P2IO|INT_TM2)




///-------------------------------------------------------------------------------///
/// Basic information
///-------------------------------------------------------------------------------///
/// intial stack pointer value for boot code
#define BOOT_SP_INIT_VAL        (0x71)
#define VENDER_NAME_SIZE        (9)
#define SVN_SIZE                (5)

#define SRAM_SIZE         (0x800)

/// Boot code watch dog
#define BOOT_WDT_0_SEC      (0x00)
#define BOOT_WDT_DISABLE    (0xFF)
#define BOOT_WDT_3_SEC      (0x03)
#define BOOT_WDT_5_SEC      (0x05)
#define BOOT_WDT_7_SEC      (0x07)
#define BOOT_WDT_10_SEC     (0x0A)

#define BOOT_WDT_CFG        BOOT_WDT_DISABLE

///-------------------------------------------------------------------------------///
/// XDATA memory mapping
///-------------------------------------------------------------------------------///

///-------------------------------------------------------------------------------///
/// Dummy values
///-------------------------------------------------------------------------------///
#define U1_DUMMY_00             (0x00)
#define U1_DUMMY_AA             (0xAA)
#define U1_DUMMY_BB             (0xBB)
#define U1_DUMMY_CC             (0xCC)
#define U2_DUMMY_0000           (0x0000)
#define U2_DUMMY_FFFF           (0xFFFF)
#define U2_DUMMY_55AA           (0x55aa)
#define U2_DUMMY_BBCC           (0xbbcc)

///-------------------------------------------------------------------------------///
/// BYTE Operation
///-------------------------------------------------------------------------------///
#define LOBYTE(x) (BYTE)((WORD)(x))
#define HIBYTE(x) (BYTE)((WORD)(x)>>8)

///-------------------------------------------------------------------------------///
/// BYTE swap high/low bytes
///-------------------------------------------------------------------------------///
#define LE2BE(x)      (WORD)((((x)&0xff)<< 8) | ((WORD)(x)>>8))

///-------------------------------------------------------------------------------///
/// Register operation
///-------------------------------------------------------------------------------///
#define READ_REG(REG)                   *((BYTE volatile xdata *)(REG))
#define WRITE_REG(REG,VALUE)            *((BYTE volatile xdata *)(REG))=(BYTE)(VALUE)

#define READ_REG16(REG)                 *((U2 volatile xdata *)(REG))
#define WRITE_REG16(REG,VALUE)          *((U2 volatile xdata *)(REG))=(U2)(VALUE)

#define READ_REG32(REG)                 *((U4 volatile xdata *)(REG))
#define WRITE_REG32(REG,VALUE)          *((U4 volatile xdata *)(REG))=(U4)(VALUE)

///-------------------------------------------------------------------------------///
///  XDATA operation
///-------------------------------------------------------------------------------///
#define READ_XDATA32(IDX)               *((U4 volatile xdata *)(IDX))
#define WRITE_XDATA32(IDX,VALUE)        *((U4 volatile xdata *)(IDX))=(U4)(VALUE)


#define XOR_BIT(REG, BIT)               WRITE_REG(REG, READ_REG(REG) ^ (BIT));
#define SET_BIT(REG, BIT)               WRITE_REG(REG, READ_REG(REG) | (BIT));
#define IS_BIT(REG, BIT)                (READ_REG(REG) & (BIT))
#define UNSET_BIT(REG, BIT)             WRITE_REG(REG, (READ_REG(REG) & (BYTE)(~(BIT))));
#define CLR_BIT(REG, BIT)               WRITE_REG(REG, (READ_REG(REG) & (BYTE)(~(BIT))));

#define WRITE_CLEAR(REG, BIT)           WRITE_REG(REG, READ_REG(REG) | (BIT));

#define XOR_BIT32(REG, BIT)             WRITE_REG32(REG, READ_REG32(REG) ^ (BIT));
#define SET_BIT32(REG, BIT)             WRITE_REG32(REG, READ_REG32(REG) | (BIT));
#define UNSET_BIT32(REG, BIT)           WRITE_REG32(REG, (READ_REG32(REG) & (U4)(~(BIT))));
#define CLR_BIT32(REG, BIT)             WRITE_REG32(REG, (READ_REG32(REG) & (U4)(~(BIT))));

#define WRITE_CLEAR32(REG, BIT)         WRITE_REG32(REG, READ_REG32(REG) | (BIT));

#define READ_XDATA(IDX)                 *((BYTE volatile xdata *)(IDX))
#define WRITE_XDATA(IDX,VALUE)          *((BYTE volatile xdata *)(IDX))=(VALUE)

///-------------------------------------------------------------------------------///
/// Interupt Enable
///-------------------------------------------------------------------------------///
#define InterruptEnable()    {__nds32__setgie_en();__nds32__dsb();}
#define InterruptDisable()   {__nds32__setgie_dis(); __nds32__dsb();}

///-------------------------------------------------------------------------------///
/// GPIO Control
///-------------------------------------------------------------------------------///

///------------------------------------------------///
/// For ZET6251 (GIO0 & GIO1)
///------------------------------------------------///
#define GIO0_OUT            (GPIO_PIN0)
#define GIO1_OUT            (GPIO_PIN1)
#define GIO2_OUT            (GPIO_PIN2)

#define REG_IODIR0          (GPIO_IODIR)
#define REG_IODIR1          (GPIO_IODIR)
#define REG_IODIR2          (GPIO_IODIR)

#define GIO0_OUTPUT()				{SET_BIT(REG_IODIR0, GIO0_OUT);}
#define GIO0_INPUT()        {CLR_BIT(REG_IODIR0, GIO0_OUT);}

#define GIO1_OUTPUT()				{SET_BIT(REG_IODIR1, GIO1_OUT);}
#define GIO1_INPUT()        {CLR_BIT(REG_IODIR1, GIO1_OUT);}

#define GIO2_OUTPUT()				{SET_BIT(REG_IODIR2, GIO2_OUT);}
#define GIO2_INPUT()        {CLR_BIT(REG_IODIR2, GIO2_OUT);}

#define ENABLE_GIO()      	{\
	                            GIO0_OUTPUT(); \
	                            GIO1_OUTPUT(); \
	                            GIO2_OUTPUT(); \
	                          }

#ifdef FEATURE_MCU_LIB_ENABLE
#define GIO0_ON()           {SET_BIT(GPIO_PORT, GPIO_PIN0);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->> {GIO0 = 1;}
#define GIO0_OFF()          {CLR_BIT(GPIO_PORT, GPIO_PIN0);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO0 = 0;}
#define GIO1_ON()           {SET_BIT(GPIO_PORT, GPIO_PIN1);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO1 = 1;}
#define GIO1_OFF()          {CLR_BIT(GPIO_PORT, GPIO_PIN1);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO1 = 0;}
#define GIO2_ON()           {SET_BIT(GPIO_PORT, GPIO_PIN2);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO2 = 1;}
#define GIO2_OFF()          {CLR_BIT(GPIO_PORT, GPIO_PIN2);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO2 = 0;}

#define GIO_PURP_ON()       {SET_BIT(GPIO_PORT, GPIO_PIN0);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO2 = 1;}
#define GIO_PURP_OF()       {CLR_BIT(GPIO_PORT, GPIO_PIN0);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO2 = 0;}
#define GIO_BLUE_ON()       {SET_BIT(GPIO_PORT, GPIO_PIN1);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO1 = 1;}
#define GIO_BLUE_OF()       {CLR_BIT(GPIO_PORT, GPIO_PIN1);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO1 = 0;}
#define GIO_GREN_ON()       {SET_BIT(GPIO_PORT, GPIO_PIN2);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO2 = 1;}
#define GIO_GREN_OF()       {CLR_BIT(GPIO_PORT, GPIO_PIN2);} //<<--- nds32le-elf-mculib-v3m  ; nds32le-elf-herlib-v3m -->>{GIO2 = 0;}

///-------------------------------------------------------------------------------///
/// INT  Control
///-------------------------------------------------------------------------------///
#else     ///def FEATURE_HER_LIB_ENABLE
#define GIO0                RegGpio.bit2      /// P32
#define GIO1                RegGpio.bit3      /// P33
#define GIO2                RegGpio.bit4      /// P34
#define INTB                RegGpio.bit5      /// P35

#define GIO0_ON()           {GIO0 = 1;}
#define GIO0_OFF()          {GIO0 = 0;}
#define GIO1_ON()           {GIO1 = 1;}
#define GIO1_OFF()          {GIO1 = 0;}
#define GIO2_ON()           {GIO2 = 1;}
#define GIO2_OFF()          {GIO2 = 0;}
///-------------------------------------------------------------------------------///
/// INT  Control
///-------------------------------------------------------------------------------///
#define 	I2C_INT_LOW()			        {INTB = 0;}
#define 	I2C_INT_HIGH()	          {INTB = 1;}
#define 	I2C_INT()			            (INTB)
#endif

///-------------------------------------------------------------------------------///
/// Tmer delay
///-------------------------------------------------------------------------------///


/// Each _nop_() is 0.125us

#define _nop_()  __asm__ ("nop\n\t")

#define DELAY_1US_NOP()    {\
                             __asm__ ("nop\n\t"); \
                           }

#define DELAY_2US_NOP()    {\
                             DELAY_1US_NOP();\
                             DELAY_1US_NOP();\
                           }
#define DELAY_3US_NOP()    {\
                             DELAY_1US_NOP();\
                             DELAY_2US_NOP();\
                           }
#define DELAY_5US_NOP()    {\
                             DELAY_2US_NOP();\
                             DELAY_3US_NOP();\
                           }
#define DELAY_10US_NOP()   {\
                             DELAY_5US_NOP();\
                             DELAY_5US_NOP();\
                           }

///-------------------------------------------------------------------------------///
/// MCU Timer
///-------------------------------------------------------------------------------///
#define MCU_TIMER0            (0)
#define MCU_TIMER1            (1)

///-------------------------------------------------------------------------------///
/// MCU TIMER
///-------------------------------------------------------------------------------///
#ifdef FEATURE_DEFAULT_SYS_CLK_4M

#define TIMER_PERIOD_250US   (65454)    ///< (65536*400 - 32768)/400 ==> 250us
#define TIMER_PERIOD_500US   (65372)    ///< (65536*200 - 32768)/200 ==> 500us
#define TIMER_PERIOD_1MS     (65209)    ///< (65536*100 - 32768)/100 ==>1ms
#define TIMER_PERIOD_5MS     (63897)    ///< (65536*20 - 32768)/20 ==>5ms
#define TIMER_PERIOD_10MS    (62259)    ///< (65536*10 - 32768)/10 ==>10ms
#define TIMER_PERIOD_20MS    (58983)    ///< (65536*5 - 32768)/5 ==> 20ms
#define TIMER_PERIOD_50MS    (49152)    ///< (65536*2 - 32768)/2 ==> 50ms
#define TIMER_PERIOD_100MS   (32768)    ///<  65536 - 32768 ==>100 ms

#else ///< for FEATURE_DEFAULT_SYS_CLK_4M

#define TIMER_PERIOD_250US   (65372)    ///< 65536 - (65536/400) = 65536 - 164 =  65372 ==> 250 uS
#define TIMER_PERIOD_500US   (65209)    ///< 65536 - (65536/200) = 65536 - 327 =  65209 ==> 500 us
#define TIMER_PERIOD_1MS     (64881)    ///< 65536 - (65536/100) = 65536 - 655 =  64881 ==> 1 ms
#define TIMER_PERIOD_5MS     (62259)    ///< 65536 - (65536/20) = 65536 - 3277 =  62259 ==> 5 ms
#define TIMER_PERIOD_10MS    (58983)    ///< 65536 - (65536/10) = 65536 - 6553 =  58983 ==> 10 ms
#define TIMER_PERIOD_20MS    (52429)    ///< 65536  - (65536/5) = 65536 - 13170= 52429 ==> 20 ms
#define TIMER_PERIOD_50MS    (32768)    ///< 65536 - 32768 = 32768 ==> 50 ms
#define TIMER_PERIOD_100MS   (0)        ///< 65536 - 0 = 65536 ==> 100 ms

#endif ///< for FEATURE_DEFAULT_SYS_CLK_4M

///-------------------------------------------------------------------------------///
/// Watch Dog
///-------------------------------------------------------------------------------///
#define SYSRC500KEnable()      {SET_BIT32(REG32_WDCON, WDTEN);}
#define SYSRC500KDisable()     {CLR_BIT32(REG32_WDCON, WDTEN);}

#define SYSWatchDogEnable()      {SET_BIT32(REG32_WDCON, WDTEN);}
#define SYSWatchDogDisable()     {CLR_BIT32(REG32_WDCON, WDTEN);}

#define SYSWatchDogRSTEnable()       {SET_BIT32(REG32_WDCON, EWT);}
#define SYSWatchDogRSTDisable()       {CLR_BIT32(REG32_WDCON, EWT);}

#define SYSWatchDogClear()       {SET_BIT32(REG32_WDCON, RWT);}

///-------------------------------------------------------------------------------///
/// EVEN / ODD
///-------------------------------------------------------------------------------///
#define EVEN						(0)
#define ODD							(1)

///-------------------------------------------------------------------------------///
/// I2C Address
///-------------------------------------------------------------------------------///

#define I2C_ADDR_76H      (0x76)
#define I2C_ADDR_77H      (0x77)

///-------------------------------------------------------------------------------///
/// GPIO
///-------------------------------------------------------------------------------///

///-------------------------------------------------------------------------------///
/// Data Flash
///-------------------------------------------------------------------------------///
#define DF_PAGE_SIZE                (128)

///-------------------------------------------------------------------------------///
/// System Mode in ZetVar.wSysMode
///-------------------------------------------------------------------------------///
#define SYS_MODE_KEY_SUPPORTED            (0x0001<<0)    ///< KEY supported
#define SYS_MODE_CALIBRATED               (0x0001<<1)    ///< Calibrated
#define SYS_MODE_CHARGER_NOISE_DETECT     (0x0001<<2)    ///< Charger noise detect
#define SYS_MODE_DEEP_SLEEP               (0x0001<<3)    ///< Deep Sleep
#define SYS_MODE_IDLE                     (0x0001<<4)    ///< IDLE
#define SYS_MODE_FINGER_REPORT            (0x0001<<5)    ///< Report Finger
#define SYS_MODE_CHARGER_MODE             (0x0001<<6)    ///< Charger Mode Enabled
#define SYS_MODE_INIT_SCAN_EN_DRI_AXIS    (0x0001<<8)    ///< Init Scan Enable
#ifdef FEATURE_CO_AXIS_PROCESS
#define SYS_MODE_SEN_COAXIS               (0x0001<<9)    ///< COAXIS
#define SYS_MODE_SEN_COAXIS_LAST          (0x0001<<10)   ///< COAXIS
#endif
#define SYS_MODE_FINGER_TOUCHED           (0x0001<<11)   ///< Finger touched
#define SYS_MODE_INIT_SCAN_EN_SEN_AXIS    (0x0001<<14)   ///< Sense Axis Init Scan Enable
#define SYS_MODE_TP_TEST_EN               (0x0001<<15)   ///< Enter Test Mode

#define SYS_MODE_REPORT_ENHANCE_EN_PERIOD  (10)

/// INIT_SCAN_EN option
#define DRIVE_AXIS_INIT_SCAN              (0x01<<0)    ///< Drive Axis Init Scan Enable
#define SENSE_AXIS_INIT_SCAN              (0x01<<1)    ///< Sense Axis Init Scan Enable
#define DRIVE_AXIS_INIT_PN_SCAN           (0x01<<2)    ///< Drive Axis Init PNcode Scan Enable
#define WATER_MODE_BASE_TEACK_EN          (0x01<<3)    ///< 
#define WATER_MODE_COORD_BY_INIT    	    (0x01<<4)    ///< 

///-------------------------------------------------------------------------------///
/// System Mode in ZetVar.wSysMode2
///-------------------------------------------------------------------------------///
#define SYS_MODE2_RAW_MUTUAL_DEV            	(0x0001<<0)  ///<	Mutual Raw Dev
#define SYS_MODE2_ROUND_RUNNING             	(0x0001<<1)  ///<	Round done
#define SYS_MODE2_YELLOW_MODE      						(0x0001<<2)  ///< balance mode
#define SYS_MODE2_GREEN_MODE       						(0x0001<<3)  ///< power saving mode
#define SYS_MODE2_RED_MODE       							(0x0001<<4)  ///< performance mode
#define SYS_MODE2_INIT_PN_SCAN_EN       			(0x0001<<5)  ///< 
#define SYS_MODE2_TOUCH_TRIG_SCAN							(0x0001<<6)  ///< 
#define SYS_MODE2_TX_MUTUAL_DEV_MODE        	(0x0001<<7)  ///<	Mutual Dev TX mode
#define SYS_MODE2_PAD_REPEAT_VIOLATE        	(0x0001<<7)  ///<	Mutual Dev TX mode
#define SYS_MODE2_ZET7100_FORCE_EVENODD_SCAN	(0x0001<<8)  ///<  

///-------------------------------------------------------------------------------///
/// Debug Mode Ctrl
///-------------------------------------------------------------------------------///
#define DEBUG_SWITCH_CTRL1            (0x0001<<0)

///-------------------------------------------------------------------------------///
/// System Test Mode
///-------------------------------------------------------------------------------///
#define TP_TEST_FPC_OPEN_EN       (0x0001<<0)     ///< TPCMD 0xA0 
#define TP_TEST_FPC_SHORT_EN      (0x0001<<1)     ///< TPCMD 0xA1
#define TP_TEST_ITO_SENSOR        (0x0001<<2)     ///< 
#define TP_TEST_ITO_SENSOR_SWITCH (0x0001<<3)     ///< 
#define TP_TEST_KEY_AD_EN         (0x0001<<4)

#define TP_TEST_NONE              (0x0000)

///-------------------------------------------------------------------------------///
/// Init Zeta FPC Open / Short
///-------------------------------------------------------------------------------///
#define INIT_ZETA_SCAN_NORMAL                    (0)
#define INIT_ZETA_SCAN_STRONG                    (1)
#define INIT_ZETA_DRIVE_STRONG_VAL							 (DSMVREF_AD_DYNAMIC_RANGE3)
#define INIT_ZETA_SENSE_STRONG_VAL							 (DSMVREF_AD_DYNAMIC_RANGE3)

///-------------------------------------------------------------------------------///
/// Calibration
///-------------------------------------------------------------------------------///
#define BASE_SUB_DEV  							(0x00)
#define BASE_ADD_DEV  							(0x01)

#define BASE_CHECK_EVERY_ROUND  		(0x00)					
#define BASE_CHECK_PERIOD_ROUND  		(0x01)

#define BASE_RECAL_ANYWAY				 		(0x00)  ///< ReK even if Finger is touched
#define BASE_RECAL_WHEN_NO_TOUCH 		(0x01)					

#define BASE_TRACK_PERIOD						(20)          
#define BASE_RECAL_CHECK_PERIOD			(23)

///-------------------------------------------------------------------------------///
/// Calibration Control
///-------------------------------------------------------------------------------///
#define BASE_EN_TRACKINGBASE    						(0x01<<0)     ///< Tracking Base Enable
#define BASE_RE_CALIBRATION     		        (0x01<<1)     ///< Re-Calibration Enable
#define BASE_CMD_RE_CALIBRATION							(0x01<<2)     ///< I2C command Re-Calibration Enable
#define BASE_TRACK_RECALIBRAION_TOTAL_CTRL	(0x01<<3)     ///< Re-Calibration / Base-Tracking total Ctrl , 0: disable 1:enable
#ifdef FEATURE_SYNC_REK
#define BASE_SYNC_REK_MODE                  (0x01<<4)     ///< Re-Calibration In Scan Duration(for In-cell scan), 0: disable 1:enable
#endif
///-------------------------------------------------------------------------------///
/// Charger Mode Dataflash Control
///-------------------------------------------------------------------------------///
#define SYS_CHG_REK_ENABLE    						(0x01<<0)     ///< Dataflash control charger mode enable/disable Rek or Not

///-------------------------------------------------------------------------------///
/// Scan Method Control
///-------------------------------------------------------------------------------///
#define METHOD_IDLE                 		(0<<0)
#define METHOD_INIT_SCAN            		(1<<0)
#define METHOD_MUTUAL_SCAN          		(1<<1)
#define METHOD_KEY_SCAN          				(1<<2)
#define METHOD_FORCE_SCAN          			(1<<2)
#define METHOD_INIT_ZETA_DRIVE_SCAN   	(1<<3)
#define METHOD_INIT_ZETA_SENSE_SCAN   	(1<<4)
#define METHOD_SCAN_FINISH 							(1<<5)
#define METHOD_REGION_SCAN          		(1<<7)

///-------------------------------------------------------------------------------///
/// Scan AD Control
///-------------------------------------------------------------------------------///
#define AD_ADC1_ON                  (0x0F)
#define AD_ADC1_OFF                 (0xF0)
#define AD_ADC2_OFF 								(0xFF)
#define AD_ADC3_OFF                 (0xFF)
#define AD_ALL_POWER_OFF           	(0<<0)
#define AD_INIT_DRIVE_SCAN          (1<<0)
#define AD_MUTUAL_SCAN           		(1<<1)
#define AD_INIT_SENSE_SCAN          (1<<2)
#define AD_MUTUAL_REV_SCAN          (1<<3)
#define AD_MUTUAL_KEY_SCAN          (1<<4)
#define AD_ALL_POWER_ON            	(1<<5)

///-------------------------------------------------------------------------------///
/// MUTUAL SCAN
///-------------------------------------------------------------------------------///
#define MUTUAL_SCAN_NONE                     							(0x00)
#define MUTUAL_SCAN_FOUND                    							(0x01)
#define MUTUAL_SCAN_FOUND_INIT               							(0xAA)
#define MUTUAL_SCAN_FOUND_DONE               							(0xBB)

///-------------------------------------------------------------------------------///
/// Mutual Scan Change Trace Value 
///-------------------------------------------------------------------------------///
#define MUTUAL_SCAN_DRIVE_SENSE_LEVEL0               			(0)  ///< Normal Use or Zeta Use(Non Sense Tie GND)
#define MUTUAL_SCAN_DRIVE_SENSE_LEVEL1              	 		(1)  ///< Zeta Use(Non Sense Tie to Floating)
#define MUTUAL_SCAN_DRIVE_SENSE_LEVEL2               			(2)  ///< nearby traces tied to ZEKI setup, and drived by VDD 

#define MUTUAL_SCAN_DRIVE_TRACE_VAL1       								(0x03) 
#define MUTUAL_SCAN_DRIVE_TRACE_VAL2       								(0x30)
#define MUTUAL_SCAN_DRIVE_TRACE_CHANGE_VAL1_FLOATING 			(0x04) 
#define MUTUAL_SCAN_DRIVE_TRACE_CHANGE_VAL2_FLOATING 			(0x40)
#define MUTUAL_SCAN_REV_SENSE_TRACE_VAL1       						(0x05) 
#define MUTUAL_SCAN_REV_SENSE_TRACE_VAL2       						(0x50) 
#define MUTUAL_SCAN_REV_SENSE_TRACE_CHANGE_VAL1_GND    		(0x35) 
#define MUTUAL_SCAN_REV_SENSE_TRACE_CHANGE_VAL2_GND    		(0x53)
#define MUTUAL_SCAN_REV_SENSE_TRACE_CHANGE_VAL1_FLOATING 	(0x45) 
#define MUTUAL_SCAN_REV_SENSE_TRACE_CHANGE_VAL2_FLOATING 	(0x54)

///-------------------------------------------------------------------------------///
/// MUTUAL SCAN TYPE
///-------------------------------------------------------------------------------///
#ifdef FEATURE_SCAN_REDUCED
#define MUTUAL_BASE 0x00
#define MUTUAL_BASE_SET_ZERO 0x01
#define MUTUAL_BASE_ACCUM 0x02
#define MUTUAL_DEV 0x05
#endif
///-------------------------------------------------------------------------------///
/// Key
///-------------------------------------------------------------------------------///
#define KEY_AT_EVEN_TRACE           (0x01)
#define KEY_AT_ODD_TRACE            (0x10)
#define KEY_AT_EVEN_ODD_TRACE       (0x11)

#define KEY_BASE_MODE								(0x00)
#define KEY_DV_MODE									(0x01)

#define KEY_SENSE_AT_TX       (1<<0)
#define KEY_SENSE_AT_TY       (1<<1)

///-------------------------------------------------------------------------------///
/// I2C INT LOW Time out
///-------------------------------------------------------------------------------///
#define I2C_INT_LOW_TIMEOUT                   (0x400)
#define I2C_INT_DUMMY_REPORT_TIMEOUT_CNT      (3)    ///< The INT_LOW timeout loop count

///-------------------------------------------------------------------------------///
/// Transfer Type
///-------------------------------------------------------------------------------///
#define TRAN_TYPE_B2_MODE           (1<<0)      ///< B2 Command
#define TRAN_TYPE_NORMAL_MODE       (1<<1)      ///< normal mode  (Dynamic Mode)
#define TRAN_TYPE_MUTUAL_DV_MODE    (1<<2)      ///< DV Data of Mutual Scan
#define TRAN_TYPE_MUTUAL_AD_MODE    (1<<3)      ///< ADC Data of Mutual Scan
#define TRAN_TYPE_INIT_DV_MODE      (1<<4)      ///< DV Data of Init Scan
#define TRAN_TYPE_INIT_AD_MODE      (1<<5)      ///< ADC Data of Init Scan
#define TRAN_TYPE_FOCAL_MODE        (1<<6)      ///< normal mode  (Dynamic Mode)
#define TRAN_TYPE_MTK_MODE          (1<<7)      ///< MTK Data Mode

///  B2 : Transfer mode
#define B2_MODE_INIT_BASE           (1<<0)
#define B2_MODE_INIT_DEV            (1<<1)
#define B2_MODE_MUTUAL_BASE         (1<<2)
#define B2_MODE_MUTUAL_DEV          (1<<3)
#define B2_MODE_COORDINATE          (1<<6)

#define B2_BYTE12_KEY_SUPPORTED     (1<<7)  ///< for the B2Cmd[12]

#define B2_BYTE16_FUNC_PHONE        (1<<0)  ///< Phone Type
#define B2_BYTE16_FUNC_TABLET       (1<<1)  ///< Tablet Type
#define B2_BYTE16_RESERVE_BIT2      (1<<2)
#define B2_BYTE16_XY_EXCHANGE       (1<<3)  ///< for the B2Cmd[15]
#define B2_BYTE16_FUNC_1T2R_TP      (1<<4)
#define B2_BYTE16_XY_ORG0  		      (1<<5)
#define B2_BYTE16_XY_ORG1     		  (1<<6)
#define B2_BYTE16_ADV_INFO					(1<<7)

///-------------------------------------------------------------------------------///
/// B2 BYTE[3]
///-------------------------------------------------------------------------------///
#define B2_CONFIG_INT_FREE                 (1<<0)
#define B2_CONFIG_INT                      (0<<0)

#define B2_CONFIG_MDEV_16_BIT              (1<<1)
#define B2_CONFIG_MDEV_8_BIT               (0<<1)

#define B2_CONFIG_IDEV_16_BIT              (1<<2)
#define B2_CONFIG_IDEV_8_BIT               (0<<2)

#define B2_CONFIG_8051                     (1<<3)
#define B2_CONFIG_32_BIT_MCU               (0<<3)

#define B2_CONFIG_SOFT_MBASE               (1<<4)
#define B2_CONFIG_FW_MBASE                 (0<<4)

#define B2_CONFIG_HEADER_3B                (1<<5)

#define B2_CONFIG_BIG_ENDIAN               (1<<6)
#define B2_CONFIG_LITTLE_ENDIAN            (0<<6)

///-------------------------------------------------------------------------------///
///  Windows 8 Dataformat
///-------------------------------------------------------------------------------///
/// Dynamic Dataformat ZByte
/// BYTE[0]  = 0x3C
/// BYTE[1]  = VALID_HIGH_BYTE
/// BYTE[2]  = VALID_LOW_BYTE
/// BYTE[3]  = X0_HIGH + Y0_HIGH
/// BYTE[4]  = X0_LOW
/// BYTE[5]  = Y0_LOW
/// BYTE[7]  = X1_HIGH + Y1_HIGH
/// BYTE[8]  = X1_LOW
/// BYTE[9]  = Y1_LOW
/// BYTE[10] = Z1.7 = CHARGER_MODE_FLAG ,  Z1.4 =WATER_MODE_FLAG
/// BYTE[11] = X1_HIGH + Y1_HIGH
/// BYTE[12] = X2_LOW
/// BYTE[13] = Y2_LOW
/// BYTE[14] = Z2 (ªÅ)
/// ........
///-------------------------------------------------------------------------------///
#define ZBYTE_CHARGER_ON          (1<<7)  ///< BYTE[10].7
#define ZBYTE_WATER_ON            (1<<4)  ///< BYTE[10].4

#define ZBYTE_PUT_DOWN            (0x00)  ///< all ZBYTE.[1:0]
#define ZBYTE_PUT_UP		          (0x01)  ///< all ZBYTE.[1:0]
#define ZBYTE_CONTACT  	          (0x02)  ///< all ZBYTE.[1:0]
#define ZBYTE_VALID_BIT	          (0x03)  ///< all ZBYTE.[1:0]

///-------------------------------------------------------------------------------///
/// TP Command
///-------------------------------------------------------------------------------///
///------------------------------------------------///
/// OPCODE = 0x80 : SRAM WRITE Hi-Nibble
///------------------------------------------------///
#define TPCMD_80_SRAM_ACCESS        (0x80)   ///<

///------------------------------------------------/// 
/// OPCODE = 0x90 : SRAM READ cmd 
///------------------------------------------------/// 
#define TPCMD_90_SRAM_READ        (0x90)   ///< 
#define TP_CMD_90_CMD_LEN         (1) 
#define TP_CMD_90_TX_DATA_LEN      (50)     ///< A temperary setting, the real number shall be decided in the TPTest function 
#define TP_CMD_90_RX_DATA_LEN      (0) 

///------------------------------------------------///
/// OPCODE = 0xA0 : FPC-Short
///------------------------------------------------///
#define TPCMD_A0_INIT_SCAN_AD        (0xA0)   ///< FPC-Short  (ADBASE Data) : Real Time Init Scan A/D (16 bits)
  #define TP_CMD_A0_CMD_LEN          (1)
  #define TP_CMD_A0_TX_DATA_LEN      (32)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_A0_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xA1 : FPC-Open
///------------------------------------------------///
#define TPCMD_A1_INIT_SCAN_DV        (0xA1)   ///< FPC-Open  (DEV Data) : Real Time Init Scan  DV (8 bits)
  #define TP_CMD_A1_CMD_LEN          (1)
  #define TP_CMD_A1_TX_DATA_LEN      (32)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_A1_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xA2 : Exit Test Mode
///------------------------------------------------///
#define TPCMD_A2_RELEASE_TEST        (0xA2)   ///< Exit Test Mode
  #define TP_CMD_A2_CMD_LEN          (1)
  #define TP_CMD_A2_TX_DATA_LEN      (0)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_A2_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xA3 : ITO Sensor Test
///------------------------------------------------///
#define TPCMD_A3_INIT_SCAN_DV        (0xA3)   ///< ITO Sensor Test  (ADBASE Data) : Real Time Init Scan  DV (8 bits)
  #define TP_CMD_A3_CMD_LEN          (1)
  #define TP_CMD_A3_TX_DATA_LEN      (32)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_A3_RX_DATA_LEN      (0)    

///------------------------------------------------///
/// OPCODE = 0xA4 : Mutual Dev Mode
///------------------------------------------------///
#define TPCMD_A4_MUTUAL_DEV          (0xA4)   ///< Mutual Dev Mode (DEV Data) : Real Time Mutual Scan  DV (8 bits)
  #define TP_CMD_A4_CMD_LEN          (1)
  #define TP_CMD_A4_TX_DATA_LEN      (0)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_A4_RX_DATA_LEN      (0)     

///------------------------------------------------///
/// OPCODE = 0xA5 : ITO Sensor Direction Reversed
///------------------------------------------------///
#define TPCMD_A5_MUTUTAL_PIN_SWITCH  (0xA5)   ///< Mutual-Scan DEV drive axis sense and sense axis drive
  #define TP_CMD_A5_CMD_LEN          (1)
  #define TP_CMD_A5_TX_DATA_LEN      (0)     
  #define TP_CMD_A5_RX_DATA_LEN      (0)   
	
///------------------------------------------------///
/// OPCODE = 0xAA : FPC-Open Enhance
///------------------------------------------------///
#define TPCMD_AA_INIT_ENHANCE_SCAN_DV (0xAA)   ///< FPC-Open Enhance (DEV Data) : Real Time Init Scan  DV (8 bits)
  #define AA_BUF_LEN                  (0)
	#define TP_CMD_AA_CMD_LEN           (2)
  #define TP_CMD_AA_TX_DATA_LEN       (AA_BUF_LEN)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_AA_RX_DATA_LEN       (0)  

///------------------------------------------------///
/// OPCODE = 0xAB : Key Base AD
///------------------------------------------------///
#define TPCMD_AB_KEY_SCAN_AD         (0xAB)   ///< Key Base AD : Real Time KEY Scan A/D (16 bits)
  #define TP_CMD_AB_CMD_LEN          (1)
  #define TP_CMD_AB_TX_DATA_LEN      (32)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_AB_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xAC : Raw Dev
///------------------------------------------------///
#define TPCMD_AC_RAW_DEV             (0xAC)   ///< Mutual Dev with RawDev
  #define TP_CMD_AC_CMD_LEN          (1)
  #define TP_CMD_AC_TX_DATA_LEN      (0)     ///< A temperary setting, the real number shall be decided in the TPTest function
  #define TP_CMD_AC_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xB0 : Software Reset
///------------------------------------------------///
#define TPCMD_B0_RESET             (0xB0)     ///< Reset device with the WDT
  #define B0_BUF_LEN                 (0) 
  #define TP_CMD_B0_CMD_LEN          (1)
  #define TP_CMD_B0_TX_DATA_LEN      (B0_BUF_LEN)
  #define TP_CMD_B0_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xB1 : Deep Sleep
///------------------------------------------------///
#define TPCMD_B1_DEEP_SLEEP        (0xB1)     ///< Enter deep sleep mode, host can wake up by wake-up  command (0xB4) . The power consumption is about 35uA
  #define B1_BUF_LEN                 (0) 
  #define TP_CMD_B1_CMD_LEN          (1)
  #define TP_CMD_B1_TX_DATA_LEN      (B1_BUF_LEN)
  #define TP_CMD_B1_RX_DATA_LEN      (0) 


///------------------------------------------------///
/// OPCODE = 0xB2 : Panel Information
///------------------------------------------------///
#define TPCMD_B2_PANEL_INFO         (0xB2)    ///< Return the panel information
  #define B2_BUF_LEN                 (17) 
  #define TP_CMD_B2_CMD_LEN          (1)
  #define TP_CMD_B2_TX_DATA_LEN      (B2_BUF_LEN)
  #define TP_CMD_B2_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xB3 : Idle 
///------------------------------------------------///
#define TPCMD_B3_IDLE               (0xB3)   ///< Enter deep sleep mode, host can wake up by wake-up  command (0xB4) . The power consumption is about 35uA
  #define B3_BUF_LEN                 (0) 
  #define TP_CMD_B3_CMD_LEN          (1)
  #define TP_CMD_B3_TX_DATA_LEN      (B3_BUF_LEN)
  #define TP_CMD_B3_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xB4 : Wake up the system from power mode
///------------------------------------------------///
#define TPCMD_B4_WAKE               (0xB4)   ///< Enter deep sleep mode, host can wake up by wake-up  command (0xB4) . The power consumption is about 35uA
  #define B4_BUF_LEN                 (0) 
  #define TP_CMD_B4_CMD_LEN          (1)
  #define TP_CMD_B4_TX_DATA_LEN      (B4_BUF_LEN)
  #define TP_CMD_B4_RX_DATA_LEN      (0)    

///------------------------------------------------///
/// OPCODE = 0xB5 : Charger Mode Enable
///------------------------------------------------///
#define TPCMD_B5_CHARGER_MODE_ENABLE   (0xB5)   ///< Enable charger mode Enable
  #define B5_BUF_LEN                 (0) 
  #define TP_CMD_B5_CMD_LEN          (1)
  #define TP_CMD_B5_TX_DATA_LEN      (B5_BUF_LEN)
  #define TP_CMD_B5_RX_DATA_LEN      (0)    

///------------------------------------------------///
/// OPCODE = 0xB6 : Charger Mode Disable
///------------------------------------------------///
#define TPCMD_B5_CHARGER_MODE_DISABLE  (0xB6)   ///< Enable charger mode Disable
  #define B6_BUF_LEN                 (0) 
  #define TP_CMD_B6_CMD_LEN          (1)
  #define TP_CMD_B6_TX_DATA_LEN      (B6_BUF_LEN)
  #define TP_CMD_B6_RX_DATA_LEN      (0)    

	
///------------------------------------------------///
/// OPCODE = 0xB7 : Deep Sleep for Gesture
///------------------------------------------------///
#define TPCMD_B7_DEEP_SLEEP_GESTURE    (0xB7)     ///< Enter deep sleep mode, host can wake up by wake-up  command (0xB4) . The power consumption is about 35uA
  #define B7_BUF_LEN                 (0) 
  #define TP_CMD_B7_CMD_LEN          (1)
  #define TP_CMD_B7_TX_DATA_LEN      (B7_BUF_LEN)
  #define TP_CMD_B7_RX_DATA_LEN      (0) 


///------------------------------------------------///
/// OPCODE = 0xB8 : Check Sum Report
///------------------------------------------------///
#define TPCMD_B8_CHECKSUM_READ  (0xB8)   ///< Enable charger mode Disable
  #define B8_BUF_LEN                 (0) 
  #define TP_CMD_B8_CMD_LEN          (1)
  #define TP_CMD_B8_TX_DATA_LEN      (B8_BUF_LEN)
  #define TP_CMD_B8_RX_DATA_LEN      (0)    


///------------------------------------------------///
/// OPCODE = 0xC0 : Set the ADC_CTRL0 Command
///------------------------------------------------///
#define TPCMD_C0_ADC_CTRL0_SET     (0xC0)  ///< SRAM Write
  #define C0_BUF_LEN                 (0)
  #define TP_CMD_C0_CMD_LEN          (2)
  #define TP_CMD_C0_TX_DATA_LEN      (C0_BUF_LEN)
  #define TP_CMD_C0_RX_DATA_LEN      (0) 

///------------------------------------------------///
/// OPCODE = 0xC1 : Set the C1 Vender Command
///------------------------------------------------///
#define TPCMD_C1_VEN_CMD_SET       (0xC1)  ///< SRAM Write
  #define C1_BUF_LEN                 (0)
  #define TP_CMD_C1_CMD_LEN          (10)  ///Address 1 byte, Mode 1 byte, Data 8 bytes
  #define TP_CMD_C1_TX_DATA_LEN      (C1_BUF_LEN)
  #define TP_CMD_C1_RX_DATA_LEN      (0)  	

///------------------------------------------------///
/// OPCODE = 0xF1 : SRAM Read Command  (8051 ONLY)
///------------------------------------------------///
#define TPCMD_F1_SRAM_READ        (0xF1)  ///< SRAM Read
  #define F1_BUF_LEN                 (64)
  #define TP_CMD_F1_CMD_LEN          (5)
  #define TP_CMD_F1_TX_DATA_LEN      (F1_BUF_LEN)
  #define TP_CMD_F1_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xF2 : SRAM Write Command (8051 ONLY)
///------------------------------------------------///
#define TPCMD_F2_SRAM_WRITE     (0xF2)      ///< SRAM Write
  #define F2_BUF_LEN                 (0)
  #define TP_CMD_F2_CMD_LEN          (5)    ///< F1, ADRR[0..1], DATA[0]
  #define TP_CMD_F2_TX_DATA_LEN      (F2_BUF_LEN)
  #define TP_CMD_F2_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xF3 : SRAM32 Read Command  (ARM ONLY)
///------------------------------------------------///
#define TPCMD_F3_SRAM32_READ       (0xF3)   ///< SRAM32 Read
  #define F3_BUF_LEN                 (64)
  #define TP_CMD_F3_CMD_LEN          (7)    ///< F3, ADDR[0..3], LEN[0..1] --> 7 bytes
  #define TP_CMD_F3_TX_DATA_LEN      (F1_BUF_LEN)
  #define TP_CMD_F3_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xF4 : SRA32M Write Command  (ARM ONLY)
///------------------------------------------------///
#define TPCMD_F4_SRAM32_WRITE     (0xF4)  ///< SRAM32 Write
  #define F4_BUF_LEN                 (0)
  #define TP_CMD_F4_CMD_LEN          (6)  ///< F4, ADDR[0..3], DATA[0]  --> 6 bytes
  #define TP_CMD_F4_TX_DATA_LEN      (F2_BUF_LEN)
  #define TP_CMD_F4_RX_DATA_LEN      (0)   

///------------------------------------------------///
/// OPCODE = 0xF7 : Force Re-Calibration Commnad
///------------------------------------------------///
#define TPCMD_F7_CALIBRATE         (0xF7)  ///< Re-calibration
  #define F7_BUF_LEN                 (0)  
  #define TP_CMD_F7_CMD_LEN          (1)
  #define TP_CMD_F7_TX_DATA_LEN      (F1_BUF_LEN)
  #define TP_CMD_F7_RX_DATA_LEN      (0)

///------------------------------------------------///
/// OPCODE = 0xF8 : Watch Dog Enable / Disable Command
///------------------------------------------------///
#define TPCMD_F8_WDT               (0xF8)  ///< 
  #define F8_BUF_LEN                 (0)  
  #define TP_CMD_F8_CMD_LEN          (2)
  #define TP_CMD_F8_TX_DATA_LEN      (0)
  #define TP_CMD_F8_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xF9 : Revision Number
///------------------------------------------------///
#define TPCMD_F9_REVISION_READ     (0xF9)  ///< SRAM Read
  #define F9_BUF_LEN                 (4)
  #define TP_CMD_F9_CMD_LEN          (1)
  #define TP_CMD_F9_TX_DATA_LEN      (F9_BUF_LEN)
  #define TP_CMD_F9_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xFA : Re-Calibration / Base-Tracking Disable/Enable
///------------------------------------------------///
#define TPCMD_FA_CALIB_BASETRACK_CTRL   (0xFA)  ///< 
  #define FA_BUF_LEN                 (0)  
  #define TP_CMD_FA_CMD_LEN          (2)
  #define TP_CMD_FA_TX_DATA_LEN      (0)
  #define TP_CMD_FA_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xFB : Device Name
///------------------------------------------------///
#define TPCMD_FB_DEVICE_NAME_READ     (0xFB)  ///< SRAM Read
  #define FB_BUF_LEN                 (12)
  #define TP_CMD_FB_CMD_LEN          (1)
  #define TP_CMD_FB_TX_DATA_LEN      (FB_BUF_LEN)
  #define TP_CMD_FB_RX_DATA_LEN      (0)  

///------------------------------------------------///
/// OPCODE = 0xC1 Commad Mode
///------------------------------------------------///
#define VENCMD_ANALOG_SET1        (0)   ///< [C1][0][bAdcCtrl0Setup][bMutDriveSenseLevel][XX][XX][XX][XX][XX][XX]
	#define VENCMD_2_VALID_ADCCTRL0     (0x70)	// Detect Range
	#define VENCMD_3_VALID_SENSELEVEL		(0xFF)
	#define VENCMD_4_VALID_DGDCTRL0 		(0x07) // Decimation Ratio
	#define VENCMD_5_VALID_DGDCTRL5 		(0x07) // Digital Filter Freq
	#define VENCMD_6_VALID_ADCCTRL8 		(0x07) // FrontEnd Clk Div
	#define VENCMD_7_VALID_OPTCLK	 			(0x07) // AD CLK Div, ZET6275 different from other IC, EX.6231/6251/6223/6221
	#define VENCMD_8_VALID_BIT4_BUFMODE_EN	(0x10) // Buffer Mode Switch:0=OFF,1=ON
	
#define VENCMD_GET_TRACE          (1)   ///< [C1][01][TRACE][00][00][00][00][00][00][00] ==> TRACE = 0x00 --> driver, TRACE = 0x01 -->Sense
  #define GET_TRACE_DRIVE                (0x00)
  #define GET_TRACE_SENSE                (0x01)
	#define GET_FORCE_TRACE_DRIVE 				 (0x02)
	#define GET_FORCE_TRACE_SENSE 				 (0x03)
	#define GET_TRACE_HW_DRIVE						 (0x04)
	#define GET_TRACE_HW_SENSE						 (0x05)

 
#define VENCMD_SET_TRAN_TYPE      (2)   ///< [C1][02][TRAN_TYPE][55][AA][00][00][00][00][00] 
  /// TRAN_TYPE_DYNAMIC		                (0x00)
  /// TRAN_TYPE_MUTUAL_SCAN_BASE          (0x01)
  /// TRAN_TYPE_MUTUAL_SCAN_DEV           (0x02)
  /// TRAN_TYPE_INIT_SCAN_BASE 		        (0x03)
  /// TRAN_TYPE_INIT_SCAN_DEV		      	  (0x04)
  /// TRAN_TYPE_KEY_MUTUAL_SCAN_BASE		  (0x05)
  /// TRAN_TYPE_KEY_MUTUAL_SCAN_DEV 		  (0x06)
  /// TRAN_TYPE_KEY_DATA  			          (0x07)
  /// TRAN_TYPE_MTK_TYPE  			          (0x0A)
  /// TRAN_TYPE_FOCAL_TYPE  		          (0x0B)
  /// TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV     (0x0D)

#define VENCMD_TP_DETIAL_LEN_QUERY   (3)   ///<
#define VENCMD_TP_DETIAL_LEN_QUERY_CMD_LEN			 (2)		
		
#define VENCMD_TP_DETIAL_INFO          (4)   ///<
#define 	TPINFO_BYTE_0 				(0)  ///< Force Touch  Sense Axis Num
#define 	TPINFO_BYTE_1 				(1)  ///< Force Touch  Drive Axis Num			
#define 	TPINFO_BYTE_2 				(2)  ///< Normal Touch HW Sense Axis Num
#define 	TPINFO_BYTE_3 				(3)  ///< Normal Touch HW Drive Axis Num		

#define 	VENCMD_TP_DETIAL_LEN_QUERY_BUF_LEN      (TPINFO_BYTE_3+1)

#define VENCMD_LOCK_IN_TEST_MODE          (5)   ///<
///-------------------------------------------------------------------------------///
/// Algorithm Status Bit in ZetVar.wAlgoStatus
///-------------------------------------------------------------------------------///
#define ALGO_SEARCH_NINE_SQUARE     (0)
#define ALGO_SEARCH_ADV_NINE_SQUARE  (1)

///-------------------------------------------------------------------------------///
/// Algorithm Status Bit in ZetVar.wAlgoStatus
///-------------------------------------------------------------------------------///
#define ALG_INIT_ABNORMAL_INIT_REK		(0x0001<<1)            ///< 
#define ALG_SLIDE_INTO_PANEL					(0x0001<<2)            ///<
#define ALG_FINGER_IN_BORDER				  (0x0001<<3)  					 ///< for FEATURE_BORDER_DEV_INTERNAL

#define ALG_INIT_ABNORMAL_MUT_ALL_SCAN  (0x0001<<4)  					///<
#define ALG_INIT_ABNORMAL_MUT_REK_QUE		(0x0001<<5)           ///< 

#define ALGO_INT_FIRST_FRAME_LOW			(0x0001<<7)            ///< 

#define ALG_RESET_POWER_TRACKBASE 		(0x0001<<9)   					///< 
#define ALG_COAXIS_HAPPEN							(0x0001<<10)   					///<
#define ALGO_INIT_MUT_PROC_EN					(0x0001<<11) 						///<
#define ALG_CALCULATE_AXIS						(0x0001<<12)   					///<  1: Drive Axis ,0:Sense Axis

#define ALGO_ENHANCE_DECI_UNDER_STEADY	 (0x0001<<12) 						 ///<
#define ALGO_ENHANCE_DECI_OVER_STEADY		 (0x0001<<13) 						 ///<
#define ALGO_ENHANCE_DECI_WHEN_SLOW_SCAN	 (0x0001<<14) 						 ///<

#define ALGO_FINGER_UP_DEBOUNCE_REPORTING (0x0001<<15) 			 ///< 0:Finger Up Debounce Report is finish

///-------------------------------------------------------------------------------///
/// Algorithm Status Bit in ZetVar.wAlgoStatus2
///-------------------------------------------------------------------------------///
#define ALG2_HEAVY_PRESS_DETECTED	  			(0x0001<<0)           ///< Heavy press enable
#define ALG2_HEAVY_PRESS_SWITCH_DETECTED	(0x0001<<1)           ///< 
#define ALG2_HEAVY_PRESS_REPORT_FROZEN		(0x0001<<2)           ///< 
#define ALG2_HEAVY_PRESS_REPORT_FROZEN_LAST	(0x0001<<3)           ///< 

///-------------------------------------------------------------------------------///
/// Algorithm Information
///-------------------------------------------------------------------------------///	
#define MAX_SUPPORTED_FINGER  	  	(16)       ///< Max Support Finger Number
#ifdef FEATURE_RING_BUFFER
#define REPORT_BUFFER_NUM		  	  	(4)        ///<  
#define RING_BUFFER_NOW_ID  				(0)        ///<  Now  Ring Buffer Counter Shift value
#define RING_BUFFER_LAST_ID  (REPORT_BUFFER_NUM-1)    ///<  Last Ring Buffer Counter Shift value  

#else
#define REPORT_BUFFER_NUM		  	  	(2)        ///<  
#endif

#define I2C_FINGER_BUFFER_NUM  	  	(10)       ///

#define MAX_COORDINATE_DIFF					(65535)    ///< Max Coordinate differential distance between two frames
#define MAX_TRACK_NUM								FINGER_MAX

#define COORD_ORG_LEFT_UP           (0x00)     ///< Coordinate Origin
#define COORD_ORG_RIGHT_UP          (0x01)     ///< Coordinate Origin
#define COORD_ORG_LEFT_DOWN					(0x02)     ///< Coordinate Origin
#define COORD_ORG_RIGHT_DOWN        (0x03)     ///< Coordinate Origin

#define COORD_DRIVE_AXIS_REVERSE    (0x01)     ///< Coord start from RIGHT side
#define COORD_SENSE_AXIS_REVERSE    (0x02)     ///< Coord start from DOWN side

#define DEBOUNCE_INIT               (0x00)     ///< Finger UP Debounce
#define DEBOUNCE_REPORT_TH          (0x02)
#define DEBOUNCE_MAX                (200)

/// Finger Tracking
#define TRACK_ID_NONE               (0xFF)
#define TRACKED_INITIALIZATON       (0x00)

/// Finger Down setting
#define FINGER_UP                   (0x00)   
#define FINGER_ID_NOT_READY		      (0x01)   ///< The finger id is not allow to track
#define FINGER_DOWN_INIT            (0x02)   ///< The first finger down, it will not be reported to the host. But, it will be used for tracking  calculation
#define FINGER_DOWN                 (0x03)   ///< The finger down after FINGER_DOWN_INIT
#define FINGER_DOWN_HOP             (0x04)   ///< The finger down after FINGER_DOWN_INIT

/// Finger Track setting
#define FINGER_TRACK_ID_DEBOUNCE				(25)
#define FINGER_TRACK_ID_MAX_DIS_RATIO		(5)

/// Finger Down setting
#define FINGER_TYPE_INIT	          (0x00)  
#define FINGER_TYPE_KEY             (0x01)   
#define FINGER_TYPE_COORD           (0x02)

#define STEADYCOUNT_MAX             (200)

/// Cluster Address
#define EMPTY_SLOT_START 		0
#define CLUSTER_PEAK_PD		  EMPTY_SLOT_START+1
#define CLUSTER_PEAK_PL		  EMPTY_SLOT_START+2
#define CLUSTER_PEAK_PU		  EMPTY_SLOT_START+3
#define CLUSTER_PEAK_PR		  EMPTY_SLOT_START+4
#define CLUSTER_PEAK_STATE	EMPTY_SLOT_START+5
#define CLUSTER_PEAK_VALID	EMPTY_SLOT_START+6

///Merge Finger Algorithm
#define  MERGE_CLOSE_FINGER               (0x10 | 0x02)
#define  MERGE_FINGER_GLUE_TH             (150)
#define  MERGE_FINGER_RELEASE_TH          (200)

/// Merge finger state
#define  MERGE_FINGER_STATE_GLUE 	         (1)
#define  MERGE_FINGER_STATE_NORMAL	       (3)
#define  MERGE_FINGER_STATE_RELEASE        (5)

#define  MERGE_FINGER_POSITIVE_CNT        (20)         ///< 20
#define  MERGE_FINGER_NEGATIVE_CNT        (0xEC)       ///< -20

/// Merge finger state Machine DEBOUNCE
#define  INIT_EXIST_ROUND    (1)
#define  INIT_CHECK_MERGR_ROUND    (3)  ///< 3 round to check divide or merge

#define  INIT_CHECK_FORCE_GLUE_ROUND    (4)

///-------------------------------------------------------------------------------///
/// Zeta Test
///-------------------------------------------------------------------------------///
#define ZETA_TX_TY_VAL											(0x88)
#define ZETA_TS_EVEN_VAL										(0x08)
#define ZETA_TS_ODD_VAL					  					(0x08)

#define TRACE_TIE_TO_VDD										(0xAA)
#define TRACE_TIE_TO_GND  									(0xBB)
#define TRACE_TIE_TO_ADC  									(0xDD)

///-------------------------------------------------------------------------------///
/// Mainloop break point
///-------------------------------------------------------------------------------///
#define BREAK_POINT_ADDR          (0x40F)

///-------------------------------------------------------------------------------///
/// I2C
///-------------------------------------------------------------------------------///
#define I2C_PACKET_ID             (0x3C)
#define I2C_MIX_DYNAMIC_MUTUALDEV_PACKET_ID (0x3D)
#define I2C_MIX_DYNAMIC_FRH_PACKET_ID       (0x3E)
#define I2C_MIX_PACKET_ID         (0x3D)
#define I2C_INT_FREE_PACKET_ID    (0x3B)
#define I2C_NULL_PACKET_ID        (0x00)

#define I2C_INT_TIMEOUT_CNT       (20)    ///< # round of mainloop as the timeout
#define MAX_MTK_MODE_SKPI_TPCMD   (MAX_SUPPORTED_FINGER * 4 + 3)    ///< 16*4 + 3 = 67

#define I2C_STA_TX_START    (1<<0)
#define I2C_STA_PACKET_HEAD (1<<1) ///< the I2C packet header glag of TX packet
#define I2C_STA_SRAM_RW_MODE (1<<2) ///< 0:Write mode 1:Read Mode

///-------------------------------------------------------------------------------///
/// Transfer Type
///-------------------------------------------------------------------------------///	
#define TRAN_TYPE_DYNAMIC		                (0x00)
#define TRAN_TYPE_MUTUAL_SCAN_BASE          (0x01) ///< 16-bit
#define TRAN_TYPE_MUTUAL_SCAN_DEV           (0x02) ///< 8-bit
#define TRAN_TYPE_INIT_SCAN_BASE 		        (0x03) ///< 16-bit
#define TRAN_TYPE_INIT_SCAN_DEV		      	  (0x04) ///< 8-bit
#define TRAN_TYPE_KEY_MUTUAL_SCAN_BASE		  (0x05) ///< 16-bit
#define TRAN_TYPE_KEY_MUTUAL_SCAN_DEV 		  (0x06)
#define TRAN_TYPE_KEY_DATA  			          (0x07)
#define TRAN_TYPE_MTK_TYPE  			          (0x0A)
#define TRAN_TYPE_FOCAL_TYPE  		          (0x0B)
#define TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV     (0x0D)
#define TRAN_TYPE_MIX_DYNAMIC_FRH           (0x0E)
#define TRAN_TYPE_CHECKSUM_READ		          (0x0F)
#define TRAN_TYPE_ADV_INIT_SCAN_DEV 				(0x10) ///< two init scan driving data
#define TRAN_TYPE_FORCE_SCAN_BASE          	(0x11) ///< 
#define TRAN_TYPE_FORCE_SCAN_DEV           	(0x12) ///< 
#define TRAN_TYPE_FORCE_C1C2_BASE           (0x13) ///< 
#define TRAN_TYPE_UNKNOWN        	          (TRAN_TYPE_FORCE_C1C2_BASE + 1)  ///< last item add one

///-------------------------------------------------------------------------------///
/// Focal Data Type
///-------------------------------------------------------------------------------///	
/// 1. DEVICE_MODE 
#define DEVICE_MODE_NORMAL                   (0x00)
#define DEVICE_MODE_SYSTEM_INFO              (0x01)
#define DEVICE_MODE_TEST_MODE                (0x04)

/// 2. GEST_ID 
#define GEST_MOVE_UP                         (0x10)
#define GEST_MOVE_LEFT                       (0x14)
#define GEST_MOVE_DOWN                       (0x18)
#define GEST_MOVE_RIGHT                      (0x1C)
#define GEST_ZOOM_IN                         (0x48)
#define GEST_ZOOM_OUT                        (0x49)
#define GEST_NONE                            (0x00)

/// 2. FINGER
#define FOCAL_PUT_DOWN                       (0x00)
#define FOCAL_PUT_UP                         (0x40)
#define FOCAL_PUT_CONTACT                    (0x80)
#define FOCAL_PUT_RESERVED                   (0xC0)

///-------------------------------------------------------------------------------///
/// Dev Data Buffer Type
///-------------------------------------------------------------------------------///
#define ALGO_DEV_DATA                        (0)
#define SCAN_DEV_DATA                        (1)

///-------------------------------------------------------------------------------///
/// Palm Status
///-------------------------------------------------------------------------------///
#define PALM_STATE_LEVEL_MAX                  (5)     ///< Max Palm State
#define PALM_STATE_LEVEL_0                    (0)     ///< No Palm

///-------------------------------------------------------------------------------///
/// Virtual Key
///-------------------------------------------------------------------------------///
#define VIRTUAL_KEY_MAX_NUM                   (4)
#define VIRTUAL_KEY_VALID_BIT		              (0x01)
#define VIRTUAL_KEY_DEBOUNCE_BIT              (0x10)
///-------------------------------------------------------------------------------///
/// DEV Process
///-------------------------------------------------------------------------------///
#define SEN_ALWAYS_EN                         (1<<0)  ///< 0: Not always enable, 1: always enable
#define SEN_CHARGER_MODE_EN                   (1<<1)  ///< 0: Not enable,  1: Enable at Charger mode 
#define SEN_DEV_PROCESS_EN                    (1<<2)  ///< 0: Do nothing,  1: Do dev process

#define DRI_ALWAYS_EN                         (1<<4)  ///< 0: Not always enable, 1: always enable
#define DRI_CHARGER_MODE_EN                   (1<<5)  ///< 0: Not enable,  1: Enable at Charger mode 
#define DRI_DEV_PROCESS_EN                    (1<<6)  ///< 0: Do nothing,  1: Do dev process

#define INIT_DEV_PROCESS_EN                   (1)  ///< 0: Do nothing,  1: Do Init dev process
#define INIT_DEV_MA_PROCESS_EN								(2)
#define INIT_DEV_PROCESS_SCALER              ((1<<0) | (1<<1) | (1<<2))  ///< 0: 12/16¡A1: 13/16¡A2: 14/16¡A3: 15/16¡A4: 16/16 (default)¡A5: 17/16¡A6: 18/16¡A7: 19/16
		
/// For UI setup
#define SEN_DEV_PROC_CTRL_DISABLE                        (0x00)   ///<
#define SEN_DEV_PROC_CTRL_NORMAL                         (1<<0)   ///<
#define SEN_DEV_PROC_CTRL_NORMAL_COAXIS									 (1<<1)   ///<	
#define SEN_DEV_PROC_CTRL_CHARGER   										 (1<<2)   ///<
#define SEN_DEV_PROC_CTRL_CHARGER_COAXIS								 (1<<3)   ///<

#define DRI_DEV_PROC_CTRL_NORMAL                         (1<<4)   ///<
#define DRI_DEV_PROC_CTRL_NORMAL_COAXIS									 (1<<5)   ///<	
#define DRI_DEV_PROC_CTRL_CHARGER   										 (1<<6)   ///<
#define DRI_DEV_PROC_CTRL_CHARGER_COAXIS								 (1<<7)   ///<

/// For UI setup
#define HW_SW_DEV_PROC_CTRL_DISABLE                        (0x00)   ///<
#define HW_SW_DEV_PROC_CTRL_ALWAYS												 (0x0F)   ///<
#define HW_SW_DEV_PROC_CTRL_NORMAL                         (1<<0)   ///<
#define HW_SW_DEV_PROC_CTRL_NORMAL_COAXIS									 (1<<1)   ///<	
#define HW_SW_DEV_PROC_CTRL_CHARGER   										 (1<<2)   ///<
#define HW_SW_DEV_PROC_CTRL_CHARGER_COAXIS								 (1<<3)   ///<

/// For FEATURE_COMP_MUT_WITH_INIT setup
#define INIT_CMP_MUT_CTRL_DISABLE                 (0x00)   ///<
#define INIT_CMP_MUT_CTRL_NORMAL                  (1<<0)   ///<
#define INIT_CMP_MUT_CTRL_NORMAL_COAXIS						(1<<1)   ///<	
#define INIT_CMP_MUT_CTRL_CHARGER   							(1<<2)   ///<
#define INIT_CMP_MUT_CTRL_CHARGER_COAXIS					(1<<3)   ///<

#define SEN_DEV_PROC_TYPE                (1<<0)   ///< 0:MA  / 1:advance
#define DRI_DEV_PROC_TYPE                (1<<1)   ///< 0:MA  / 1:advance

///-------------------------------------------------------------------------------///
/// Psudo random
///-------------------------------------------------------------------------------///
#define PSEUDO_MODE_DISABLE                   (0)
#define PSEUDO_MODE_CHARGER_ENABLE            (1)
#define PSEUDO_MODE_ALWAYS_ENABLE             (2)

#define PSEUDO_ENABLE()                       (PSEUDO &= ~RST_PSEUDO)
#define PSEUDO_DISABLE()                      (PSEUDO |= RST_PSEUDO)

///-------------------------------------------------------------------------------///
///  Heavy Press
///-------------------------------------------------------------------------------///
#define HEAVY_PRESS_DISABLE                   (0)  ///< 0: disable the heavy press mode
#define HEAVY_PRESS_CHARGER_MODE_ENABLE_ONLY  (1)  ///< 1: Enable the heavy press algorithm only at charger mode
#define HEAVY_PRESS_ALWAYS_ENABLE             (2)  ///< 2: Enable the heavy press algorithm, always

#define MAX_2X_MUTUAL_GAIN                    (14) ///< Max gain = 7x2 = 14
///-------------------------------------------------------------------------------///
///  RC-Trim
///-------------------------------------------------------------------------------///
#define RC_TRIM_DISABLE                         (0)  ///<
#define RC_TRIM_NORMAL_ENABLE                   (1)  ///<

#define RC_TRIM_R_MASK                          (CK_RS0 | CK_RS0| CK_RS0) ///< BIT#0~BIT#2
#define RC_TRIM_C_MASK                          (CK_CS0 | CK_CS1| CK_CS2) ///< BIT#3~BIT#5

///-------------------------------------------------------------------------------///
///  Dev Collect
///-------------------------------------------------------------------------------///
#define DEV_CUT_MAX                        (FALSE)
#define DEV_NO_CUT_MAX                     (TRUE)   

/// For UI setup
#define DEV_COLLECT_DISABLE                            (0x00)   ///<
#define DEV_COLLECT_DISABLE2                           (0xFF)   ///<
#define DEV_COLLECT_ALWAYS_ENABLE                      (0x01)   ///< normal  on, normal-coaxis on , charger  on, Charge-coaxis on

#define DEV_COLLECT_CHARGER_ENABLE                     (0x02)   ///< normal off, normal-coaxis off, charger  on, Charge-coaxis off
#define DEV_COLLECT_CHARGER_OFF_AND_COAXIS_ON_ENABLE   (0x03)   ///< normal off, normal-coaxis off, charger off, Charge-coaxis on
#define DEV_COLLECT_CHARGER_ON_AND_COAXIS_ON_ENABLE    (0x04)   ///< normal off, normal-coaxis off, charger  on, Charge-coaxis on

#define DEV_COLLECT_NORMAL_ENABLE   									 (0x05)   ///< normal on , normal-coaxis off, charger off, Charge-coaxis off
#define DEV_COLLECT_NORMAL_OFF_AND_COAXIS_ON_ENABLE    (0x06)   ///< normal off, normal-coaxis on , charger off, Charge-coaxis off
#define DEV_COLLECT_NORMAL_ON_AND_COAXIS_ON_ENABLE     (0x07)   ///< normal on , normal-coaxis on , charger off, Charge-coaxis off

/// For software setup
#define SW_COLLECT_CTRL_DISABLE                        (0x00)   ///<
#define SW_COLLECT_CTRL_NORMAL                         (1<<0)   ///<
#define SW_COLLECT_CTRL_NORMAL_COAXIS									 (1<<1)   ///<	
#define SW_COLLECT_CTRL_CHARGER   										 (1<<2)   ///<
#define SW_COLLECT_CTRL_CHARGER_COAXIS								 (1<<3)   ///<

/// For hardware setup
#define HW_COLLECT_CTRL_DISABLE                        (0x00)   ///<
#define HW_COLLECT_CTRL_NORMAL                         (1<<0)   ///<
#define HW_COLLECT_CTRL_NORMAL_COAXIS									 (1<<1)   ///<	
#define HW_COLLECT_CTRL_CHARGER   										 (1<<2)   ///<
#define HW_COLLECT_CTRL_CHARGER_COAXIS								 (1<<3)   ///<
#define HW_COLLECT_CTRL_KEY														 (1<<4)   ///<

/// For Mode control
#define DEV_COLLECT_MODE_DISABLE                  (0x00)   ///< Disable
#define DEV_COLLECT_MODE_ENABLE                   (1<<0)   ///< 1: Function enabled, 0:Function disabled
#define DEV_COLLECT_MODE_ALWAYS_ENABLE            (1<<1)   ///< 1: Alwyas enable
#define DEV_COLLECT_MODE_CHARGER_ENABLE           (1<<2)   ///< 1: Charger Mode enable only,
#define DEV_COLLECT_MODE_NORMAL_ENABLE            (1<<3)   ///< 1: Normal Mode enable only
#define DEV_COLLECT_MODE_COAXIS_MODE_ENABLE       (1<<4)   ///< 1: Function enabled, 0:Function disabled
#define DEV_COLLECT_MODE_NO_CUT                   (1<<5)

#define DEV_COLLECT_MODE0     (0)
#define DEV_COLLECT_MODE1     (1)
#define DEV_COLLECT_MODE2     (2)
#define DEV_COLLECT_MODE3     (3)
#define DEV_COLLECT_MODE4     (4)
#define DEV_COLLECT_MODE5     (5)
#define DEV_COLLECT_MODE_MAX  (DEV_COLLECT_MODE5)

///-------------------------------------------------------------------------------///
/// COAXIS_MODE
///-------------------------------------------------------------------------------///
#define COAXIS_MODE_DISABLE                   (0)
#define COAXIS_MODE_CHARGER_ENABLE            (1)
#define COAXIS_MODE_ALWAYS_ENABLE             (2)
#define COAXIS_MODE_NORMAL_ENABLE             (3)

#define COAXIS_CHECK_EXECUTE_NORMAL_ENABLE    (1<<0)
#define COAXIS_CHECK_EXECUTE_CHARGER_ENABLE   (1<<1)

///-------------------------------------------------------------------------------///
/// Charger noise detection
///-------------------------------------------------------------------------------///
#define NOISE_DETECT_DISABLE                   (0)
#define NOISE_DETECT_ALWAYS_ENABLE             (1)
#define NOISE_DETECT_CHARGER_ENABLE            (2)

///-------------------------------------------------------------------------------///
/// Init Scan
///-------------------------------------------------------------------------------///
#define INIT_SCAN_NORMAL_MODE       (0)

///-------------------------------------------------------------------------------///
/// Finger Distance initialization 
///-------------------------------------------------------------------------------///
#define DEFAULT_INITIAL_DISTANCE     1

///-------------------------------------------------------------------------------///
/// FingerCoordinateRecord Finger Status
///-------------------------------------------------------------------------------///
#define FINGER_STATUS_INIT											(0x00)		/// Finger status initialization

#define FINGER_STATUS_VALID  									  (1<<0)		/// Finger is tracked with some search finger record

#define FINGER_STATUS_IS_MOVED									(1<<2)		/// Finger is moving 
#define FINGER_STATUS_IS_HIT_IN_BORDER_EDGE			(1<<3)		/// Finger is initial touch in border edge area
#define FINGER_STATUS_IS_MOV_IN_BORDER_EDGE			(1<<4)		/// Finger is touch in border edge area
#define FINGER_STATUS_IS_MOV_IN_BORDER_DOUBLE_EDGE			(1<<5)		/// Finger is touch in border double edge area

#define FINGER_STATUS_EDGE_SCHMIT_XON				  	(1<<6)		/// 
#define FINGER_STATUS_EDGE_SCHMIT_YON				  	(1<<7)		/// 

#define FINGER_STATUS_FORCE_NO_FINGER						(1<<8)		/// Force no finger
#define FINGER_STATUS_FORCE_NO_FINGER_LAST2			(1<<9)		/// 
#define FINGER_STATUS_FORCE_NO_FINGER_LAST3			(1<<10)		/// 
#define FINGER_STATUS_REHOOK										(1<<11)		/// 

#define FINGER_STATUS_FORCE_NO_FINGER_CHECK			(FINGER_STATUS_FORCE_NO_FINGER|FINGER_STATUS_FORCE_NO_FINGER_LAST2)		/// 
#define FINGER_STATUS_EDGE_MAX_DIST							(255)

///-------------------------------------------------------------------------------///
/// Border Comp Algorithm
///-------------------------------------------------------------------------------///
#define BORDER_COMP_CLOSE_DEV_DELTA							(1)				/// Border compensation will use close dev delta algorithm
#define BORDER_COMP_DYNAMIC_DEV_DELTA						(2)				/// Border compensation will use fixed dev delta algorithm
#define BORDER_COMP_DYNAMIC_DEV_DIVIDER					(32)			/// 

#define BORDER_SEN_FAR                          (0)
#define BORDER_SEN_NEAR                         (1)
#define BORDER_DRI_FAR                          (2)
#define BORDER_DRI_NEAR                         (3)

///-------------------------------------------------------------------------------///
/// FEATURE_COORD_EDGE_POSITION_LOCK
///-------------------------------------------------------------------------------///
#define COORD_LIMIT_REPORT_DISABLE   (0)  ///< if (coord > max) or (coord < min) , clear report finger valid bit

#define COORD_LIMIT_REPORT_NONE   (1)  ///< if (coord > max) or (coord < min) , clear report finger valid bit
#define COORD_LIMIT_REPORT_MAXMIN	  (2)	 ///< if (coord > max) or (coord < min) , report the edge coord value
#define COORD_LIMIT_REPORT_SLIDE 	(3)	 ///< if (coord > max) or (coord < min) , report the Old coord value
#define COORD_LIMIT_REPORT_SLIDE_TOUCH_DOWN 	(4)	 ///< if (coord > max) or (coord < min) , report the Old coord value

#define SENSE_AXIS   (0) 
#define DRIVE_AXIS   (1) 

///-------------------------------------------------------------------------------///
/// Coordinate compensation
///-------------------------------------------------------------------------------///
#define TYPE_COORX (0)
#define TYPE_COORY (1)

///-------------------------------------------------------------------------------///
/// Power Saving
///-------------------------------------------------------------------------------///
#define POWER_SAVING_SHORT_IDLE   (1<<0)
///-------------------------------------------------------------------------------///
/// Power Saving
///-------------------------------------------------------------------------------///
#define KEY_SCAN_BASE             (0)   ///< Get the Key scan base
#define KEY_SCAN_DEV              (1)   ///< Get the Key scan dev
#define KEY_DRIVE_TRACE_NUM				(1)

///-------------------------------------------------------------------------------///
/// Timer
///-------------------------------------------------------------------------------///
#define TIMER_PERIOD_GREEN          0x5E9C   ///< 2.457us x 0x5E9C ~=  60ms, 16Hz frame rate scan ///< 2.457us x 0x4F7E ~=  50ms, 20Hz frame rate scan ///< 2.457us x 0x3f98 ~=  40ms, 25Hz frame rate scan,
#define TIMER_EN                  		(1<<0)
#define TIMER_SETUP_IMMEDIATELY   		(1)
#define TIMER_SETUP_NON_IMMEDIATELY		(0)

#define RC512K_TRIM_REFERENCE        116  ///<  With Real case get reference:  407KHz-OSC count 2^12 ~= 10.063ms , 34.72Mhz count 3000 ~= 86.4us , 10.063ms / 86.4us = 116
#define ONE_SECOND_COUNT             407000 ///< 407KHz-OSC count 407000 for 1-second 

///-------------------------------------------------------------------------------///
/// Timer Status in ZetVar.bTimerStatus
///-------------------------------------------------------------------------------///
#define TIMER_RELOAD_FLAG							        (1<<0)    ///< 1: Timer Reload-value change flag, 0: not need changed! 
#define TIMER_INITIAL_FLAG                    (1<<7)

///-------------------------------------------------------------------------------///
/// Coordinate
///-------------------------------------------------------------------------------///
#define RAW_POINT_CNT  (15)

#define RAW_POINT_LEN  (3)
//#define COOR_RAW_IDX_LAST3      (RAW_POINT_LEN-4) // 0
#define COOR_RAW_IDX_LAST2      (RAW_POINT_LEN-3) // 1
#define COOR_RAW_IDX_LAST       (RAW_POINT_LEN-2) // 2
#define COOR_RAW_IDX_NEW        (RAW_POINT_LEN-1) // 3

#define EXE_POINT_LEN  (3)
#define COOR_EXE_IDX_LAST3      (EXE_POINT_LEN-3) // 0
#define COOR_EXE_IDX_LAST2      (EXE_POINT_LEN-2) // 1
#define COOR_EXE_IDX_LAST       (EXE_POINT_LEN-1) // 2

#define REC_DISTANECE_LEN  (3)
//#define DISTANCE_IDX_LAST3      (REC_DISTANECE_LEN-4) // 0
#define DISTANCE_IDX_LAST2      (REC_DISTANECE_LEN-3) // 1
#define DISTANCE_IDX_LAST       (REC_DISTANECE_LEN-2) // 2
#define DISTANCE_IDX_NEW        (REC_DISTANECE_LEN-1) // 3

#define REC_INIT_CNT	  (0)

#define SMOOTH_ALG_IIR_FIXED				(0)
#define SMOOTH_ALG_IIR_DYNAMIC			(1)
#define SMOOTH_ALG_PD_TYPE					(2)
///-------------------------------------------------------------------------------///
/// Kelman Filter
///-------------------------------------------------------------------------------///
#ifdef FEATURE_KALMAN_FILTER_SMOOTHING
#define KALMAN_MQ         (5) ///< The smaller the more smooth, the bigger the more fast
#define KALMAN_MR         (200)
#define KALMAN_MP_INIT    (10000)
#define KALMAN_FACTOR     (1000)
#endif
///-------------------------------------------------------------------------------///
/// Sharp Dev Cut
///-------------------------------------------------------------------------------///
#define FEATURE_SHARP_DEV_CLUSTER_EN            (1<<0)    ///< enable the sharp dev cut only at one finger cluster
#define FEATURE_SHARP_DEV_NINE_SQUARE_EN        (1<<1)    ///< enable the sharp dev cut when nine-square finger search

///-------------------------------------------------------------------------------///
/// Check Sum
///-------------------------------------------------------------------------------///
#define MCU_8_BIT 	(1) ///instruction length
#define MCU_16_BIT  (2)
#define MCU_32_BIT  (4)

///-------------------------------------------------------------------------------///
/// Advance Finger Track
///-------------------------------------------------------------------------------///
#define MASKSTAR 			(1) 
#define MASKPRIME 		(2) 
#define TRACK_STEP_1 	(1) 
#define TRACK_STEP_2 	(2) 
#define TRACK_STEP_3 	(3) 
#define TRACK_STEP_4 	(4) 
#define TRACK_STEP_5 	(5) 
#define TRACK_STEP_6 	(6) 
#define TRACK_STEP_7 	(7) 

///-------------------------------------------------------------------------------///
/// Cluster variable
///-------------------------------------------------------------------------------///
#define MAX_CLUSTER_SUPPORT        	(FINGER_MAX_REPORT+1)
#define CLUSTER_ID_ARRAY_NUM       	(2)
#define CLUSTER_ID_LAST  						(CLUSTER_ID_ARRAY_NUM-1)
#define CLUSTER_ID_LAST2 						(CLUSTER_ID_ARRAY_NUM-2)
#define CLUSTER_ID_INITIALIZATION		(0xFF)
#define CLUSTER_MIN_SIZE_TH		 			(4) 
#define CLUSTER_MIN_WEIGHT_TH 			(25) 

#define CLUSTER_DEV_CAL_RADIUS_SML	 (0) 
#define CLUSTER_DEV_CAL_RADIUS_MID	 (1) 
#define CLUSTER_DEV_CAL_RADIUS_BIG	 (3) 

#define CLUSTER_DEV_CAL_RADIUS_BIG	 (3) 
#define CLUSTER_RE_SEARCH_PEAK_TRUE					 (1)
#define CLUSTER_RE_SEARCH_PEAK_FALSE				 (0)

#define CLUSTER_MIN_DEV 			(20) 
#define CLUSTER_MAX_DEV 			(90)
#define CLUSTER_RESEARCH_SIZE	(6) 
#define CLUSTER_ID_INIT 			(255)
#define CLUSTER_PRE_ALLOCATED_ID 			(254)  /// dev found, cluster allocate, but only pre-ID

#define FINGER_5_COMBINATION_NUMBER 	(10)  ///< C(5 ,2) = 10;
#define FINGER_10_COMBINATION_NUMBER 	(45)  ///< C(10,2) = 45; 

///-------------------------------------------------------------------------------///
/// For Cluster Coordiante Calculate
///-------------------------------------------------------------------------------///
#define CLUSTER_ID_VALID			 	(1<<0)
#define CLUSTER_FINGER_VALID	 	(1<<1)
#define CLUSTER_RE_SUM_WEIGHT	 	(1<<2)

///-------------------------------------------------------------------------------///
/// For 1st Tracked Coordinate structure
///-------------------------------------------------------------------------------///
#define TRACK_STATUS_VAILD				 	(1<<0)
#define TRACK_STATUS_TRACKED			 	(1<<1)

#define TRACK_STATUS_NEED_MERGED		(1<<2)
#define TRACK_STATUS_RE_COMPUTE			(1<<3)

#define TRACK_STATUS_MERGE_STATUS		(1<<2) ///< 1:Merge , 0:Divid
#define TRACK_STATUS_FORCE_MERGE    (1<<3) ///< 

#define TRACK_STATUS_SCHMIT_ON			(1<<4)
#define TRACK_STATUS_IN_BORDER			(1<<5)
#define TRACK_STATUS_TRACKED_REMOVED			(1<<6)
#define TRACK_STATUS_SHIFT_IS_RECORD			(1<<7)

#ifdef FEATURE_BORDER_AREA_DOUBLE
///#define TRACK_STATUS_IN_BORDER_DOUBLE			(1<<5)
#endif

#define MERGE_DEBOUNCE_MAX_NUM			(8)
#define MERGE_HALF_RANGE						(1)

#define MERGE_MAX_NUM								(50)   /// C(10,2) = 45
#define MERGE_STATE_INITIALIZATION (0)
#define MERGE_STATE_GLUE 					 (1)
#define MERGE_STATE_DIVID 				 (2)

#define MERGE_DEBOUNCE_INIT_VALUE	 (0)
#define MERGE_DEBOUNCE_OUT_GLUE	 	 (24)
#define MERGE_DEBOUNCE_OUT_DIVID	 (4)
//#define MERGE_DIST_TH_DIVID   		 (128)
//#define MERGE_DIST_TH_GLUED  		   (64)

#define CLUSTER_SIZE_CHANGE		 (3)
#define CLUSTER_SIZE_STANDARD	 (4)

#define CLUSTER_SQUARE	 (5)  ///< 5x5 array flag
#define CLUSTER_SQUARE_CENTER	 (CLUSTER_SQUARE/2)  ///< array center
#define CLUSTER_SQUARE_RADIUS	 (CLUSTER_SQUARE/2 )  ///< array center
#define PEAK_VIBRATION_RANGE  (20)

#define CLUSTER_MERGE_RATIO_ORDER	   (7)  /// 2^N  7=1/128
#define CLUSTER_MERGE_RATIO	 (109)
#define CLUSTER_DIVID_RATIO	 (35)

#define CLUSTER_SIZE_DELTA_RATIO_ORDER	   (7)  /// 2^N  7=1/128

#define CLUSTER_PEAK_MERGE_DEBOUNCE_MAX	 (12)
#define CLUSTER_FAKE_DIVIDE_DEBOUNCE	 (12)

///-------------------------------------------------------------------------------///
///  Dev % Ratio Control
///-------------------------------------------------------------------------------///
#define SENSE_ADC_DEV_RATIO   (1) 
#define DRIVE_ADC_DEV_RATIO   (2) 

///-------------------------------------------------------------------------------///
///  Initial Abnormal Rek 
///-------------------------------------------------------------------------------///
#define MUT_REK_FRAME_NUM   	3//(3)  ///< the Neg-finger continue Frmae Num to execute Mut Rek
#define MUT_REK_FRAME_MAX   	(100)  ///<

#define MUT_REK_1NF_FRAME_NUM   	(20)//(3)  ///< the Neg-finger continue Frmae Num to execute Mut Rek

#define MUT_REK_NFINGER_NUM   (2)  ///< the Neg-finger num to immediately execute Mut Rek
#define MUT_REK_MAX_TIMES   	(2)//(3)  ///< the max Mut Rek times , if bigger than this , should 
#define MUT_REK_DEBOUCNE_NUM  (12) ///< the debounce delay to execute Mut-ReK

#define STOP_CONT_REK_PENDING_ROUND   (1024)
#define MAX_CONT_REK_TIMES   (5)

///-------------------------------------------------------------------------------///
///  Function Control 
///-------------------------------------------------------------------------------///
#define  FUN_STEADY_SIGNAL_ENHANCE  		    (1<<0) ///< enable : when move < steady range , decimation enhance 1 times
#define  FUN_BORDER_IIR_FIXED       		    (1<<1) ///< enable : when move < steady range , decimation enhance 1 times
#define  FUN_MERGE_DEV      						    (1<<2) ///< 
#define  FUN_BORDER_WAVE_REDUCE_ENABLE      (1<<3) ///<   FEATURE_BORDER_DEV_INTERNAL
#define  FUN_AXIS_PROC_NEG_CONCERN 			    (1<<4) ///<   Axis Process Concerned Neg-Value
#define  FUN_LTYPE_ROOT_RAWDEV		 			    (1<<5) ///<   L type Root Raw Dev
#define  FUN_SENSE_DEV_PROC_FINGER_PROTECT  (1<<6) ///<   Finger protection enable of sense axis dev process
#define  FUN_RAW_DATA_VIEW				 			    (1<<7) ///<  Raw Data View enable for APK

///-------------------------------------------------------------------------------///
///  Function Control 1
///-------------------------------------------------------------------------------///
#define  FUN1_SENSE_DEV_PROC_PRIORITY_CTRL  (1<<0) ///<   Sense Axis Process Processing Priority Control

///-------------------------------------------------------------------------------///
///  Coord extend 
///-------------------------------------------------------------------------------///
#define  COOR_EXTEND_ENABLE  			(1<<0) ///< enable/disable 
#define  COOR_EXTEND_BASE_AXIS  	(1<<1) ///< 0: Drive Axis / 1:Sense Axis

///-------------------------------------------------------------------------------///
///  Panel Type
///-------------------------------------------------------------------------------///
#define  TP_TYPE_H         (1) ///< 
#define  TP_TYPE_L         (2) ///< only for Pad
#define  TP_TYPE_1T2R      (3) ///< for IC_Synopsys
#define  TP_TYPE_1T2R_NO_EDGE   (4) ///<  without head Row & last Row
#ifdef FEATURE_NINE_SQUARE_ENABLE
#define  TP_TYPE_1T2R_REVERSE   (5) ///<  without head Row & last Row only for Pad
#endif
//#define  TP_TYPE_1T2R_ROW_SHIFT_A (0x66) ///<
//#define  TP_TYPE_1T2R_ROW_SHIFT_B (0x69) ///<

#define  TP_TYPE_1T2R_ROW_F (0x6) ///<
#define  TP_TYPE_1T2R_ROW_R (0x9) ///<

#define  TP_TYPE_TYPE		   (0x0F) ///<  00:normal ,  01:H  , 02:L
///-------------------------------------------------------------------------------///
///  Panel Sub Type
///-------------------------------------------------------------------------------///
#define  TP_TYPE_1T2R_CROSS_TPYE			(0x03)  //  0:disable , 1:Odd Cross 2:Even Cross
#define  TP_TYPE_H_AXIS		 						(1)<<2  // (0x04) ///< 0:DriveAxis , 1:SenseAxis
#define  TP_TYPE_H_1_EVEN_ODD_START	 	(1)<<4  // (0x10) ///< 
#define  TP_TYPE_H_2_EVEN_ODD_START	 	(1)<<5  // (0x20) ///< 
#define  TP_TYPE_L_EVEN_ODD_START	 		(1)<<6  //(0x40) ///< 


///-------------------------------------------------------------------------------///
///  Panel Near End and Finger Enhancement
///-------------------------------------------------------------------------------///
#define  FINGER_ENHAN_EN     (1) ///< The Bit 0 of FINGER_ENHANCE_EN, 0: disable, 1: enable finger enhancement function
#define  TP_NEAR_END         (2) ///< The Bit 1 of FINGER_ENHANCE_EN, 0: upper, 1: lower side of MUT DEV

#define  FINGER_ENHAN_SUPPORT_NUM  (5)

///-------------------------------------------------------------------------------///
/// 1T2R Dev Process
///-------------------------------------------------------------------------------///
#define SEN_DEV_PORC_STEP_1T1R  2
#define SEN_DEV_PORC_STEP_1T2R  1

/// bR1toR2en But Option
#define RATIO_R1R2_EN      		  0x01<<0
#define RATIO_NEAR_FAR_EN       0x01<<1
#define RATIO_NEAR_NEBIGHBOR    0x01<<2 // 0: Left  1:Right
#define RATIO_NEAR_FAR_MA_EN    0x01<<3
#define RATIO_NEAR_FAR_2ND_EN   0x01<<4
#define RATIO_NEAR_FAR_SHOW_EST    0x01<<5
#define RATIO_NEAR_FAR_2ND_REV_DIR    0x01<<6
#define DRI_DEV_PORC_EVEN  0	///<  Drive Dev Process exe Even part
#define DRI_DEV_PORC_ODD   1	///<  Drive Dev Process exe odd part

///-------------------------------------------------------------------------------///
/// Heavy Press Data Flash Ctrl bit
///-------------------------------------------------------------------------------///
#define HEAVY_PRESS_CTRL_CUT_AVG_EN     (0x01<<0)
#define HEAVY_PRESS_CTRL_CHANGE_GAIN_EN (0x01<<1)  ///<
#define HEAVY_PRESS_CTRL_FIX_PT_EN      (0x01<<2)
#define HEAVY_PRESS_CTRL_CLR_PT_EN      (0x01<<3)

///-------------------------------------------------------------------------------///
/// FINGER_UP_ENABLE (bFingerUpEnable) Ctrl bit
///-------------------------------------------------------------------------------///
#define FINGER_UP_REPORT_BIT0     			(0x01<<0)
#define FINGER_UP_MV_BIT1     					(0x02<<0)

///-------------------------------------------------------------------------------///
/// Initial Calibration Process Mode Ctrl
///-------------------------------------------------------------------------------///
#define INITIAL_DEV_MODE     			(0)
#define INITIAL_AD_MODE     			(1)
#define KEY_DEV_MODE     					(0)
#define KEY_AD_MODE     					(1)

///-------------------------------------------------------------------------------///
/// ZET7100 Frequecny Hop Tone ID
///-------------------------------------------------------------------------------///
#ifdef FEATURE_7101_DEVELOPMENT
#define MAX_HOP_NUM   			(10)
#define MAX_TONE_NUM   			(1)
#else
#define MAX_HOP_NUM   			(32)
#define MAX_TONE_NUM   			(2)
#endif
#define TONE_INIT			(0)
#define TONE_A   			(0)
#define TONE_B   			(1)

#define HISTORY_SAVE_NUM   	(8)

#define FREQ_HOP_SORT_NUM		(4)	

#define HOP_SCAN_GET_ADDELTA_MODE (0)
#define HOP_SCAN_GET_ADBASE_MODE (1)

#define DYNAMIC_HOP_BASE_IS_READY						(1<<0)		///< for All Frequency-Set ADBase
#define DYNAMIC_HOP_ROUND_FINISH						(1<<1)		///<
#define DYNAMIC_HOP_RESET_DATABASE					(1<<2)		///<
#define DYNAMIC_HOP_QUCIK_SCAN_EN						(1<<3)		///< if detct current Freq is basd , immedicatedily Scan all Freq-Set
#define DYNAMIC_HOP_NOISE_DETECT_EN					(1<<4)		///<
#define DYNAMIC_HOP_CURRENT_BASE_IS_READY		(1<<5)		///< for Currnent Frequency-Set ADBase
#define DYNAMIC_HOP_CURRNET_MODE						(1<<6)		///< 1:for Current-Frequency-Set , 0:for Frequency-Hop-Scan Mode
#define DYNAMIC_HOP_EXECUTE									(1<<7)		///< 1:New FreqID is found , need to execute change Freq & ReK

///-------------------------------------------------------------------------------///
/// ZET7100 Command Write SRAM Parameter ()
///-------------------------------------------------------------------------------///
#define SPARA_TONEA   			(0)  ///< SRAM[0x00100000] CMD[0x8000]  ToneA mul
#define SPARA_TONEB   			(1)  ///< SRAM[0x00100001] CMD[0x8001]  ToneB mul
#define SPARA_PNCHIPNUM			(2)  ///< SRAM[0x00100002] CMD[0x8002]  [4:0]:PnChip num ; [7]:Dual Tone enable
#define SPARA_ADCTRL0				(3)  ///< SRAM[0x00100003] CMD[0x8003]  [6:4]:FrontEnd Gain = 0~7 (Small -> Big)
#define SPARA_ADCTRL1				(4)  ///< SRAM[0x00100004] CMD[0x8004]  [6:4]:LpFilter Gain = 0~7 (Small -> Big)
#define SPARA_ADCTRL10			(5)  ///< SRAM[0x00100005] CMD[0x8005]  [4  ]:0=Buffer Mode,1:Scaleing Mode
#define SPARA_CHGPUMP				(6)  ///< SRAM[0x00100006] CMD[0x8006]  [0  ]:ChargePump 1:Enable , 0:Disable
#define SPARA_PN_CTRL				(7)  ///< SRAM[0x00100007] CMD[0x8007]  
#define SPARA_TX_CTRL_H			(8)  ///< SRAM[0x00100007] CMD[0x8008]  

#define SPARA_FORCE_PNCHIPNUM			(0x22)  ///< SRAM[0x00100007] CMD[0x8002]  [4:0]:PnChip num ; [7]:Dual Tone enable
#define SPARA_FORCE_ADCTRL0				(0x23)  ///< SRAM[0x00100008] CMD[0x8003]  [6:4]:FrontEnd Gain = 0~7 (Small -> Big)
#define SPARA_FORCE_ADCTRL1				(0x24)  ///< SRAM[0x00100009] CMD[0x8004]  [6:4]:LpFilter Gain = 0~7 (Small -> Big)
#define SPARA_FORCE_ADCTRL10			(0x25)  ///< SRAM[0x0010000A] CMD[0x8005]  [4  ]:0=Buffer Mode,1:Scaleing Mode
#define SPARA_FORCE_CHGPUMP				(0x26)  ///< SRAM[0x0010000B] CMD[0x8006]  [0  ]:ChargePump 1:Enable , 0:Disable
#define SPARA_FORCE_PN_CTRL				(0x27)  ///< SRAM[0x00100007] CMD[0x8007]  
#define SPARA_FORCE_TX_CTRL_H			(0x28)  ///< SRAM[0x00100007] CMD[0x8008]  

///-------------------------------------------------------------------------------///
/// ZET7100 Finger Merge State Machine
///-------------------------------------------------------------------------------///
#define STATE_NULL  0x00
#define STATE_INIT  0x01
#define STATE_MERGE 0x02
#define STATE_DIVID 0x04

#define CAP_MODE  	0x00
#define FORCE_MODE  0x01
#define INIT_MODE   0x02
#define KEY_MODE  	0x03

///-------------------------------------------------------------------------------///
/// Linearity Compensation
///-------------------------------------------------------------------------------///
#define LINEARITY_COMPEN_SENSE_EN						(1<<0)		///< Sense axis compensation enable
#define LINEARITY_COMPEN_DRIVE_EN						(1<<1)		///< Drive axis compensation enable


///-------------------------------------------------------------------------------///
/// I2C/SPI Command working state
///-------------------------------------------------------------------------------///

#define WORKING_STATE_ZET_CMD_STATE 1
#define WORKING_STATE_CUSTOMER_INITIAL 2
#define WORKING_STATE_CUSTOMER_NORMAL 3

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// CUSTOMER PROTOCOL
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#ifdef FEATURE_CUSTOMER_PROTOCOL
//#define TPCUSTOMERCMD_D8       (0xD8)
//#define TPCUSTOMERCMD_58       (0x58)
#define  CUSTOMER_BUF_LEN  200 // 333 //  256

#define CUSTOMER_RESNED_NUM  2  //resend N-1 times,  need >=2
#endif
///-------------------------------------------------------------------------------///
/// Import Dev Frames Ctrl
///-------------------------------------------------------------------------------///
#define IMPORT_CTRL_WAIT_CMD	  (1<<0)
#define IMPORT_CTRL_START		 	 	(1<<1)

///-------------------------------------------------------------------------------///
/// Water Status 
///-------------------------------------------------------------------------------///
#define WATER_FOUND_ON_INIT						(1<<0)
#define WATER_FOUND_DETECT						(1<<1)
#define WATER_FINGER_ON_AREA					(1<<2)
#define WATER_FOUND_ON_INIT_DELAY			(1<<3)
#define WATER_FOUND_MUT_OUTSIDE_AREA	(1<<5)

#define NORMAL_MODE_PARA							(1<<0)
#define WATER_MODE_PARA								(2<<0)

#define WATER_FOUND_POSTPONE_ROUND				(5)   ///< if init found , will postpone n cycle

// interface device type
#define SPI_TYPE 1
#define I2C_TYPE 2

#define MAX_RX_BUF_LEN 20


///-------------------------------------------------------------------------------///
/// SPI Flash control 
///-------------------------------------------------------------------------------///
#if 1 //def SPI_FLASH_WR_ENABLE 
//SPI_BA ---0x19_C100
#define SPI_STATUS           0x19C100	        
#define SPI_ENABLE        (1L<<0)
#define SPI_SHIFT_START   (1L<<2)
#define SPI_CSB           (1l<<5)

#define SPI_SHIFT            0x19C104	       
#define SPI_CTL              0x19C108	        
#define SPI_DATA             0x19C10C	        
#define SPI_IF               0x19C110

//FLASH command
#define FLASH_WREN  0x06
#define FLASH_WRDI  0x04
#define FLASH_RDID  0x9F
#define FLASH_RD    0x03

#define FLASH_SE    0x20 //Sector Erase
#define FLASH_PP    0x02 //Page programing
#define FLASH_RDUID 0x4B

#endif
#endif ///< __DEFINE_H__
