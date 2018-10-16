/**
 * @file ICSetup.h
 *
 *  Register definition
 *
 *
 * @version $Revision: 5 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/


//#ifndef __REGDEF_H__
//#define __REGDEF_H__

/// ===============================================================///
/// DEFINITION OF  8032 REGISTER
/// ===============================================================///

///-------------------------------------------------------------------------------///
///  RAM(0x00~0x7f) internal ram
///  SFR(0x80~0xff)  special function registers  
///-------------------------------------------------------------------------------///
#ifdef FEATURE_PROJECT_ZET7100
#include "ZET7100.h"
#define PROJECT_ZET7100
#define IC_NAME     "ZET7100" 
#define ROM_SIZE    (60*1024)
#define ROM_PCODE0  (0x11)
#define ROM_PCODE1  (0x01)  ///< Version Code -> 0x00
#define ROM_PCODE2  (0x02)  ///< Sub-Version Code
#define ROM_PCODE3  (B2_CONFIG_IDEV_16_BIT|B2_CONFIG_MDEV_16_BIT)  /// (0x08)  ///< Code Base -> 0x08 : Dynamic Mode
#define MAX_1T2R_EVEN_TRACE_NUM  (20)

#define PACKAGE_TYPE_A // A=P35 B=P21

//----- P35 -------
// bINTpinSel[0xE01C]
#define INT_CTRL_P3  0x40
#define INT_PIN_P5   P35_OUT
//----- P12 -------
#define INT_CTRL_P1  0x04
#define INT_PIN_P2   P12_OUT

#ifdef PACKAGE_TYPE_A
#define INT_IODIR  	REG32_IODIRP3 //REG32_IODIR 	0x0018C04C
#define INT_PORT    REG32_P3 //REG32_P1 					0x0018C040
#define INT_PIN 	  P35_OUT //P12_OUT
#endif

#ifdef PACKAGE_TYPE_B
#define INT_IODIR  	REG32_IODIR //REG32_IODIR			0x0018C010
#define INT_PORT    REG32_P1 //REG32_P1					0x0018C004
#define INT_PIN 	  P12_OUT //P12_OUT	
#endif



#define GPIO_IODIR  REG32_IODIR //REG32_IODIR
#define GPIO_PORT   REG32_P1 //REG32_IODIR
#define GPIO_PIN0   P12_OUT  //P35_OUT 
#define GPIO_PIN1   P11_OUT
#define GPIO_PIN2   P10_OUT
#endif

#ifdef FEATURE_PROJECT_ZET7101
#include "ZET7101.h"
#define PROJECT_ZET7101
#define IC_NAME     "ZET7101" 
#define ROM_SIZE    (52*1024)
#define ROM_PCODE0  (0x12)
#define ROM_PCODE1  (0x01)  ///< Version Code -> 0x00
#define ROM_PCODE2  (0x02)  ///< Sub-Version Code
#define ROM_PCODE3  (B2_CONFIG_IDEV_16_BIT|B2_CONFIG_MDEV_16_BIT)  /// (0x08)  ///< Code Base -> 0x08 : Dynamic Mode
#define MAX_1T2R_EVEN_TRACE_NUM  (20)

#define PACKAGE_TYPE_A // A=P35 B=P21

//----- P35 -------
// bINTpinSel[0xE01C]
#define INT_CTRL_P3  0x40
#define INT_PIN_P5   P35_OUT
//----- P12 -------
#define INT_CTRL_P1  0x04
#define INT_PIN_P2   P12_OUT

#define INT_CTRL_P2  0x20





#ifdef PACKAGE_TYPE_A
#define INT_IODIR  	REG32_IODIRP3 //REG32_IODIR 	0x0018C04C
#define INT_PORT    REG32_P3 //REG32_P1 					0x0018C040
#define INT_PIN 	  P35_OUT //P12_OUT
#endif

#ifdef PACKAGE_TYPE_B
#define INT_IODIR  	REG32_IODIR //REG32_IODIR			0x0018C010
#define INT_PORT    REG32_P1 //REG32_P1					0x0018C004
#define INT_PIN 	  P12_OUT //P12_OUT	
#endif



#define GPIO_IODIR  REG32_IODIR //REG32_IODIR
#define GPIO_PORT   REG32_P1 //REG32_IODIR
#define GPIO_PIN0   P12_OUT  //P35_OUT 
#define GPIO_PIN1   P11_OUT
#define GPIO_PIN2   P10_OUT
#endif

//#endif  ///< __REGDEF_H__

