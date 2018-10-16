
/**
 * @file Nds.h
 *
 * nds reset functions
 *
 *
 * @version $Revision: 21 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __NDS_H__
#define __NDS_H__

// Please put the defines shared by makefile projects and AndeSight projects

#ifndef	CFG_MAKEFILE

// The defines are only used by AndeSight projects

//----------------------------------------------------------------------------------------------------

// Users can configure the defines in this area
// to match different environment setting


#define	CFG_16MB		// platform is 16MB, if it isn't defined, platform is 4GB

#define BUILD_MODE	BUILD_XIP	//BUILD_BURN


//----------------------------------------------------------------------------------------------------
// The followings are predefined settings
// Please do not modify them

#define	BUILD_LOAD	1		// The program is loaded by GDB or eBIOS
#define	BUILD_BURN	2		// The program is burned to the flash, but run in RAM
							// demo-ls2 use BURN mode
#define	BUILD_XIP	3		// The program is burned to the flash and run in the flash
							// To use this mode, xip linker script files must be used
							// demo-ls1 use XIP mode
#define	BUILD_SIMU	4		// The program is run in the simulation environment

#ifdef DEMO_LS1
	// demo-ls1 needs to use the XIP mode
	#undef BUILD_MODE
	#define BUILD_MODE	BUILD_XIP
#endif

#ifdef DEMO_LS2
	// demo-ls2 needs to use the BURN mode
	#undef BUILD_MODE
	#define BUILD_MODE	BUILD_BURN
#endif

#ifdef DEMO_LS3
	// demo-ls3 needs to use the XIP mode without RAM
	#undef BUILD_MODE
	#define BUILD_MODE	BUILD_XIP
	#define CFG_NORAM
#endif

#ifndef NDS32_EXT_EX9
	// This toolchain cannot support EX9
	#define CONFIG_NO_NDS32_EXT_EX9
#endif

#if BUILD_MODE == BUILD_BURN
	// Burn mode
	#define CFG_LLINIT      // do low level init
	#define CFG_REMAP       // do remap
#elif BUILD_MODE == BUILD_XIP
	// XIP mode
	#define CFG_LLINIT      // do low level init
#elif BUILD_MODE == BUILD_SIMU
	// Simu mode
	#define	CFG_SIMU
	#define CFG_LLINIT      // do low level init
#else
	// Load mode
#endif

#ifdef NDS32_BASELINE_V3M
	// v3m toolchain only support 16MB
	#undef CFG_16MB
	#define CFG_16MB
#endif

#endif // CFG_MAKEFILE

///---------------------------------------------------///
/// Use compiler builtin memcpy and memset 
///---------------------------------------------------///
#define MEMCPY(des, src, n) __builtin_memcpy ((des), (src), (n))
#define MEMSET(s, c, n)     __builtin_memset ((s), (c), (n))

#endif // __NDS_H__

