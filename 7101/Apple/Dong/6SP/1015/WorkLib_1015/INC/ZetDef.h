/**
 * @file Zetdef.h
 *
 *  The global header file for all the source files
 *
 *  The header file gathers the following header files
 *  It includes:
 *  1. standard header
 *  2. type definition
 *  3. vender definition
 *  4. register for the system
 *  5. macro definition
 *  6. global variables definition
 *  7. TP command definition
 *  8. memory definition
 *  8. global function definition
 *
 * @version $Revision: 5 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __ZEITEC_H__
#define __ZEITEC_H__

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <nds32_intrinsic.h>
#include <nds32_isr.h>
#include "Nds.h"
#include <stdint.h>
/// definition for  function and vender features
#include "Vender.h"
/// definition for macros
#include "Define.h"
/// definition for types
#include "TypeDef.h"
/// definition for 
#include "ICSetup.h"
/// definition for global variables
#include "VarDef.h"
/// definition for global function
#include "FuncDef.h"
/// trace setup 
#ifdef FEATURE_PROJECT_ZET7101
/// trace setup 
#include "ZET7101_Trace.h"
#else
/// trace setup 
#include "ZET7100_Trace.h"
#endif

/// Device Name : Gen from compile-batch 
#include "ICDeviceName.h"
/// definition for Dataflash Ctrl bit
#include "DFBitDefine.h"
/// definition for Sram Ctrl bit
#include "SramBitDefine.h"

#endif ///<  for __ZEITEC_H__
