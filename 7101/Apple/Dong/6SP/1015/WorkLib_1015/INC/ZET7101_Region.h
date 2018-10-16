/**
 * @file Region.h
 *
 *  Trace.h define the global variables
 *
 *
 * @version $Revision$
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#ifndef __REGION_H__
#define __REGION_H__

/// =============================================================///
/// MACRO DEFINITION
/// =============================================================///

/// =============================================================///
/// TYPE DEFINITION
/// =============================================================///
//#define TABLE_DEFINE_REGION_ROW	(1)
//#define MAX_REGION_ROUND       	(CHANNEL_MAX_NUM)
//#define MAX_REGION_ROW         	(CHANNEL_MAX_NUM)
#define REGION_NUM  						(CHANNEL_MAX_NUM) ///< MAX_TRACE_NUM / MAX_REGION_ROW ==>  there are 32/1 = 32 regions
//#define MAX_REGION_ROW_SHIFT  	(0)
//#define REGION_ROW_NUM  				(1)

#define REGION_INVALID        	(99)



#endif ///< __REGION_H__


