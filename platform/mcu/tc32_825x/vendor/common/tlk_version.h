/********************************************************************************************************
 * @file     tlk_version.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#pragma once

// can't use "enum" here, because cstartup.S not support

//#define   BUILD_VERSION  	"Revision: 45:49M"
//#define   BUILD_TIME  		"2012-07-24-21:37:43"

#include "tlk_config.h"

#define VERSION_GET(low, high)      ((low)|(high << 8))

#define FW_VERSION_TELINK_RELEASE   (VERSION_GET(0x31, 0x30))       // user don't modify


#define MESH_PID_SEL		(0)
#define MESH_VID		    FW_VERSION_TELINK_RELEASE       // user can redefine

#define BUILD_VERSION		(MESH_PID_SEL|(MESH_VID << 16))	// if value change, must make clean. same sequence with cps


