/********************************************************************************************************
 * @file     compiler.h 
 *
 * @brief    This is the header file for TLSR8258
 *
 * @author	 public@telink-semi.com;
 * @date     May 8, 2018
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#pragma once


#define _attribute_packed_		__attribute__((packed))
#define _attribute_aligned_(s)	__attribute__((aligned(s)))
#define _attribute_session_(s)	__attribute__((section(s)))
#define _attribute_ram_code_  	_attribute_session_(".ram_code")
#define _attribute_custom_code_  	_attribute_session_(".custom") volatile
#define _attribute_no_inline_   __attribute__((noinline)) 

#define _inline_ 				inline				//   C99 meaning


#ifndef	BLC_PM_DEEP_RETENTION_MODE_EN
#define	BLC_PM_DEEP_RETENTION_MODE_EN					1
#endif

#if (BLC_PM_DEEP_RETENTION_MODE_EN)
//#define _attribute_data_retention_   __attribute__((section(".retention_data")))
// #define _attribute_bss_retention_    __attribute__((section(".retention_bss")))          // not use now, always take a space in firmware for private section.
#define _attribute_no_retention_data_   __attribute__((section(".no_ret_data")))
#define _attribute_no_retention_bss_   __attribute__((section(".no_ret_bss")))
#endif

#ifndef _attribute_data_retention_
#define _attribute_data_retention_
#endif
#ifndef _attribute_bss_retention_
#define _attribute_bss_retention_       //
#endif
#ifndef _attribute_no_retention_data_
#define _attribute_no_retention_data_
#endif
#ifndef _attribute_no_retention_bss_
#define _attribute_no_retention_bss_    //
#endif

