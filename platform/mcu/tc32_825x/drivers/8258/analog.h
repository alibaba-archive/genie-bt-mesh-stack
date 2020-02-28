/********************************************************************************************************
 * @file     analog.h 
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

#include "compiler.h"


/**
 * @brief      This function serves to analog register read.
 * @param[in]  addr - address need to be read.
 * @return     the result of read.
 */
unsigned char analog_read(unsigned char addr);

/**
 * @brief      This function serves to analog register write.
 * @param[in]  addr - address need to be write.
 * @param[in]  v - the value to write.
 * @return     none.
 */
void analog_write(unsigned char addr, unsigned char v);


#define WriteAnalogReg			analog_write
#define ReadAnalogReg			analog_read



