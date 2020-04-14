/**************************************************************************************************

  Phyplus Microelectronics Limited. All rights reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

**************************************************************************************************/



/*******************************************************************************
* @file		log.h
* @brief	Contains all functions support for uart driver
* @version	0.0
* @date		31. Jan. 2018
* @author	eagle.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#ifndef __LOG_H__
#define __LOG_H__
#include "uart.h"
int phy_printf(const char *format, ...);
void phy_printf_init(void);

#ifndef DEBUG_INFO
#error "DEBUG_INFO undefined!"
#endif

#if(DEBUG_INFO == 1)
#include <stdio.h>
#define LOG   printf
#define LOG_INIT() //phy_printf_init()
#else
#define LOG(...)
#define LOG_INIT()  {clk_gate_enable(MOD_UART);clk_reset(MOD_UART);clk_gate_disable(MOD_UART);}
#endif
#endif //__LOG_H__

