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




#ifndef _HAL_CLOCK_H
#define _HAL_CLOCK_H

#include "types.h"
#include "hal_mcu.h"
#include "common.h"


typedef enum {
    CLK_32K_XTAL        = 0,
    CLK_32K_RCOSC       = 1,

} CLK32K_e;

void clk_gate_enable(MODULE_e module);
void clk_gate_disable(MODULE_e module);
void clk_reset(MODULE_e module);
uint32_t clk_hclk(void);
uint32_t clk_pclk(void);
void hal_rtc_clock_config(uint8_t clk32Mode);

void hal_rtc_clock_from_Xtal32K(void);

uint32_t hal_systick(void);
uint32_t hal_ms_intv(uint32_t tick);

#endif



