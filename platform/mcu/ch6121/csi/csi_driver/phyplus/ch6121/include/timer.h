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



#ifndef __TIMER_H__
#define __TIMER_H__

#include "OSAL.h"
#include "types.h"

//void wait_for_time(int time);

void init_timer(void);

void set_timer(uint8 timeId, int time);

//int  clearTimerInt(AP_TIM_TypeDef *TIMx);
//void clear_timer(AP_TIM_TypeDef *TIMx);

void set_timer1(int time);
void set_timer2(int time);
void set_timer3(int time);
void clear_timer1(void);
void clear_timer2(void);

uint32_t read_current_fine_time(void);

// add by HZF
uint32 getMcuPrecisionCount(void);

uint32  read_LL_remainder_time(void);


extern int   current_base_time;
extern int   current_fine_time;

#define BASE_TIME_UNITS   1000000

#endif

