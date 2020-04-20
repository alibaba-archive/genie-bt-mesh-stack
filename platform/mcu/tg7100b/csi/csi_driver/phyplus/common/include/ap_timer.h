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



#ifndef __AP_TIMER_H__
#define __AP_TIMER_H__

#include "OSAL.h"
#include "types.h"

//ap timer interrupt process function.
void __attribute__((weak)) AP_TIMER_IRQHandler(void);

//enable ap timer interrupt.
void ap_timer_init(void);

//disable ap timer interrupt and disable its clock.
void ap_timer_deinit(void);

//set timer registers,ready to run.
//timeId should form 1 to 4.time mutiple four should not more than 0xffffff.
void set_ap_timer(uint8  timeId, int time);

//clear timer registers,stop it.
void clear_ap_timer(uint8 timeId);


void mask_ap_timer_int(uint8  timeId, uint8_t en);
void set_ap_timer_enable(uint8  timeId, uint8_t en);

#endif

