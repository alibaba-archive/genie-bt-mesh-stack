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


#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#include "types.h"
#include "ap_cp.h"

//watchdog reset cycyle is 2s(1.999969482421875).
void Watchdog_Init(uint8 task_id);
uint16 Watchdog_ProcessEvent(uint8 task_id, uint16 events);

#endif
