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




#ifndef __RF_LIB_H
#define __RF_LIB_H

int __attribute__((weak)) TIM0_IRQHandler1(void);
void __attribute__((weak)) LL_IRQHandler1(void);

void rflib_vesion(uint8_t *major, uint8_t *minor, uint8_t *revision, char *test_build);


#endif

