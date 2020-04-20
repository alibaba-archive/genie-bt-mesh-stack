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





#ifndef _HAL_PWRMGR_HD
#define _HAL_PWRMGR_HD

#include "hal_mcu.h"
#include "gpio.h"

#define PWR_MODE_NO_SLEEP 	1
#define PWR_MODE_SLEEP			2
#define PWR_MODE_PWROFF			3
#define PWR_MODE_PWROFF_NO_SLEEP			4


#define HAL_PWRMGR_TASK_MAX_NUM 10

#define   RET_SRAM0         BIT(0)  /*32K, 0x1fff0000~0x1fff7fff*/
#define   RET_SRAM1         BIT(1)  /*32K, 0x1fff8000~0x1fffffff*/
#define   RET_SRAM2         BIT(2)  /*64K, 0x20000000~0x2000ffff*/
#define   RET_SRAM3         BIT(3)  /*8K,  0x20010000~0x20011fff*/
#define   RET_SRAM4         BIT(4)  /*2K,  0x20012000~0x200127ff*/


typedef struct {
    GPIO_Pin_e pin;
    IO_Wakeup_Pol_e type;
} pwroff_cfg_t;

typedef void (*pwrmgr_Hdl_t)(void);

int hal_pwrmgr_init(void);
bool hal_pwrmgr_is_lock(MODULE_e mod);
int hal_pwrmgr_lock(MODULE_e mod);
int hal_pwrmgr_unlock(MODULE_e mod);
int hal_pwrmgr_register(MODULE_e mod, pwrmgr_Hdl_t sleepHandle, pwrmgr_Hdl_t wakeupHandle);
int hal_pwrmgr_unregister(MODULE_e mod);
int hal_pwrmgr_wakeup_process(void) __attribute__((weak));
int hal_pwrmgr_sleep_process(void) __attribute__((weak));
int hal_pwrmgr_RAM_retention(uint32_t sram);
int hal_pwrmgr_RAM_retention_clr(void);
int hal_pwrmgr_RAM_retention_set(void);
int hal_pwrmgr_LowCurrentLdo_enable(void);
int hal_pwrmgr_LowCurrentLdo_disable(void);

int hal_pwrmgr_poweroff(pwroff_cfg_t *pcfg, uint8_t wakeup_pin_num);
void system_on_handler(GPIO_Pin_e pin, uint32_t timer);


#endif


