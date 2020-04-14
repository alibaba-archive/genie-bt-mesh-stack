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



/**************************************************************************************************
  Filename:       hal_mcu.h
  Revised:
  Revision:

  Description:    Describe the purpose and contents of the file.



**************************************************************************************************/

#ifndef _HAL_MCU_H
#define _HAL_MCU_H



/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_defs.h"
#include "types.h"
#include <stdint.h>
#include <cmsis_gcc.h>
#include "csi_core.h"

/* ------------------------------------------------------------------------------------------------
 *                                        Target Defines
 * ------------------------------------------------------------------------------------------------
 */

#define MAXMEMHEAP 4096

typedef enum {
    MOD_NONE = 0,           MOD_SOFT_RESET      = 0, MOD_CPU   = 0,
    MOD_LOCKUP_RESET_EN = 1,
    MOD_WDT_RESET_EN    = 2,
    MOD_DMA     = 3,
    MOD_AES     = 4,
    MOD_TIMER   = 5,
    MOD_WDT     = 6,
    MOD_COM     = 7,
    MOD_UART    = 8,
    MOD_I2C0    = 9,
    MOD_I2C1    = 10,
    MOD_SPI0    = 11,
    MOD_SPI1    = 12,
    MOD_GPIO    = 13,
    MOD_I2S     = 14,
    MOD_QDEC    = 15,
    MOD_RNG     = 16,
    MOD_ADCC    = 17,
    MOD_PWM     = 18,
    MOD_SPIF    = 19,
    MOD_VOC     = 20,
    MOD_KSCAN   = 31,
    MOD_USR0    = 32,
    MOD_USR1    = 33,
    MOD_USR2    = 34,
    MOD_USR3    = 35,
    MOD_USR4    = 36,
    MOD_USR5    = 37,
    MOD_USR6    = 38,
    MOD_USR8    = 39,
} MODULE_e;

/* ------------------------------------------------------------------------------------------------
 *                                     Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                        Interrupt Macros
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_ISER   *((volatile uint32_t *)(0xe000e100))
#define HAL_ICER   *((volatile uint32_t *)(0xe000e180))

#define HAL_ENABLE_INTERRUPTS()         st( __enable_irq(); )
#define HAL_DISABLE_INTERRUPTS()        st( __disable_irq(); )
#define HAL_INTERRUPTS_ARE_ENABLED()    (HAL_ISER)

typedef uint32_t halIntState_t;
//#define HAL_ENTER_CRITICAL_SECTION(x)   st()
//#define HAL_EXIT_CRITICAL_SECTION(x)    st()
//#define HAL_CRITICAL_STATEMENT(x)       st()

//#define HAL_ENTER_CRITICAL_SECTION(x)   st(x = HAL_ISER; HAL_ICER = 0xffffffff;)
//#define HAL_EXIT_CRITICAL_SECTION(x)    st( HAL_ISER = x; )
//#define HAL_CRITICAL_STATEMENT(x)       st( halIntState_t _s; HAL_ENTER_CRITICAL_SECTION(_s); x; HAL_EXIT_CRITICAL_SECTION(_s); )

//#define HAL_ENTER_CRITICAL_SECTION()  __disable_irq()
//#define HAL_EXIT_CRITICAL_SECTION()   __enable_irq()

#define HAL_ENTER_CRITICAL_SECTION(x)   (x = csi_irq_save())
#define HAL_EXIT_CRITICAL_SECTION(x)    (csi_irq_restore(x))

/**************************************************************************************************
 */
#endif
