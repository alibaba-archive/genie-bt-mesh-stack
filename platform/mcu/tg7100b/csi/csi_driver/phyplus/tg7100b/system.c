/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     system.c
 * @brief    CSI Device System Source File
 * @version  V1.0
 * @date     10. April 2019
 ******************************************************************************/

#include <csi_config.h>
#include <soc.h>
#include <io.h>
#include <drv_irq.h>

#include "global_config.h"
//#include "jump_function.h"

#include "uart.h"
#include "adc.h"
#include "gpio.h"
#include "spi.h"
#include "common.h"
#include "pwrmgr.h"
#include "string.h"
#include "clock.h"
#include "log.h"
#include "ap_timer.h"

#include "ap_timer.h"

#ifndef CONFIG_SYSTICK_HZ
#define CONFIG_SYSTICK_HZ 1000
#endif

extern volatile uint8_t g_system_clk;
extern void mm_heap_initialize(void);
extern void init_config(void);
extern void hal_rfphy_init(void);

void hal_init(void)
{
    phy_gpio_init();

    init_config();

    hal_rfphy_init();

    hal_ret_sram_enable(RET_SRAM0 | RET_SRAM1 | RET_SRAM2 | RET_SRAM3 | RET_SRAM4);

    hal_rtc_clock_config(CLK_32K_RCOSC); //CLK_32K_XTAL

    subWriteReg(0x4000f014,26,26, 1);
}

/**
  * @brief  initialize the system
  *         Initialize the psr and vbr.
  * @param  None
  * @return None
  */

uint32 *pGlobal_config;
extern uint32 global_config[];

__attribute__((weak)) void SystemInit(void)
{
    /* Clear active and pending IRQ, and disable IRQ */
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;

    g_system_clk = SYS_CLK_DLL_48M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;

    pGlobal_config = &global_config[0];

    hal_init();

#ifndef CONFIG_KERNEL_NONE
    __disable_irq();
#else
    __enable_irq();
#endif

    SysTick_Config(48000);

#ifndef CONFIG_KERNEL_RHINO
#ifndef CONFIG_NUTTXMM_NONE
    mm_heap_initialize();
#endif
#endif
}
