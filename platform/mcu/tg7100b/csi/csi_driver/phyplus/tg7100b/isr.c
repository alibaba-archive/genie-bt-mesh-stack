/*
 * Copyright (C) 2017-2019 C-SKY Microsystems Co., Ltd. All rights reserved.
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
 * @file     isr.c
 * @brief    source file for the interrupt server route
 * @version  V1.0
 * @date     15. May 2019
 ******************************************************************************/

#include <drv_common.h>
#include <csi_config.h>
#include "soc.h"
#ifdef CONFIG_KERNEL_RHINO
#include <k_api.h>
#endif

#ifdef CONFIG_KERNEL_RHINO
#define CSI_INTRPT_ENTER() krhino_intrpt_enter()
#define CSI_INTRPT_EXIT()  krhino_intrpt_exit()
#else
#define CSI_INTRPT_ENTER()
#define CSI_INTRPT_EXIT()
#endif

extern void systick_handler(void);
extern void dw_usart_irqhandler(int32_t idx);
extern void dw_gpio_irqhandler(int32_t idx);
extern void dw_timer_irqhandler(int32_t idx);
extern void dw_spi_irqhandler(int32_t idx);
extern void phy_rtc_irqhandler(int32_t idx);
extern void dw_wdt_irqhandler(int32_t idx);
extern void dw_iic_irqhandler(int32_t idx);

__WEAK void PendSV_Handler(void)
{
}

__WEAK void LL_IRQHandler(void)
{
}

void CSI_SysTick_Handler(void)
{
    CSI_INTRPT_ENTER();
#ifndef CONFIG_KERNEL_NONE
    systick_handler();
#endif
    CSI_INTRPT_EXIT();
}

void CSI_UART0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_usart_irqhandler(0);
    CSI_INTRPT_EXIT();
}

void CSI_GPIO_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_gpio_irqhandler(0);
    CSI_INTRPT_EXIT();
}

void CSI_AP_TIMER_IRQHandler(void)
{
    if (AP_TIM1->status & 0x1) {
        dw_timer_irqhandler(0);
    }

    if (AP_TIM2->status & 0x1) {
        dw_timer_irqhandler(1);
    }

    if (AP_TIM3->status & 0x1) {
        dw_timer_irqhandler(2);
    }

    if (AP_TIM4->status & 0x1) {
        dw_timer_irqhandler(3);
    }
}

__WEAK void CSI_TIM0_IRQHandler(void)
{
}

void CSI_RTC_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    phy_rtc_irqhandler(0);
    CSI_INTRPT_EXIT();
}

void CSI_WDT_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_wdt_irqhandler(0);
    CSI_INTRPT_EXIT();
}
void CSI_SPI0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_spi_irqhandler(0);
    CSI_INTRPT_EXIT();
}

void CSI_SPI1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_spi_irqhandler(1);
    CSI_INTRPT_EXIT();
}

void CSI_IIC0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_iic_irqhandler(0);
    CSI_INTRPT_EXIT();
}

void CSI_IIC1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_iic_irqhandler(1);
    CSI_INTRPT_EXIT();
}
