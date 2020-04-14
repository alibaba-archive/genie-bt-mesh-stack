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

/**************************************************************************//**
 * @file     soc.h
 * @brief    CSI Core Peripheral Access Layer Header File for
 *           CSKYSOC Device Series
 * @version  V1.0
 * @date     15. May 2019
 ******************************************************************************/

#ifndef _SOC_H_
#define _SOC_H_

#include <ARMCM0.h>
#include <ap_cp.h>
#include <sys_freq.h>
#include <hal_mcu.h>

#ifndef IHS_VALUE
#define  IHS_VALUE    (8000000)
#endif

#ifndef EHS_VALUE
#define  EHS_VALUE    (8000000)
#endif

#ifndef ILS_VALUE
#define  ILS_VALUE    (5000)
#endif

#ifndef ELS_VALUE
#define  ELS_VALUE    (16384)
#endif

#define CSKY_UART0_BASE  AP_UART0_BASE


#define UART0_IRQn UART_IRQ

#define CONFIG_USART_NUM 1
#define CONFIG_GPIO_NUM  2
#define CONFIG_SPI_NUM      2
#define CONFIG_IIC_NUM      2

#define CONFIG_GPIO_PIN_NUM 35
#define CONFIG_TIMER_NUM 4
#define CONFIG_RTC_NUM 1
#define CONFIG_WDT_NUM 1
#define CONFIG_TRNG_NUM 1
#define CONFIG_AES_NUM 1
#define CONFIG_PMU_NUM  1
#define CONFIG_ADC_NUM 1

#endif  /* _SOC_H_ */
