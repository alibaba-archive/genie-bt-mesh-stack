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
 * @file     lib.c
 * @brief    source file for the lib
 * @version  V1.0
 * @date     23. May 2019
 ******************************************************************************/

#include <csi_config.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <soc.h>
#include <sys_freq.h>
#include <ARMCM0.h>

static void _500udelay(void)
{
    uint32_t load  = SysTick->LOAD;
    uint32_t start = SysTick->VAL;
    uint32_t cnt   = (drv_get_sys_freq() / 1000 / 2);

    while (1) {
        uint32_t cur = SysTick->VAL;

        if (start > cur) {
            if (start - cur >= cnt) {
                return;
            }
        } else {
            if (load - cur + start > cnt) {
                return;
            }
        }
    }
}

void mdelay(uint32_t ms)
{
    while (ms--) {
        _500udelay();
        _500udelay();
    }
}

static void _10udelay(void)
{
    uint32_t load  = SysTick->LOAD;
    uint32_t start = SysTick->VAL;
    uint32_t cnt   = (drv_get_sys_freq() / 1000 / 100);

    while (1) {
        uint32_t cur = SysTick->VAL;

        if (start > cur) {
            if (start - cur >= cnt) {
                return;
            }
        } else {
            if (load - cur + start > cnt) {
                return;
            }
        }
    }
}

void udelay(uint32_t us)
{
    us = (us / 10);

    while (us--) {
        _10udelay();
    }
}
