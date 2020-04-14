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
 * @file     sys_freq.c
 * @brief    source file for setting system frequency.
 * @version  V1.0
 * @date     17. May 2019
 ******************************************************************************/

#include <csi_config.h>
#include <stdint.h>
#include <soc.h>
#include <io.h>
#include <clock.h>
#include <sys_freq.h>

extern volatile uint8_t g_system_clk;

void drv_set_sys_freq(clk_src_e source, clk_val_e val)
{
}

void drv_rtcclk_config(clk_src_e source)
{
    if (source == ILS_CLK) {
        hal_rtc_clock_config(CLK_32K_RCOSC);
    } else {
        hal_rtc_clock_config(CLK_32K_XTAL);
    }
}

int32_t drv_get_sys_freq(void)
{
    switch (g_system_clk) {
        case SYS_CLK_XTAL_16M:
            return 16000000;

        case SYS_CLK_RC_32M:
        case SYS_CLK_DLL_32M:
        case SYS_CLK_DBL_32M:
            return 32000000;

        case SYS_CLK_DLL_48M:
            return 48000000;

        case SYS_CLK_DLL_64M:
            return 64000000;

        case SYS_CLK_DLL_96M:
            return 96000000;

        default:
            return 32000000;
    }
}

int32_t drv_get_apb_freq(void)
{
    return drv_get_sys_freq();
}

int32_t drv_get_timer_freq(void)
{
    return 4000000;
}

int32_t drv_get_rtc_freq(void)
{
    return 32768;
}
