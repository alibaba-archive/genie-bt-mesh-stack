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

#ifndef YOC_LPM_H
#define YOC_LPM_H

#include <time.h>
#include <aos/list.h>
#include <aos/aos.h>
#include <drv_gpio.h>

#define KV_LPM_POLICY   "lpm_policy"
#define KV_LPM_LS_THRES "lpm_ls"
#define KV_LPM_DS_THRES "lpm_ds"

typedef enum {
    LPM_POLICY_NO_POWER_SAVE,
    LPM_POLICY_LOW_POWER,
    LPM_POLICY_DEEP_SLEEP,
} pm_policy_t;

typedef struct {
    slist_t     next;

    void *dev;
    void *param;
} pm_dev_node_t;

typedef struct {
    int         policy;     //0: no low power, 1: low power, 2: support deep sleep
    uint32_t    agree_halt; //execution context will be lost
    uint32_t    alarm_ms;
    uint32_t    ls_ms;      //low power sleep threshold
    uint32_t    ds_ms;      //deep sleep threshold
    uint32_t    suspend_tick;
    aos_mutex_t mutex;
    void        (*lpm_handle)(void);
} pm_ctx_t;

typedef enum  {
    LPM_POWER_OFF,                        ///< Power off: no operation possible
    LPM_POWER_LOW,                        ///< Low Power mode: retain state, detect and signal wake-up events
    LPM_POWER_FULL,                       ///< Power on: full operation at maximum performance
    LPM_POWER_SUSPEND,                    ///< Power suspend: power saving operation
} lpm_power_stat_e;

void pm_init(void);
void pm_config_policy(int policy);
void pm_agree_halt(uint32_t ms);
void setup_wake_irq(gpio_pin_handle_t handle, gpio_irq_mode_e mode);
#endif
