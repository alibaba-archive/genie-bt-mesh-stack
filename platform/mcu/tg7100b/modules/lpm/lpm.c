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
#include <yoc_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <k_api.h>
#include <aos/aos.h>
#include <yoc/lpm.h>


#include <drv_common.h>

/* rihino internal api */
int sys_soc_init(pm_ctx_t *pm_ctx);
int sys_soc_suspend(pm_ctx_t *pm_ctx);
int sys_soc_resume(pm_ctx_t *pm_ctx, int pm_state);

pm_ctx_t g_pm_ctx;

int32_t aos_kernel_suspend(void)
{
    if (is_klist_empty(&g_tick_head))
    {
        return -1;
    }

    ktask_t * p_tcb  = krhino_list_entry(g_tick_head.next, ktask_t, tick_list);
    return  p_tcb->tick_match > g_tick_count ?  p_tcb->tick_match - g_tick_count : 0;
}

void aos_kernel_resume(int32_t ticks)
{
    tick_list_update((tick_i_t)ticks);
    core_sched();
}

#define RHINO_OS_MS_PERIOD_TICK      (1000 / RHINO_CONFIG_TICKS_PER_SECOND)
uint64_t aos_kernel_tick2ms(uint32_t ticks)
{
    return ((uint64_t)ticks * RHINO_OS_MS_PERIOD_TICK);
}

uint64_t aos_kernel_ms2tick(uint32_t ms)
{
    if (ms < RHINO_OS_MS_PERIOD_TICK) {
        return 0;
    }

    return (((uint64_t)ms) / RHINO_OS_MS_PERIOD_TICK);
}



//static const char *TAG = "lpm";
static void lpm_handle(void)
{
    int pm_state;
    int ticks;
    CPSR_ALLOC();

    if (g_pm_ctx.policy == LPM_POLICY_NO_POWER_SAVE) {
        return;
    }

    //aos_kernel_sched_suspend();
    RHINO_CRITICAL_ENTER();

    g_pm_ctx.suspend_tick = aos_kernel_suspend();

    //how many pm_state is decided by SOC implement
    pm_state = sys_soc_suspend(&g_pm_ctx);
    ticks = sys_soc_resume(&g_pm_ctx, pm_state);

    if (ticks) {
        aos_kernel_resume(ticks);
    }

    RHINO_CRITICAL_EXIT();
    //aos_kernel_sched_resume();

    return;
}

void lpm_idle_hook(void){
    if (g_pm_ctx.lpm_handle) {
        g_pm_ctx.lpm_handle();
    }
}

/******************************************
// YoC PM APIs
******************************************/
void pm_init(void)
{
    memset(&g_pm_ctx, 0, sizeof(pm_ctx_t));
    aos_mutex_new(&g_pm_ctx.mutex);

    sys_soc_init(&g_pm_ctx);

    g_pm_ctx.lpm_handle = lpm_handle;
}

void pm_config_policy(int policy)
{
    aos_mutex_lock(&g_pm_ctx.mutex, AOS_WAIT_FOREVER);
    g_pm_ctx.policy = policy;
    aos_kv_setint(KV_LPM_POLICY, policy);
    aos_mutex_unlock(&g_pm_ctx.mutex);
}

//standby mode is managed by application, poweroff of a device is also managed by application.
//application should close device first, then agree deep sleep.
void pm_agree_halt(uint32_t ms)
{
    aos_mutex_lock(&g_pm_ctx.mutex, AOS_WAIT_FOREVER);
    if (ms < 100)
        ms = 100;

    g_pm_ctx.alarm_ms = ms;
    g_pm_ctx.agree_halt = 1;
    aos_mutex_unlock(&g_pm_ctx.mutex);
}
