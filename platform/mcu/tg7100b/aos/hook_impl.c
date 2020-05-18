/*
 * Copyright (C) 2016 YunOS Project. All rights reserved.
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

#include <k_api.h>
#include <drv_timer.h>
#include <csi_config.h>
#include <csi_core.h>
/* auto define heap size */
extern size_t __heap_start;
extern size_t __heap_end;
extern k_mm_region_t   g_mm_region[];

extern void soc_hw_timer_init(void);

#if (RHINO_CONFIG_USER_HOOK > 0)
void krhino_init_hook(void)
{
#if (RHINO_CONFIG_HW_COUNT > 0)
    soc_hw_timer_init();
#endif

    /* auto define heap size */
    //g_mm_region[0].len = (uint32_t)(&__heap_end) - (uint32_t)(&__heap_start);
}

void __attribute__((weak)) lpm_idle_pre_hook(void)
{
}


__attribute__((weak)) void krhino_start_hook(void)
{

}

__attribute__((weak)) void krhino_task_create_hook(ktask_t *task)
{

}

__attribute__((weak)) void krhino_task_del_hook(ktask_t *task, res_free_t *arg)
{
}

__attribute__((weak)) void krhino_task_abort_hook(ktask_t *task)
{

}

__attribute__((weak)) void krhino_task_switch_hook(ktask_t *orgin, ktask_t *dest)
{

}


__attribute__((weak)) void krhino_tick_hook(void)
{

}

__attribute__((weak)) void krhino_idle_pre_hook(void)
{
    lpm_idle_pre_hook();
}

__attribute__((weak)) int32_t _sleep_tick_get()
{
    if (is_klist_empty(&g_tick_head))
    {
        return -1;
    }

    ktask_t * p_tcb  = krhino_list_entry(g_tick_head.next, ktask_t, tick_list);
    return  p_tcb->tick_match > g_tick_count ?  p_tcb->tick_match - g_tick_count : 0;
}


__attribute__((weak)) void krhino_idle_hook(void)
{
    extern void lpm_idle_hook(void);
    lpm_idle_hook();
}

__attribute__((weak)) void krhino_intrpt_hook(int irq)
{

}

__attribute__((weak)) void krhino_mm_alloc_hook(void *mem, size_t size)
{

}

#endif

__attribute__((weak)) void krhino_intrpt_enter_hook(int irq)
{

}

__attribute__((weak)) void krhino_intrpt_exit_hook(int irq)
{

}


