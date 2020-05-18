/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "aos/kernel.h"
#include <k_api.h>
#include "aos/init.h"
#include <aos/kernel.h>
#include <ble_config.h>
#include <stdio.h>
#include <stdlib.h>

extern void board_base_init(void);
#define INIT_TASK_STACK_SIZE 2048
static cpu_stack_t app_stack[INIT_TASK_STACK_SIZE / 4] __attribute((section(".data")));

ktask_t *g_aos_init;
krhino_err_proc_t g_err_proc = soc_err_proc;
size_t soc_get_cur_sp()
{
    volatile size_t dummy = (size_t)&dummy;
    return dummy;
}

#if (RHINO_CONFIG_HW_COUNT > 0)
hr_timer_t soc_hr_hw_cnt_get(void)
{
    return 0;
}
#endif

#define HEAP_BUFF_SIZE (1024*8)
uint32_t g_heap_buff[HEAP_BUFF_SIZE / 4];
extern volatile uint32_t __heap_end;
extern volatile uint32_t __heap_start;
k_mm_region_t g_mm_region[1];
int           g_region_num  = sizeof(g_mm_region) / sizeof(k_mm_region_t);

void soc_err_proc(kstat_t err)
{
    printf("kernel panic,err %d!\n", err);

}

uint32_t aos_get_version_info(void)
{
    return 0;
}

static kinit_t kinit;
void board_cli_init(void)
{
    kinit.argc = 0;
    kinit.argv = NULL;
    kinit.cli_enable = 1;
}

extern void app_main();
static void application_task_entry(void *arg)
{
    board_base_init();
    board_cli_init();

#ifdef AOS_KV
    aos_kv_init();
#endif

#ifdef CONFIG_AOS_CLI
    aos_cli_init();
#endif

#ifdef AOS_LOOP
    aos_loop_init();
#endif

    aos_components_init(&kinit);
#ifndef AOS_BINS
    application_start(kinit.argc, kinit.argv); /* jump to app/example entry */
#endif
}

int main(void)
{
    g_mm_region[0].start = g_heap_buff;
    g_mm_region[0].len = HEAP_BUFF_SIZE;

    memset(g_mm_region[0].start, 0, g_mm_region[0].len);

    aos_init();

    //pm_init();

    ktask_t app_task_handle = {0};
    /* init task */
    krhino_task_create(&app_task_handle, "aos-init", NULL,
                       AOS_DEFAULT_APP_PRI, 0, app_stack,
                       INIT_TASK_STACK_SIZE / 4, application_task_entry, 1);
    aos_start();
    return 0;
}

