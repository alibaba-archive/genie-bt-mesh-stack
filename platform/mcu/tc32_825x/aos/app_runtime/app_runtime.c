/********************************************************************************************************
 * @file     app_runtime.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#include <aos/aos.h>
#include "hal/soc/soc.h"

#ifdef AOS_BINS

const int *syscall_tbl = NULL;

extern unsigned int _app_data_ram_begin;
extern unsigned int _app_data_ram_end;
extern unsigned int _app_data_flash_begin;
extern unsigned int _app_bss_start;
extern unsigned int _app_bss_end;
extern unsigned int _app_heap_start;
extern unsigned int _app_heap_end;
extern unsigned int app_type;
extern int application_start(int argc, char **argv);

int aos_application_init(void)
{
    printf("aos application init.");
    return 0;
}

static void app_entry(void *ksyscall_tbl, int argc, char *argv[])
{
    /* syscall_tbl assignment must be first, every syscall need array address */
    syscall_tbl = (int *)ksyscall_tbl;

    aos_application_init();
    application_start(argc, argv);
}

__attribute__ ((used, section(".app_info"))) struct m_app_info_t app_info = {
    app_entry,
    &_app_data_ram_begin,
    &_app_data_ram_end,
    &_app_data_flash_begin,
    &_app_bss_start,
    &_app_bss_end,
    &_app_heap_start,
    &_app_heap_end,
    0x0,
    0x0,
    &app_type
};
#endif

