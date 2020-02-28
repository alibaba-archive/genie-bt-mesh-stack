/********************************************************************************************************
 * @file     soc_impl.c 
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

#include <k_api.h>
#include <assert.h>
#include <stdio.h>
#include <sys/time.h>
#include "application/print/putchar_sim.h"


#if (RHINO_CONFIG_HW_COUNT > 0)
void soc_hw_timer_init(void)
{
}

hr_timer_t soc_hr_hw_cnt_get(void)
{
    return 0;
    //return *(volatile uint64_t *)0xc0000120;
}

lr_timer_t soc_lr_hw_cnt_get(void)
{
    return 0;
}

float soc_hr_hw_freq_mhz(void)
{
}

#endif /* RHINO_CONFIG_HW_COUNT */

#if (RHINO_CONFIG_INTRPT_STACK_OVF_CHECK > 0)
void soc_intrpt_stack_ovf_check(void)
{
}
#endif

#if (RHINO_CONFIG_MM_TLF > 0)

#if defined (__CC_ARM) /* Keil / armcc */
extern unsigned int Image$$RW_IRAM1$$ZI$$Limit;
//comes from component board, may different to each other as hardware resource
extern size_t g_iram1_start;
extern size_t g_iram1_total_size;
k_mm_region_t g_mm_region[1];
int           g_region_num = 1;
void aos_heap_set(void)
{
    g_mm_region[0].start = (uint8_t*)&Image$$RW_IRAM1$$ZI$$Limit;
    g_mm_region[0].len   = 
        (g_iram1_start + g_iram1_total_size - (size_t)&Image$$RW_IRAM1$$ZI$$Limit);
	  printf("g_mm_region[0].start is 0x%x, g_mm_region[0].len is 0x%x \r\n", (size_t)g_mm_region[0].start, g_mm_region[0].len);
}
#elif defined (__ICCARM__)/* IAR */
#define HEAP_BUFFER_SIZE 1024*20
uint8_t g_heap_buf[HEAP_BUFFER_SIZE];
k_mm_region_t g_mm_region[] = {{g_heap_buf, HEAP_BUFFER_SIZE}};
int           g_region_num = 1;
void aos_heap_set(void)
{
}
#else /* GCC */
#include "tlk_config.h"

extern void         *_estack;
extern void         *_end_bss_;
extern void         *_no_retention_bss_end_;
/* __bss_end__ and _estack is set by linkscript(*.ld)
   heap and stack begins from __bss_end__ to _estack */
k_mm_region_t g_mm_region[1];
int           g_region_num = 1;
//static unsigned char tmp[0x200];
void aos_heap_set(void)
{
    g_mm_region[0].start  = (uint8_t*)&_no_retention_bss_end_;
    int max_size = (TC_RAM_START_ADDR + TC_RAM_SIZE_MAX - (256+32)) - ((uint32_t)(g_mm_region[0].start)); // - stack of main() need 240bytes, and it will use again when retention wakeup.
    // can't printf here, because clock_init_ have not run before.
    g_mm_region[0].len    = max_size; // 0x3200;
}
/*static unsigned char tmp[0x200];
void aos_heap_set(void)
{
    g_mm_region[0].start = (uint8_t*)tmp;
    g_mm_region[0].len   = 0x200;
}*/
#endif
#endif

void aos_heap_check_log()   // must after tc816x_hal_init_ when call this
{
    int max_size = (int)g_mm_region[0].len;
    printf("heap start:%p, end:%p, size:%d\r\n", g_mm_region[0].start, g_mm_region[0].start+max_size,max_size);
    if(max_size < 1024 * 1){
        printf("error: heap size is not enough !!\r\n", max_size);
        //while(1);   // 
    }
}

#if (RHINO_CONFIG_MM_LEAKCHECK > 0 )

extern int __bss_start__, __bss_end__, _sdata, _edata;

void aos_mm_leak_region_init(void)
{
#if (RHINO_CONFIG_MM_DEBUG > 0)
    krhino_mm_leak_region_init(&__bss_start__, &__bss_end__);
    krhino_mm_leak_region_init(&_sdata, &_edata);
#endif
}

#endif


#if (RHINO_CONFIG_TASK_STACK_CUR_CHECK > 0)
__attribute__((noinline)) size_t soc_get_cur_sp()   // must no inline, if not, return value is always 0.
{
    asm("tmov		r0, r13");			// r13: task SP;
}

static void soc_print_stack()
{
    void    *cur, *end;
    int      i=0;
    int     *p;

    end   = krhino_cur_task_get()->task_stack_base + krhino_cur_task_get()->stack_size;
    cur = (void *)soc_get_cur_sp();
    p = (int*)cur;
    while(p < (int*)end) {
        if(i%4==0) {
            printf("\r\n%08x:",(uint32_t)p);
        }
        printf("%08x ", *p);
        i++;
        p++;
    }
    printf("\r\n");
    return;
}
#endif

void soc_err_proc(kstat_t err)
{
    (void)err;
    
    #if (RHINO_CONFIG_TASK_STACK_CUR_CHECK > 0)
    soc_print_stack();
    #endif
    assert(0);
}

krhino_err_proc_t g_err_proc = soc_err_proc;

