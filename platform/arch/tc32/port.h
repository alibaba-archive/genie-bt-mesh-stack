/********************************************************************************************************
 * @file     port.h 
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

#ifndef PORT_H
#define PORT_H

cpu_cpsr_t cpu_intrpt_save(void);
void cpu_intrpt_restore(cpu_cpsr_t cpsr);
void   cpu_intrpt_switch(void);
void   cpu_task_switch(void);
void   cpu_first_task_start(void);
void  *cpu_task_stack_init(cpu_stack_t *base, size_t size, void *arg, task_entry_t entry);

RHINO_INLINE uint8_t cpu_cur_get(void)
{
    return 0;
}

#define CPSR_ALLOC() size_t cpsr; (void)cpsr

#define RHINO_CPU_INTRPT_DISABLE() { *((volatile char*) (0x800643)) = 0; }
#define RHINO_CPU_INTRPT_ENABLE()  { *((volatile char*) (0x800643)) = 1; asm("tnop"); asm("tnop"); asm("tnop"); asm("tnop"); asm("tnop"); asm("tnop"); asm("tnop"); asm("tnop");}	//to ensure  timer interrupt triggered as expect

#endif /* PORT_H */

