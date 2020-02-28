/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

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

