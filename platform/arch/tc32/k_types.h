/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef TYPES_H
#define TYPES_H

#if 0
#define RHINO_NO_WAIT                0
#define RHINO_WAIT_FOREVER           0xffffffff     /* 32 bit value, if tick type is 64 bit, you need change it to 64 bit */
#define RHINO_TASK_STACK_OVF_MAGIC   0xdeadbeaf     /* 32 bit or 64 bit stack overflow magic value */
#define RHINO_INTRPT_STACK_OVF_MAGIC 0xdeaddead     /* 32 bit or 64 bit stack overflow magic value */
#define RHINO_MM_FRAG_ALLOCATED      0xabcddcab     /* 32 bit value, if 64 bit system, you need change it to 64 bit */
#define RHINO_MM_FRAG_FREE           0xfefdecdb     /* 32 bit value, if 64 bit system, you need change it to 64 bit */
#define RHINO_INLINE                 static __inline /* inline keyword, it may change under different compilers */

typedef char     name_t;
typedef unsigned long sem_count_t;
typedef unsigned long cpu_stack_t;

typedef unsigned long hr_timer_t;
typedef unsigned long lr_timer_t;

typedef unsigned long tick_t;
typedef unsigned long long idle_count_t;
typedef unsigned long long sys_time_t;
typedef unsigned long mutex_nested_t;
typedef unsigned char  suspend_nested_t;

typedef unsigned long long ctx_switch_t;

/* keil compiler not define ssize_t */
#ifdef __CC_ARM
typedef int32_t ssize_t;
#endif

#endif

#define RHINO_TASK_STACK_OVF_MAGIC   0xdeadbeafu     /* 32 bit or 64 bit stack overflow magic value */
#define RHINO_INTRPT_STACK_OVF_MAGIC 0xdeaddeadu     /* 32 bit or 64 bit stack overflow magic value */
#define RHINO_MM_CORRUPT_DYE         0xFEFEFEFE
#define RHINO_MM_FREE_DYE            0xABABABAB
#define RHINO_INLINE                 static inline /* inline keyword, it may change under different compilers */

typedef char     name_t;
typedef unsigned long sem_count_t;
typedef unsigned long cpu_stack_t;
typedef unsigned long hr_timer_t;
typedef unsigned long lr_timer_t;
typedef unsigned long mutex_nested_t;
typedef unsigned char  suspend_nested_t;
typedef unsigned long long ctx_switch_t;
typedef unsigned long cpu_cpsr_t;


#endif /* TYPES_H */

