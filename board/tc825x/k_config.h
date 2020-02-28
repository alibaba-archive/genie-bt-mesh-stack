/********************************************************************************************************
 * @file     k_config.h 
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

#ifndef CONFIG_H
#define CONFIG_H

/* chip level conf */
#ifndef RHINO_CONFIG_LITTLE_ENDIAN
#define RHINO_CONFIG_LITTLE_ENDIAN           1    //小端
#endif
#ifndef RHINO_CONFIG_CPU_STACK_DOWN
#define RHINO_CONFIG_CPU_STACK_DOWN          1    //栈向下增长
#endif

/* kernel feature conf */
#ifndef RHINO_CONFIG_SEM
#define RHINO_CONFIG_SEM                     1   // 信号量
#endif
#ifndef RHINO_CONFIG_QUEUE
#define RHINO_CONFIG_QUEUE                   1   //队列
#endif
#ifndef RHINO_CONFIG_TASK_SEM
#define RHINO_CONFIG_TASK_SEM                1   //任务信号量
#endif
#ifndef RHINO_CONFIG_EVENT_FLAG
#define RHINO_CONFIG_EVENT_FLAG              1   //事件标志
#endif
#ifndef RHINO_CONFIG_TIMER
#define RHINO_CONFIG_TIMER                   1   //定时器
#endif
#ifndef RHINO_CONFIG_BUF_QUEUE
#define RHINO_CONFIG_BUF_QUEUE               1  //缓冲队列
#endif
#ifndef RHINO_CONFIG_MM_BLK
#define RHINO_CONFIG_MM_BLK                  1
#endif
#ifndef RHINO_CONFIG_MM_DEBUG
#define RHINO_CONFIG_MM_DEBUG                1
#endif
#ifndef RHINO_CONFIG_MM_TLF
#define RHINO_CONFIG_MM_TLF                  1
#endif
#ifndef RHINO_CONFIG_MM_TLF_BLK_SIZE
#define RHINO_CONFIG_MM_TLF_BLK_SIZE         1024 // 2048
#endif
#ifndef RHINO_CONFIG_MM_MAXMSIZEBIT
#define RHINO_CONFIG_MM_MAXMSIZEBIT          19
#endif
#ifndef RHINO_CONFIG_GCC_RETADDR
#define RHINO_CONFIG_GCC_RETADDR             1
#endif
#ifndef RHINO_CONFIG_MM_LEAKCHECK
#define RHINO_CONFIG_MM_LEAKCHECK            0
#endif
#ifndef RHINO_CONFIG_RINGBUF_VENDOR
#define RHINO_CONFIG_RINGBUF_VENDOR          0   //环形缓冲
#endif

#ifndef RHINO_CONFIG_KOBJ_SET
#define RHINO_CONFIG_KOBJ_SET                1
#endif

/* kernel task conf */
#ifndef RHINO_CONFIG_TASK_SUSPEND
#define RHINO_CONFIG_TASK_SUSPEND            1   //任务挂起
#endif
#ifndef RHINO_CONFIG_TASK_INFO
#define RHINO_CONFIG_TASK_INFO               1  //任务信息
#endif
#ifndef RHINO_CONFIG_TASK_DEL
#define RHINO_CONFIG_TASK_DEL                1  //任务删除
#endif

#ifndef RHINO_CONFIG_TASK_STACK_CUR_CHECK
#define RHINO_CONFIG_TASK_STACK_CUR_CHECK    1
#endif

#ifndef RHINO_CONFIG_TASK_WAIT_ABORT
#define RHINO_CONFIG_TASK_WAIT_ABORT         1  //任务等待打断
#endif
#ifndef RHINO_CONFIG_TASK_STACK_OVF_CHECK
#define RHINO_CONFIG_TASK_STACK_OVF_CHECK    1
#endif
#ifndef RHINO_CONFIG_SCHED_RR
#define RHINO_CONFIG_SCHED_RR                1 //时间片轮转
#endif
#ifndef RHINO_CONFIG_TIME_SLICE_DEFAULT
#define RHINO_CONFIG_TIME_SLICE_DEFAULT      50 //时间片时间
#endif
#ifndef RHINO_CONFIG_PRI_MAX
#define RHINO_CONFIG_PRI_MAX                 62 //最大优先级
#endif
#ifndef RHINO_CONFIG_USER_PRI_MAX
#define RHINO_CONFIG_USER_PRI_MAX            (RHINO_CONFIG_PRI_MAX - 2)
#endif

/* kernel workqueue conf */
#ifndef RHINO_CONFIG_WORKQUEUE
#define RHINO_CONFIG_WORKQUEUE               1
#endif
#ifndef RHINO_CONFIG_WORKQUEUE_STACK_SIZE
#define RHINO_CONFIG_WORKQUEUE_STACK_SIZE    (512-256)
#endif

/* kernel mm_region conf */
#ifndef RHINO_CONFIG_MM_REGION_MUTEX
#define RHINO_CONFIG_MM_REGION_MUTEX         1
#endif

/* kernel timer&tick conf */
#ifndef RHINO_CONFIG_HW_COUNT
#define RHINO_CONFIG_HW_COUNT                0
#endif
#ifndef RHINO_CONFIG_TICK_TASK
#define RHINO_CONFIG_TICK_TASK               0
#endif

#if (RHINO_CONFIG_TICK_TASK > 0)
#ifndef RHINO_CONFIG_TICK_TASK_STACK_SIZE
#define RHINO_CONFIG_TICK_TASK_STACK_SIZE    256
#endif
#ifndef RHINO_CONFIG_TICK_TASK_PRI
#define RHINO_CONFIG_TICK_TASK_PRI           1
#endif
#endif

#ifndef RHINO_CONFIG_TICKLESS
#define RHINO_CONFIG_TICKLESS                0
#endif
#ifndef RHINO_CONFIG_TICKS_PER_SECOND
#define RHINO_CONFIG_TICKS_PER_SECOND        100
#endif
/* must be 2^n size!, such as 1, 2, 4, 8, 16,32, etc....... */
#ifndef RHINO_CONFIG_TICK_HEAD_ARRAY
#define RHINO_CONFIG_TICK_HEAD_ARRAY         8
#endif

/*must reserve enough stack size for timer cb will consume*/
#ifndef RHINO_CONFIG_TIMER_TASK_STACK_SIZE
#define RHINO_CONFIG_TIMER_TASK_STACK_SIZE   300
#endif
#ifndef RHINO_CONFIG_TIMER_RATE
#define RHINO_CONFIG_TIMER_RATE              1
#endif
#ifndef RHINO_CONFIG_TIMER_TASK_PRI
#define RHINO_CONFIG_TIMER_TASK_PRI          5
#endif

/* kernel intrpt conf */
#ifndef RHINO_CONFIG_INTRPT_STACK_REMAIN_GET
#define RHINO_CONFIG_INTRPT_STACK_REMAIN_GET 0
#endif
#ifndef RHINO_CONFIG_INTRPT_STACK_OVF_CHECK
#define RHINO_CONFIG_INTRPT_STACK_OVF_CHECK  0
#endif
#ifndef RHINO_CONFIG_INTRPT_MAX_NESTED_LEVEL
#define RHINO_CONFIG_INTRPT_MAX_NESTED_LEVEL 188u
#endif
#ifndef RHINO_CONFIG_INTRPT_GUARD
#define RHINO_CONFIG_INTRPT_GUARD            0
#endif

/* kernel dyn alloc conf */
#ifndef RHINO_CONFIG_KOBJ_DYN_ALLOC
#define RHINO_CONFIG_KOBJ_DYN_ALLOC          1
#endif

#if (RHINO_CONFIG_KOBJ_DYN_ALLOC > 0)
#ifndef RHINO_CONFIG_K_DYN_QUEUE_MSG
#define RHINO_CONFIG_K_DYN_QUEUE_MSG         30
#endif
#ifndef RHINO_CONFIG_K_DYN_TASK_STACK
#define RHINO_CONFIG_K_DYN_TASK_STACK        128
#endif
#ifndef RHINO_CONFIG_K_DYN_MEM_TASK_PRI
#define RHINO_CONFIG_K_DYN_MEM_TASK_PRI      6
#endif
#endif

/* kernel idle conf */
#ifndef RHINO_CONFIG_IDLE_TASK_STACK_SIZE
#define RHINO_CONFIG_IDLE_TASK_STACK_SIZE    128
#endif

/* kernel hook conf */
#ifndef RHINO_CONFIG_USER_HOOK
#define RHINO_CONFIG_USER_HOOK               0
#endif

/* kernel stats conf */
#ifndef RHINO_CONFIG_SYSTEM_STATS
#define RHINO_CONFIG_SYSTEM_STATS            1
#endif
#ifndef RHINO_CONFIG_SCHED_STATS
#define RHINO_CONFIG_SCHED_STATS             0
#endif
#ifndef RHINO_CONFIG_INTRPT_STATS
#define RHINO_CONFIG_INTRPT_STATS            0
#endif
#ifndef RHINO_CONFIG_TASK_SCHED_STATS
#define RHINO_CONFIG_TASK_SCHED_STATS        0
#endif

#ifndef RHINO_CONFIG_CPU_NUM
#define RHINO_CONFIG_CPU_NUM                 1
#endif

/* kernel trace conf */
#ifndef RHINO_CONFIG_TRACE
#define RHINO_CONFIG_TRACE                   0
#endif

#ifndef CONFIG_ADV_MIN_INTERVAL
#define CONFIG_ADV_MIN_INTERVAL              ADV_INT_FAST
#endif

#endif /* K_CONFIG_H */

