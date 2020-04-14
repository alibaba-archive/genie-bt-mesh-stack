/*
 * Copyright (C) 2017-2019 C-SKY Microsystems Co., Ltd. All rights reserved.
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
 /******************************************************************************
 * @file     startup.S
 * @brief    startup file. Should use with
 *           GCC for CSKY Embedded Processors
 * @version  V1.0
 * @date     15. May 2019
 ******************************************************************************/

#include <csi_config.h>

    .section .startup
    .align   2
	.long   g_top_irqstack
    .long   Reset_Handler
	.rept  46
	.long   0x0
	.endr
_start:
    .section .startup
    .align   2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    movs    r0, #0x0
    msr     control, r0

    ldr     r0, =g_top_irqstack
    mov     sp, r0
/*
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
    ldr     r1, =__bss_start__
    ldr     r2, =__bss_end__

    movs    r0, #0

    sub     r2, r2, r1
    cmp     r2, #0
    beq     .L_loop1_done

.L_loop1:
    str     r0, [r1]
    add     r1, r1, #4
    sub     r2, r2, #4
    cmp     r2, #0
    bne      .L_loop1
.L_loop1_done:

#ifndef __NO_SYSTEM_INIT
    ldr     r0, =SystemInit
    blx     r0
#endif

#ifndef __NO_BOARD_INIT
    ldr     r0, =board_init
    blx     r0
#endif

    ldr     r0, =main
    blx     r0

    .size   Reset_Handler, . - Reset_Handler
__exit:
    b       __exit

.section .bss

    .align  2
    .global g_intstackalloc
    .global g_intstackbase
    .global g_top_irqstack
g_intstackalloc:
g_intstackbase:
    .space  CONFIG_ARCH_INTERRUPTSTACK
g_top_irqstack:
