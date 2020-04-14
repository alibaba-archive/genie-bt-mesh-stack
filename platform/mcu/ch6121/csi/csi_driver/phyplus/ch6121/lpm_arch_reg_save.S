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
 /****************************************************************************
 FILE_NAME           : arch_reg_save.S
 FUNCTION            : save the cpu context:
 ******************************************************************************/

.global arch_do_cpu_save
.global arch_do_cpu_resume
.global arch_resume_context

.text
.align 2
#.thumb
#.syntax unified

.thumb_func
arch_resume_context:
    LDR     R0, =g_arch_cpu_saved
    LDR     R1, [R0, #32]
    MSR     MSP, R1
    LDR     R1, [R0, #36]
    MSR     PSP, R1
    B       arch_do_cpu_resume

.thumb_func
arch_do_cpu_resume:
    LDR     R0, =g_arch_cpu_saved
    MOV     R1, R0
    ADD     R0, #16
    LDMIA   R0!, {R4-R7}
    MOV     R8,  R4
    MOV     R9,  R5
    MOV     R10, R6
    MOV     R11, R7

    LDMIA   R1!, {R4-R7}

    LDR     R2, [R0]
    MSR     MSP, R2

    LDR     R2, [R0, #4]
    MSR     PSP, R2

    LDR     R2, [R0, #8]
    MOV     LR, R2

    LDR     R2, [R0, #12]
    MSR     IEPSR, R2

    LDR     R2, [R0, #16]
    MSR     PRIMASK, R2

    LDR     R2, [R0, #20]
    MSR     FAULTMASK, R2

    LDR     R2, [R0, #24]
    MSR     BASEPRI, R2

    LDR     R2, [R0, #28]
    MSR     CONTROL, R2


    MOV     R0, #1
    BX      LR

.thumb_func
arch_do_cpu_save:
    LDR     R0, =g_arch_cpu_saved
    STMIA   R0!, {R4-R7}
    MOV     R4, R8
    MOV     R5, R9
    MOV     R6, R10
    MOV     R7, R11
    STMIA   R0!, {R4-R7}

    MRS     R1, MSP
    STR     R1, [R0]

    MRS     R1, PSP
    STR     R1, [R0, #4]

    MOV     R1, LR
    STR     R1, [R0, #8]

    MRS     R1, xPSR
    STR     R1, [R0, #12]

    MRS     R1, PRIMASK
    STR     R1, [R0, #16]

    MRS     R1, FAULTMASK
    STR     R1, [R0, #20]

    MRS     R1, BASEPRI
    STR     R1, [R0, #24]

    MRS     R1, CONTROL
    STR     R1, [R0, #28]

    LDR     R0, =g_arch_cpu_saved
    MOV     R1, R0
    ADD     R0, #16
    LDMIA   R0!, {R4-R7}
    MOV     R8,  R4
    MOV     R9,  R5
    MOV     R10, R6
    MOV     R11, R7

    LDMIA   R1!, {R4-R7}

    MOV     R0, #0
    BX      LR

