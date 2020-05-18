/**************************************************************************************************

  Phyplus Microelectronics Limited. All rights reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

**************************************************************************************************/




/**************************************************************************************************
  Filename:       jump_table.c
  Revised:
  Revision:

  Description:    Jump table that holds function pointers and veriables used in ROM code.


**************************************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include "jump_function.h"
#include "global_config.h"
#include "OSAL_Tasks.h"
#include "rf_phy_driver.h"
#include "pwrmgr.h"
#include "adc.h"
#include "gpio.h"
#include "uart.h"
#include "i2c_common.h"
//#include "kscan.h"
#include "rflib.h"
#include "log.h"
#include "spi.h"
#include "watchdog.h"
#include "ap_timer.h"
#include "lib_printf.h"

extern void PendSV_Handler(void);
extern void CSI_SysTick_Handler(void);

extern void llConnTerminate1(llConnState_t *connPtr, uint8 reason);
extern void llPduLengthUpdate1(uint16 connHandle);
extern void llMasterEvt_TaskEndOk1(void);
extern uint8 llSetupNextMasterEvent1(void);
extern uint8 llProcessMasterControlProcedures1(llConnState_t *connPtr);
extern void llProcessMasterControlPacket1(llConnState_t *connPtr, uint8  *pBuf);

extern void llSlaveEvt_TaskEndOk1(void);
extern uint8 llSetupNextSlaveEvent1(void);
extern uint8 llProcessSlaveControlProcedures1(llConnState_t *connPtr);
extern void llProcessTxData1(llConnState_t *connPtr, uint8 context);
extern uint8 llProcessRxData1(void);
extern void LL_IRQHandler1(void);
extern uint8 LL_SetScanControl1( uint8 scanMode,uint8 filterReports );
extern uint8 LL_SetAdvControl1(uint8 advMode);

extern uint8 llSetupAdv1( void );
extern uint8 LL_CreateConn1( uint16 scanInterval,
                          uint16 scanWindow,
                          uint8  initWlPolicy,
                          uint8  peerAddrType,
                          uint8  *peerAddr,
                          uint8  ownAddrType,
                          uint16 connIntervalMin,
                          uint16 connIntervalMax,
                          uint16 connLatency,
                          uint16 connTimeout,
                          uint16 minLength,
                          uint16 maxLength );

extern void CSI_UART0_IRQHandler(void);
extern void CSI_GPIO_IRQHandler(void);
extern int CSI_TIM0_IRQHandler(void);
extern void CSI_SPI0_IRQHandler(void);
extern void CSI_SPI1_IRQHandler(void);
extern void CSI_AP_TIMER_IRQHandler(void);
extern void CSI_RTC_IRQHandler(void);
extern void CSI_WDT_IRQHandler(void);
extern void CSI_IIC0_IRQHandler(void);
extern void CSI_IIC1_IRQHandler(void);
extern int32_t arch_resume_context(void);
extern int TIM0_IRQHandler1(void);
extern void LL_master_conn_event1(void);
extern void __attribute__((used)) phy_ADC_VoiceIRQHandler(void);
extern void ll_hw_go1(void);

/*******************************************************************************
 * MACROS
 */
void (*trap_c_callback)(void);

extern const uint32_t *const jump_table_base[];
void _hard_fault(uint32_t *arg)
{
    uint32_t *stk = (uint32_t *)((uint32_t)arg);

    printk("[Hard fault handler]\n");
    printk("R0  = 0x%x\n", stk[9]);
    printk("R1  = 0x%x\n", stk[10]);
    printk("R2  = 0x%x\n", stk[11]);
    printk("R3  = 0x%x\n", stk[12]);
    printk("R4  = 0x%x\n", stk[1]);
    printk("R5  = 0x%x\n", stk[2]);
    printk("R6  = 0x%x\n", stk[3]);
    printk("R7  = 0x%x\n", stk[4]);
    printk("R8  = 0x%x\n", stk[5]);
    printk("R9  = 0x%x\n", stk[6]);
    printk("R10 = 0x%x\n", stk[7]);
    printk("R11 = 0x%x\n", stk[8]);
    printk("R12 = 0x%x\n", stk[13]);
    printk("SP  = 0x%x\n", stk[0]);
    printk("LR  = 0x%x\n", stk[14]);
    printk("PC  = 0x%x\n", stk[15]);
    printk("PSR = 0x%x\n", stk[16]);
    printk("HFSR = 0x%x\n", *(volatile uint32_t *)0xE000ED2C);

    if (trap_c_callback) {
        trap_c_callback();
    }

    while (1);
}

#ifdef __GNUC__
__attribute__((naked)) void hard_fault(void)
{
    asm("ldr     r1, =0xFFFFFFFD\n\r"
        "ldr     r0, [sp, #4]\n\r"
        "cmp     r1, r0\n\r"
        "beq     .Lstore_in_psp\n\r"
        "mov     r0, sp\n\r"
        "add     r0, r0, #8\n\r"
        "sub     r0, r0, #0x24\n\r"
        "mov     sp, r0\n\r"
        "b       .Lstore_in_msp\n\r"
        ".Lstore_in_psp:\n\r"
        "mrs     r0, psp\n\r"
        "sub     r0, r0, #0x24\n\r"
        ".Lstore_in_msp:\n\r"
        "mov     r1, r0\n\r"
        "add     r1, r1, #0x44\n\r"
        "str     r1, [r0]\n\r"
        "add     r0, r0, #4\n\r"
        "stmia   r0!, {r4-r7}\n\r"
        "mov     r4, r8\n\r"
        "mov     r5, r9\n\r"
        "mov     r6, r10\n\r"
        "mov     r7, r11\n\r"
        "stmia   r0!, {r4-r7}\n\r"
        "sub     r0, r0, #36\n\r"
        "ldr     r1, =_hard_fault\n\r"
        "bx      r1\n\r");
}
#else
__asm void hard_fault(void)
{
    IMPORT  _hard_fault
    ldr     r1, = 0xFFFFFFFD
                  ldr     r0, [sp, #4]
                  cmp     r1, r0
                  beq     Lstore_in_psp
                  mov     r0, sp
                  adds    r0, r0, #8
                  subs    r0, r0, #0x24
                  mov     sp, r0
                  b       Lstore_in_msp
                  Lstore_in_psp
                  mrs     r0, psp
                  subs    r0, r0, #0x24
                  Lstore_in_msp
                  mov     r1, r0
                  adds    r1, r1, #0x44
                  str     r1, [r0]
                  adds    r0, r0, #4
                  stmia   r0!, {r4 - r7}
                  mov     r4, r8
                  mov     r5, r9
                  mov     r6, r10
                  mov     r7, r11
                  stmia   r0!, {r4 - r7}
                  subs    r0, r0, #32
                  subs    r0, r0, #36
                  ldr     r1, = _hard_fault
                                bx      r1
}
#endif

/*******************************************************************************
 * CONSTANTS
 */
// jump table, this table save the function entry which will be called by ROM code
// item 1 - 4 for OSAL task entry
// item 224 - 255 for ISR(Interrupt Service Routine) entry
// others are reserved by ROM code
const uint32_t *const jump_table_base[256] __attribute__((section("jump_table_mem_area"))) = {
    (const uint32_t *)0,                        // 0. write Log
    (const uint32_t *)osalInitTasks,            // 1. init entry of app
    (const uint32_t *)tasksArr,                 // 2. task list
    (const uint32_t *) &tasksCnt,               // 3. task count
    (const uint32_t *) &tasksEvents,            // 4. task events
    0, 0, 0, 0, 0,                              // 5 - 9, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, (const uint32_t *)LL_SetAdvControl1, 0,               // 10 - 19, reserved by phyplus
    0, 0, 0, 0, 0, 0, (const uint32_t *)llSlaveEvt_TaskEndOk1, (const uint32_t *)llSetupNextSlaveEvent1, 0, (const uint32_t *)llProcessSlaveControlProcedures1,   // 20 - 29, reserved by phyplus
    0, 0, (const uint32_t *)llProcessRxData1, (const uint32_t *)llProcessTxData1, (const uint32_t *)llConnTerminate1, 0, 0, 0,                  // <30 - - 37>
    0, (const uint32_t *)llSetupAdv1,
    0, 0, 0, 0, 0, 0, //40 - 45
    0, 0, 0, 0,                                 //46 - 49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 50 - 59, reserved by phyplus
    0,   // < 60 -
    0,
    (const uint32_t *)arch_resume_context,
    0,
    0, 0, (const uint32_t *)ll_hw_go1,
    0, 0, 0,                          //  -69>, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 70 -79, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 80 - 89, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 90 - 99, reserved by phyplus
    0,//(const uint32_t*)hal_pwrmgr_sleep_process,         // <100 -
    0,//(const uint32_t*)hal_pwrmgr_wakeup_process,
    0,//(const uint32_t*)rf_phy_ini,
    0,
    0,
    0,
    0, 0, 0, 0,                       // - 109, reserved by phyplus
    (const uint32_t *)0, (const uint32_t *)0, (const uint32_t *)0, (const uint32_t *)0, 0,(const uint32_t *)0, //,<-115
    (const uint32_t *)LL_SetScanControl1, 0, (const uint32_t *)0, 0, // 116 -119, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 120 -129, reserved by phyplus
    0, (const uint32_t *)llPduLengthUpdate1, 0, 0, 0, 0, 0, 0, 0, 0,    // 130 -139, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 140 -149, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 150 -159, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 160 -169, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 170 -179, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 180 -189, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 190 -199, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 200 - 209, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 210 - 219, reserved by phyplus
    (const uint32_t *)hard_fault, 0, (const uint32_t *)PendSV_Handler, (const uint32_t *)CSI_SysTick_Handler, 0, 0,(const uint32_t *)TIM0_IRQHandler1, 0,       // 220 - 227
    (const uint32_t *)LL_IRQHandler1, 0,      // 228 - 229
    /*(const uint32_t *)RTC_IRQHandler*/0, 0, 0, /*(const uint32_t *)AP_TIMER_IRQHandler*/0, /*(const uint32_t *)WDT_IRQHandler*/0, // 230 - 234
    0,     // 235 uart irq handler
    /*(const uint32_t *)IIC0_IRQHandler*/0,
    /*(const uint32_t *)IIC1_IRQHandler*/0,
    /*(const uint32_t *)SPI0_IRQHandler*/0,
    /*(const uint32_t *)SPI1_IRQHandler*/0,    // 236 - 239
    0, //240 gpio interrupt handler
    0, 0, 0, 0, 0, 0, 0, 0, 0,     // 241 - 249, for ISR entry
    0, 0, 0, 0 /*hal_ADC_IRQHandler*/,
    0, 0                 // 250 - 255, for ISR entry
};

/*******************************************************************************
 * Prototypes
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */
uint32 global_config[SOFT_PARAMETER_NUM] __attribute__((section("global_config_area")));


