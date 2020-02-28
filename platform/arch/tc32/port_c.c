/********************************************************************************************************
 * @file     port_c.c 
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
#include <stdio.h>
#include "common/types.h"

#include "port_macro.h"
#include "drivers/8258/compiler.h"
#include "drivers/8258/irq.h"

void *cpu_task_stack_init(cpu_stack_t *base, size_t size, void *arg, task_entry_t entry)
{
    uint32_t temp = (uint32_t)(base + size - 1);
    temp &= 0xfffffff8;		// stack need 8 bytes align
	cpu_stack_t *pxTopOfStack = (cpu_stack_t *)temp;

    pxTopOfStack-- ;// *pxTopOfStack-- = (uint32_t)arg; 					// arg    saved
    *pxTopOfStack-- = (uint32_t)krhino_task_deathbed; 	// lr	 for task cleanup,  must right above entry !!!
	*pxTopOfStack-- = ((uint32_t)entry) & (~ 0x01);		// entry,  make it even
	pxTopOfStack -= 7;								// r1 -- r7
	*pxTopOfStack-- = (uint32_t)arg; //r0
	pxTopOfStack -= 5;								// r8 -- r12
	*pxTopOfStack-- = (uint32_t)0x13; 			//CPSR
	*pxTopOfStack-- = 1; 										//IRQ status
	*pxTopOfStack = (uint32_t)krhino_task_deathbed; 			/* LR   */
	return pxTopOfStack;
	
}

_attribute_no_inline_ cpu_cpsr_t cpu_intrpt_save(void){   // to be used as cpsr for aos.
	asm("tmrcs    	r0");
}

_attribute_no_inline_ void cpu_intrpt_restore(cpu_cpsr_t cpsr){
	asm("tmcsr    	r0");
}

_attribute_ram_code_ __attribute__((naked)) void cpu_first_task_start(void){
	asm("tloadr		r0, =g_active_task");
	asm("tloadr		r0, [r0] ");	// task Ptr
	asm("tloadr		r1, [r0] ");	// SP

	asm("tloadr	 	r0, [r1] ");	// load R14
	asm("tmov	 	r14, r0 ");		// set R14
	asm("tadd	 	r1, #4 ");		// irq status
	asm("tmov	 	r4, r1 ");		// irq status
	asm("tadd	 	r1, #64 ");		// bottom of SP
	asm("tmov	 	r13, r1 ");		// set task SP

	//  
	asm("tmov		r0, #146");
	asm("tmcsr		r0");			// switch to irq mode
	
	asm("tloadr		r2, _REG_IRQ_EN ");	// irq enable
	asm("tloadr 	r0, [r4] ");	// load irq status
	asm("tstorerb	r0, [r2] ");

	asm("tmov	 	r2, r13 ");
	asm("tmov		r3, #15");
	asm("_TASK_TO_IRQ:");
	asm("tsub		r1, #4");
	asm("tsub		r2, #4");
	asm("tloadr		r0, [r1]");
	asm("tstorer 	r0, [r2]");
	asm("tsub		r3, #1");
	asm("tcmp		r3, #0");
	asm("tjne		_TASK_TO_IRQ");
	asm("tmov	 	r13, r2 ");

	asm("tpop		{r0-r5}");
	asm("tmov		r8, r1");
	asm("tmov		r9, r2");
	asm("tmov		r10,r3");
	asm("tmov		r11,r4");
	asm("tmov		r12,r5");
	asm("tmssr    	r0");
	asm("tpop		{r0-r7}");
	asm("treti     	{r15}");			// PC
	asm(".align 	4");
asm("_REG_IRQ_EN:");
	asm(".word		0x800643");
}

uint32_t taskLR;
//uint32_t AAA_1 = 1,AAA_1cnt;
_attribute_ram_code_ __attribute__((naked))  void cpu_task_switch_i(void){
	asm("tpush 		{r14}");
	asm("tpush		{r0-r7}");
	asm("tmrcs		r0");				// save CPSR
	asm("tmov 		r1, r8");
	asm("tmov 		r2, r9");
	asm("tmov 		r3, r10");
	asm("tmov 		r4, r11");
	asm("tmov 		r5, r12");
	asm("tpush		{r0-r5}");

	asm("tloadr 	r2, _REG_IRQ_EN3 "); // irq enable
	asm("tloadrb 	r0, [r2]"); 
	asm("tpush 		{r0}"); 			// push irq status

	asm("tloadr 	r1, =taskLR"); 
	asm("tloadr 	r0, [r1]"); 
	asm("tpush 		{r0}"); 			// push r14
	
	asm("tloadr		r0, =g_active_task");
	asm("tloadr		r0, [r0]");			// R0 = g_active_task->task_stack which is pointer
	asm("tmov		r1, r13");
	asm("tstorer 	r1, [r0]");			// set old SP to task context: (g_active_task->task_stack.SP) = r1

	asm("tloadr		r2, =g_preferred_ready_task");
	asm("tloadr		r1, [r2]");			// R1 = g_preferred_ready_task->task_stack which is pointer
#if 0 // always copy
	asm("tcmp		r0, r1");			// compare old and new task pointer
	asm("tjeq		_CONTEXT_RESTORE_");// old SP is not changed..  RESTORE IS safe
#endif

	asm("tloadr		r0, =g_active_task");
	asm("tstorer 	r1, [r0]");			// set new SP to task contex
	asm("tjl		cpu_first_task_start");
	asm(".align 	4");
asm("_REG_IRQ_EN3:");
	asm(".word		0x800643");
}
__attribute__((naked)) void cpu_task_switch(){
	asm("tpush		{lr}");
	asm("tloadr 	r1, _REG_IRQ_EN2 ");
	asm("tloadr 	r2, [r1] "); 		 // save irq status
	asm("tmov 		r0, #0"); 			 //  irq disable  // make sure irq disable should be better even alios has disable before.
	asm("tstorerb	r0, [r1]"); 

	asm("tloadr 	r1, =taskLR"); 
	asm("tmov 		r0, r14"); 
	asm("tstorer 	r0, [r1]"); 

	asm("tjl		cpu_task_switch_i"); 

	asm("tloadr 	r1, _REG_IRQ_EN2 ");
	asm("tstorerb	r2, [r1]");  		// irq restore
	asm("tpop		{pc}");
asm(".align 	4");
asm("_REG_IRQ_EN2:");
	asm(".word		0x800643");
}

cpu_stack_t	 irqStackSave;
_attribute_ram_code_ __attribute__((naked))  void cpu_intrpt_switch(void){
	asm("tpush 		{r14}");			// push r14, return address
	asm("tpush		{r0-r7}");			

	asm("tmrcs 		r0");				// save CPSR
	asm("tmov 		r6, r0");
#if 1
    asm("tloadr     r0, _CPSR_NORMAL ");
#else // sometimes, this will cause PC stop at "0x880090", confirm later.
	asm("tmov		r0, #19");			// don't worry CPSR, be will overide later
#endif
	asm("tmcsr		r0");				// switch from irq mode to normal mode

	// get  normal  SP
	asm("tmov		r3, #15");
	asm("tmov		r2, r13");			// r2 the the old task SP
	asm("tloadr		r1, =irqStackSave");
	asm("tloadr		r1, [r1]");
	asm("tadd		r1, #60");			// r1 is irqStackSave + 60,  that is the bottom of stack, pointer to "cpsr"

	asm("_RAM_TO_TASK:");
	asm("tsub		r1, #4");			// r1: old task register position
	asm("tsub		r2, #4");			// r2: old task current SP position.
	asm("tloadr		r0, [r1]");
	asm("tstorer 	r0, [r2]");         // copy 15 u32 (from RAM(irqStackSave) to TASK(old SP)), include break point by IRQ in first byte
	asm("tsub		r3, #1");
	asm("tcmp		r3, #0");
	asm("tjne		_RAM_TO_TASK");

	asm("tmov 		r0, #1"); 
	asm("tsub		r2, #4");
	asm("tstorer 	r0, [r2]");		// irq must be enabled, because we are in IRQ

	asm("tmov		r0, r14");			// save normal r14
	asm("tsub		r2, #4");
	asm("tstorer	r0, [r2]");
	
	asm("tloadr	 	r1, =g_active_task");
	asm("tloadr	 	r1, [r1]");
	asm("tstorer 	r2, [r1]");		// save task SP

	//g_active_task[0] = g_preferred_ready_task[0];
	asm("tloadr		r0, =g_preferred_ready_task");
	asm("tloadr		r0, [r0]");
	asm("tloadr		r1, =g_active_task");
	asm("tstorer 	r0, [r1]");
	asm("tmov		r4, r0");		// save stack pointer for later use
	asm("tloadr 	r1, [r0]");		// svc SP to r1
	
	asm("tloadr		r0, [r1]");
	asm("tmov		r14, r0");		// restore normal r14
	asm("tadd		r1, #4");		// pop

	asm("tloadr		r7, [r1]");     // get irq status
	asm("tadd		r1, #4");		// pop

	asm("tloadr		r0, =irqStackSave");
	asm("tloadr		r2, [r0]");		// saved irq SP to r2:  that is a pointer to the top of the calling frame.   see irq routine
	asm("tmov		r3, #15");
	asm("_TASK_TO_IRQ1:");
	asm("tloadr		r0, [r1]");
	asm("tstorer 	r0, [r2]");
	asm("tadd		r1, #4");       // R1 is new stak current SP position
	asm("tadd		r2, #4");       // R2 is irqStackSave, restore register when exist IRQ.
	asm("tsub		r3, #1");
	asm("tcmp		r3, #0");
	asm("tjne		_TASK_TO_IRQ1");

	asm("tmov		r13, r1");		// Update SVC SP
	asm("tstorer 	r1, [r4]");		// pop up svc stack

	asm("tmov		r0, r6");		// Restore CPSR
	asm("tmcsr		r0");			// switch from svc to irq

	asm("tloadr 	r0, _REG_IRQ_EN1 "); // irq en register
	asm("tstorerb	r7, [r0] ");	// restore irq status
	
	asm("tpop 		{r0-r7}");		// call entry	// pop the first push in this function.
	asm("tpop		{r15}");

	asm(".align 	4");
asm("_REG_IRQ_EN1:");
	asm(".word		0x800643");
asm("_CPSR_NORMAL:");
    asm(".word      0x00000013");   // 0x60000013

}


