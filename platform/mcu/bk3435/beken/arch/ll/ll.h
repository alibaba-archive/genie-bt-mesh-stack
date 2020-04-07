/**
 ****************************************************************************************
 *
 * @file ll.h
 *
 * @brief Declaration of low level functions.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef LL_H_
#define LL_H_



#include <stdint.h>
#include <k_api.h>

#include "compiler.h"

#define KEIL_INTRINSICS_INTERRUPT


/** @brief Enable interrupts globally in the system.
 * This macro must be used when the initialization phase is over and the interrupts
 * can start being handled by the system.
 */
 
/* keil Compiler intrinsics for controlling IRQ and FIQ interrupts
*/

//extern void __enable_irq(void);
//extern void __enable_fiq(void);
//extern int  __disable_irq(void);
//extern int  __disable_fiq(void);

#define __enable_irq	portENABLE_IRQ
#define __enable_fiq	portENABLE_FIQ
#define __disable_irq	portDISABLE_IRQ
#define __disable_fiq	portDISABLE_FIQ
 
 
#define GLOBAL_INT_START(); \
do { \
    __enable_fiq(); \
		__enable_irq(); \
} while(0);

/** @brief Disable interrupts globally in the system.
 * This macro must be used when the system wants to disable all the interrupt
 * it could handle.
 */

#define GLOBAL_INT_STOP();		\
do { \
						__disable_fiq(); \
						__disable_irq(); \
} while(0);


/* * @brief Disable interrupts globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE macro since this
 * last one will close the brace that the current macro opens.  This means that both
 * macros must be located at the same scope level.
 */
#ifdef ALIOS_KERNEL
#define GLOBAL_INT_DISABLE() \
do { \
    CPSR_ALLOC(); \
	RHINO_CRITICAL_ENTER()

#define GLOBAL_INT_RESTORE() \
	RHINO_CRITICAL_EXIT();    \
} while(0)

#define GLOBAL_INT_DECLARATION_N() CPSR_ALLOC()
#define GLOBAL_INT_DISABLE_N()     RHINO_CRITICAL_ENTER()
#define GLOBAL_INT_RESTORE_N()     RHINO_CRITICAL_EXIT()
#else
#define GLOBAL_INT_DISABLE(); \
do { \
    uint32_t  fiq_tmp; \
	  uint32_t  irq_tmp; \
		fiq_tmp = __disable_fiq(); \
		irq_tmp = __disable_irq();

#define GLOBAL_INT_RESTORE(); \
			if(!fiq_tmp)           \
			{                      \
					__enable_fiq();    \
			}                      \
			if(!irq_tmp)           \
			{                      \
					__enable_irq();    \
			}                      \
} while(0) ;                                       

#define GLOBAL_INT_DECLARATION_N() uint32_t fiq_tmp, irq_tmp
#define GLOBAL_INT_DISABLE_N()     do{\
										fiq_tmp = portDISABLE_FIQ();\
										irq_tmp = portDISABLE_IRQ();\
									}while(0)


#define GLOBAL_INT_RESTORE_N()     do{                         \
                                        if(!fiq_tmp)           \
                                        {                      \
                                            portENABLE_FIQ();    \
                                        }                      \
                                        if(!irq_tmp)           \
                                        {                      \
                                            portENABLE_IRQ();    \
                                        }                      \
                                   }while(0)
#endif

#if 0
/** @brief Invoke the wait for interrupt procedure of the processor.
 *
 * @warning It is suggested that this macro is called while the interrupts are disabled
 * to have performed the checks necessary to decide to move to sleep mode.
 *
 */
__INLINE void WFI(void)
{
    uint32_t __l_rd;
#pragma arm   
    __asm                                                
    {                                    
        MOV __l_rd, 0;               
        MCR p15, 0, __l_rd, c7, c0, 4;
    } 
}
#endif

#endif // LL_H_


