/**
 ****************************************************************************************
 *
 * @file bim_uart.h
 *
 * @brief UART Driver for HCI over UART operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _DEBUG_UART_H_
#define _DEBUG_UART_H_

/**
 ****************************************************************************************
 * @defgroup UART UART
 * @ingroup DRIVERS
 * @brief UART driver
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions

 

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */


void uart_stack_register(void* cb);
void uart_stack_debug(const char *fmt,...);
int stack_printf(const char *fmt,...);
int stack_printf_null(const char *fmt,...);
	
#define UART_PRINTF stack_printf //stack_printf_null
void stack_DEBUG_MSG(uint8_t x);

/// @} UART
#endif /* _BIM_UART_H_ */
