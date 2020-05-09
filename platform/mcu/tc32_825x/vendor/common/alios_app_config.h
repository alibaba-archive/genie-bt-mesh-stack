/********************************************************************************************************
 * @file     app_config.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 public@telink-semi.com;
 * @date     May. 12, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif


/////////////////// Clock  /////////////////////////////////
#define CLOCK_SYS_CLOCK_HZ  	48000000

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};


/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms

/////////////////// PCBA setting  //////////////////////////////
#define PCBA_DONGLE_48PIN           1
#define PCBA_LSD4BT                 2

#define PCBA_SEL            PCBA_LSD4BT // PCBA_DONGLE_48PIN // 

#if (PCBA_SEL == PCBA_DONGLE_48PIN)
#define PWM_R       GPIO_PWM1A3		//red
#define PWM_G       GPIO_PWM0A2		//green
#define PWM_B       GPIO_PWM3B0		//blue
#define PWM_W       GPIO_PWM4B1		//white

#define DMA_UART_TX     UART_TX_PD7
#define DMA_UART_RX     UART_RX_PA0
#else
#define PWM_R       GPIO_PWM4B4		// LED1,  orange
#define PWM_G       GPIO_PWM1C3		// LED3
#define PWM_B       GPIO_PC4		// LED2, AS_SWM
#define PWM_W       GPIO_PWM3D2  	// LED? NC now
#define BTN_K1      GPIO_PWM3D2  	// LED? NC now
#define BTN_K2      GPIO_PWM5B5  	// LED? NC now


#ifdef CONIFG_LIGHT_HONGYAN
#define DMA_UART_TX     UART_TX_PD3
#else
#define DMA_UART_TX     UART_TX_PB1
#endif
#define DMA_UART_RX     UART_RX_PA0
#endif

///////////////// printf_put_char //////////////////////////
#define PRINT_ENABLE            1
#define IS_PRINT_USE_SIM        0

#if(IS_PRINT_USE_SIM==1)
#define PRINT_BAUD_RATE			115200
#define DEBUG_INFO_TX_PIN		DMA_UART_TX	// dongle
//#define DEBUG_INFO_TX_PIN		GPIO_PB1	// develop board
#else
//hal-->uart.c, hal_uart_init(): DMA_UART_TX, DMA_UART_RX,115200
#define HW_UART_RING_BUF_EN		1
#endif

#define GPIO_IRQ_ENABLE					0

#if GPIO_IRQ_ENABLE
#define GPIO_IRQ_SUPPORTED_NUM			2
#endif

//#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
