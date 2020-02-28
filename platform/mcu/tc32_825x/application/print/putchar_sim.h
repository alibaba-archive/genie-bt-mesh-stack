/********************************************************************************************************
 * @file     putchar_sim.h 
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

#ifndef PUT_CHAR_SIM_H
#define PUT_CHAR_SIM_H
#include "tl_common.h"

int sim_putchar(int c);
int sim_printf(const char *format, ...);
int sim_printf_buf(unsigned char *buf, int len, const char *format, ...);
int printf_Bin2Text (char *lpD, int lpD_len_max, char *lpS, int n);


void hal_uart_send_loop(int wait_flag);
void hal_uart_send_print(u16 len);

int uart_putc(char byte);
#define tc8258_printf	    sim_printf
#define tc8258_printf_buf   sim_printf_buf
#endif
