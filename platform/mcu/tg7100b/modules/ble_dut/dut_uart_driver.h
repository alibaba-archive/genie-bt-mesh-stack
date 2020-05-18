/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef __DUT_UART_DRIVER_H_
#define __DUT_UART_DRIVER_H_
#include "drv_usart.h"
#include "commons.h"

typedef struct {
    int idx;
    uint32_t                baud_rate;
    usart_data_bits_e       data_width;
    usart_parity_e          parity;
    usart_stop_bits_e       stop_bits;
} dut_uart_cfg_t;

int dut_uart_init(dut_uart_cfg_t *uart_cfg);
void dut_uart_putchar(unsigned char data);
int dut_at_send(const char *command, ...);

#endif
