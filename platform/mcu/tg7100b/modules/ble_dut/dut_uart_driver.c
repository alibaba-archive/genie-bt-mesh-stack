/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

/*******************************************************************************
 * INCLUDES
 */
#include "log.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "pinmux.h"
#include <gpio.h>
#include <aos/aos.h>
#include <aos/kernel.h>
//#include "commons.h"
//#include "k_buf_queue.h"
#include "dut_uart_driver.h"

aos_queue_t  g_dut_queue;
volatile uint8_t g_dut_queue_num = 0;

/****************************************************************/
#define AT_OUTPUT_TERMINATION  "\r\n"
//#define CONFIG_UART_BUF_SIZE  128

//extern volatile uint8_t g_dut_queue_num; 
//extern aos_queue_t  g_dut_queue;

static usart_handle_t g_usart_handle;
static volatile uint8_t tx_async_flag = 0;
unsigned char dut_uart_buf[CONFIG_UART_BUF_SIZE - 1] __attribute__((section("noretention_mem_area0")));
static volatile uint8_t uart_rcv_len = 0;
static volatile uint8_t at_rcv_flag = 0;


void usart_event_cb_query(int32_t idx, uint32_t event)
{
    uint8_t data[16];

    memset(data, 0, sizeof(data));

    switch (event) {
        case USART_EVENT_SEND_COMPLETE:
            tx_async_flag = 1;
            break;

        case USART_EVENT_RECEIVE_COMPLETE:
            break;

        case USART_EVENT_RECEIVED:
        {
            int tmp_len = 0;

            tmp_len = csi_usart_receive_query(g_usart_handle, data, sizeof(data));

            if ((tmp_len > 2 ) && (data[tmp_len - 1] != 0xd)  && (data[tmp_len - 1] != '\0') && (at_rcv_flag == 0)) {
                /* this is first pack of at cmd */
                for (int i = 0; i < tmp_len; i++) {
                    dut_uart_buf[i] = data[i];
                }
                uart_rcv_len = tmp_len;
                at_rcv_flag = 1;
            } else if (at_rcv_flag == 1) {

                /* this is continue pack of at cmd */
                if (tmp_len > (CONFIG_UART_BUF_SIZE - tmp_len - 1)) {
                    /* over flow, clear buf */
                    memset(dut_uart_buf, 0, sizeof(dut_uart_buf));
                    uart_rcv_len = 0;
                    at_rcv_flag = 0;
                }
                for (int i = 0; i < tmp_len; i++) {
                    dut_uart_buf[i + uart_rcv_len] = data[i];
                }

                uart_rcv_len += tmp_len;

                if ((dut_uart_buf[uart_rcv_len - 1] == 0xd) || (dut_uart_buf[uart_rcv_len - 1] == '\0')) {
                    at_rcv_flag = 0;
                    dut_uart_buf[uart_rcv_len - 1] = '\0';
                    aos_queue_send(&g_dut_queue, dut_uart_buf, uart_rcv_len);
                    g_dut_queue_num = 1;
                    memset(dut_uart_buf, 0, sizeof(dut_uart_buf));
                    uart_rcv_len = 0;

                } else {
                    //wait for at end
                    break;
                }
            } else {
                if ((data[tmp_len - 1] == 0xd) || (data[tmp_len - 1] == '\0')){
                    data[tmp_len - 1] = '\0';
                } else {
                    data[tmp_len] = '\0';
                }
                uart_rcv_len = tmp_len;
                /*since dtm timeout is 1s, we send data to dut task directly */
                aos_queue_send(&g_dut_queue, data, uart_rcv_len);
                g_dut_queue_num = 1;
                memset(dut_uart_buf, 0, sizeof(dut_uart_buf));
                uart_rcv_len = 0;
            }
            break;
        }
        default:
            break;
    }
}

int dut_uart_init(dut_uart_cfg_t *uart_cfg)
{
    int ret;
    if (uart_cfg == NULL) {
        return -1;
    }
    g_usart_handle = csi_usart_initialize(uart_cfg->idx, (usart_event_cb_t)usart_event_cb_query);

    if (g_usart_handle == NULL) {
        return -1;
    }

    ret = csi_usart_config(g_usart_handle,
                           uart_cfg->baud_rate,
                           USART_MODE_ASYNCHRONOUS,
                           uart_cfg->parity,
                           uart_cfg->stop_bits,
                           uart_cfg->data_width);

    if (ret < 0) {
        printf("csi_usart_config error %x\n", ret);
        return -1;
    }
    return 0;
}

int32_t usart_send_async(usart_handle_t usart, const void *data, uint32_t num)
{
    int time_out = 0x7ffff;
    tx_async_flag = 0;

    csi_usart_send(usart, data, num);

    while (time_out) {
        time_out--;

        if (tx_async_flag == 1) {
            break;
        }
    }

    if (0 == time_out) {
        return -1;
    }

    tx_async_flag = 0;

    return 0;
}

int dut_sendv(const char *command, va_list args)
{
    int ret = -EINVAL;
    char *send_buf = NULL;

    if (vasprintf(&send_buf, command, args) >= 0) {
        ret = usart_send_async(g_usart_handle, send_buf, strlen(send_buf));

        if (ret == 0) {
            ret = usart_send_async(g_usart_handle, AT_OUTPUT_TERMINATION, strlen(AT_OUTPUT_TERMINATION));
        }

        free(send_buf);
    }

    return ret;
}

int dut_at_send(const char *command, ...)
{
    int ret;
    va_list args;

    aos_check_return_einval(command);

    va_start(args, command);

    ret = dut_sendv(command, args);

    va_end(args);

    return ret;
}

void dut_uart_putchar(unsigned char data)
{
    usart_send_async(g_usart_handle, &data, 1);
}

