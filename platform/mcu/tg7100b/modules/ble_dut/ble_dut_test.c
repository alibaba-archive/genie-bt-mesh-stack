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

/******************************************************************************
 * @file     dut_test.c
 * @brief    DUT Test Source File
 * @version  V1.0
 * @date     10. Dec 2019
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "log.h"
#include "clock.h"
#include "drv_usart.h"
#include "pinmux.h"
#include <yoc/partition.h>
#include <math.h>
#include <gpio.h>
#include <drv_wdt.h>
#include <aos/aos.h>
#include <aos/kernel.h>
#include "dtm_test.h"
#include "dut_rf_test.h"
#include "dut_uart_driver.h"
#include "commons.h"
#include "dut_utility.h"
#include "dut_at_cmd.h"


#define CONFIG_UART_BUF_SIZE  96
#define CONFIG_QUEUE_COUNT    2

#define  CONFIG_QUEUE_BUF_SIZE             (CONFIG_QUEUE_COUNT * CONFIG_UART_BUF_SIZE)
extern volatile uint8_t g_dut_queue_num;
extern aos_queue_t  g_dut_queue;
/***************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/*******************    msg queue     *******************************/
static unsigned char  dut_queue_buf[CONFIG_QUEUE_BUF_SIZE]  __attribute__((section("noretention_mem_area0")));

/****************************************************************/
static volatile uint8_t dut_mode_flag;

/** uart config **/
dut_uart_cfg_t g_dut_uart_cfg = {
    0, //uart idx
    115200,
    USART_DATA_BITS_8,
    USART_PARITY_NONE,
    USART_STOP_BITS_1,
};

#define   NO_ADD    0x1
#define   ADD       0x2
#define   WHY       0x4
#define   DOU       0x8
#define   FIND      0x10

#define   EXECUTE     0x1
#define   FCURRENT    0x2
#define   HLOOKUP     0x3

typedef int (*cmd_function_t)(int argc, char *argv[]);

struct cli_cmd {
    const char *cmd_name;
    cmd_function_t cb;
    const char *help;
    cmd_function_t fcb;;
};

static const struct cli_cmd dut_commands[]  = {
    //AT+TEST     AT+TEST=?           AT+TEST?
    { "RXMODE", dut_cmd_rx_mode, "+RXMODE='value'", dut_cmd_ftest},
    { "SLEEP", dut_cmd_sleep, "+SLEEP" , NULL},
    { "IREBOOT", dut_cmd_ireboot, "+IREBOOT='mode'" , NULL},
    { "MAC", dut_cmd_opt_mac, "+MAC='xx:xx:xx:xx:xx:xx'" , fdut_cmd_opt_mac},
    { "FREQOFF", dut_cmd_freq_off, "+FREQ_OFF='value'" , fdut_cmd_freq_off},
    { "GPIO", dut_cmd_gpio, "+GPIO='mode'", fdut_cmd_gpio},
    { "TRITUPLE", dut_cmd_trituple_component, "+TRITUPLE='mode'", fdut_cmd_trituple_component},
    { "FLASH", dut_cmd_flash, "+FLASH='mode'", fdut_cmd_flash},

    { NULL, NULL, NULL, NULL },
};

static int at_cmd_type(char *s)
{
    int flag = 0;

    flag |= NO_ADD;

    for (int i = 0; * (s + i) != '\0'; i++) {
        if (*(s + i) == '+') {
            flag = flag & (~0x1);
            flag |= ADD;    //+ no =
        }

        if (*(s + i) == '=') { //, or not ,
            if (*(s + i + 1) == '?') {
                return  FIND;
            } else if (*(s + i + 1) != '\0') {
                return DOU;
            } else {
                return 0;
            }
        }

        if (*(s + i) == '?') {
            flag = flag & (~0x2); //no +
            flag |= WHY;
        }
    }

    return flag;
}
static void cmd_cli_func(int type, int argc, char **argv)
{
    int i = 0;
    int err;

    if (argc < 2) {
        printf("AT support commands\n");

        for (i = 0; dut_commands[i].cmd_name != NULL; i ++) {
            printf("    %s %s\n", dut_commands[i].cmd_name, dut_commands[i].help);
        }
        return;
    }

    for (i = 0; dut_commands[i].cmd_name != NULL; i ++) {
        if (strlen(dut_commands[i].cmd_name) == strlen(argv[1]) &&
            !strncmp(dut_commands[i].cmd_name, argv[1], strlen(dut_commands[i].cmd_name))) {
            if (type == EXECUTE) {
                if (dut_commands[i].cb) {
                    err = dut_commands[i].cb(argc - 1, &argv[1]);

                    if (err) {
                        printf("ERROR:%s,%d\n", dut_commands[i].cmd_name, err);
                    } else {
                        printf("OK\r\n");
                    }

                }
            } else if (type == FCURRENT) {
                if (dut_commands[i].fcb) {
                    err = dut_commands[i].fcb(argc - 1, &argv[1]);

                    if (err) {
                        printf("ERROR:%s, %d\n", dut_commands[i].cmd_name, err);
                    } else {
                        printf("OK\r\n");
                    }
                }
            } else {  //HLOOKUP
                if (dut_commands[i].help) {
                    printf("%s\n", dut_commands[i].help);
                    printf("OK\r\n");
                }
            }

            break;
        }
    }

    if ((i + 1) == ARRAY_SIZE(dut_commands)) {
        printf("ERROR:%s,NOT FOUND\n", argv[1]);
    }
}

static int is_at_cmd(char *data)
{
    if (strlen(data) < 2) {
        return 0;
    }

    if ((*data == 'A') && (*(data + 1) == 'T')) {
        return 1;
    }
    return 0;
}

static void dut_test_at(char data[])
{
    char argv[8][36];
    char *hcc;
    char *argqv[8];

    if (0 == is_at_cmd(data)) {
        return;
    }

    memset(argv, 0, sizeof(argv));
    int ustype  = at_cmd_type(data);
    argv[0][0] = 'A';
    argv[0][1] = 'T';
    argv[0][2] = '\0';
    switch (ustype) {
        case NO_ADD:
            printf("OK\r\n");
            break;

        case ADD://execute
            if (char_cut(argv[1], data, '+', '\0') == NULL) {
                return;
            }
            argqv[0] = (char *)(&argv[0]);
            argqv[1] = (char *)(&argv[1]);
            cmd_cli_func(EXECUTE, 2, (char **)(&argqv));
            break;

        case WHY://?
            if (char_cut((char *)argv[1], data, '+', '?') == NULL) {
                return;
            }

            argqv[0] = (char *)(&argv[0]);
            argqv[1] = (char *)(&argv[1]);
            cmd_cli_func(FCURRENT, 2, (char **)(&argqv));
            break;

        case DOU:
            if (char_cut((char *)argv[1], data, '+', '=') == NULL) {
                return;
            }

            hcc = strchr(data, '=');
            int num = argc_len(hcc);

            if (num == 0) {
                printf("num:err 0\r\n");
                return;
            }

            if (num == 1) {
                char_cut((char *)argv[2], hcc, '=', '\0');
                argqv[2] = (char *)(&argv[2]);
            } else {
                char_cut((char *)argv[2], hcc, '=', ',');
                argqv[2] = (char *)(&argv[2]);

                for (int i = 1; i < num; i++) {
                    hcc = strchr(hcc, ',');
                    char_cut((char *)argv[2 + i], hcc, ',', ',');
                    argqv[2 + i] = (char *)(&argv[2 + i]);
                    hcc++;
                }
            }

            argqv[0] = (char *)(&argv[0]);
            argqv[1] = (char *)(&argv[1]);
            cmd_cli_func(EXECUTE, num + 2, (char **)(&argqv));
            break;

        case FIND://=  ?
            hcc = strrchr(data, '+');
            hcc++;
            str_chr(argv[1], hcc, '=');
            argqv[0] = (char *)(&argv[0]);
            argqv[1] = (char *)(&argv[1]);
            cmd_cli_func(HLOOKUP, 2, (char **)(&argqv));
            break;

        default :
            printf("unknow type:%d\r\n", ustype);
            break;
    }

}


static void dut_test_entry(void)
{
    char msg_recv[CONFIG_UART_BUF_SIZE] = {0};
    unsigned int size_recv = 0;
    dut_mode_flag  = 1;

    /* print rf config */
    rf_phy_direct_print();

    rf_phy_dtm_init(dut_uart_putchar);

    g_dut_queue_num = 0;

    while (1) {
        memset(msg_recv, 0, sizeof(msg_recv));

        while (g_dut_queue_num == 0);
        aos_queue_recv(&g_dut_queue, 0, msg_recv, &size_recv);
        g_dut_queue_num = 0;

        if (0 == is_at_cmd(msg_recv)) {
            dtm_test_loop(msg_recv, size_recv);
        } else {
            dut_test_at(msg_recv);
        }
    }
}

static void config_uart_pin(void)
{
    drv_pinmux_config(P9, UART_TX);
    drv_pinmux_config(P10, UART_RX);
}
void boot_wdt_close(void)
{
    wdt_handle_t wdt_handle = NULL;

    wdt_handle = csi_wdt_initialize(0, NULL);
    if (wdt_handle == NULL) {
        printf("WDT init Err!\n");
        return;
    }
    csi_wdt_uninitialize(wdt_handle);
}


int ble_dut_start(void)
{
    int ret;

    dut_mode_flag = 0;

    dut_at_cmd_init();

    /* close wdt */
    boot_wdt_close();

    /* init queue for uart data transimit */
    ret = aos_queue_new(&g_dut_queue, dut_queue_buf, CONFIG_QUEUE_BUF_SIZE, CONFIG_UART_BUF_SIZE);
    if (ret) {
        printf("queue new error\r\n");
        return -1;
    }
    /* config uart pin function */
    config_uart_pin();

    /* config uart for AT cmd input */
    ret = dut_uart_init(&g_dut_uart_cfg);
    if (ret) {
        printf("queue new error\r\n");
        return -1;
    }

    dut_test_entry();

    return 0;
}

uint8_t get_dut_test_flag(void)
{
    return dut_mode_flag;
}

