/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef __DUT_COMMON_H_
#define __DUT_COMMON_H_

#define CONFIG_UART_BUF_SIZE  96
#define CONFIG_QUEUE_COUNT    2

#define  CONFIG_QUEUE_BUF_SIZE             (CONFIG_QUEUE_COUNT * CONFIG_UART_BUF_SIZE)

extern void board_ble_init(void);
extern void feed_wdt(void);
extern void boot_wdt_close(void);
extern void enableSleepInPM(uint8_t flag);

/*
    Disable LPM according to bit value of flag
*/
extern void disableSleepInPM(uint8_t flag);

/*
    Enable LPM if all bits value of flag is 0
*/
extern void enableSleepInPM(uint8_t flag);

extern void mdelay(uint32_t ms);



#endif
