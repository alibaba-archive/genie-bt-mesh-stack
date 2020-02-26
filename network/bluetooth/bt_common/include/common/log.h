/** @file
 *  @brief Bluetooth subsystem logging helpers.
 */

/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BT_LOG_H
#define __BT_LOG_H

#include <zephyr.h>

#include <bluetooth.h>
#include <hci.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(BT_DBG_ENABLED)
#define BT_DBG_ENABLED 1
#endif

#if defined(CONFIG_BT_DEBUG_MONITOR)
#include <stdio.h>

/* These defines follow the values used by syslog(2) */
#define BT_LOG_ERR      3
#define BT_LOG_WARN     4
#define BT_LOG_INFO     6
#define BT_LOG_DBG      7

__printf_like(2, 3) void bt_log(int prio, const char *fmt, ...);

#define BT_DBG(fmt, ...)                                    \
    if (BT_DBG_ENABLED) {                                   \
        bt_log(BT_LOG_DBG, "%s (%p): " fmt,                 \
               __func__, k_current_get(), ##__VA_ARGS__);   \
    }

#define BT_ERR(fmt, ...) bt_log(BT_LOG_ERR, "%s: " fmt,     \
                __func__, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) bt_log(BT_LOG_WARN, "%s: " fmt,   \
                __func__, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) bt_log(BT_LOG_INFO, fmt, ##__VA_ARGS__)

/* Enabling debug increases stack size requirement */
#ifdef BOARD_PCA10040
#define BT_STACK_DEBUG_EXTRA    300
#else
#define BT_STACK_DEBUG_EXTRA    10
#endif

#elif defined(CONFIG_BT_DEBUG_LOG)

#if !defined(SYS_LOG_DOMAIN)
#define SYS_LOG_DOMAIN "bt"
#endif
#define SYS_LOG_LEVEL SYS_LOG_LEVEL_DEBUG

#define BT_DBG(fmt, ...)                    \
    if (BT_DBG_ENABLED) {                   \
        SYS_LOG_DBG("[%s]" fmt, __func__,   \
                ##__VA_ARGS__);             \
    }

#define BT_ERR(fmt, ...) SYS_LOG_ERR(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) SYS_LOG_WRN(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) SYS_LOG_INF(fmt, ##__VA_ARGS__)

/* Enabling debug increases stack size requirement considerably */
#ifdef BOARD_PCA10040
#define BT_STACK_DEBUG_EXTRA    300
#else
#define BT_STACK_DEBUG_EXTRA    10
#endif

#define BT_PRT printf
#else

#define BT_DBG(fmt, ...)
#define BT_ERR BT_DBG
#define BT_WARN BT_DBG
#define BT_INFO BT_DBG

#define BT_STACK_DEBUG_EXTRA    0

#endif

#ifdef GENIE_DEBUG_COLOR
#define F_BLACK  "\033[0;30m"
#define F_RED    "\033[0;31m"
#define F_GREEN  "\033[0;32m"
#define F_YELLOW "\033[0;33m"
#define F_BLUE   "\033[0;34m"
#define F_PURPLE "\033[0;35m"
#define F_DGREEN "\033[0;36m"
#define F_WHITE  "\033[0;37m"
#define F_END    "\033[0m"
#define BT_DBG_R(fmt, ...)                    \
                                if (BT_DBG_ENABLED) {                   \
                                    SYS_LOG_DBG(F_RED "[%s]" fmt F_END, __func__,   \
                ##__VA_ARGS__);             \
                                }
#else
#define F_BLACK
#define F_RED
#define F_GREEN
#define F_YELLOW
#define F_BLUE
#define F_PURPLE
#define F_DGREEN
#define F_WHITE
#define F_END

#define BT_DBG_R(fmt, ...)
#endif

#define MESH_MSG_TAG "[MESH] "

#ifdef MESH_DEBUG_RX
#define RX_COLOR F_BLUE
#define MESH_RX_TAG ">>>>\t\t[RX]"
#define MESH_MSG_RX(f, ...) printf(RX_COLOR MESH_RX_TAG MESH_MSG_TAG f F_END"\n", ##__VA_ARGS__);
                
#define MESH_MSG_RX_BUFF(buff, len) {                                           \
                        u16_t rx_i = 0;                                                         \
                        while(rx_i < len) {                                                     \
                            if(rx_i && (rx_i & 0x0F) == 0) printf("\n");                        \
                            if((rx_i & 0x0F) == 0) printf(RX_COLOR MESH_RX_TAG MESH_MSG_TAG);   \
                            printf("%02X ", buff[rx_i]);                                        \
                            rx_i++;                                                             \
                        }                                                                       \
                        if(rx_i) printf(F_END"\n");                                             \
                    }
#else
#define MESH_MSG_RX(fmt, ...)
#define MESH_MSG_RX_BUFF(fmt, ...)
#endif

#ifdef MESH_DEBUG_TX
#define TX_COLOR F_YELLOW
#define MESH_TX_TAG "<<<<\t[TX]"
#define MESH_MSG_TX(f, ...) printf(TX_COLOR MESH_TX_TAG MESH_MSG_TAG f F_END"\n", ##__VA_ARGS__);
#define MESH_MSG_TX_BUFF(buff, len) {                                       \
                        u16_t tx_i = 0;                                                         \
                        while(tx_i < len) {                                                     \
                            if(tx_i && (tx_i & 0x0F) == 0) printf("\n");                        \
                            if((tx_i & 0x0F) == 0) printf(TX_COLOR MESH_TX_TAG MESH_MSG_TAG);   \
                            printf("%02X ", buff[tx_i]);                                        \
                            tx_i++;                                                             \
                        }                                                                       \
                        if(tx_i) printf(F_END"\n");                                             \
                    }
#else
#define MESH_MSG_TX(fmt, ...)
#define MESH_MSG_TX_BUFF(fmt, ...)
#endif

#ifndef BT_PRT
#define BT_PRT(fmt, ...)
#endif


#define BT_ASSERT(cond) if (!(cond)) {                  \
                BT_ERR("assert: '" #cond "' failed");   \
                k_oops();                               \
            }

#define BT_STACK(name, size)        \
        K_THREAD_STACK_MEMBER(name, (size) + BT_STACK_DEBUG_EXTRA)
#define BT_STACK_NOINIT(name, size) \
        K_THREAD_STACK_DEFINE(name, (size) + BT_STACK_DEBUG_EXTRA)

/* This helper is only available when BT_DEBUG is enabled */
const char *bt_hex(const void *buf, size_t len);

void dump_print(char *p, size_t len);
u8_t stringtohex(char *str, u8_t *out, u8_t count);
void hextostring(const uint8_t *source, char *dest, int len);
u8_t stringtohex(char *str, u8_t *out, u8_t count);

/* These helpers are only safe to be called from internal threads as they're
 * not multi-threading safe
 */
const char *bt_addr_str(const bt_addr_t *addr);
const char *bt_addr_le_str(const bt_addr_le_t *addr);

#define BT_KEY(key) {for(char pki;pki<16;pki++)printf("%02x ",key[pki]);printf("\n");}

#ifdef __cplusplus
}
#endif

#endif /* __BT_LOG_H */

