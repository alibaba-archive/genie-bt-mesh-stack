/*
 * Copyright (C) 2017 C-SKY Microsystems Co., All rights reserved.
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

#ifndef YOC_NET_MGR_H
#define YOC_NET_MGR_H

#include <yoc/event.h>
#include <sys/socket.h>
#include <devices/net.h>

typedef enum {
    ETH_MAC_SPEED_10M,
    ETH_MAC_SPEED_100M,
    ETH_MAC_SPEED_1000M,
} eth_mac_speed_e;

typedef enum {
    ETH_MAC_DUPLEX_HALF,
    ETH_MAC_DUPLEX_FULL,
} eth_mac_duplex_e;

typedef void *netmgr_hdl_t;

/* netmgr init */
netmgr_hdl_t netmgr_dev_eth_init();
netmgr_hdl_t netmgr_dev_gprs_init();
netmgr_hdl_t netmgr_dev_wifi_init();
netmgr_hdl_t netmgr_dev_nbiot_init();

netmgr_hdl_t netmgr_get_handle(const char *name);
dev_t *netmgr_handle_to_dev(netmgr_hdl_t);

/* netmgr config */
int netmgr_config_wifi(netmgr_hdl_t hdl, char *ssid, uint8_t ssid_length, char *psk, uint8_t psk_length);
int netmgr_config_gprs(netmgr_hdl_t hdl, int mode);
int netmgr_config_eth(netmgr_hdl_t hdl, eth_mac_duplex_e duplex, eth_mac_speed_e speed);
int netmgr_ipconfig(netmgr_hdl_t hdl, int dhcp_en, char *ipaddr, char *netmask, char *gw);

int netmgr_start(netmgr_hdl_t hdl);
int netmgr_reset(netmgr_hdl_t hdl, int sec);
int netmgr_stop(netmgr_hdl_t hdl);
int netmgr_is_gotip(netmgr_hdl_t hdl);
int netmgr_get_info(netmgr_hdl_t hdl);

#endif

