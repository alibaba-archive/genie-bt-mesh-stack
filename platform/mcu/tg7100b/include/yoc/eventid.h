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

#ifndef YOC_EVENT_ID_H
#define YOC_EVENT_ID_H

#include "event.h"


/* net driver event
*  from 0x100 to 0x1FF
*  net.h
*/

#define EVENT_NETMGR_GOT_IP          0x01011
#define EVENT_NETMGR_NET_DISCON      0x01012

#define EVENT_IOT_CONNECT_SUCCESS    0x01030
#define EVENT_IOT_CONNECT_FAILED     0x01031
#define EVENT_IOT_DISCONNECTED       0x01032
#define EVENT_IOT_PUSH_SUCCESS       0x01034
#define EVENT_IOT_PUSH_FAILED        0x01035

#define EVENT_OS_REBOOT              0x01050

#define EVENT_FOTA_START             0x01060
#define EVENT_FOTA_UPDATE            0x01061

#define EVENT_BLE                    0x05000000

#define EVENT_BLE_MESH               0x05100000

#define EVENT_USER                   0x10000000

#endif
