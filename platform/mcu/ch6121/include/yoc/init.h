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

#ifndef YOC_INCLUDE_INIT_H
#define YOC_INCLUDE_INIT_H

#include <stdint.h>
#include <yoc/uservice.h>

/* board */
void board_init(void);
void board_base_init(void);

/* service */
extern void console_init(int id, uint32_t baud, uint16_t buf_size);
extern const char *console_get_devname(void);
extern uint16_t console_get_buffer_size(void);

extern int  lwip_tcpip_init(void);
extern void at_server_init(utask_t *task);
extern void netmgr_service_init(utask_t *task);

#endif
