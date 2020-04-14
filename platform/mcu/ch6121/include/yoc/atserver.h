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

#ifndef YOC_AT_SERVER_H
#define YOC_AT_SERVER_H

#include <stdarg.h>

#include <yoc/uservice.h>
#include <devices/uart.h>

#define BUFFER_MIN_SIZE (128)
#define BUFFER_MAX_SIZE (1024)
#define BUFFER_STEP_SIZE (32)

typedef enum {
    TEST_CMD,
    READ_CMD,
    WRITE_CMD,
    EXECUTE_CMD,

    INVAILD_CMD
} AT_CMD_TYPE;

typedef void (*cmd_callback)(char *cmd, int type, char *data);

typedef struct atserver_cmd {
    const char        *cmd;
    const cmd_callback cb;
} atserver_cmd_t;

int  atserver_init(utask_t *task, const char *name, uart_config_t *config);
int  atserver_send(const char *format, ...);
int  atserver_sendv(const char *command, va_list args);
int  atserver_write(const void *data, int size);
int  atserver_cmd_link(const atserver_cmd_t *at_cmd);
int  atserver_add_command(const atserver_cmd_t at_cmd[]);
void atserver_set_output_terminator(const char *terminator);
void atserver_set_para_delimiter(char delimiter);
void atserver_set_timeout(int timeout);
int  atserver_scanf(const char *format, ...);
uint8_t atserver_get_echo(void);
void atserver_set_echo(uint8_t flag);
void atserver_lock(void);
void atserver_unlock(void);

#endif
