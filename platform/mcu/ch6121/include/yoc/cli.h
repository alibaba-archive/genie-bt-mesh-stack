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

#ifndef YOC_CLI_H
#define YOC_CLI_H

#include <aos/list.h>
#include <yoc/uservice.h>

#define CLI_CMD_LEN_MAX 256

#define CLI_CMD_MAX_ARG_NUM 11

/* Structure for registering CLI commands */
struct cli_command {
    const char *name;
    const char *help;
    void (*function)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
};

typedef struct cmd_list {
    const struct cli_command *cmd;
    slist_t                   next;
} cmd_list_t;

void    cli_service_init(utask_t *task);
int     cli_service_stop(void);
int     cli_service_reg_cmd(const struct cli_command *info);
int     cli_service_unreg_cmd(const struct cli_command *info);
int     cli_service_reg_cmds(const struct cli_command commands[], int num_commands);
int     cli_service_unreg_cmds(const struct cli_command commands[], int num_commands);
slist_t cli_service_get_cmd_list(void);


// default command list
extern void cli_reg_cmd_help(void);
extern void cli_reg_cmd_ping(void);
extern void cli_reg_cmd_ifconfig(void);
extern void cli_reg_cmd_ifconfig_wifi(void);
extern void cli_reg_cmd_ifconfig_eth(void);
extern void cli_reg_cmd_ifconfig_gprs(void);

extern void cli_reg_cmd_ntp(void);
extern void cli_reg_cmd_ps(void);
extern void cli_reg_cmd_free(void);
extern void cli_reg_cmd_factory(void);
extern void cli_reg_cmd_sysinfo(void);
extern void cli_reg_cmd_sysconf(void);
extern void cli_reg_cmd_addr(void);
extern void cli_reg_cmd_lpm(void);
extern void cli_reg_cmd_kvtool(void);
extern void cli_test_register(void);
extern void cli_reg_cmd_ble();

extern void cli_reg_cmd_ls(void);
extern void cli_reg_cmd_rm(void);
extern void cli_reg_cmd_cat(void);
extern void cli_reg_cmd_mkdir(void);
extern void cli_reg_cmd_mv(void);
extern void cli_reg_cmd_mkfatfs(void);

extern void test_yunit_test_register_cmd();

#endif
