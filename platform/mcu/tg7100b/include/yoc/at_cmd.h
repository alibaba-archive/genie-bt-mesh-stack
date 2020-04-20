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

#ifndef YOC_AT_UART_BASIC_CMD_H
#define YOC_AT_UART_BASIC_CMD_H

#include <sys/types.h>

// basic:
#define AT              {"AT", at_cmd_at}
#define AT_HELP         {"AT+HELP", at_cmd_help}
#define AT_CGMR         {"AT+CGMR", at_cmd_cgmr}
#define AT_FWVER        {"AT+FWVER", at_cmd_fwver}
#define AT_SYSTIME      {"AT+SYSTIME", at_cmd_systime}
#define AT_SAVE         {"AT+SAVE", at_cmd_save}
#define AT_FACTORYW     {"AT-FACTORYW", at_cmd_factory_w}
#define AT_FACTORYR     {"AT+FACTORYR", at_cmd_factory_r}
#define AT_REBOOT       {"AT+REBOOT", at_cmd_reboot}
#define AT_EVENT        {"AT+EVENT", at_cmd_event}
#define AT_ECHO         {"AT+ECHO", at_cmd_echo}
#define AT_SLEEP        {"AT+SLEEP", at_cmd_sleep}
#define AT_MODEL        {"AT+MODEL", at_cmd_model}

// kv cmd
#define AT_KVGET        {"AT+KVGET", at_cmd_kv_get}
#define AT_KVSET        {"AT+KVSET", at_cmd_kv_set}
#define AT_KVDEL        {"AT+KVDEL", at_cmd_kv_del}
#define AT_KVGETINT     {"AT+KVGETINT", at_cmd_kv_getint}
#define AT_KVSETINT     {"AT+KVSETINT", at_cmd_kv_setint}
#define AT_KVDELINT     {"AT+KVDELINT", at_cmd_kv_delint}

// ali cloud
#if 1
#define AT_ALIMQTT_PUB        {"AT+PUB", at_cmd_alimqtt_pub}
#define AT_ALIMQTT_CONN       {"AT+ALIYUNCONN", at_cmd_alimqtt_conn}
#define AT_ALIMQTT_DISCONN    {"AT+ALIYUNDISCONN", at_cmd_alimqtt_disconn}

#define AT_ALICOAP_PUB        {"AT+PUB", at_cmd_alicoap_pub}
#define AT_ALICOAP_CONN       {"AT+ALIYUNCONN", at_cmd_alicoap_conn}
#define AT_ALICOAP_DISCONN    {"AT+ALIYUNDISCONN", at_cmd_alicoap_disconn}
#endif
#define AT_ALI_PUB              {"AT+PUB", at_cmd_pub}
#define AT_ALI_CONN             {"AT+ALIYUNCONN", at_cmd_aliconn}
#define AT_ALI_DISCONN          {"AT+ALIYUNDISCONN", at_cmd_alidisconn}
//onenet
#define AT_MIPLCREATE           {"AT+MIPLCREATE", at_cmd_onet_miplcreate}
#define AT_MIPLDELETE           {"AT+MIPLDELETE", at_cmd_onet_mipldel}
#define AT_MIPLOPEN             {"AT+MIPLOPEN", at_cmd_onet_miplopen}
#define AT_MIPLADDOBJ           {"AT+MIPLADDOBJ", at_cmd_onet_mipladdobj}
#define AT_MIPLDELOBJ           {"AT+MIPLDELOBJ", at_cmd_onet_mipldelobj}
#define AT_MIPLCLOSE            {"AT+MIPLCLOSE", at_cmd_onet_miplclose}
#define AT_MIPLNOTIFY           {"AT+MIPLNOTIFY", at_cmd_onet_miplnotify}
#define AT_MIPLREADRSP          {"AT+MIPLREADRSP", at_cmd_onet_miplreadrsp}
#define AT_MIPLWRITERSP         {"AT+MIPLWRITERSP", at_cmd_onet_miplwritersp}
#define AT_MIPLEXECUTERSP       {"AT+MIPLEXECUTERSP", at_cmd_onet_miplexecutersp}
#define AT_MIPLOBSERVERSP       {"AT+MIPLOBSERVERSP", at_cmd_onet_miplobserveresp}
#define AT_MIPLDISCOVERRSP      {"AT+MIPLDISCOVERRSP", at_cmd_onet_mipldiscoverresp}
#define AT_MIPLPARAMETERRSP     {"AT+MIPLPARAMETERRSP", at_cmd_onet_miplparameterresp}
#define AT_MIPLUPDATE           {"AT+MIPLUPDATE", at_cmd_onet_miplupdate}
#define AT_MIPLVER              {"AT+MIPLVER", at_cmd_onet_miplver}
#define AT_COPREG               {"AT+COPREG", at_cmd_onet_copreg}

// socket
#define AT_CIPSTART         {"AT+CIPSTART", at_cmd_cip_start}
#define AT_CIPSTOP          {"AT+CIPSTOP", at_cmd_cip_stop}
#define AT_CIPRECVCFG       {"AT+CIPRECVCFG", at_cmd_cip_recv_cfg}
#define AT_CIPID            {"AT+CIPID", at_cmd_cip_id}
#define AT_CIPSTATUS        {"AT+CIPSTATUS", at_cmd_cip_status}
#define AT_CIPSEND          {"AT+CIPSEND", at_cmd_cip_send}
#define AT_CIPSENDPSM       {"AT+CIPSENDPSM", at_cmd_cip_sendpsm}
#define AT_CIPRECV          {"AT+CIPRECV", at_cmd_cip_recv}

// ztw socket
#define AT_ZIPOPEN      {"AT+ZIPOPEN", at_cmd_zip_open}
#define AT_ZIPSEND      {"AT+ZIPSEND", at_cmd_zip_send}
#define AT_ZIPCLOSE     {"AT+ZIPCLOSE", at_cmd_zip_close}
#define AT_ZIPSTAT      {"AT+ZIPSTAT", at_cmd_zip_stat}
#define AT_ZDTMODE      {"AT+ZDTMODE", at_cmd_zdt_mode}

// hisi socket
#define AT_NSOCR        {"AT+NSOCR", at_cmd_nsocr}
#define AT_NSOST        {"AT+NSOST", at_cmd_nsost}
#define AT_NSOSTF       {"AT+NSOSTF", at_cmd_nsostf}
#define AT_NSORF        {"AT+NSORF", at_cmd_nsorf}
#define AT_NSOCL        {"AT+NSOCL", at_cmd_nsocl}


// CHIP_ZX297100
#define AT_ZNVSET       {"AT+ZNVSET", at_cmd_znvset}
#define AT_ZNVGET       {"AT+ZNVGET", at_cmd_znvget}
#define AT_RAMDUMP      {"AT^RAMDUMP", at_cmd_ramdump}
#define AT_EXTRTC       {"AT+EXTRTC", at_cmd_extrtc}
#define AT_LPMENTER     {"AT+LPMENTER", at_cmd_quick_lpm}

// AMT
#define AT_ZFLAG            {"AT+ZFLAG", at_cmd_zflag}
#define AT_AMTDEMO          {"AT+AMTDEMO", at_cmd_amtdemo}
#define AT_BOARDNUM         {"AT+BOARDNUM", at_cmd_boardnum}
#define AT_MSN              {"AT+MSN", at_cmd_msn}
#define AT_PRODTEST         {"AT+PRODTEST", at_cmd_prodtest}
#define AT_RTESTINFO        {"AT+RTESTINFO", at_cmd_rtestinfo}
#define AT_ZVERSIONTYPE     {"AT+ZVERSIONTYPE", at_cmd_zversiontype}
#define AT_PLATFORM         {"AT+PLATFORM", at_cmd_chip_platform}

// fota
#define AT_FOTASTART        {"AT+FOTASTART", at_cmd_fotastart}
#define AT_FOTAGETFULL      {"AT+FOTAGETFULL", at_cmd_fotagetfull}
#define AT_FOTASTOP         {"AT+FOTASTOP", at_cmd_fotastop}
#define AT_FOTAGETDIFF      {"AT+FOTAGETDIFF", at_cmd_fotagetdiff}

#define AT_NULL             {NULL,NULL}

/* basic cmd */
void at_cmd_at(char *cmd, int type, char *data);
void at_cmd_help(char *cmd, int type, char *data);
void at_cmd_cgmr(char *cmd, int type, char *data);
void at_cmd_fwver(char *cmd, int type, char *data);
void at_cmd_systime(char *cmd, int type, char *data);
void at_cmd_save(char *cmd, int type, char *data);
void at_cmd_factory_w(char *cmd, int type, char *data);
void at_cmd_factory_r(char *cmd, int type, char *data);
void at_cmd_reboot(char *cmd, int type, char *data);
void at_cmd_event(char *cmd, int type, char *data);
void at_cmd_echo(char *cmd, int type, char *data);
void at_cmd_model(char *cmd, int type, char *data);
void at_cmd_sleep(char *cmd, int type, char *data);

/* service cmd */
void at_cmd_fotastart(char *cmd, int type, char *data);
void at_cmd_fotastop(char *cmd, int type, char *data);
void at_cmd_fotagetfull(char *cmd, int type, char *data);
void at_cmd_fotagetdiff(char *cmd, int type, char *data);
void at_cmd_fota(char *cmd, int type, char *data);
void at_cmd_fotaurl(char *cmd, int type, char *data);
void at_cmd_kv_set(char *cmd, int type, char *data);
void at_cmd_kv_get(char *cmd, int type, char *data);
void at_cmd_kv_del(char *cmd, int type, char *data);
void at_cmd_kv_setint(char *cmd, int type, char *data);
void at_cmd_kv_getint(char *cmd, int type, char *data);
void at_cmd_kv_delint(char *cmd, int type, char *data);


/* cloud cmd */
void at_cmd_pub(char *cmd, int type, char *data);
void at_cmd_aliconn(char *cmd, int type, char *data);
void at_cmd_alidisconn(char *cmd, int type, char *data);

void at_cmd_alicoap_pub(char *cmd, int type, char *data);
void at_cmd_alicoap_conn(char *cmd, int type, char *data);
void at_cmd_alicoap_disconn(char *cmd, int type, char *data);

void at_cmd_alimqtt_pub(char *cmd, int type, char *data);
void at_cmd_alimqtt_conn(char *cmd, int type, char *data);
void at_cmd_alimqtt_disconn(char *cmd, int type, char *data);
//void at_setup_cmd_conntimeout(uint8_t id, uint16_t len, uint8_t *data);
//void at_query_cmd_conntimeout(uint8_t id);



void at_cmd_cip_start(char *cmd, int type, char *data);
void at_cmd_cip_stop(char *cmd, int type, char *data);
void at_cmd_cip_recv_cfg(char *cmd, int type, char *data);
void at_cmd_cip_id(char *cmd, int type, char *data);
void at_cmd_cip_status(char *cmd, int type, char *data);
void at_cmd_cip_send(char *cmd, int type, char *data);
#ifdef CONFIG_YOC_LPM
void at_cmd_cip_sendpsm(char *cmd, int type, char *data);
#endif
void at_cmd_cip_recv(char *cmd, int type, char *data);


#ifdef CONFIG_FOTA
void at_cmd_fotastart(char *cmd, int type, char *data);
void at_cmd_fotagetfull(char *cmd, int type, char *data);
void at_cmd_fotastop(char *cmd, int type, char *data);
void at_cmd_fotagetdiff(char *cmd, int type, char *data);
#endif

// onenet AT command
void at_cmd_onet_miplcreate(char *cmd, int type, char *data);
void at_cmd_onet_mipldel(char *cmd, int type, char *data);
void at_cmd_onet_miplopen(char *cmd, int type, char *data);
void at_cmd_onet_mipladdobj(char *cmd, int type, char *data);
void at_cmd_onet_mipldelobj(char *cmd, int type, char *data);
void at_cmd_onet_miplclose(char *cmd, int type, char *data);
void at_cmd_onet_miplnotify(char *cmd, int type, char *data);
void at_cmd_onet_miplreadrsp(char *cmd, int type, char *data);
void at_cmd_onet_miplwritersp(char *cmd, int type, char *data);
void at_cmd_onet_miplexecutersp(char *cmd, int type, char *data);
void at_cmd_onet_miplobserveresp(char *cmd, int type, char *data);
void at_cmd_onet_mipldiscoverresp(char *cmd, int type, char *data);
void at_cmd_onet_miplparameterresp(char *cmd, int type, char *data);
void at_cmd_onet_miplupdate(char *cmd, int type, char *data);
void at_cmd_onet_miplver(char *cmd, int type, char *data);
void at_cmd_onet_copreg(char *cmd, int type, char *data);

// ZTW AT command
void at_cmd_zip_open(char *cmd, int type, char *data);
void at_cmd_zip_send(char *cmd, int type, char *data);
void at_cmd_zip_close(char *cmd, int type, char *data);
void at_cmd_zip_stat(char *cmd, int type, char *data);
void at_cmd_zdt_mode(char *cmd, int type, char *data);

// HISI AT command
void at_cmd_nsocr(char *cmd, int type, char *data);
void at_cmd_nsost(char *cmd, int type, char *data);
void at_cmd_nsostf(char *cmd, int type, char *data);
void at_cmd_nsorf(char *cmd, int type, char *data);
void at_cmd_nsocl(char *cmd, int type, char *data);

#if defined(CONFIG_CHIP_ZX297100)
// ZX297100 AT command
void at_cmd_znvset(char *cmd, int type, char *data);
void at_cmd_znvget(char *cmd, int type, char *data);
void at_cmd_ramdump(char *cmd, int type, char *data);
#ifdef CONFIG_YOC_LPM
void at_cmd_extrtc(char *cmd, int type, char *data);
void at_cmd_quick_lpm(char *cmd, int type, char *data);
#endif
#ifdef CONFIG_AMT
void at_cmd_zflag(char *cmd, int type, char *data);
void at_cmd_amtdemo(char *cmd, int type, char *data);
void at_cmd_boardnum(char *cmd, int type, char *data);
void at_cmd_msn(char *cmd, int type, char *data);
void at_cmd_prodtest(char *cmd, int type, char *data);
void at_cmd_rtestinfo(char *cmd, int type, char *data);
void at_cmd_zversiontype(char *cmd, int type, char *data);
void at_cmd_chip_platform(char *cmd, int type, char *data);
#endif
#endif

#endif
