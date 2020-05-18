/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef __DUT_AT_CMD_H_
#define __DUT_AT_CMD_H_

int dut_cmd_rx_mode(int argc, char *argv[]);

int dut_cmd_ftest(int argc, char *argv[]);

int dut_cmd_sleep(int argc, char *argv[]);

int dut_cmd_ireboot(int argc, char *argv[]);

int dut_cmd_opt_mac(int argc, char *argv[]);
int fdut_cmd_opt_mac(int argc, char *argv[]);

int  dut_cmd_gpio(int argc, char *argv[]);
int  fdut_cmd_gpio(int argc, char *argv[]);

int dut_cmd_freq_off(int argc, char *argv[]);
int fdut_cmd_freq_off(int argc, char *argv[]);

int  fdut_cmd_trituple_component(int argc, char *argv[]);
int  dut_cmd_trituple_component(int argc, char *argv[]);

int  dut_cmd_flash(int argc, char *argv[]);
int  fdut_cmd_flash(int argc, char *argv[]);

void dut_at_cmd_init(void);

#endif
