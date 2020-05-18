/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef __DUT_UTILITY_H_
#define __DUT_UTILITY_H_

int char2hex(const char *c, uint8_t *x);
int str2_char(const char *str, uint8_t *addr);
int int_num_check(char *data);
char *str_chr(char *d, char *s, int c);
int argc_len(char *s);
char *char_cut(char *d, char *s, int b , int e);

#endif
