/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

/*******************************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <math.h>
#include "log.h"
#include <aos/aos.h>

int char2hex(const char *c, uint8_t *x)
{
    if (*c >= '0' && *c <= '9') {
        *x = *c - '0';
    } else if (*c >= 'a' && *c <= 'f') {
        *x = *c - 'a' + 10;
    } else if (*c >= 'A' && *c <= 'F') {
        *x = *c - 'A' + 10;
    } else {
        return -EINVAL;
    }

    return 0;
}

int str2_char(const char *str, uint8_t *addr)
{
    int i, j;
    uint8_t tmp;

    if (strlen(str) != 17) {
        return -EINVAL;
    }

    for (i = 0, j = 1; *str != '\0'; str++, j++) {
        if (!(j % 3) && (*str != ':')) {
            return -EINVAL;
        } else if (*str == ':') {
            i++;
            continue;
        }

        addr[i] = addr[i] << 4;

        if (char2hex(str, &tmp) < 0) {
            return -EINVAL;
        }

        addr[i] |= tmp;
    }

    return 0;
}

int int_num_check(char *data)
{
    if ((*data != '-')&&(*data != '+')&&((*data < 0x30)||(*data >0x39))){
        return  -1;
    }
    for (int i =1; *(data+i) != '\0'; i++) {
        if ((*(data+i) < 0x30)||(*(data+i) > 0x39)) {
            return  -1;
        }
    }
    return  0;
}

char *str_chr(char *d, char *s, int c)
{
    int i = 0;
    char  *q = d;
    char  *p = s;

    for (i = 0; * (s + i) != (char) c; i++) {
        *(q++) = *(p++);

        if (*(s + i) == '\0') {
            return NULL;
        }
    }

    *(q++) = '\0';
    return  d;
}

int  argc_len(char *s)
{
    int i = 0;
    int j = 0;

    for (i = 0; * (s + i) != '\0'; i++) {
        if ((*(s + i) == ',') || (*(s + i) == '?') || (*(s + i) == '=')) {
            j++;
        }
    }
    return j;
}

char *char_cut(char *d, char *s, int b , int e)
{
    char *stc;

    if (b == '\0') {
        return NULL;
    }

    stc = strchr(s, b);

    if (stc == NULL) {
        printf("not execute\r\n");
        return NULL;
    }

    stc++;
    str_chr(d, stc, e);

    return d;//(char *)d;
}
