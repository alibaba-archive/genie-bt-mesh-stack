/*
 * Copyright (C) 2017-2019 C-SKY Microsystems Co., Ltd. All rights reserved.
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

/******************************************************************************
 * @file     lib_printf.c
 * @brief    lib printf file
 * @version  V1.0
 * @date     28. May 2019
 ******************************************************************************/

#include <csi_config.h>
#ifndef CONFIG_KERNEL_NONE
#include <k_api.h>
#endif
#include "lib_printf.h"

extern int putchar(int ch);

#define PAD_RIGHT 1
#define PAD_ZERO  2

#define PRINT_BUF_LEN 32

/***************************************************************************
 * Private Types
 ***************************************************************************/

/********************************************************************************
 * Private Function Prototypes
 ********************************************************************************/
static void printchar(char **str, int c)
{
    if (str) {
        **str = c;
        ++(*str);
    }

    (void)putchar(c);
}

static int prints(char **out, const char *string, int width, int pad)
{
    register int pc = 0, padchar = ' ';

    if (width > 0) {
        register int         len = 0;
        register const char *ptr;

        for (ptr = string; *ptr; ++ptr) {
            ++len;
        }

        if (len >= width) {
            width = 0;
        } else {
            width -= len;
        }

        if (pad & PAD_ZERO) {
            padchar = '0';
        }
    }

    if (!(pad & PAD_RIGHT)) {
        for (; width > 0; --width) {
            printchar(out, padchar);
            ++pc;
        }
    }

    for (; *string ; ++string) {
        printchar(out, *string);
        ++pc;
    }

    for (; width > 0; --width) {
        printchar(out, padchar);
        ++pc;
    }

    return pc;
}

static int printi(char **out, long long i, int b, int sg, int width, int pad, int letbase)
{
    char                  print_buf[PRINT_BUF_LEN];
    register char        *s;
    register int          t, neg = 0, pc = 0;
    register unsigned int u = i;

    if (i == 0) {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return prints(out, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0) {
        neg = 1;
        u   = -i;
    }

    s  = print_buf + PRINT_BUF_LEN - 1;
    *s = '\0';

    while (u) {
        t = u % b;

        if (t >= 10) {
            t += letbase - '0' - 10;
        }

        *--s = t + '0';
        u   /= b;
    }

    if (neg) {
        if (width && (pad & PAD_ZERO)) {
            printchar(out, '-');
            ++pc;
            --width;
        } else {
            *--s = '-';
        }
    }

    return pc + prints(out, s, width, pad);
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

int printf_mutex_init(void)
{
    return 0;
}

int print(char **out, const char *format, va_list args)
{
    register int width, pad;
    register int pc = 0;
    char         scr[2];

    for (; *format != 0; ++format) {
        if (*format == '%') {
            ++format;
            width = pad = 0;

            if (*format == '\0') {
                break;
            }

            if (*format == '%') {
                goto out;
            }

            if (*format == '-') {
                ++format;
                pad = PAD_RIGHT;
            }

            while (*format == '0') {
                ++format;
                pad |= PAD_ZERO;
            }

            for (; *format >= '0' && *format <= '9'; ++format) {
                width *= 10;
                width += *format - '0';
            }

            if (*format == 's') {
                register char *s = (char *)va_arg(args, int);
                pc += prints(out, s ? s : "(null)", width, pad);
                continue;
            }

            if (*format == 'd') {
                pc += printi(out, va_arg(args, int), 10, 1, width, pad, 'a');
                continue;
            }

            if ((*format == 'l') && (*(format + 1) == 'l') && (*(format + 2) == 'd')) {
                format++;
                format++;
                pc += printi(out, va_arg(args, long long), 10, 1, width, pad, 'a');
                continue;
            }

            if (*format == 'x') {
                pc += printi(out, va_arg(args, int), 16, 0, width, pad, 'a');
                continue;
            }

            if (*format == 'p') {
                printchar(out, '0');
                printchar(out, 'x');
                pc += printi(out, va_arg(args, int), 16, 0, width, pad, 'a');
                continue;
            }

            if ((*format == 'l') && (*(format + 1) == 'l') && (*(format + 2) == 'x')) {
                format++;
                format++;
                pc += printi(out, va_arg(args, long long), 16, 0, width, pad, 'a');
                continue;
            }

            if (*format == 'X') {
                pc += printi(out, va_arg(args, int), 16, 0, width, pad, 'A');
                continue;
            }

            if (*format == 'u') {
                pc += printi(out, va_arg(args, int), 10, 0, width, pad, 'a');
                continue;
            }

            if ((*format == 'l') && (*(format + 1) == 'l') && (*(format + 2) == 'u')) {
                format++;
                format++;
                pc += printi(out, va_arg(args, long long), 10, 0, width, pad, 'a');
                continue;
            }

            if (*format == 'c') {
                /* char are converted to int then pushed on the stack */
                scr[0] = (char)va_arg(args, int);
                scr[1] = '\0';
                pc += prints(out, scr, width, pad);
                continue;
            }
        } else {
out:
            printchar(out, *format);
            ++pc;
        }
    }

    if (out) {
        **out = '\0';
    }

    va_end(args);
    return pc;
}

__attribute__((weak))  int printf(const char *format, ...)
{
    va_list args;

    va_start(args, format);

#ifndef CONFIG_KERNEL_NONE
    krhino_sched_disable();
    print(0, format, args);
    krhino_sched_enable();
#else
    print(0, format, args);
#endif

    va_end(args);

    return 0;
}

int printk(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    print(0, format, args);
    va_end(args);

    return 0;
}

#if 0
int sprintf(char *out, const char *format, ...)
{
    va_list args;

    va_start(args, format);

    yunos_mutex_lock(&mutex_io, YUNOS_WAIT_FOREVER);
    print(&out, format, args);
    yunos_mutex_unlock(&mutex_io);

    return 0;
}
#endif
