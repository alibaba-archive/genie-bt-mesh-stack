#ifndef PUT_CHAR_SIM_H
#define PUT_CHAR_SIM_H
#include "tl_common.h"

int sim_putchar(int c);
int sim_printf(const char *format, ...);
int sim_printf_buf(unsigned char *buf, int len, const char *format, ...);
int printf_Bin2Text (char *lpD, int lpD_len_max, char *lpS, int n);


void hal_uart_send_loop(int wait_flag);
void hal_uart_send_print(u16 len);

int uart_putc(char byte);
#define tc8258_printf	    sim_printf
#define tc8258_printf_buf   sim_printf_buf
#endif
