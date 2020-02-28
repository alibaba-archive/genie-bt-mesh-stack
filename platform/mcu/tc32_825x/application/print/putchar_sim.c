/********************************************************************************************************
 * @file     putchar_sim.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#include "application/print/putchar_sim.h"
#include "tl_common.h"
#include "drivers.h"
#include "vendor/common/alios_app_config.h"
#include <stdarg.h>
#include <stdio.h>
#include "drivers/8258/irq.h"

#if(PRINT_ENABLE)
#if(IS_PRINT_USE_SIM)
#ifndef		BIT_INTERVAL
#define		BIT_INTERVAL		(16000000/PRINT_BAUD_RATE)
#endif

_attribute_ram_code_ int uart_putc(char byte) //GPIO simulate uart print func
{
	unsigned char  j = 0;
	unsigned int t1 = 0,t2 = 0;

	REG_ADDR8(0x582+((DEBUG_INFO_TX_PIN>>8)<<3)) &= ~(DEBUG_INFO_TX_PIN & 0xff) ;//Enable output

	unsigned int  pcTxReg = (0x583+((DEBUG_INFO_TX_PIN>>8)<<3));//register GPIO output
	unsigned char tmp_bit0 = read_reg8(pcTxReg) & (~(DEBUG_INFO_TX_PIN & 0xff));
	unsigned char tmp_bit1 = read_reg8(pcTxReg) | (DEBUG_INFO_TX_PIN & 0xff);
	unsigned char bit[10] = {0};

	bit[0] = tmp_bit0;
	bit[1] = (byte & 0x01)? tmp_bit1 : tmp_bit0;
	bit[2] = ((byte>>1) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[3] = ((byte>>2) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[4] = ((byte>>3) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[5] = ((byte>>4) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[6] = ((byte>>5) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[7] = ((byte>>6) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[8] = ((byte>>7) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[9] = tmp_bit1;

	unsigned char r = irq_disable();
	t1 = read_reg32(0x740);
	for(j = 0;j<10;j++)
	{
		t2 = t1;
		while(t1 - t2 < BIT_INTERVAL){
			t1  = read_reg32(0x740);
		}
		write_reg8(pcTxReg,bit[j]);        //send bit0
	}
	irq_restore(r);

	return byte;
}

int sim_putchar(int c)
{
	return uart_putc((char)c);
}

#else
//#include "hal/soc/uart.h"
int sim_putchar(int c)
{
	return (int)hal_uart_send(NULL,&c,1,10*1000);
}
#endif

static void sim_printchar(char **str, int c) {
	if (str) {
		**str = c;
		++(*str);
	} else
		(void) sim_putchar(c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int sim_prints(char **out, const char *string, int width, int pad) {
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr)
			++len;
		if (len >= width)
			width = 0;
		else
			width -= len;
		if (pad & PAD_ZERO)
			padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for (; width > 0; --width) {
			sim_printchar(out, padchar);
			++pc;
		}
	}
	for (; *string; ++string) {
		sim_printchar(out, *string);
		++pc;
	}
	for (; width > 0; --width) {
		sim_printchar(out, padchar);
		++pc;
	}

	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int sim_printi(char **out, int i, int b, int sg, int width, int pad,
		int letbase) {
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return sim_prints(out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN - 1;
	*s = '\0';

	while (u) {
		t = u % b;
		if (t >= 10)
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if (width && (pad & PAD_ZERO)) {
			sim_printchar(out, '-');
			++pc;
			--width;
		} else {
			*--s = '-';
		}
	}

	return pc + sim_prints(out, s, width, pad);
}

int sim_print(char **out, const char *format, va_list args) {
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0')
				break;
			if (*format == '%')
				goto out;
			if (*format == 'z'){
                ++format;   // skip now, confirm later
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
				register char *s = (char *) va_arg( args, int );
				pc += sim_prints(out, s ? s : "(null)", width, pad);
				continue;
			}
			if (*format == 'd') {
				pc += sim_printi(out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if (*format == 'p') {
                //uart_putc('0');
                //uart_putc('x');
				sim_putchar('0');
				sim_putchar('x');
                pc += sim_printi(out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if (*format == 'x') {
				pc += sim_printi(out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if (*format == 'X') {
				pc += sim_printi(out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if (*format == 'u') {
				pc += sim_printi(out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if (*format == 'c') {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char) va_arg( args, int );
				scr[1] = '\0';
				pc += sim_prints(out, scr, width, pad);
				continue;
			}
		} else {
		    if(('\n' == *format) && (*(format-1) != '\r')){
				sim_putchar('\r');  // always enter
		    }
			out: sim_printchar(out, *format);
			++pc;
		}
	}
	if (out)
		**out = '\0';
	va_end( args );
	
#if HW_UART_RING_BUF_EN
    hal_uart_send_print(pc);
#endif

	return pc;
}

int sim_printf(const char *format, ...) {
	va_list args;
	va_start( args, format );
	return sim_print(0, format, args);
}

int sim_printf_buf(unsigned char *buf, int len, const char *format, ...) {
	register int pc = 0;
    
	va_list args;
	va_start( args, format );
	pc += sim_print(0, format, args);
	
    if(len){
        char log_dst[256];// = {0};     // make sure enough RAM
    	printf_Bin2Text((char *)(log_dst), sizeof(log_dst), (char *)(buf), len);
        pc += sim_printf("%s\r\n", log_dst);
    }
	return pc;
}


// ------- BIN2TEXT
const char printf_arrb2t[] = "0123456789abcdef";

u32 get_len_Bin2Text(u32 buf_len)
{
    return (buf_len*3 + (((buf_len+15)/16)*(1+2))); // 1: space, 2: /r/n
}

int printf_Bin2Text (char *lpD, int lpD_len_max, char *lpS, int n)
{
    int i = 0;
	int m = n;
	int d = 0;

    #define LINE_MAX_LOG        (6)
	if(m > BIT(LINE_MAX_LOG)){
	    if(lpD_len_max > 2){
    		lpD[d++] = '\r';
    		lpD[d++] = '\n';
    		lpD_len_max -= 2;
		}
	}
	
	for (i=0; i<m; i++) {
	    if((d + 6 + 2) > lpD_len_max){
	        break;
	    }
	    
        if((0 == (i & BIT_MASK_LEN(LINE_MAX_LOG))) && i){
            lpD[d++] = '\r';
            lpD[d++] = '\n';
        }

		lpD[d++] = printf_arrb2t [(lpS[i]>>4) & 15];
		lpD[d++] = printf_arrb2t [lpS[i] & 15];

		if((i & BIT_MASK_LEN(LINE_MAX_LOG)) != BIT_MASK_LEN(LINE_MAX_LOG)){	// end of line
    		lpD[d++] = ' ';

            if(m > BIT(LINE_MAX_LOG)){
    		    if ((i&7)==7){
    		        lpD[d++] = ' ';
    		    }
    		}
		}
	}

    #if 1   // just conver into string
    if(lpD_len_max >= d){
        lpD[d-1] = '\0';
    }else{
        lpD[d++] = '\0';
    }
    #else
	if(lpD_len_max >= d+2){
    	lpD[d++] = '\r';    // lpS is always ture here. so can't distinguish whether there is buffer or not.
    	lpD[d++] = '\n';
        // lpD[d++] = '\0';        // can't add 0, because some UART Tool will show error.
    }
    #endif
    
	return d;
}
//--- end


#endif

#if 0
#include <k_api.h>
#include <hal/soc/soc.h>

#define u32 uint32_t

int32_t hal_uart_send(uart_dev_t *p_uart_dev, const void *data, uint32_t size, uint32_t timeout)
{
	for (int i = 0; i < size; i++) 
  	{
   		 //uart_sim_send_one_byte(((uint8_t *)data)[i]);
			uart_sim_send_one_byte(0x11);
  	}
}
#endif

#if 0
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)

PUTCHAR_PROTOTYPE
{
  //u8 r = irq_disable();
  #if(PRINT_ENABLE)
  uart_putc(ch);
  #endif
  //irq_restore(r);	
}
#endif

