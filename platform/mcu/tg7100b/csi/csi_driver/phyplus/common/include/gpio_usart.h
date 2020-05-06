#ifndef _GPIO_USART_H_
#define _GPIO_USART_H_

#include <drv_usart.h>

int drv_gpio_usart_init(int32_t gpio_pin);
int drv_gpio_usart_config(uint32_t baud,
                          usart_mode_e mode,
                          usart_parity_e parity,
                          usart_stop_bits_e stopbits,
                          usart_data_bits_e bits);
int drv_gpio_usart_send_char(char ch);

#endif
