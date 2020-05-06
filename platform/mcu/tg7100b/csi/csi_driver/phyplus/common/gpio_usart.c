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

#include <stdint.h>
#include <ap_cp.h>
#include <io.h>
#include <drv_gpio.h>
#include <gpio_usart.h>
#include <pinmux.h>

#define ERR_USART(errno) (CSI_DRV_ERRNO_USART_BASE | errno)

#define TIMER_LOAD_MAX 0xFFFFFF
#define GPIO_TIMER     AP_TIM4

typedef struct _gpio_uart_conf_t {
    uint8_t databit;
    uint8_t stop;
    uint8_t mode;
    uint8_t parity;
    uint32_t baudrate;
} gpio_uart_conf_t;

typedef struct _gpio_uart_t {
    gpio_uart_conf_t config;
    int32_t tx_pin;
    gpio_pin_handle_t tx_gpio_handle;
    int32_t bit_delay;
} gpio_uart_t;

static gpio_uart_t uart_env;
static uint8_t gpio_uart_inited = 0;
static uint8_t gpio_uart_config = 0;

static void tx_gpio_write(int32_t pin, uint8_t on)
{
    if (pin < 18) {
        if (on) {
            set_bit(pin, (volatile uint32_t *)AP_GPIOA_BASE);
        } else {
            clear_bit(pin, (volatile uint32_t *)AP_GPIOA_BASE);       //set pin input(clear bit)
        }
    } else {
        if (on) {
            set_bit(pin - 18, (volatile uint32_t *)AP_GPIOB_BASE);
        } else {
            clear_bit(pin - 18, (volatile uint32_t *)AP_GPIOB_BASE);      //set pin input(clear bit)
        }
    }
}

static void delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t cur;
    start = GPIO_TIMER->CurrentCount;

    while (1) {
        cur = GPIO_TIMER->CurrentCount;

        if (start >= cur) {
            if (start - cur >= (us << 2)) {
                return;
            }
        } else {
            if (TIMER_LOAD_MAX - cur + start >  us << 2) {
                return;
            }
        }
    }

}

static int set_gpio_uart_timer()
{
    AP_PCR->CLKG |= BIT(MOD_TIMER); //enable timer clock gate

    GPIO_TIMER->ControlReg = 0x0; //disable timer
    GPIO_TIMER->LoadCount =  TIMER_LOAD_MAX; //set max load value
    GPIO_TIMER->ControlReg = 0x7; // mask int & free mode & enable timer
    return 0;
}

int drv_gpio_usart_send_char(char ch)
{
    if (!gpio_uart_config) {
        return ERR_USART(DRV_ERROR);
    }

    uint8_t data_bits = uart_env.config.databit + 5;
    uint8_t bit_send;
    int32_t tx_pin = uart_env.tx_pin;
    int32_t bit_delay = uart_env.bit_delay;
    uint8_t parity_t = 0;
    int ret;

    halIntState_t cs;
    HAL_ENTER_CRITICAL_SECTION(cs);

    //START BIT
    //this operation will result in about 5us low I/O
    ret = csi_gpio_pin_config_direction(uart_env.tx_gpio_handle, GPIO_DIRECTION_OUTPUT);

    if (ret) {
        return ret;
    }

    tx_gpio_write(uart_env.tx_pin, 0);

    delay_us(bit_delay - 5);

    //data bit
    for (int i = 0; i < data_bits; i++) {
        bit_send = (ch >> i) & 0x1;
        tx_gpio_write(tx_pin, bit_send);
        parity_t ^= bit_send;
        delay_us(bit_delay);
    }

    //parity bit
    if (uart_env.config.parity != USART_PARITY_NONE) {
        if (uart_env.config.parity == USART_PARITY_ODD) {
            bit_send = (~parity_t) & 1;
        } else {
            bit_send = parity_t;
        }

        tx_gpio_write(tx_pin, bit_send);
        delay_us(bit_delay);
    }

    //stop bit
    tx_gpio_write(tx_pin, 1);
    delay_us(bit_delay);

    HAL_EXIT_CRITICAL_SECTION(cs);

    return 0;
}

int drv_gpio_usart_config(uint32_t baud,
                          usart_mode_e mode,
                          usart_parity_e parity,
                          usart_stop_bits_e stopbits,
                          usart_data_bits_e bits)
{
    if (gpio_uart_inited == 0) {
        return ERR_USART(DRV_ERROR);
    }

    if (baud != 38400 && baud != 9600) {
        return ERR_USART(USART_ERROR_BAUDRATE);
    }

    if (parity != USART_PARITY_NONE && parity != USART_PARITY_EVEN
        && parity != USART_PARITY_ODD) {
        return ERR_USART(USART_ERROR_PARITY);
    }

    if (stopbits != USART_STOP_BITS_1) {
        return ERR_USART(USART_ERROR_STOP_BITS);
    }

    if (bits != USART_DATA_BITS_8) {
        return ERR_USART(USART_ERROR_DATA_BITS);
    }

    uart_env.config.baudrate  = baud;
    uart_env.config.parity = parity;
    uart_env.config.mode = mode;
    uart_env.config.stop =  stopbits;
    uart_env.config.databit = bits;
    /*err correction of timer*/
    uart_env.bit_delay = (1000000 / baud) - 1;

    set_gpio_uart_timer();
    gpio_uart_config = 1;
    return 0;
}

int drv_gpio_usart_init(int32_t gpio_pin)
{
    int ret = 0 ;

    ret = drv_pinmux_config(gpio_pin, PIN_FUNC_GPIO);

    if (ret) {
        return ret;
    }

    uart_env.tx_gpio_handle = csi_gpio_pin_initialize(gpio_pin, NULL);

    if (!uart_env.tx_gpio_handle) {
        return -1;
    }

    ret = csi_gpio_pin_config_mode(uart_env.tx_gpio_handle, GPIO_MODE_PULLUP);

    if (ret) {
        return ret;
    }

    tx_gpio_write(gpio_pin, 1);

    uart_env.tx_pin = gpio_pin;
    gpio_uart_inited = 1;
    return 0;
}
