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

/******************************************************************************
 * @file     devices.c
 * @brief    source file for the devices
 * @version  V1.0
 * @date     15. May 2019
 ******************************************************************************/

#include <stdio.h>
#include <csi_config.h>
#include <soc.h>
#include <drv_usart.h>
#include <drv_rtc.h>
#include <pin_name.h>

extern void CSI_UART0_IRQHandler(void);
extern void CSI_GPIO_IRQHandler(void);
extern void CSI_AP_TIMER_IRQHandler(void);
extern void CSI_SPI0_IRQHandler(void);
extern void CSI_SPI1_IRQHandler(void);
extern void CSI_RTC_IRQHandler(void);
extern void CSI_WDT_IRQHandler(void);
extern void CSI_IIC0_IRQHandler(void);
extern void CSI_IIC1_IRQHandler(void);

typedef struct {
    int count;
    struct {
        uint32_t base;
        uint32_t irq;
        void *handler;
    } config[];
} device_irq_table_t;

int32_t target_get(const device_irq_table_t *table, int32_t idx, uint32_t *base, uint32_t *irq, void **handler)
{
    if (idx < table->count) {
        if (base) {
            *base = table->config[idx].base;
        }

        if (irq) {
            *irq = table->config[idx].irq;
        }

        if (handler) {
            *handler = table->config[idx].handler;
        }

        return idx;
    }

    return -1;
}

static const device_irq_table_t sg_usart_config = {
    .count = CONFIG_USART_NUM,
    .config = {
        {AP_UART0_BASE, UART_IRQ, CSI_UART0_IRQHandler},
    }
};

int32_t target_usart_init(int32_t idx, uint32_t *base, uint32_t *irq, void **handler)
{
    return target_get(&sg_usart_config, idx, base, irq, handler);
}

struct {
    uint32_t base;
    uint32_t irq;
    void *handler;
    uint32_t pin_num;
    port_name_e port;
}
const sg_gpio_config[CONFIG_GPIO_NUM] = {
    {AP_GPIOA_BASE, GPIO_IRQ, CSI_GPIO_IRQHandler, 18, PORTA},
    {0x4000800c, GPIO_IRQ, CSI_GPIO_IRQHandler, 16, PORTB},
};

typedef struct {
    uint8_t gpio_pin;
    uint8_t cfg_idx;
} gpio_pin_map_t;
const static gpio_pin_map_t s_gpio_pin_map[] = {
    {P0, 0},
    {P1, 0},
    {P2, 0},
    {P3, 0},
    {P4, 0},
    {P5, 0},
    {P6, 0},
    {P7, 0},
    {P8, 0},
    {P9, 0},
    {P10, 0},
    {P11, 0},
    {P12, 0},
    {P13, 0},
    {P14, 0},
    {P15, 0},
    {P16, 0},
    {P17, 0},
    {P18, 1},
    {P19, 1},
    {P20, 1},
    {P21, 1},
    {P22, 1},
    {P23, 1},
    {P24, 1},
    {P25, 1},
    {P26, 1},
    {P27, 1},
    {P28, 1},
    {P29, 1},
    {P30, 1},
    {P31, 1},
    {P32, 1},
    {P33, 1},
    {P34, 1},
};

int32_t target_gpio_port_init(port_name_e port, uint32_t *base, uint32_t *irq, void **handler, uint32_t *pin_num)
{
    int i;

    for (i = 0; i < CONFIG_GPIO_NUM; i++) {
        if (sg_gpio_config[i].port == port) {
            if (base != NULL) {
                *base = sg_gpio_config[i].base;
            }

            if (irq != NULL) {
                *irq = sg_gpio_config[i].irq;
            }

            if (pin_num !=  NULL) {
                *pin_num = sg_gpio_config[i].pin_num;
            }

            if (handler !=  NULL) {
                *handler = sg_gpio_config[i].handler;
            }

            return i;
        }
    }

    return -1;
}

int32_t target_gpio_pin_init(int32_t gpio_pin, uint32_t *port_idx)
{
    uint32_t idx;

    for (idx = 0; idx < sizeof(s_gpio_pin_map) / sizeof(gpio_pin_map_t); idx++) {
        if (s_gpio_pin_map[idx].gpio_pin == gpio_pin) {
            if (port_idx != NULL) {
                *port_idx = s_gpio_pin_map[idx].cfg_idx;
                break;
            }
        }
    }

    if (idx >= P18) {
        return idx - P18;
    } else {
        return idx;
    }

    return -1;
}

static const device_irq_table_t sg_timer_config = {
    .count = CONFIG_TIMER_NUM,
    .config = {
        {AP_TIM1_BASE, TIMER_IRQ, CSI_AP_TIMER_IRQHandler},
        {AP_TIM2_BASE, TIMER_IRQ, CSI_AP_TIMER_IRQHandler},
        {AP_TIM3_BASE, TIMER_IRQ, CSI_AP_TIMER_IRQHandler},
        {AP_TIM4_BASE, TIMER_IRQ, CSI_AP_TIMER_IRQHandler},
    },
};

int32_t target_get_timer(int32_t idx, uint32_t *base, uint32_t *irq, void **handler)
{
    return target_get(&sg_timer_config, idx, base, irq, handler);
}

static const device_irq_table_t sg_spi_config = {
    .count = CONFIG_SPI_NUM,
    .config = {
        {AP_SPI0_BASE, SPI0_IRQ, CSI_SPI0_IRQHandler},
        {AP_SPI1_BASE, SPI1_IRQ, CSI_SPI1_IRQHandler}
    }
};
static uint8_t spi_ssel[CONFIG_SPI_NUM] = {P17, P21};

int32_t target_spi_init(int32_t idx, uint32_t *base, uint32_t *irq, void **handler, uint32_t *ssel)
{
    int ret = target_get(&sg_spi_config, idx, base, irq, handler);

    if (ret != 0) {
        if (ssel != NULL) {
            *ssel = spi_ssel[idx];
        }
    }

    return ret;
}
struct {
    uint32_t base;
    uint32_t irq;
    void *handler;
}

const sg_rtc_config[CONFIG_RTC_NUM] = {
    {AP_RTC_BASE, RTC_IRQ, CSI_RTC_IRQHandler},
};

int32_t target_get_rtc_count(void)
{
    return CONFIG_RTC_NUM;
}

int32_t target_get_rtc(int32_t idx, uint32_t *base, uint32_t *irq, void **handler)
{
    if (idx >= target_get_rtc_count()) {
        return -1;
    }

    if (base != NULL) {
        *base = sg_rtc_config[idx].base;
    }

    if (irq != NULL) {
        *irq = sg_rtc_config[idx].irq;
    }

    if (handler != NULL) {
        *handler = (void *)sg_rtc_config[idx].handler;
    }

    return idx;
}

static const device_irq_table_t sg_wdt_config = {
    .count = CONFIG_WDT_NUM,
    .config = {
        {AP_WDT_BASE, WDT_IRQ, CSI_WDT_IRQHandler}
    }
};

int32_t target_get_wdt(int32_t idx, uint32_t *base, uint32_t *irq, void **handler)
{
    return target_get(&sg_wdt_config, idx, base, irq, handler);
}

static const device_irq_table_t sg_iic_config = {
    .count = CONFIG_IIC_NUM,
    .config = {
        {AP_I2C0_BASE, I2C0_IRQ, CSI_IIC0_IRQHandler},
        {AP_I2C1_BASE, I2C1_IRQ, CSI_IIC1_IRQHandler}
    }
};

int32_t target_iic_init(int32_t idx, uint32_t *base, uint32_t *irq, void **handler)
{
    return target_get(&sg_iic_config, idx, base, irq, handler);
}

