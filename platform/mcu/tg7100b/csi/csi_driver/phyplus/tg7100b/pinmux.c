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
 * @file     pinmux.c
 * @brief    source file for the pinmux
 * @version  V1.0
 * @date     18. May 2019
 ******************************************************************************/

#include <csi_config.h>
#include <stdint.h>
#include <stddef.h>
#include <drv_gpio.h>
#include "pinmux.h"
#include "pin_name.h"

#define readl(addr) \
    ({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#define writel(b,addr) (void)((*(volatile unsigned int *) (addr)) = (b))


static uint32_t pinmux_regs_saved[21];

/**
  \brief       initial hobbit pinmux.
  \return      none
*/
void ioreuse_initial(void)
{
}

/**
  \brief       config the pin function.
  \param[in]   pin       refs to pin_name_e.
  \param[in]   pin_func  refs to pin_func_e.
  \return      0-success or -1-failure
*/
int32_t drv_pinmux_config(pin_name_e pin, pin_func_e pin_func)
{
    if (pin_func == PIN_FUNC_GPIO) {
        phy_gpio_fmux(pin, Bit_DISABLE);

        if ((pin >= 4 && pin <= 15) || (pin >= 18 && pin <= 30)) {
            /* do noting */
        } else if ((pin >= 16) && (pin <= 17)) {
            phy_gpio_cfg_analog_io(pin, 0);
        } else {
            phy_gpio_pin0to3_pin31to34_control(pin, 1);
        }
    } else {
        if ((pin >= 4 && pin <= 15) || (pin >= 18 && pin <= 30)) {
        } else if ((pin >= 16) && (pin <= 17)) {
            phy_gpio_cfg_analog_io(pin, 1);
        } else {
            phy_gpio_pin0to3_pin31to34_control(pin, 0);
        }
        phy_gpio_fmux_set(pin, pin_func);
    }

    return 0;
}

/**
  \brief       config the pin mode.
  \param[in]   port      refs to port_name_e.
  \param[in]   offset    the offset of the pin in corresponding port.
  \param[in]   pin_mode  refs to gpio_mode_e.
  \return      0-success or -1-failure
*/
int32_t drv_pin_config_mode(port_name_e port, uint8_t offset, gpio_mode_e pin_mode)
{
    GPIO_Pin_e pin = offset;
    IO_Pull_Type_e type;

    switch(pin_mode) {
        case GPIO_MODE_PULLNONE:
            type = FLOATING;
            break;
        case GPIO_MODE_PULLUP:
            type = STRONG_PULL_UP;
            break;
        case GPIO_MODE_PULLDOWN:
            type = PULL_DOWN;
            break;
        case GPIO_MODE_OPEN_DRAIN:
        case GPIO_MODE_PUSH_PULL:
        default:
            return -1;
    }

    phy_gpio_pull_set(pin, type);
    return 0;
}

/**
  \brief       get the pin function.
  \param[in]   pin       refs to pin_name_e.
  \return      pin function count
*/
pin_func_e drv_pinmux_get_config(pin_name_e pin)
{
    return 0;
}

struct uart_tx_gpio_map {
    int32_t idx;
    pin_name_e  uart_tx;
    pin_func_e  uart_tx_fun;
};

struct uart_rx_gpio_map {
    int32_t idx;
    pin_name_e  uart_rx;
    pin_func_e  uart_rx_fun;
};

void drv_pinmux_dwuart_save(int32_t idx, pin_name_e *tx_pin, pin_name_e *rx_pin)
{
}

void drv_pinmux_dwuart_restore(pin_name_e tx_pin, pin_name_e rx_pin)
{
}

void drv_pinmux_reset(void)
{
}

void csi_pinmux_prepare_sleep_action()
{
    uint32_t addr = 0x40003800;
    registers_save(pinmux_regs_saved, (uint32_t *)addr, 21);
}

void csi_pinmux_wakeup_sleep_action(void)
{
    uint32_t addr = 0x40003800;
    registers_save((uint32_t *)addr, pinmux_regs_saved, 21);
}

