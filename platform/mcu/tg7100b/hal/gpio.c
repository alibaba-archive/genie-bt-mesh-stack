/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#include "soc.h"
#include "hal/soc/gpio.h"
#include "drv_gpio.h"
#include "pin_name.h"


typedef struct {
    uint8_t     portidx;
    uint8_t     idx;
    uint8_t     offset;
    gpio_event_cb_t cb;
} dw_gpio_pin_priv_t;

static gpio_pin_handle_t gpio_handlers[CONFIG_GPIO_PIN_NUM] = {NULL};

int32_t hal_gpio_init(gpio_dev_t *gpio)
{
    int ret = 0;
    gpio_pin_handle_t gpio_handle = NULL;

    switch (gpio->config) {
        case ANALOG_MODE:
        case PWM_CHANNEL_1:
        case PWM_CHANNEL_2:
            break;

        default:
            drv_pinmux_config(gpio->port, PIN_FUNC_GPIO);
            break;
    }

    gpio_handle = csi_gpio_pin_initialize(gpio->port, NULL);

    if (NULL == gpio_handle) {
        return -1;
    }

    gpio_handlers[gpio->port] = gpio_handle;

    switch (gpio->config) {
        case INPUT_PULL_UP:
            ret = csi_gpio_pin_config(gpio_handle, GPIO_MODE_PULLUP, GPIO_DIRECTION_INPUT);
            break;

        case INPUT_PULL_DOWN:
            ret = csi_gpio_pin_config(gpio_handle, GPIO_MODE_PULLDOWN, GPIO_DIRECTION_INPUT);
            break;

        case INPUT_HIGH_IMPEDANCE:
            ret = csi_gpio_pin_config(gpio_handle, GPIO_MODE_PULLNONE, GPIO_DIRECTION_INPUT);
            break;

        case OUTPUT_PUSH_PULL:
            ret = csi_gpio_pin_config(gpio_handle, GPIO_MODE_PULLNONE, GPIO_DIRECTION_OUTPUT);
            break;

        case ANALOG_MODE:
        case PWM_CHANNEL_1:
        case PWM_CHANNEL_2:
        case IRQ_MODE:
            break;

        case OUTPUT_OPEN_DRAIN_PULL_UP:
        case OUTPUT_OPEN_DRAIN_NO_PULL:
        default:
            return -1; 
    }

    return ret;
}


int32_t hal_gpio_finalize(gpio_dev_t *gpio)
{
    if (!gpio_handlers[gpio->port]) {
        return -1;
    }
    return  csi_gpio_pin_uninitialize(gpio_handlers[gpio->port]);
}

int32_t hal_gpio_output_high(gpio_dev_t *gpio)
{
    if (!gpio_handlers[gpio->port]) {
        return -1;
    }

    return csi_gpio_pin_write(gpio_handlers[gpio->port], 1);
}

int32_t hal_gpio_output_low(gpio_dev_t *gpio)
{

    if (!gpio_handlers[gpio->port]) {
        return -1;
    }

    return csi_gpio_pin_write(gpio_handlers[gpio->port], 0);

}

int32_t hal_gpio_output_toggle(gpio_dev_t *gpio)
{

    return -1;
}

int32_t hal_gpio_input_get(gpio_dev_t *gpio, uint32_t *value)
{
    if (!gpio_handlers[gpio->port]) {
        return -1;
    }
    csi_gpio_pin_read(gpio_handlers[gpio->port], value);
    return 0;
}


int32_t hal_gpio_enable_irq(gpio_dev_t *gpio, gpio_irq_trigger_t trigger,
                            gpio_irq_handler_t handler, void *arg)
{
    if (!gpio_handlers[gpio->port]) {
        return -1;
    }

    ((dw_gpio_pin_priv_t *)gpio_handlers[gpio->port])->cb = handler;

    switch (trigger) {
        case IRQ_TRIGGER_RISING_EDGE:
            csi_gpio_pin_config_mode(gpio_handlers[gpio->port], GPIO_MODE_PULLDOWN);
            csi_gpio_pin_config_direction(gpio_handlers[gpio->port], GPIO_DIRECTION_INPUT);
            csi_gpio_pin_set_irq(gpio_handlers[gpio->port], GPIO_IRQ_MODE_RISING_EDGE, true);
            break;

        case IRQ_TRIGGER_FALLING_EDGE:
            csi_gpio_pin_config_mode(gpio_handlers[gpio->port], GPIO_MODE_PULLUP);
            csi_gpio_pin_config_direction(gpio_handlers[gpio->port], GPIO_DIRECTION_INPUT);
            csi_gpio_pin_set_irq(gpio_handlers[gpio->port], GPIO_IRQ_MODE_FALLING_EDGE, true);
            break;

        default:
            return -1;
    }

    return 0;
}

int32_t hal_gpio_disable_irq(gpio_dev_t *gpio)
{
    if (!gpio_handlers[gpio->port]) {
        return -1;
    }
    csi_gpio_pin_set_irq(gpio_handlers[gpio->port], GPIO_IRQ_MODE_DOUBLE_EDGE,false);
    return 0;
}

int32_t hal_gpio_clear_irq(gpio_dev_t *gpio)
{
    return -1;
}
