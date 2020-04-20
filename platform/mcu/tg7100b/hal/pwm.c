/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "soc.h"
#include "hal/soc/pwm.h"
#include "drv_pwm.h"
#include "gpio.h"
#include "pin_name.h"
#include "pinmux.h"

static pwm_handle_t pwm_handlers = NULL;
#define PWM_GPIO_BASE 14
#define PWM_CH_FUNC_BASE 10
typedef struct {
    uint8_t port;
    uint8_t func;
} pwm_port_func_t;

#define PWM_CHANNEL_COUNT  6
pwm_port_func_t pwm_port_channel[PWM_CHANNEL_COUNT] = {
    {0xFF, PWM0},
    {0xFF, PWM1},
    {0xFF, PWM2},
    {0xFF, PWM3},
    {0xFF, PWM4},
    {0xFF, PWM5},
};
static uint8_t current_pwm_port_channel_idx = 0;

static int8_t active_pwm_func(uint8_t port, uint8_t *func)
{
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (pwm_port_channel[i].port == 0xFF) {
            pwm_port_channel[i].port = port;
            *func = pwm_port_channel[i].func;
            return 0;
        }
    }
    return -1;
}

static int8_t deactive_pwm_func(uint8_t port)
{
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (pwm_port_channel[i].port == port) {
            pwm_port_channel[i].port = 0xFF;
            return 0;
        }
    }
    return -1;
}

static int8_t search_pwm_func(uint8_t port, uint8_t *func)
{
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (pwm_port_channel[i].port == port) {
            *func = pwm_port_channel[i].func;
            return 0;
        }
    }
    return -1;
}

int32_t hal_pwm_init(pwm_dev_t *pwm)
{
    int ret = 0;
    uint8_t func;

    if (pwm == NULL) {
        return -1;
    }
    pwm_handlers = csi_pwm_initialize(0);

    if (!pwm_handlers) {
        printk("init faild\n");
        return -1;
    }


    ret = search_pwm_func(pwm->port, &func);
    if (ret) {
        ret = active_pwm_func(pwm->port, &func);
        if (ret) {
            return -1;
        }
    }
    drv_pinmux_config(pwm->port, func);
    return 0;
}

/**
 * Starts Pulse-Width Modulation signal output on a PWM pin
 *
 * @param[in]  pwm  the PWM device
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_pwm_start(pwm_dev_t *pwm)
{
    int ret;
    uint8_t func;
    uint8_t pwm_chn;

    if (!pwm_handlers) {
        return -1;
    }

    ret = search_pwm_func(pwm->port, &func);
    if (ret) {
        return -1;
    }
    pwm_chn = func - PWM0;

    csi_pwm_start(pwm_handlers, pwm_chn);
    return 0;
}

/**
 * Stops output on a PWM pin
 *
 * @param[in]  pwm  the PWM device
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_pwm_stop(pwm_dev_t *pwm)
{
    int ret;
    uint8_t func;
    uint8_t pwm_chn;

    if (!pwm_handlers) {
        return -1;
    }

    ret = search_pwm_func(pwm->port, &func);
    if (ret) {
        return -1;
    }
    pwm_chn = func - PWM0;

    csi_pwm_stop(pwm_handlers, pwm_chn);
    return 0;
}

/**
 * change the para of pwm
 *
 * @param[in]  pwm  the PWM device
 * @param[in]  para the para of pwm
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_pwm_para_chg(pwm_dev_t *pwm, pwm_config_t para)
{
    uint32_t duty_cycle;
    int ret;
    uint8_t func;
    uint8_t pwm_chn;

    if (!pwm_handlers) {
        return -1;
    }

    ret = search_pwm_func(pwm->port, &func);
    if (ret) {
        return -1;
    }
    pwm_chn = func - PWM0;

    duty_cycle = para.duty_cycle;

    //printf("port %d, chan %d period_us %d, duty %d\n", pwm->port, pwm_chn, para.freq, para.duty_cycle);

    if (para.duty_cycle == 0) {
        csi_pwm_stop(pwm_handlers, pwm_chn);
        drv_pinmux_config(pwm->port, PIN_FUNC_GPIO);
        phy_gpio_pull_set(pwm->port, STRONG_PULL_UP);

    } else {
        drv_pinmux_config(pwm->port, func);
        ret = csi_pwm_config(pwm_handlers, pwm_chn, para.freq, para.duty_cycle);

        if (ret) {
            return -1;
        }

        csi_pwm_start(pwm_handlers, pwm_chn);
    }
    return 0;
}

/**
 * De-initialises an PWM interface, Turns off an PWM hardware interface
 *
 * @param[in]  pwm  the interface which should be de-initialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_pwm_finalize(pwm_dev_t *pwm)
{
    if (!pwm_handlers) {
        return -1;
    }
    hal_pwm_stop(pwm);
    csi_pwm_uninitialize(pwm_handlers);
    return 0;
}

