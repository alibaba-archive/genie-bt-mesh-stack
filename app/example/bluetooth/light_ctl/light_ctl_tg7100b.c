/*
 * Copyright (C) 2015-2020 Alibaba Group Holding Limited
 */

#include <hal/soc/pwm.h>
#include "pin_name.h"

#define  WARM_PIN   24  //warm led
#define  COLD_PIN   20  //cold led
#define  LIGHT_DRIVE_TYPE OUTPUT_PUSH_PULL
#define  LIGHT_PERIOD     2000

static pwm_dev_t light_led_c;
static pwm_dev_t light_led_w;

uint16_t duty_list[] = {
    #include "duty_list_fsl.h"
};

/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void _led_init(void)
{
    light_led_c.port = COLD_PIN;
    light_led_c.config.duty_cycle = 0;
    light_led_c.config.freq = LIGHT_PERIOD;
    hal_pwm_init(&light_led_c);
    hal_pwm_start(&light_led_c);

    light_led_w.port = WARM_PIN;
    light_led_w.config.duty_cycle = 0;
    light_led_w.config.freq = LIGHT_PERIOD;
    hal_pwm_init(&light_led_w);
    hal_pwm_start(&light_led_w);
}

//temperature 800~20000
//ligntness 1~65535
//return duty 1-100
static void _get_led_duty(uint8_t *p_duty, uint16_t actual, uint16_t temperature)
{
    uint8_t cold = 0;
    uint8_t warm = 0;

    if(temperature > LIGHT_CTL_TEMP_MAX) {
        temperature = LIGHT_CTL_TEMP_MAX;
    }
    if(temperature < LIGHT_CTL_TEMP_MIN) {
        temperature = LIGHT_CTL_TEMP_MIN;
    }

    //0-100
    cold = (temperature - LIGHT_CTL_TEMP_MIN) * 100 / (LIGHT_CTL_TEMP_MAX - LIGHT_CTL_TEMP_MIN);
    warm = 100 - cold;

    p_duty[LED_COLD_CHANNEL] = (actual * cold) / 65500;
    p_duty[LED_WARM_CHANNEL] = (actual * warm) / 65500;
    if (p_duty[LED_COLD_CHANNEL] == 0 && p_duty[LED_WARM_CHANNEL] == 0) {
        if (temperature > (LIGHT_CTL_TEMP_MAX - LIGHT_CTL_TEMP_MIN)>>1) {
            p_duty[LED_COLD_CHANNEL] = 1;
        } else {
            p_duty[LED_WARM_CHANNEL] = 1;
        }
    }

    //LIGHT_DBG("%d %d [%d %d] [%d %d]", actual, temperature, warm, cold, p_duty[LED_COLD_CHANNEL], p_duty[LED_WARM_CHANNEL]);

}

static int _set_pwm_duty(uint8_t channel, uint8_t duty)
{
    int err = -1;
    pwm_config_t pwm_cfg;
    pwm_dev_t *pwm_dev = NULL;

    if(duty > 100) {
        LIGHT_DBG(">>duty invaild\r\n");
        return -1;
    }

    pwm_cfg.freq = LIGHT_PERIOD;
    pwm_cfg.duty_cycle = duty_list[duty];

    if (channel == LED_COLD_CHANNEL) {
        pwm_dev = &light_led_c;
    } else if (channel == LED_WARM_CHANNEL) {
        pwm_dev = &light_led_w;
    } else {
        return -1;
    }

    err = hal_pwm_para_chg(pwm_dev, pwm_cfg);
    if (err) {
        LIGHT_DBG("pwm err %d\n", err);
        return -1;
        }

    return 0;
}

//lightness 1-65535
static void _led_set(uint8_t onoff, uint16_t actual, uint16_t temperature)
{
    static uint8_t last_duty[LED_CHANNEL_MAX] = {0xFF, 0xFF};
    uint8_t duty[LED_CHANNEL_MAX];  //0~100

    if(onoff == 0) {
        duty[LED_COLD_CHANNEL] = 0;
        duty[LED_WARM_CHANNEL] = 0;
    } else {
        _get_led_duty(duty, actual, temperature);
    }

    if(last_duty[LED_COLD_CHANNEL] != duty[LED_COLD_CHANNEL]) {
        last_duty[LED_COLD_CHANNEL] = duty[LED_COLD_CHANNEL];
        _set_pwm_duty(LED_COLD_CHANNEL, duty[LED_COLD_CHANNEL]);
    }
    if(last_duty[LED_WARM_CHANNEL] != duty[LED_WARM_CHANNEL]) {
        last_duty[LED_WARM_CHANNEL] = duty[LED_WARM_CHANNEL];
        _set_pwm_duty(LED_WARM_CHANNEL, duty[LED_WARM_CHANNEL]);
    }
}

