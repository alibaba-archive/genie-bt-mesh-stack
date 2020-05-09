

#include "drivers/8258/gpio_8258.h"
#include "vendor/common/alios_app_config.h"

#ifdef CONIFG_LIGHT_HONGYAN
#define WARM_PIN            TC825X_GET_PIN_NUM(GPIO_PB0)
#define COLD_PIN            TC825X_GET_PIN_NUM(GPIO_PB1)
#else
#define WARM_PIN            TC825X_GET_PIN_NUM(PWM_R)
#define COLD_PIN            TC825X_GET_PIN_NUM(PWM_G)
#endif
#define  LIGHT_FREQ       32000

static pwm_dev_t light_led_c;
static pwm_dev_t light_led_w;

uint16_t duty_list[] = {
    #include "duty_list.h"
};

static void _led_init(void)
{
    light_led_c.port = COLD_PIN;
    light_led_c.config.duty_cycle = 0;
    light_led_c.config.freq = LIGHT_FREQ;
    hal_pwm_init(&light_led_c);
    hal_pwm_start(&light_led_c);

    light_led_w.port = WARM_PIN;
    light_led_w.config.duty_cycle = 0;
    light_led_w.config.freq = LIGHT_FREQ;
    hal_pwm_init(&light_led_w);
    hal_pwm_start(&light_led_w);
}

//temperature 800~20000
//ligntness 655~65535
//return duty 0-100
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

    //0-100
    p_duty[LED_COLD_CHANNEL] = (actual * cold) / 65500;
    p_duty[LED_WARM_CHANNEL] = (actual * warm) / 65500;
    if(p_duty[LED_COLD_CHANNEL] == 0 && p_duty[LED_WARM_CHANNEL] == 0) {
        if(temperature > (LIGHT_CTL_TEMP_MAX - LIGHT_CTL_TEMP_MIN)>>1) {
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

    if(duty > 100) {
        LIGHT_DBG("invaild");
        return -1;
    }

    pwm_cfg.freq = LIGHT_FREQ;
    pwm_cfg.duty_cycle = (float)duty_list[duty]/duty_list[100];

    if (channel == LED_COLD_CHANNEL) {
        err = hal_pwm_para_chg(&light_led_c, pwm_cfg);
        if (err) {
            LIGHT_DBG("cold err %d", err);
            return -1;
        }

    } else if (channel == LED_WARM_CHANNEL) {
        err = hal_pwm_para_chg(&light_led_w, pwm_cfg);
        if (err) {
            LIGHT_DBG("warm err %d", err);
            return -1;
        }
    }
    return 0;
}

static void _led_set(uint8_t onoff, uint16_t actual, uint16_t temperature)
{
    static uint8_t last_duty[LED_CHANNEL_MAX] = {0xFF, 0xFF};  //0~100
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

