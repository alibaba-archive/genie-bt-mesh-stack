/* helloworld.c - helloworld */

/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#define BT_DBG_ENABLED 1
#include "common/log.h"


#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include <misc/printk.h>
#include <misc/byteorder.h>
#include <hal/soc/gpio.h>
#include <hal/soc/pwm.h>

#include <bluetooth.h>
#include <soc.h>
#include <api/mesh.h>
#include "genie_app.h"

#define DEFAULT_MESH_GROUP1 0xC000
#define DEFAULT_MESH_GROUP2 0xCFFF

#ifdef BOARD_TC825X
#include "drivers/8258/gpio_8258.h"
#include "vendor/common/alios_app_config.h"

#define LED_PIN            TC825X_GET_PIN_NUM(PWM_R)

#endif

#define MESH_ELEM_COUNT 1
#define MESH_ELEM_STATE_COUNT MESH_ELEM_COUNT

#define MESH_PBADV_TIME 600 //unit:s

static uint8_t led_ctrl(pwm_dev_t *led, bool on);

static uint8_t leds_init(void);


static pwm_dev_t light_led;

elem_state_t g_elem_state[MESH_ELEM_STATE_COUNT];

static struct bt_mesh_model element_models[] = {
    BT_MESH_MODEL_CFG_SRV(),
    BT_MESH_MODEL_HEALTH_SRV(),

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    MESH_MODEL_GEN_ONOFF_SRV(&g_elem_state[0]),
#endif
};

static struct bt_mesh_model g_element_vendor_models[] = {

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    MESH_MODEL_VENDOR_SRV(&g_elem_state[0]),
#endif
};

struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, element_models, g_element_vendor_models, 0),
};

uint32_t get_mesh_pbadv_time(void)
{
    return MESH_PBADV_TIME*1000;
}

uint8_t get_vendor_element_num(void)
{
    return MESH_ELEM_COUNT;
}

void mesh_sub_init(u16_t *p_sub)
{
    uint16_t sub_list[CONFIG_BT_MESH_MODEL_GROUP_COUNT];
    memset(sub_list, 0, sizeof(sub_list));

#ifdef DEFAULT_MESH_GROUP1
    sub_list[0] = DEFAULT_MESH_GROUP1;
#endif

#ifdef DEFAULT_MESH_GROUP2
    sub_list[1] = DEFAULT_MESH_GROUP2;
#endif

    memcpy(p_sub, sub_list, sizeof(sub_list));
}

void user_data_init(void)
{
    memset(g_elem_state, 0, sizeof(g_elem_state));
    elem_state_init(MESH_ELEM_STATE_COUNT, g_elem_state);
}

void reset_light_para(void)
{
    uint8_t i = 0;

    while(i < MESH_ELEM_STATE_COUNT) {
        g_elem_state[i].state.onoff[T_CUR] = GEN_ONOFF_DEFAULT;
        g_elem_state[i].state.onoff[T_TAR] = GEN_ONOFF_DEFAULT;
        i++;
    }

    BT_DBG("+ done");
}


void user_prov_complete(u16_t net_idx, u16_t addr)
{
    //flash 3 timers
}

void user_prov_reset(void)
{
    //flash 5 timers
}

void user_init()
{
    user_data_init();
    leds_init();
}

void led_flash(uint8_t times)
{
    printk("%d\n", times);
}

u16_t vendor_model_msg_handle(vnd_model_msg *p_msg)
{
    printk("vendor model message received\n");
    if (!p_msg)
        return -1;

    printk("opcode:0x%x, tid:%d, len:%d", p_msg->opid, p_msg->tid, p_msg->len);
    if (p_msg->data && p_msg->len)
        printk("payload: %s", bt_hex(p_msg->data, p_msg->len));

    switch (p_msg->opid) {
        case VENDOR_OP_ATTR_GET_STATUS:
            /* report VENDOR_OP_ATTR_STATUS */
            //_light_report_status();
            break;
        case VENDOR_OP_ATTR_SET_ACK:
            /* TODO: set status
             * report VENDOR_OP_ATTR_STATUS
             * */
            //_light_report_status();
            break;
        case VENDOR_OP_ATTR_SET_UNACK:
            /* TODO: set status */
            break;
        case VENDOR_OP_ATTR_CONFIME:
            /* clear indicate status */

            break;
        case VENDOR_OP_ATTR_TRANS_MSG:
            break;
        default:
            break;
    }

    return 0;
}

#if defined(CONFIG_MESH_MODEL_GEN_ONOFF_SRV)
static void _led_set(uint8_t elem_index, uint8_t on)
{
    static uint8_t last_onoff = 0;
    BT_DBG(">>>>elem %d set %d\n", elem_index, on);
    if(last_onoff != on) {
        last_onoff = on;
        led_ctrl(&light_led, last_onoff);
    }
}
#endif

#ifdef CONFIG_GENIE_OTA
bool ota_check_reboot(void)
{
    return true;
}
#endif

void user_event(E_GENIE_EVENT event, void *p_arg)
{
    E_GENIE_EVENT next_event = event;

    //BT_DBG("%s, %s %p\n", __func__, genie_event_str[event], p_arg);
    switch(event) {
        case GENIE_EVT_SW_RESET:
        case GENIE_EVT_HW_RESET_START:
            BT_DBG("FLASH x5");
            led_flash(5);
            reset_light_para();
            break;
        case GENIE_EVT_SDK_MESH_INIT:
            user_init();
            if (!genie_reset_get_flag()) {
                next_event = GENIE_EVT_SDK_ANALYZE_MSG;
            }
            break;
        case GENIE_EVT_SDK_MESH_PROV_SUCCESS:
            BT_DBG("FLASH x3");
            led_flash(3);
            break;
        case GENIE_EVT_SDK_ACTION_DONE:
        {
            elem_state_t *p_elem = (elem_state_t *)p_arg;
#if defined(CONFIG_MESH_MODEL_GEN_ONOFF_SRV)
            _led_set(p_elem->elem_index, p_elem->state.onoff[T_CUR]);
#endif
            break;
        }
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
        case GENIE_EVT_SDK_INDICATE:
            break;
        case GENIE_EVT_SDK_VENDOR_MSG:
        {
            vendor_model_msg_handle((vnd_model_msg *)p_arg);
            break;
        }
#endif
        case GENIE_EVT_HW_RESET_DONE:
            printk("GENIE_EVT_HW_RESET_DONE\n");
            break;
        default:
            break;
    }
    
    if(next_event != event) {
        genie_event(next_event, p_arg);
    }
}

static uint8_t led_ctrl(pwm_dev_t *led, bool on)
{
    if (!led)
        return -1;


    printk("%s:%d\n", __func__, on);
    
#ifdef BOARD_TC825X
        pwm_config_t pwm_cfg_c;
        if (on)
            pwm_cfg_c.duty_cycle = 65535;
        else
            pwm_cfg_c.duty_cycle = 0;
        pwm_cfg_c.freq = 300000;
        hal_pwm_para_chg(&light_led,pwm_cfg_c);
#endif

    return 0;
}

/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static u8_t leds_init(void)
{
#ifdef BOARD_TC825X
    light_led.port = LED_PIN;
    light_led.config.duty_cycle = 0;
    light_led.config.freq = 300000;
    hal_pwm_init(&light_led);
    hal_pwm_start(&light_led);
#endif
    printk("%s:%d\n", __func__, __LINE__);
    return 0;
}

int application_start(int argc, char **argv)
{
    // init genie (app & mesh)
    genie_init();

    printk("BUILD_TIME:%s\n", __DATE__","__TIME__);
    //aos init
#ifndef BOARD_TC825X    // telink confirm later: don't use this, because we will run while(1) of sys_init later.
    aos_loop_run();
#endif

    return 0;
}

