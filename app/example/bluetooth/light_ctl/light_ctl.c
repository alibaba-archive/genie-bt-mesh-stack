/* main.c - light demo */

/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

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

#ifndef CONFIG_INFO_DISABLE
#define LIGHT_DBG(fmt, ...)  printf("[%s]"fmt"\n", __func__, ##__VA_ARGS__)
#else
#define LIGHT_DBG(fmt, ...)
#endif

#define LIGHT_CTL_TEMP_MIN            (0x0320)    // 800
#define LIGHT_CTL_TEMP_MAX            (0x4E20)    // 20000

#define LED_FLASH_CYCLE MESH_TRNSATION_CYCLE

#define LED_FLASH_PERIOD 1000
#define LED_FLASH_ON_TIME 600
#define LED_FLASH_OFF_TIME 400

typedef struct{
    struct k_timer timer;
    uint16_t temp_cur;
    uint16_t temp_tar;
    uint16_t actual_start;
    uint16_t actual_cur;
    uint16_t actual_tar;
    uint32_t time_end;
} led_flash_t;

led_flash_t g_flash_para;

/* unprovision device beacon adv time */
#define MESH_PBADV_TIME 600 //unit:s

#define DEFAULT_MESH_GROUP1 0xC000
#define DEFAULT_MESH_GROUP2 0xCFFF

enum {
    LED_COLD_CHANNEL = 0,
    LED_WARM_CHANNEL,
    LED_CHANNEL_MAX
};

uint32_t get_mesh_pbadv_time(void)
{
    return MESH_PBADV_TIME*1000;    //ms
}

/* element configuration start */
#define MESH_ELEM_COUNT 1
#define MESH_ELEM_STATE_COUNT MESH_ELEM_COUNT

elem_state_t g_elem_state[MESH_ELEM_STATE_COUNT];
model_powerup_t g_powerup[MESH_ELEM_STATE_COUNT];

struct k_delayed_work light_state_store_work;
#define LIGHT_STATE_STORE_DELAY_TIME (5000)

static struct bt_mesh_model element_models[] = {
    BT_MESH_MODEL_CFG_SRV(),
    BT_MESH_MODEL_HEALTH_SRV(),

    MESH_MODEL_GEN_ONOFF_SRV(&g_elem_state[0]),
    MESH_MODEL_LIGHTNESS_SRV(&g_elem_state[0]),
    MESH_MODEL_CTL_SRV(&g_elem_state[0]),
#ifndef CONFIG_ALI_SIMPLE_MODLE
    MESH_MODEL_GEN_LEVEL_SRV(&g_elem_state[0]),
    MESH_MODEL_CTL_SETUP_SRV(&g_elem_state[0]),
#endif

};

static struct bt_mesh_model g_element_vendor_models[] = {
    MESH_MODEL_VENDOR_SRV(&g_elem_state[0]),
};

struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, element_models, g_element_vendor_models, 0),
};

static void light_state_store(struct k_work *work)
{
    uint8_t *p_read = aos_malloc(sizeof(g_powerup));
    genie_flash_read_userdata(GFI_MESH_POWERUP, p_read, sizeof(g_powerup));
    model_powerup_t *pu = (model_powerup_t *)p_read;

    if(memcmp(g_powerup, p_read, sizeof(g_powerup))) {
        LIGHT_DBG("save %d %d %d", g_powerup[0].last_onoff, g_powerup[0].last_actual, g_powerup[0].last_temp);
        genie_flash_write_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));
    }

    aos_free(p_read);

#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    if (g_powerup[0].last_onoff == 0 && ais_get_ota_ready() == 1) {
        //Means have ota, wait for reboot while light off
        aos_reboot();
    } else {
        g_powerup[0].last_onoff = 1;
    }
#endif
}

uint8_t get_vendor_element_num(void)
{
    return MESH_ELEM_COUNT;
}
/* element configuration end */

void mesh_sub_init(u16_t *p_sub)
{
    memset(p_sub, 0, CONFIG_BT_MESH_MODEL_GROUP_COUNT<<1);

    p_sub[0] = DEFAULT_MESH_GROUP1;
    p_sub[1] = DEFAULT_MESH_GROUP2;
}

/* functions in light_ctl_board.c */
static void _led_init(void);
static void _led_set(uint8_t onoff, uint16_t actual, uint16_t temperature);

#ifdef CONFIG_GENIE_OTA
bool ota_check_reboot(void)
{
    // the device will reboot when it is off
    if (g_elem_state[0].state.onoff[T_CUR] == 0) {
        // save light para, always off
        g_powerup[0].last_onoff = 0;
        genie_flash_write_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));
        LIGHT_DBG("Allow to reboot!");
        return true;
    }
    LIGHT_DBG("no reboot!");
    return false;
}
#endif

static void _init_light_para(void)
{
    uint8_t i = 0;
    E_GENIE_FLASH_ERRCODE ret;

    // init element state
    memset(g_elem_state, 0, sizeof(g_elem_state));
    elem_state_init(MESH_ELEM_STATE_COUNT, g_elem_state);

    // load light para
    ret = genie_flash_read_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));
    
    if(ret == GENIE_FLASH_SUCCESS) {
        while(i < MESH_ELEM_STATE_COUNT) {
#ifdef CONFIG_GENIE_OTA
            // if the device reboot by ota, it must be off.
            if(g_powerup[0].last_onoff == 0) {
                g_elem_state[0].powerup.last_onoff = g_powerup[0].last_onoff;
                g_elem_state[0].state.onoff[T_TAR] = 0;
                // load lightness
                if(g_powerup[0].last_actual) {
                    g_elem_state[0].state.actual[T_TAR] = g_powerup[0].last_actual;
                    g_elem_state[0].powerup.last_actual = g_powerup[0].last_actual;
                }
                // load temperature
                if(g_powerup[0].last_temp) {
                    g_elem_state[0].state.temp[T_TAR] = g_powerup[0].last_temp;
                    g_elem_state[0].powerup.last_temp = g_powerup[0].last_temp;
                }
                clear_trans_para(&g_elem_state[0]);
            } else
#endif
            {
                memcpy(&g_elem_state[0].powerup, &g_powerup[0], sizeof(model_powerup_t));
                // load lightness
                if(g_powerup[0].last_actual) {
                    g_elem_state[0].state.actual[T_TAR] = g_powerup[0].last_actual;
                }
                // load temperature
                if(g_powerup[0].last_temp) {
                    g_elem_state[0].state.temp[T_TAR] = g_powerup[0].last_temp;
                }
                //LIGHT_DBG("l:%d t:%d", g_powerup[0].last_actual, g_powerup[0].last_temp);

                // cal transition
                if(g_elem_state[0].state.onoff[T_TAR] == 1) {
                    g_elem_state[0].state.trans_start_time = k_uptime_get() + g_elem_state[0].state.delay * 5;
                    g_elem_state[0].state.trans_end_time = g_elem_state[0].state.trans_start_time + get_transition_time(g_elem_state[0].state.trans);
                }
            }
            g_elem_state[0].state.temp[T_CUR] = g_elem_state[0].state.temp[T_TAR];

            i++;
        }
    }
    //LIGHT_DBG("done");
}

static void _reset_light_para(void)
{
    uint8_t i = 0;

    while(i < MESH_ELEM_STATE_COUNT) {
        g_elem_state[i].state.onoff[T_CUR] = GEN_ONOFF_DEFAULT;
        g_elem_state[i].state.actual[T_CUR] = LIGHTNESS_DEFAULT;
        g_elem_state[i].state.temp[T_CUR] = CTL_TEMP_DEFAULT;
        g_elem_state[i].state.onoff[T_TAR] = GEN_ONOFF_DEFAULT;
        g_elem_state[i].state.actual[T_TAR] = LIGHTNESS_DEFAULT;
        g_elem_state[i].state.temp[T_TAR] = CTL_TEMP_DEFAULT;
        g_elem_state[i].state.trans = 0;
        g_elem_state[i].state.delay = 0;
        g_elem_state[i].state.trans_start_time = 0;
        g_elem_state[i].state.trans_end_time = 0;

        g_elem_state[i].powerup.last_actual = LIGHTNESS_DEFAULT;
        g_elem_state[i].powerup.last_temp = CTL_TEMP_DEFAULT;

        g_powerup[i].last_onoff = GEN_ONOFF_DEFAULT;
        g_powerup[i].last_actual = LIGHTNESS_DEFAULT;
        g_powerup[i].last_temp = CTL_TEMP_DEFAULT;

        i++;
    }

    genie_flash_write_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));

    LIGHT_DBG("done");
}

static void _save_light_state(elem_state_t *p_elem)
{
    if(p_elem->state.actual[T_CUR] != 0) {
        p_elem->powerup.last_actual = p_elem->state.actual[T_CUR];
        g_powerup[p_elem->elem_index].last_actual = p_elem->state.actual[T_CUR];
    }

    p_elem->powerup.last_temp = p_elem->state.temp[T_CUR];
    g_powerup[p_elem->elem_index].last_temp = p_elem->state.temp[T_CUR];
    // always on
    p_elem->powerup.last_onoff = p_elem->state.onoff[T_CUR];
    g_powerup[p_elem->elem_index].last_onoff = p_elem->state.onoff[T_CUR];

    k_delayed_work_submit(&light_state_store_work, LIGHT_STATE_STORE_DELAY_TIME);
    LIGHT_DBG("light state %d %d %d", g_powerup[p_elem->elem_index].last_onoff, g_powerup[p_elem->elem_index].last_actual, g_powerup[p_elem->elem_index].last_temp);

#if 0
    uint8_t *p_read = aos_malloc(sizeof(g_powerup));
    genie_flash_read_userdata(GFI_MESH_POWERUP, p_read, sizeof(g_powerup));

    if(memcmp(g_powerup, p_read, sizeof(g_powerup))) {
        LIGHT_DBG("save %d %d", g_powerup[p_elem->elem_index].last_actual, g_powerup[p_elem->elem_index].last_temp);
        genie_flash_write_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));
    }
    aos_free(p_read);
#endif
}

static void _user_init(void)
{
    k_delayed_work_init(&light_state_store_work, light_state_store);

#ifdef CONFIG_GENIE_OTA
    // check ota flag
    if(ais_get_ota_indicat()) {
        g_indication_flag |= INDICATION_FLAG_VERSION;
    }
#endif
}

static void _led_ctrl(elem_state_t *p_elem)
{
    static uint8_t last_onoff = 0;
    static uint16_t last_acual = 0;
    static uint16_t last_temperature = 0;
    
    uint8_t onoff = p_elem->state.onoff[T_CUR];
    uint16_t actual = p_elem->state.actual[T_CUR];
    uint16_t temperature = p_elem->state.temp[T_CUR];

    if(last_onoff != onoff || last_acual != actual || last_temperature != temperature) {
        last_onoff = onoff;
        last_acual = actual;
        last_temperature = temperature;
        //LIGHT_DBG("%d,%d,%d", onoff, actual, temperature);
        _led_set(onoff, actual, temperature);
    }
}

void _cal_flash_next_step(uint32_t delta_time)
{
    uint16_t actual_end;
    if(delta_time < 1000) {
        actual_end = g_flash_para.actual_tar;
        g_flash_para.temp_cur = g_flash_para.temp_tar;
    } else {
        actual_end = g_flash_para.actual_start;
        delta_time %= 1000;
    }
    if(delta_time > LED_FLASH_ON_TIME) {
        delta_time -= LED_FLASH_ON_TIME;
        g_flash_para.actual_cur = g_flash_para.actual_start * delta_time / LED_FLASH_OFF_TIME;
    } else {
        g_flash_para.actual_cur = actual_end * (LED_FLASH_ON_TIME - delta_time) / LED_FLASH_ON_TIME;
    }
    //LIGHT_DBG("delta %d, actual %04x", delta_time, g_flash_para.actual_cur);
}

static void _led_flash_timer_cb(void *p_timer, void *p_arg)
{
    uint32_t cur_time = k_uptime_get();
    if(cur_time >= g_flash_para.time_end) {
        _led_set(1, g_flash_para.actual_tar, g_flash_para.temp_tar);
    } else {
        _cal_flash_next_step(g_flash_para.time_end - cur_time);
        _led_set(1, g_flash_para.actual_cur, g_flash_para.temp_cur);
        k_timer_start(&g_flash_para.timer, LED_FLASH_CYCLE);
    }
}

static void _led_flash(uint8_t times, uint8_t reset)
{
    static uint8_t inited = 0;
    if(inited == 0) {
        k_timer_init(&g_flash_para.timer, _led_flash_timer_cb, NULL);
        inited = 1;
    }

    if(g_elem_state[0].state.onoff[T_CUR] == 1) {
        if(g_elem_state[0].state.actual[T_CUR]) {
            g_flash_para.actual_start = g_flash_para.actual_cur = g_elem_state[0].state.actual[T_CUR];
        } else {
            g_flash_para.actual_start = g_flash_para.actual_cur = LIGHTNESS_DEFAULT;
        }
        if(g_elem_state[0].state.temp[T_CUR]) {
            g_flash_para.temp_cur = g_elem_state[0].state.temp[T_CUR];
        } else {
            g_flash_para.temp_cur = CTL_TEMP_DEFAULT;
        }
        if(reset) {
            g_flash_para.actual_tar = LIGHTNESS_DEFAULT;
            g_flash_para.temp_tar = CTL_TEMP_DEFAULT;
        } else {
            g_flash_para.actual_tar = g_flash_para.actual_cur;
            g_flash_para.temp_tar = g_flash_para.temp_cur;
        }
        g_flash_para.time_end = k_uptime_get() + times*LED_FLASH_PERIOD;
    } else {
        if(g_elem_state[0].powerup.last_actual && !reset) {
            g_flash_para.actual_start = g_flash_para.actual_tar = g_elem_state[0].powerup.last_actual;
        } else {
            g_flash_para.actual_start = g_flash_para.actual_tar = LIGHTNESS_DEFAULT;
        }
        g_flash_para.actual_cur = 0;
        if(g_elem_state[0].powerup.last_temp) {
            g_flash_para.temp_cur = g_elem_state[0].powerup.last_temp;
        } else {
            g_flash_para.temp_cur = CTL_TEMP_DEFAULT;
        }
        if(reset) {
            g_flash_para.temp_tar = CTL_TEMP_DEFAULT;
        } else {
            g_flash_para.temp_tar = g_flash_para.temp_cur;
        }
        g_flash_para.time_end = k_uptime_get() + times*LED_FLASH_PERIOD - LED_FLASH_OFF_TIME;
    }
    //LIGHT_DBG("%d (%d-%d) tar %04x", times, k_uptime_get(), g_flash_para.time_end, g_flash_para.actual_tar);

    k_timer_start(&g_flash_para.timer, LED_FLASH_CYCLE);
}

void user_event(E_GENIE_EVENT event, void *p_arg)
{
    E_GENIE_EVENT next_event = event;

    //BT_DBG("%s, %s %p\n", __func__, genie_event_str[event], p_arg);
    switch(event) {
        case GENIE_EVT_SW_RESET:
        case GENIE_EVT_HW_RESET_START:
            _led_flash(5, 1);
            break;
        case GENIE_EVT_HW_RESET_DONE:
            _reset_light_para();
            BT_DBG("GENIE_EVT_HW_RESET_DONE\n");
            break;
        case GENIE_EVT_SDK_MESH_INIT:
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
            _user_init();
#else
            _led_init();
            _init_light_para();
            _user_init();
            if (!genie_reset_get_flag()) {
                next_event = GENIE_EVT_SDK_ANALYZE_MSG;
            }
#endif
            break;
        case GENIE_EVT_SDK_MESH_PROV_SUCCESS:
            _led_flash(3, 0);
            break;
        case GENIE_EVT_SDK_TRANS_CYCLE:
        case GENIE_EVT_SDK_ACTION_DONE:
        {
            elem_state_t *p_elem = (elem_state_t *)p_arg;
            _led_ctrl(p_elem);
            if(event == GENIE_EVT_SDK_ACTION_DONE)
                _save_light_state(p_elem);
            break;
        }
        case GENIE_EVT_SDK_INDICATE:
            break;
        case GENIE_EVT_SDK_VENDOR_MSG:
            break;
        default:
            break;
    }
    
    if(next_event != event) {
        genie_event(next_event, p_arg);
    }
}

int application_start(int argc, char **argv)
{
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    led_startup();
#endif

    /* genie initilize */
    genie_init();

    BT_INFO("BUILD_TIME:%s", __DATE__","__TIME__);

#ifndef BOARD_TC825X
    // telink will run while(1) of sys_init later.
    aos_loop_run();
#endif

    return 0;
}


#if defined(BOARD_TC825X)
#include "light_ctl_tc825x.c"
#elif defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
#include "light_ctl_tg7100b.c"
#else
#include "light_ctl_default.c"
#endif

