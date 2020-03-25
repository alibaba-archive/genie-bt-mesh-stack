/* main.c - light demo */

/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
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
#ifdef BOARD_BK3435DEVKIT
#include "gpio_pub.h"
#endif

#define DEFAULT_MESH_GROUP1 0xC000
#define DEFAULT_MESH_GROUP2 0xCFFF

#ifdef BOARD_TC825X
#include "drivers/8258/gpio_8258.h"
#include "vendor/common/alios_app_config.h"

#define WARM_PIN            TC825X_GET_PIN_NUM(PWM_R)
#define COLD_PIN            TC825X_GET_PIN_NUM(PWM_R)

#define LIGHT_LED           TC825X_GET_PIN_NUM(PWM_R)
#define LIGHT_LEVEL_UP      TC825X_GET_PIN_NUM(PWM_G)
#define LIGHT_LEVEL_DOWN    TC825X_GET_PIN_NUM(PWM_B)
#define LIGHT_DRIVE_TYPE    OUTPUT_PUSH_PULL

#define LIGHT_BTN_ONOFF     TC825X_GET_PIN_NUM(BTN_K1)
#define LIGHT_BTN_LEVEL_UP  TC825X_GET_PIN_NUM(BTN_K2)
#define LIGHT_BTN_LEVEL_DOWN    TC825X_GET_PIN_NUM(PWM_W)
#define LIGHT_BTN_INPUT_TYPE    INPUT_PULL_UP
#define LIGHT_BTN_IRQ_TRIGGER IRQ_TRIGGER_FALLING_EDGE

#define CW_TEMP_MIN            (0x0320)    // 800
#define CW_TEMP_MAX            (0x4E20)    // 20000

#endif

#ifdef BOARD_PCA10040
#define WARM_PIN 15
#define COLD_PIN 14

#define LIGHT_LED 17
#define LIGHT_LEVEL_UP 18
#define LIGHT_LEVEL_DOWN 19
#define LIGHT_DRIVE_TYPE OUTPUT_PUSH_PULL

#define LIGHT_BTN_ONOFF 13
#define LIGHT_BTN_LEVEL_UP 14
#define LIGHT_BTN_LEVEL_DOWN 15
#define LIGHT_BTN_IRQ_TRIGGER IRQ_TRIGGER_FALLING_EDGE
#define LIGHT_BTN_INPUT_TYPE INPUT_PULL_UP

#define CW_TEMP_MIN            (0x0320)    // 800
#define CW_TEMP_MAX            (0x4E20)    // 20000
#endif

#ifdef BOARD_BK3435DEVKIT
#define WARM_PIN GPIO_P10 ///GPIO P10
#define COLD_PIN GPIO_P11 ///GPIO P11

#define LIGHT_BTN_DEBOUNCE_TIME 50 ///50ms
#define LIGHT_LEVEL_NUMBER      100 ///light 1~100 level
#define LIGHT_MIN_LEVEL         10  ///light min level
#define LIGHT_MAX_LEVEL         LIGHT_LEVEL_NUMBER  ///light max level
#define LIGHT_LEVEL_ADJ_BASE    10

#define LIGHT_LED        GPIO_P14 ///GPIO P14
#define LIGHT_LEVEL_UP   GPIO_P15 ///GPIO P15
#define LIGHT_LEVEL_DOWN GPIO_P16 ///GPIO P16
#define LIGHT_DRIVE_TYPE OUTPUT_PUSH_PULL

#define LIGHT_BTN_ONOFF       GPIO_P20 ///GPIO P20
#define LIGHT_BTN_LEVEL_UP    GPIO_P21 ///GPIO P21
#define LIGHT_BTN_LEVEL_DOWN  GPIO_P22 ///GPIO P22
#define LIGHT_BTN_IRQ_TRIGGER IRQ_TRIGGER_FALLING_EDGE
#define LIGHT_BTN_INPUT_TYPE  INPUT_PULL_UP

#define CW_TEMP_MIN            (0x0320)    // 800
#define CW_TEMP_MAX            (0x4E20)    // 20000
#endif

#define MESH_ELEM_COUNT 1
#define MESH_ELEM_STATE_COUNT MESH_ELEM_COUNT

#define MESH_PBADV_TIME 600 //unit:s

typedef struct {
#if 0
    bool onoff;
    u16_t lightness;
    u16_t temp;
    bool lightness_changing;
    bool temp_changing;
#endif
    gpio_dev_t warm_pin;
    gpio_dev_t cold_pin;
    pwm_dev_t warm_pwm;
    pwm_dev_t cold_pwm;
} _LIGHT_CTL;

typedef struct{
    uint16_t cw;
    uint16_t pwm[2];
}cw_table_t;

static const cw_table_t lsd_cw_table[] = {
    {2700,{0,65535}},
    {2800,{1724,63810}},
    {2900,{3449,62085}},
    {3000,{5173,60361}},
    {3100,{6898,58636}},
    {3200,{8623,56911}},
    {3300,{10347,55187}},
    {3400,{12072,53462}},
    {3500,{13796,51738}},
    {3600,{15521,50013}},
    {3700,{17246,48288}},
    {3800,{18970,46564}},
    {3900,{20695,44839}},
    {4000,{22419,43115}},
    {4100,{24144,41390}},
    {4200,{25869,39665}},
    {4300,{27593,37941}},
    {4400,{29318,36216}},
    {4500,{31042,34492}},
    {4600,{32767,32767}},
    {4700,{34492,31042}},
    {4800,{36216,29318}},
    {4900,{37941,27593}},
    {5000,{39665,25869}},
    {5100,{41390,24144}},
    {5200,{43115,22419}},
    {5300,{44839,20695}},
    {5400,{46564,18970}},
    {5500,{48288,17246}},
    {5600,{50013,15521}},
    {5700,{51738,13796}},
    {5800,{53462,12072}},
    {5900,{55187,10347}},
    {6000,{56911,8623}},
    {6100,{58636,6898}},
    {6200,{60361,5173}},
    {6300,{62085,3449}},
    {6400,{63810,1724}},
    {6500,{65535,0}},
};

void load_light_state(void);

static uint8_t led_ctrl(gpio_dev_t *led, bool on);
static uint8_t lsd_led_update_last(uint8_t on, uint16_t actual, uint16_t temperature);
static void temperature_to_pwm(uint16_t cw_in,uint16_t *cw_pwm);

static uint8_t leds_init(void);


static gpio_dev_t light_led;

static pwm_dev_t light_led_c;
static pwm_dev_t light_led_w;

elem_state_t g_elem_state[MESH_ELEM_STATE_COUNT];
model_powerup_t g_powerup[MESH_ELEM_STATE_COUNT];


static struct bt_mesh_model element_models[] = {
    BT_MESH_MODEL_CFG_SRV(),
    BT_MESH_MODEL_HEALTH_SRV(),

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    MESH_MODEL_GEN_ONOFF_SRV(&g_elem_state[0]),
#endif

#ifdef CONFIG_MESH_MODEL_GEN_LEVEL_SRV
    MESH_MODEL_GEN_LEVEL_SRV(&g_elem_state[0]),
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    MESH_MODEL_LIGHTNESS_SRV(&g_elem_state[0]),
#endif

#ifdef CONFIG_MESH_MODEL_CTL_SRV
    MESH_MODEL_CTL_SRV(&g_elem_state[0]),
#ifndef CONFIG_ALI_SIMPLE_MODLE
    MESH_MODEL_CTL_SETUP_SRV(&g_elem_state[0]),
#endif
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

    //read flash
    memset(g_elem_state, 0, sizeof(g_elem_state));
    elem_state_init(MESH_ELEM_STATE_COUNT, g_elem_state);

    load_light_state();
    //tmall_mesh_user_task_init();
}

void reset_light_para(void)
{
    uint8_t i = 0;

    while(i < MESH_ELEM_STATE_COUNT) {
#ifdef CONFIG_ALI_SIMPLE_MODLE
        g_elem_state[i].state.onoff[T_CUR] = GEN_ONOFF_DEFAULT;
        g_elem_state[i].state.actual[T_CUR] = LIGHTNESS_DEFAULT;
        g_elem_state[i].state.temp[T_CUR] = CTL_TEMP_DEFAULT;
        g_elem_state[i].state.onoff[T_TAR] = GEN_ONOFF_DEFAULT;
        g_elem_state[i].state.actual[T_TAR] = LIGHTNESS_DEFAULT;
        g_elem_state[i].state.temp[T_TAR] = CTL_TEMP_DEFAULT;
#ifdef CONFIG_MESH_MODEL_TRANS
        g_elem_state[i].state.trans = 0;
        g_elem_state[i].state.delay = 0;
        g_elem_state[i].state.trans_start_time = 0;
        g_elem_state[i].state.trans_end_time = 0;
#endif
        g_elem_state[i].powerup.last_actual = LIGHTNESS_DEFAULT;
        g_elem_state[i].powerup.last_temp = CTL_TEMP_DEFAULT;
        g_powerup[i].last_actual = LIGHTNESS_DEFAULT;
        g_powerup[i].last_temp = CTL_TEMP_DEFAULT;

#endif
        i++;
    }

    genie_flash_write_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));

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
    printk("%s %d\n", times);
    //aos_msleep(times * 1000);
}

void save_light_state(elem_state_t *p_elem)
{
    if(p_elem->state.actual[T_CUR] != 0) {
        p_elem->powerup.last_actual = p_elem->state.actual[T_CUR];
        g_powerup[p_elem->elem_index].last_actual = p_elem->state.actual[T_CUR];
    }

    p_elem->powerup.last_temp = p_elem->state.temp[T_CUR];
    g_powerup[p_elem->elem_index].last_temp = p_elem->state.temp[T_CUR];

    BT_DBG_R("elem %d, actual %d temp %d", p_elem->elem_index, g_powerup[p_elem->elem_index].last_actual, g_powerup[p_elem->elem_index].last_temp);

    genie_flash_write_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));
}

void load_light_state(void)
{
    uint8_t i = 0;
    E_GENIE_FLASH_ERRCODE ret = genie_flash_read_userdata(GFI_MESH_POWERUP, (uint8_t *)g_powerup, sizeof(g_powerup));

    if(ret == GENIE_FLASH_SUCCESS) {
        while(i < MESH_ELEM_STATE_COUNT) {
            memcpy(&g_elem_state[i].powerup, &g_powerup[i], sizeof(model_powerup_t));
            BT_DBG_R("elem %d, actual %d temp %d", i, g_powerup[i].last_actual, g_powerup[i].last_temp);
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
            if(g_powerup[i].last_actual) {
                g_elem_state[i].state.actual[T_TAR] = g_powerup[i].last_actual;
            }
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
            if(g_powerup[i].last_temp) {
                g_elem_state[i].state.temp[T_TAR] = g_powerup[i].last_temp;
            }
#endif
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
            if(g_elem_state[i].state.actual[T_TAR] == 0) {
                g_elem_state[i].state.onoff[T_TAR] = 0;
            } else {
                g_elem_state[i].state.onoff[T_TAR] = 1;
            }
#endif
#ifdef CONFIG_MESH_MODEL_TRANS
            if(g_elem_state[i].state.onoff[T_TAR] == 0) {
                g_elem_state[i].state.trans = 0;
                g_elem_state[i].state.delay = 0;
                g_elem_state[i].state.trans_start_time = 0;
                g_elem_state[i].state.trans_end_time = 0;
            } else {
                g_elem_state[i].state.trans = 0x41;
                g_elem_state[i].state.delay = 100;
                g_elem_state[i].state.trans_start_time = k_uptime_get() + g_elem_state[i].state.delay * 5;
                g_elem_state[i].state.trans_end_time = g_elem_state[i].state.trans_start_time + get_transition_time(g_elem_state[i].state.trans);
            }
#endif
#endif
            i++;
        }
        BT_DBG("+ done");
    }
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

#if defined(CONFIG_MESH_MODEL_CTL_SRV)
static void _led_set(uint8_t elem_index, uint8_t on, uint16_t actual, uint16_t temperature)
{
    static uint8_t last_onoff = 0;
    static uint16_t last_acual = 0;
    static uint16_t last_temperature = 0;

    if(last_onoff != on) {
        last_onoff = on;
        lsd_led_update_last(last_onoff,last_acual, last_temperature);
    }
    if(last_acual != actual) {
        last_acual = actual;
        lsd_led_update_last(last_onoff,last_acual, last_temperature);
    }
    if(last_temperature != temperature) {
        last_temperature = temperature;
        lsd_led_update_last(last_onoff,last_acual, last_temperature);
    }
}
#elif defined(CONFIG_MESH_MODEL_LIGHTNESS_SRV)
static void _led_set(uint8_t elem_index, uint8_t on, uint16_t actual)
{
    static uint8_t last_onoff = 0;
    static uint16_t last_acual = 0;
    printk(">>>>elem %d set %d %d\n", elem_index, on, actual);
    if(last_onoff != on) {
        last_onoff = on;
        led_ctrl(&light_led, last_onoff);
    }
    if(last_acual != actual) {
        last_acual = actual;
    }
}
#elif defined(CONFIG_MESH_MODEL_GEN_ONOFF_SRV)
static void _led_set(uint8_t elem_index, uint8_t on)
{
    static uint8_t last_onoff = 0;
    printk(">>>>elem %d set %d\n", elem_index, on);
    if(last_onoff != on) {
        last_onoff = on;
        led_ctrl(&light_led, last_onoff);
    }
}
#endif

static uint8_t lsd_led_update_last(uint8_t on, uint16_t actual, uint16_t temperature)
{
    uint8_t err = 1;
    pwm_config_t pwm_cfg_c;
    pwm_config_t pwm_cfg_w;
    uint16_t temperature_table = 0;
    uint16_t temp_pwm[2] = {0};

    if(on > 1)  return err;

    if(CW_TEMP_MIN > temperature){
       temperature = CW_TEMP_MIN;
    }
    if(CW_TEMP_MAX < temperature){
        temperature = CW_TEMP_MAX;
    }

    pwm_cfg_c.freq = 300000;
    pwm_cfg_w.freq = 300000;

    if (on) {
        temperature_table = ((float)(temperature-800)/(20000-800)*3800)+2700;
        temperature_to_pwm(temperature_table, temp_pwm);

        pwm_cfg_c.duty_cycle = ((float)actual/65535)*((float)temp_pwm[0]/65535);
        pwm_cfg_w.duty_cycle = ((float)actual/65535)*((float)temp_pwm[1]/65535);
    }
    else{
        pwm_cfg_c.duty_cycle = 0;
        pwm_cfg_w.duty_cycle = 0;
    }

#ifdef BOARD_TC825X
    err = hal_pwm_para_chg(&light_led_c,pwm_cfg_c);
    err += hal_pwm_para_chg(&light_led_w,pwm_cfg_w);
#endif
    return err;
}

static void temperature_to_pwm(uint16_t cw_in,uint16_t *cw_pwm)
{
    if(cw_in<2700){
        cw_in = 2700;
    }
    if(cw_in>6500){
        cw_in = 6500;
    }

    uint8_t i = 0;
    uint8_t num = sizeof(lsd_cw_table)/sizeof(cw_table_t);
    for(i=0;i<num-1;i++){
        if(cw_in<lsd_cw_table[i+1].cw)
            break;
    }

    if(i>=(num-1)){
        for(uint8_t j=0;j<2;j++){
            cw_pwm[j] = lsd_cw_table[num-1].pwm[j];
        }
        return;
    }

    uint8_t lcnt = i;
    uint8_t hcnt = i+1;

    for(uint8_t j=0;j<2;j++)
    {
        if(lsd_cw_table[hcnt].pwm[j]>lsd_cw_table[lcnt].pwm[j]){
            cw_pwm[j] = lsd_cw_table[lcnt].pwm[j] +((cw_in-lsd_cw_table[lcnt].cw)*(lsd_cw_table[hcnt].pwm[j]-lsd_cw_table[lcnt].pwm[j])/(lsd_cw_table[hcnt].cw-lsd_cw_table[lcnt].cw));
        }
        else{
            cw_pwm[j] = lsd_cw_table[lcnt].pwm[j] - ((cw_in-lsd_cw_table[lcnt].cw)*(lsd_cw_table[lcnt].pwm[j]-lsd_cw_table[hcnt].pwm[j])/(lsd_cw_table[hcnt].cw-lsd_cw_table[lcnt].cw));
        }
    }
}

#ifdef CONFIG_GENIE_OTA
bool ota_check_reboot(void)
{
    if(g_elem_state[0].state.onoff[T_CUR] == 0) {
        BT_DBG_R("reboot!");
        return true;
    }
    BT_DBG_R("no reboot!");
    return false;
}
#endif

void user_event(E_GENIE_EVENT event, void *p_arg)
{
    E_GENIE_EVENT next_event = event;

    //BT_DBG_R("%s, %s %p\n", __func__, genie_event_str[event], p_arg);
    switch(event) {
        case GENIE_EVT_SW_RESET:
        case GENIE_EVT_HW_RESET_START:
            BT_DBG_R("FLASH x5");
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
            BT_DBG_R("FLASH x3");
            led_flash(3);
            break;
#ifdef CONFIG_MESH_MODEL_TRANS
        case GENIE_EVT_SDK_TRANS_CYCLE:
#endif
        case GENIE_EVT_SDK_ACTION_DONE:
        {
            elem_state_t *p_elem = (elem_state_t *)p_arg;
#if defined(CONFIG_MESH_MODEL_CTL_SRV)
            _led_set(p_elem->elem_index, p_elem->state.onoff[T_CUR], p_elem->state.actual[T_CUR], p_elem->state.temp[T_CUR]);
#elif defined(CONFIG_MESH_MODEL_LIGHTNESS_SRV)
            _led_set(p_elem->elem_index, p_elem->state.onoff[T_CUR], p_elem->state.actual[T_CUR]);
#elif defined(CONFIG_MESH_MODEL_GEN_ONOFF_SRV)
            _led_set(p_elem->elem_index, p_elem->state.onoff[T_CUR]);
#endif
            if(event == GENIE_EVT_SDK_ACTION_DONE)
                save_light_state(p_elem);
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
        case GENIE_EVT_APP_FAC_QUIT:
            break;
        default:
            break;
    }
    
    if(next_event != event) {
        genie_event(next_event, p_arg);
    }
}

static uint8_t led_ctrl(gpio_dev_t *led, bool on)
{
    if (!led)
        return -1;

#if BOARD_TC825X
    on = !on;   // PCB is high valid
#endif

    if (on)
        hal_gpio_output_low(led);
    else
        hal_gpio_output_high(led);
}

/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static u8_t leds_init(void)
{
#ifdef BOARD_TC825X
    light_led_c.port = LIGHT_LEVEL_UP;
    light_led_c.config.duty_cycle = 0;
    light_led_c.config.freq = 300000;
    hal_pwm_init(&light_led_c);
    hal_pwm_start(&light_led_c);

    light_led_w.port = LIGHT_LEVEL_DOWN;
    light_led_w.config.duty_cycle = 0;
    light_led_w.config.freq = 300000;
    hal_pwm_init(&light_led_w);
    hal_pwm_start(&light_led_w);
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

