/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#if 0
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include <misc/printk.h>
#include <hal/hal.h>
#include <bluetooth.h>
#include <api/mesh.h>
#include <api/genie_mesh.h>

#include "net.h"
#include "transport.h"

#include "genie_mesh.h"
#endif
#include "genie_app.h"
#include "mesh_hal_ble.h"
#include "mesh/cfg_srv.h"
#include "mesh.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_EVENT)
#include "common/log.h"

extern void user_event(E_GENIE_EVENT event, void *p_arg);
extern elem_state_t g_elem_state[];


static uint8_t g_in_prov = 0;
static uint8_t g_prov_enable = 0;
const char *genie_event_str[] = {
    "SW_RESET",
    "HW_RESET_START",
    "HW_RESET_DONE",

    "SDK->MESH_INIT",
    "SDK->PBADV_START",
    "SDK->PBADV_TIMEOUT",
    "SDK->SILENT_START",

    "SDK->PROV_START",
    "SDK->PROV_DATA",
    "SDK->PROV_TIMEOUT",
    "SDK->PROV_SUCCESS",
    "SDK->PROV_FAIL",

    "SDK->APPKEY_ADD",
    "SDK->APPKEY_DEL",
    "SDK->APPKEY_UPDATE",
    "SDK->NETKEY_ADD",
    "SDK->NETKEY_DEL",
    "SDK->NETKEY_UPDATE",
    "SDK->SUB_ADD",
    "SDK->SUB_DEL",
    "SDK->SEQ_UPDATE",
    "SDK->STATE_SYNC",

    "SDK->ANALYZE_MSG",
#ifdef CONFIG_MESH_MODEL_TRANS
    "SDK->DELAY_START",
    "SDK->DELAY_END",
    "SDK->TRANS_START",
    "SDK->TRANS_CYCLE",
    "SDK->TRANS_END",
#endif
    "SDK->ACTION_DONE",
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    "SDK->PWRON_INDC",
    "SDK->INDICATE",
    "SDK->VENDOR_MSG",
#endif

    "APP->FAC_QUIT",
};

#define GENIE_MESH_EVENT_PRINT(id) BT_DBG(F_YELLOW "%s" F_END, genie_event_str[id])

static E_GENIE_EVENT _genie_event_handle_sw_reset(void)
{
    genie_flash_reset_system();
    bt_mesh_adv_stop();
    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);

    return GENIE_EVT_SDK_MESH_PBADV_START;
}

static E_GENIE_EVENT _genie_event_handle_hw_reset_start(void)
{
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    if(bt_mesh_is_provisioned()) {
        genie_indicate_hw_reset_event(); // Indicate hardware reset event to cloud
    }
#endif
    bt_mesh_adv_stop();
    genie_reset_done();

    return GENIE_EVT_HW_RESET_START;
}

static E_GENIE_EVENT _genie_event_handle_hw_reset_done(void)
{
    genie_reset_clean_count();
    //restart adv
    bt_mesh_reset();
    //BT_DBG("reset mesh done");
    genie_flash_reset_system();
    //BT_DBG("reset flash done");
    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    return GENIE_EVT_SDK_MESH_PBADV_START;
}

static E_GENIE_EVENT _genie_event_handle_mesh_init(void)
{
    //check provsioning status
    uint16_t addr;
    uint32_t seq;

    ais_check_ota_change();

    print_sw_info();

    if(genie_flash_read_addr(&addr) == GENIE_FLASH_SUCCESS && genie_flash_read_seq(&seq) == GENIE_FLASH_SUCCESS){
        BT_DBG("addr(0x%04x) seq(%d)", addr, seq);
        if(genie_flash_read_para(&bt_mesh) == GENIE_FLASH_SUCCESS){
            bt_mesh_setup(seq, addr);
            genie_sub_list_init();
            genie_mesh_setup();
            ais_service_register();
            printk(F_GREEN ">>>proved<<<\n" F_END);

            if (!genie_reset_get_flag()) {
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
                poweron_indicate_start();
#endif
            }
        }
    } else {
        if(!g_prov_enable) {
            printk(F_RED ">>>unprovisioned<<<\n" F_END);
            g_prov_enable = 1;
            bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
            ais_service_register();
        }
        return GENIE_EVT_SDK_MESH_PBADV_START;
    }
    if (genie_reset_get_flag()) {
        return GENIE_EVT_HW_RESET_START;
    }
    return GENIE_EVT_SDK_MESH_INIT;
}

static E_GENIE_EVENT _genie_event_handle_pbadv_start(void)
{
    genie_pbadv_timer_start();
    return GENIE_EVT_SDK_MESH_PBADV_START;
}

static E_GENIE_EVENT _genie_event_handle_pbadv_timeout(void)
{
    genie_pbadv_timer_stop();
    bt_mesh_prov_disable(BT_MESH_PROV_GATT);
    return GENIE_EVT_SDK_MESH_SILENT_START;
}

static E_GENIE_EVENT _genie_event_handle_silent_start(void)
{
    genie_pbadv_start_silent_adv();
    return GENIE_EVT_SDK_MESH_SILENT_START;
}

static E_GENIE_EVENT _genie_event_handle_prov_start(void)
{
    if(g_in_prov == 0) {
        g_in_prov = 1;
        /* disable adv timer */
        genie_pbadv_timer_stop();
        /* enable prov timer */
        genie_prov_timer_start();
    }
    return GENIE_EVT_SDK_MESH_PROV_START;
}

static E_GENIE_EVENT _genie_event_handle_prov_data(uint16_t *p_addr)
{
    genie_flash_write_addr(p_addr);
    return GENIE_EVT_SDK_MESH_PROV_DATA;
}

static E_GENIE_EVENT _genie_event_handle_prov_timeout(void)
{
    return GENIE_EVT_SDK_MESH_PROV_FAIL;
}

static E_GENIE_EVENT _genie_event_handle_prov_success(void)
{
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    return GENIE_EVT_SDK_STATE_SYNC;
#else
    return GENIE_EVT_SDK_MESH_PROV_SUCCESS;
#endif
}

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
static E_GENIE_EVENT _genie_event_handle_sync(void)
{
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    g_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    g_indication_flag |= INDICATION_FLAG_LIGHTNESS;
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
    g_indication_flag |= INDICATION_FLAG_CTL;
#endif
    return GENIE_EVT_SDK_INDICATE;
}
#endif

static E_GENIE_EVENT _genie_event_handle_prov_fail(void)
{
    g_in_prov = 0;
    /* clean prov data */
    /* restart adv timer */
    genie_pbadv_timer_start();
    return GENIE_EVT_SDK_MESH_PROV_FAIL;
}

static void _genie_event_save_mesh_data(uint8_t *p_status)
{
    if(*p_status == 0) {
        genie_flash_write_para(&bt_mesh);
    }
}

static E_GENIE_EVENT _genie_event_handle_appkey_add(uint8_t *p_status)
{
    if(g_in_prov) {
        g_in_prov = 0;
        /* disable prov timer */
        genie_prov_timer_stop();
        if(*p_status == 0) {
            genie_flash_write_para(&bt_mesh);
            return GENIE_EVT_SDK_MESH_PROV_SUCCESS;
        } else {
            return GENIE_EVT_SDK_MESH_PROV_FAIL;
        }
    } else {
        _genie_event_save_mesh_data(p_status);
        return GENIE_EVT_SDK_APPKEY_ADD;
    }
}

static E_GENIE_EVENT _genie_event_handle_sub_add(void)
{
    genie_flash_write_sub(g_sub_list);
    return GENIE_EVT_SDK_SUB_ADD;
}

static E_GENIE_EVENT _genie_event_handle_seq_update(void)
{
    uint32_t seq = bt_mesh.seq;
    genie_flash_write_seq(&seq);
    return GENIE_EVT_SDK_SEQ_UPDATE;
}

static E_GENIE_EVENT _genie_event_handle_analyze_msg(elem_state_t *p_elem)
{
#ifdef CONFIG_MESH_MODEL_TRANS
    if(p_elem->state.trans || p_elem->state.delay) {
        if(p_elem->state.delay) {
            return GENIE_EVT_SDK_DELAY_START;
        } else {
            return GENIE_EVT_SDK_TRANS_START;
        }
    }
#endif
    return GENIE_EVT_SDK_ACTION_DONE;
}

#ifdef CONFIG_MESH_MODEL_TRANS
static E_GENIE_EVENT _genie_event_handle_delay_start(elem_state_t *p_elem)
{
    mesh_timer_stop(p_elem);
    k_timer_start(&p_elem->state.delay_timer, p_elem->state.delay * 5);
    return GENIE_EVT_SDK_DELAY_START;
}

static E_GENIE_EVENT _genie_event_handle_delay_end(elem_state_t *p_elem)
{
    u32_t cur_time = k_uptime_get();

    p_elem->state.delay = 0;

    if(p_elem->state.trans == 0 || cur_time >= p_elem->state.trans_end_time) {
        clear_trans_para(p_elem);
        return GENIE_EVT_SDK_ACTION_DONE;
    } else {
        return GENIE_EVT_SDK_TRANS_START;
    }
}

static E_GENIE_EVENT _genie_event_handle_trans_start(elem_state_t *p_elem)
{
    u32_t cur_time = k_uptime_get();

    mesh_timer_stop(p_elem);

    //check time
    if(cur_time >= p_elem->state.trans_end_time - MESH_TRNSATION_CYCLE) {
        return GENIE_EVT_SDK_TRANS_END;
    } else {
        //start cycle
        k_timer_start(&p_elem->state.trans_timer, MESH_TRNSATION_CYCLE);
        BT_DBG("start trans %p", &p_elem->state.trans_timer);
        return GENIE_EVT_SDK_TRANS_START;
    }
}

static E_GENIE_EVENT _genie_event_handle_trans_cycle(elem_state_t *p_elem)
{
    if(calc_cur_state(p_elem) == 0) {
        p_elem->state.trans = 0;
    }
    return GENIE_EVT_SDK_TRANS_CYCLE;
}

static E_GENIE_EVENT _genie_event_handle_trans_end(elem_state_t *p_elem)
{
    //clear paras
    clear_trans_para(p_elem);
    //action done
    return GENIE_EVT_SDK_ACTION_DONE;
}
#endif

static E_GENIE_EVENT _genie_event_handle_action_done(elem_state_t *p_elem)
{
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    BT_DBG("onoff cur(%d) tar(%d)", p_elem->state.onoff[T_CUR], p_elem->state.onoff[T_TAR]);

    if(p_elem->state.onoff[T_CUR] != p_elem->state.onoff[T_TAR]) {
        p_elem->state.onoff[T_CUR] = p_elem->state.onoff[T_TAR];
    }
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    BT_DBG("actual cur(%04x) tar(%04x)", p_elem->state.actual[T_CUR], p_elem->state.actual[T_TAR]);

    if(p_elem->state.actual[T_CUR] != p_elem->state.actual[T_TAR]) {
        p_elem->state.actual[T_CUR] = p_elem->state.actual[T_TAR];
    }
#endif

#ifdef CONFIG_MESH_MODEL_CTL_SRV
    BT_DBG("temp cur(%04x) tar(%04x)", p_elem->state.temp[T_CUR], p_elem->state.temp[T_TAR]);

    if(p_elem->state.temp[T_CUR] != p_elem->state.temp[T_TAR]) {
        p_elem->state.temp[T_CUR] = p_elem->state.temp[T_TAR];
    }
#endif

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    if(bt_mesh_is_provisioned()) {
        return GENIE_EVT_SDK_INDICATE;
    }
#endif
    return GENIE_EVT_SDK_ACTION_DONE;
}

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
static E_GENIE_EVENT _genie_event_handle_pwron_indc(elem_state_t *p_elem)
{
    g_indication_flag |= INDICATION_FLAG_POWERON;
    return GENIE_EVT_SDK_INDICATE;
}

static E_GENIE_EVENT _genie_event_handle_indicate(elem_state_t *p_elem)
{
    if(g_indication_flag) {
        standart_indication(p_elem);
    }
    return GENIE_EVT_SDK_INDICATE;
}

static E_GENIE_EVENT _genie_event_handle_vnd_msg(vnd_model_msg *p_msg)
{
    BT_DBG("vendor message received");
    genie_vnd_msg_handle(p_msg);
    return GENIE_EVT_SDK_VENDOR_MSG;
}
#endif

void genie_event(E_GENIE_EVENT event, void *p_arg)
{
    E_GENIE_EVENT next_event = event;
    uint8_t ignore_user_event = 0;

#ifdef CONFIG_MESH_MODEL_TRANS
    if(event != GENIE_EVT_SDK_TRANS_CYCLE) {
        GENIE_MESH_EVENT_PRINT(event);
    }
#endif
    switch(event) {
        case GENIE_EVT_SW_RESET:
            //call user_event first
            user_event(GENIE_EVT_SW_RESET, p_arg);
            ignore_user_event = 1;
            next_event = _genie_event_handle_sw_reset();
            break;

        case GENIE_EVT_HW_RESET_START:
            _genie_event_handle_hw_reset_start();
            break;

        case GENIE_EVT_HW_RESET_DONE:
            //call user_event first
            user_event(GENIE_EVT_HW_RESET_DONE, p_arg);
            ignore_user_event = 1;
            next_event = _genie_event_handle_hw_reset_done();
            break;

        case GENIE_EVT_SDK_MESH_INIT:
            //update p_arg to user_event
            p_arg = (void *)&g_elem_state[0];
            next_event = _genie_event_handle_mesh_init();
            break;

        case GENIE_EVT_SDK_MESH_PBADV_START:
            next_event = _genie_event_handle_pbadv_start();
            break;

        case GENIE_EVT_SDK_MESH_PBADV_TIMEOUT:
            next_event = _genie_event_handle_pbadv_timeout();
            break;

        case GENIE_EVT_SDK_MESH_SILENT_START:
            next_event = _genie_event_handle_silent_start();
            break;

        case GENIE_EVT_SDK_MESH_PROV_START:
            next_event = _genie_event_handle_prov_start();
            break;

        case GENIE_EVT_SDK_MESH_PROV_DATA:
            next_event = _genie_event_handle_prov_data((uint16_t *)p_arg);
            break;

        case GENIE_EVT_SDK_MESH_PROV_TIMEOUT:
            next_event = _genie_event_handle_prov_timeout();
            break;

        case GENIE_EVT_SDK_MESH_PROV_SUCCESS:
            next_event = _genie_event_handle_prov_success();
            break;

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
        case GENIE_EVT_SDK_STATE_SYNC:
            next_event = _genie_event_handle_sync();
            break;
#endif
        case GENIE_EVT_SDK_MESH_PROV_FAIL:
            next_event = _genie_event_handle_prov_fail();
            break;

        case GENIE_EVT_SDK_APPKEY_ADD:
        {
            next_event = _genie_event_handle_appkey_add((uint8_t *)p_arg);
            if(next_event == GENIE_EVT_SDK_MESH_PROV_SUCCESS) {
                p_arg = &g_elem_state[0];
            }
            break;
        }

        case GENIE_EVT_SDK_APPKEY_DEL:
        case GENIE_EVT_SDK_APPKEY_UPDATE:
        case GENIE_EVT_SDK_NETKEY_ADD:
        case GENIE_EVT_SDK_NETKEY_DEL:
        case GENIE_EVT_SDK_NETKEY_UPDATE:
        //case GENIE_EVT_SDK_IVI_UPDATE:
            _genie_event_save_mesh_data((uint8_t *)p_arg);
            break;

        case GENIE_EVT_SDK_SUB_ADD:
            next_event = _genie_event_handle_sub_add();
            break;

        case GENIE_EVT_SDK_SUB_DEL:
            break;

        case GENIE_EVT_SDK_SEQ_UPDATE:
            next_event = _genie_event_handle_seq_update();
            break;

        case GENIE_EVT_SDK_ANALYZE_MSG:
            next_event = _genie_event_handle_analyze_msg((elem_state_t *)p_arg);
            break;

#ifdef CONFIG_MESH_MODEL_TRANS
        case GENIE_EVT_SDK_DELAY_START:
            next_event = _genie_event_handle_delay_start((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_DELAY_END:
            next_event = _genie_event_handle_delay_end((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_TRANS_START:
            next_event = _genie_event_handle_trans_start((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_TRANS_CYCLE:
            next_event = _genie_event_handle_trans_cycle((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_TRANS_END:
            next_event = _genie_event_handle_trans_end((elem_state_t *)p_arg);
            break;
#endif

        case GENIE_EVT_SDK_ACTION_DONE:
            next_event = _genie_event_handle_action_done((elem_state_t *)p_arg);
            break;

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
        case GENIE_EVT_SDK_MESH_PWRON_INDC:
            next_event = _genie_event_handle_pwron_indc((elem_state_t *)p_arg);
            p_arg = &g_elem_state[0];
            break;

        case GENIE_EVT_SDK_INDICATE:
            next_event = _genie_event_handle_indicate((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_VENDOR_MSG:
            next_event = _genie_event_handle_vnd_msg((vnd_model_msg *)p_arg);
            break;
#endif
        case GENIE_EVT_APP_FAC_QUIT:
            break;
        default:
            break;
    }

    if(!ignore_user_event)
        user_event(event, p_arg);

    if(next_event != event) {
        genie_event(next_event, p_arg);
    }
}

