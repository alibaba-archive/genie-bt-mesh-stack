/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include "genie_app.h"
#include "mesh_hal_ble.h"
#include "mesh/cfg_srv.h"
#include "mesh.h"
#include "prov.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_EVENT)
#include "common/log.h"

extern void user_event(E_GENIE_EVENT event, void *p_arg);
extern elem_state_t g_elem_state[];

static bool g_genie_provisioned = 0;
static uint8_t g_in_prov = 0;
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
    "SDK->HB_SET",
    "SDK->SEQ_UPDATE",
    "SDK->STATE_SYNC",

    "SDK->ANALYZE_MSG",
    "SDK->AIS_DISCON",
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

static void _genie_reset_prov(void)
{
    g_genie_provisioned = 0;
    /* reset prov */
    bt_mesh_reset();
    genie_flash_reset_system();
#ifdef GENIE_ULTRA_PROV
    ultra_prov_free();
#endif
}

static E_GENIE_EVENT _genie_event_handle_sw_reset(void)
{
    _genie_reset_prov();
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
    _genie_reset_prov();
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    aos_reboot();
#endif
    /* restart adv */
    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    return GENIE_EVT_SDK_MESH_PBADV_START;
}

static E_GENIE_EVENT _genie_event_handle_mesh_init(void)
{
    //check provsioning status
    uint16_t addr;
    uint32_t seq;
    uint8_t devkey[16];
    mesh_netkey_para_t netkey;
    mesh_appkey_para_t appkey;

#ifdef CONFIG_GENIE_OTA
    ais_check_ota_change();
#endif

    print_sw_info();

    // bit0:unicast_address
    // bit1:seq
    // bit2:devkey
    // bit3:netkey
    // bit4:appkey
    uint8_t read_flag = 0;
    if(genie_flash_read_addr(&addr) == GENIE_FLASH_SUCCESS) {
        read_flag |= 0x01;
    }
    if(genie_flash_read_seq(&seq) == GENIE_FLASH_SUCCESS) {
        read_flag |= 0x02;
    }
#ifdef CONIFG_OLD_FLASH_PARA
    if(genie_flash_read_para(&bt_mesh) == GENIE_FLASH_SUCCESS){
        BT_DBG_R("read old");
        read_flag |= 0x1C;  //0001 1100
        // save data by new format.
        memcpy(devkey, bt_mesh.dev_key, 16);
        memset(&netkey, 0, sizeof(netkey));
        memcpy(netkey.key, bt_mesh.sub[0].keys[0].net, 16);
        memset(&appkey, 0, sizeof(appkey));
        memcpy(appkey.key, bt_mesh.app_keys[0].keys[0].val, 16);
        genie_flash_write_devkey(devkey);
        genie_flash_write_netkey(&netkey);
        genie_flash_write_appkey(&appkey);
        genie_flash_delete_reliable(GFI_MESH_PARA);
        memset(&bt_mesh, 0, sizeof(bt_mesh));
        bt_mesh.sub[0].net_idx = BT_MESH_KEY_UNUSED;
        bt_mesh.app_keys[0].net_idx = BT_MESH_KEY_UNUSED;
    } else
#endif
    {
        if(genie_flash_read_devkey(devkey) == GENIE_FLASH_SUCCESS) {
            read_flag |= 0x04;
        }
        if(genie_flash_read_netkey(&netkey) == GENIE_FLASH_SUCCESS) {
            read_flag |= 0x08;
        }
        if(genie_flash_read_appkey(&appkey) == GENIE_FLASH_SUCCESS) {
            read_flag |= 0x10;
        }
    }

    BT_DBG("flag %02x", read_flag);
    if((read_flag & 0x1F) == 0x1F) {
        printk(F_GREEN ">>>proved<<<\n" F_END);

        bt_mesh_provision(netkey.key, netkey.net_index, netkey.flag, netkey.ivi, seq, addr, devkey);
        extern void genie_appkey_register(u16_t net_idx, u16_t app_idx, const u8_t val[16], bool update);
        genie_appkey_register(appkey.net_index, appkey.key_index, appkey.key, appkey.flag);

#ifdef CONFIG_GENIE_OTA
        ais_service_register();
#endif
        /* check hb */
        mesh_hb_para_t hb_para = {.count = 0};
        genie_flash_read_hb(&hb_para);
        if(hb_para.count == 0xFF) {
            extern u8_t genie_heartbeat_set(mesh_hb_para_t *p_para);
            genie_heartbeat_set(&hb_para);
        }

        if (!genie_reset_get_flag()) {
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_POWERON;
            genie_indicate_start(0, &g_elem_state[0]);
#endif
        } else {
            return GENIE_EVT_HW_RESET_START;
        }
    } else if(read_flag){
        printk(F_YELLOW ">>>error<<<\n" F_END);
        genie_flash_reset_system();
        aos_reboot();
    } else {
        printk(F_RED ">>>unprovisioned<<<\n" F_END);
        if (genie_reset_get_flag()) {
            return GENIE_EVT_HW_RESET_START;
        }
#ifdef CONFIG_GENIE_OTA
        ais_service_register();
#endif
        bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
        return GENIE_EVT_SDK_MESH_PBADV_START;
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
    bt_mesh_prov_disable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
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
    g_genie_provisioned = 1;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    return GENIE_EVT_SDK_STATE_SYNC;
#else
    return GENIE_EVT_SDK_MESH_PROV_SUCCESS;
#endif
}

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
static E_GENIE_EVENT _genie_event_handle_sync(elem_state_t *p_elem)
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
    genie_indicate_start(1000, p_elem);

    return GENIE_EVT_SDK_STATE_SYNC;
}
#endif

static E_GENIE_EVENT _genie_event_handle_prov_fail(void)
{
    /* reset prov */
    g_in_prov = 0;
    _genie_reset_prov();
    /* restart adv */
    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    return GENIE_EVT_SDK_MESH_PBADV_START;
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
            //genie_flash_write_para(&bt_mesh);
            uint8_t devkey[16];
            mesh_netkey_para_t netkey;
            mesh_appkey_para_t appkey;
            
            memcpy(devkey, bt_mesh.dev_key, 16);
            memset(&netkey, 0, sizeof(netkey));
            memcpy(netkey.key, bt_mesh.sub[0].keys[0].net, 16);
            memset(&appkey, 0, sizeof(appkey));
            memcpy(appkey.key, bt_mesh.app_keys[0].keys[0].val, 16);
            genie_flash_write_devkey(devkey);
            genie_flash_write_netkey(&netkey);
            genie_flash_write_appkey(&appkey);
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

static E_GENIE_EVENT _genie_event_handle_hb_set(mesh_hb_para_t *p_para)
{
    BT_DBG("save");
    genie_flash_write_hb(p_para);
    return GENIE_EVT_SDK_HB_SET;
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

static E_GENIE_EVENT _genie_event_handle_ais_discon(elem_state_t *p_elem)
{
    /* restart adv */
    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    return GENIE_EVT_SDK_MESH_PBADV_START;
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
    //g_indication_flag |= INDICATION_FLAG_POWERON;
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
#ifdef MESH_MODEL_VENDOR_TIMER
static E_GENIE_EVENT _genie_event_handle_order_msg(vendor_attr_data_t *attr_data)
{
    if (attr_data->type == ONOFF_T) {
        g_elem_state[0].state.onoff[T_TAR] = attr_data->para;
    }
#ifdef CONFIG_MESH_MODEL_TRANS
    return GENIE_EVT_SDK_TRANS_CYCLE;
#else
    return GENIE_EVT_SDK_ACTION_DONE;
#endif
}

#endif
#endif

bool genie_is_provisioned(void)
{
    return g_genie_provisioned;
}

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
            next_event = _genie_event_handle_sync(p_arg);
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
            //_genie_event_save_mesh_data((uint8_t *)p_arg);
            break;

        case GENIE_EVT_SDK_SUB_ADD:
            next_event = _genie_event_handle_sub_add();
            break;

        case GENIE_EVT_SDK_SUB_DEL:
            break;

        case GENIE_EVT_SDK_HB_SET:
            next_event = _genie_event_handle_hb_set((mesh_hb_para_t *)p_arg);
            break;

        case GENIE_EVT_SDK_SEQ_UPDATE:
            next_event = _genie_event_handle_seq_update();
            break;

        case GENIE_EVT_SDK_ANALYZE_MSG:
            next_event = _genie_event_handle_analyze_msg((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_AIS_DISCON:
            next_event = _genie_event_handle_ais_discon((elem_state_t *)p_arg);
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
            //p_arg = &g_elem_state[0];
            break;

        case GENIE_EVT_SDK_INDICATE:
            next_event = _genie_event_handle_indicate((elem_state_t *)p_arg);
            break;

        case GENIE_EVT_SDK_VENDOR_MSG:
            next_event = _genie_event_handle_vnd_msg((vnd_model_msg *)p_arg);
            break;
#endif
#ifdef MESH_MODEL_VENDOR_TIMER
        case GENIE_EVT_TIME_OUT:
        {
            next_event = _genie_event_handle_order_msg((vendor_attr_data_t *)p_arg);
            p_arg = &g_elem_state[0];
            break;
        }
#endif
        default:
            break;
    }

    if(!ignore_user_event)
        user_event(event, p_arg);

    if(next_event != event) {
        genie_event(next_event, p_arg);
    }
}

