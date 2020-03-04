/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include <misc/printk.h>
#include <hal/hal.h>
#include <bluetooth.h>
#include <api/mesh.h>

//#include "genie_app.h"
#include "net.h"
#include "transport.h"
#include "genie_app.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_MODEL)
#include "common/log.h"

extern u32_t get_mesh_pbadv_time(void);
extern void mesh_sub_init(u16_t *p_sub);
extern uint8_t get_vendor_element_num(void);
extern void user_prov_complete(u16_t net_idx, u16_t addr);
extern void user_prov_reset(void);

//implemented at ./network/bluetooth/bt_mesh/src/beacon.c:51
extern void unprov_beacon_interval_set(uint32_t delay);
extern int hci_driver_init(void);

#define MESH_PROVISIONING_TIMEOUT 60*1000

static struct bt_mesh_prov prov;
static struct bt_mesh_comp comp;

extern struct bt_mesh_elem elements[];


uint8_t g_indication_flag;

static struct k_timer g_pbadv_timer;
static struct k_timer g_prov_timer;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
static struct k_timer g_indc_timer;
#endif

#ifdef CONFIG_MESH_MODEL_TRANS
void mesh_timer_stop(elem_state_t *p_elem)
{
    k_timer_stop(&p_elem->state.delay_timer);
    k_timer_stop(&p_elem->state.trans_timer);
}

static void _mesh_delay_timer_cb(void *p_timer, void *p_arg)
{
    elem_state_t *p_elem = (elem_state_t *)p_arg;

    mesh_timer_stop(p_elem);
    genie_event(GENIE_EVT_SDK_DELAY_END, p_arg);
}

void clear_trans_para(elem_state_t *p_elem)
{
    p_elem->state.trans = 0;
    p_elem->state.trans_start_time = 0;
    p_elem->state.trans_end_time = 0;
}

static void _mesh_trans_timer_cycle(void *p_timer, void *p_arg)
{
    elem_state_t *p_elem = (elem_state_t *)p_arg;
    model_state_t *p_state = &p_elem->state;

    mesh_timer_stop(p_elem);

    //do cycle
    genie_event(GENIE_EVT_SDK_TRANS_CYCLE, p_arg);
    //BT_DBG(">>>>>%d %d", (u32_t)cur_time, (u32_t)p_elem->state.trans_end_time);

    if(p_state->trans == 0) {
        genie_event(GENIE_EVT_SDK_TRANS_END, p_arg);
    }
    else {
        k_timer_start(&p_state->trans_timer, MESH_TRNSATION_CYCLE);
    }
}

uint8_t calc_cur_state(elem_state_t * p_elem)
{
    model_state_t *p_state = &p_elem->state;
    u32_t cur_time = k_uptime_get();
    uint8_t cycle = 0;

    //stop cycle when timeout
    if(cur_time <= p_state->trans_end_time - MESH_TRNSATION_CYCLE) {
#if defined(CONFIG_MESH_MODEL_LIGHTNESS_SRV) || defined(CONFIG_MESH_MODEL_CTL_SRV)
        uint16_t step = (p_state->trans_end_time - cur_time)/MESH_TRNSATION_CYCLE;
#endif

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
        if(p_state->onoff[T_CUR] == 0 && p_state->onoff[T_TAR] == 1) {
            p_state->onoff[T_CUR] = 1;
        } else if(p_state->onoff[T_CUR] == 1 && p_state->onoff[T_TAR] == 0) {
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
            //turn off when lightness is zero
            if(p_state->actual[T_CUR] == 0)
#endif
                p_state->onoff[T_CUR] = 0;
        }
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
        if(p_state->actual[T_CUR] != p_state->actual[T_TAR]) {
            if(p_state->actual[T_TAR] > p_state->actual[T_CUR]) {
                p_state->actual[T_CUR] += (p_state->actual[T_TAR]-p_state->actual[T_CUR])/step;
            } else {
                p_state->actual[T_CUR] -= (p_state->actual[T_CUR]-p_state->actual[T_TAR])/step;
            }
            //model_bind_operation(B_LIGHTNESS_ID, p_elem, T_CUR);
            cycle = 1;
        }
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
        if(p_state->temp[T_CUR] != p_state->temp[T_TAR]) {
            if(p_state->temp[T_TAR] > p_state->temp[T_CUR]) {
                p_state->temp[T_CUR] += (p_state->temp[T_TAR]-p_state->temp[T_CUR])/step;
            } else {
                p_state->temp[T_CUR] -= (p_state->temp[T_CUR]-p_state->temp[T_TAR])/step;
            }
            cycle = 1;
        }
#endif
    }
#if 0
    BT_DBG("next: %d->%d|%02x->%02x|%02x->%02x", p_state->onoff[T_CUR], p_state->onoff[T_TAR],
        p_state->actual[T_CUR], p_state->actual[T_TAR], p_state->temp[T_CUR], p_state->temp[T_TAR]);
#endif
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    if(p_state->onoff[T_CUR] == p_state->onoff[T_TAR])
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    if(p_state->actual[T_CUR] == p_state->actual[T_TAR])
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
    if(p_state->temp[T_CUR] == p_state->temp[T_TAR])
#endif
        cycle = 0;

    //BT_DBG("cycle %d", cycle);
    if(cycle == 0) {
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
        //BT_DBG("onoff %d->%d", p_state->onoff[T_CUR], p_state->onoff[T_TAR]);
        p_state->onoff[T_CUR] = p_state->onoff[T_TAR];
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
        //BT_DBG("actual %02x->%02x", p_state->actual[T_CUR], p_state->actual[T_TAR]);
        p_state->actual[T_CUR] = p_state->actual[T_TAR];
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
        //BT_DBG("temp %02x->%02x", p_state->temp[T_CUR], p_state->temp[T_TAR]);
        p_state->temp[T_CUR] = p_state->temp[T_TAR];
#endif
        p_state->trans = 0;
    }
    return cycle;
}
#endif

static void _genie_pbadv_timer_cb(void *p_timer, void *args)
{
    genie_event(GENIE_EVT_SDK_MESH_PBADV_TIMEOUT, NULL);
}

void genie_pbadv_timer_start(void)
{
    static uint8_t inited = 0;
    u32_t timeout = get_mesh_pbadv_time();

    if(!inited) {
        k_timer_init(&g_pbadv_timer, _genie_pbadv_timer_cb, NULL);
        inited = 1;
    }
    k_timer_start(&g_pbadv_timer, timeout);
}

void genie_pbadv_timer_stop(void)
{
    k_timer_stop(&g_pbadv_timer);
}

void genie_pbadv_start_silent_adv(void)
{
    genie_tri_tuple_set_silent_adv();
    bt_mesh_prov_disable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    unprov_beacon_interval_set(K_SECONDS(60));
    bt_mesh_prov_enable(BT_MESH_PROV_ADV);
}

static void _genie_prov_timer_cb(void *p_timer, void *args)
{
    genie_event(GENIE_EVT_SDK_MESH_PROV_TIMEOUT, NULL);
}

void genie_prov_timer_start(void)
{
    static uint8_t inited = 0;

    if(!inited) {
        k_timer_init(&g_prov_timer, _genie_prov_timer_cb, NULL);
        inited = 1;
    }
    k_timer_start(&g_prov_timer, MESH_PROVISIONING_TIMEOUT);
}

void genie_prov_timer_stop(void)
{
    k_timer_stop(&g_prov_timer);
}


#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
static void _poweron_indicate_cb(void *p_timer, void *args)
{
    genie_prov_timer_stop();
    genie_event(GENIE_EVT_SDK_MESH_PWRON_INDC, NULL);
}

void poweron_indicate_start(void)
{
    static uint8_t inited = 0;
    uint16_t random_time;

    if(!inited) {
        k_timer_init(&g_indc_timer, _poweron_indicate_cb, NULL);
        inited = 1;
    }
    bt_rand(&random_time, 1);
#ifdef CONFIG_MESH_MODEL_TRANS
    random_time = 2000 + 8000*random_time/255;
#else
    random_time = 500 + 9500*random_time/255;
#endif
    BT_DBG("indicate random(%d)ms", random_time);
    k_timer_start(&g_indc_timer, random_time);
}

u16_t genie_indicate_hw_reset_event (void)
{
    vnd_model_msg reply_msg;
    uint8_t payload[3] = {0};

    payload[0] = DEVICE_EVENT_T & 0xff;
    payload[1] = (DEVICE_EVENT_T >> 8) & 0xff;
    payload[2] = (uint8_t)EL_HW_RESET_T;

    reply_msg.opid = VENDOR_OP_ATTR_INDICATE;
    reply_msg.tid = 0;
    reply_msg.data = payload;
    reply_msg.len = 3;
    reply_msg.p_elem = &elements[0];
    reply_msg.retry_period = 120 + 300;
    reply_msg.retry = VENDOR_MODEL_MSG_MAX_RETRY_TIMES;

    genie_vendor_model_msg_send(&reply_msg);

    return 0;
}
#endif

u16_t genie_vnd_msg_handle(vnd_model_msg *p_msg){
    uint8_t *p_data = NULL;
    printk("vendor model message received\n");
    if (!p_msg || !p_msg->data)
        return -1;
    p_data = p_msg->data;
    printk("opcode:0x%x, tid:%d, len:%d", p_msg->opid, p_msg->tid, p_msg->len);
    if (p_data && p_msg->len)
        printk("payload: %s", bt_hex(p_data, p_msg->len));

    switch (p_msg->opid) {
        case VENDOR_OP_ATTR_INDICATE:
        {
            //u16_t attr_type = *p_data++ + (*p_data++ << 8);
            u16_t attr_type = *p_data++;
            attr_type += (*p_data++ << 8);
            if (attr_type == DEVICE_EVENT_T) {
                uint8_t event_id = *p_data;
                switch (event_id) {
                    case EL_DEV_UP_T:
                        poweron_indicate_start(); // When receiving genie's device up status, indication device's state and device up event in sequence
                        break;
                    default:
                        break;
                }
            }

            break;
        }
        default:
            break;
    }

    return 0;
}

#if 0
uint8_t vendor_indication_buff(elem_state_t *p_elem_state, uint8_t *p_buff, uint8_t len)
{
    vnd_model_msg reply_msg;

    reply_msg.opid = VENDOR_OP_ATTR_INDICATE;
    reply_msg.tid = 0;
    reply_msg.data = p_buff;
    reply_msg.len = len;
    reply_msg.p_elem = &elements[p_elem_state->elem_index];
    reply_msg.retry = VENDOR_MODEL_MSG_DFT_RETRY_TIMES;

    genie_vendor_model_msg_send(&reply_msg);
    return 0;
}
#endif

void mdoel_standart_event(elem_state_t *p_elem)
{
    E_GENIE_EVENT next_event = GENIE_EVT_SDK_ACTION_DONE;

    genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (void *)p_elem);

#ifdef CONFIG_MESH_MODEL_TRANS
    if(p_elem->state.trans || p_elem->state.delay) {
        if(p_elem->state.delay) {
            next_event = GENIE_EVT_SDK_DELAY_START;
        } else {
            next_event = GENIE_EVT_SDK_TRANS_START;
        }
    }
#endif
    genie_event(next_event, (void *)p_elem);
}

uint8_t get_seg_count(uint16_t msg_len)
{
    if(msg_len <= 11) {
        return 1;
    } else {
        msg_len -= 8;
        if(msg_len % 12) {
            return msg_len/12+2;
        } else {
            return msg_len/12+1;
        }
    }
}

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
/*
indication onoff/lightness/temperature/power on
*/
void standart_indication(elem_state_t *p_elem)
{
    uint8_t buff[14];
    uint16_t i = 0;
    uint8_t cur_indication_flag = g_indication_flag;
    uint8_t seg_count = 0;

    BT_DBG("start flag %02x", g_indication_flag);

    if(cur_indication_flag & INDICATION_FLAG_POWERON) {
        g_indication_flag &= ~INDICATION_FLAG_POWERON;
        // the device will indication all states when powerup
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
        cur_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
        cur_indication_flag |= INDICATION_FLAG_LIGHTNESS;
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
        cur_indication_flag |= INDICATION_FLAG_CTL;
#endif
    } else {
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
        if(g_indication_flag & INDICATION_FLAG_ONOFF) {
            g_indication_flag &= ~INDICATION_FLAG_ONOFF;
        }
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
        if(g_indication_flag & INDICATION_FLAG_LIGHTNESS) {
            g_indication_flag &= ~INDICATION_FLAG_LIGHTNESS;
        }
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
        if(g_indication_flag & INDICATION_FLAG_CTL) {
            g_indication_flag &= ~INDICATION_FLAG_CTL;
        }
#endif
    }
    BT_DBG("real flag %02x", cur_indication_flag);

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    if(cur_indication_flag & INDICATION_FLAG_ONOFF) {
        buff[i++] = ONOFF_T & 0xff;
        buff[i++] = (ONOFF_T >> 8) & 0xff;
        buff[i++] = p_elem->state.onoff[T_CUR];
    }
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    if(cur_indication_flag & INDICATION_FLAG_LIGHTNESS) {
        buff[i++] = LIGHTNESS_T & 0xff;
        buff[i++] = (LIGHTNESS_T >> 8) & 0xff;
        buff[i++] = p_elem->state.actual[T_CUR] & 0xff;
        buff[i++] = (p_elem->state.actual[T_CUR] >> 8) & 0xff;
    }
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
    if(cur_indication_flag & INDICATION_FLAG_CTL) {
        buff[i++] = TEMPERATURE_T & 0xff;
        buff[i++] = (TEMPERATURE_T >> 8) & 0xff;
        buff[i++] = p_elem->state.temp[T_CUR] & 0xff;
        buff[i++] = (p_elem->state.temp[T_CUR] >> 8) & 0xff;
    }
#endif
    if(cur_indication_flag & INDICATION_FLAG_POWERON) {
        buff[i++] = DEVICE_EVENT_T & 0xff;
        buff[i++] = (DEVICE_EVENT_T >> 8) & 0xff;
        buff[i++] = EL_DEV_UP_T;
        cur_indication_flag &= ~INDICATION_FLAG_POWERON;
    }
    BT_DBG("end flag %02x", g_indication_flag);

    if(i) {
        vnd_model_msg reply_msg;
        seg_count = get_seg_count(i + 4);

        reply_msg.opid = VENDOR_OP_ATTR_INDICATE;
        reply_msg.tid = 0;
        reply_msg.data = buff;
        reply_msg.len = i;
        reply_msg.p_elem = &elements[p_elem->elem_index];
        reply_msg.retry_period = 120 * seg_count + 400;
        if(seg_count > 1) {
            reply_msg.retry_period *= 2;
        }
        reply_msg.retry = VENDOR_MODEL_MSG_DFT_RETRY_TIMES;

        genie_vendor_model_msg_send(&reply_msg);
    }
}
#endif


void genie_sub_list_init(void)
{
    if(genie_flash_read_sub(g_sub_list) != GENIE_FLASH_SUCCESS) {
        mesh_sub_init(g_sub_list);
        genie_flash_write_sub(g_sub_list);
    }
    BT_DBG("");
    dump_print((char *)g_sub_list, sizeof(g_sub_list));
    //BT_DBG("0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x", g_sub_list[0], g_sub_list[1], g_sub_list[2], g_sub_list[3], g_sub_list[4], g_sub_list[5]);
}

s16_t genie_vendor_model_msg_send(vnd_model_msg *p_vendor_msg) {
    s16_t r = -1;
    uint8_t opid = 0;

    if (!p_vendor_msg)
        return r;

    opid = p_vendor_msg->opid;
    BT_DBG("opcode:0x%x, tid:%d, len:%d", opid, p_vendor_msg->tid, p_vendor_msg->len);

#if 0
    // vnedor confirm message contains 0 extra data except opid
    if (!p_vendor_msg->data && !p_vendor_msg->len)
        return r;
#endif
    BT_DBG("payload: %s", p_vendor_msg->len ? bt_hex(p_vendor_msg->data, p_vendor_msg->len) : "empty");

    switch (opid) {
        case VENDOR_OP_ATTR_STATUS:
        case VENDOR_OP_ATTR_INDICATE:
        case VENDOR_OP_ATTR_INDICATE_TG:
        case VENDOR_OP_ATTR_TRANS_MSG:
            vendor_model_msg_send(p_vendor_msg);
            break;
        default:
            BT_WARN("unknown opid:0x%x", opid);
            break;
    }
    return 0;
}

#if 0
s16_t genie_light_action_notify(elem_state_t *p_elem) {

    s16_t ret = -1;

#if 0
    if (!p_elem) {
        BT_WARN("invalid p_arg");
        ret = -1;
    } else {
        model_powerup_t *p_powerup = &((elem_state_t *)p_elem)->powerup;
        model_state_t *p_state = &((elem_state_t *)p_elem)->state;

        if (p_state->onoff[T_TAR]) {
            p_state->trans_start_time = k_uptime_get();
            p_state->trans_end_time = p_state->trans_start_time + 1000; // unit:ms
            p_powerup->last_onoff = p_state->onoff[T_CUR];
            genie_event(GENIE_EVT_SDK_TRANS_START, p_elem);
        } else {
            genie_event(GENIE_EVT_SDK_ACTION_DONE, p_elem);
            ret = 0;
        }
    }
#endif
    return ret;
}
#endif

uint8_t elem_state_init(uint8_t state_count, elem_state_t *p_elem)
{
    uint8_t i = 0;

    while(i < state_count) {
        p_elem[i].elem_index = i;
#ifdef CONFIG_MESH_MODEL_TRANS
        //memcpy(&elem[i].powerup, &f_power_up[i], sizeof(model_powerup_t));
        k_timer_init(&p_elem[i].state.delay_timer, _mesh_delay_timer_cb, &p_elem[i]);
        k_timer_init(&p_elem[i].state.trans_timer, _mesh_trans_timer_cycle, &p_elem[i]);
#endif
#ifdef CONFIG_ALI_SIMPLE_MODLE
        p_elem[i].state.onoff[T_TAR] = GEN_ONOFF_DEFAULT;
#ifdef MESH_MODEL_LIGHTNESS_SRV
        p_elem[i].state.actual[T_TAR] = LIGHTNESS_DEFAULT;
#endif
#ifdef MESH_MODEL_CTL_SRV
        p_elem[i].state.temp[T_TAR] = CTL_TEMP_DEFAULT;
#endif
#ifdef CONFIG_MESH_MODEL_TRANS
        p_elem[i].state.trans = 0x41;
        p_elem[i].state.delay = 100;
        if(p_elem[i].state.trans) {
            p_elem[i].state.trans_start_time = k_uptime_get() + p_elem[i].state.delay * 5;
            p_elem[i].state.trans_end_time = p_elem[i].state.trans_start_time + get_transition_time(p_elem[i].state.trans);
        }
#endif
#endif
        i++;
    }
    BT_DBG("+ done");
    return 0;
}

#ifdef CONFIG_MESH_MODEL_TRANS  //transation api
u16_t TRANS_TIMES[] = {1, 10, 100, 6000};

static uint8_t _get_transition_byte(u32_t time)
{
    //BT_DBG("time(%d)", time);

    time /= 100;

    if(time > TRANS_TIMES[3] * 62) {
        return 0;
    }
    else if(time > TRANS_TIMES[2] * 62) {
        return (time/TRANS_TIMES[3]) | 0xC0;
    }
    else if(time > TRANS_TIMES[1] * 62) {
        return (time/TRANS_TIMES[2]) | 0x80;
    }
    else if(time > TRANS_TIMES[0] * 62) {
        return (time/TRANS_TIMES[1]) | 0x40;
    }
    else
        return (time/TRANS_TIMES[0]);
}

//unit is 1ms
u32_t get_transition_time(uint8_t byte)
{
    if((byte & 0x3F) == 0x3F)
    {
        MODEL_E("%s ERROR, invalid 0x%02X!!!\n", __func__, byte);
        return 0xFFFFFFFF;
    }
    return (byte & 0x3F) * TRANS_TIMES[byte>>6] * 100;
}

uint8_t get_remain_byte(model_state_t *p_state, bool is_ack)
{
    uint8_t remain_byte = p_state->trans;
    u32_t cur_time = k_uptime_get();

    if (!is_ack && p_state->trans_start_time < cur_time) {
        cur_time -= p_state->trans_start_time;
        u32_t l_trans = get_transition_time(p_state->trans);
        if(l_trans == 0xFFFFFFFF) {
            remain_byte = 0x3F;
        }
        else if(l_trans > cur_time) {
            remain_byte = _get_transition_byte(l_trans - cur_time);
        }
        else {
            remain_byte = 0;
        }

    }

    //BT_DBG("remain_byte(0x%02x)", remain_byte);

    return remain_byte;
}
#endif

#define RECV_MSG_TID_QUEUE_SIZE 5

#define TMALL_GENIE_UADDR_START 0x0001
#define TMALL_GENIE_UADDR_END   0x0010

typedef struct {
    uint8_t tid;
    u16_t addr;
    u32_t time;
} _TID_QUEUE_S;

_TID_QUEUE_S tid_queue[RECV_MSG_TID_QUEUE_SIZE];

E_MESH_ERROR_TYPE mesh_check_tid(u16_t src_addr, uint8_t tid)
{
    static uint8_t cur_index = 0;
    uint8_t i = cur_index;
    uint8_t ri = 0;
    u32_t cur_time = k_uptime_get();
    u32_t end_time = 0;

    if(src_addr >= TMALL_GENIE_UADDR_START && src_addr <= TMALL_GENIE_UADDR_END) {
        src_addr = TMALL_GENIE_UADDR_START;
    }

    while(i < cur_index + RECV_MSG_TID_QUEUE_SIZE) {
        ri = i % RECV_MSG_TID_QUEUE_SIZE;
        if(tid_queue[ri].tid == tid && tid_queue[ri].addr == src_addr) {
            end_time = tid_queue[ri].time + 6000;
            if(cur_time < end_time) {
                break;
            }
            //BT_DBG("---- cur(%lld) last(%lld)", cur_time, end_time);
        }
        i++;
    }
    if(i < cur_index + RECV_MSG_TID_QUEUE_SIZE) {
        return MESH_TID_REPEAT;
    }
    else {
        tid_queue[cur_index].tid = tid;
        tid_queue[cur_index].addr = src_addr;
        tid_queue[cur_index].time = cur_time;
        cur_index++;
        cur_index %= RECV_MSG_TID_QUEUE_SIZE;
        return MESH_SUCCESS;
    }
}

#if 1   //init
static void _prov_complete(u16_t net_idx, u16_t addr)
{
    printk("Provisioning completed!\n");
    printk("Net ID: %u\n", net_idx);
    printk("Unicast addr: 0x%04x\n", addr);

    user_prov_complete(net_idx, addr);
}

static void _prov_reset(void)
{
    printk("reset provisioning\n");
    user_prov_reset();
}

static void _genie_mesh_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk(">>>Mesh initialized<<<\n");
    err = bt_mesh_init(&prov, &comp);
    if (err) {
        printk("Initializing mesh failed (err %d)\n", err);
        return;
    }

    //send event
    genie_event(GENIE_EVT_SDK_MESH_INIT, NULL);
}

void genie_mesh_init(void)
{
    int ret;

    printk("Initializing genie mesh...\n");

    genie_tri_tuple_load();

    prov.uuid = genie_tri_tuple_get_uuid();
#ifdef GENIE_OLD_AUTH
    prov.static_val = genie_tri_tuple_get_auth();
    prov.static_val_len = STATIC_OOB_LENGTH;
#endif
    prov.complete = _prov_complete;
    prov.reset = _prov_reset;

    comp.cid = CONFIG_CID_TAOBAO;
    comp.pid = 0;
    comp.vid = 1; // firmware version fir ota
    comp.elem = elements;
    comp.elem_count = get_vendor_element_num();
    hci_driver_init();

    ret = bt_enable(_genie_mesh_ready);
    if (ret) {
        printk("Bluetooth init failed (err %d)\n", ret);
    }
}
#endif
