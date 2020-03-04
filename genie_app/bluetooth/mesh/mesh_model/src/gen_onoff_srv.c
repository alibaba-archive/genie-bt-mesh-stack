/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <api/mesh.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_MODEL)
#include "common/log.h"

struct bt_mesh_model_pub g_gen_onoff_pub = {
#ifndef CONFIG_BT_MESH_NOPUB
    .msg = NET_BUF_SIMPLE(2 + 3 + 4),
#endif
};

static void _gen_onoff_prepear_buf(struct bt_mesh_model *p_model, struct net_buf_simple *p_msg, bool is_ack)
{
    model_state_t *p_state = &((elem_state_t *)p_model->user_data)->state;
    u8_t remain_byte = 0;

    BT_DBG("onoff cur(%d) tar(%d)", p_state->onoff[T_CUR], p_state->onoff[T_TAR]);
#ifdef CONFIG_MESH_MODEL_TRANS
    remain_byte = get_remain_byte(p_state, is_ack);

    BT_DBG("remain(0x%02x)", remain_byte);
#endif

    //prepear buff
    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x04));

    if(is_ack && remain_byte == 0) {
        net_buf_simple_add_u8(p_msg, p_state->onoff[T_TAR]);
    } else {
        net_buf_simple_add_u8(p_msg, p_state->onoff[T_CUR]);
#ifdef CONFIG_MESH_MODEL_TRANS
        if(remain_byte){
            net_buf_simple_add_u8(p_msg, p_state->onoff[T_TAR]);
            net_buf_simple_add_u8(p_msg, remain_byte);
        }
#endif
    }
}

static void _gen_onoff_status(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx, bool is_ack)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 3 + 4);

    BT_DBG("addr(0x%04x)", p_model->elem->addr);

    _gen_onoff_prepear_buf(p_model, p_msg, is_ack);

    if(bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send OnOff Status");
    }
    BT_DBG("Success!!!");
}

struct net_buf_simple *p_ack_buff;

static E_MESH_ERROR_TYPE _gen_onoff_analyze(struct bt_mesh_model *p_model,
                                            u16_t src_addr, struct net_buf_simple *p_buf)
{
    u8_t onoff = 0;
    u8_t tid = 0;
    u8_t trans = 0;
    u8_t delay = 0;
    elem_state_t *p_elem = NULL;

    if (!p_model || !p_buf) return MESH_ANALYZE_ARGS_ERROR;

    p_elem = p_model->user_data;

    if(p_buf->len != 2 && p_buf->len != 4) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR buf->len(%d)", p_buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    //get message info
    onoff = net_buf_simple_pull_u8(p_buf);
    tid = net_buf_simple_pull_u8(p_buf);
#ifdef CONFIG_MESH_MODEL_TRANS
    if(p_buf->len) {
        trans = net_buf_simple_pull_u8(p_buf);
        delay = net_buf_simple_pull_u8(p_buf);
    } else {
#ifdef CONFIG_ALI_SIMPLE_MODLE
        trans = 0;
#else
        trans = p_elem->powerup.def_trans;
#endif
        delay = 0;
    }
#endif

    if(onoff >> 1) {
        BT_ERR("MESH_ANALYZE_ARGS_ERROR onoff(0x%02x)", onoff);
        return MESH_ANALYZE_ARGS_ERROR;
    }

#ifdef CONFIG_MESH_MODEL_TRANS
    if((trans & 0x3F) == 0x3F) {
        BT_ERR("MESH_SET_TRANSTION_ERROR");
        return MESH_SET_TRANSTION_ERROR;
    }
#endif

    if(mesh_check_tid(src_addr, tid) != MESH_SUCCESS) {
        BT_ERR("MESH_TID_REPEAT src_addr(0x%04x) tid(0x%02x)", src_addr, tid);
        return MESH_TID_REPEAT;
    }
#ifndef CONFIG_ALI_SIMPLE_MODLE
    p_elem->powerup.last_onoff = p_elem->state.onoff[T_CUR];
#endif
    p_elem->state.onoff[T_TAR] = onoff;
    BT_DBG("onoff cur(%d) tar(%d)", p_elem->state.onoff[T_CUR], p_elem->state.onoff[T_TAR]);
#ifdef CONFIG_MESH_MODEL_TRANS
    p_elem->state.trans = trans;
    p_elem->state.delay = delay;
    if(p_elem->state.trans) {
        p_elem->state.trans_start_time = k_uptime_get() + p_elem->state.delay*5;
        p_elem->state.trans_end_time = p_elem->state.trans_start_time + get_transition_time(p_elem->state.trans);
    }
    BT_DBG("trans(0x%02x) delay(0x%02x)", p_elem->state.trans, p_elem->state.delay);
#endif

    if(p_elem->state.onoff[T_CUR] != p_elem->state.onoff[T_TAR]) {
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
        g_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
        model_bind_operation(B_GEN_ONOFF_ID, p_elem, T_TAR);
    }

    return MESH_SUCCESS;
}

void gen_onoff_publication(struct bt_mesh_model *p_model)
{
#ifndef CONFIG_ALI_SIMPLE_MODLE
    struct net_buf_simple *msg = p_model->pub->msg;
    int err;

    if(!p_model) return;

    BT_DBG("addr(0x%04x)", p_model->pub->addr);

    /*
     * If a server has a publish address, it is required to
     * publish status on a state change
     *
     * See Mesh Profile Specification 3.7.6.1.2
     *
     * Only publish if there is an assigned address
     */
    if (p_model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
        _gen_onoff_prepear_buf(p_model, msg, 0);

        err = bt_mesh_model_publish(p_model);
        if (err) {
            BT_ERR("bt_mesh_model_publish err %d\n", err);
        }
        BT_DBG("Success!!!");
    }
#endif
}

static void _gen_onoff_get(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx,
                          struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _gen_onoff_status(p_model, p_ctx, 0);
}

static void _gen_onoff_set(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx,
                          struct net_buf_simple *p_buf)
{
    E_MESH_ERROR_TYPE ret = _gen_onoff_analyze(p_model, p_ctx->addr, p_buf);

    BT_DBG("ret %d", ret);

    if(ret == MESH_SUCCESS || ret == MESH_TID_REPEAT) {
        _gen_onoff_status(p_model, p_ctx, 1);
        if(ret == MESH_SUCCESS) {
            genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
        }
    }
}

static void _gen_onoff_set_unack(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx,
                          struct net_buf_simple *p_buf)
{
    BT_DBG("");

    E_MESH_ERROR_TYPE ret = _gen_onoff_analyze(p_model, p_ctx->addr, p_buf);

    if(ret == MESH_SUCCESS) {
        genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
    }
}

const struct bt_mesh_model_op g_gen_onoff_op[] = {
    { BT_MESH_MODEL_OP_2(0x82, 0x01), 0, _gen_onoff_get },
    { BT_MESH_MODEL_OP_2(0x82, 0x02), 2, _gen_onoff_set },
    { BT_MESH_MODEL_OP_2(0x82, 0x03), 2, _gen_onoff_set_unack },
    BT_MESH_MODEL_OP_END,
};

#ifdef CONFIG_MESH_MODEL_CTL_SRV
void bind_onoff_with_ctl(elem_state_t *p_elem, E_VALUE_TYPE type)
{
    model_state_t *p_state = &p_elem->state;

    if(type == T_TAR) {
        if(p_state->onoff[T_TAR] == 0) {
            //turn on
            p_state->onoff[T_TAR] = 1;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
            model_bind_operation(B_GEN_ONOFF_ID, p_elem, T_TAR);
        }
    } else if(type == T_CUR) {
    }
}
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
void bind_onoff_with_lightness(elem_state_t *p_elem, E_VALUE_TYPE type)
{
    model_state_t *p_state = &p_elem->state;

    if(type == T_TAR) {
        if(p_state->actual[T_CUR] == 0 && p_state->actual[T_TAR] != 0) {
            //turn on
            p_state->onoff[T_TAR] = 1;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
        } else if(p_state->actual[T_CUR] != 1 && p_state->actual[T_TAR] == 0) {
            //turn off
            p_state->onoff[T_TAR] = 0;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
        }
    } else if(type == T_CUR) {
        if(p_state->actual[T_CUR] == 0 && p_state->actual[T_TAR] == 0) {
            //turn off
            p_state->onoff[T_CUR] = 0;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_ONOFF;
#endif
        }
    }
}
#endif


