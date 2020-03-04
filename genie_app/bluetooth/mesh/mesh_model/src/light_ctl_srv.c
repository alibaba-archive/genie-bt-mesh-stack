/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <api/mesh.h>
#include <mesh.h>
#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_MODEL)
#include "common/log.h"
#include "genie_app.h"

struct bt_mesh_model_pub g_ctl_srv_pub = {
#ifndef CONFIG_ALI_SIMPLE_MODLE
    .msg = NET_BUF_SIMPLE(2 + 9 + 4),
#endif
};

#if CONFIG_MESH_MODEL_CTL_SRV   //light ctl server
static void _ctl_prepare_buf(struct bt_mesh_model *p_model, struct net_buf_simple *p_msg, bool is_ack)
{
    model_state_t *p_state = &((elem_state_t *)p_model->user_data)->state;
    u8_t remain_byte = 0;

    BT_DBG("temp cur(0x%04x) tar(0x%04x)", p_state->temp[T_CUR], p_state->temp[T_TAR]);
#ifdef CONFIG_MESH_MODEL_TRANS
    remain_byte = get_remain_byte(p_state, is_ack);

    BT_DBG("remain(0x%02x)", remain_byte);
#endif

    //prepear buff
    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x60));
    if(is_ack && remain_byte == 0) {
        net_buf_simple_add_le16(p_msg, p_state->actual[T_TAR]);
        net_buf_simple_add_le16(p_msg, p_state->temp[T_TAR]);
    } else {
        net_buf_simple_add_le16(p_msg, p_state->actual[T_CUR]);
        net_buf_simple_add_le16(p_msg, p_state->temp[T_CUR]);

#ifdef CONFIG_MESH_MODEL_TRANS
        if(remain_byte){
            net_buf_simple_add_le16(p_msg, p_state->actual[T_TAR]);
            net_buf_simple_add_le16(p_msg, p_state->temp[T_TAR]);
            net_buf_simple_add_u8(p_msg, remain_byte);
        }
#endif
    }
}

static void _ctl_status(struct bt_mesh_model *p_model,
                       struct bt_mesh_msg_ctx *p_ctx, bool is_ack)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 9 + 4);

    BT_DBG("addr(0x%04x)", p_model->elem->addr);

    _ctl_prepare_buf(p_model, p_msg, is_ack);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send ctl Status");
    }
    BT_DBG("Success!!!");
}

static u8_t _ctl_analyze(struct bt_mesh_model *p_model,
                            u16_t src_addr, struct net_buf_simple *p_buf)
{
    u16_t lightness = 0;
    u16_t temp = 0;
    u16_t uv = 0;
    u8_t tid = 0;
    u8_t trans = 0;
    u8_t delay = 0;
    elem_state_t *p_elem = NULL;

    if (!p_model || !p_buf) return MESH_ANALYZE_ARGS_ERROR;

    p_elem = p_model->user_data;

    if(p_buf->len != 7 && p_buf->len != 9) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR p_buf->len(%d)", p_buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    //get message info
    lightness = net_buf_simple_pull_le16(p_buf);
    temp = net_buf_simple_pull_le16(p_buf);
    uv = net_buf_simple_pull_le16(p_buf);
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
    if((trans & 0x3F) == 0x3F) {
        BT_ERR("MESH_SET_TRANSTION_ERROR");
        return MESH_SET_TRANSTION_ERROR;
    }
#endif

    //check temp
    if(temp < CTL_TEMP_MIN || temp > CTL_TEMP_MAX) {
        BT_ERR("MESH_ANALYZE_ARGS_ERROR temp(0x%04x)", temp);
        return MESH_ANALYZE_ARGS_ERROR;
    }

    if(mesh_check_tid(src_addr, tid) != MESH_SUCCESS)
    {
        BT_ERR("MESH_TID_REPEAT src_addr(0x%04x) tid(0x%02x)", src_addr, tid);
        return MESH_TID_REPEAT;
    }

    p_elem->state.temp[T_TAR] = temp;
    p_elem->powerup.last_temp = p_elem->state.temp[T_TAR];
    BT_DBG("temp cur(%04x) tar(%04x)", p_elem->state.temp[T_CUR], p_elem->state.temp[T_TAR]);
#ifndef CONFIG_ALI_SIMPLE_MODLE
    p_elem->state.actual[T_TAR] = lightness;
    p_elem->state.UV[T_TAR] = uv;
    BT_DBG("actual cur(%04x) tar(%04x)", p_elem->state.actual[T_CUR], p_elem->state.actual[T_TAR]);
#endif
#ifdef CONFIG_MESH_MODEL_TRANS
    p_elem->state.trans = trans;
    p_elem->state.delay = delay;
    if(p_elem->state.trans) {
        p_elem->state.trans_start_time = k_uptime_get() + p_elem->state.delay * 5; // delay is in 5 millisecond steps
        p_elem->state.trans_end_time = p_elem->state.trans_start_time + get_transition_time(p_elem->state.trans);
    }
    BT_DBG("trans(0x%02x) delay(0x%02x)", p_elem->state.trans, p_elem->state.delay);
#endif

#ifdef CONFIG_ALI_SIMPLE_MODLE
    //only check temp when ali simaple model
    if(p_elem->state.temp[T_CUR] != p_elem->state.temp[T_TAR]) {
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
        g_indication_flag |= INDICATION_FLAG_CTL;
#endif
#ifdef CONFIG_ALI_SIMPLE_MODLE
        //only bind temp when ali_simple_model is enable
        model_bind_operation(B_LIGHT_CTL_TEMP_ID, p_elem, T_TAR);
#endif
    }
#endif

    return MESH_SUCCESS;
}

void ctl_publication(struct bt_mesh_model *p_model)
{
#ifndef CONFIG_ALI_SIMPLE_MODLE
    struct net_buf_simple *p_msg = NULL;
    int err;

    if(!p_model) return;
    p_msg = p_model->pub->msg;
    BT_DBG("addr(0x%04x)", p_model->pub->addr);

    if (p_model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
        _ctl_prepare_buf(p_model, p_msg, 0);

        err = bt_mesh_model_publish(p_model);
        if (err) {
            BT_ERR("bt_mesh_model_publish err %d\n", err);
        }
        BT_DBG("Success!!!");
    }
#endif
}

static void _ctl_get(struct bt_mesh_model *p_model,
                    struct bt_mesh_msg_ctx *p_ctx,
                    struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _ctl_status(p_model, p_ctx, 0);
}

static void _ctl_set(struct bt_mesh_model *p_model,
                    struct bt_mesh_msg_ctx *p_ctx,
                    struct net_buf_simple *p_buf)
{
    E_MESH_ERROR_TYPE ret = _ctl_analyze(p_model, p_ctx->addr, p_buf);

    BT_DBG("ret %d", ret);

    if(ret == MESH_SUCCESS || ret == MESH_TID_REPEAT) {
        _ctl_status(p_model, p_ctx, 1);
        if(ret == MESH_SUCCESS) {
            genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
        }
    }
}

static void _ctl_set_unack(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx,
                          struct net_buf_simple *p_buf)
{
    BT_DBG("");

    E_MESH_ERROR_TYPE ret = _ctl_analyze(p_model, p_ctx->addr, p_buf);

    if(ret == MESH_SUCCESS) {
        genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
    }
}

#ifndef CONFIG_ALI_SIMPLE_MODLE
static void _ctl_temp_range_status(struct bt_mesh_model *p_model, struct bt_mesh_msg_ctx *p_ctx)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 5 + 4);
    elem_state_t *p_elem = p_model->user_data;


    BT_DBG("range_status(0x%02x) min_temp(0x%04x) max_temp(0x%04x)",
            p_elem->powerup.range_status, p_elem->powerup.min_temp,
            p_elem->powerup.max_temp);

    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x63));
    net_buf_simple_add_u8(p_msg, p_elem->powerup.range_status);
    net_buf_simple_add_le16(p_msg, p_elem->powerup.min_temp);
    net_buf_simple_add_le16(p_msg, p_elem->powerup.max_temp);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send temp range Status");
    }
    BT_DBG("Success!!!");
}

static void _ctl_temp_range_get(struct bt_mesh_model *p_model,
                               struct bt_mesh_msg_ctx *p_ctx,
                               struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _ctl_temp_range_status(p_model, p_ctx);
}

static void _ctl_defatult_status(struct bt_mesh_model *p_model, struct bt_mesh_msg_ctx *p_ctx)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 6 + 4);
    elem_state_t *p_elem = p_model->user_data;


    BT_DBG("actual(0x%04x) temp(0x%04x) uv(0x%04x)",
            p_elem->powerup.default_actual, p_elem->powerup.default_temp,
            p_elem->powerup.default_UV);

    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x68));
    net_buf_simple_add_le16(p_msg, p_elem->powerup.default_actual);
    net_buf_simple_add_le16(p_msg, p_elem->powerup.default_temp);
    net_buf_simple_add_le16(p_msg, p_elem->powerup.default_UV);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send default Status");
    }
    BT_DBG("Success!!!");
}

static void _ctl_default_get(struct bt_mesh_model *p_model,
                            struct bt_mesh_msg_ctx *p_ctx,
                            struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _ctl_defatult_status(p_model, p_ctx);
}
#endif

const struct bt_mesh_model_op g_ctl_srv_op[CTL_OPC_NUM] = {
        { BT_MESH_MODEL_OP_2(0x82, 0x5d), 0, _ctl_get },
        { BT_MESH_MODEL_OP_2(0x82, 0x5e), 7, _ctl_set },
        { BT_MESH_MODEL_OP_2(0x82, 0x5f), 7, _ctl_set_unack },
#ifndef CONFIG_ALI_SIMPLE_MODLE
        { BT_MESH_MODEL_OP_2(0x82, 0x62), 0, _ctl_temp_range_get },
        { BT_MESH_MODEL_OP_2(0x82, 0x67), 0, _ctl_default_get },
#endif
        BT_MESH_MODEL_OP_END,
};
#endif

#ifndef CONFIG_ALI_SIMPLE_MODLE
#if CONFIG_MESH_MODEL_CTL_SETUP_SRV   //light ctl setup server
static E_MESH_ERROR_TYPE _ctl_default_analyze(struct bt_mesh_model *p_model, u16_t src_addr, struct net_buf_simple *p_buf)
{
    u16_t actual = 0;
    u16_t temp = 0;
    u16_t uv = 0;
    elem_state_t *p_elem = p_model->user_data;

    if(p_buf->len != 6) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR p_buf->len(%d)", p_buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    actual = net_buf_simple_pull_le16(p_buf);
    temp = net_buf_simple_pull_le16(p_buf);
    uv = net_buf_simple_pull_le16(p_buf);

    if(temp < p_elem->powerup.min_temp || temp > p_elem->powerup.max_temp) {
        BT_ERR("MESH_ANALYZE_ARGS_ERROR tar(0x%04x) min(0x%04x) max(0x%04x)",
                temp, p_elem->powerup.min_temp, p_elem->powerup.max_temp);
        return MESH_ANALYZE_ARGS_ERROR;
    }

    p_elem->powerup.default_actual = actual;
    p_elem->powerup.default_temp = temp;
    p_elem->powerup.default_UV = uv;

    BT_DBG("actual(0x%04x) temp(0x%04x) uv(0x%04x)", p_elem->powerup.default_actual,
            p_elem->powerup.default_temp, p_elem->powerup.default_UV);

    genie_event(GENIE_EVT_SDK_ANALYZE_MSG, p_elem);

    if(p_elem->state.trans || p_elem->state.delay) {
        if(p_elem->state.delay) {
            genie_event(GENIE_EVT_SDK_DELAY_START, p_elem);
        }
        else {
            genie_event(GENIE_EVT_SDK_TRANS_START, p_elem);
        }
    }
    else {
        genie_event(GENIE_EVT_SDK_ACTION_DONE, p_elem);
    }

    return MESH_SUCCESS;
}

static void _ctl_default_set(struct bt_mesh_model *p_model,
                            struct bt_mesh_msg_ctx *p_ctx,
                            struct net_buf_simple *p_buf)
{
    BT_DBG("");

    if(_ctl_default_analyze(p_model, p_ctx->addr, p_buf) == MESH_SUCCESS) {
        _ctl_defatult_status(p_model, p_ctx);
    }
}

static void _ctl_default_set_unack(struct bt_mesh_model *p_model,
                                  struct bt_mesh_msg_ctx *p_ctx,
                                  struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _ctl_default_analyze(p_model, p_ctx->addr, p_buf);
}

static E_MESH_ERROR_TYPE _ctl_temp_range_analyze(struct bt_mesh_model *p_model, u16_t src_addr, struct net_buf_simple *p_buf)
{
    u16_t min = 0;
    u16_t max = 0;
    elem_state_t *p_elem = p_model->user_data;

    if(p_buf->len != 4) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR p_buf->len(%d)", p_buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    min = net_buf_simple_pull_le16(p_buf);
    max = net_buf_simple_pull_le16(p_buf);

    if(!min || !max || min > max || min < CTL_TEMP_MIN || max > CTL_TEMP_MAX)
    {
        BT_ERR("MESH_ANALYZE_ARGS_ERROR min(0x%04x) max(0x%04x)", min, max);
        return MESH_ANALYZE_ARGS_ERROR;
    }

    p_elem->powerup.min_temp = min;
    p_elem->powerup.max_temp = max;

    BT_DBG("min_temp(0x%04x) max_temp(0x%04x)",
            p_elem->powerup.min_temp, p_elem->powerup.max_temp);

    genie_event(GENIE_EVT_SDK_ANALYZE_MSG, p_elem);

    if(p_elem->state.trans || p_elem->state.delay) {
        if(p_elem->state.delay) {
            genie_event(GENIE_EVT_SDK_DELAY_START, p_elem);
        }
        else {
            genie_event(GENIE_EVT_SDK_TRANS_START, p_elem);
        }
    }
    else {
        genie_event(GENIE_EVT_SDK_ACTION_DONE, p_elem);
    }

    return MESH_SUCCESS;
}

static void _ctl_temp_range_set(struct bt_mesh_model *p_model,
                               struct bt_mesh_msg_ctx *p_ctx,
                               struct net_buf_simple *p_buf)
{
    BT_DBG("");

    if(_ctl_temp_range_analyze(p_model, p_ctx->addr, p_buf) == MESH_SUCCESS) {
        _ctl_temp_range_status(p_model, p_ctx);
    }
}

static void _ctl_temp_range_set_unack(struct bt_mesh_model *p_model,
                                     struct bt_mesh_msg_ctx *p_ctx,
                                     struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _ctl_temp_range_analyze(p_model, p_ctx->addr, p_buf);
}

const struct bt_mesh_model_op g_ctl_setup_srv_op[CTL_SETUP_OPC_NUM] = {
        { BT_MESH_MODEL_OP_2(0x82, 0x69), 6, _ctl_default_set },
        { BT_MESH_MODEL_OP_2(0x82, 0x6A), 6, _ctl_default_set_unack },
        { BT_MESH_MODEL_OP_2(0x82, 0x6B), 4, _ctl_temp_range_set },
        { BT_MESH_MODEL_OP_2(0x82, 0x6C), 4, _ctl_temp_range_set_unack },
        BT_MESH_MODEL_OP_END,
};
#endif

#if CONFIG_MESH_MODEL_CTL_TEMPERATURE_SRV   //ctl temperatur server
static void _ctl_temp_prepear_buf(struct net_buf_simple *p_msg, bool is_ack)
{
    u8_t remain_byte = get_remain_byte(is_ack);

    BT_DBG("cur_temp(0x%04x) tar_temp(0x%04x) uv(0x%04x) uv(0x%04x) remain(0x%02x)",
            p_elem->state.temp[T_CUR], p_elem->state.temp[T_TAR],
            p_elem->state.UV[T_CUR], p_elem->state.UV[T_TAR], remain_byte);

    //prepear buff
    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x66));

    net_buf_simple_add_le16(p_msg, p_elem->state.temp[T_CUR]);
    net_buf_simple_add_le16(p_msg, p_elem->state.UV[T_CUR]);

    if(remain_byte){
        net_buf_simple_add_le16(p_msg, p_elem->state.temp[T_TAR]);
        net_buf_simple_add_le16(p_msg, p_elem->state.UV[T_TAR]);
        net_buf_simple_add_u8(p_msg, remain_byte);
    }
}

static void _ctl_temp_status(struct bt_mesh_model *p_model,
                            struct bt_mesh_msg_ctx *p_ctx, bool is_ack)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 9 + 4);

    BT_DBG("addr(0x%04x)", p_model->elem->addr);

    _ctl_temp_prepear_buf(p_msg, is_ack);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send ctl Status");
    }
    BT_DBG("Success!!!");
}

static u8_t _ctl_temp_analyze(u16_t src_addr, struct net_buf_simple *p_buf)
{
    u16_t temp = 0;
    u16_t uv = 0;
    u8_t tid = 0;
    u8_t trans = 0;
    u8_t delay = 0;
    E_MESH_ERROR_TYPE ret = MESH_SUCCESS;

    if(p_buf->len != 5 && p_buf->len != 7) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR p_buf->len(%d)", p_buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    temp = net_buf_simple_pull_le16(p_buf);
    if(temp < CTL_TEMP_MIN || temp > CTL_TEMP_MAX) {
        BT_ERR("MESH_ANALYZE_ARGS_ERROR temp(0x%04x)", temp);
        return MESH_ANALYZE_ARGS_ERROR;
    }
    if(temp < p_elem->powerup.min_temp){
        temp = p_elem->powerup.min_temp;
    }
    if(temp > p_elem->powerup.max_temp) {
        temp = p_elem->powerup.max_temp;
    }

    uv = net_buf_simple_pull_le16(p_buf);

    tid = net_buf_simple_pull_u8(p_buf);
    if(mesh_check_TID(src_addr, tid) != MESH_SUCCESS)
    {
        BT_ERR("MESH_TID_REPEAT src_addr(0x%04x) tid(0x%02x)", src_addr, tid);
        return MESH_TID_REPEAT;
    }

    if(p_buf->len) {
        trans = net_buf_simple_pull_u8(p_buf);
        delay = net_buf_simple_pull_u8(p_buf);
    }
    if((trans & 0x3F) == 0x3F) {
        BT_ERR("MESH_SET_TRANSTION_ERROR");
        return MESH_SET_TRANSTION_ERROR;
    }

    p_elem->state.temp[T_TAR] = temp;
    p_elem->state.UV[T_TAR] = uv;
    mesh_state_bound(LIGHT_CTL_TEMP, T_TAR);

    p_elem->state.trans = trans?trans:p_elem->powerup.def_trans;
    p_elem->state.delay = delay;
    if(p_elem->state.trans) {
        p_elem->state.trans_start_time = aos_now_ms() + p_elem->state.delay*5;
    }

    BT_DBG("temp(0x%04x) uv(0x%04x) trans(0x%02x) delay(0x%02x)",
            p_elem->state.temp[T_TAR], p_elem->state.UV[T_TAR],
            p_elem->state.trans, p_elem->state.delay);

    genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (void *)p_elem);

    if(mesh_state->trans || mesh_state->delay) {
        if(mesh_state->delay) {
            genie_event(GENIE_EVT_SDK_DELAY_START, (void *)p_elem);
        } else {
            genie_event(GENIE_EVT_SDK_TRANS_START, (void *)p_elem);
        }
    } else {
        genie_event(GENIE_EVT_SDK_ACTION_DONE, (void *)p_elem);
    }

    return MESH_SUCCESS;
}

static void _ctl_temp_gpio(u16_t temp)
{
    BT_DBG("0x%04x", temp);
}

static void _ctl_temp_done(void)
{
    BT_DBG("");
    p_elem->state.temp[T_CUR] = p_elem->state.temp[T_TAR];
    p_elem->state.UV[T_CUR] = p_elem->state.UV[T_TAR];
    mesh_state_bound(LIGHT_CTL_TEMP, T_CUR);
    ctl_gpio(p_elem->state.temp[T_CUR]);
}

static void _ctl_temp_transition(struct bt_mesh_model *p_model)
{
    BT_DBG("trans %d", get_transition_time(p_elem->state.trans));
    aos_msleep(get_transition_time(p_elem->state.trans));
    BT_DBG("trans end");

    p_elem->state.trans = 0;
    p_elem->state.trans_start_time = 0;

    _ctl_temp_done();
    mesh_publication(p_model->elem, MESH_PUB_CTL_TEMP);
}


static void _ctl_temp_delay(struct bt_mesh_model *p_model)
{
    BT_DBG("delay %d", p_elem->state.delay*5);
    aos_msleep(p_elem->state.delay*5);
    BT_DBG("delay end");

    p_elem->state.delay = 0;

    if(p_elem->state.trans == 0) {
        _ctl_temp_done();
        mesh_publication(p_model->elem, MESH_PUB_CTL_TEMP);
    }
    else {
        aos_schedule_call(_ctl_temp_transition, p_model);
    }
}

static bool _ctl_temp_action(struct bt_mesh_model *p_model)
{
    if(p_elem->state.trans || p_elem->state.delay) {
        if(p_elem->state.delay) {
            aos_schedule_call(_ctl_temp_delay, p_model);
        }
        else {
            aos_schedule_call(_ctl_temp_transition, p_model);
        }
        return 0;
    }
    else {
        _ctl_temp_done();
        return 1;
    }
}

void ctl_temp_publication(struct bt_mesh_model *p_model)
{
    struct net_buf_simple *p_msg = p_model->pub->msg;
    int err;

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
        _ctl_temp_prepear_buf(p_msg, 0);

        err = bt_mesh_model_publish(p_model);
        if (err) {
            BT_ERR("bt_mesh_model_publish err %d\n", err);
        }
        BT_DBG("Success!!!");
    }
}

static void _ctl_temp_get(struct bt_mesh_model *p_model,
                         struct bt_mesh_msg_ctx *p_ctx,
                         struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _ctl_temp_status(p_model, p_ctx, 0);
}

static void _ctl_temp_set(struct bt_mesh_model *p_model,
                         struct bt_mesh_msg_ctx *p_ctx,
                         struct net_buf_simple *p_buf)
{
    u8_t pub_need = 0;

    BT_DBG("");

    if(_ctl_temp_analyze(p_ctx->addr, p_buf) == MESH_SUCCESS) {
#if 0
        pub_need = _ctl_temp_action(p_model);
        _ctl_temp_status(p_model, p_ctx, 1);
        if(pub_need && p_model->pub->addr != p_ctx->addr) {
            mesh_publication(p_model->elem, MESH_PUB_CTL_TEMP);
        }
#else
        _ctl_temp_status(p_model, p_ctx, 1);
#endif
    }
}

static void _ctl_temp_set_unack(struct bt_mesh_model *p_model,
                               struct bt_mesh_msg_ctx *p_ctx,
                               struct net_buf_simple *p_buf)
{
    u8_t pub_need = 0;

    BT_DBG("");

    if(_ctl_temp_analyze(p_ctx->addr, p_buf) == MESH_SUCCESS) {
#if 0
        pub_need = _ctl_temp_action(p_model);
        if(pub_need && p_model->pub->addr != p_ctx->addr) {
            mesh_publication(p_model->elem, MESH_PUB_CTL_TEMP);
        }
#endif
    }
}

const struct bt_mesh_model_op g_ctl_temp_srv_op[CTL_TEMP_OPC_NUM] = {
        { BT_MESH_MODEL_OP_2(0x82, 0x61), 0, _ctl_temp_get },
        { BT_MESH_MODEL_OP_2(0x82, 0x64), 5, _ctl_temp_set },
        { BT_MESH_MODEL_OP_2(0x82, 0x65), 0, _ctl_temp_set_unack },
        BT_MESH_MODEL_OP_END,
};
#endif
#endif

void bind_ctl_with_onoff(elem_state_t *p_elem, E_VALUE_TYPE type)
{
    model_state_t *p_state = &p_elem->state;

    BT_DBG("onoff cur(%d) tar(%d)", p_state->onoff[T_CUR], p_state->onoff[T_TAR]);
    if(type == T_TAR) {
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
        if(p_state->onoff[T_CUR] == 0 && p_state->onoff[T_TAR] == 1) {
#ifdef CONFIG_ALI_SIMPLE_MODLE
            //turn on, check temperature
            BT_DBG("temp cur(%d) tar(%d)", p_state->temp[T_CUR], p_state->temp[T_TAR]);
            if(p_state->temp[T_CUR] != p_state->temp[T_TAR]) {
                if(p_elem->powerup.last_temp) {
                    p_state->temp[T_TAR] = p_elem->powerup.last_temp;
                } else {
                    p_state->temp[T_TAR] = CTL_TEMP_DEFAULT;
                }
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
                g_indication_flag |= INDICATION_FLAG_CTL;
#endif
            }
#endif
        }
#endif
    } else if(type == T_CUR) {
    }
    BT_DBG("temp cur(%04x) tar(%04x)", p_state->temp[T_CUR], p_state->temp[T_TAR]);
}

