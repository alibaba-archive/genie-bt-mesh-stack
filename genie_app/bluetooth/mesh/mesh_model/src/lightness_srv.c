/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <api/mesh.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_MODEL)

#include "common/log.h"
#include "access.h"

struct bt_mesh_model_pub g_lightness_pub = {
    .msg = NET_BUF_SIMPLE(2 + 5 + 4),
};

static void _lightness_prepare_buf(struct bt_mesh_model *p_model, struct net_buf_simple *p_msg, bool is_ack)
{
    model_state_t *p_state = &((elem_state_t *)p_model->user_data)->state;
    u8_t remain_byte = 0;

    BT_DBG("actual cur(0x%04x) tar(0x%04x)", p_state->actual[T_CUR], p_state->actual[T_TAR]);
#ifdef CONFIG_MESH_MODEL_TRANS
        remain_byte = get_remain_byte(p_state, is_ack);

        BT_DBG("remain(0x%02x)", remain_byte);
#endif

    //prepear buff
    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x4E));
    if(is_ack && remain_byte == 0) {
        net_buf_simple_add_le16(p_msg, p_state->actual[T_TAR]);
    } else {
        net_buf_simple_add_le16(p_msg, p_state->actual[T_CUR]);

#ifdef CONFIG_MESH_MODEL_TRANS
        if(remain_byte){
            net_buf_simple_add_le16(p_msg, p_state->actual[T_TAR]);
            net_buf_simple_add_u8(p_msg, remain_byte);
        }
#endif
    }
}

static void _lightness_status(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx, bool is_ack)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 5 + 4);

    BT_DBG("addr(0x%04x)", p_model->elem->addr);

    _lightness_prepare_buf(p_model, p_msg, is_ack);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send lightness Status");
    }
    BT_DBG("Success!!!");
}

static E_MESH_ERROR_TYPE _lightness_analyze(struct bt_mesh_model *p_model,
                                                u16_t src_addr, struct net_buf_simple *p_buf)
{
    s16_t lightness = 0;
    u8_t tid = 0;
    u8_t trans = 0;
    u8_t delay = 0;
    elem_state_t *p_elem = NULL;

    if (!p_model || !p_buf) return MESH_ANALYZE_ARGS_ERROR;

    p_elem = p_model->user_data;

    if(p_buf->len != 3 && p_buf->len != 5) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR buf->len(%d)", p_buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    lightness = (s16_t)net_buf_simple_pull_le16(p_buf);
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

    if(mesh_check_tid(src_addr, tid) != MESH_SUCCESS) {
        BT_ERR("MESH_TID_REPEAT src_addr(0x%04x) tid(0x%02x)", src_addr, tid);
        return MESH_TID_REPEAT;
    }

    //light_set_target_with_ctl(lightness, p_elem->state.temp[T_TAR]);

    p_elem->state.actual[T_TAR] = lightness;
    if(p_elem->state.actual[T_TAR]) {
        p_elem->powerup.last_actual = p_elem->state.actual[T_TAR];
    }
    BT_DBG("actual cur(%04x) tar(%04x)", p_elem->state.actual[T_CUR], p_elem->state.actual[T_TAR]);
#ifdef CONFIG_MESH_MODEL_TRANS
    p_elem->state.trans = trans;
    p_elem->state.delay = delay;
    if(p_elem->state.trans) {
        p_elem->state.trans_start_time = k_uptime_get() + p_elem->state.delay * 5; // delay is in 5 millisecond steps
        p_elem->state.trans_end_time = p_elem->state.trans_start_time + get_transition_time(p_elem->state.trans);
    }
    BT_DBG("trans(0x%02x) delay(0x%02x)", p_elem->state.trans, p_elem->state.delay);
#endif

    if(p_elem->state.actual[T_CUR] != p_elem->state.actual[T_TAR]) {
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
        g_indication_flag |= INDICATION_FLAG_LIGHTNESS;
#endif
        model_bind_operation(B_LIGHTNESS_ID, p_elem, T_TAR);
    }

    return MESH_SUCCESS;
}

#ifndef CONFIG_ALI_SIMPLE_MODLE
void lightness_publication(struct bt_mesh_model *p_model)
{
    struct net_buf_simple *p_msg = NULL;
    int err;

    if(!p_model) return;

    BT_DBG("addr(0x%04x)", p_model->pub->addr);

    p_msg = p_model->pub->msg;
    if (p_model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
        _lightness_prepare_buf(p_model, p_msg, 0);

        err = bt_mesh_model_publish(p_model);
        if (err) {
            BT_ERR("bt_mesh_model_publish err %d\n", err);
        }
        BT_DBG("Success!!!");
    }
}
#endif


static void _lightness_get(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx,
                          struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _lightness_status(p_model, p_ctx, 0);
}


static void _lightness_set(struct bt_mesh_model *p_model,
                          struct bt_mesh_msg_ctx *p_ctx,
                          struct net_buf_simple *p_buf)
{
    E_MESH_ERROR_TYPE ret = _lightness_analyze(p_model, p_ctx->addr, p_buf);

    BT_DBG("ret %d", ret);

    if(ret == MESH_SUCCESS || ret == MESH_TID_REPEAT) {
        _lightness_status(p_model, p_ctx, 1);
        if(ret == MESH_SUCCESS) {
            genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
        }
    }
}

static void _lightness_set_unack(struct bt_mesh_model *p_model,
                                struct bt_mesh_msg_ctx *p_ctx,
                                struct net_buf_simple *p_buf)
{
    BT_DBG("");

    E_MESH_ERROR_TYPE ret = _lightness_analyze(p_model, p_ctx->addr, p_buf);

    if(ret == MESH_SUCCESS) {
        genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
    }
}

#ifndef CONFIG_ALI_SIMPLE_MODLE
static void _lightness_linear_prepare_buf(struct bt_mesh_model *p_model, struct net_buf_simple *p_msg, bool is_ack)
{
    model_state_t *p_state = &((elem_state_t *)p_model->user_data)->state;
    u8_t remain_byte = 0;

    if (!p_model || !p_model->user_data || !p_msg) return;

    remain_byte = get_remain_byte(p_state, is_ack);

    BT_DBG("cur_linear(0x%04x) tar_linear(0x%04x) remain(0x%02x)",
            p_state->linear[T_CUR], p_state->linear[T_TAR], remain_byte);

    //prepare buff
    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x52));

    net_buf_simple_add_le16(p_msg, p_state->linear[T_CUR]);

    if(remain_byte){
        net_buf_simple_add_le16(p_msg, p_state->linear[T_TAR]);
        net_buf_simple_add_u8(p_msg, remain_byte);
    }
}

static void _lightness_linear_status(struct bt_mesh_model *p_model, struct bt_mesh_msg_ctx *ctx, u8_t is_ack)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 5 + 4);

    if (!p_model || !p_model->elem) return;

    BT_DBG("addr(0x%04x)", p_model->elem->addr);

    _lightness_linear_prepare_buf(p_model, p_msg, is_ack);

    if (bt_mesh_model_send(p_model, ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send linear Status");
    }
    BT_DBG("Success!!!");
}

static E_MESH_ERROR_TYPE _lightness_linear_analyze(struct bt_mesh_model *p_model, u16_t src_addr, struct net_buf_simple *buf)
{
    u16_t linear = 0;
    u8_t tid = 0;
    u8_t l_trans = 0;
    u8_t l_delay = 0;
    elem_state_t *p_elem = NULL;
    model_state_t *mesh_state = NULL;

    if(!p_model || !p_model->user_data || !buf) return MESH_ANALYZE_ARGS_ERROR;

    p_elem = p_model->user_data;
    mesh_state = &p_elem->state;

    if(buf->len != 3 && buf->len != 5) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR buf->len(%d)", buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    linear = net_buf_simple_pull_le16(buf);

    tid = net_buf_simple_pull_u8(buf);
    if(mesh_check_tid(src_addr, tid) != MESH_SUCCESS)
    {
        BT_ERR("MESH_TID_REPEAT src_addr(0x%04x) TID(0x%02x)", src_addr, tid);
        return MESH_TID_REPEAT;
    }

    if(buf->len) {
        l_trans = net_buf_simple_pull_u8(buf);
        l_delay = net_buf_simple_pull_u8(buf);
    }

    if((l_trans & 0x3F) == 0x3F) {
        BT_ERR("MESH_SET_TRANSTION_ERROR");
        return MESH_SET_TRANSTION_ERROR;
    }

    mesh_state->linear[T_TAR] = linear;

    //model_bind_operation(B_LIGHTNESS_LINEAR_ID, p_elem, T_TAR);

    mesh_state->trans = l_trans ? l_trans : p_elem->powerup.def_trans;
    mesh_state->delay = l_delay;
    if(mesh_state->trans) {
        mesh_state->trans_start_time = k_uptime_get_32() + mesh_state->delay * 5;
    }

    BT_DBG("tar_linear(0x%04x) trans(0x%02x) delay(0x%02x)",
        mesh_state->linear[T_TAR], mesh_state->trans, mesh_state->delay);

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

#if 0
static void lightness_linear_done(void)
{
    BT_DBG("");
    p_elem->state.linear[T_CUR] = p_elem->state.linear[T_TAR];
    mesh_state_bound(LIGHT_LIGHTNESS_LINEAR, T_CUR);
    lightness_gpio(p_elem->state.actual[T_CUR]);
}

static void lightness_linear_transition(struct bt_mesh_model *p_model)
{
    BT_DBG("trans %d", get_transition_time(p_elem->state.trans));
    aos_msleep(get_transition_time(p_elem->state.trans));
    BT_DBG("trans end");

    p_elem->state.trans = 0;
    p_elem->state.trans_start_time = 0;

    lightness_linear_done();
    //mesh_publication(p_model->p_elem, MESH_PUB_LEVEL);
}

static void lightness_linear_delay(struct bt_mesh_model *p_model)
{
    BT_DBG("delay %d", p_elem->state.delay*5);
    aos_msleep(p_elem->state.delay*5);
    BT_DBG("delay end");

    p_elem->state.delay = 0;

    if(p_elem->state.trans == 0) {
        lightness_linear_done();
        //mesh_publication(p_model->p_elem, MESH_PUB_LEVEL);
    }
    else {
        aos_schedule_call(lightness_linear_transition, p_model);
    }
}

static bool lightness_linear_action(struct bt_mesh_model *p_model)
{
    if(p_elem->state.trans || p_elem->state.delay) {
        if(p_elem->state.delay) {
            aos_schedule_call(lightness_linear_delay, p_model);
        }
        else {
            aos_schedule_call(lightness_linear_transition, p_model);
        }
        return 0;
    }
    else {
        lightness_linear_done();
        return 1;
    }
}
#endif

static void _lightness_linear_get(struct bt_mesh_model *p_model,
                                 struct bt_mesh_msg_ctx *p_ctx,
                                 struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _lightness_linear_status(p_model, p_ctx, 0);
}

static void _lightness_linear_set(struct bt_mesh_model *p_model,
                                 struct bt_mesh_msg_ctx *p_ctx,
                                 struct net_buf_simple *p_buf)
{
    E_MESH_ERROR_TYPE ret = _lightness_linear_analyze(p_model, p_ctx->addr, p_buf);

    BT_DBG("ret %d", ret);

    if(ret == MESH_SUCCESS || ret == MESH_TID_REPEAT) {
        _lightness_linear_status(p_model, p_ctx, 1);
        if(ret == MESH_SUCCESS) {
            //genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
        }
    }
}

static void _lightness_linear_set_unack(struct bt_mesh_model *p_model,
                                       struct bt_mesh_msg_ctx *p_ctx,
                                       struct net_buf_simple *p_buf)
{
    BT_DBG("");

    E_MESH_ERROR_TYPE ret = _lightness_linear_analyze(p_model, p_ctx->addr, p_buf);

    if(ret == MESH_SUCCESS) {
        //genie_event(GENIE_EVT_SDK_ANALYZE_MSG, (elem_state_t *)p_model->user_data);
    }
}

static void _lightness_last_status(struct bt_mesh_model *p_model, struct bt_mesh_msg_ctx *p_ctx)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 2 + 4);
    elem_state_t *p_elem = NULL;
    model_powerup_t *mesh_powerup = NULL;

    if (!p_model || !p_model->elem || !p_model->user_data || !p_ctx) return;

    p_elem = p_model->user_data;
    mesh_powerup = &p_elem->powerup;

    BT_DBG("addr(0x%04x) last_actual(0x%04x)", p_model->elem->addr,mesh_powerup->last_actual);

    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x54));
    net_buf_simple_add_le16(p_msg, mesh_powerup->last_actual);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send last Status");
    }
    BT_DBG("Success!!!");
}

static void _lightness_last_get(struct bt_mesh_model *p_model,
                               struct bt_mesh_msg_ctx *p_ctx,
                               struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _lightness_last_status(p_model, p_ctx);
}

static void _lightness_defatult_status(struct bt_mesh_model *p_model, struct bt_mesh_msg_ctx *p_ctx)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 2 + 4);
    elem_state_t *p_elem = NULL;
    model_powerup_t *mesh_powerup = NULL;

    if (!p_model || !p_model->user_data || !p_model->elem || !p_ctx) return;

    p_elem = p_model->user_data;
    mesh_powerup = &p_elem->powerup;

    BT_DBG("addr(0x%04x) default_actual(0x%04x)", p_model->elem->addr, mesh_powerup->default_actual);

    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x56));
    net_buf_simple_add_le16(p_msg, mesh_powerup->default_actual);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send default Status");
    }
    BT_DBG("Success!!!");
}

static void _lightness_default_get(struct bt_mesh_model *p_model,
                                  struct bt_mesh_msg_ctx *p_ctx,
                                  struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _lightness_defatult_status(p_model, p_ctx);
}

static void _lightness_range_status(struct bt_mesh_model *p_model, struct bt_mesh_msg_ctx *p_ctx)
{
    struct net_buf_simple *p_msg = NET_BUF_SIMPLE(2 + 5 + 4);
    elem_state_t *p_elem = NULL;
    model_powerup_t *mesh_powerup = NULL;

    if (!p_model || !p_model->user_data || !p_model->elem || !p_ctx) return;

    p_elem = p_model->user_data;
    mesh_powerup = &p_elem->powerup;

    BT_DBG("addr(0x%04x) min(0x%04x) max(0x%04x)", p_model->elem->addr, mesh_powerup->min_actual, mesh_powerup->max_actual);

    bt_mesh_model_msg_init(p_msg, BT_MESH_MODEL_OP_2(0x82, 0x58));
    net_buf_simple_add_u8(p_msg, mesh_powerup->range_status);
    net_buf_simple_add_le16(p_msg, mesh_powerup->min_actual);
    net_buf_simple_add_le16(p_msg, mesh_powerup->max_actual);

    if (bt_mesh_model_send(p_model, p_ctx, p_msg, NULL, NULL)) {
        BT_ERR("Unable to send range Status");
    }
    BT_DBG("Success!!!");
}

static void _lightness_range_get(struct bt_mesh_model *p_model,
                                struct bt_mesh_msg_ctx *p_ctx,
                                struct net_buf_simple *p_buf)
{
    BT_DBG("");

    _lightness_range_status(p_model, p_ctx);
}

//light lightness setup server
static E_MESH_ERROR_TYPE _lightness_default_analyze(struct bt_mesh_model *p_model, u16_t src_addr, struct net_buf_simple *buf)
{
    elem_state_t *p_elem = NULL;
    model_powerup_t *mesh_powerup = NULL;

    if (!p_model || !p_model->user_data || !buf) return MESH_ANALYZE_ARGS_ERROR;

    p_elem = p_model->user_data;
    mesh_powerup = &p_elem->powerup;

    if(buf->len != 2) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR buf->len(%d)", buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    mesh_powerup->default_actual = net_buf_simple_pull_le16(buf);

    BT_DBG("default_actual(0x%04x)", mesh_powerup->default_actual);

    //model_bind_operation(B_LIGHTNESS_DFT_ID, p_elem, T_CUR);
    return MESH_SUCCESS;
}

static void _lightness_default_set(struct bt_mesh_model *p_model,
                                        struct bt_mesh_msg_ctx *ctx,
                                        struct net_buf_simple *buf)
{
    BT_DBG("");

    if(_lightness_default_analyze(p_model, ctx->addr, buf) == MESH_SUCCESS) {
        _lightness_defatult_status(p_model, ctx);
    }
}

static void _lightness_default_set_unack(struct bt_mesh_model *p_model,
                                              struct bt_mesh_msg_ctx *ctx,
                                              struct net_buf_simple *buf)
{
    BT_DBG("");

    _lightness_default_analyze(p_model, ctx->addr, buf);
}

static E_MESH_ERROR_TYPE _lightness_range_analyze(struct bt_mesh_model *p_model, u16_t src_addr, struct net_buf_simple *buf)
{
    u16_t min = 0;
    u16_t max = 0;
    elem_state_t *p_elem = NULL;
    model_powerup_t *mesh_powerup = NULL;

    if (!p_model || !buf) return MESH_ANALYZE_ARGS_ERROR;

    p_elem = p_model->user_data;
    mesh_powerup = &p_elem->powerup;

    if(buf->len != 4) {
        BT_ERR("MESH_ANALYZE_SIZE_ERROR buf->len(%d)", buf->len);
        return MESH_ANALYZE_SIZE_ERROR;
    }

    min = net_buf_simple_pull_le16(buf);
    max = net_buf_simple_pull_le16(buf);

    if(!min || !max || min > max)
    {
        BT_ERR("MESH_ANALYZE_ARGS_ERROR min(0x%04x) max(0x%04x)", min, max);
        return MESH_ANALYZE_ARGS_ERROR;
    }

    mesh_powerup->min_actual = min;
    mesh_powerup->max_actual = max;

    BT_DBG("min_actual(0x%04x) max_actual(0x%04x)",
        mesh_powerup->min_actual, mesh_powerup->max_actual);

    //model_bind_operation(B_LIGHTNESS_RANGE_ID, p_elem, T_CUR);

    return MESH_SUCCESS;
}

static void _lightness_range_set(struct bt_mesh_model *p_model,
                                      struct bt_mesh_msg_ctx *ctx,
                                      struct net_buf_simple *buf)
{
    BT_DBG("");

    if(_lightness_range_analyze(p_model, ctx->addr, buf) == MESH_SUCCESS) {
        _lightness_range_status(p_model, ctx);
    }
}

static void _lightness_range_set_unack(struct bt_mesh_model *p_model,
                                            struct bt_mesh_msg_ctx *ctx,
                                            struct net_buf_simple *buf)
{
    BT_DBG("");

    _lightness_range_analyze(p_model, ctx->addr, buf);
}
#endif

const struct bt_mesh_model_op g_lightness_op[LIGHTNESS_OPC_NUM] = {

/* Light Lightness */
    { BT_MESH_MODEL_OP_2(0x82, 0x4b), 0, _lightness_get },
    { BT_MESH_MODEL_OP_2(0x82, 0x4c), 2, _lightness_set },        /* bind operation executed after action done in genie_mesh */
    { BT_MESH_MODEL_OP_2(0x82, 0x4d), 2, _lightness_set_unack },

#ifndef CONFIG_ALI_SIMPLE_MODLE
    { BT_MESH_MODEL_OP_2(0x82, 0x4f), 0, _lightness_linear_get },
    { BT_MESH_MODEL_OP_2(0x82, 0x50), 2, _lightness_linear_set }, /* bind operation executed immediately */
    { BT_MESH_MODEL_OP_2(0x82, 0x51), 2, _lightness_linear_set_unack },

    { BT_MESH_MODEL_OP_2(0x82, 0x53), 0, _lightness_last_get },

    { BT_MESH_MODEL_OP_2(0x82, 0x55), 0, _lightness_default_get },

    { BT_MESH_MODEL_OP_2(0x82, 0x57), 0, _lightness_range_get },

/* Light Lightness Setup */
    { BT_MESH_MODEL_OP_2(0x82, 0x59), 2, _lightness_default_set }, /* bind operation executed immediately */
    { BT_MESH_MODEL_OP_2(0x82, 0x5a), 2, _lightness_default_set_unack },
    { BT_MESH_MODEL_OP_2(0x82, 0x5b), 4, _lightness_range_set },   /* bind operation executed immediately */
    { BT_MESH_MODEL_OP_2(0x82, 0x5c), 4, _lightness_range_set_unack },
#endif
    BT_MESH_MODEL_OP_END,
};

void bind_lightness_with_onoff(elem_state_t *p_elem, E_VALUE_TYPE type)
{
    model_state_t *p_state = &p_elem->state;

    BT_DBG("onoff cur(%d) tar(%d)", p_state->onoff[T_CUR], p_state->onoff[T_TAR]);
    if(type == T_TAR) {
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
        if(p_state->onoff[T_CUR] == 0 && p_state->onoff[T_TAR] == 1) {
            //turn on, set default lightness
#ifdef CONFIG_ALI_SIMPLE_MODLE
            if(p_elem->powerup.last_actual) {
                p_state->actual[T_TAR] = p_elem->powerup.last_actual;
            } else {
                p_state->actual[T_TAR] = LIGHTNESS_DEFAULT;
            }
#endif
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_LIGHTNESS;
#endif
        } else if(p_state->onoff[T_CUR] == 1 && p_state->onoff[T_TAR] == 0) {
            //turn off, set zero
            p_state->actual[T_TAR] = 0;
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
            g_indication_flag |= INDICATION_FLAG_LIGHTNESS;
#endif
        }
#endif
    } else if(type == T_CUR) {
    }
    BT_DBG("actual cur(%04x) tar(%04x)", p_state->actual[T_CUR], p_state->actual[T_TAR]);
}

