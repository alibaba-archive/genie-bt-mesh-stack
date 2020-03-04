/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GEN_LEVEL_SRV_H_
#define _GEN_LEVEL_SRV_H_

#define GEN_LV_OPC_NUM 8

#define MESH_MODEL_GEN_LEVEL_SRV(_user_data) BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, \
                                                g_gen_level_op, &g_gen_level_pub, _user_data)

extern struct bt_mesh_model_pub g_gen_level_pub;
extern const struct bt_mesh_model_op g_gen_level_op[GEN_LV_OPC_NUM];


/**
 * @brief publish the model info
 * @param[in] model: the model to be published.
 */
void gen_level_publication(struct bt_mesh_model *model);

/**
 * @brief bind the levels between state and powerup
 * @param[in] elem: the element providing state and powerup
 */
void bound_level_by_onoff(elem_state_t *elem);

#endif // _GEN_LEVEL_SRV_H_
