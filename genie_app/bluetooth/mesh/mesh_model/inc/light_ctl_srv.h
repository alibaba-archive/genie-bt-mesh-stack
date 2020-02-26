/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _ALI_MODEL_LIGHT_CTL_SRV_H_
#define _ALI_MODEL_LIGHT_CTL_SRV_H_

#define CTL_TEMP_MIN 800
#define CTL_TEMP_MAX 20000
#define CTL_TEMP_DEFAULT CTL_TEMP_MAX

#ifdef CONFIG_ALI_SIMPLE_MODLE
#define CTL_OPC_NUM 4
#else
#define CTL_OPC_NUM 6
#define CTL_SETUP_OPC_NUM 5
#define CTL_TEMP_OPC_NUM 4
#endif

#define MESH_MODEL_CTL_SRV(_user_data) BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_CTL_SRV, \
                                            g_ctl_srv_op, &g_ctl_srv_pub, _user_data)

#ifndef CONFIG_ALI_SIMPLE_MODLE
#define MESH_MODEL_CTL_SETUP_SRV(_user_data) BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_CTL_SETUP_SRV, \
                                                g_ctl_setup_srv_op, NULL, _user_data)

#if CONFIG_MESH_MODEL_CTL_TEMPERATURE_SRV
#define MESH_MODEL_CTL_TEMP_SRV(_user_data) BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_CTL_TEMP_SRV, \
                                                g_ctl_temp_srv_op, &g_ctl_srv_pub, _user_data)
#endif
#endif

extern struct bt_mesh_model_pub g_ctl_srv_pub;
extern const struct bt_mesh_model_op g_ctl_srv_op[CTL_OPC_NUM];
#ifndef CONFIG_ALI_SIMPLE_MODLE
extern const struct bt_mesh_model_op g_ctl_setup_srv_op[CTL_SETUP_OPC_NUM];
#if CONFIG_MESH_MODEL_CTL_TEMPERATURE_SRV
extern const struct bt_mesh_model_op g_ctl_temp_srv_op[CTL_TEMP_OPC_NUM];
#endif
#endif


/**
 * @brief
 * @param[in] model
 */
void ctl_publication(struct bt_mesh_model *model);

/**
 * @brief
 * @param[in] model
 */
void ctl_temp_publication(struct bt_mesh_model *model);

#endif // _ALI_MODEL_LIGHT_CTL_SRV_H_
