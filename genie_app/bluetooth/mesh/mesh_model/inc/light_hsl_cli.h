/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _ALI_MODEL_LIGHT_HSL_CLI_H_
#define _ALI_MODEL_LIGHT_HSL_CLI_H_

/**
 * @brief
 * @param[in] p_elem
 * @param[in] type
 */
void bind_ctl_with_onoff(elem_state_t *p_elem, E_VALUE_TYPE type);

#define MESH_MODEL_LIGHT_HSL_CLI() BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_HSL_CLI, \
                                                NULL, NULL, NULL)

#endif // _ALI_MODEL_LIGHT_HSL_CLI_H_


