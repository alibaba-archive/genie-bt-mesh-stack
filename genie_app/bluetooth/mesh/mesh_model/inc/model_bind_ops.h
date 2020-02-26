/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _MODEL_BOUND_OPERATION_H
#define _MODEL_BOUND_OPERATION_H

#include "genie_app.h"

typedef enum _BIND_OPERATION_ID_ {
/* !!!START!!! --- Don't add new ID before this ID */
    B_OPS_START_ID = -1,

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
/* Generic OnOff */
    B_GEN_ONOFF_ID = 0,
#endif

#ifdef CONFIG_MESH_MODEL_GEN_LEVEL_SRV
/* Generic Level */
    B_GEN_LEVEL_ID,
    B_GEN_DELTA_ID,
    B_GEN_MOVE_ID,
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
/* Light Lightness */
    B_LIGHTNESS_ID,
    B_LIGHTNESS_LINEAR_ID,
    B_LIGHTNESS_LAST_ID,
    B_LIGHTNESS_DFT_ID,
    B_LIGHTNESS_RANGE_ID,
#endif

#ifdef CONFIG_MESH_MODEL_CTL_SRV
/* Light CTL */
    B_LIGHT_CTL_ID,
    B_LIGHT_CTL_TEMP_ID,
    B_LIGHT_CTL_DFT_TEMP_ID,
    B_LIGHT_CTL_TEMP_RANGE_ID,
#endif

/* !!!END!!! --- Don't add new ID after this ID */
    B_OPS_END_ID

} E_BIND_OPERATION_ID;

/**
 * @brief
 * @param[in] id
 * @param[in] p_elem
 * @param[in] type
 * @return
 */
u16_t model_bind_operation(E_BIND_OPERATION_ID id, S_ELEM_STATE *p_elem, u8_t type);


#endif //_MODEL_BOUND_OPERATION_H
