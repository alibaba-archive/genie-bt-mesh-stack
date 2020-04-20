/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GENIE_MESH_H_
#define _GENIE_MESH_H_

#include <stddef.h>
#include <net/buf.h>

#include <mesh/access.h>
#include <mesh/main.h>
#include <mesh/cfg_srv.h>
#include <mesh/health_srv.h>
#include "mesh_model/inc/vendor_model.h"
#include "bluetooth/mesh/genie_mesh_flash.h"

#define CONFIG_CID_TAOBAO            0x01A8

#define CONFIG_MESH_VENDOR_COMPANY_ID CONFIG_CID_TAOBAO
#define CONFIG_MESH_VENDOR_MODEL_SRV  0x0000
#define CONFIG_MESH_VENDOR_MODEL_CLI  0x0001

#define STATIC_OOB_LENGTH            16

#define MESH_TRNSATION_CYCLE 10

#define UNPROV_ADV_FEATURE_AUTO_BIND_MODEL_SUB     0x02
#define UNPROV_ADV_FEATURE_SILENT_ADV              0x01
#define UNPROV_ADV_FLAG_GENIE_MESH_STACK           0x10     //bit4-7
#define UNPROV_ADV_FEATURE_ULTRA_PROV              0x03     //bit0-1

typedef enum {
    T_CUR = 0,
    T_TAR,
    TYPE_NUM,
} E_VALUE_TYPE;

typedef struct{
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    u8_t onoff[TYPE_NUM];
#endif

#ifdef CONFIG_MESH_MODEL_GEN_LEVEL_SRV
    s16_t level[TYPE_NUM];

    u8_t trans_id;
    u8_t trans_src;
    s16_t trans_level;
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    u16_t actual[TYPE_NUM];
#ifndef CONFIG_ALI_SIMPLE_MODLE
    u16_t linear[TYPE_NUM];
#endif
#endif

#ifdef CONFIG_MESH_MODEL_CTL_SRV
    u16_t temp[TYPE_NUM];
    u16_t UV[TYPE_NUM];
#endif

#ifdef CONFIG_MESH_MODEL_TRANS
    u8_t delay;    //unit:5ms
    u8_t trans;    //unit:100ms

    u32_t trans_start_time;
    u32_t trans_last_time;
    u32_t trans_end_time;
    s16_t actual_trans_step;
    s16_t temp_trans_step;

    struct k_timer delay_timer;
    struct k_timer trans_timer;
#endif
}  model_state_t;

typedef struct{
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    u16_t last_actual;
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
    u16_t last_temp;
#endif

#ifndef CONFIG_ALI_SIMPLE_MODLE
#ifdef CONFIG_MESH_MODEL_TRANS
    u8_t def_trans;
#endif

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
    u8_t default_onoff;
    u8_t last_onoff;
#endif

#ifndef CONFIG_ALI_SIMPLE_MODLE
    u8_t range_status;
#endif

#ifdef CONFIG_MESH_MODEL_GEN_LEVEL_SRV
    s16_t default_level;
    s16_t last_level;
#endif

#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
    u16_t default_actual;
    u16_t min_actual;
    u16_t max_actual;
#endif

#ifdef CONFIG_MESH_MODEL_CTL_SRV
    //temp
    u16_t default_temp;
    u16_t min_temp;
    u16_t max_temp;

    u16_t default_UV;
    u16_t last_UV;
#endif
#endif
}  model_powerup_t;

typedef struct{
    u8_t elem_index;
    model_state_t state;
    model_powerup_t powerup;
    void *user_data;
} elem_state_t;

#define GENIE_MAX_ELEMENT_COUNT 10

#define ALI_MODEL_TAG "\t[ALI_MODEL]"

#define MODEL_D(f, ...) printf("%d "ALI_MODEL_TAG"[D] %s "f"\n", (u32_t)aos_now_ms(), __func__, ##__VA_ARGS__)
#define MODEL_I(f, ...) printf(ALI_MODEL_TAG"[I] %s "f"\n", __func__, ##__VA_ARGS__)
#define MODEL_E(f, ...) printf(ALI_MODEL_TAG"[E] %s "f"\n", __func__, ##__VA_ARGS__)

typedef enum{
    MESH_SUCCESS = 0,
    MESH_TID_REPEAT,
    MESH_ANALYZE_SIZE_ERROR,
    MESH_ANALYZE_ARGS_ERROR,
    MESH_SET_TRANSTION_ERROR,
} E_MESH_ERROR_TYPE;

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
#define INDICATION_FLAG_POWERON 0x80
#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
#define INDICATION_FLAG_ONOFF 0x01
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
#define INDICATION_FLAG_LIGHTNESS 0x02
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
#define INDICATION_FLAG_CTL 0x04
#endif
//#define INDICATION_FLAG_VENDOR 0x08
#endif

extern uint8_t g_indication_flag;


/**
 * @brief check whether there is this tid in record, and record it if not.
 * @param[in] src_addr indicates the device which hold this tid.
 * @param[in] tid
 * @return MESH_SUCCESS means successed, otherwise failed.
 */
E_MESH_ERROR_TYPE mesh_check_tid(u16_t src_addr, u8_t tid);

/**
 * @brief get the remain bytes of message
 * @param[in] p_state: model state
 * @param[in] is_ack: ack or not
 * @return the bytes
 */
uint8_t get_remain_byte(model_state_t *p_state, bool is_ack);
#ifdef CONFIG_MESH_MODEL_TRANS

/**
 * @brief get the transition time.
 * @param[in] byte means the byte of message.
 * @return the time in milliseconds.
 */
u32_t get_transition_time(u8_t byte);
#endif

/**
 * @brief send the vendor model message
 * @param[in] p_vendor_msg refers to the message to be sent
 * @return 0 for success; negative for failure
 */
s16_t genie_vendor_model_msg_send(vnd_model_msg *p_vendor_msg);

/**
 * @brief
 * @param[in] p_elem
 * @return
 */
s16_t genie_light_action_notify(elem_state_t *p_elem);

/**
 * @brief stop the delay_timer and trans_timer for element.
 * @param[in] p_elem refers to the element.
 */
void mesh_timer_stop(elem_state_t *p_elem);

/**
 * @brief handle the vendor message
 * @param[in] p_msg refers to the message to be handled
 * @return 0 for successed, -1 for failed.
 */
u16_t genie_vnd_msg_handle(vnd_model_msg *p_msg);

/**
 * @brief
 */
void poweron_indicate_start(void);

/**
 * @brief
 */
void genie_pbadv_timer_start(void);

/**
 * @brief
 */
void genie_pbadv_timer_stop(void);

/**
 * @brief
 */
void genie_prov_timer_start(void);

/**
 * @brief
 */
void genie_prov_timer_stop(void);

/**
 * @brief
 * @param[in] p_elem
 */
void clear_trans_para(elem_state_t *p_elem);

/**
 * @brief
 * @param[in] p_elem
 * @return
 */
uint8_t calc_cur_state(elem_state_t * p_elem);

/**
 * @brief
 */
void genie_pbadv_start_silent_adv(void);

/**
 * @brief
 * @return
 */
u16_t genie_indicate_hw_reset_event (void);
uint8_t elem_state_init(uint8_t state_count, elem_state_t *p_elem);
void standart_indication(elem_state_t *p_elem);
void genie_sub_list_init(void);

#ifdef CONFIG_MESH_MODEL_GEN_ONOFF_SRV
#include "bluetooth/mesh/mesh_model/inc/gen_onoff_srv.h"
#include "bluetooth/mesh/mesh_model/inc/model_bind_ops.h"
#endif
#ifdef CONFIG_MESH_MODEL_GEN_LEVEL_SRV
#include "bluetooth/mesh/mesh_model/inc/gen_level_srv.h"
#endif
#ifdef CONFIG_MESH_MODEL_LIGHTNESS_SRV
#include "bluetooth/mesh/mesh_model/inc/lightness_srv.h"
#endif
#ifdef CONFIG_MESH_MODEL_CTL_SRV
#include "bluetooth/mesh/mesh_model/inc/light_ctl_srv.h"
#endif
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
#include "bluetooth/mesh/mesh_model/inc/vendor_model_srv.h"
#endif

#include "bluetooth/host/profile/ais_srv/ais_service.h"

void genie_mesh_init(void);

#endif /* __BT_MESH_H */
