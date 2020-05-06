/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GENIE_MESH_FLASH_H_
#define _GENIE_MESH_FLASH_H_

#include <genie_app.h>

typedef struct{
    uint16_t dst;
    uint8_t count;
    uint8_t period;
    uint8_t ttl;
    uint16_t feat;
    uint16_t net_idx;
} mesh_hb_para_t;

typedef struct{
    uint16_t net_index;
    uint8_t flag;
    uint32_t ivi;
    uint8_t key[16];
} mesh_netkey_para_t;

typedef struct{
    uint16_t net_index;
    uint16_t key_index;
    uint8_t flag;
    uint8_t key[16];
} mesh_appkey_para_t;

E_GENIE_FLASH_ERRCODE genie_flash_write_trituple(uint32_t *p_pid, uint8_t *p_mac,  uint8_t *p_key);
E_GENIE_FLASH_ERRCODE genie_flash_read_trituple(uint32_t *p_pid, uint8_t *p_mac,  uint8_t *p_key);

/**
 * @brief save the subscription address to flash
 * @param[in] p_sub: the list of subscription address
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_sub(uint16_t *p_sub);

/**
 * @brief read the subscription address from flash
 * @param[out] p_sub: the list of subscription address
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_sub(uint16_t *p_sub);

/**
 * @brief save the parameters for mesh to flash
 * @param[in] mesh_data: the parameters for mesh
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_para(struct bt_mesh_net *mesh_data);

/**
 * @brief read the parameters for mesh from flash
 * @param[in] mesh_data: the parameters for mesh
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_para(struct bt_mesh_net *mesh_data);

/**
 * @brief save the address of peer mesh devices to flash
 * @param[in] p_addr: address
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_addr(uint16_t *p_addr);

/**
 * @brief read the address of peer mesh devices from flash
 * @param[out] p_addr: address
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_addr(uint16_t *p_addr);

/**
 * @brief write the heartbeat parameters to flash
 * @param[in] p_para: heartbeat parameters
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_hb(mesh_hb_para_t *p_para);

/**
 * @brief read the heartbeat parameters from flash
 * @param[out] p_para: heartbeat parameters
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_hb(mesh_hb_para_t *p_para);

/**
 * @brief save the devices key to flash
 * @param[in] status: devcie key
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_devkey(uint8_t *p_devkey);

/**
 * @brief read the devices key from flash
 * @param[out] status: devcie key
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_devkey(uint8_t *p_devkey);

/**
 * @brief save the net key to flash
 * @param[in] status: net key
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_netkey(mesh_netkey_para_t *p_netkey);

/**
 * @brief read the net key from flash
 * @param[out] status: net key
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_netkey(mesh_netkey_para_t *p_netkey);

/**
 * @brief save the app key to flash
 * @param[in] status: app key
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_appkey(mesh_appkey_para_t *p_appkey);

/**
 * @brief read the app key from flash
 * @param[out] status: app key
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_appkey(mesh_appkey_para_t *p_appkey);


/**
 * @brief save the seq number to flash
 * @param[in] status: seq number
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_seq(uint32_t *p_seq);

/**
 * @brief read the seq number from flash
 * @param[out] status: seq number
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_seq(uint32_t *p_seq);

/**
 * @brief erase everyting on flash except the trituple info
 * @return the status of operation, 0 means successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_reset_system(void);

#endif/* _GENIE_MESH_FLASH_H_ */
