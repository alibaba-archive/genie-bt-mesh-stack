/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#ifndef _AIS_SERVICE_H_
#define _AIS_SERVICE_H_

#include <misc/byteorder.h>
#include <gatt.h>

/**
 * @brief the ais service action when gatt is disconnected.
 * @param[in] the disconnect reason.
 */
void ais_ota_disconnect(uint8_t reason);

/**
 * @brief the ais service action when gatt is connected.
 * @param[in] the handle of connect.
 */
void ais_connect(struct bt_conn *p_conn);

/**
 * @brief regist the ais service.
 * @return the result of registion.
 */
int ais_service_register(void);

/**
 * @brief check the ota status, the device will finish ota and reboot.
 */
void ais_check_ota_change(void);

#endif

