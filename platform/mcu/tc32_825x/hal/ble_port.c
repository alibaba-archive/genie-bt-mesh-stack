/********************************************************************************************************
 * @file     ble_port.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#if 0
#ifndef CONFIG_MESH_STACK_ALONE
#include <bluetooth.h>
#include <storage.h>
#include <hci.h>
#endif

#include <aos/kernel.h>
#include <flash.h>
#include <aos/errno.h>

#include "ble_port.h"

#ifndef AOS_COMP_KV
#define MAX_REMOTE_DEV_SIZE (5)
#ifndef CONFIG_MESH_STACK_ALONE
typedef struct{
    uint8_t mac[6];
    bt_addr_le_t local_mac;
    uint8_t local_IRK[16];
    bt_addr_le_t remote_dev[MAX_REMOTE_DEV_SIZE];
    uint8_t remote_IRK[16];
    struct bt_storage_ltk LTK;
    struct bt_storage_ltk slave_LTK;
} bt_storage_t;
#endif
#endif

#ifndef CONFIG_MESH_STACK_ALONE
static ssize_t storage_read(const bt_addr_le_t *addr, u16_t key, void *data,
                            size_t length)
{
    uint8_t mac[6];
    int err_code;
    uint8_t s[20];
    sprintf(s ,"BT_STORAGE_%02x", key);
#ifdef AOS_COMP_KV
     err_code = aos_kv_get(s, data, length, 1);
#else
    unsigned int off = 0;
    bt_storage_t local_storage;
    memset(&local_storage, 0, sizeof(bt_storage_t));
    err_code = hal_flash_read(HAL_PARTITION_PARAMETER_3, &off, &local_storage, sizeof(bt_storage_t));
    if(!err_code){
        memcpy(mac, (uint8_t*)local_storage.local_mac.a.val, sizeof(mac));
    }
#endif

    if(err_code == 0){
        if(BT_STORAGE_ID_ADDR == key){
            uint8_t mac[6];
            memcpy(mac ,((bt_addr_le_t *)data)->a.val, 6);
            printf("%s: valid mac read - 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\r\n",
                       __func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            return sizeof(bt_addr_le_t);
       }
    } else if(err_code == -ENOENT){
        if(BT_STORAGE_ID_ADDR == key){
            printf("%s: no valid mac read\r\n", __func__);
            return 0;
	}
    } else{
        printf("KV read failed(%d)\n", err_code);
        return -1;
    }
}

static ssize_t storage_write(const bt_addr_le_t *addr, u16_t key,
                             const void *data, size_t length)
{
    uint32_t err_code;
    uint8_t mac[6];
    uint8_t s[20];

    sprintf(s ,"BT_STORAGE_%02x", key);
#ifdef AOS_COMP_KV
    err_code = aos_kv_set(s, data, length, 1);
#else
    unsigned int off = 0;
    bt_storage_t local_storage;
    err_code = hal_flash_read(HAL_PARTITION_PARAMETER_3, &off, &local_storage, sizeof(bt_storage_t));
    if(memcmp(mac, (uint8_t*)local_storage.local_mac.a.val, sizeof(mac)) == 0){
        printf("No need to store MAC\n");
        return sizeof(bt_addr_le_t);
    }

    if(!err_code){
        memcpy( (uint8_t*)local_storage.local_mac.a.val, mac,sizeof(mac));
        err_code = hal_flash_erase(HAL_PARTITION_PARAMETER_3, &off, 4096);
	if(!err_code){
	    printf("Flash erase failed\n");
	    return -1;
	}
    }
    err_code = hal_flash_erase_write(HAL_PARTITION_PARAMETER_3, &off, &local_storage, sizeof(bt_storage_t));
#endif

    if (err_code != 0) {
        printf("%s failed.\r\n", __func__);
        return 0;
    }

    return sizeof(bt_addr_le_t);
}

static int storage_clear(const bt_addr_le_t *addr)
{
    return 0;
}
#endif

int ble_storage_init(void)
{
#ifndef CONFIG_MESH_STACK_ALONE
        static const struct bt_storage storage = {
                .read  = storage_read,
                .write = storage_write,
                .clear = storage_clear
        };
        //TBD:should check lower flash APIs
        bt_storage_register(&storage);
#endif

        return 0;
}
#endif
