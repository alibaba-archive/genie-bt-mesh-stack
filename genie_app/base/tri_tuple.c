/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <aos/aos.h>

#include <misc/printk.h>
#include <misc/byteorder.h>
#include <tinycrypt/sha256.h>
#include <tinycrypt/constants.h>
#include <port/mesh_hal_sec.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_MODEL)
#include "common/log.h"

#include "genie_app.h"
#include "tri_tuple_default.h"

#define RETURN_WHEN_ERR(result, err_code) { if(result != 0) { BT_ERR("%s[%d] error ret(%d)", __func__, __LINE__, result); return err_code; } }

#define GENIE_SIZE_PID 4
#define GENIE_SIZE_KEY 16
#define GENIE_SIZE_MAC 6
#define GENIE_SIZE_TRI_TRUPLE (GENIE_SIZE_PID+GENIE_SIZE_KEY+GENIE_SIZE_MAC)

static uint32_t g_pid;
uint8_t g_mac[GENIE_SIZE_MAC];
static uint8_t g_key[GENIE_SIZE_KEY];
static uint8_t g_auth[32];
static uint8_t g_uuid[16];
static uint8_t g_ble_key[32];

E_GENIE_FLASH_ERRCODE genie_flash_write_trituple(uint32_t *p_pid, uint8_t *p_mac,  uint8_t *p_key)
{
    //4+6+32
    uint8_t data[GENIE_SIZE_TRI_TRUPLE];

    BT_DBG("");

    memcpy(data, p_pid, GENIE_SIZE_PID);
    memcpy(data+GENIE_SIZE_PID, p_key, GENIE_SIZE_KEY);
    memcpy(data+GENIE_SIZE_PID+GENIE_SIZE_KEY, p_mac, GENIE_SIZE_MAC);

    return genie_flash_write_reliable(GFI_MESH_TRITUPLE, data, GENIE_SIZE_TRI_TRUPLE);
}

E_GENIE_FLASH_ERRCODE genie_flash_read_trituple(uint32_t *p_pid, uint8_t *p_mac,  uint8_t *p_key)
{
    E_GENIE_FLASH_ERRCODE ret;
    uint8_t data[GENIE_SIZE_TRI_TRUPLE];

    BT_DBG("");

    ret = genie_flash_read_reliable(GFI_MESH_TRITUPLE, data, GENIE_SIZE_TRI_TRUPLE);
    RETURN_WHEN_ERR(ret, ret);

    memcpy(p_pid, data, GENIE_SIZE_PID);
    memcpy(p_key, data+GENIE_SIZE_PID, GENIE_SIZE_KEY);
    memcpy(p_mac, data+GENIE_SIZE_PID+GENIE_SIZE_KEY, GENIE_SIZE_MAC);
    return GENIE_FLASH_SUCCESS;
}

void genie_tri_tuple_set_silent_adv(void)
{
    g_uuid[13] |= UNPROV_ADV_FEATURE_SILENT_ADV;
}

uint8_t *genie_tri_tuple_get_uuid(void)
{
    int i;

    // all fields in uuid should be in little-endian
    // CID: Taobao
    g_uuid[0] = 0xa8;
    g_uuid[1] = 0x01;

    // PID
    // Bit0~Bit3: 0001 (broadcast version)
    // Bit4：1 (one secret pre device)
    // Bit5: 1 (OTA support)
    // Bit6~Bit7: 01 (ble 4.0)
    g_uuid[2] = 0x71;

    // Product ID
    for (i = 0; i < 4; i++) {
        g_uuid[3 + i] = (g_pid >> (i<<3)) & 0xFF;
    }

    // mac addr (device name)
    for (i = 0; i < 6; i++) {
        g_uuid[7 + i] = g_mac[5 - i];
    }

    g_uuid[13] = UNPROV_ADV_FEATURE_AUTO_BIND_MODEL_SUB;

    g_uuid[14] = UNPROV_ADV_FLAG_GENIE_MESH_STACK;

    BT_DBG("uuid: %s\n", bt_hex(g_uuid, 16));

    return g_uuid;
}
#ifdef GENIE_OLD_AUTH
uint8_t *genie_tri_tuple_get_auth(void)
{
    int ret;
    char mac_str[(GENIE_SIZE_MAC<<1)+1] = "";
    char key_str[(GENIE_SIZE_KEY<<1)+1] = "";
    char static_str[55] = ""; // pid + ',' + mac + ',' + secret = 8+1+12+1+32+'\0'
    struct tc_sha256_state_struct sha256_ctx;

    hextostring(g_mac, mac_str, GENIE_SIZE_MAC);
    hextostring(g_key, key_str, GENIE_SIZE_KEY);

    sprintf(static_str, "%08x,%s,%s", g_pid, mac_str, key_str);

    BT_DBG("static oob: %s", static_str);

    /* calculate the sha256 of oob info and
     * fetch the top 16 bytes as static value
     */
    ret = tc_sha256_init(&sha256_ctx);
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 init fail\n");
    }

    ret = tc_sha256_update(&sha256_ctx, (const uint8_t *)static_str, strlen(static_str));
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 udpate fail\n");
    }

    ret = tc_sha256_final(g_auth, &sha256_ctx);
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 final fail\n");
    } else {
        BT_DBG("auth: %s", bt_hex((char *)g_auth, 16));
    }
    return g_auth;
}
#else
uint8_t *genie_tri_tuple_get_auth(const uint8_t random[16])
{
    int ret;
    char mac_str[(GENIE_SIZE_MAC<<1)+1] = "";
    char key_str[(GENIE_SIZE_KEY<<1)+1] = "";
    char static_str[88] = ""; // pid + ',' + mac + ',' + secret = 8+1+12+1+32+1+32'\0'
    char rad_str[33] = "";
    struct tc_sha256_state_struct sha256_ctx;

    hextostring(g_mac, mac_str, GENIE_SIZE_MAC);
    hextostring(g_key, key_str, GENIE_SIZE_KEY);
    hextostring(random, rad_str, 16);

    sprintf(static_str, "%08x,%s,%s,%s", g_pid, mac_str, key_str, rad_str);

    BT_DBG("static oob: %s", static_str);

    /* calculate the sha256 of oob info and
     * fetch the top 16 bytes as static value
     */
    ret = tc_sha256_init(&sha256_ctx);
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 init fail\n");
    }

    ret = tc_sha256_update(&sha256_ctx, (const uint8_t *)static_str, strlen(static_str));
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 udpate fail\n");
    }

    ret = tc_sha256_final(g_auth, &sha256_ctx);
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 final fail\n");
    } else {
        BT_DBG("auth: %s", bt_hex((char *)g_auth, 16));
    }
    return g_auth;
}
#endif

#if 1
void genie_ais_encrypt(const uint8_t data_in[16], uint8_t data_out[16])
{
    uint8_t aes_iv[16] = {0x31, 0x32, 0x33, 0x61, 0x71, 0x77, 0x65, 0x64,
                          0x23, 0x2a, 0x24, 0x21, 0x28, 0x34, 0x6a, 0x75};
    uint8_t local_data[16];
    uint8_t i = 0;

    memcpy(local_data, data_in, 16);
    while(i < 16) {
        local_data[i] ^= aes_iv[i];
        i++;
    }
    bt_mesh_aes_encrypt(g_ble_key, local_data, data_out);
}

void genie_ais_decrypt(const uint8_t data_in[16], uint8_t data_out[16])
{
    bt_mesh_aes_decrypt(g_ble_key, data_in, data_out);
    uint8_t aes_iv[16] = {0x31, 0x32, 0x33, 0x61, 0x71, 0x77, 0x65, 0x64,
                          0x23, 0x2a, 0x24, 0x21, 0x28, 0x34, 0x6a, 0x75};
    uint8_t i = 0;

    while(i < 16) {
        data_out[i] ^= aes_iv[i];
        i++;
    }
}

void genie_ais_get_cipher(const uint8_t random[16], uint8_t *cipher)
{
    int ret;
    char mac_str[(GENIE_SIZE_MAC<<1)+1] = "";
    char key_str[(GENIE_SIZE_KEY<<1)+1] = "";
    char static_str[72] = ""; // random + ',' + pid + ',' + mac + ',' + secret = 16+1+8+1+12+1+32'\0'
    struct tc_sha256_state_struct sha256_ctx;
    
    hextostring(g_mac, mac_str, GENIE_SIZE_MAC);
    hextostring(g_key, key_str, GENIE_SIZE_KEY);

    memcpy(static_str, random, 16);
    sprintf(static_str+16, ",%08x,%s,%s", g_pid, mac_str, key_str);

    BT_DBG("string: %s", static_str);

    /* calculate the sha256 of oob info and
     * fetch the top 16 bytes as static value
     */
    ret = tc_sha256_init(&sha256_ctx);
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 init fail\n");
    }

    ret = tc_sha256_update(&sha256_ctx, (const uint8_t *)static_str, strlen(static_str));
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 udpate fail\n");
    }

    ret = tc_sha256_final(g_ble_key, &sha256_ctx);
    if (ret != TC_CRYPTO_SUCCESS) {
        BT_ERR("sha256 final fail\n");
    } else {
        BT_DBG("auth: %s", bt_hex((char *)g_ble_key, 16));
    }
    genie_ais_encrypt(random, cipher);

    BT_DBG("cipher: %s", bt_hex((char *)cipher, 16));
}

void genie_ais_reset(void)
{
    memset(g_ble_key, 0, 32);
}

void genie_ais_adv_init(uint8_t ad_structure[14])
{
    ad_structure[3] |= 0x08; //FMSK auth enable

    memcpy(ad_structure+4, &g_pid, 4);
    memcpy(ad_structure+8, g_mac, 6);
}
#endif

uint8_t genie_tri_tuple_check_pid(uint32_t pid)
{
    return (g_pid == pid);
}

int8_t genie_tri_tuple_load(void)
{
    char l_mac[] = DEFAULT_MAC;
    char l_key[] = DEFAULT_SECRET;
    E_GENIE_FLASH_ERRCODE ret;

    ret = genie_flash_read_trituple(&g_pid, g_mac, g_key);

    if(ret != GENIE_FLASH_SUCCESS) {
        BT_ERR("read error, use default");
        g_pid = DEFAULT_PID;
        stringtohex(l_key, g_key, 16);
        stringtohex(l_mac, g_mac, 6);
        return -1;
    }

    return 0;
}

void genie_tri_tuple_show(void)
{
    uint8_t i;

    printk(F_BLUE "%d ", g_pid);

    for(i = 0; i < 16; i++) {
        printk("%02x", g_key[i]);
    }

    printk(" ");
    for(i = 0; i < 6; i++) {
        printk("%02x", g_mac[i]);
    }
    printk("\n" F_END);
}




