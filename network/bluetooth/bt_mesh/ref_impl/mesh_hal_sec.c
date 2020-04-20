/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#include <errno.h>
#include <stddef.h>
#include <bluetooth.h>
#include <port/mesh_hal_sec.h>

#ifndef CONFIG_MESH_STACK_ALONE
#include <crypto.h>
#include <tinycrypt/constants.h>
#include <tinycrypt/utils.h>
#include <tinycrypt/aes.h>
#include <tinycrypt/cmac_mode.h>
#include <tinycrypt/ccm_mode.h>
#include <zephyr.h>
#include "ecc.h"

int bt_mesh_rand(void *buf, size_t len)
{
    return bt_rand(buf, len);
}

int bt_mesh_aes_encrypt(const uint8_t key[16], const uint8_t plaintext[16],
                        uint8_t enc_data[16])
{
#if BOARD_TC825X    // telink confirm later
    extern void tn_aes_128(unsigned char *key, unsigned char *plaintext, unsigned char *result);
    tn_aes_128((unsigned char *)key, (unsigned char *)plaintext, enc_data);   // use hardware AES. already disable irq inside.
    return 0;
#else
    return bt_encrypt_be(key, plaintext, enc_data);
#endif
}

int bt_mesh_aes_decrypt(const uint8_t key[16], const uint8_t enc_data[16],
                        uint8_t dec_data[16])
{
#if BOARD_TC825X    // telink confirm later
    extern void aes_ecb_decryption(uint8_t *key, uint8_t *encrypted_data, uint8_t *decrypted_data);
    aes_ecb_decryption((uint8_t *)key, (uint8_t *)enc_data, dec_data);   // use hardware AES. already disable irq inside.
    return 0;
#else
    return bt_decrypt_be(key, enc_data, dec_data);
#endif
}

int bt_mesh_aes_cmac(const uint8_t key[16], struct bt_mesh_sg *sg,
                     size_t sg_len, uint8_t mac[16])
{
#if defined(BOARD_CH6121EVB) || defined(BOARD_TG7100B)
    struct bt_cmac_t ctx;

    if (bt_cmac_setup(&ctx, key)) {
        return -EIO;
    }

    for (; sg_len; sg_len--, sg++) {
        if (bt_cmac_update(&ctx, sg->data,
                           sg->len)) {
            return -EIO;
        }
    }

    if (bt_cmac_finish(&ctx, mac)) {
        return -EIO;
    }
#else
#ifndef CONFIG_MESH_STACK_ALONE
    struct tc_aes_key_sched_struct sched;
    struct tc_cmac_struct state;

    if (tc_cmac_setup(&state, key, &sched) == TC_CRYPTO_FAIL) {
        return -EIO;
    }

    for (; sg_len; sg_len--, sg++) {
        if (tc_cmac_update(&state, sg->data,
                           sg->len) == TC_CRYPTO_FAIL) {
            return -EIO;
        }
    }

    if (tc_cmac_final(mac, &state) == TC_CRYPTO_FAIL) {
        return -EIO;
    }
#endif
#endif
    return 0;
}

const uint8_t *bt_mesh_pub_key_get(void)
{
    return bt_pub_key_get();
}

int bt_mesh_dh_key_gen(const uint8_t remote_pk[64], bt_mesh_dh_key_cb_t cb)
{
    return bt_dh_key_gen(remote_pk, cb);
}

int bt_mesh_pub_key_gen(struct bt_mesh_pub_key_cb *cb)
{
    return bt_pub_key_gen((struct bt_pub_key_cb *)cb);
}
#else
int bt_mesh_rand(void *buf, size_t len)
{
    return 0;
}

int bt_mesh_aes_encrypt(const uint8_t key[16], const uint8_t plaintext[16],
                        uint8_t enc_data[16])
{
    return 0;
}

int bt_mesh_aes_cmac(const uint8_t key[16], struct bt_mesh_sg *sg,
                     size_t sg_len, uint8_t mac[16])
{
    return 0;
}

const uint8_t *bt_mesh_pub_key_get(void)
{
    return 0;
}

int bt_mesh_dh_key_gen(const uint8_t remote_pk[64], bt_mesh_dh_key_cb_t cb)
{
    return 0;
}

int bt_mesh_pub_key_gen(struct bt_mesh_pub_key_cb *cb)
{
    return 0;
}
#endif
