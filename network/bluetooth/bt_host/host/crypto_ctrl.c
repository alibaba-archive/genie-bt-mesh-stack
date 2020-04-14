/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <errno.h>

#include <zephyr.h>
#include <misc/byteorder.h>

#include <bluetooth.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/aes.h>

#define BT_DBG_ENABLED 1
#include "common/log.h"

#include "hci_api.h"

int bt_rand(void *buf, size_t len)
{
    uint8_t tmp[8];
    uint8_t *pdata = buf;

    if (buf == NULL || len <= 0) {
        return -EIO;
    }

    while (len > 8) {
        hci_api_le_rand(pdata);
        len -= 8;
        pdata += 8;
    }

    if (len) {
        hci_api_le_rand(tmp);
        memcpy(pdata, tmp, len);
    }

    return 0;
}

int bt_encrypt_le(const u8_t key[16], const u8_t plaintext[16],
                  u8_t enc_data[16])
{
    int err;
    u8_t tmp_key[16];
    u8_t tmp_plaintext[16];
    BT_DBG("key %s plaintext %s", bt_hex(key, 16), bt_hex(plaintext, 16));

    sys_memcpy_swap(tmp_key, key, 16);

    sys_memcpy_swap(tmp_plaintext, plaintext, 16);

    err = hci_api_le_enc((uint8_t *)tmp_key, (uint8_t *)tmp_plaintext, (uint8_t *)enc_data);

    if (err) {
        BT_ERR("enc le fail %d", err);
        return err;
    }

    sys_mem_swap(enc_data, 16);

    BT_DBG("enc_data %s", bt_hex(enc_data, 16));

    return 0;
}

int bt_encrypt_be(const u8_t key[16], const u8_t plaintext[16],
                  u8_t enc_data[16])
{
    int err;
    u8_t tmp_key[16];
    u8_t tmp_plaintext[16];
    memcpy(tmp_key, key, 16);
    memcpy(tmp_plaintext, plaintext, 16);
    err = hci_api_le_enc((uint8_t *)tmp_key, (uint8_t *)tmp_plaintext, (uint8_t *)enc_data);

    if (err) {
        BT_ERR("enc le fail %d", err);
        return err;
    }

    return 0;
}

int bt_decrypt_be(const u8_t key[16], const u8_t enc_data[16],
		  u8_t dec_data[16])
{
    struct tc_aes_key_sched_struct s;

    if (tc_aes128_set_decrypt_key(&s, key) == TC_CRYPTO_FAIL) {
        return -EINVAL;
    }

    if (tc_aes_decrypt(dec_data, enc_data, &s) == TC_CRYPTO_FAIL) {
        return -EINVAL;
    }

    BT_DBG("dec_data %s", bt_hex(dec_data, 16));

    return 0;
}



/* bt_cmac_* functions is modified from tinycrypt cmac_mode.c. the aes function is replaced
   by bt_encrypt_be which call aes ctypto from controller.
*/

/*
 *  Copyright (C) 2017 by Intel Corporation, All Rights Reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    - Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *    - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    - Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  gf_wrap -- In our implementation, GF(2^128) is represented as a 16 byte
 *  array with byte 0 the most significant and byte 15 the least significant.
 *  High bit carry reduction is based on the primitive polynomial
 *
 *                     X^128 + X^7 + X^2 + X + 1,
 *
 *  which leads to the reduction formula X^128 = X^7 + X^2 + X + 1. Indeed,
 *  since 0 = (X^128 + X^7 + X^2 + 1) mod (X^128 + X^7 + X^2 + X + 1) and since
 *  addition of polynomials with coefficients in Z/Z(2) is just XOR, we can
 *  add X^128 to both sides to get
 *
 *       X^128 = (X^7 + X^2 + X + 1) mod (X^128 + X^7 + X^2 + X + 1)
 *
 *  and the coefficients of the polynomial on the right hand side form the
 *  string 1000 0111 = 0x87, which is the value of gf_wrap.
 *
 *  This gets used in the following way. Doubling in GF(2^128) is just a left
 *  shift by 1 bit, except when the most significant bit is 1. In the latter
 *  case, the relation X^128 = X^7 + X^2 + X + 1 says that the high order bit
 *  that overflows beyond 128 bits can be replaced by addition of
 *  X^7 + X^2 + X + 1 <--> 0x87 to the low order 128 bits. Since addition
 *  in GF(2^128) is represented by XOR, we therefore only have to XOR 0x87
 *  into the low order byte after a left shift when the starting high order
 *  bit is 1.
 */
static const unsigned char gf_wrap = 0x87;

/*
 *  assumes: out != NULL and points to a GF(2^n) value to receive the
 *            doubled value;
 *           in != NULL and points to a 16 byte GF(2^n) value
 *            to double;
 *           the in and out buffers do not overlap.
 *  effects: doubles the GF(2^n) value pointed to by "in" and places
 *           the result in the GF(2^n) value pointed to by "out."
 */
static void gf_double(uint8_t *out, uint8_t *in)
{

    /* start with low order byte */
    uint8_t *x = in + (16 - 1);

    /* if msb == 1, we need to add the gf_wrap value, otherwise add 0 */
    uint8_t carry = (in[0] >> 7) ? gf_wrap : 0;

    out += (16 - 1);

    for (;;) {
        *out-- = (*x << 1) ^ carry;

        if (x == in) {
            break;
        }

        carry = *x-- >> 7;
    }
}

int bt_cmac_setup(struct bt_cmac_t *ctx, const uint8_t key[16])
{
    if (NULL == ctx || NULL == key) {
        return -EINVAL;
    }

    memset(ctx, 0, sizeof(struct bt_cmac_t));
    memcpy(ctx->aes_key, key, 16);

    memset(ctx->iv, 0, 16);
    bt_encrypt_be(ctx->aes_key, ctx->iv, ctx->iv);

    gf_double(ctx->K1, ctx->iv);
    gf_double(ctx->K2, ctx->K1);

    memset(ctx->iv, 0, 16);
    memset(ctx->leftover, 0, 16);
    ctx->leftover_offset = 0;

    ctx->countdown = ((uint64_t)1 << 48);

    return 0;
}

int bt_cmac_update(struct bt_cmac_t *ctx, const uint8_t *data, uint16_t data_length)
{
    unsigned int i;

    /* input sanity check: */
    if (ctx == NULL) {
        return -EINVAL;
    }

    if (data_length == 0) {
        return 0;
    }

    if (data == NULL) {
        return -EINVAL;
    }

    if (ctx->countdown == 0) {
        return -EIO;
    }

    ctx->countdown--;

    if (ctx->leftover_offset > 0) {
        /* last data added to s didn't end on a 16 byte boundary */
        size_t remaining_space = 16 - ctx->leftover_offset;

        if (data_length < remaining_space) {
            /* still not enough data to encrypt this time either */
            memcpy(&ctx->leftover[ctx->leftover_offset], data, data_length);
            ctx->leftover_offset += data_length;
            return 0;
        }

        /* leftover block is now full; encrypt it first */
        memcpy(&ctx->leftover[ctx->leftover_offset],
               data,
               remaining_space);
        data_length -= remaining_space;
        data += remaining_space;
        ctx->leftover_offset = 0;

        for (i = 0; i < 16; ++i) {
            ctx->iv[i] ^= ctx->leftover[i];
        }

        bt_encrypt_be(ctx->aes_key, ctx->iv, ctx->iv);
    }

    /* CBC encrypt each (except the last) of the data blocks */
    while (data_length > 16) {
        for (i = 0; i < 16; ++i) {
            ctx->iv[i] ^= data[i];
        }

        bt_encrypt_be(ctx->aes_key, ctx->iv, ctx->iv);
        data += 16;
        data_length  -= 16;
    }

    if (data_length > 0) {
        /* save leftover data for next time */
        memcpy(ctx->leftover, data, data_length);
        ctx->leftover_offset = data_length;
    }

    return 0;
}

int bt_cmac_finish(struct bt_cmac_t *ctx, uint8_t out[16])
{
    uint8_t *k;
    unsigned int i;

    if (ctx == NULL || out == NULL) {
        return -EINVAL;
    }

    if (ctx->leftover_offset == 16) {
        /* the last message block is a full-sized block */
        k = (uint8_t *) ctx->K1;
    } else {
        /* the final message block is not a full-sized  block */
        size_t remaining = 16 - ctx->leftover_offset;

        memset(&ctx->leftover[ctx->leftover_offset], 0, remaining);
        ctx->leftover[ctx->leftover_offset] = 0x80;
        k = (uint8_t *) ctx->K2;
    }

    for (i = 0; i < 16; ++i) {
        ctx->iv[i] ^= ctx->leftover[i] ^ k[i];
    }

    bt_encrypt_be(ctx->aes_key, ctx->iv, out);

    memset(ctx, 0, sizeof(*ctx));

    return 0;
}
