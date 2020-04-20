/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     trng.c
 * @brief    CSI Source File for TRNG Driver
 * @version  V1.0
 * @date     29. May 2019
 ******************************************************************************/

#include <csi_config.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "drv_trng.h"
#include "trng.h"
#include <hal_defs.h>
#include <clock.h>

#define ERR_TRNG(errno) (CSI_DRV_ERRNO_TRNG_BASE | errno)
#define TRNG_NULL_PARAM_CHK(para) HANDLE_PARAM_CHK(para, ERR_TRNG(DRV_ERROR_PARAMETER))

typedef struct {
    uint32_t base;
    trng_event_cb_t cb;
    trng_status_t status;
} ck_trng_priv_t;

static ck_trng_priv_t trng_handle[CONFIG_TRNG_NUM];

/* Driver Capabilities */
static const trng_capabilities_t driver_capabilities = {
    .lowper_mode = 0 /* low power mode */
};

ck_trng_reg_t *trng_reg = NULL;

extern void udelay(uint32_t us);

/**
  \brief       Initialize TRNG Interface. 1. Initializes the resources needed for the TRNG interface 2.registers event callback function
  \param[in]   idx device id
  \param[in]   cb_event  Pointer to \ref trng_event_cb_t
  \return      pointer to trng handle
*/
trng_handle_t csi_trng_initialize(int32_t idx, trng_event_cb_t cb_event)
{
    if (idx < 0 || idx >= CONFIG_TRNG_NUM) {
        return NULL;
    }

    /* obtain the trng information */
    ck_trng_priv_t *trng_priv = &trng_handle[idx];
    trng_priv->base = 0x4000c000;

    trng_reg = (ck_trng_reg_t *)(trng_priv->base);
    trng_priv->cb = cb_event;
    trng_priv->status.busy = 0;
    trng_priv->status.data_valid = 0;

    return (trng_handle_t)trng_priv;
}

/**
  \brief       De-initialize TRNG Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle  trng handle to operate.
  \return      error code
*/
int32_t csi_trng_uninitialize(trng_handle_t handle)
{
    TRNG_NULL_PARAM_CHK(handle);

    ck_trng_priv_t *trng_priv = handle;
    trng_priv->cb = NULL;

    return 0;
}

/**
  \brief       control trng power.
  \param[in]   handle  trng handle to operate.
  \param[in]   state   power state.\ref csi_power_stat_e.
  \return      error code
*/
int32_t csi_trng_power_control(trng_handle_t handle, csi_power_stat_e state)
{
    TRNG_NULL_PARAM_CHK(handle);

    if (state == DRV_POWER_FULL) {
        clk_gate_enable(MOD_RNG);
        return 0;
    } else if (state == DRV_POWER_OFF) {
        clk_gate_disable(MOD_RNG);
        return 0;
    } else {
        return ERR_TRNG(DRV_ERROR_UNSUPPORTED);
    }
}

/**
  \brief       Get driver capabilities.
  \param[in]   idx device id.
  \return      \ref trng_capabilities_t
*/
trng_capabilities_t csi_trng_get_capabilities(int32_t idx)
{
    if (idx < 0 || idx >= CONFIG_TRNG_NUM) {
        trng_capabilities_t ret;
        memset(&ret, 0, sizeof(trng_capabilities_t));
        return ret;
    }

    return driver_capabilities;
}

/**
  \brief       Get data from the TRNG.
  \param[in]   handle  trng handle to operate.
  \param[out]  data  Pointer to buffer with data get from TRNG
  \param[in]   num   Number of data items to obtain
  \return      error code
*/
int32_t csi_trng_get_data(trng_handle_t handle, void *data, uint32_t num)
{
    TRNG_NULL_PARAM_CHK(handle);
    TRNG_NULL_PARAM_CHK(data);
    TRNG_NULL_PARAM_CHK(num);

    int delay = 0;
    uint32_t rnd;
    uint8_t *buf = data;
    uint32_t len = num;

    *(volatile uint32_t *) 0x4000f048 |= 1 << 23;           // enable rng analog block
    udelay(50);
    *(volatile uint32_t *) 0x4000f048 |= 1 << 27;           // reset rng
    delay = 50;

    while (delay --);

    *(volatile uint32_t *) 0x4000f048 &= ~(1 << 27);        // disable rng reset
    udelay(50);

    for (int i = 0; i < len; i += 4) {
        *(volatile uint32_t *) 0x4000c004 |= 0x3;               // enable rng, enable rng interrupt
        udelay(50);

        delay = 50;

        while ((*(volatile uint32_t *) 0x4000c008 == 0) && delay > 0) {
            delay --;
        }

        rnd = *(volatile uint32_t *) 0x4000c000;
        *buf++ = BREAK_UINT32(rnd, 0);
        *buf++ = BREAK_UINT32(rnd, 1);
        *buf++ = BREAK_UINT32(rnd, 2);
        *buf++ = BREAK_UINT32(rnd, 3);

//        *(volatile uint32_t *) 0x4000c004 = 0x0;                // disable rng, rng interrupt
//        *(volatile uint32_t *) 0x4000c008 = 0x1;                // clear rng flag
    }

    return 0x00;
}

/**
  \brief       Get TRNG status.
  \param[in]   handle  trng handle to operate.
  \return      TRNG status \ref trng_status_t
*/
trng_status_t csi_trng_get_status(trng_handle_t handle)
{
    if (handle == NULL) {
        trng_status_t ret;
        memset(&ret, 0, sizeof(trng_status_t));
        return ret;
    }

    ck_trng_priv_t *trng_priv = handle;

    if (*(volatile uint32_t *) 0x4000c004 == 0) {
        trng_priv->status.busy = 1;
        trng_priv->status.data_valid = 0;
    } else {
        trng_priv->status.busy = 0;
        trng_priv->status.data_valid = 1;
        *(volatile uint32_t *) 0x4000c004 = 0x0;                // disable rng, rng interrupt
        *(volatile uint32_t *) 0x4000c008 = 0x1;                // clear rng flag
    }

    return trng_priv->status;
}
