/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include "genie_app.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_MODEL)
#include "common/log.h"

struct k_timer g_genie_reset_timer;
//1:need reset 0:no reset
uint8_t g_genie_reset_flag = 0;
uint8_t hal_led_ctrl(bool onoff);

uint8_t genie_reset_get_flag(void)
{
    return g_genie_reset_flag;
}

void _genie_reset_set_flag(uint8_t flag)
{
    g_genie_reset_flag = flag;
}

void _genie_reset_done_cb(void *p_timer, void *args)
{
    genie_event(GENIE_EVT_HW_RESET_DONE, NULL);
}

void genie_reset_done(void)
{
    k_timer_init(&g_genie_reset_timer, _genie_reset_done_cb, NULL);
    k_timer_start(&g_genie_reset_timer, GENIE_RESET_WAIT_TIMEOUT);
}

static E_GENIE_FLASH_ERRCODE _genie_reset_write_count(uint8_t count)
{
    uint8_t data = count;

    BT_DBG("%d", count);

    return genie_flash_write_userdata(GFI_MESH_RESET_CNT, &data, 1);
}

static E_GENIE_FLASH_ERRCODE _genie_reset_read_count(uint8_t *p_count)
{
    E_GENIE_FLASH_ERRCODE ret;

    ret = genie_flash_read_userdata(GFI_MESH_RESET_CNT, p_count, 1);

    if(ret != GENIE_FLASH_SUCCESS) {
        *p_count = 0;
        BT_ERR("read error %d", ret);
    }

    return ret;
}

static void _genie_reset_timer_cb(void *p_timer, void *args)
{
    uint8_t number = 0;

    BT_DBG("");
    _genie_reset_write_count(number);
}

void genie_reset_clean_count(void)
{
    _genie_reset_write_count(0);
    _genie_reset_set_flag(0);
}

void genie_reset_by_repeat_init(void)
{
    uint8_t count;
    E_GENIE_FLASH_ERRCODE flash_err;

    /* we should check flash flag first */
    flash_err = _genie_reset_read_count(&count);
    if (flash_err != GENIE_FLASH_SUCCESS) {
        count = 0;
    }

#if 0
    if (number == GENIE_RESET_BY_REPEAT_COUNTER) {
        BT_DBG("Genie Event Reset By Repeat Notify");
        genie_event(GENIE_EVT_RESET_BY_REPEAT_NOTIFY, &by_boot);
        number = 0;
    }
#endif

    BT_DBG("read count %d", count);

    /* update number and check if exceeded */
    if (count > GENIE_RESET_BY_REPEAT_COUNTER) {
        count = 0;
    }
    if (count < GENIE_RESET_BY_REPEAT_COUNTER) {
        count++;
        _genie_reset_write_count(count);
        k_timer_init(&g_genie_reset_timer, _genie_reset_timer_cb, NULL);
        k_timer_start(&g_genie_reset_timer, GENIE_RESET_BY_REPEAT_TIMEOUT);
    } else {
        //genie_event(GENIE_EVT_HW_RESET, NULL);
        _genie_reset_set_flag(1);
    }
}

