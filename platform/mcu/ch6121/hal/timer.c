/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "soc.h"
#include "hal/soc/timer.h"
#include "drv_timer.h"

typedef void *timer_handler_t;
static timer_handler_t  timer_handlers[CONFIG_TIMER_NUM] = {NULL};

int32_t hal_timer_init(timer_dev_t *tim)
{
    if (!tim) {
        return -1;
    }

    timer_handler_t tim_handle = NULL;
    tim_handle = csi_timer_initialize(tim->port, tim->config.cb);

    if (!tim_handle) {
        return -1;
    }

    timer_handlers[tim->port] = tim_handle;
    csi_timer_set_timeout(tim_handle, tim->config.period);

    switch (tim->config.reload_mode) {
        case (TIMER_RELOAD_AUTO):
            csi_timer_config(tim_handle, TIMER_MODE_RELOAD);
            break;

        case (TIMER_RELOAD_MANU):
            csi_timer_config(tim_handle, TIMER_MODE_FREE_RUNNING);
            break;

        default:
            csi_timer_config(tim_handle, TIMER_MODE_RELOAD);
            break;
    }

    return 0;
}

/**
 * start a hardware timer
 *
 * @param[in]  tim  timer device
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_timer_start(timer_dev_t *tim)
{
    if (!timer_handlers[tim->port]) {
        return -1;
    }

    return csi_timer_start(timer_handlers[tim->port]);
}

/**
 * stop a hardware timer
 *
 * @param[in]  tim  timer device
 *
 * @return  none
 */
void hal_timer_stop(timer_dev_t *tim)
{
    if (!timer_handlers[tim->port]) {
        return -1;
    }

    return csi_timer_stop(timer_handlers[tim->port]);
}

/**
 * De-initialises an TIMER interface, Turns off an TIMER hardware interface
 *
 * @param[in]  tim  timer device
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_timer_finalize(timer_dev_t *tim)
{
    if (!timer_handlers[tim->port]) {
        return -1;
    }
   return csi_timer_uninitialize(timer_handlers[tim->port]);
}

