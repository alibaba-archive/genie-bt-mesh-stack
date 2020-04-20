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

#ifndef DEVICE_REGISTER_H
#define DEVICE_REGISTER_H

#include <stdint.h>
#include <aos/list.h>
#include <aos/kernel.h>

#define DEVICE_PM_ACTIVE_STATE          1
#define DEVICE_PM_LOW_POWER_STATE       2
#define DEVICE_PM_SUSPEND_STATE         3
#define DEVICE_PM_OFF_STATE             4

typedef struct _dev_obj {
//    slist_t     node;
    uint8_t     id;
    uint8_t     busy;
    uint16_t    ref;
    aos_mutex_t mutex;
    void        *drv;
    void        *config;
} yoc_dev_t;

/**
  \brief       open one device
  \param[in]   name     device name
  \param[in]   id       device index to be register
  \return      null on error, device object pointer on success
*/
yoc_dev_t *device_open(const char *name);

yoc_dev_t *device_open_id(const char *name, int id);

/**
  \brief       close one device
  \param[in]   dev      device name
  \return      <0 error, >0 return device close error
*/
int device_close(yoc_dev_t *dev);

int device_is_busy(void);
void device_manage_power(int pm_state);

#endif
