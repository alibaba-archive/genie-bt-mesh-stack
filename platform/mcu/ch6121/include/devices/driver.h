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

#ifndef DEVICE_DRIVER_H
#define DEVICE_DRIVER_H

#include <stdint.h>
#include <string.h>

#include <aos/list.h>
#include <aos/log.h>
#include <aos/debug.h>

#include "device.h"

typedef struct _driver_obj driver_t;

struct _driver_obj {
    slist_t node;
    char   *name;
    char   *type;
    int16_t ref;
    int16_t device_id;

    yoc_dev_t * (*init)(driver_t *drv, void *config, int id);
    void    (*uninit)(yoc_dev_t *dev);
    int     (*lpm)(yoc_dev_t *dev, int state);
    int     (*open)(yoc_dev_t *dev);
    int     (*close)(yoc_dev_t *dev);
};

#define DRIVER(dev)  ((driver_t*)(dev->drv))

/**
  \brief       register device object into device driver framework.
  \param[in]   drv      device driver object
  \param[in]   count    device number
  \return      <0 error, >0 return dev count.
*/
int driver_register(driver_t *drv, void *config, int idx);

/**
  \brief       remove device object from device driver framework.
  \param[in]   name     device name
  \return      <0 error, >0 return operation result
*/
int driver_unregister(const char *name);

/**
  \brief       get device object by device name
  \param[in]   name     device name
  \param[in]   id       device index
  \return      null on error, device object pointer on success
*/
yoc_dev_t *device_find(const char *name, int id);

/**
  \brief       allocate new node for new device driver
  \param[in]   drv      device driver object
  \param[in]   size     device driver object size
  \param[in]   id       device node index
  \retrun      null on error, device object pointer on success
*/
yoc_dev_t *device_new(driver_t *drv, int size, int id);

/**
  \brief       free a device node
  \param[in]   dev      device driver object
  \return      none
*/
void device_free(yoc_dev_t *dev);

/**
  \brief       lock device to prevent to access
  \param[in]   dev      device driver object
  \return      none
*/
int device_lock(yoc_dev_t *dev);

/**
  \brief       unlock device to allow to access
  \param[in]   dev      device driver object
  \return      none
*/
int device_unlock(yoc_dev_t *dev);

/**
  \brief       valid a device 's name
  \param[in]   dev      device driver object
  \param[in]   name     device name
  \return      =0 success other is error
*/
int device_valid(yoc_dev_t *dev, const char *name);

int device_tags(yoc_dev_t *dev);

#endif
