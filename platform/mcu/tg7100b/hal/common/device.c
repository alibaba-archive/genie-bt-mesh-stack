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

#include <stdio.h>
#include <string.h>
//#include <unistd.h>
#include <errno.h>

#include <aos/kernel.h>
#include <devices/driver.h>
#include <devices/device.h>

#ifndef CONFIG_MAX_DEV_COUNT
#define CONFIG_MAX_DEV_COUNT 4
#endif

// static slist_t device_list;

#define CONFIG_DEVICE_THREAD_SAFE 0

#ifdef CONFIG_DEVICE_THREAD_SAFE
static aos_mutex_t device_mutex;

#define LIST_LOCK()                                                                                \
    do {                                                                                           \
        if (!aos_mutex_is_valid(&device_mutex))                                                    \
            aos_mutex_new(&device_mutex);                                                          \
        aos_mutex_lock(&device_mutex, AOS_WAIT_FOREVER);                                           \
    } while (0)

#define LIST_UNLOCK()                                                                              \
    do {                                                                                           \
        aos_mutex_unlock(&device_mutex);                                                           \
    } while (0)

#else

#define LIST_LOCK()
#define LIST_UNLOCK()

#endif

#define DEVICE_MAX_SIZE (128 + sizeof(yoc_dev_t))
#define DEVICE_FREE 0
#define DEVICE_USED 1

static uint32_t device_list[CONFIG_MAX_DEV_COUNT][DEVICE_MAX_SIZE / 4] = {0};

yoc_dev_t *device_new(driver_t *drv, int size, int id)
{
    yoc_dev_t *dev = NULL; //(yoc_dev_t*)aos_zalloc(size);
    int i = 0;

    if (size > DEVICE_MAX_SIZE) {
        return NULL;
    }

    for (i = 0; i < CONFIG_MAX_DEV_COUNT; i++) {
        if (device_list[i][0] == DEVICE_FREE) {
            dev = (yoc_dev_t *)&device_list[i][1];
            device_list[i][0] = DEVICE_USED;
            break;
        }
    }

    if (dev) {
        dev->id = id;
        dev->drv = drv;

        if (aos_mutex_new(&dev->mutex) != 0) {
            device_list[i][0] = DEVICE_FREE;
            return NULL;
        }
    }

    return dev;
}

void device_free(yoc_dev_t *dev)
{
    aos_mutex_free(&dev->mutex);

    uint8_t *device = (uint8_t*)dev;

    *(device - 1) =  DEVICE_FREE;

}

int device_lock(yoc_dev_t *dev)
{
    return aos_mutex_lock(&dev->mutex, AOS_WAIT_FOREVER);
}

int device_unlock(yoc_dev_t *dev)
{
    return aos_mutex_unlock(&dev->mutex);
}

static int alloc_device_id()
{
    uint32_t index = rand() & 0xFFFF;
    int i;

    LIST_LOCK();
    while (1) {
        index++;

        int found = 0;
        yoc_dev_t *node;

        for (i = 0; i < CONFIG_MAX_DEV_COUNT; i++) {
            if (device_list[i][0] == DEVICE_USED) {
                node = (yoc_dev_t *)&device_list[i][1];
                driver_t *drv = (driver_t *)node->drv;
                //aos_check_param(drv);
                if (drv->device_id == index << 16) {
                    found = 1;
                    break;
                }
            }
        }

        if (found == 0)
            break;
    }
    LIST_UNLOCK();

    return index << 16;
}

int driver_register(driver_t *drv, void *config, int idx)
{
    //aos_check_param(drv && drv->init);

    /* name exist check */
    if (device_find(drv->name, idx) != NULL)
        return -1;

    yoc_dev_t *dev = drv->init(drv, config, idx);

    if (dev) {
        dev->id = idx;
        ((driver_t *)(dev->drv))->device_id = alloc_device_id();

        return 0;
    }

    return -1;
}

yoc_dev_t *device_find(const char *name, int id)
{
    yoc_dev_t *node;
    int i;
    LIST_LOCK();

    for (i = 0; i < CONFIG_MAX_DEV_COUNT; i++) {
        node = NULL;
        if (device_list[i][0] == DEVICE_USED) {
            node = (yoc_dev_t *)&device_list[i][1];
            driver_t *drv = (driver_t *)node->drv;
            //aos_check_param(drv);
            if (strcmp(drv->name, name) == 0 && node->id == id)
                break;
        }
    }

    LIST_UNLOCK();

    return node;
}

static char *dev_name_parse(const char *name, int *id, char *n)
{
    //char *n = strdup(name);

    // if (n == NULL)
    //     return NULL;

    int len = strlen(n);

    for (int i = 0; i < len; i++) {
        if (n[i] >= '0' && n[i] <= '9') {
            *id = atoi(n + i);
            n[i] = '\0';
            break;
        }
    }

    return n;
}

int driver_unregister(const char *name)
{
    int id = 0;

    // char *n = dev_name_parse(name, &id);
    // if (n == NULL)
    //     return -ENOMEM;

    yoc_dev_t *dev = device_find(name, id);
    //aos_free(n);

    if (dev) {
        driver_t *drv = (driver_t *)dev->drv;
        aos_assert(drv);
        if (drv->uninit)
            drv->uninit(dev);

        return 0;
    }

    return -1;
}

yoc_dev_t *device_open(const char *name)
{
    int id = 0;
    char buffer[12] = {0};

    strncpy(buffer, name, 12 - 1);

    char *n = dev_name_parse(name, &id, buffer);
    if (n != NULL) {
        yoc_dev_t *dev = device_open_id(n, id);
        //aos_free(n);
        return dev;
    }

    return NULL;
}

yoc_dev_t *device_open_id(const char *name, int id)
{
    yoc_dev_t *dev = device_find(name, id);

    if (dev) {
        device_lock(dev);
        if (dev->ref == 0) {
            if (DRIVER(dev)->open(dev) != 0) {
                device_unlock(dev);
                return NULL;
            }
        }
        dev->ref++;
        device_unlock(dev);
    }

    return dev;
}

int device_close(yoc_dev_t *dev)
{
    int ret = -EBADF;

    device_lock(dev);
    if (dev->ref > 0) {
        dev->ref--;
        if (dev->ref == 0)
            ret = DRIVER(dev)->close(dev);
    }
    device_unlock(dev);

    return ret;
}

int device_valid(yoc_dev_t *dev, const char *name)
{
    if (dev && dev->drv) {
        const char *n = NULL;
        driver_t *drv = (driver_t*)dev->drv;
        if (drv->type)
            n = drv->type;
        else
            n = drv->name;

        if (n)
            return strcmp(n, name);
    }

    return -1;
}

int device_tags(yoc_dev_t *dev)
{
    driver_t *drv = (driver_t*)dev->drv;
    //aos_check_param(drv);

    return drv->device_id;
}

int device_is_busy(void)
{
    yoc_dev_t *node;
    int i;
    int busy = 0;

    LIST_LOCK();

    for (i = 0; i < CONFIG_MAX_DEV_COUNT; i++) {
        if (device_list[i][0] == DEVICE_USED) {
            node = (yoc_dev_t *)&device_list[i][1];
            if (node->busy == 1) {
                busy = 1;
                break;
            }
        }
    }

    LIST_UNLOCK();

    return busy;
}

void device_manage_power(int pm_state)
{
    yoc_dev_t *node;
    int i;
    LIST_LOCK();

    for (i = 0; i < CONFIG_MAX_DEV_COUNT; i++) {
        if (device_list[i][0] == DEVICE_USED) {
            node = (yoc_dev_t *)&device_list[i][1];
            driver_t *drv = (driver_t *)node->drv;
            //aos_check_param(drv);
            if (drv->lpm) {
                drv->lpm(node, pm_state);
            }
        }
    }

    LIST_UNLOCK();
}

