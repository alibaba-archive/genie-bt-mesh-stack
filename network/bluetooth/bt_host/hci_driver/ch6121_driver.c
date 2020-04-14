/*
 * Copyright (C) 2016 YunOS Project. All rights reserved.
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
#include <errno.h>
#include <stddef.h>

#include <zephyr.h>

#define BT_DBG_ENABLED 0

#include <common/log.h>
#include <bluetooth.h>
#include <hci.h>
#include <hci_driver.h>

#include <device.h>
#include <devices/hci.h>
#include <aos/hci_api.h>
#include <hci_ecc.h>
#include <aos/kernel.h>
#include <misc/byteorder.h>

static struct k_thread rx_thread_data;
#define CONFIG_BT_RX_STACK_SIZE 1024
static BT_STACK_NOINIT(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);

#define H4_NONE      0x00
#define H4_CMD       0x01
#define H4_ACL_UP    0x02
#define H4_SCO       0x03
#define H4_EVT       0x04
#define H4_ACL_DOWN  0x05

static int h4_open(void);
static int h4_send(struct net_buf *buf);

typedef struct _simple_data_t {
    sys_snode_t node;
    void *pool_ref;
    uint8_t in_use;
    uint8_t reserve;
    uint16_t data_len;
    uint8_t *data;
} simple_data_t;

typedef struct _simple_pool_t {
    uint16_t data_count;
    uint16_t data_size;
    uint32_t alloc_count;
    uint32_t peek_count;
    sys_slist_t free;
    simple_data_t *simple_data_ref;
    void *data_ref;
} simple_pool_t;

#define ALIGN_MASK       (sizeof(void*)-1)
#define ALIGN_UP(a)   (((a) + ALIGN_MASK) & ~ALIGN_MASK)

#define SIMPLE_POOL_INIT(_name,_count,_data_size)    \
    static uint32_t _name##_data[_count][ALIGN_UP(_data_size) / 4] __attribute__((section("noretention_mem_area0"))) = {0};  \
    static simple_data_t  _name##_simple_data[_count]; \
    static simple_pool_t _name = {                 \
                                                   .data_count = _count,                      \
                                                   .data_size = _data_size,                        \
                                                   .simple_data_ref = (simple_data_t *)_name##_simple_data,  \
                                                   .data_ref = _name##_data,  \
                                                   .alloc_count = 0,  \
                                 };

void simple_data_init(simple_pool_t *pool)
{
    int i;
    simple_data_t *simple_data;

    if (pool == NULL) {
        return;
    }

    sys_slist_init(&pool->free);

    for (i = 0; i < pool->data_count; i++) {
        simple_data = &pool->simple_data_ref[i];
        memset(simple_data, 0, sizeof(simple_data_t));
        simple_data->in_use = 0;
        simple_data->pool_ref = pool;
        simple_data->data = (uint8_t *)pool->data_ref + ALIGN_UP(pool->data_size) * i;
        sys_slist_append(&pool->free, &simple_data->node);
    }

    return;
}

simple_data_t *simple_data_alloc(simple_pool_t *pool)
{
    if (pool == NULL) {
        return NULL;
    }

    size_t irq_flags;
    irq_flags = cpu_intrpt_save();

    simple_data_t *simple_data = (simple_data_t *) sys_slist_get(&pool->free);

    if (simple_data) {
        simple_data->in_use = 1;
        simple_data->node.next = NULL;
        pool->alloc_count++;

        if (pool->alloc_count > pool->peek_count) {
            pool->peek_count = pool->alloc_count;
        }
    }

    cpu_intrpt_restore(irq_flags);

    return simple_data;
}

void simple_data_free(simple_data_t *simple_data)
{
    size_t irq_flags;
    irq_flags = cpu_intrpt_save();
    simple_data->in_use = 0;
    simple_pool_t *pool = simple_data->pool_ref;
    sys_slist_append(&pool->free, &simple_data->node);
    pool->alloc_count--;
    cpu_intrpt_restore(irq_flags);
}

static struct bt_hci_driver drv = {
    .name       = "H4",
    .bus        = BT_HCI_DRIVER_BUS_VIRTUAL,
    .open       = h4_open,
    .send       = h4_send,
};

static struct {
    const struct bt_hci_driver *drv;
    char *dev_name;
    dev_t *dev;
} hci_h4 = {
    &drv,
    NULL,
    NULL,
};

#define HCI_DATA_SIZE 256
#define HCI_DATA_NUM 8

#define HCI_ADV_DATA_SIZE 46
#define HCI_ADV_NUM 15

SIMPLE_POOL_INIT(hci_data_pool, HCI_DATA_NUM, HCI_DATA_SIZE);
SIMPLE_POOL_INIT(hci_adv_data_pool, HCI_ADV_NUM, HCI_ADV_DATA_SIZE);

static int h4_send(struct net_buf *buf)
{
    int ret;

    if (IS_ENABLED(CONFIG_BT_HARD_ECC)) {

        if (bt_buf_get_type(buf) == BT_BUF_CMD) {
            struct bt_hci_cmd_hdr *chdr = (void *)buf->data;

            switch (sys_le16_to_cpu(chdr->opcode)) {
                case BT_HCI_OP_LE_P256_PUBLIC_KEY:
                    net_buf_pull(buf, sizeof(*chdr));
                    le_p256_pub_key(buf);
                    return 0;

                case BT_HCI_OP_LE_GENERATE_DHKEY:
                    net_buf_pull(buf, sizeof(*chdr));
                    le_gen_dhkey(buf);
                    return 0;

                case BT_HCI_OP_LE_SET_EVENT_MASK:
                    clear_ecc_events(buf);
                    break;

                default:
                    break;
            }
        }
    }

    uint8_t type  = bt_buf_get_type(buf);

    if (type == BT_BUF_ACL_OUT) {
        net_buf_push_u8(buf, H4_ACL_DOWN);
    } else if (type == BT_BUF_CMD) {
        net_buf_push_u8(buf, H4_CMD);
    } else {
        BT_ERR("Unknown buffer type");
        return -1;
    }

    BT_DBG("buf %p type %u len %u:%s", buf, type, buf->len, bt_hex(buf->data, buf->len));

    ret = hci_send(hci_h4.dev, buf->data, buf->len);

    net_buf_unref(buf);

    return ret;
}

struct kfifo {
    struct k_queue _queue;
};

static struct kfifo  rx_fifo;
void hci_event_recv(simple_data_t *simple_data);
void hci_acl_recv(simple_data_t *simple_data);

static void rx_thread(void *p1)
{
    simple_data_t *simple_data;

    ARG_UNUSED(p1);

    BT_DBG("started");

    while (1) {

        simple_data = k_fifo_get(&rx_fifo, K_FOREVER);

        while (simple_data) {
            if (simple_data->data[0] == H4_EVT) {
                hci_event_recv(simple_data);
            } else if (simple_data->data[0] == H4_ACL_UP) {
                hci_acl_recv(simple_data);
            } else {
                simple_data_free(simple_data);
                continue;
            }

            simple_data = k_fifo_get(&rx_fifo, K_NO_WAIT);
        };

        //krhino_task_yield();
    }
}

static inline int is_adv_report_event(uint8_t *data, uint16_t len)
{
    return (data[0] == H4_EVT && data[1] == BT_HCI_EVT_LE_META_EVENT
            && data[3] == BT_HCI_EVT_LE_ADVERTISING_REPORT);
}

void hci_event_recv(simple_data_t *simple_data)
{
    struct net_buf *buf;
    uint8_t *pdata = simple_data->data;
    int32_t len = simple_data->data_len;
    struct bt_hci_evt_hdr hdr;
    uint8_t sub_event = 0;
    uint8_t discardable = 0;

    if (pdata == NULL || len == 0) {
        return;
    }

    if (*pdata++ != H4_EVT) {
        goto data_free;
    }

    if (len < 3) {
        goto data_free;
    }

    hdr.evt = *pdata++;
    hdr.len = *pdata++;

    if (len < hdr.len + 3) {
        goto data_free;
    }

    if (hdr.evt == BT_HCI_EVT_LE_META_EVENT) {
        sub_event = *pdata++;

        if (sub_event == BT_HCI_EVT_LE_ADVERTISING_REPORT) {
            discardable = 1;
        }
    }

    if (hdr.evt == BT_HCI_EVT_CMD_COMPLETE ||
        hdr.evt  == BT_HCI_EVT_CMD_STATUS) {
        buf = bt_buf_get_cmd_complete(0);

        if (buf == NULL) {
            goto data_free;
        }
    }

    buf = bt_buf_get_rx(BT_BUF_EVT, 0);

    if (!buf && discardable) {
        goto data_free;
    }

    if (!buf) {
        goto data_free;
    }

    bt_buf_set_type(buf, BT_BUF_EVT);

    net_buf_add_mem(buf, ((uint8_t *)(simple_data->data)) + 1, hdr.len + sizeof(hdr));

    simple_data_free(simple_data);

    BT_DBG("event %s", bt_hex(buf->data, buf->len));

    if (bt_hci_evt_is_prio(hdr.evt)) {
        bt_recv_prio(buf);
    } else {
        bt_recv(buf);
    }

    return;

data_free:
    simple_data_free(simple_data);
    return;
}

void hci_acl_recv(simple_data_t *simple_data)
{
    struct net_buf *buf;
    uint8_t *pdata = simple_data->data;
    int32_t len = simple_data->data_len;
    uint16_t handle;
    uint16_t acl_len;

    if (pdata == NULL || len == 0) {
        return;
    }

    if (*pdata++ != H4_ACL_UP) {
        goto data_free;
    }

    if (len < 5) {
        goto data_free;
    }

    handle = ((*pdata + 1) << 16) | (*(pdata));
    pdata += 2;
    acl_len = ((*pdata + 1) << 16) | (*(pdata));
    pdata += 2;

    (void)handle;

    if (len < acl_len + 5) {
        goto data_free;
    }

    buf = bt_buf_get_rx(BT_BUF_ACL_IN, 0);

    if (!buf) {
        //g_hci_debug_counter.hci_in_is_null_count++;
        goto data_free;
    }

    bt_buf_set_type(buf, BT_BUF_ACL_IN);

    net_buf_add_mem(buf, (uint8_t *)(simple_data->data) + 1, acl_len + 4);
    simple_data_free(simple_data);
    //g_hci_debug_counter.acl_in_count++;

    bt_recv(buf);
    return;

data_free:
    simple_data_free(simple_data);
    return;
}

int hci_recv(void *data, int32_t len)
{
    simple_data_t *hci_data = NULL;

    if (data == NULL || len == 0) {
        return -1;
    }

    if (is_adv_report_event(data, len)) {
        if (len > hci_adv_data_pool.data_size) {
            return -1;
        }

        hci_data = simple_data_alloc(&hci_adv_data_pool);

        if (hci_data == NULL) {
            return 0;
        }
    } else {
        if (len > hci_data_pool.data_size) {
            return -1;
        }

        hci_data = simple_data_alloc(&hci_data_pool);
    }

    if (hci_data == NULL) {
        return -1;
    }

    memcpy(hci_data->data, data, len);
    hci_data->data_len = len;
    k_fifo_put(&rx_fifo, &hci_data->node);

    return 0;
}

static int h4_open(void)
{
    int ret = 0;

    if (IS_ENABLED(CONFIG_BT_HARD_ECC)) {
        bt_hci_ecc_init();
    }

    k_fifo_init(&rx_fifo);
    simple_data_init(&hci_data_pool);
    simple_data_init(&hci_adv_data_pool);

    ret =  k_thread_create(&rx_thread_data, rx_thread_stack, sizeof(rx_thread_stack), rx_thread,
                           NULL, NULL, NULL, CONFIG_BT_RX_PRIO, 0, K_NO_WAIT);

    if (ret) {
        return ret;
    }

    hci_h4.dev = hci_open_id(hci_h4.dev_name, 0);

    if (hci_h4.dev == NULL) {
        BT_ERR("device open fail");
        return -1;
    }

    return 0;
}

int hci_driver_init()
{
    int ret;
    hci_driver_ch6121_register(0);
    hci_h4.dev_name = "hci_ch6121";
    ret = bt_hci_driver_register(&drv);

    if (ret) {
        return ret;
    }

    return 0;
}
