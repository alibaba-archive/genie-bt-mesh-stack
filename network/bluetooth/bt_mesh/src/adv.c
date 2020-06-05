/*  Bluetooth Mesh */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/stack.h>
#include <misc/util.h>

#include <net/buf.h>
#include <bluetooth.h>
#include <hci.h>
#include <conn.h>
#include <api/mesh.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_ADV)
#include "common/log.h"

#include "hci_core.h"

#include "mesh_def.h"
#include "adv.h"
#include "foundation.h"
#include "net.h"
#include "beacon.h"
#include "prov.h"
#include "proxy.h"
#include <port/mesh_hal_ble.h>
#include <port/mesh_hal_os.h>
#include "bt_mesh_custom_log.h"
#ifdef CONFIG_BT_MESH_MULTIADV
#include "multi_adv.h"
#endif

/* Window and Interval are equal for continuous scanning */
#define MESH_SCAN_INTERVAL 0x10
#define MESH_SCAN_WINDOW   0x10

/* Convert from ms to 0.625ms units */
#define ADV_INT(_ms) ((_ms) * 8 / 5)

/* Pre-5.0 controllers enforce a minimum interval of 100ms
 * whereas 5.0+ controllers can go down to 20ms.
 */
#define ADV_INT_DEFAULT  K_MSEC(100)
#define ADV_INT_FAST     K_MSEC(20)

#ifndef GENIE_DEFAULT_DURATION
#define GENIE_DEFAULT_DURATION 125
#endif

/* TinyCrypt PRNG consumes a lot of stack space, so we need to have
 * an increased call stack whenever it's used.
 */
#if (defined(BOARD_CH6121EVB))  || defined(BOARD_TG7100B) || ((defined(CONFIG_BT_TINYCRYPT_ECC)) && (!defined(BOARD_BK3435DEVKIT)) && (!defined(BOARD_TC825X)))
#define ADV_STACK_SIZE 768
#else
#define ADV_STACK_SIZE (512-256)
#endif

static K_FIFO_DEFINE(adv_queue);
static struct k_thread adv_thread_data;
static BT_STACK_NOINIT(adv_thread_stack, ADV_STACK_SIZE);

static const u8_t adv_type[] = {
    [BT_MESH_ADV_PROV]   = BT_MESH_DATA_MESH_PROV,
    [BT_MESH_ADV_DATA]   = BT_MESH_DATA_MESH_MESSAGE,
    [BT_MESH_ADV_BEACON] = BT_MESH_DATA_MESH_BEACON,
};

NET_BUF_POOL_DEFINE(adv_buf_pool, CONFIG_BT_MESH_ADV_BUF_COUNT,
                    BT_MESH_ADV_DATA_SIZE, BT_MESH_ADV_USER_DATA_SIZE, NULL);

static struct bt_mesh_adv adv_pool[CONFIG_BT_MESH_ADV_BUF_COUNT];

static const bt_addr_le_t *dev_addr;

#ifdef CONFIG_BT_MESH_MULTIADV
static struct mesh_multi_adv g_mesh_multi_adv;
static struct k_delayed_work g_mesh_adv_timer;
#endif

static struct bt_mesh_adv *adv_alloc(int id)
{
    return &adv_pool[id];
}

static inline void adv_send_start(u16_t duration, int err,
                                  const struct bt_mesh_send_cb *cb,
                                  void *cb_data)
{
    if (cb && cb->start) {
        cb->start(duration, err, cb_data);
    }
}

static inline void adv_send_end(int err, const struct bt_mesh_send_cb *cb,
                                void *cb_data)
{
    if (cb && cb->end) {
        cb->end(err, cb_data);
    }
}

static inline void adv_send(struct net_buf *buf)
{
    s32_t adv_int_min = ((bt_dev.hci_version >= BT_HCI_VERSION_5_0) ?
                               ADV_INT_FAST : ADV_INT_DEFAULT);
    const struct bt_mesh_send_cb *cb = BT_MESH_ADV(buf)->cb;
    void *cb_data = BT_MESH_ADV(buf)->cb_data;
    struct bt_mesh_le_adv_param param;
    u16_t duration, adv_int;
    struct bt_mesh_data ad;
    int err;

    /* do not check ADV interval */
#ifdef CONFIG_ADV_MIN_INTERVAL
    adv_int_min = CONFIG_ADV_MIN_INTERVAL;
#endif

    adv_int = max(adv_int_min, BT_MESH_ADV(buf)->adv_int);
#ifdef CONFIG_ALI_SIMPLE_MODLE
    duration = GENIE_DEFAULT_DURATION;
#else
    duration = (BT_MESH_ADV(buf)->count + 1) * (adv_int + 10);
#endif

    BT_DBG("type %u len %u: %s", BT_MESH_ADV(buf)->type,
           buf->len, bt_hex(buf->data, buf->len));
    BT_DBG("count %u interval %ums, %ums, %ums duration %ums",
           BT_MESH_ADV(buf)->count + 1, adv_int, adv_int, BT_MESH_ADV(buf)->adv_int, duration);

#ifdef GENIE_ULTRA_PROV
    if(BT_MESH_ADV(buf)->tiny_adv == 1) {
        ad.type = 0xFF;
    } else {
        ad.type = adv_type[BT_MESH_ADV(buf)->type];
    }
#else
    ad.type = adv_type[BT_MESH_ADV(buf)->type];
#endif
    ad.data_len = buf->len;
    ad.data = buf->data;

    param.options = 0;
    param.interval_min = ADV_INT(adv_int);
    param.interval_max = param.interval_min;
    param.own_addr = NULL;

    err = bt_mesh_adv_start(&param, &ad, 1, NULL, 0);
    net_buf_unref(buf);
    adv_send_start(duration, err, cb, cb_data);
    if (err) {
        BT_ERR("Advertising failed: err %d", err);
        goto exit;
    }

    BT_DBG("Advertising started. Sleeping %u ms", duration);

    k_sleep(duration);

exit:
    err = bt_mesh_adv_stop();
    adv_send_end(err, cb, cb_data);
    if (err) {
        BT_ERR("Stopping advertising failed: err %d", err);
        return;
    }

    BT_DBG("Advertising stopped");
}

#ifdef CONFIG_BT_MESH_MULTIADV
static inline int adv_send_multi(struct net_buf *buf)
{
    s32_t adv_int_min = ((bt_dev.hci_version >= BT_HCI_VERSION_5_0) ?
                               ADV_INT_FAST : ADV_INT_DEFAULT);
    const struct bt_mesh_send_cb *cb = BT_MESH_ADV(buf)->cb;
    void *cb_data = BT_MESH_ADV(buf)->cb_data;
    struct bt_mesh_le_adv_param param;
    u16_t duration, adv_int;
    struct bt_mesh_data ad;
    int err, instant_id = 0;
    struct mesh_multi_adv *multi_adv = &g_mesh_multi_adv;

    /* do not check ADV interval */
#ifdef CONFIG_ADV_MIN_INTERVAL
    adv_int_min = CONFIG_ADV_MIN_INTERVAL;
#endif

    adv_int = max(adv_int_min, BT_MESH_ADV(buf)->adv_int);
    duration = (BT_MESH_ADV(buf)->count + 1) * adv_int + 10;

    BT_DBG("type %u len %u: %s", BT_MESH_ADV(buf)->type,
           buf->len, bt_hex(buf->data, buf->len));
    BT_DBG("count %u interval %ums, interval buff %ums,duration %ums",
           BT_MESH_ADV(buf)->count + 1, adv_int, BT_MESH_ADV(buf)->adv_int, duration);

#ifdef GENIE_ULTRA_PROV
    if(BT_MESH_ADV(buf)->tiny_adv == 1) {
        ad.type = 0xFF;
    } else {
        ad.type = adv_type[BT_MESH_ADV(buf)->type];
    }
#else
    ad.type = adv_type[BT_MESH_ADV(buf)->type];
#endif
    ad.data_len = buf->len;
    ad.data = buf->data;

    param.options = 0;
    param.interval_min = ADV_INT(adv_int);
    param.interval_max = param.interval_min;
    param.own_addr = NULL;

    err = bt_mesh_multi_adv_start(&param, &ad, 1, NULL, 0, &instant_id);
    net_buf_unref(buf);
    if (err) {
        adv_send_end(err, cb, cb_data);
        return 0;
    }

    adv_send_start(duration, err, cb, cb_data);
    multi_adv->mesh_intant_id = instant_id;
    multi_adv->cb = (void *)cb;
    multi_adv->cb_data = cb_data;

    BT_DBG("Mesh Ad Inst started state %d. duration %u ms", err, duration);

    return duration;
}

void bt_mesh_adv_event_timer_callback(struct k_work *timer)
{
    kevent_t *event = bt_mesh_multi_adv_get_event();

    k_event_set(event, EVENT_TYPE_MESH_TIMER_EVENT);
    return;
}

void bt_mesh_adv_event_init(void)
{
    struct mesh_multi_adv *multi_adv = &g_mesh_multi_adv;

    multi_adv->state = 0;
    multi_adv->mesh_intant_id = 0;

    k_delayed_work_init(&g_mesh_adv_timer, bt_mesh_adv_event_timer_callback);
    k_delayed_work_submit(&g_mesh_adv_timer, 1000);
}

void bt_mesh_adv_timer_event_process(void)
{
    struct mesh_multi_adv *multi_adv = &g_mesh_multi_adv;

    /* stop adv instant */
    if (multi_adv->mesh_intant_id) {
        bt_mesh_multi_adv_stop(multi_adv->mesh_intant_id);
        multi_adv->mesh_intant_id = 0;
    }

    /* adv end */
    if (multi_adv->cb) {
        const struct bt_mesh_send_cb *cb = multi_adv->cb;
        void *cb_data = multi_adv->cb_data;
    
        adv_send_end(0, cb, cb_data);
        multi_adv->cb = 0;
        multi_adv->cb_data = 0;
    }
}

void bt_mesh_adv_event_process(void)
{
    struct mesh_multi_adv *multi_adv = &g_mesh_multi_adv;
    struct net_buf *buf;
    uint32_t timeout = 0;

    /* check if adv instant exist */
    if (multi_adv->mesh_intant_id) {
        return;
    }

    /* check if any adv buffer in queue */
    buf = net_buf_get(&adv_queue, K_NO_WAIT);
    if (buf) {
        if (BT_MESH_ADV(buf)->busy) {
            BT_MESH_ADV(buf)->busy = 0;
            timeout = adv_send_multi(buf);
        }
    }
    if (timeout == 0) {
        timeout = 1000;
    }

    if (timeout) {
        k_delayed_work_submit(&g_mesh_adv_timer, timeout);
    }
}
#endif

static void adv_thread(void *p1, void *p2, void *p3)
{
    BT_DBG("started");

#ifdef CONFIG_BT_MESH_MULTIADV
    bt_mesh_multi_adv_thread_init();
#endif
    while (1) {
#ifdef CONFIG_BT_MESH_MULTIADV
        bt_mesh_multi_adv_thread_run();
#else
        struct net_buf *buf;

        if (IS_ENABLED(CONFIG_BT_MESH_GATT_PROXY)) {
#if defined(BOARD_CH6121EVB) || defined(BOARD_TG7100B)
            buf = net_buf_get(&adv_queue, NOCONN_ADV_DATA_TIEMOUT);
            if (!buf) {
                s32_t timeout = bt_mesh_proxy_adv_start();
                BT_DBG("Proxy Advertising up to %d ms", timeout);
                buf = net_buf_get(&adv_queue, timeout);
                bt_mesh_proxy_adv_stop();
            }
#else
            uint32_t time_start;
            buf = net_buf_get(&adv_queue, K_NO_WAIT);
            while (!buf) {
                s32_t timeout;

                timeout = bt_mesh_proxy_adv_start();
                time_start = k_uptime_get_32();
                BT_DBG("Proxy Advertising up to %d ms", timeout);
                while (buf == NULL) {
                    buf = net_buf_get(&adv_queue, timeout);
                    if (buf || timeout == K_NO_WAIT ||
                        (timeout != -1 && (k_uptime_get_32() - time_start) >= timeout)) {
                        break;
                    } else {
                        k_sleep(1);
                    }
                }
                bt_mesh_proxy_adv_stop();
            }
#endif
        } else {
            buf = net_buf_get(&adv_queue, K_FOREVER);
        }

        if (!buf) {
            continue;
        }

        /* busy == 0 means this was canceled */
        if (BT_MESH_ADV(buf)->busy) {
            BT_MESH_ADV(buf)->busy = 0;
            adv_send(buf);
        } else {
             // unref it even if it is canceled.
            net_buf_unref(buf);
        }
#endif

        STACK_ANALYZE("adv stack", adv_thread_stack);

        /* Give other threads a chance to run */
        k_yield();
    }
}

void bt_mesh_adv_update(void)
{
    BT_DBG("");

    k_fifo_cancel_wait(&adv_queue);
}

struct net_buf *bt_mesh_adv_create_from_pool(struct net_buf_pool *pool,
                                             bt_mesh_adv_alloc_t get_id,
                                             enum bt_mesh_adv_type type,
                                             u8_t xmit_count, u16_t xmit_int,
                                             s32_t timeout)
{
    struct bt_mesh_adv *adv;
    struct net_buf *buf;

    buf = net_buf_alloc(pool, timeout);
    if (!buf) {
        return NULL;
    }

    adv = get_id(net_buf_id(buf));
    BT_MESH_ADV(buf) = adv;

    memset(adv, 0, sizeof(*adv));

    adv->type         = type;
    adv->count        = xmit_count;
    adv->adv_int      = xmit_int;

    return buf;
}

struct net_buf *bt_mesh_adv_create(enum bt_mesh_adv_type type, u8_t xmit_count,
                                   u16_t xmit_int, s32_t timeout)
{
    return bt_mesh_adv_create_from_pool(&adv_buf_pool, adv_alloc, type,
                                        xmit_count, xmit_int, timeout);
}

void bt_mesh_adv_send(struct net_buf *buf, const struct bt_mesh_send_cb *cb,
                      void *cb_data)
{
    BT_DBG("type 0x%02x len %u: %s", BT_MESH_ADV(buf)->type, buf->len,
           bt_hex(buf->data, buf->len));

    BT_MESH_ADV(buf)->cb = cb;
    BT_MESH_ADV(buf)->cb_data = cb_data;
    BT_MESH_ADV(buf)->busy = 1;

    net_buf_put(&adv_queue, net_buf_ref(buf));
#ifdef CONFIG_BT_MESH_MULTIADV
    {
        kevent_t *event = bt_mesh_multi_adv_get_event();

        k_event_set(event, EVENT_TYPE_MESH_EVENT);
    }
#endif
}

const bt_addr_le_t *bt_mesh_pba_get_addr(void)
{
	return dev_addr;
}

static void bt_mesh_scan_cb(const bt_mesh_addr_le_t *addr, s8_t rssi,
                            u8_t adv_type, struct net_buf_simple *buf)
{
    if (adv_type != BT_MESH_LE_ADV_NONCONN_IND && adv_type != BT_LE_ADV_IND) {
        return;
    }

    //BT_DBG("len %u: %s", buf->len, bt_hex(buf->data, buf->len));

	dev_addr = (bt_addr_le_t *)addr;

    while (buf->len > 1) {
        struct net_buf_simple_state state;
        u8_t len, type;

        len = net_buf_simple_pull_u8(buf);
        /* Check for early termination */
        if (len == 0) {
            return;
        }

        if (len > buf->len || buf->len < 1) {
            BT_WARN("AD malformed");
            return;
        }

        net_buf_simple_save(buf, &state);

        type = net_buf_simple_pull_u8(buf);

        buf->len = len - 1;
        if (adv_type == BT_LE_ADV_NONCONN_IND) {
            switch (type) {
                case BT_MESH_DATA_MESH_MESSAGE:
                    bt_mesh_net_recv(buf, rssi, BT_MESH_NET_IF_ADV);
                    break;
    #if defined(CONFIG_BT_MESH_PB_ADV)
                case BT_MESH_DATA_MESH_PROV:
                    bt_mesh_pb_adv_recv(buf);
                    break;
    #endif
                case BT_MESH_DATA_MESH_BEACON:
                    bt_mesh_beacon_recv(buf);
                    break;
                default:
                    break;
            }
        } else if (adv_type == BT_LE_ADV_IND) {
            switch (type) {
#ifdef GENIE_ULTRA_PROV
                case 0xFF:
                {
                    uint16_t company = net_buf_simple_pull_be16(buf);
                    if(company == 0x01A8) {
                        uint8_t fixed_byte = net_buf_simple_pull_u8(buf);
                        if(fixed_byte == 0x0D) {
                            uint8_t prov_cmd = net_buf_simple_pull_u8(buf);
                            switch(prov_cmd) {
                                case 0x00:
                                    ultra_prov_recv_random(buf->data);
                                    break;
                                case 0x02:
                                    ultra_prov_recv_prov_data(buf->data);
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                    break;
                }
#endif
                default:
                    break;
            }
    	}

        net_buf_simple_restore(buf, &state);
        net_buf_simple_pull(buf, len);
    }
}

void bt_mesh_adv_init(void)
{
    k_fifo_init(&adv_queue);
    k_lifo_init(&adv_buf_pool.free);
#if defined(BOARD_CH6121EVB) || defined(BOARD_TG7100B)
    bt_mesh_adv_scan_schd_init();
#endif
    k_thread_create(&adv_thread_data, adv_thread_stack,
                    K_THREAD_STACK_SIZEOF(adv_thread_stack), adv_thread,
                    NULL, NULL, NULL, CONFIG_BT_MESH_ADV_PRIO, 0, K_NO_WAIT);
}

int bt_mesh_scan_enable(void)
{
    struct bt_mesh_le_scan_param scan_param = {
        .type       = BT_MESH_HCI_LE_SCAN_PASSIVE,
        .filter_dup = BT_MESH_HCI_LE_SCAN_FILTER_DUP_DISABLE,
        .interval   = MESH_SCAN_INTERVAL,
        .window     = MESH_SCAN_WINDOW
    };

    BT_DBG("%s", __func__);

    return bt_mesh_scan_start(&scan_param, bt_mesh_scan_cb);
}

int bt_mesh_scan_disable(void)
{
    BT_DBG("");

    return bt_mesh_scan_stop();
}
