/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#include <port/mesh_hal_ble.h>
#include <errno.h>

#ifndef CONFIG_MESH_STACK_ALONE
#include <conn.h>
#include <gatt.h>
#include <bluetooth.h>
#ifdef BOARD_CH6121EVB

#define SCHD_LOGD(...) //printf
#define CONN_ADV_DATA_TIEMOUT   (6)
#define NOCONN_ADV_DATA_TIEMOUT (2)

typedef enum {
    SCHD_IDLE = 0,
    SCHD_ADV,
    SCHD_SCAN,
    SCHD_ADV_SCAN,

    SCHD_INVAILD,
} adv_scan_schd_state_en;

typedef enum {
    ADV_ON = 0,
    ADV_OFF,
    SCAN_ON,
    SCAN_OFF,

    ACTION_INVAILD,
} adv_scan_schd_action_en;

typedef int (*adv_scan_schd_func_t)(adv_scan_schd_state_en st);

static int adv_scan_schd_idle_enter(adv_scan_schd_state_en st);
static int adv_scan_schd_idle_exit(adv_scan_schd_state_en st);
static int adv_scan_schd_adv_enter(adv_scan_schd_state_en st);
static int adv_scan_schd_adv_exit(adv_scan_schd_state_en st);
static int adv_scan_schd_scan_enter(adv_scan_schd_state_en st);
static int adv_scan_schd_scan_exit(adv_scan_schd_state_en st);
static int adv_scan_schd_adv_scan_enter(adv_scan_schd_state_en st);
static int adv_scan_schd_adv_scan_exit(adv_scan_schd_state_en st);

struct {
    adv_scan_schd_func_t enter;
    adv_scan_schd_func_t exit;
} adv_scan_schd_funcs[] = {
    {adv_scan_schd_idle_enter, adv_scan_schd_idle_exit},
    {adv_scan_schd_adv_enter, adv_scan_schd_adv_exit},
    {adv_scan_schd_scan_enter, adv_scan_schd_scan_exit},
    {adv_scan_schd_adv_scan_enter, adv_scan_schd_adv_scan_exit},
};

adv_scan_schd_state_en adv_scan_schd_st_change_map[4][4] = {
    {SCHD_ADV, SCHD_IDLE, SCHD_SCAN, SCHD_IDLE},
    {SCHD_ADV, SCHD_IDLE, SCHD_ADV_SCAN, SCHD_ADV},
    {SCHD_ADV_SCAN, SCHD_SCAN, SCHD_SCAN, SCHD_IDLE},
    {SCHD_ADV_SCAN, SCHD_SCAN, SCHD_ADV_SCAN, SCHD_ADV},
};

struct adv_scan_data_t {
    uint8_t ad_data[31];
    size_t ad_len;
    uint8_t sd_data[31];
    size_t sd_len;
    struct bt_le_adv_param adv_param;
    struct bt_le_scan_param scan_param;
    bt_le_scan_cb_t *scan_cb;
};

#define FLAG_RESTART 1

struct {
    struct k_mutex mutex;
    k_timer_t timer;
    uint8_t flag;
    adv_scan_schd_state_en cur_st;
    struct adv_scan_data_t param;
} adv_scan_schd = {0};

static int adv_scan_schd_idle_enter(adv_scan_schd_state_en st)
{
    SCHD_LOGD("idle enter\n");
    memset(&adv_scan_schd.param, 0, sizeof(struct adv_scan_data_t));
    return 0;
}

static int adv_scan_schd_idle_exit(adv_scan_schd_state_en st)
{
    SCHD_LOGD("idle exit\n");
    // do nothing
    return 0;
}

static int adv_scan_schd_adv_enter(adv_scan_schd_state_en st)
{
    int ret;
    SCHD_LOGD("adv on enter\n");

    if (st == SCHD_IDLE || st == SCHD_ADV_SCAN || st == SCHD_ADV) {
        if (adv_scan_schd.param.ad_len) {
            ret = bt_le_adv_start_instant(&adv_scan_schd.param.adv_param,
                                          adv_scan_schd.param.ad_data, adv_scan_schd.param.ad_len,
                                          adv_scan_schd.param.sd_data, adv_scan_schd.param.sd_len);

            if (ret) {
                return ret;
            }

            return 0;
        }
    }

    return -EINVAL;;
}

static int adv_scan_schd_adv_exit(adv_scan_schd_state_en st)
{
    SCHD_LOGD("adv on exit\n");

    if (st == SCHD_ADV_SCAN || st == SCHD_IDLE || st == SCHD_ADV) {
        return bt_le_adv_stop_instant();
    }

    return -EINVAL;
}

static int adv_scan_schd_scan_enter(adv_scan_schd_state_en st)
{
    if (st == SCHD_SCAN)
    {
        return 0;
    }

    SCHD_LOGD("scan on enter\n");
    if (st == SCHD_IDLE || st == SCHD_ADV_SCAN) {
        return bt_le_scan_start(&adv_scan_schd.param.scan_param, adv_scan_schd.param.scan_cb);
    }

    return -EINVAL;
}

static int adv_scan_schd_scan_exit(adv_scan_schd_state_en st)
{
    if (st == SCHD_SCAN)
    {
        return 0;
    }

    SCHD_LOGD("scan on exit\n");
    if (st == SCHD_ADV_SCAN || st == SCHD_IDLE) {
        return bt_le_scan_stop();
    }

    return -EINVAL;
}

static int adv_scan_schd_adv_scan_enter(adv_scan_schd_state_en st)
{
    SCHD_LOGD("adv scan on enter\n");

    if (st == SCHD_ADV || st == SCHD_SCAN || st == SCHD_ADV_SCAN) {
        adv_scan_schd.flag = FLAG_RESTART;
        k_timer_start(&adv_scan_schd.timer, 1);
        return 0;
    }

    return -EINVAL;
}

static int adv_scan_schd_adv_scan_exit(adv_scan_schd_state_en st)
{
    int ret;
    SCHD_LOGD("adv scan on exit\n");

    if (st == SCHD_ADV || st == SCHD_SCAN || st == SCHD_ADV_SCAN) {
        k_timer_stop(&adv_scan_schd.timer);

        ret = bt_le_scan_stop();

        if (ret && ret != -EALREADY) {
            SCHD_LOGD("scan stop err %d\n", ret);
            return ret;
        }

        ret = bt_le_adv_stop();

        if (ret && ret != -EALREADY) {
            SCHD_LOGD("adv stop err %d\n", ret);
            return ret;
        }

        return 0;
    }

    return -EINVAL;
}

int bt_mesh_adv_scan_schd(adv_scan_schd_state_en st)
{
    int ret;
    SCHD_LOGD("%d->%d\n", adv_scan_schd.cur_st, st);

    if (st < SCHD_INVAILD) {
        ret = adv_scan_schd_funcs[adv_scan_schd.cur_st].exit(st);

        if (ret) {
            return ret;
        }

        ret = adv_scan_schd_funcs[st].enter(adv_scan_schd.cur_st);

        if (ret) {
            return ret;
        }

        k_mutex_lock(&adv_scan_schd.mutex, K_FOREVER);
        adv_scan_schd.cur_st = st;
        k_mutex_unlock(&adv_scan_schd.mutex);

        return 0;
    }

    return -EINVAL;
}

int bt_mesh_adv_scan_schd_action(adv_scan_schd_action_en action)
{
    int ret;

    if (action < ACTION_INVAILD) {
        adv_scan_schd_state_en cur_st = adv_scan_schd.cur_st;
        adv_scan_schd_state_en target_st = adv_scan_schd_st_change_map[cur_st][action];
        ret = bt_mesh_adv_scan_schd(target_st);

        if (ret) {
            printf("action %d, cur_st %d target_st %d, ret %d\n", action, cur_st, target_st, ret);
        }

        return ret;
    }

    return -EINVAL;
}

void adv_scan_timer(void *timer, void *arg)
{
    int ret;
    static enum  {
        ADV = 0,
        SCAN,
    } next_state = ADV;
    static int adv_time = 0;

    if (adv_scan_schd.flag == FLAG_RESTART) {
        next_state = ADV;
        adv_scan_schd.flag = 0;
    }

    uint32_t next_time = 0;

    if (next_state == ADV) {
        ret = bt_le_scan_stop();

        if (ret && ret != -EALREADY) {
            SCHD_LOGD("scan stop err %d\n", ret);
        }

        struct bt_le_adv_param param = adv_scan_schd.param.adv_param;

        param.interval_min = BT_GAP_ADV_SLOW_INT_MIN;

        param.interval_max = param.interval_min;

        ret = bt_le_adv_start_instant(&param,
                                      adv_scan_schd.param.ad_data, adv_scan_schd.param.ad_len,
                                      adv_scan_schd.param.sd_data, adv_scan_schd.param.sd_len);

        if (ret) {
            SCHD_LOGD("adv start err %d\n", ret);
        }

        next_state = SCAN;
        adv_time =(!(param.options & BT_LE_ADV_OPT_CONNECTABLE))? NOCONN_ADV_DATA_TIEMOUT : CONN_ADV_DATA_TIEMOUT;
        next_time = adv_time;
    } else if (next_state == SCAN) {
        ret = bt_le_adv_stop();

        if (ret && ret != -EALREADY) {
            SCHD_LOGD("adv stop err %d\n", ret);
        }

        next_time = adv_scan_schd.param.adv_param.interval_min * 5 / 8 - adv_time;

        if (next_time > 1) {
            ret = bt_le_scan_start(&adv_scan_schd.param.scan_param, adv_scan_schd.param.scan_cb);

            if (ret) {
                SCHD_LOGD("scan err %d\n", ret);
            }
        }
        adv_time = 0;
        next_state = ADV;
    }

    k_timer_start(&adv_scan_schd.timer, krhino_ms_to_ticks(next_time));
}

int bt_mesh_adv_scan_schd_init()
{
    memset(&adv_scan_schd, 0, sizeof(adv_scan_schd));
    k_timer_init(&adv_scan_schd.timer, adv_scan_timer,  &adv_scan_schd);
    k_mutex_init(&adv_scan_schd.mutex);
    return 0;
}

static int set_ad_data(uint8_t *data, const struct bt_data *ad, size_t ad_len)
{
    int i;
    int set_len = 0;

    for (i = 0; i < ad_len; i++) {
        int len = ad[i].data_len;
        u8_t type = ad[i].type;

        /* Check if ad fit in the remaining buffer */
        if (set_len + len + 2 > 31) {
            len = 31 - (set_len + 2);

            if (type != BT_DATA_NAME_COMPLETE || !len) {
                return -EINVAL;
            }

            type = BT_DATA_NAME_SHORTENED;
        }

        data[set_len++] = len + 1;
        data[set_len++] = type;

        memcpy(&data[set_len], ad[i].data, len);
        set_len += len;
    }

    return set_len;
}

int bt_mesh_adv_start(const struct bt_mesh_le_adv_param *param,
                      const struct bt_mesh_data *ad, size_t ad_len,
                      const struct bt_mesh_data *sd, size_t sd_len)
{
    if (param == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&adv_scan_schd.mutex, K_FOREVER);
    adv_scan_schd.param.adv_param = *(const struct bt_le_adv_param *)param;
    adv_scan_schd.param.ad_len = set_ad_data(adv_scan_schd.param.ad_data, (const struct bt_data *)ad, ad_len);
    adv_scan_schd.param.sd_len = set_ad_data(adv_scan_schd.param.sd_data, (const struct bt_data *)sd, sd_len);
    k_mutex_unlock(&adv_scan_schd.mutex);
    bt_mesh_adv_scan_schd_action(ADV_ON);
    return 0;
}

int bt_mesh_adv_start_instant(const struct bt_mesh_le_adv_param *param,
                      const uint8_t *ad_data, size_t ad_len,
                      const uint8_t *sd_data, size_t sd_len)

{
    if (param == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&adv_scan_schd.mutex, K_FOREVER);
    adv_scan_schd.param.adv_param = *(const struct bt_le_adv_param *)param;
    memcpy(adv_scan_schd.param.ad_data, ad_data, ad_len);
    memcpy(adv_scan_schd.param.sd_data, sd_data, sd_len);
    adv_scan_schd.param.ad_len = ad_len;
    adv_scan_schd.param.sd_len = sd_len;
    k_mutex_unlock(&adv_scan_schd.mutex);
    bt_mesh_adv_scan_schd_action(ADV_ON);
    return 0;
}

int bt_mesh_adv_stop(void)
{
    bt_mesh_adv_scan_schd_action(ADV_OFF);
    return 0;
}

int bt_mesh_scan_start(const struct bt_mesh_le_scan_param *param, bt_mesh_le_scan_cb_t cb)
{
    k_mutex_lock(&adv_scan_schd.mutex, K_FOREVER);
    adv_scan_schd.param.scan_param = *(const struct bt_le_scan_param *)param;
    adv_scan_schd.param.scan_cb = (bt_le_scan_cb_t *)cb;
    k_mutex_unlock(&adv_scan_schd.mutex);
    bt_mesh_adv_scan_schd_action(SCAN_ON);
    return 0;
}

int bt_mesh_scan_stop(void)
{
    bt_mesh_adv_scan_schd_action(SCAN_OFF);
    return 0;
}

#ifdef CONFIG_BT_MESH_MULTIADV
int bt_mesh_multi_adv_start(const struct bt_mesh_le_adv_param *param,
                      const struct bt_mesh_data *ad, size_t ad_len,
                      const struct bt_mesh_data *sd, size_t sd_len, int *instant_id)
{
    return bt_le_multi_adv_start((const struct bt_le_adv_param *)param,
                           (const struct bt_data *)ad, ad_len,
                           (const struct bt_data *)sd, sd_len, instant_id);
}

int bt_mesh_multi_adv_stop(int instant_id)
{
    return bt_le_multi_adv_stop(instant_id);
}
#endif

#else
int bt_mesh_adv_start(const struct bt_mesh_le_adv_param *param,
                      const struct bt_mesh_data *ad, size_t ad_len,
                      const struct bt_mesh_data *sd, size_t sd_len)
{
    return bt_le_adv_start((const struct bt_le_adv_param *)param,
                           (const struct bt_data *)ad, ad_len,
                           (const struct bt_data *)sd, sd_len);
}

int bt_mesh_adv_start_instant(const struct bt_mesh_le_adv_param *param,
                           const uint8_t *ad_data, size_t ad_len,
                           const uint8_t *sd_data, size_t sd_len)

{
    return bt_le_adv_start_instant((const struct bt_le_adv_param *) param, 
                                    ad_data, ad_len,sd_data, sd_len);
}

int bt_mesh_adv_stop(void)
{
    return bt_le_adv_stop();
}

#ifdef CONFIG_BT_MESH_MULTIADV
int bt_mesh_multi_adv_start(const struct bt_mesh_le_adv_param *param,
                      const struct bt_mesh_data *ad, size_t ad_len,
                      const struct bt_mesh_data *sd, size_t sd_len, int *instant_id)
{
    return bt_le_multi_adv_start((const struct bt_le_adv_param *)param,
                           (const struct bt_data *)ad, ad_len,
                           (const struct bt_data *)sd, sd_len, instant_id);
}

int bt_mesh_multi_adv_stop(int instant_id)
{
    return bt_le_multi_adv_stop(instant_id);
}
#endif

int bt_mesh_scan_start(const struct bt_mesh_le_scan_param *param, bt_mesh_le_scan_cb_t cb)
{
    return bt_le_scan_start((const struct bt_le_scan_param *)param, cb);
}

int bt_mesh_scan_stop(void)
{
    return bt_le_scan_stop();
}

#endif

struct bt_conn_cb conn_callbacks;

void bt_mesh_conn_cb_register(struct bt_mesh_conn_cb *cb)
{
    conn_callbacks.connected = cb->connected;
    conn_callbacks.disconnected = cb->disconnected;
    bt_conn_cb_register(&conn_callbacks);
}

bt_mesh_conn_t bt_mesh_conn_ref(bt_mesh_conn_t conn)
{
    return bt_conn_ref((struct bt_conn *)conn);
}

void bt_mesh_conn_unref(bt_mesh_conn_t conn)
{
    bt_conn_unref((struct bt_conn *)conn);
}

int bt_mesh_conn_disconnect(bt_mesh_conn_t conn, uint8_t reason)
{
    return bt_conn_disconnect((struct bt_conn *)conn, reason);
}

#define SVC_ENTRY_MAX 16

struct svc_paire_node {
    struct bt_mesh_gatt_service *msvc;
    struct bt_gatt_service svc;
} _svc_paire[SVC_ENTRY_MAX] = {{0}};

/* TODO: manage the services in linked list. */
int bt_mesh_gatt_service_register(struct bt_mesh_gatt_service *svc)
{
    struct svc_paire_node *node = &_svc_paire[0];
    int i = 0;

    while (i < SVC_ENTRY_MAX) {
        if (node->msvc != NULL) {
            node++;
            i++;
        } else {
            break;
        }
    }

    if (i >= SVC_ENTRY_MAX) {
        printf("Error: no space left for service register.");
        return -1;
    }

    node->msvc = svc;
    node->svc.attrs = (struct bt_gatt_attr *)svc->attrs;
    node->svc.attr_count = svc->attr_count;

    return bt_gatt_service_register(&(node->svc));
}

int bt_mesh_gatt_service_unregister(struct bt_mesh_gatt_service *svc)
{
    struct svc_paire_node *node = &_svc_paire[0];
    int ret, i = 0;

    while (i < SVC_ENTRY_MAX) {
        if (node->msvc != svc) {
            node++;
            i++;
        } else {
            break;
        }
    }

    if (i >= SVC_ENTRY_MAX) {
        return 0;
    }

    ret = bt_gatt_service_unregister(&(node->svc));

    return ret;
}

int bt_mesh_gatt_notify(bt_mesh_conn_t conn, const struct bt_mesh_gatt_attr *attr,
                        const void *data, uint16_t len)
{
    return bt_gatt_notify((struct bt_conn *)conn, (const struct bt_gatt_attr *)attr, data, len);
}

int bt_mesh_gatt_attr_read(bt_mesh_conn_t conn, const struct bt_mesh_gatt_attr *attr,
                           void *buf, uint16_t buf_len, uint16_t offset,
                           const void *value, uint16_t value_len)
{
    return bt_gatt_attr_read((struct bt_conn *)conn, (const struct bt_gatt_attr *)attr, buf, buf_len, offset, value, value_len);
}

uint16_t bt_mesh_gatt_get_mtu(bt_mesh_conn_t conn)
{
    return bt_gatt_get_mtu((struct bt_conn *)conn);
}

int bt_mesh_gatt_attr_read_service(bt_mesh_conn_t conn,
                                   const struct bt_mesh_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read_service((struct bt_conn *)conn, (const struct bt_gatt_attr *)attr, buf, len, offset);
}

int bt_mesh_gatt_attr_read_chrc(bt_mesh_conn_t conn,
                               const struct bt_mesh_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read_chrc((struct bt_conn *)conn, (const struct bt_gatt_attr *)attr, buf, len, offset);
}

#else
void bt_mesh_conn_cb_register(struct bt_mesh_conn_cb *cb)
{
    return;
}

bt_mesh_conn_t bt_mesh_conn_ref(bt_mesh_conn_t conn)
{
    return conn;
}

void bt_mesh_conn_unref(bt_mesh_conn_t conn)
{
    return;
}

int bt_mesh_conn_disconnect(bt_mesh_conn_t conn, uint8_t reason)
{
    return 0;
}

int bt_mesh_gatt_service_register(struct bt_mesh_gatt_service *svc)
{
    return 0;
}

int bt_mesh_gatt_service_unregister(struct bt_mesh_gatt_service *svc)
{
    return 0;
}

int bt_mesh_gatt_notify(bt_mesh_conn_t conn, const struct bt_mesh_gatt_attr *attr,
                        const void *data, uint16_t len)
{
    return 0;
}

int bt_mesh_gatt_attr_read(bt_mesh_conn_t conn, const struct bt_mesh_gatt_attr *attr,
                           void *buf, uint16_t buf_len, uint16_t offset,
                           const void *value, uint16_t value_len)
{
    return 0;
}

uint16_t bt_mesh_gatt_get_mtu(bt_mesh_conn_t conn)
{
    return 0;
}

int bt_mesh_gatt_attr_read_service(bt_mesh_conn_t conn,
                                   const struct bt_mesh_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
    return 0;
}

int bt_mesh_gatt_attr_read_chrc(bt_mesh_conn_t conn,
                               const struct bt_mesh_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset)
{
    return 0;
}

int bt_mesh_adv_start(const struct bt_mesh_le_adv_param *param,
                      const struct bt_mesh_data *ad, size_t ad_len,
                      const struct bt_mesh_data *sd, size_t sd_len)
{
    return 0;
}

int bt_mesh_adv_stop(void)
{
    return 0;
}

int bt_mesh_scan_start(const struct bt_mesh_le_scan_param *param, bt_mesh_le_scan_cb_t cb)
{
    return 0;
}

int bt_mesh_scan_stop(void)
{
    return 0;
}
#endif
