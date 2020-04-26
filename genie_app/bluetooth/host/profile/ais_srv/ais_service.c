/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <port/mesh_hal_ble.h>

#include "genie_app.h"
#include "multi_adv.h"
#include "ali_dfu_port.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_OTA)
#include "common/log.h"

#define AIS_OTA_AUTH_TIMEOUT 10000      //10s
#define AIS_OTA_DISCONN_TIMEOUT 60000   //60s
#define AIS_OTA_REPORT_TIMEOUT (CONFIG_AIS_TOTAL_FRAME*400)
#define AIS_OTA_REBOOT_TIMEOUT 3000     //3s

#define AIS_SERVICE_UUID           BT_UUID_DECLARE_16(0xFEB3)
#define AIS_READ_UUID              BT_UUID_DECLARE_16(0xFED4)
#define AIS_WRITE_UUID             BT_UUID_DECLARE_16(0xFED5)
#define AIS_INDICATE_UUID          BT_UUID_DECLARE_16(0xFED6)
#define AIS_WRITE_WO_RESP_UUID     BT_UUID_DECLARE_16(0xFED7)
#define AIS_NOTIFY_UUID            BT_UUID_DECLARE_16(0xFED8)

enum {
    AIS_RESP_ERR = 0x0F,
    AIS_SCRT_RANDOM = 0x10,
    AIS_SCRT_CIPHER = 0x11,
    AIS_SCRT_RESULT = 0x12,
    AIS_SCRT_ACK = 0x13,
    AIS_LINK_STATUS = 0x14,
    AIS_LINK_ACK = 0x15,
    AIS_OTA_VER_REQ = 0x20,
    AIS_OTA_VER_RESP= 0x21,
    AIS_OTA_UPD_REQ = 0x22,
    AIS_OTA_UPD_RESP = 0x23,
    AIS_OTA_STATUS = 0x24,
    AIS_OTA_CHECK_REQ = 0x25,
    AIS_OTA_CHECK_RESP = 0x26,
    AIS_OTA_DATA = 0x2f,
    AIS_CMD_MAX = 0xff
};

//AIS PDU FORMAT
typedef struct {
    uint8_t msg_id: 4;
    uint8_t enc: 1;
    uint8_t ver: 3;
    uint8_t cmd;
    uint8_t seq: 4;
    uint8_t total_frame: 4;
    uint8_t payload_len;
}  __attribute__ ((packed))  ais_header_t;

typedef struct {
    uint8_t random[16];
} __attribute__ ((packed)) ais_scrt_random_t;

typedef struct {
    uint8_t cipher[16];
} __attribute__ ((packed)) ais_scrt_cipher_t;

typedef struct {
    uint8_t result;
} __attribute__ ((packed)) ais_scrt_result_t;

typedef struct {
    uint8_t ack;
} __attribute__ ((packed)) ais_scrt_ack_t;

typedef struct {
    uint8_t image_type;
} __attribute__ ((packed)) ais_ota_ver_req_t;

typedef struct {
    uint8_t image_type;
    uint32_t ver;
} __attribute__ ((packed)) ais_ota_ver_resp_t;

typedef struct {
    uint8_t image_type;
    uint32_t ver;
    uint32_t fw_size;
    uint16_t crc16;
    uint8_t ota_flag;
} __attribute__ ((packed)) ais_ota_upd_req_t;

typedef struct {
    uint8_t state;
    uint32_t rx_size;
    uint8_t total_frame;
} __attribute__ ((packed)) ais_ota_upd_resp_t;

typedef struct {
    uint8_t last_seq: 4;
    uint8_t total_frame: 4;
    uint32_t rx_size;
}  __attribute__ ((packed))  ais_ota_status_report_t;

typedef struct {
    uint8_t state;
}  __attribute__ ((packed))  ais_ota_check_req_t;

typedef struct {
    uint8_t state;
} __attribute__ ((packed)) ais_ota_check_resp_t;


typedef struct {
    ais_header_t header;
    uint8_t payload[16];
} __attribute__ ((packed)) ais_pdu_t;

typedef struct {
    uint8_t err_count;

    uint8_t last_seq;
    uint8_t total_frame;
    uint8_t except_seq;
    uint32_t rx_size;
    uint8_t len_4B;
    uint8_t data_4B[4];

    uint8_t image_type;
    uint32_t image_ver;
    uint32_t image_size;
    uint16_t image_crc16;
    uint8_t ota_flag;
} ota_info_t;

enum{
    AIS_STATE_DISCON,
    AIS_STATE_CONNECT,
    AIS_STATE_AUTH,
    AIS_STATE_IDLE,
    AIS_STATE_OTA,
    AIS_STATE_REBOOT,
};

enum{
    OTA_FLAG_SILENT = 0x02,
};

typedef struct {
    //uint8_t auth;
    //uint8_t ota_process;
    //uint8_t disconnect;
    uint8_t state;
    struct bt_conn *p_conn;
    k_timer_t timer;
    ota_info_t ota_info;
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    uint8_t flash_clean:1;
#endif
} ais_srv_ctx_t;

static ais_srv_ctx_t g_ais_srv_ctx; 

static struct bt_gatt_ccc_cfg ais_ic_ccc_cfg[BT_GATT_CCC_MAX] = {};
static struct bt_gatt_ccc_cfg ais_nc_ccc_cfg[BT_GATT_CCC_MAX] = {};
static struct bt_gatt_attr _ais_srv_attrs[];
static struct bt_gatt_indicate_params *p_indicate = NULL;
static uint8_t g_ais_conn = 0;

extern bool ota_check_reboot(void);

bool ais_get_ota_indicat(void)
{
    uint8_t ota_indicat = 0xFF;
    genie_flash_read_userdata(GFI_OTA_INDICAT, &ota_indicat, sizeof(ota_indicat));
    if(ota_indicat == 1) {
        return 1;
    }
    return 0;
}

void ais_clear_ota_indicat(void)
{
    genie_flash_delete_userdata(GFI_OTA_INDICAT);
}

void ais_check_ota_change(void)
{
#ifdef CONFIG_GENIE_OTA_PINGPONG
    uint32_t ota_change = 0xFFFFFFFF;
    uint8_t ota_image = DFU_IMAGE_ERR;
    E_GENIE_FLASH_ERRCODE ret;
    ret = genie_flash_read_reliable(GFI_OTA_IMAGE_CHANGE, (uint8_t *)&ota_change, sizeof(ota_change));
    if(ret == GENIE_FLASH_SUCCESS) {
        ota_image = ota_change & 0xFF;
        if((ota_change>>8) == 0xA5A5A5 && ota_image < DFU_IMAGE_ERR) {
            if(ota_image != get_program_image()) {
                BT_DBG("switch to %d", ota_image);
                if(change_program_image(ota_image) == ota_image) {
                    genie_flash_delete_reliable(GFI_OTA_IMAGE_CHANGE);
                    hal_reboot();
                }
            }
        }
        BT_DBG("del change %d", ota_image);
        genie_flash_delete_reliable(GFI_OTA_IMAGE_CHANGE);
    }
#else
    genie_flash_delete_reliable(GFI_OTA_IMAGE_CHANGE);
#endif
}

static void _ais_set_ota_change(void)
{
    uint8_t ota_indicat = 1;

    genie_flash_write_userdata(GFI_OTA_INDICAT, &ota_indicat, sizeof(ota_indicat));

#ifdef CONFIG_GENIE_OTA_PINGPONG
    uint32_t ota_change = 0xA5A5A500;
    uint8_t ota_image = DFU_IMAGE_TOTAL - get_program_image();
    if(ota_image < DFU_IMAGE_ERR) {
        ota_change |= ota_image;
        BT_DBG("switch to %d", ota_image);
        genie_flash_write_reliable(GFI_OTA_IMAGE_CHANGE, (uint8_t *)&ota_change, sizeof(ota_change));
    } else {
        BT_ERR("image err");
    }
#else
    uint32_t ota_change = 0xA5A5A501;
    genie_flash_write_reliable(GFI_OTA_IMAGE_CHANGE, (uint8_t *)&ota_change, sizeof(ota_change));
#endif
}

static void _ais_ota_encrypt(uint8_t *payload, uint8_t len)
{
    uint8_t enc[16];
    BT_DBG("payload: %s", bt_hex(payload, 16));
    genie_ais_encrypt(payload, enc);
    memcpy(payload, enc, 16);
    //BT_DBG("enc: %s", bt_hex(payload, 16));
}

static void _ais_ota_decrypt(uint8_t *payload, uint8_t len)
{
    uint8_t dec[16];
    //BT_DBG("payload: %s", bt_hex(payload, 16));
    genie_ais_decrypt(payload, dec);
    memcpy(payload, dec, 16);
    BT_DBG("dec: %s", bt_hex(payload, 16));
}

static void _ais_indicate_rsp(struct bt_conn *conn, const struct bt_gatt_attr *attr, u8_t err)
{
    BT_DBG("err 0x%02x", err);
    if(p_indicate) {
        //BT_DBG("free %p", p_indicate);
        aos_free(p_indicate);
        p_indicate = NULL;
    }
}

static void _ais_server_indicate(uint8_t msg_id, uint8_t cmd, uint8_t *p_msg, uint16_t len)
{
    ais_pdu_t msg;

    BT_DBG("msg_id %02x %02x", msg_id, cmd);
    if(!p_indicate) {
        p_indicate = aos_malloc(sizeof(struct bt_gatt_indicate_params));
    }

    if(!p_indicate) {
        BT_ERR("no mem");
        return;
    }

    memset(&msg, 0, sizeof(msg));
    if(g_ais_srv_ctx.state == AIS_STATE_IDLE) {
        msg.header.enc = 1;
    } else {
        msg.header.enc = 0;
    }
    msg.header.msg_id = msg_id;
    msg.header.cmd = cmd;
    msg.header.payload_len = len;
    if(p_msg) {
        memcpy(msg.payload, p_msg, len);
    }
    
    BT_DBG("len %d: %s", len+4, bt_hex(&msg, len+4));

    //indicate._req
    p_indicate->attr = &_ais_srv_attrs[6];
    p_indicate->func = _ais_indicate_rsp;
    p_indicate->data = &msg;
    p_indicate->len = len + 4;
    bt_gatt_indicate(g_ais_srv_ctx.p_conn, p_indicate);
}

static void _ais_server_notify(uint8_t msg_id, uint8_t cmd, uint8_t *p_msg, uint16_t len)
{
    ais_pdu_t msg;

    BT_DBG("msg_id %02x %02x", msg_id, cmd);

    memset(&msg, 0, sizeof(msg));
    if(g_ais_srv_ctx.state >= AIS_STATE_IDLE && g_ais_srv_ctx.state <= AIS_STATE_REBOOT) {
        msg.header.enc = 1;
    } else {
        msg.header.enc = 0;
    }
    msg.header.msg_id = msg_id;
    msg.header.cmd = cmd;
    msg.header.payload_len = len;
    if(p_msg) {
        memcpy(msg.payload, p_msg, len);
    }

    BT_DBG("len %d: %s", len+4, bt_hex(&msg, len+4));

    bt_gatt_notify(g_ais_srv_ctx.p_conn, &_ais_srv_attrs[11], &msg, len+4);
}

void ais_ota_disconnect(uint8_t reason)
{
    g_ais_conn = 0;
    if(g_ais_srv_ctx.state != AIS_STATE_REBOOT) {
        BT_DBG("disconnect reason 0x%x", reason);
        k_timer_stop(&g_ais_srv_ctx.timer);
        bt_conn_disconnect(g_ais_srv_ctx.p_conn, reason);
        g_ais_srv_ctx.state = AIS_STATE_DISCON;
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
        /* Flash is dirty, need erase */
        if (g_ais_srv_ctx.flash_clean == 0) {
            erase_dfu_flash();
            g_ais_srv_ctx.flash_clean = 1;
        }
#endif
        /* restart adv */
        genie_event(GENIE_EVT_SDK_AIS_DISCON, NULL);
    }
}

static void _ais_timer_refresh(void)
{
    BT_DBG_R(" %d", g_ais_srv_ctx.state);
    if(!g_ais_conn) {
        BT_DBG_R("connect from proxy, ignore");
        return;
    }
    switch(g_ais_srv_ctx.state) {
        case AIS_STATE_DISCON:
            ais_ota_disconnect(BT_HCI_ERR_UNACCEPT_CONN_PARAM);
            break;
        case AIS_STATE_CONNECT:
        case AIS_STATE_IDLE:
            k_timer_start(&g_ais_srv_ctx.timer, AIS_OTA_DISCONN_TIMEOUT);
            break;
        case AIS_STATE_AUTH:
            k_timer_start(&g_ais_srv_ctx.timer, AIS_OTA_AUTH_TIMEOUT);
            break;
        case AIS_STATE_OTA:
            k_timer_start(&g_ais_srv_ctx.timer, AIS_OTA_REPORT_TIMEOUT);
            break;
        case AIS_STATE_REBOOT:
            k_timer_start(&g_ais_srv_ctx.timer, AIS_OTA_REBOOT_TIMEOUT);
            break;
        default:
            break;
    }
}

void ais_connect(struct bt_conn *p_conn)
{
    BT_DBG("status %d", g_ais_srv_ctx.state);
    g_ais_srv_ctx.p_conn = p_conn;
    g_ais_srv_ctx.state = AIS_STATE_CONNECT;
    _ais_timer_refresh();
}

static void _ais_ota_status_report(void)
{
    uint8_t payload[16];
    ais_ota_status_report_t *p_status_report = (ais_ota_status_report_t *)payload;

    memset(payload, 11, sizeof(payload));
    p_status_report->last_seq = g_ais_srv_ctx.ota_info.last_seq;
    p_status_report->total_frame = g_ais_srv_ctx.ota_info.total_frame;
    p_status_report->rx_size = g_ais_srv_ctx.ota_info.rx_size;

    BT_DBG_R("last[%d] total[%d] size[%d] err[%d]", p_status_report->last_seq, p_status_report->total_frame, p_status_report->rx_size, g_ais_srv_ctx.ota_info.err_count);
    _ais_ota_encrypt(payload, 16);
    
    _ais_server_notify(0, AIS_OTA_STATUS, payload, 16);
}

static void _ais_ota_timer_cb(void *p_timer, void *args)
{
    BT_DBG_R(" %d", g_ais_srv_ctx.state);
    switch(g_ais_srv_ctx.state) {
        case AIS_STATE_CONNECT:
            case AIS_STATE_IDLE:
            g_ais_srv_ctx.state = AIS_STATE_DISCON;
            break;
        case AIS_STATE_AUTH:
            genie_ais_reset();
            g_ais_srv_ctx.state = AIS_STATE_CONNECT;
            break;
        case AIS_STATE_OTA:
            //BT_DBG_R("");
            if(g_ais_srv_ctx.ota_info.err_count++ >= 5) {
                BT_DBG_R("OTA failed");
                g_ais_srv_ctx.state = AIS_STATE_IDLE;
                _ais_server_indicate(0, AIS_RESP_ERR, NULL, 0);
            } else {
                _ais_ota_status_report();
            }
            break;
        case AIS_STATE_REBOOT:
            if(g_ais_srv_ctx.ota_info.ota_flag != OTA_FLAG_SILENT || ota_check_reboot()) {
                bt_conn_disconnect(g_ais_srv_ctx.p_conn, BT_HCI_ERR_SUCCESS);
                //clear image change
                genie_flash_delete_reliable(GFI_OTA_IMAGE_CHANGE);
                dfu_reboot();
            } else {
                BT_DBG_R("silent");
                g_ais_srv_ctx.state = AIS_STATE_IDLE;
            }
            break;
        default:
            break;
    }
    _ais_timer_refresh();
}

static bool _ais_scrt_random(uint8_t msg_id, ais_scrt_random_t *p_scrt_random)
{
    uint8_t cipher[16];

    g_ais_srv_ctx.state = AIS_STATE_AUTH;
    genie_ais_get_cipher(p_scrt_random->random, cipher);
    _ais_server_indicate(msg_id, AIS_SCRT_CIPHER, cipher, 16);

    return true;
}

static bool _ais_scrt_result(uint8_t msg_id, ais_scrt_result_t *p_scrt_result)
{
    uint8_t ack = 0;

    if(p_scrt_result->result == 1) {
        genie_ais_reset();
        g_ais_srv_ctx.state = AIS_STATE_CONNECT;
    } else {
        g_ais_srv_ctx.state = AIS_STATE_IDLE;
    }

    _ais_server_indicate(msg_id, AIS_SCRT_ACK, &ack, 1);

    return true;
}

static bool _ais_link_ack(uint8_t msg_id, ais_scrt_result_t *p_scrt_result)
{
    uint8_t payload[16];

    if(p_scrt_result->result == 0) {
        genie_ais_reset();
        g_ais_srv_ctx.state = AIS_STATE_CONNECT;
    }
    memset(payload, 0x0F, sizeof(payload));
    payload[0] = 1;
    _ais_ota_encrypt(payload, 16);

    _ais_server_indicate(msg_id, AIS_LINK_ACK, payload, 16);

    return true;
}

static bool _ais_ota_ver_req(uint8_t msg_id, ais_ota_ver_req_t *p_ver_req)
{
    if(p_ver_req->image_type == 0){
        uint8_t payload[16];
        ais_ota_ver_resp_t *p_ver_resp = (ais_ota_ver_resp_t *)payload;

        memset(payload, 11, sizeof(payload));
        p_ver_resp->image_type = 0;
        p_ver_resp->ver = PROJECT_SW_VERSION;

        if(g_ais_srv_ctx.state == AIS_STATE_IDLE) {
            _ais_ota_encrypt(payload, 16);
            _ais_server_notify(msg_id, AIS_OTA_VER_RESP, payload, 16);
        } else {
            _ais_server_notify(msg_id, AIS_OTA_VER_RESP, payload, 5);
        }
        return true;
    }
    return false;
}

static bool _ais_ota_upd_req(uint8_t msg_id, ais_ota_upd_req_t *p_ota_req)
{
    uint32_t tmp_size = DFU_IMAGE_SIZE_MAX;
    uint8_t payload[16];
    ais_ota_upd_resp_t *p_upd_resp = (ais_ota_upd_resp_t *)payload;

    BT_DBG("ota_ver %08x size %d temp: %d", p_ota_req->ver, p_ota_req->fw_size, tmp_size);

    memset(payload, 10, sizeof(payload));
    if (p_ota_req->image_type != 0 || p_ota_req->ver <= PROJECT_SW_VERSION ||
        tmp_size < p_ota_req->fw_size || 0 == p_ota_req->fw_size) {
        p_upd_resp->state = 0;
        p_upd_resp->rx_size = 0;
    } else {
        g_ais_srv_ctx.state = AIS_STATE_OTA;
        memset(&g_ais_srv_ctx.ota_info, 0, sizeof(g_ais_srv_ctx.ota_info));
        g_ais_srv_ctx.ota_info.image_type = p_ota_req->image_type;
        g_ais_srv_ctx.ota_info.image_ver = p_ota_req->ver;
        g_ais_srv_ctx.ota_info.image_size = p_ota_req->fw_size;
        g_ais_srv_ctx.ota_info.image_crc16 = p_ota_req->crc16;
        g_ais_srv_ctx.ota_info.ota_flag = p_ota_req->ota_flag;
        p_upd_resp->state = 1;
        //TODO duandianxuchuan   req:flag crc/resp:size
        p_upd_resp->rx_size = 0;
#ifdef BOARD_TC825X
        extern void bls_ota_clearNewFwDataArea(void);
        //bls_ota_clearNewFwDataArea();
#endif
    }
    p_upd_resp->total_frame = CONFIG_AIS_TOTAL_FRAME - 1;
    _ais_ota_encrypt(payload, 16);

    _ais_server_notify(msg_id, AIS_OTA_UPD_RESP, payload, 16);

    return true;
}

static bool _ais_ota_data(ais_pdu_t *p_msg)
{
    uint8_t offset = 0;
    uint8_t *p_payload = p_msg->payload;
    uint16_t payload_len = p_msg->header.payload_len;

    if(p_msg->header.seq > p_msg->header.total_frame) {
        BT_ERR("invalid");
        return false;
    }

    if (p_msg->header.seq != g_ais_srv_ctx.ota_info.except_seq) {
        BT_DBG("expected %d, rx %d, len %d", g_ais_srv_ctx.ota_info.except_seq, p_msg->header.seq, p_msg->header.payload_len);
        if(g_ais_srv_ctx.ota_info.err_count++ == 0) {
            /* send fail */
            _ais_ota_status_report();
            /* refresh timer */
            return true;
        }
        return false;
    }

    BT_DBG("b4:rx %d/%d", g_ais_srv_ctx.ota_info.rx_size, g_ais_srv_ctx.ota_info.image_size);
    if(g_ais_srv_ctx.ota_info.rx_size + p_msg->header.payload_len > g_ais_srv_ctx.ota_info.image_size) {
        BT_ERR("out of size, rx %d, recv %d", g_ais_srv_ctx.ota_info.rx_size, p_msg->header.payload_len);
        return false;
    }

    g_ais_srv_ctx.ota_info.err_count = 0;

    if (g_ais_srv_ctx.ota_info.len_4B) {
        memcpy(g_ais_srv_ctx.ota_info.data_4B + g_ais_srv_ctx.ota_info.len_4B,
                p_msg->payload, 4 - g_ais_srv_ctx.ota_info.len_4B);

        //BT_DBG_R("save 4B.2 %d", 4 - g_ais_srv_ctx.ota_info.len_4B);
        unlock_flash_all();
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
        g_ais_srv_ctx.flash_clean = 0;
#endif
        ali_dfu_image_update(g_ais_srv_ctx.ota_info.image_type,
                       g_ais_srv_ctx.ota_info.rx_size - g_ais_srv_ctx.ota_info.len_4B, 4,
                       (int *)g_ais_srv_ctx.ota_info.data_4B);
        lock_flash();

        offset = 4 - g_ais_srv_ctx.ota_info.len_4B;
        p_payload += offset;
        payload_len -= (payload_len<offset) ? payload_len : offset;
    }

    g_ais_srv_ctx.ota_info.len_4B = payload_len & 0x0003;
    payload_len = payload_len & 0xfffc;
    if (payload_len) {
        //BT_DBG_R("save %d", payload_len);
        unlock_flash_all();
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
        g_ais_srv_ctx.flash_clean = 0;
#endif
        ali_dfu_image_update(g_ais_srv_ctx.ota_info.image_type,
                        g_ais_srv_ctx.ota_info.rx_size + offset, payload_len,
                        (int *)p_payload);
        lock_flash();
    }

    if (g_ais_srv_ctx.ota_info.len_4B) {
        memcpy(g_ais_srv_ctx.ota_info.data_4B, p_payload + payload_len, g_ais_srv_ctx.ota_info.len_4B);
        //BT_DBG_R("4B.1 %d", g_ais_srv_ctx.ota_info.len_4B);
    }

    g_ais_srv_ctx.ota_info.last_seq = p_msg->header.seq;
    g_ais_srv_ctx.ota_info.total_frame = p_msg->header.total_frame;
    g_ais_srv_ctx.ota_info.rx_size += p_msg->header.payload_len;
    BT_DBG("rx %d/%d", g_ais_srv_ctx.ota_info.rx_size, g_ais_srv_ctx.ota_info.image_size);
    if(p_msg->header.seq == p_msg->header.total_frame) {
        g_ais_srv_ctx.ota_info.except_seq = 0;
    } else {
        g_ais_srv_ctx.ota_info.except_seq = p_msg->header.seq + 1;
    }

    if(g_ais_srv_ctx.ota_info.rx_size == g_ais_srv_ctx.ota_info.image_size
                        || p_msg->header.seq == p_msg->header.total_frame) {
        _ais_ota_status_report();
    }

    return true;
}

static bool _ais_ota_check_req(uint8_t msg_id, ais_ota_check_req_t *p_check_req)
{
    g_ais_srv_ctx.state = AIS_STATE_IDLE;

    if(p_check_req->state == 1) {
        uint8_t payload[16];
        uint16_t crc16;
        ais_ota_check_resp_t *p_check_resp = (ais_ota_check_resp_t *)payload;

        memset(payload, 15, sizeof(payload));

        p_check_resp->state = dfu_check_checksum(g_ais_srv_ctx.ota_info.image_type, &crc16);


        BT_DBG("check %d %04x %04x", p_check_resp->state, g_ais_srv_ctx.ota_info.image_crc16, crc16);
        if(p_check_resp->state && crc16 != g_ais_srv_ctx.ota_info.image_crc16){
            p_check_resp->state = 0;
            BT_DBG_R("crc error");
        }

        if (p_check_resp->state) {
            g_ais_srv_ctx.state = AIS_STATE_REBOOT;
            _ais_set_ota_change();
            BT_DBG("ota success, reboot in 3s!");
        } else {
            BT_DBG("ota failed");
        }

        _ais_ota_encrypt(payload, 16);
        _ais_server_notify(msg_id, AIS_OTA_CHECK_RESP, payload, 16);

        return true;
    } else {
        return false;
    }
}

static bool _ais_msg_check_header(ais_header_t *p_msg_header)
{
    //check seq & total, in ota case, the seq & total must be 0
    if(p_msg_header->total_frame != 0 || p_msg_header->seq != 0
                            || p_msg_header->ver != 0
                            || p_msg_header->seq > p_msg_header->total_frame) {
        BT_DBG("fail %s", bt_hex(p_msg_header, sizeof(ais_header_t)));
        return false;
    }
    return true;
}

static void _ais_server_msg_handle(struct bt_conn *p_conn, ais_pdu_t *p_msg, uint16_t len)
{
    bool timer_refresh = false;

    BT_DBG("cmd %02x", p_msg->header.cmd);
    BT_DBG("len %d: %s", len, bt_hex(p_msg, len));

    if(p_msg->header.cmd != AIS_OTA_DATA && !_ais_msg_check_header((ais_header_t *)p_msg)) {
        BT_ERR("invalid msg, ignore");
    }

    switch (p_msg->header.cmd) {
        case AIS_SCRT_RANDOM:
            //len = 4+16
            if(len == 20 && (g_ais_srv_ctx.state == AIS_STATE_CONNECT
                          || g_ais_srv_ctx.state == AIS_STATE_IDLE)) {
                timer_refresh = _ais_scrt_random(p_msg->header.msg_id,
                                        (ais_scrt_random_t *)p_msg->payload);
            }
            break;

        case AIS_SCRT_RESULT:
            //len = 4+1
            if(len == 5 && g_ais_srv_ctx.state == AIS_STATE_AUTH) {
                timer_refresh = _ais_scrt_result(p_msg->header.msg_id,
                                        (ais_scrt_result_t *)p_msg->payload);
            }
            break;

        case AIS_LINK_STATUS:
            //len = 4+16
            if(len == 20 && g_ais_srv_ctx.state == AIS_STATE_IDLE) {
                _ais_ota_decrypt(p_msg->payload, 16);
                timer_refresh = _ais_link_ack(p_msg->header.msg_id,
                                        (ais_scrt_result_t *)p_msg->payload);
            }
            break;

        case AIS_OTA_VER_REQ:
            if((len == 20 && g_ais_srv_ctx.state == AIS_STATE_IDLE)
             || (len == 5 && g_ais_srv_ctx.state == AIS_STATE_CONNECT)) {
                if(g_ais_srv_ctx.state == AIS_STATE_IDLE) {
                    _ais_ota_decrypt(p_msg->payload, 16);
                }
                timer_refresh = _ais_ota_ver_req(p_msg->header.msg_id,
                                        (ais_ota_ver_req_t *)p_msg->payload);
            }
            break;

        case AIS_OTA_UPD_REQ:
            //len = 4+16
            if(len == 20 && g_ais_srv_ctx.state == AIS_STATE_IDLE) {
                _ais_ota_decrypt(p_msg->payload, 16);
                timer_refresh = _ais_ota_upd_req(p_msg->header.msg_id,
                                        (ais_ota_upd_req_t *)p_msg->payload);
            }
            break;

        case AIS_OTA_DATA:
            if(len == sizeof(ais_header_t) + p_msg->header.payload_len
                && p_msg->header.ver == 0 && g_ais_srv_ctx.state == AIS_STATE_OTA) {
                timer_refresh = _ais_ota_data(p_msg);
            }
            break;

        case AIS_OTA_CHECK_REQ:
            if(len == 20 && g_ais_srv_ctx.state == AIS_STATE_OTA) {
                _ais_ota_decrypt(p_msg->payload, 16);
                timer_refresh = _ais_ota_check_req(p_msg->header.msg_id, (ais_ota_check_req_t *)p_msg->payload);
            }
            break;

        default:
            break;
    }
    //BT_DBG("ret %d", ret);
    if (timer_refresh) {
        _ais_timer_refresh();
    } else {
        if(g_ais_srv_ctx.state != AIS_STATE_OTA) {
            _ais_server_indicate(p_msg->header.msg_id, AIS_RESP_ERR, NULL, 0);
        }
    }
}

static ssize_t _ais_server_read(struct bt_conn *p_conn, const struct bt_gatt_attr *p_attr,
                                        void *buf, u16_t len, u16_t offset)
{
    u16_t *value = p_attr->user_data;

    BT_DBG("len %d: %s", len, bt_hex(buf, len));

    return bt_gatt_attr_read(p_conn, p_attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t _ais_service_write(struct bt_conn *p_conn, const struct bt_gatt_attr *p_attr,
                                        const void *p_buf, u16_t len, u16_t offset, u8_t flags)
{
    //BT_DBG("len %d: %s", len, bt_hex(p_buf, len));

    if(len != 0) {
        _ais_server_msg_handle(p_conn, (ais_pdu_t *)p_buf, len);
    }

    return len;
}

static ssize_t _ais_service_write_nr(struct bt_conn *p_conn, const struct bt_gatt_attr *p_attr,
                                            const void *p_buf, u16_t len, u16_t offset, u8_t flags)
{
    //BT_DBG("len %d: %s", len, bt_hex(p_buf, len));
    return _ais_service_write(p_conn, p_attr, p_buf, len, offset, flags);
}

static void _ais_service_ccc_cfg_changed(const struct bt_gatt_attr *p_attr, uint16_t value)
{
    if(value) {
        g_ais_conn = 1;
    }
    BT_DBG("value %d", value);
}

/* AIS OTA Service Declaration */
static struct bt_gatt_attr _ais_srv_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(AIS_SERVICE_UUID),

    BT_GATT_CHARACTERISTIC(AIS_READ_UUID, BT_GATT_CHRC_READ),
    BT_GATT_DESCRIPTOR(AIS_READ_UUID, BT_GATT_PERM_READ, _ais_server_read, NULL, NULL),

    BT_GATT_CHARACTERISTIC(AIS_WRITE_UUID, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE),
    BT_GATT_DESCRIPTOR(AIS_WRITE_UUID, BT_GATT_PERM_WRITE, NULL, _ais_service_write, NULL),

    BT_GATT_CHARACTERISTIC(AIS_INDICATE_UUID, BT_GATT_CHRC_READ|BT_GATT_CHRC_INDICATE),
    BT_GATT_DESCRIPTOR(AIS_INDICATE_UUID,  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, _ais_server_read, NULL, NULL),

    BT_GATT_CCC(ais_ic_ccc_cfg, _ais_service_ccc_cfg_changed),

    BT_GATT_CHARACTERISTIC(AIS_WRITE_WO_RESP_UUID, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
    BT_GATT_DESCRIPTOR(AIS_WRITE_WO_RESP_UUID, BT_GATT_PERM_WRITE, NULL, _ais_service_write_nr, NULL),

    BT_GATT_CHARACTERISTIC(AIS_NOTIFY_UUID, BT_GATT_CHRC_READ |  BT_GATT_CHRC_NOTIFY),
    BT_GATT_DESCRIPTOR(AIS_NOTIFY_UUID, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, _ais_server_read, NULL, NULL),

    BT_GATT_CCC(ais_nc_ccc_cfg, _ais_service_ccc_cfg_changed),
};

static struct bt_gatt_service _ais_srv = BT_GATT_SERVICE(_ais_srv_attrs);

static u8_t g_ais_adv_data[14] = {
    0xa8, 0x01, //taobao
    0x85,       //vid & sub
    0x15,       //FMSK
    0x15, 0x11, 0x22, 0x33,             //PID
    0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33  //MAC
};

struct bt_data g_ais_adv[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME, 0xB3, 0xFE),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, g_ais_adv_data, 14),
};

static const struct bt_data g_ais_sd[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

struct bt_le_adv_param fast_adv_param = {
    .options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME),\
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,\
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,   \
    .own_addr = NULL, \
};

int g_multiadv_instant_id;
void bt_gatt_adv_init(void)
{
    BT_DBG("");

#ifdef CONFIG_BT_MESH_MULTIADV
    {
        int err;
        genie_ais_adv_init(g_ais_adv_data, 0);

//        printk("%s multi adv start\n");
        err = bt_le_multi_adv_start(&fast_adv_param, g_ais_adv, ARRAY_SIZE(g_ais_adv), g_ais_sd, ARRAY_SIZE(g_ais_sd), &g_multiadv_instant_id);
//        printk("%s multi adv end %d\n", instant_id);
        if (err) {
            BT_ERR("Multi Advertising failed to start (err %d)\n", err);
        }
    }
#else
#if 0
    k_thread_create(&gatt_adv_thread_data, gatt_adv_thread_stack,
        K_THREAD_STACK_SIZEOF(gatt_adv_thread_stack), ota_adv_thread,
        NULL, NULL, NULL, CONFIG_BT_MESH_ADV_PRIO, 0, K_NO_WAIT);
#endif
#endif
}

int ais_service_register(void)
{
    BT_DBG_R("");
    bt_gatt_adv_init();

    bt_gatt_service_register(&_ais_srv);

    memset(&g_ais_srv_ctx, 0, sizeof(g_ais_srv_ctx));
    k_timer_init(&g_ais_srv_ctx.timer, _ais_ota_timer_cb, NULL);

#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    erase_dfu_flash();
    g_ais_srv_ctx.flash_clean = 1;
#endif
    return 0;
}


