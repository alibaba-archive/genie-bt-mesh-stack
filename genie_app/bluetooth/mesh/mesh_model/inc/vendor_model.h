/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _VENDOR_MODEL_H
#define _VENDOR_MODEL_H

#define VENDOR_OP_ATTR_GET_STATUS        0xD0
#define VENDOR_OP_ATTR_SET_ACK           0xD1
#define VENDOR_OP_ATTR_SET_UNACK         0xD2
#define VENDOR_OP_ATTR_STATUS            0xD3
#define VENDOR_OP_ATTR_INDICATE          0xD4
#define VENDOR_OP_ATTR_CONFIME           0xD5
#define VENDOR_OP_ATTR_INDICATE_TG       0xDE
#define VENDOR_OP_ATTR_CONFIME_TG        0xDF
#define VENDOR_OP_ATTR_TRANS_MSG         0xCF
#define VENDOR_OP_ATTR_TRANS_INDICATE    0xCE
#define VENDOR_OP_ATTR_TRANS_ACK         0xCD

#define ONOFF_T           0x0100
#define LIGHTNESS_T       0x0121
#define TEMPERATURE_T     0x0122
#define DEVICE_EVENT_T    0xF009

/* Event List - Refer to Device Event List defined in https://yuque.antfin-inc.com/iotconnect/wwfb/dbyytw#1c38cf1b */
#define EL_FAULT_T        0x00 /* malfunction event */
#define EL_LOW_BAT_T      0x01 /* low battery event */
#define EL_DEV_UP_T       0x03 /* device power up */
#define EL_HW_RESET_T     0x23 /* hardware reset event */

/* Vendor timer error code */
#define ERR_CODE_UNIXTIME                 0x80
#define ERR_CODE_NOTSUP_ATTR_OP           0x82
#define ERR_CODE_NOTSUP_ATTR_PARAM        0x83
#define ERR_CODE_TIMER_SETTING            0x84
#define ERR_CODE_TIMER_INDEX              0x85
#define ERR_CODE_TIMER_FULL               0x86
#define ERR_CODE_TIMER_PRIORDIC_PARAM     0x87

/* Vendor timer ali attr type */
#define ERROR_CODE_T                0x0000
#define TIMING_TIMEOUT_T            0xF009
#define TIMING_SETTING_T            0xF010
#define TIMING_PERIODIC_SETTING_T   0xF011
#define TIMING_DELETE_T             0xF012
#define TIMING_SYNC_T               0xF01D
#define TIMEZONE_SETTING_T          0xF01E
#define UNIX_TIME_T                 0xF01F

#define EL_TIMING_TIMEOUT_T 0x11 /* timer timeout event */

#define VENDOR_MODEL_MSG_DFT_RETRY_TIMES 6
#define VENDOR_MODEL_MSG_MAX_RETRY_TIMES 10
#define VENDOR_MODEL_MSG_RETRY_PERIOD 400

/**
 * p_elem: pointer to the element which the messsage want to be sent to
 * retry: retry counts before desired confirm message received
 * * fill negative value if retransmission is not necessary
 * * fill positive value if retransmission is needed
 * * * will be round in this scope - [VENDOR_MODEL_MSG_DFT_RETRY_TIMES, VENDOR_MODEL_MSG_MAX_RETRY_TIMES]
 * retry_period: wait for retry_period before retransmit the message, in unit of ms
 * opid: hightest byte in Opcode defined in the vendor model spec designed by Alibaba IoT Group
 * * e.g. for Vendor Message Attr Get message, Opcode is 0xD001A8, corresponding opid is 0xD0
 * * refer to the marco named VENDOR_OP_ATTR_xxxx
 * tid:
 * * if the message is with type of VENDOR_OP_ATTR_CONFIME or VENDOR_OP_ATTR_CONFIME_TG,
 * * tid should be filled with the replied message's tid
 * len: payload length
 * data: pointer to the vendor message's payload
 * */
typedef struct _vnd_model_msg {
    struct bt_mesh_elem *p_elem;
    s8_t  retry;
    u16_t  retry_period;
    u8_t  opid;
    u8_t  tid;
    u16_t len;
    u8_t  *data;
} vnd_model_msg;

/**
 * @brief send vendor model message
 * @param[in] p_model_msg refers to the vendor_model_msg struct
 * @return 0 for success, otherwise for failed reason
 */
s16_t vendor_model_msg_send(vnd_model_msg *p_model_msg);

#endif //_VENDOR_MODEL_H
