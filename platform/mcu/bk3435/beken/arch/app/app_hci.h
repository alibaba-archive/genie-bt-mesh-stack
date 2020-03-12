/**
 ****************************************************************************************
 *
 * @file app_hci.h
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2019-2020
 *
 *
 ****************************************************************************************
 */

#ifndef _APP_HCI_H_
#define _APP_HCI_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <buf.h>
#include "co_list.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///HCI Transport Header length - change if different transport
#define HCI_TRANSPORT_HDR_LEN                       0x01

///UART header: command message type
#define HCI_CMD_MSG_TYPE                            0x01

///UART header: ACL data message type
#define HCI_ACL_MSG_TYPE                            0x02

///UART header: Synchronous data message type
#define HCI_SYNC_MSG_TYPE                           0x03

///UART header: event message type
#define HCI_EVT_MSG_TYPE                            0x04

///UART header: event message type
#define HCI_TCI_MSG_TYPE                            0xFF

#define HCI_RECV_BUF_LEN	CONFIG_BT_RX_BUF_LEN

#define HCI_LOG		printf
#define HCI_INFO	//HCI_LOG
#define HCI_DBG		//HCI_LOG

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#pragma pack(push, 1) /* push current alignment to stack & set alignment to 1 byte boundary */
typedef struct recv_buf_info_s {
    /// list header
    struct co_list_hdr hdr;

    struct net_buf *nbuf;
	uint8_t  type;
    uint32_t buf_len;
	union {
		uint8_t  buf[2];
		struct { // hci event format
			uint8_t  event;
			uint8_t  para_len;
			uint8_t  para[];
		} evt;
	};
} app_recv_buf_info_t;

typedef struct hci_buf_s {
	struct net_buf *nbuf;
	uint8_t  cont;
	uint8_t  type;
    uint32_t buf_len;
	union {
		uint8_t  buf[HCI_RECV_BUF_LEN];
		struct { // hci command format
			uint16_t opcode;
			uint8_t  para_len;
			uint8_t  para[];
		} cmd;
		struct { // hci acl format
			uint16_t handle;
			uint16_t data_len;
			uint8_t  data[];
		} acl;
		struct { // hci event format
			uint8_t  event;
			uint8_t  para_len;
			uint8_t  para[];
		} evt;
	};
} app_hci_buf_t;
#pragma pack(pop) /* restore original alignment from stack */

typedef uint8_t (*hci_recv_cb_t) (app_hci_buf_t*);


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

//uint16_t appm_hci_send(app_hci_buf_t *hci_send);
uint16_t appm_hci_recv_reg_callback(hci_recv_cb_t callback);
uint16_t appm_hci_send_to_controller(app_hci_buf_t *hci_send);


#endif /* _APP_HCI_H_ */
