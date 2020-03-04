/********************************************************************************************************
 * @file     hci_driver.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/
#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <zephyr.h>
#include "hal/soc/soc.h"
//#include <init.h>
#include <device.h>
#include <atomic.h>

#include <misc/util.h>
#include <misc/stack.h>
#include <misc/byteorder.h>

#include <bluetooth.h>
#include <hci.h>
#include <hci_driver.h>
//#include "hci_internal.h"

int blc_hci_handler_aos (unsigned char *p_ocf, int n, unsigned char hci_type);
extern void event_callback(uint8_t event_type);;


#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#include "common/log.h"

#if 0   // nrf
#include "util/util.h"
#include "hal/ccm.h"
#include "hal/radio.h"
#include "ll_sw/pdu.h"
#include "ll_sw/ctrl.h"
#include "ll.h"
#include "hci_internal.h"
#include "init.h"
#include "hal/debug.h"
#endif

#define NODE_RX(_node) CONTAINER_OF(_node, struct radio_pdu_node_rx, \
				    hdr.onion.node)

#ifndef CONFIG_CONTROLLER_IN_ONE_TASK
#define CONFIG_BT_CTLR_RX_PRIO_STACK_SIZE 512
#define CONFIG_BT_RX_STACK_SIZE 512
#define CONFIG_BT_CTLR_RX_PRIO 18
static K_SEM_DEFINE(sem_prio_recv, 0, UINT_MAX);
static K_SEM_DEFINE(sem_fifo_recv, 0, UINT_MAX);
static K_FIFO_DEFINE(recv_fifo);

struct k_thread prio_recv_thread_data;
static BT_STACK_NOINIT(prio_recv_thread_stack,
		       CONFIG_BT_CTLR_RX_PRIO_STACK_SIZE);
struct k_thread recv_thread_data;
static BT_STACK_NOINIT(recv_thread_stack, CONFIG_BT_RX_STACK_SIZE);
#endif

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
static struct k_poll_signal hbuf_signal =
		K_POLL_SIGNAL_INITIALIZER(hbuf_signal);
static sys_slist_t hbuf_pend;
static s32_t hbuf_count;
#endif
void blt_rxfifo_poll();
extern u8_t blt_rx_fifo_poll_flag;

int check_adv_map_and_set(u8_t *cmd, u8_t new_map)
{
    u16_t opcode = cmd[0]+cmd[1]*256;
    u8_t ogf = BT_OGF(opcode);
    u8_t ocf = BT_OCF(opcode);
    if(BT_OGF_LE == ogf){
        if(0x06 == ocf){
            cmd[16] = new_map;   // adv map
            return 0;
        }
    }

    return -1;
}

const char hci_cmd_string_CBC[][64] = {
    // 00
    {"HCI_CMD_INVALID_IP_00"},
    {"HCI_CMD_SET_EVENT_MASK"},
    {"HCI_CMD_INVALID_IP_00"},
    {"HCI_CMD_RESET"},
};

const char hci_cmd_string_LE[][64] = {
    // 00
    {"HCI_CMD_INVALID 00"},
    {"HCI_CMD_LE_SET_EVENT_MASK"},
    {"HCI_CMD_LE_READ_BUF_SIZE"},
    {"HCI_CMD_LE_READ_LOCAL_SUPPORTED_FEATURES"},
    // 04
    {"HCI_CMD_INVALID 04"},
    {"HCI_CMD_LE_SET_RANDOM_ADDR"},
    {"HCI_CMD_LE_SET_ADVERTISE_PARAMETERS"},
    {"HCI_CMD_LE_READ_ADVERTISING_CHANNEL_TX_POWER"},
    // 08
    {"HCI_CMD_LE_SET_ADVERTISE_DATA"},
    {"HCI_CMD_LE_SET_SCAN_RSP_DATA"},
    {"HCI_CMD_LE_SET_ADVERTISE_ENABLE"},
    {"HCI_CMD_LE_SET_SCAN_PARAMETERS"},
    // 0C
    {"HCI_CMD_LE_SET_SCAN_ENABLE"},
    {"HCI_CMD_LE_CREATE_CONNECTION"},
    {"HCI_CMD_LE_CREATE_CONNECTION_CANCEL"},
    {"HCI_CMD_LE_READ_WHITE_LIST_SIZE"},
    // 10
    {"HCI_CMD_LE_CLEAR_WHITE_LIST"},
    {"HCI_CMD_LE_ADD_DEVICE_TO_WHITE_LIST"},
    {"HCI_CMD_LE_CONNECTION_UPDATE"},
    {"HCI_CMD_LE_REMOVE_DEVICE_FROM_WL"},
    // 14
    {"HCI_CMD_LE_SET_HOST_CHANNEL_CLASSIFICATION"},
    {"HCI_CMD_LE_READ_CHANNEL_MAP"},
    {"HCI_CMD_LE_READ_REMOTE_USED_FEATURES"},
    {"HCI_CMD_LE_ENCRYPT"},
    // 18
    {"HCI_CMD_LE_RANDOM"},
    {"HCI_CMD_LE_START_ENCRYPTION"},
    {"HCI_CMD_LE_LONG_TERM_KEY_REQUESTED_REPLY"},
    {"HCI_CMD_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY"},
    // 1C
    {"HCI_CMD_LE_READ_SUPPORTED_STATES"},
    {"HCI_CMD_LE_RECEIVER_TEST"},
    {"HCI_CMD_LE_TRANSMITTER_TEST"},
    {"HCI_CMD_LE_TEST_END"},
};

const char hci_cmd_string_IP[][64] = {
    // 00
    {"HCI_CMD_INVALID IP 00"},
    {"HCI_CMD_READ_LOCAL_VER_INFO"},
    {"HCI_CMD_READ_LOCAL_SUPPORTED_CMDS"},
    {"HCI_CMD_READ_LOCAL_SUPPORTED_FEATURES"},
    // 04
    {"HCI_CMD_READ_EXTENDED_LOCAL_SUPPORTED_FEATURES"},
    {"HCI_CMD_READ_BUFFER_SIZE_COMMAND"},
    {"HCI_CMD_INVALID IP 06"},
    {"HCI_CMD_INVALID IP 07"},
    // 08
    {"HCI_CMD_INVALID IP 08"},
    {"HCI_CMD_READ_BD_ADDR"},
};

const char hci_cmd_string_VENDOR[][64] = {
    // 00
    {" BT_HCI_INVALID_VENDOR_00"},
    {" BT_HCI_OP_VS_READ_VERSION_INFO"},
    {" BT_HCI_OP_VS_READ_SUPPORTED_COMMANDS"},
    {" BT_HCI_OP_VS_READ_SUPPORTED_FEATURES"},
};

const char * get_hci_cmd_string__(u8_t *cmd)
{
	u16_t opcode = cmd[0]+cmd[1]*256;
    u8_t ogf = BT_OGF(opcode);
    u8_t ocf = BT_OCF(opcode);
    if(BT_OGF_BASEBAND == ogf){
        if(ocf < ARRAY_SIZE(hci_cmd_string_CBC)){
            return hci_cmd_string_CBC[ocf];
        }
        return "HCI_CMD_XXXXXX_UNKNOW_CBC";
    }else if(BT_OGF_LE == ogf){
        if(ocf < ARRAY_SIZE(hci_cmd_string_LE)){
            return hci_cmd_string_LE[ocf];
        }
        return "HCI_CMD_XXXXXX_UNKNOW_LE";
    }else if(BT_OGF_INFO == ogf){
        if(ocf < ARRAY_SIZE(hci_cmd_string_IP)){
            return hci_cmd_string_IP[ocf];
        }
        return "HCI_CMD_XXXXXX_UNKNOW_IP";
    }else if(BT_OGF_VS == ogf){
        if(ocf < ARRAY_SIZE(hci_cmd_string_VENDOR)){
            return hci_cmd_string_VENDOR[ocf];
        }
        return "HCI_CMD_XXXXXX_UNKNOW_VENDOR";
    }
    
    return "HCI_CMD_XXXXXX_UNKNOW";
}

const char * get_hci_cmd_string(u8_t *cmd)
{
    return (get_hci_cmd_string__(cmd) + 8); // remove "HCI_CMD_"
}


int hci_driver_recv(void)
{
    //u8_t r = tlk_irq_disable();
    blt_rxfifo_poll();  // will call blc_hci_send_event() --> blc_hci_send_event_aos_cb, when there is data or event
    blt_rx_fifo_poll_flag = 0;
    //tlk_irq_resrote(r);
    return 0;
}

#ifndef CONFIG_CONTROLLER_IN_ONE_TASK
static void prio_recv_thread(void *p1, void *p2, void *p3)
{
	while (1) {
		struct radio_pdu_node_rx *node_rx;
		u8_t num_cmplt;
		u16_t handle;

		while ((num_cmplt = radio_rx_get(&node_rx, &handle))) {
#if defined(CONFIG_BT_CONN)
			struct net_buf *buf;

			buf = bt_buf_get_rx(BT_BUF_EVT, K_FOREVER);
			hci_num_cmplt_encode(buf, handle, num_cmplt);
			BT_DBG("Num Complete: 0x%04x:%u", handle, num_cmplt);
			bt_recv_prio(buf);
			k_yield();
#endif
		}

		if (node_rx) {

			radio_rx_dequeue();

			BT_DBG("RX node enqueue");
			k_fifo_put(&recv_fifo, node_rx);
            k_sem_give(&sem_fifo_recv);

			continue;
		}

		BT_DBG("sem take...");
		k_sem_take(&sem_prio_recv, K_FOREVER);
		BT_DBG("sem taken");

#if defined(CONFIG_INIT_STACKS)
		if (k_uptime_get_32() - prio_ts > K_SECONDS(5)) {
			STACK_ANALYZE("prio recv thread stack",
				      prio_recv_thread_stack);
			prio_ts = k_uptime_get_32();
		}
#endif
	}
}
#endif

#if 0
static inline struct net_buf *encode_node(struct radio_pdu_node_rx *node_rx,
					  s8_t class)
{
	struct net_buf *buf = NULL;

	/* Check if we need to generate an HCI event or ACL data */
	switch (class) {
	case HCI_CLASS_EVT_DISCARDABLE:
	case HCI_CLASS_EVT_REQUIRED:
	case HCI_CLASS_EVT_CONNECTION:
		if (class == HCI_CLASS_EVT_DISCARDABLE) {
			buf = bt_buf_get_rx(BT_BUF_EVT, K_NO_WAIT);
		} else {
			buf = bt_buf_get_rx(BT_BUF_EVT, K_FOREVER);
		}
		if (buf) {
			hci_evt_encode(node_rx, buf);
		}
		break;
#if defined(CONFIG_BT_CONN)
	case HCI_CLASS_ACL_DATA:
		/* generate ACL data */
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
		hci_acl_encode(node_rx, buf);
		break;
#endif
	default:
		LL_ASSERT(0);
		break;
	}

	radio_rx_fc_set(node_rx->hdr.handle, 0);
	node_rx->hdr.onion.next = 0;
	radio_rx_mem_release(&node_rx);

	return buf;
}

static inline struct net_buf *process_node(struct radio_pdu_node_rx *node_rx)
{
	s8_t class = hci_get_class(node_rx);
	struct net_buf *buf = NULL;

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
	if (hbuf_count != -1) {
		bool pend = !sys_slist_is_empty(&hbuf_pend);

		/* controller to host flow control enabled */
		switch (class) {
		case HCI_CLASS_EVT_DISCARDABLE:
		case HCI_CLASS_EVT_REQUIRED:
			break;
		case HCI_CLASS_EVT_CONNECTION:
			/* for conn-related events, only pend is relevant */
			hbuf_count = 1;
			/* fallthrough */
		case HCI_CLASS_ACL_DATA:
			if (pend || !hbuf_count) {
				sys_slist_append(&hbuf_pend,
						 &node_rx->hdr.onion.node);
				BT_DBG("FC: Queuing item: %d", class);
				return NULL;
			}
			break;
		default:
			LL_ASSERT(0);
			break;
		}
	}
#endif

	/* process regular node from radio */
	buf = encode_node(node_rx, class);

	return buf;
}

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
static inline struct net_buf *process_hbuf(struct radio_pdu_node_rx *n)
{
	/* shadow total count in case of preemption */
	struct radio_pdu_node_rx *node_rx = NULL;
	s32_t hbuf_total = hci_hbuf_total;
	struct net_buf *buf = NULL;
	sys_snode_t *node = NULL;
	s8_t class;
	int reset;

	reset = atomic_test_and_clear_bit(&hci_state_mask, HCI_STATE_BIT_RESET);
	if (reset) {
		/* flush queue, no need to free, the LL has already done it */
		sys_slist_init(&hbuf_pend);
	}

	if (hbuf_total <= 0) {
		hbuf_count = -1;
		return NULL;
	}

	/* available host buffers */
	hbuf_count = hbuf_total - (hci_hbuf_sent - hci_hbuf_acked);

	/* host acked ACL packets, try to dequeue from hbuf */
	node = sys_slist_peek_head(&hbuf_pend);
	if (!node) {
		return NULL;
	}

	/* Return early if this iteration already has a node to process */
	node_rx = NODE_RX(node);
	class = hci_get_class(node_rx);
	if (n) {
		if (class == HCI_CLASS_EVT_CONNECTION ||
		    (class == HCI_CLASS_ACL_DATA && hbuf_count)) {
			/* node to process later, schedule an iteration */
			BT_DBG("FC: signalling");
			k_poll_signal(&hbuf_signal, 0x0);
		}
		return NULL;
	}

	switch (class) {
	case HCI_CLASS_EVT_CONNECTION:
		BT_DBG("FC: dequeueing event");
		(void) sys_slist_get(&hbuf_pend);
		break;
	case HCI_CLASS_ACL_DATA:
		if (hbuf_count) {
			BT_DBG("FC: dequeueing ACL data");
			(void) sys_slist_get(&hbuf_pend);
		} else {
			/* no buffers, HCI will signal */
			node = NULL;
		}
		break;
	case HCI_CLASS_EVT_DISCARDABLE:
	case HCI_CLASS_EVT_REQUIRED:
	default:
		LL_ASSERT(0);
		break;
	}

	if (node) {
		buf = encode_node(node_rx, class);
		/* Update host buffers after encoding */
		hbuf_count = hbuf_total - (hci_hbuf_sent - hci_hbuf_acked);
		/* next node */
		node = sys_slist_peek_head(&hbuf_pend);
		if (node) {
			node_rx = NODE_RX(node);
			class = hci_get_class(node_rx);

			if (class == HCI_CLASS_EVT_CONNECTION ||
			    (class == HCI_CLASS_ACL_DATA && hbuf_count)) {
				/* more to process, schedule an
				 * iteration
				 */
				BT_DBG("FC: signalling");
				k_poll_signal(&hbuf_signal, 0x0);
			}
		}
	}

	return buf;
}
#endif
#endif

#ifndef CONFIG_CONTROLLER_IN_ONE_TASK
static void recv_thread(void *p1, void *p2, void *p3)
{
#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
	/* @todo: check if the events structure really needs to be static */
	static struct k_poll_event events[2] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
						K_POLL_MODE_NOTIFY_ONLY,
						&hbuf_signal, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&recv_fifo, 0),
	};
#endif

	while (1) {
		struct radio_pdu_node_rx *node_rx = NULL;
		struct net_buf *buf = NULL;

		BT_DBG("blocking");
#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
		int err;

		err = k_poll(events, 2, K_FOREVER);
		LL_ASSERT(err == 0);
		if (events[0].state == K_POLL_STATE_SIGNALED) {
			events[0].signal->signaled = 0;
		} else if (events[1].state ==
			   K_POLL_STATE_FIFO_DATA_AVAILABLE) {
			node_rx = k_fifo_get(events[1].fifo, 0);
		}

		events[0].state = K_POLL_STATE_NOT_READY;
		events[1].state = K_POLL_STATE_NOT_READY;

		/* process host buffers first if any */
		buf = process_hbuf(node_rx);

#else
        k_sem_take(&sem_fifo_recv, K_FOREVER);
		node_rx = k_fifo_get(&recv_fifo, K_FOREVER);
#endif
		BT_DBG("unblocked");

		if (node_rx && !buf) {
			/* process regular node from radio */
			buf = process_node(node_rx);
		}

		if (buf) {
			if (buf->len) {
				BT_DBG("Packet in: type:%u len:%u",
					bt_buf_get_type(buf), buf->len);
				bt_recv(buf);
			} else {
				net_buf_unref(buf);
			}
		}

		k_yield();

#if defined(CONFIG_INIT_STACKS)
		if (k_uptime_get_32() - rx_ts > K_SECONDS(5)) {
			STACK_ANALYZE("recv thread stack", recv_thread_stack);
			rx_ts = k_uptime_get_32();
		}
#endif
	}
}
#endif


static int hci_driver_send(struct net_buf *buf)
{
	u8_t type;
	int err = 0;

	BT_DBG("enter");
    //printf("xxxx hci_driver_send 1\r\n");

	if (!buf->len) {
        //printf("xxxx hci_driver_send 2\r\n");
		BT_ERR("Empty HCI packet");
        //printf("hci_driver_send len error %s: %d, %s\r\n", get_hci_cmd_string(buf->data),buf->len,bt_hex(buf->data,buf->len));
		return -EINVAL;
	}

#if 0 // test 880090
	static volatile unsigned int sys_0test880090 = 1;
	if(sys_0test880090){
	    int *p = (int*)soc_get_cur_sp();
	    for(unsigned int i = 0; i < (0xf000000); ++i){
            if(i%4==0) {
                printf("\r\n%08x:",(uint32_t)p);
            }
            printf("%08x ", *p);
            p++;
            static volatile unsigned int sys_0test_cnt;sys_0test_cnt++;
        }
        sys_0test880090 = 0;
	}
#endif

	type = bt_buf_get_type(buf);
	switch (type) {
#if defined(CONFIG_BT_CONN)
	case BT_BUF_ACL_OUT:
        //printf("hci send ACL  : %2d, %s\r\n", buf->len,bt_hex(buf->data,buf->len));
	    blc_hci_handler_aos(buf->data, buf->len, type);
		break;
#endif /* CONFIG_BT_CONN */
	case BT_BUF_CMD:
        #if 0
        {
            u8_t a[4] = {0x0a,0x20,0x01,0x00};  // adv disable
            if(!memcmp(a, buf->data, 4)){
                //buf->data[3] = 1;
            }
        }
        #endif
	    //check_adv_map_and_set(buf->data, 0x02);
        //printf("hci send CMD %s: %d, %s\r\n", get_hci_cmd_string(buf->data),buf->len,bt_hex(buf->data,buf->len));//min(buf->len, 10)
	    blc_hci_handler_aos(buf->data, buf->len, type);
        
		break;
	default:
		BT_ERR("Unknown HCI type %u", type);
		return -EINVAL;
	}

	if (!err) {
            net_buf_unref(buf);
	}

	BT_DBG("exit: %d", err);

	return err;
}

void pkt_recv_callback(void)
{
#ifdef CONFIG_CONTROLLER_IN_ONE_TASK
    //unsigned int key;
    extern struct k_poll_signal g_pkt_recv;

    //key = irq_lock();
    g_pkt_recv.signaled = 1;
    //irq_unlock(key);

    event_callback(K_POLL_TYPE_DATA_RECV);
#else
    k_sem_give(&sem_prio_recv);
#endif
}

static int hci_driver_open(void)
{
#if 1
    return 0;
#else
	u32_t err;

	DEBUG_INIT();
#ifndef CONFIG_CONTROLLER_IN_ONE_TASK
        k_sem_init(&sem_prio_recv, 0, UINT_MAX);
        k_sem_init(&sem_fifo_recv, 0, UINT_MAX);
#endif
	err = ll_init(pkt_recv_callback);
	if (err) {
		BT_ERR("LL initialization failed: %u", err);
		return err;
	}

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
	hci_init(&hbuf_signal);
#else
	hci_init(NULL);
#endif

#ifndef CONFIG_CONTROLLER_IN_ONE_TASK
    k_fifo_init(&recv_fifo);

	k_thread_create(&prio_recv_thread_data, prio_recv_thread_stack,
			K_THREAD_STACK_SIZEOF(prio_recv_thread_stack),
			prio_recv_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_CTLR_RX_PRIO), 0, K_NO_WAIT);

	k_thread_create(&recv_thread_data,  recv_thread_stack,
			K_THREAD_STACK_SIZEOF(recv_thread_stack),
			recv_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_RX_PRIO), 0, K_NO_WAIT);
#endif
	BT_DBG("Success.");
	return 0;
#endif
}

static const struct bt_hci_driver drv = {
	.name	= "Controller",
	.bus	= BT_HCI_DRIVER_BUS_VIRTUAL,
	.open	= hci_driver_open,
	.send	= hci_driver_send,
    .recv = hci_driver_recv,
};

int hci_driver_init()
{

	bt_hci_driver_register(&drv);

	return 0;
}


