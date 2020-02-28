
#include <stddef.h>
#include <zephyr/types.h>
#include <string.h>
//#include <version.h>

#include "hal/soc/soc.h"
#include <toolchain.h>
#include <errno.h>
#include <atomic.h>
#include <hci.h>
#include <hci_vs.h>
#include <buf.h>
#include <bluetooth.h>
#include "../../../../../network/bluetooth/bt_host/include/hci_driver.h"


#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#include "common/log.h"
//#include "hal/debug.h"

/* opcode of the HCI command currently being processed. The opcode is stored
 * by hci_cmd_handle() and then used during the creation of cmd complete and
 * cmd status events to avoid passing it up the call chain.
 */
//static u16_t _opcode;

#if CONFIG_BT_CTLR_DUP_FILTER_LEN > 0
/* Scan duplicate filter */
struct dup {
	u8_t         mask;
	bt_addr_le_t addr;
};
static struct dup dup_filter[CONFIG_BT_CTLR_DUP_FILTER_LEN];
static s32_t dup_count;
static u32_t dup_curr;
#endif

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
s32_t    hci_hbuf_total;
u32_t    hci_hbuf_sent;
u32_t    hci_hbuf_acked;
u16_t    hci_hbuf_pend[CONFIG_BT_MAX_CONN];
atomic_t hci_state_mask;
static struct k_poll_signal *hbuf_signal;
#endif


#define DEFAULT_EVENT_MASK           0x1fffffffffff
#define DEFAULT_EVENT_MASK_PAGE_2    0x0
#define DEFAULT_LE_EVENT_MASK 0x1f


int bt_recv_prio(struct net_buf *buf);

void blc_hci_send_data_aos(u8_t hci_evt_cmd, u8_t *para, u8_t n, int prio_flag)
{
    //printf("xxxx blc_hci_send_data_aos 0x%02x,0x%02x,%s\r\n",hci_evt_cmd, n, bt_hex(para,n));
    struct net_buf *buf;
	buf = bt_buf_get_cmd_complete(K_FOREVER);
	//u8_t r = tlk_irq_disable();
	//printf("cmd_complete len 1:%d,%s\r\n", (buf)->len, bt_hex((buf)->data,(buf)->len));
	//printf("cmd_complete buf:%s\r\n", bt_hex((buf),32));
	u8_t *p_data = net_buf_add(buf, 1); // sizeof(hci_evt_cmd)
	*p_data = hci_evt_cmd;
	p_data = net_buf_add(buf, 1); // sizeof(n)
	*p_data = n;
	p_data = net_buf_add(buf, n);
	memcpy(p_data, para, n);
	//printf("cmd_complete buf:%s\r\n", bt_hex((buf),32));
	//sleep_us(20*1000);
	//tlk_irq_resrote(r);
    if(prio_flag){
        //printf("hci complete to host, len:%2d,data:%s\r\n", (buf)->len, bt_hex((buf)->data,(buf)->len));
        bt_recv_prio(buf);
    }else{
        //printf("hci RX EVENT to host, len:%2d,data:%s\r\n", (buf)->len, bt_hex((buf)->data,(buf)->len));
        bt_recv(buf);
    }
}

#define L2CAP_CONTINUING_PKT             0x01

void blc_hci_send_data_aos_acl_in(u16_t hci_evt_cmd, u8_t *para, int n)
{
    if(n <= 2){
        return ;
    }
    // para[0] = type; para[1] = rf_len;

    n = para[1];    // rf_len. if segment, it has been re-calculate.
    struct net_buf *buf;
    buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
	u8_t *p_data = net_buf_add(buf, 2); // sizeof(hci_evt_cmd)
	p_data[0] = hci_evt_cmd & 0xff;
	p_data[1] = ((hci_evt_cmd >> 8) & 0xff) | ((para[0]&3) == L2CAP_CONTINUING_PKT ? 0x10 : 0x20);  //start llid 2 ->0x20 ;  continue llid 1 ->0x10;
	p_data = net_buf_add(buf, 2); // sizeof(n)
	p_data[0] = n & 0xff;
	p_data[1] = (n >> 8) & 0xff;
	p_data = net_buf_add(buf, n);
	memcpy(p_data, para+2, n);
	
    // printf("hci RX ACL len: %d, %s\r\n", n,bt_hex(buf->data,buf->len));
    bt_recv(buf);
}

struct net_buf *hci_cmd_handle(struct net_buf *cmd)
{
    return 0;
}

int hci_acl_handle(struct net_buf *buf, struct net_buf **evt)
{
    return 0;
}

