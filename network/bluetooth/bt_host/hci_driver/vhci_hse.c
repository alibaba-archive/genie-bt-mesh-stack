#include <errno.h>
#include <stddef.h>

#include <zephyr.h>

#include <misc/util.h>
#include <misc/byteorder.h>
#include <string.h>

#ifndef BT_DBG_ENABLED
#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BLUETOOTH_DEBUG_HCI_DRIVER)
#endif
#include <bluetooth.h>
#include <hci.h>
#include <hci_driver.h>
#include <common/log.h>
#include "hal_lld.h"

#define H4_NONE 0x00
#define H4_CMD  0x01
#define H4_ACL  0x02
#define H4_SCO  0x03
#define H4_EVT  0x04

#define HSE_KEVENT_FLAGS 0x0001

static kevent_t hse_kevent;

static struct {
    uint8_t type;
    struct k_fifo   fifo;
} tx;

void hse_signal_set(void);

static void hc_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    hse_schedule();
}

static int vhci_send(struct net_buf *buf)
{
    BT_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

    net_buf_put(&tx.fifo, buf);
    /* wakeup hc thread */
    hse_signal_set();

    return 0;
}

static int vhci_open(void)
{
    static struct k_thread vhci_hc_thread;
    static BT_STACK_NOINIT(hc_thread_stack, CONFIG_BT_HCI_RX_STACK_SIZE);

    BT_INFO("");

    krhino_event_create(&hse_kevent, "hse", HSE_KEVENT_FLAGS);

    k_thread_create(&vhci_hc_thread, /*"hc",*/ hc_thread_stack, sizeof(hc_thread_stack), hc_thread,
                    NULL, NULL, NULL, 1/*prio*/, 0, K_NO_WAIT);

    k_fifo_init(&tx.fifo);

    return 0;
}

static struct bt_hci_driver vhci_drv = {
    .name       = "VHCI",
    .bus        = BT_HCI_DRIVER_BUS_UART,
    .open       = vhci_open,
    .send       = vhci_send,
};

void hse_vhci_rx_handler(uint8_t type, uint8_t *pkt, uint16_t len)
{
    struct net_buf *buf;
    bool prio;
    uint8_t evt = ((struct bt_hci_evt_hdr *)pkt)->evt;

    BT_DBG("type 0x%02x, evt 0x%02x", type, evt);

    if (type == H4_EVT && (evt == BT_HCI_EVT_CMD_COMPLETE ||
                           evt == BT_HCI_EVT_CMD_STATUS)) {
        buf = bt_buf_get_cmd_complete(K_FOREVER);
    }
    else {
        buf = bt_buf_get_rx(BT_BUF_EVT, K_FOREVER);
    }

    memcpy(net_buf_tail(buf), pkt, len);
    net_buf_add(buf, len);

    prio = (type == H4_EVT && bt_hci_evt_is_prio(evt));
    if (type == H4_EVT) {
        bt_buf_set_type(buf, BT_BUF_EVT);
    } else {
        bt_buf_set_type(buf, BT_BUF_ACL_IN);
    }
    if (prio) {
        BT_DBG("Calling bt_recv_prio(%p)", buf);
        bt_recv_prio(buf);
    } else {
        BT_DBG("Putting buf %p to rx fifo", buf);
        bt_recv(buf);
    }
}

void hse_vhci_tx_handler(void)
{
    struct net_buf *buf;
    uint8_t type;
    uint8_t *p_pkt;

    buf = net_buf_get(&tx.fifo, K_NO_WAIT);
    if (NULL == buf) return;

    switch (bt_buf_get_type(buf)) {
    case BT_BUF_ACL_OUT:
        type = H4_ACL;
        break;

    case BT_BUF_CMD:
        type = H4_CMD;
        break;

    default:
        BT_ERR("Unknown buffer type");
        goto done;
    }

    BT_DBG("write type=%02x: data %s", type, bt_hex(buf->data, buf->len));
    /* trick: store the h4 hci packet type in the reserved headroom */
    p_pkt = buf->data-1;
    *p_pkt = type;
    hci_virtual_alloc_send(p_pkt, buf->len+1);

    net_buf_pull(buf, buf->len);

done:
    net_buf_unref(buf);

    return 0;
}

void hse_signal_wait(void)
{
    uint32_t actl_flags;
    krhino_event_get(&hse_kevent, HSE_KEVENT_FLAGS, RHINO_AND_CLEAR, &actl_flags, RHINO_WAIT_FOREVER);
}
void hse_signal_set(void)
{
    krhino_event_set(&hse_kevent, HSE_KEVENT_FLAGS, RHINO_OR);
}

int hci_driver_init()
{
    BT_INFO("hci_driver_init hse");
    hse_init();
    /* BT_IRQn is enabled in SYSirq_Initialise(), so change irq priority here */
    nvicEnableVector(BT_IRQn, HS_BT_IRQ_PRIORITY);
    bt_hci_driver_register(&vhci_drv);
    return 0;
}
