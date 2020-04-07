/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <zephyr.h>
#include "errno.h"

#include <misc/slist.h>
#include <bluetooth.h>
#include <hci_driver.h>
#include <common/log.h>
#include <misc/byteorder.h>
#include <arch.h>
#include "aos/kernel.h"

#include "uart_pub.h"
#include "app_hci.h"

extern const uint16_t stack_support_flash;

//static uint8_t ble_bd_addr[6] = { 0x78, 0xda, 0x07, 0xc1, 0x1c, 0xa8 };
extern uint8_t g_mac[];
static uint8_t *ble_bd_addr = g_mac;

static cpu_stack_t hci_recv_prio_stack[CONFIG_BK_HCI_RECV_PRIO_TASK_STACK_SIZE];
static cpu_stack_t hci_recv_stack[CONFIG_BK_HCI_RECV_TASK_STACK_SIZE];
static cpu_stack_t ble_hci_stack[CONFIG_BK_HCI_TASK_STACK_SIZE];

ble_hdr_arg_t  g_ble_hdr_arg={0};


static ktask_t ble_hci_recv_prio_hdr;
static ktask_t ble_hci_recv_hdr;
static ktask_t ble_hci_hdr;
static ksem_t  recv_prio_sem;
static ksem_t  recv_sem;
static struct co_list recv_prio_wait_list;
static struct co_list recv_wait_list;
uint16_t wait_recv_prio_cnt;
uint16_t wait_recv_cnt;

struct hci_context
{
    aos_mutex_t lock;
    int         initialized;
};
static struct hci_context g_hci_send_context;

/*
 * GLOBAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
static void hci_lock(struct hci_context *context)
{
    if (context->initialized)
    {
        aos_mutex_lock(&(context->lock), AOS_WAIT_FOREVER);
    }
}

static void hci_unlock(struct hci_context *context)
{
    if (context->initialized)
    {
        aos_mutex_unlock(&(context->lock));
    }
}

void ble_recv_prio_handler(void *arg)
{
	co_list_init(&recv_prio_wait_list);

	krhino_sem_create(&recv_prio_sem, "ble_hdl_recv_prio", 0);

	app_recv_buf_info_t *recv_buf_info;

	wait_recv_prio_cnt = 0;

	while(1)
	{
		///check have receive data
		while(!co_list_is_empty(&recv_prio_wait_list))
		{
			///get receive buffer
			GLOBAL_INT_DISABLE();
			recv_buf_info = (app_recv_buf_info_t*)co_list_pop_front(&recv_prio_wait_list);
			if(wait_recv_prio_cnt) wait_recv_prio_cnt--;
			GLOBAL_INT_RESTORE();

			uint8_t net_buf_null_flag = 0;
	        while(!recv_buf_info->nbuf) {
				///get net_buf
				if(recv_buf_info->type == HCI_EVT_MSG_TYPE && (recv_buf_info->evt.event == BT_HCI_EVT_CMD_COMPLETE ||
						recv_buf_info->evt.event == BT_HCI_EVT_CMD_STATUS)) {
					recv_buf_info->nbuf = bt_buf_get_cmd_complete(K_NO_WAIT);
				}
				else {
					recv_buf_info->nbuf = bt_buf_get_rx(BT_BUF_EVT, K_NO_WAIT);
				}

				///check no get net_buf
				if(!recv_buf_info->nbuf && !net_buf_null_flag) {
					printf("hci get net_buf is NULL!\r\n");
					net_buf_null_flag = 1;
				}
	        }

	        ///fill net_buf
			net_buf_add_mem(recv_buf_info->nbuf, recv_buf_info->buf, recv_buf_info->buf_len);
			if (recv_buf_info->type == HCI_EVT_MSG_TYPE) {
				bt_buf_set_type(recv_buf_info->nbuf, BT_BUF_EVT);
			} else {
				bt_buf_set_type(recv_buf_info->nbuf, BT_BUF_ACL_IN);
			}

			{
				HCI_DBG("Calling bt_recv_prio(%p), len:%d\r\n", recv_buf_info->nbuf, recv_buf_info->nbuf->len);
				bt_recv_prio(recv_buf_info->nbuf);
			}

	        krhino_mm_free(recv_buf_info);
		}

        krhino_sem_take(&recv_prio_sem, CONFIG_BK_HCI_TASK_SEM_TIMEROUT);
	}
}

void ble_recv_handler(void *arg)
{
	co_list_init(&recv_wait_list);

	krhino_sem_create(&recv_sem, "ble_hdl_recv", 0);

	app_recv_buf_info_t *recv_buf_info;

	wait_recv_cnt = 0;

	while(1)
	{
		///check have receive data
		while(!co_list_is_empty(&recv_wait_list))
		{
			///get receive buffer
			GLOBAL_INT_DISABLE();
			recv_buf_info = (app_recv_buf_info_t*)co_list_pop_front(&recv_wait_list);
			if(wait_recv_cnt) wait_recv_cnt--;
			GLOBAL_INT_RESTORE();

			uint8_t net_buf_null_flag = 0;
	        while(!recv_buf_info->nbuf) {
				///get net_buf
				if(recv_buf_info->type == HCI_EVT_MSG_TYPE && (recv_buf_info->evt.event == BT_HCI_EVT_CMD_COMPLETE ||
						recv_buf_info->evt.event == BT_HCI_EVT_CMD_STATUS)) {
					recv_buf_info->nbuf = bt_buf_get_cmd_complete(K_NO_WAIT);
				}
				else {
					recv_buf_info->nbuf = bt_buf_get_rx(BT_BUF_EVT, K_NO_WAIT);
				}

				///check no get net_buf
				if(!recv_buf_info->nbuf && !net_buf_null_flag) {
					printf("hci get net_buf is NULL!\r\n");
					net_buf_null_flag = 1;
				}
	        }

	        ///fill net_buf
			net_buf_add_mem(recv_buf_info->nbuf, recv_buf_info->buf, recv_buf_info->buf_len);
			if (recv_buf_info->type == HCI_EVT_MSG_TYPE) {
				bt_buf_set_type(recv_buf_info->nbuf, BT_BUF_EVT);
			} else {
				bt_buf_set_type(recv_buf_info->nbuf, BT_BUF_ACL_IN);
			}

			{
				HCI_DBG("Putting buf %p to rx fifo, len:%d\r\n", recv_buf_info->nbuf, recv_buf_info->nbuf->len);
				bt_recv(recv_buf_info->nbuf);
			}

	        krhino_mm_free(recv_buf_info);
		}

        /* Give other threads a chance to run */
        //k_yield();

        krhino_sem_take(&recv_sem, CONFIG_BK_HCI_TASK_SEM_TIMEROUT);
	}
}

static uint8_t hci_driver_recv(app_hci_buf_t *recv_buf)
{
    uint8_t  err = 0;
    ksem_t   *give_sem;
    uint16_t *wait_cnt;
    struct co_list *wait_list;

    do {
        ///check no support the command
        if(!recv_buf) {
            HCI_DBG("hci driver no support the command!\r\n");
            err = -1;    break;
        }

        if (recv_buf->type == HCI_EVT_MSG_TYPE) {
            HCI_INFO("rx evt:0x%x, len:%d\r\n", recv_buf->evt.event, recv_buf->evt.para_len);
        }
        else if (recv_buf->type == HCI_ACL_MSG_TYPE) {
            HCI_INFO("rx acl:0x%x, len:%d\r\n", recv_buf->acl.handle, recv_buf->acl.data_len);
        }

		if(recv_buf->type == HCI_EVT_MSG_TYPE && bt_hci_evt_is_prio(recv_buf->evt.event))
		{
			///check wait receive queue over wait limit
			if(wait_recv_prio_cnt >= CONFIG_BK_HCI_RECV_PRIO_WAIT_LIMIT) {
				//printf("hci wait receive priority queue reach limit %d!\r\n", wait_recv_prio_cnt);
				///skip oldest receive buffer
				app_recv_buf_info_t *buf_info;
				GLOBAL_INT_DISABLE();
				buf_info = (app_recv_buf_info_t*)co_list_pop_front(&recv_prio_wait_list);
				wait_recv_prio_cnt--;
				GLOBAL_INT_RESTORE();
				krhino_mm_free(buf_info);
				//err = -1;    break;
			}
			wait_list = &recv_prio_wait_list;
			give_sem  = &recv_prio_sem;
			wait_cnt  = &wait_recv_prio_cnt;
		}
		else
		{
			///check wait receive queue over wait limit
			if(wait_recv_cnt >= CONFIG_BK_HCI_RECV_WAIT_LIMIT) {
				//printf("hci wait receive queue reach limit %d!\r\n", wait_recv_cnt);
				///skip oldest receive buffer
				app_recv_buf_info_t *buf_info;
				GLOBAL_INT_DISABLE();
				buf_info = (app_recv_buf_info_t*)co_list_pop_front(&recv_wait_list);
				wait_recv_cnt--;
				GLOBAL_INT_RESTORE();
				krhino_mm_free(buf_info);
				//err = -1;    break;
			}
			wait_list = &recv_wait_list;
			give_sem  = &recv_sem;
			wait_cnt  = &wait_recv_cnt;
		}

		///make receive buffer information
		app_recv_buf_info_t *buf = krhino_mm_alloc(sizeof(app_recv_buf_info_t)+recv_buf->buf_len);

		buf->nbuf    = NULL;
		buf->type    = recv_buf->type;
		buf->buf_len = recv_buf->buf_len;
		memcpy(buf->buf, recv_buf->buf, buf->buf_len);

		///push information to receive wait queue
		GLOBAL_INT_DISABLE();
		co_list_push_back(wait_list, &buf->hdr);
		(*wait_cnt) ++;
		GLOBAL_INT_RESTORE();

		krhino_sem_give(give_sem);
    } while(0);

    HCI_DBG("%s end!\r\n", __func__);
    return err;
}

static int hci_driver_send(struct net_buf *buf)
{
    uint16_t err = 0;

    hci_lock(&g_hci_send_context);

    if (NULL == buf) {
    	printf("%s, buffer is NULL\r\n", __func__);
    	hci_unlock(&g_hci_send_context);
        return -1;
    }

    ///fill command data to send command buffer
    ///use alios allocate memory save send data
    app_hci_buf_t *hci_send = krhino_mm_alloc(sizeof(app_hci_buf_t));
    memcpy(hci_send->buf, buf->data, buf->len);
    hci_send->buf_len = buf->len;

    ///check buffer type
    uint8_t data_type = bt_buf_get_type(buf);
    switch (data_type) {
        case BT_BUF_CMD:
            if(buf->len != (hci_send->cmd.para_len+3))
            {
            	printf("%s, type:%d, data length is error\r\n", __func__, data_type);
                err = -1;
                break;
            }

            hci_send->type = HCI_CMD_MSG_TYPE;
            HCI_INFO("tx cmd:0x%x, len:%d\r\n", hci_send->cmd.opcode, hci_send->cmd.para_len);
            break;

        case BT_BUF_ACL_OUT:
            if(buf->len != (hci_send->acl.data_len+4))
            {
            	printf("%s, type:%d, data length is error\r\n", __func__, data_type);
                err = -1;
                break;
            }

            hci_send->type = HCI_ACL_MSG_TYPE;
            HCI_INFO("tx acl:0x%x, len:%d\r\n", hci_send->acl.handle, hci_send->acl.data_len);
            break;

        default:
            printf("Unknown buffer type");
            err = -1;
            break;
    }
    net_buf_unref(buf);

    if(!err)
    {
		///send command
		HCI_DBG("hci driver send...\r\n");
		err = appm_hci_send_to_controller(hci_send);
		if(err) {
			printf("hci driver send error!\r\n", __func__);
		}
    }

	///free send data  memory
	krhino_mm_free(hci_send);

    if(err)    {
    	hci_unlock(&g_hci_send_context);
        return -1;
    }
    hci_unlock(&g_hci_send_context);
    return 0;
}

static void hci_delayed_action(void *arg)
{

#if 0
    aos_post_delayed_action(2000, hci_delayed_action, NULL);
    printf("now ticks:%d\r\n", fclk_get_tick()*100/fclk_tick_a_second());

    static uint8_t old_pri = 0;
    if(old_pri <= AOS_DEFAULT_APP_PRI) {
        ktask_t *c_task = krhino_cur_task_get();
        krhino_task_pri_change(c_task, AOS_DEFAULT_APP_PRI+10, &old_pri);
        printf("%s task change pri from %d to %d\r\n", c_task->task_name, old_pri, AOS_DEFAULT_APP_PRI+10);
    }
#endif

#if 0
    ///dump AliOS information
    static uint8_t dump_info = 0;
    if(!dump_info)
    {
        dump_info ++;
        krhino_overview();
#if (RHINO_CONFIG_MM_DEBUG > 0)
        dumpsys_mm_info_func(0);
#endif
    }
#endif

#if 1
    ///if memory isn't enough
    ///delete task to release memory of initial task
    ktask_t *c_task = krhino_cur_task_get();
    printf("del task:%s to release memory\r\n", c_task->task_name);
    krhino_task_dyn_del(c_task);
#endif
}

static int hci_driver_open(void)
{
    ksem_t  ready_sem;
    kstat_t err = 0;
    HCI_DBG("initial BLE...\r\n");

    ///register hci event receive call back
    appm_hci_recv_reg_callback(hci_driver_recv);

    ///create semaphore
    err = krhino_sem_create(&ready_sem, "ble_hdl_ready", 0);
    if(err) {
        printf("ready semaphore create error:0x%x\r\n", err);
        return -ENOMEM;
    }

    ///get bd address from flash
	struct bd_addr_t bd_addr={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
#if 0
	if(stack_support_flash == 8) ///check support flash type
		flash_read(&bd_addr, sizeof(struct bd_addr_t), 0xFB000); ///for 8Mb flash
	else
		flash_read(&bd_addr, sizeof(struct bd_addr_t), 0x7B000); ///for 4Mb flash
#endif

	if((bd_addr.addr[0]!=0xFF)&&(bd_addr.addr[0]!=0xFF)&&(bd_addr.addr[0]!=0xFF)&&
		(bd_addr.addr[0]!=0xFF)&&(bd_addr.addr[0]!=0xFF)&&(bd_addr.addr[0]!=0xFF))
	{
		memcpy(ble_bd_addr, &bd_addr, sizeof(struct bd_addr_t));
	}

	g_ble_hdr_arg.public_addr  = ble_bd_addr;
    g_ble_hdr_arg.hci_hdr      = &ble_hci_hdr;
    g_ble_hdr_arg.ready_sem    = &ready_sem;
    g_ble_hdr_arg.task_timeout = CONFIG_BK_HCI_TASK_SEM_TIMEROUT;

    if(0 == aos_mutex_new(&g_hci_send_context.lock))
    {
        g_hci_send_context.initialized = 1;
    }

    ///create ble hci handler task
    err = krhino_task_create(&ble_hci_recv_prio_hdr, "ble_hci_recv_prio", NULL,
    		                 CONFIG_BK_HCI_RECV_PRIO_TASK_PRI, 0, hci_recv_prio_stack,
    		                 sizeof(hci_recv_prio_stack) / sizeof(cpu_stack_t),
                             (task_entry_t)ble_recv_prio_handler, 1);
    err = krhino_task_create(&ble_hci_recv_hdr, "ble_hci_recv", NULL,
    		                 CONFIG_BK_HCI_RECV_TASK_PRI, 0, hci_recv_stack,
    		                 sizeof(hci_recv_stack) / sizeof(cpu_stack_t),
                             (task_entry_t)ble_recv_handler, 1);
    err = krhino_task_create(&ble_hci_hdr, "ble_hci_driver", (void*)&g_ble_hdr_arg,
    		                 CONFIG_BK_HCI_TASK_PRI, 0, ble_hci_stack,
    		                 sizeof(ble_hci_stack) / sizeof(cpu_stack_t),
                             (task_entry_t)ble_handler, 1);

    if(!err) {
        ///wait ble hci driver start
        err = krhino_sem_take(&ready_sem, RHINO_WAIT_FOREVER);
        if(err) HCI_DBG("ready semaphore error:0x%x\r\n", err);
    }
    else {
        printf("ble hci task create error : 0x%x\r\n", err);
    }

    ///delete semaphore
    g_ble_hdr_arg.ready_sem = NULL;
    krhino_sem_del(&ready_sem);

    if(err) {
        printf("initial BLE is error!\r\n");
        return -1;
    }

    return 0;
}


static struct bt_hci_driver drv = {
    .name   = "bk3435 Controller",
    .bus    = BT_HCI_DRIVER_BUS_VIRTUAL,
    .open   = hci_driver_open,
    .send   = hci_driver_send,
};

int hci_driver_init()
{
    bt_hci_driver_register(&drv);
    return 0;
}
