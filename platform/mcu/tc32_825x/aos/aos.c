/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#if 0
#include <aos/aos.h>
#include <k_api.h>
#include <aos/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hal/wifi.h"
#include "hal/ota.h"

#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "hal/soc/uart.h"

#include "nrf_drv_systick.h"

#ifdef AOS_COMP_PWRMGMT
#include <pwrmgmt_api.h>
#endif /* AOS_COMP_PWRMGMT */

#if POWER_ENABLED
#include "nrf_drv_power.h"
#endif
#endif

#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"

#include "vendor/common/blt_led.h"
#include "vendor/common/blt_common.h"
#include "application/keyboard/keyboard.h"
#include "application/usbstd/usbkeycode.h"

#include "application/print/putchar_sim.h"
#include <k_api.h>
#include <stdio.h>
#include <aos/kernel.h>
#include <aos/init.h>
#include <hal/soc/uart.h>

void blc_hci_send_data_aos(u8 hci_evt_cmd, u8 *para, u8 n, int prio_flag);
void uart_rx_proc(void);
extern int8_t genie_tri_tuple_load(void);
void pkt_recv_callback(void);


#define 	ADV_IDLE_ENTER_DEEP_TIME			60  //60 s
#define 	CONN_IDLE_ENTER_DEEP_TIME			60  //60 s

#define 	MY_DIRECT_ADV_TMIE					2000000


#define     MY_APP_ADV_CHANNEL					BLT_ENABLE_ADV_ALL
#define 	MY_ADV_INTERVAL_MIN					ADV_INTERVAL_10MS
#define 	MY_ADV_INTERVAL_MAX					ADV_INTERVAL_10MS


#define		MY_RF_POWER_INDEX					RF_POWER_P3p01dBm


#define		BLE_DEVICE_ADDRESS_TYPE 			BLE_DEVICE_ADDRESS_PUBLIC

_attribute_data_retention_	own_addr_type_t 	app_own_address_type = OWN_ADDRESS_PUBLIC;

#define RX_FIFO_SIZE	64
#define RX_FIFO_NUM		16

#define TX_FIFO_SIZE	40
#define TX_FIFO_NUM		16

#define HCI_TXFIFO_SIZE 80
#define HCI_TXFIFO_NUM  8

STATIC_ASSERT((OTA_FIRMWARE_SIZE_MAX % 4096) == 0);

MYFIFO_INIT(blt_rxfifo, RX_FIFO_SIZE, RX_FIFO_NUM);
MYFIFO_INIT(blt_txfifo, TX_FIFO_SIZE, TX_FIFO_NUM);
MYFIFO_INIT(hci_tx_fifo, HCI_TXFIFO_SIZE, HCI_TXFIFO_NUM);

_attribute_ram_code_ void	user_set_rf_power (u8 e, u8 *p, int n)
{
	rf_set_power_level_index (MY_RF_POWER_INDEX);
}

void	task_connect (u8 e, u8 *p, int n)
{
#if 0
	bls_l2cap_requestConnParamUpdate (8, 8, 19, 200);  // 200mS
//	bls_l2cap_requestConnParamUpdate (8, 8, 99, 400);  // 1 S
//	bls_l2cap_requestConnParamUpdate (8, 8, 149, 600);  // 1.5 S
//	bls_l2cap_requestConnParamUpdate (8, 8, 199, 800);  // 2 S
//	bls_l2cap_requestConnParamUpdate (8, 8, 249, 800);  // 2.5 S
//	bls_l2cap_requestConnParamUpdate (8, 8, 299, 800);  // 3 S

	latest_user_event_tick = clock_time();

	device_in_connection_state = 1;//

	interval_update_tick = clock_time() | 1; //none zero
#endif
}


void	task_conn_update_req (u8 e, u8 *p, int n)
{

}

void	task_conn_update_done (u8 e, u8 *p, int n)
{

}

void 	ble_remote_terminate(u8 e,u8 *p, int n) //*p is terminate reason
{

}

#define GPIO_LED    GPIO_PB1		//white

void show_ota_result(int result)
{

#if 1 // (1 && BLT_APP_LED_ENABLE)
    gpio_set_func(GPIO_LED, AS_GPIO);
	gpio_set_output_en(GPIO_LED, 1);

	if(result == OTA_SUCCESS){  //OTA success
		gpio_write(GPIO_LED, 1);
		sleep_us(500000);
		gpio_write(GPIO_LED, 0);
		sleep_us(500000);
		gpio_write(GPIO_LED, 1);
		sleep_us(500000);
		gpio_write(GPIO_LED, 0);
		sleep_us(500000);
	}
	else{  //OTA fail

		#if 0 //this is only for debug,  can not use this in application code
			irq_disable();
			WATCHDOG_DISABLE;

			write_reg8(0x40000, 0x33);
			while(1){
				gpio_write(GPIO_LED, 1);
				sleep_us(200000);
				gpio_write(GPIO_LED, 0);
				sleep_us(200000);
			}
			write_reg8(0x40000, 0x44);
		#endif

	}

	gpio_set_output_en(GPIO_LED, 0);
#endif
}


#define BT_MESH_DATA_MESH_PROV               0x29 /* Mesh Provisioning PDU */
#define BT_MESH_DATA_MESH_MESSAGE            0x2a /* Mesh Networking PDU */
#define BT_MESH_DATA_MESH_BEACON             0x2b /* Mesh Beacon */

_attribute_ram_code_ int  blc_ll_procScanPkt_aos(u8 *raw_pkt, u8 *new_pkt, u32 tick_now)
{
	rf_packet_adv_t * pAdv = (rf_packet_adv_t *) (raw_pkt + DMA_RFRX_LEN_HW_INFO);
	//u8 txAddrType = pAdv->header.txAddr;
	//u8 rxAddrType = pAdv->header.rxAddr;
	u8 adv_type = pAdv->header.type;

    if(adv_type==LL_TYPE_ADV_NONCONN_IND){
        //printf("--scan beacon--\n");
        u8 mesh_pkt_type = pAdv->data[1];
        if ((BT_MESH_DATA_MESH_PROV == mesh_pkt_type)||
            (BT_MESH_DATA_MESH_MESSAGE == mesh_pkt_type)||
            (BT_MESH_DATA_MESH_BEACON == mesh_pkt_type)){
            return 1;
        }
    }else{
        //printf("--other pkt, type: %d--\n", adv_type);
    }
    return 0;
}

/*
synchronous event: call in hci_driver rec thread.
blc_hci_send_event_aos_cb is equal to blc_hci_send_event.
*/
_attribute_ram_code_ int blc_hci_send_event_aos_cb (u32 h, u8 *para, int n)
{
    //u8 r = tlk_irq_disable();
    if (h & HCI_FLAG_EVENT_TLK_MODULE)
    {
    }
    else if (h & HCI_FLAG_EVENT_BT_STD)
    {
        //printf("async event:type:0x%02x,len:%d, data:%s\r\n",h&0xff,n,bt_hex(para,n));
        int prio_flag = (HCI_EVT_NUM_OF_COMPLETE_PACKETS == (h & 0xff)) ?  1 : 0;
        blc_hci_send_data_aos(h&0xff, para, n&0xff, prio_flag);
    }
    else if (h & HCI_FLAG_EVENT_STACK)
    {
    }
    else if (h & HCI_FLAG_ACL_BT_STD)           //ACL data
    {
		// hanle in blc_hci_send_data()
    }
    //tlk_irq_resrote(r);
    return 0;
}

void blc_ll_initScanning_module_aos(u8 *public_adr)
{
	blc_ll_procScanPktCb = blc_ll_procScanPkt_aos;  // for filter pkt.
	blc_ll_procScanDatCb = blc_ll_procScanData; // for handle data


	memcpy(pkt_scan_req.scanA, public_adr, BLE_ADDR_LEN);
	blts_scan_interval = 60000 * sys_tick_per_us;  //60ms
}

//------- aos init --------
#define SYS_INIT_TASK_PRIORITY     5    // can't not be same with RHINO_CONFIG_K_DYN_MEM_TASK_PRI
#define AOS_START_STACK            (400)  // unit:4bytes

//#define WIFI_PRODUCT_INFO_SIZE                      ES_WIFI_MAX_SSID_NAME_SIZE

#if 0
ktask_t *g_aos_init;
ktask_t *g_aos_app = NULL;
#else
static ktask_t sys_init_task_obj;
cpu_stack_t sys_init_task_stack[AOS_START_STACK];
#endif

extern int application_start(int argc, char **argv);
extern void aos_heap_set(void);
extern void aos_heap_check_log();


static void timer0_init(void){
	//timer0 [1000/RHINO_CONFIG_TICKS_PER_SECOND] ms interval irq
	reg_irq_mask |= FLD_IRQ_TMR0_EN;
	reg_tmr0_tick = 0;                    //claer counter
	reg_tmr0_capt = (1000/RHINO_CONFIG_TICKS_PER_SECOND) * CLOCK_SYS_CLOCK_1MS;
	reg_tmr_sta = FLD_TMR_STA_TMR0;       //clear irq status
	reg_tmr_ctrl |= FLD_TMR0_EN;          //start timer
}

volatile int irq_cnt_timer0 = 0;
_attribute_ram_code_ void timer0_irq(void)
{
	u32 src = reg_irq_src;
	
	if(src & FLD_IRQ_TMR0_EN){
		reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status	
		++irq_cnt_timer0;
		//gpio_toggle(GPIO_PC6);
		krhino_intrpt_enter();
    	krhino_tick_proc();
		krhino_intrpt_exit();
	}
}

_attribute_ram_code_ void irq_handler(void)
{
    static u32 irq_cnt_1handle; irq_cnt_1handle++;
    irq_blt_sdk_handler();
    
	timer0_irq();
#if((PRINT_ENABLE==1) && (IS_PRINT_USE_SIM==0))
	uart_rx_proc();
#endif

#if GPIO_IRQ_ENABLE
	gpio_irq_handler();
#endif
}

extern u8 genie_flash_init(void);
void user_init_normal(void)
{
	//random number generator must be initiated here( in the beginning of user_init_nromal)
	//when deepSleep retention wakeUp, no need initialize again
	random_generator_init();  //this is must



////////////////// BLE stack initialization ////////////////////////////////////
#if 1
    genie_flash_init();
    genie_tri_tuple_load();
    extern u8 g_mac[6];
    u8  mac_public[6] = {0};
    foreach(i,sizeof(mac_public)){
        mac_public[i] = g_mac[(sizeof(mac_public) - 1) - i];
    }
    
    // mac_public[0] = 0x22;
#else
    u8  mac_public[6] = {0x11,0x22,0x58,0x82,0xaa,0xaa};
#endif

#if 0
	u8  mac_random_static[6];   //this is not available for HCI controller, cause host will set random address to it
	blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

	#if(BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_PUBLIC)
		app_own_address_type = OWN_ADDRESS_PUBLIC;
	#elif(BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_RANDOM_STATIC)
		app_own_address_type = OWN_ADDRESS_RANDOM;
		blc_ll_setRandomAddr(mac_random_static);
	#endif
#endif
	////// Controller Initialization  //////////
	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(mac_public);				//mandatory
	blc_ll_initAdvertising_module(mac_public); 	//adv module: 		 mandatory for BLE slave,
	blc_ll_initSlaveRole_module();				//slave module: 	 mandatory for BLE slave,
	//blc_ll_initPowerManagement_module();        //pm module:      	 optional

	////// Host Initialization  //////////
	blc_l2cap_register_handler (blc_hci_sendACLData2Host);  	//l2cap initialization


#if 1
    blc_ll_addScanningInAdvState();
    blc_ll_addScanningInConnSlaveRole();
    blc_ll_initScanning_module_aos(mac_public);
	extern int event_handler(u32 h, u8 *para, int n);
	blc_hci_registerControllerEventHandler(blc_hci_send_event_aos_cb);		//register event callback
#else
    blc_ll_setScanEnable (BLS_FLAG_SCAN_ENABLE | BLS_FLAG_ADV_IN_SLAVE_MODE, 0);
#endif


///////////////////// USER application initialization ///////////////////
//	bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );
//	bls_ll_setScanRspData( (u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

	u8 status = bls_ll_setAdvParam(  MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
											 ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC,
											 0,  NULL,
											 MY_APP_ADV_CHANNEL,
											 ADV_FP_NONE);
	if(status != BLE_SUCCESS) { write_reg8(0x40002, 0x11); 	while(1); }  //debug: adv setting err

	bls_ll_setAdvEnable(0);  //adv enable


	//set rf power index, user must set it after every suspend wakeup, cause relative setting will be reset in suspend
	user_set_rf_power(0, 0, 0);
	bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_EXIT, &user_set_rf_power);



	//ble event call back
	bls_app_registerEventCallback (BLT_EV_FLAG_CONNECT, &task_connect);
	bls_app_registerEventCallback (BLT_EV_FLAG_TERMINATE, &ble_remote_terminate);


	bls_app_registerEventCallback (BLT_EV_FLAG_CONN_PARA_REQ, &task_conn_update_req);
	bls_app_registerEventCallback (BLT_EV_FLAG_CONN_PARA_UPDATE, &task_conn_update_done);



	///////////////////// Power Management initialization///////////////////
#if(BLE_APP_PM_ENABLE)
	blc_ll_initPowerManagement_module();

	#if (PM_DEEPSLEEP_RETENTION_ENABLE)
		bls_pm_setSuspendMask (SUSPEND_ADV | DEEPSLEEP_RETENTION_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_CONN);
		blc_pm_setDeepsleepRetentionThreshold(95, 95);
		blc_pm_setDeepsleepRetentionEarlyWakeupTiming(TEST_CONN_CURRENT_ENABLE ? 220 : 240);
		//blc_pm_setDeepsleepRetentionType(DEEPSLEEP_MODE_RET_SRAM_LOW32K); //default use 16k deep retention
	#else
		bls_pm_setSuspendMask (SUSPEND_ADV | SUSPEND_CONN);
	#endif

	bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_ENTER, &ble_remote_set_sleep_wakeup);
#else
	bls_pm_setSuspendMask (SUSPEND_DISABLE);
#endif

#if 1 // (BLE_OTA_ENABLE)
	// OTA init
	bls_ota_clearNewFwDataArea(); //must
	//bls_ota_registerStartCmdCb(entry_ota_mode);
	//bls_ota_registerResultIndicateCb(show_ota_result);
#endif
}

_attribute_ram_code_ void user_init_deepRetn(void)
{
#if (PM_DEEPSLEEP_RETENTION_ENABLE)

	blc_ll_initBasicMCU();   //mandatory
	rf_set_power_level_index (MY_RF_POWER_INDEX);

	blc_ll_recoverDeepRetention();

	DBG_CHN0_HIGH;    //debug

	irq_enable();

	#if (!TEST_CONN_CURRENT_ENABLE)
		/////////// keyboard gpio wakeup init ////////
		u32 pin[] = KB_DRIVE_PINS;
		for (int i=0; i<(sizeof (pin)/sizeof(*pin)); i++)
		{
			cpu_set_gpio_wakeup (pin[i], Level_High,1);  //drive pin pad high wakeup deepsleep
		}
	#endif
#endif
}

static void tc825x_hal_init(void)
{
    irq_disable();
	bls_ota_set_fwSize_and_fwBootAddr(OTA_FIRMWARE_SIZE_MAX_K, 0x40000);

	blc_pm_select_internal_32k_crystal();

	cpu_wakeup_init();

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	rf_drv_init(RF_MODE_BLE_1M);

	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	clock_init(SYS_CLK_16M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	clock_init(SYS_CLK_24M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
	clock_init(SYS_CLK_32M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
	clock_init(SYS_CLK_48M_Crystal);
#endif

	blc_app_loadCustomizedParameters();  //load customized freq_offset cap value
#if((PRINT_ENABLE==1) && (IS_PRINT_USE_SIM==0))
	hal_uart_init(NULL);    // must before user_init, because it will print log inside user_init.
#endif

#if PM_DEEPSLEEP_RETENTION_ENABLE
	if( deepRetWakeUp ){
		user_init_deepRetn ();
	}
	else
#endif
	{
		user_init_normal ();
	}

	timer0_init();
	irq_enable();
}

#define LA_TEST_ENABLE      0

#if LA_TEST_ENABLE
void lpn_debug_loop_pin(u8 level)
{
#if LA_TEST_ENABLE
    #define CURRENT_PIN_MESH      GPIO_PB2
    gpio_set_func(CURRENT_PIN_MESH, AS_GPIO);
    gpio_set_output_en(CURRENT_PIN_MESH, 1);
    gpio_write(CURRENT_PIN_MESH, level);
#endif
}

void lpn_debug_adv_enable_pin(u8 level)
{
#if LA_TEST_ENABLE
    #define DEBUG_PIN_MESH      GPIO_PB3
    gpio_set_func(DEBUG_PIN_MESH, AS_GPIO);
    gpio_set_output_en(DEBUG_PIN_MESH, 1);
    gpio_write(DEBUG_PIN_MESH, level);
#endif
}

void lpn_debug_gatt_beacon_pin(u8 level)
{
#if LA_TEST_ENABLE
    #define EVENT_PIN_MESH      GPIO_PB4
    gpio_set_func(EVENT_PIN_MESH, AS_GPIO);
    gpio_set_output_en(EVENT_PIN_MESH, 1);
    gpio_write(EVENT_PIN_MESH, level);
#endif
}

void lpn_debug_adv_interval_ok_pin(u8 level)
{
#if LA_TEST_ENABLE
    #define IRQ_PIN_MESH      GPIO_PB5
    gpio_set_func(IRQ_PIN_MESH, AS_GPIO);
    gpio_set_output_en(IRQ_PIN_MESH, 1);
    gpio_write(IRQ_PIN_MESH, level);
#endif
}

void lpn_debug_adv_tx_pin(u8 level)
{
#if LA_TEST_ENABLE
    #define IRQ_PIN_MESH      GPIO_PC2
    gpio_set_func(IRQ_PIN_MESH, AS_GPIO);
    gpio_set_output_en(IRQ_PIN_MESH, 1);
    gpio_write(IRQ_PIN_MESH, level);
#endif
}

void lpn_debug_alter_loop_pin(int reset)
{
#if LA_TEST_ENABLE
    static u8 debug_pin_level = 0;
    if(reset){
        debug_pin_level = 0;
    }else{
        debug_pin_level = !debug_pin_level;
    }
    
    lpn_debug_loop_pin(debug_pin_level);
#endif
}
#endif

u8 blt_rx_fifo_poll_flag = 0;

extern void dumpsys_cli_init(void);
void sys_init(void *arg)
{
	tc825x_hal_init();
	for(int i=0;i<1;i++){
		printf("\r\n\r\n\r\n--sys_init--\r\n");
	}
	
	aos_heap_check_log();
	#if 1
    kinit_t kinit = {
        .argc = 0,
        .argv = NULL,
        .cli_enable = 0,
    };
	aos_kernel_init(&kinit);
    dumpsys_cli_init();
	#else
    application_start(0, NULL); // mesh init inside, include read mac.
    #endif
    
    #if 1
    #define DEMO_TEST_SLEEP_TICK    (1)
	for(;;){
        krhino_task_sleep(DEMO_TEST_SLEEP_TICK);
        blt_sdk_main_loop();
        //led_ctrl_test();

        #if HW_UART_RING_BUF_EN
		hal_uart_send_loop(0);
        #endif
        
        #if 1
        static u32 rx_fifo_poll_tick;
        //if(++rx_fifo_poll_tick >= (4+(DEMO_TEST_SLEEP_TICK - 1))/DEMO_TEST_SLEEP_TICK){
            if((blt_rxfifo.wptr != blt_rxfifo.rptr) || my_fifo_get(&hci_tx_fifo)){
                if(!blt_rx_fifo_poll_flag){
                    rx_fifo_poll_tick = 0;
                    pkt_recv_callback();
                    blt_rx_fifo_poll_flag = 1;
                }
            }
        //}
        #endif
        static u32 sys_task_cnt;
        if(sys_task_cnt % (100/DEMO_TEST_SLEEP_TICK) == 0){
            //printf("sys task-%d\r\n", sys_task_cnt);
        }
        sys_task_cnt++;
    }
    #else
	krhino_task_del(&sys_init_task_obj); //(g_aos_app); // 
    
	for(;;){
		printf("--task can't run here--\n");
	}
	#endif
}

extern u8 adc_hw_initialized;
extern u8 blt_dma_tx_rptr;
// memset(pmParam,0,)

int main(void)
{
    /*adc_hw_initialized = */blt_dma_tx_rptr = 0;   // must init when retention wakeup
	aos_heap_set();

#if 0
    aos_init();
	krhino_task_dyn_create(&g_aos_app, "aos-init", 0,SYS_INIT_TASK_PRIORITY, 0, AOS_START_STACK, (task_entry_t)sys_init, 1);		
	
	//irq_enable();
    aos_start();
#else
	krhino_init();
	krhino_task_create(&sys_init_task_obj, "krhino_sys_init", 0,SYS_INIT_TASK_PRIORITY, 
        0, sys_init_task_stack, AOS_START_STACK, (task_entry_t)sys_init, 1);		
	
	//irq_enable();
    krhino_start();
#endif
    return 0;
}


