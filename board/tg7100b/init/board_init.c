/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     board_init.c
 * @brief    CSI Source File for board init
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <ap_cp.h>
#include <gpio.h>
#include <pinmux.h>
#include <clock.h>
#include <global_config.h>
#include <rf_phy_driver.h>
#include <drv_usart.h>
#include <aos/kernel.h>
#include <hal/soc/flash.h>

extern volatile uint8_t   g_system_clk;
extern uint32 *pGlobal_config;
extern uint32 global_config[];
extern uint32_t  g_smartWindowSize;

const hal_logic_partition_t hal_partitions[] = 
{
	[HAL_PARTITION_PARAMETER_4] =
	{
	 .partition_owner			 = HAL_FLASH_EMBEDDED,
	 .partition_description 	 = "seq",
	 .partition_start_addr		 = 0x1107A000,
	 .partition_length			 = 0x1000, // 4k bytes
	 .partition_options 		 = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_PARAMETER_1] =
	{
	 .partition_owner			 = HAL_FLASH_EMBEDDED,
	 .partition_description 	 = "system",
	 .partition_start_addr		 = 0x1107B000,
	 .partition_length			 = 0x1000, // 4k bytes
	 .partition_options 		 = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_PARAMETER_2] =
	{
	 .partition_owner			 = HAL_FLASH_EMBEDDED,
	 .partition_description 	 = "userdata",
	 .partition_start_addr		 = 0x1107C000,
	 .partition_length			 = 0x1000, // 4k bytes
	 .partition_options 		 = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_PARAMETER_3] =
	{
	 .partition_owner			 = HAL_FLASH_EMBEDDED,
	 .partition_description 	 = "recycle",
	 .partition_start_addr		 = 0x1107D000,
	 .partition_length			 = 0x1000, //4k bytes
	 .partition_options 		 = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_CUSTOM_1] =
	{
	 .partition_owner			 = HAL_FLASH_EMBEDDED,
	 .partition_description 	 = "kv",
	 .partition_start_addr		 = 0x1107E000,
	 .partition_length			 = 0x1000, //4k bytes
	 .partition_options 		 = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
    [HAL_PARTITION_CUSTOM_2] =
    {
    .partition_owner            = HAL_FLASH_EMBEDDED,
    .partition_description      = "freq",
    .partition_start_addr       = 0x1107F000,
    .partition_length           = 0x1000, //4k bytes
    .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
	[HAL_PARTITION_OTA_TEMP] =
	{
	 .partition_owner			 = HAL_FLASH_EMBEDDED,
	 .partition_description 	 = "misc",
	 .partition_start_addr		 = 0x11024000,
	 .partition_length			 = 0x37000,
	 .partition_options 		 = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
};

void set_adv_channel_interval(int us)
{
    pGlobal_config[ADV_CHANNEL_INTERVAL] = us;
}

void init_config(void)
{
    int i;

    *(uint32 *)(0x1fff101c) = 0x1fff0c00;        // set ROM code pGlobal_config
    pGlobal_config = &global_config[0];

    for (i = 0; i < 256; i ++) {
        pGlobal_config[i] = 0;
    }

    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
//    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;

    // LL switch setting
    pGlobal_config[LL_SWITCH] =  LL_DEBUG_ALLOW | SLAVE_LATENCY_ALLOW | LL_WHITELIST_ALLOW
                                 | SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW | GAP_DUP_RPT_FILTER_DISALLOW; //RC32_TRACKINK_ALLOW

    // sleep delay
    pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10;      // 10ms, temporary set

    // system clock setting
    pGlobal_config[CLOCK_SETTING] = g_system_clk;//CLOCK_32MHZ;

    //------------------------------------------------------------------------
    // wakeup time cose
    // t1. HW_Wakeup->MCU relase 62.5us
    // t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
    // t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
    // t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
    // WAKEUP_ADVANCE should be larger than t1+t2+t3+t4
    //------------------------------------------------------------------------
    // wakeup advance time, in us

    pGlobal_config[WAKEUP_ADVANCE] = 1350;//650;//600;//310;

    if (g_system_clk == SYS_CLK_XTAL_16M) {
        pGlobal_config[WAKEUP_DELAY] = 16;
    } else if (g_system_clk == SYS_CLK_DLL_48M) {
        pGlobal_config[WAKEUP_DELAY] = 20;
    } else if (g_system_clk == SYS_CLK_DLL_64M) {
        pGlobal_config[WAKEUP_DELAY] = 24;
    }


    // sleep time, in us
    pGlobal_config[MAX_SLEEP_TIME] = 30000000;
    pGlobal_config[MIN_SLEEP_TIME] = 1500;

    pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 60;// 30.5 per tick

    // LL engine settle time
    pGlobal_config[LL_HW_BB_DELAY] = 54;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY] = 52;

    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 58;//57;          // 2019/3/20 A2: 57 --> 58
    pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 49;//50; //65     // 2019/3/20 A2: 50 --> 49

    //------------------------------------------------2MPHY
    // LL engine settle time
    pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
    pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 70;//72
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 57;//72

    //------------------------------------------------CODEPHY 500K
    // LL engine settle time CODEPHY 500K
    pGlobal_config[LL_HW_BB_DELAY_500KPHY] = 50;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY_500KPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_500KPHY] = 52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_500KPHY] =  2;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_500KPHY] = 66;//72

    //------------------------------------------------CODEPHY 125K
    // LL engine settle time CODEPHY 125K
    pGlobal_config[LL_HW_BB_DELAY_125KPHY] = 30;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY_125KPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_125KPHY] = 52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_125KPHY] = 5;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_125KPHY] = 66;//72

    // LL engine settle time, for advertisement
    pGlobal_config[LL_HW_BB_DELAY_ADV] = 90;
    pGlobal_config[LL_HW_AFE_DELAY_ADV] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_ADV] = 60;

    // adv channel interval
    pGlobal_config[ADV_CHANNEL_INTERVAL] = 2300;//6250;


    if (g_system_clk == SYS_CLK_XTAL_16M) {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 16;//23;        //  2019/3/19 A2: 23 --> 16
    } else if (g_system_clk == SYS_CLK_DLL_48M) {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 11;
    } else if (g_system_clk == SYS_CLK_DLL_64M) {   //  2019/3/26 add
        pGlobal_config[SCAN_RSP_DELAY] = 8;
    }

    // conn_req -> slave connection event calibration time, will advance the receive window
    pGlobal_config[CONN_REQ_TO_SLAVE_DELAY] = 300;//192;//500;//192;

    // calibration time for 2 connection event, will advance the next conn event receive window
    // SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
    pGlobal_config[SLAVE_CONN_DELAY] = 300;//0;//1500;//0;//3000;//0;          ---> update 11-20
    pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 160;

    // RTLP timeout
    pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
    pGlobal_config[LL_HW_RTLP_TO_GAP]       = 1000;

    pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 4000 + pGlobal_config[SLAVE_CONN_DELAY] * 2;//500;


    // direct adv interval configuration
    pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
    pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;

    // A1 ROM metal change for HDC direct adv,
    pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.

    // A1 ROM metal change
    pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 8;//8;
    pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 8;//8;

    pGlobal_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8;     //0:        disable adaptive
    //other:    adaptive max limitation

    //smart window configuration
    pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]  = 2;
    pGlobal_config[LL_SMART_WINDOW_TARGET]      = 600;
    pGlobal_config[LL_SMART_WINDOW_INCREMENT]   = 9;
    pGlobal_config[LL_SMART_WINDOW_LIMIT]       = 20000;
    pGlobal_config[LL_SMART_WINDOW_ACTIVE_THD]  = 8;
    pGlobal_config[LL_SMART_WINDOW_ACTIVE_RANGE]= 0; //300

    pGlobal_config[LL_SMART_WINDOW_FIRST_WINDOW] = 5000;

    g_smartWindowSize = pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] ;

    //====== A2 metal change add, for scanner & initiator
    if (g_system_clk == SYS_CLK_XTAL_16M) {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] = 18;//20;     //  2019/3/19 A2: 20 --> 18
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY] = 25;//27;     //  2019/3/19 A2: 27 --> 25
    } else if (g_system_clk == SYS_CLK_DLL_48M) {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] = 10;//12;     //  2019/3/19 A2: 12 --> 10
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY] = 16;// 12->13->16 zyf
    } else if (g_system_clk == SYS_CLK_DLL_64M) {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] = 8;               //  2019/3/26 add
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY] = 10;
    }

    // TRLP timeout
    pGlobal_config[LL_HW_TRLP_LOOP_TIMEOUT] = 50000;    // enough for 8Tx + 8Rx : (41 * 8 + 150) * 16 - 150 = 7498us
    pGlobal_config[LL_HW_TRLP_TO_GAP]       = 1000;
    pGlobal_config[LL_MOVE_TO_MASTER_DELAY] = 100;

    pGlobal_config[LL_CONN_REQ_WIN_SIZE] = 5;
    pGlobal_config[LL_CONN_REQ_WIN_OFFSET] = 2;

    pGlobal_config[LL_MASTER_PROCESS_TARGET] = 200;   // reserve time for preparing master conn event, delay should be insert if needn't so long time
    pGlobal_config[LL_MASTER_TIRQ_DELAY] = 0;         // timer IRQ -> timer ISR delay


    pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;  // 0.125us

    pGlobal_config[MAC_ADDRESS_LOC] = 0x11004000;

    // for simultaneous conn & adv/scan
    pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400;
    pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;

    pGlobal_config[LL_SEC_SCAN_MARGIN] = 2500;//1400;
    pGlobal_config[LL_MIN_SCAN_TIME] = 2000;
}

extern void ble_init(void);
void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    LL_SetTxPowerLevel(RF_PHY_TX_POWER_MAX);
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   = RF_PHY_FREQ_FOFF_N80KHZ;

    int8_t freq_off = *(volatile uint32_t *)(0x11004020);

    if (freq_off != 0xff) {
        g_rfPhyFreqOffSet = freq_off;
    }

    ble_init();

    //Quick Boot setting and
    *(volatile uint32_t *) 0x4000f01c = 0x0000004;       //  3'b1xx: 62.5us.  control bits for main digital part reset wait time after power up charge pump.

    //========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw

    //========= UART RX Pull up
    phy_gpio_pull_set(P10, WEAK_PULL_UP);

    DCDC_CONFIG_SETTING(0x0d);

    NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);
}

usart_handle_t *hal_uart_get_handle(uint8_t port);
int fputc(int ch, FILE *stream)
{
    int data;
    usart_handle_t console_handle;
    console_handle = hal_uart_get_handle(STDIO_UART);

    if (console_handle == NULL) {
        return -1;
    }

    if (ch == '\n') {
        data = '\r';
        csi_usart_putchar(console_handle, data);
    }

    csi_usart_putchar(console_handle, ch);

    return 0;
}

int os_critical_enter(unsigned int *lock)
{
    aos_kernel_sched_suspend();

    return 0;
}

int os_critical_exit(unsigned int *lock)
{
    aos_kernel_sched_resume();

    return 0;
}

void __attribute__((weak)) board_init(void)
{
    drv_pinmux_config(P9, UART_TX);
    drv_pinmux_config(P10, UART_RX);
}
