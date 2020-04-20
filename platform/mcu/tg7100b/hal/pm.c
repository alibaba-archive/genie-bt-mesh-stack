#include <yoc_config.h>

#include <stdlib.h>
#include <stdio.h>

#include <soc.h>
#include <drv_rtc.h>
#include <drv_pmu.h>
#include <drv_usart.h>
#include <dw_wdt.h>
#include <drv_wdt.h>

//#include <aos/aos.h>
//#include <yoc/partition.h>

#include <yoc/lpm.h>
#include "ap_cp.h"
#include "common.h"
#include "pwrmgr.h"
#include "uart.h"
#include "pinmux.h"
#include "ll.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"

#include "ll_sleep.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
//#include "power_manager.h"
#include "hal_mcu.h"
#include "burn.h"


#define PART_ADDR_FLASH_BEGIN           (0x10000000)

/* Constants identifying power state categories */
#define SYS_PM_ACTIVE_STATE     0 /* SOC and CPU are in active state */
#define SYS_PM_LOW_POWER_STATE      1 /* CPU low power state */
#define SYS_PM_DEEP_SLEEP       2 /* SOC low power state */

#define SYS_PM_NOT_HANDLED      SYS_PM_ACTIVE_STATE

#define MIN_TIME_TO_SLEEP       50      //ms
#define MIN_TIME_TO_SUSPEND     10000   //ms

//pwrmgr_attribute_t pwrmgr_attribute;
extern uint32 ll_remain_time;
extern uint32_t  g_wakeup_rtc_tick;// = 0;
static uint32 sleep_tick;
extern uint32 sleep_flag;// = 0;                   // when sleep, set this value to SLEEP_MAGIC. when wakeup, set this value to 0
extern uint32 osal_sys_tick;
static halIntState_t intState;

extern uint32_t  g_counter_traking_avg ;//      = 3906;
extern volatile uint32 llWaitingIrq;

//used for sleep timer sync
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;//  =   0;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;//=0;
extern uint32_t  g_TIM2_IRQ_PendingTick;//=0;
extern uint32_t  g_osal_tick_trim;//=0;
extern uint32_t  g_osalTickTrim_mod;//=0;
extern uint32_t  g_TIM2_wakeup_delay;//=0;
extern uint32_t  rtc_mod_value;// = 0;
extern uint32_t  g_counter_traking_cnt;// = 0;

extern uint32 counter_tracking;                // 24bit tracking counter, read from 0x4000f064

extern void wakeup_init(void);

extern void ll_hw_tx2rx_timing_config(uint8 pkt);

enum power_states {
    SYS_POWER_STATE_RUN,       /* C1 state */
    SYS_POWER_STATE_WAIT,       /* C1 state */
    SYS_POWER_STATE_DOZE,     /* C2 state */
    SYS_POWER_STATE_STOP,     /* C2LP state */
    SYS_POWER_STATE_STANDBY,    /* SLEEP state */

    SYS_POWER_STATE_MAX
};

static pmu_handle_t pmu_handle = NULL;
static Sleep_Mode sleepMode = SYSTEM_SLEEP_MODE;// MCU_SLEEP_MODE;
static uint8 bSleepDisable;

#ifdef CONFIG_WDT
#define WDT_TIMEOUT             2000
static wdt_handle_t wdt_handle = NULL;
#endif

#define LPM_FLASH_NAME   "lpm"
#define BOOT_FLASH_NAME  "boot"
#define PMU_ADDROFF      (0x1F0)

#if 0
static rtc_handle_t g_rtc_handle;
static const char *TAG = "soc_pm";
static uint32_t g_bootloadr_addr = 0xFFFFFFFF;
static gpio_pin_handle_t g_gpio_handle = NULL;
static gpio_irq_mode_e g_gpio_mode;

static void rtc_cb(int32_t idx, rtc_event_e event)
{
    LOGI("lpm", "rtc event");
}

static int rtc_init(void)
{
    struct tm current_time;
    int ret = 0;

    current_time.tm_sec     = 10;
    current_time.tm_min     = 26;
    current_time.tm_hour    = 17;
    current_time.tm_mday    = 4;
    current_time.tm_mon     = 7;
    current_time.tm_year    = 117;

    g_rtc_handle = csi_rtc_initialize(0, rtc_cb);

    if (g_rtc_handle == NULL) {
        LOGE("lpm", "rtc init error");
        return -1;
    }

    //run about 10 second
    ret = csi_rtc_set_time(g_rtc_handle, &current_time);

    if (ret < 0) {
        LOGE("lpm", "csi rtc timer set error");
        return -1;
    }

    ret = csi_rtc_start(g_rtc_handle);

    if (ret < 0) {
        LOGE("lpm", "csi rtc start error");
        return -1;
    }

    return 0;
}

static int rtc_timer_start(uint32_t ms)
{
    int32_t ret = 0;
    time_t  sec_time = 0;
    struct tm tm_time;

    ret =  csi_rtc_get_time(g_rtc_handle, &tm_time);

    if (ret < 0) {
        LOGE("lpm", "rtc get time error");
        return -1;
    }

    sec_time = mktime(&tm_time);

    sec_time += (ms / 1000);

    gmtime_r(&sec_time, &tm_time);

    ret = csi_rtc_set_alarm(g_rtc_handle, &tm_time);

    if (ret < 0) {
        LOGE("lpm", "rtc timer config error");
        return -1;
    }

    ret = csi_rtc_enable_alarm(g_rtc_handle, 1);

    if (ret < 0) {
        LOGE("lpm", "rtc timer enable error");
        return -1;
    }

    return ret;
}

/*
        RUN WAIT    DOZE    STOP        STANDBY
EHS     X1  X1      OFF     SHUT DOWN   SHUT DOWN
IHS     X1  X1      OFF     SHUT DOWN   SHUT DOWN
PLL     X1  OFF     OFF     SHUT DOWN   SHUT DOWN
ELS     X1  X1      X1      X1          X1
ILS     X1  X1      X1      X1          X1
CPU     ON  OFF     OFF     SHUT DOWN   SHUT DOWN
FLASH   ON  X       OFF     SHUT DOWN   SHUT DOWN
RAM     ON  X       OFF     OFF         SHUT DOWN
RTC     X   X       X       X           X
PMU     ON  ON      ON      ON          ON
PERIPH  X   X       OFF     SHUT DOWN   SHUT DOWN

X: clock programmable on; power on
ON: clock on; power on
OFF: clock off; power on
*/
static int check_pm_policy(pm_ctx_t *pm_ctx)
{
    int pm_state = 0;
    uint32_t ticks = pm_ctx->suspend_tick;
    uint64_t ms = aos_kernel_tick2ms(ticks);
    int policy = pm_ctx->policy;

    /* policy:
    * 0 = no power saving operation
    * 1 = low power state
    * 2 = deep sleep
    */
    //LOGI(TAG, "suspend %dms", ms);
    if (policy == LPM_POLICY_NO_POWER_SAVE) {
        pm_state = SYS_POWER_STATE_RUN;
    } else if (policy == LPM_POLICY_LOW_POWER) {
        pm_state = SYS_POWER_STATE_WAIT;
    } else if (policy == LPM_POLICY_DEEP_SLEEP) {
        if (pm_ctx->agree_halt) {
            //assist application to halt system
            pm_state = SYS_POWER_STATE_STANDBY;
        } else {
            if (ms <= pm_ctx->ls_ms) {
                pm_state = SYS_POWER_STATE_RUN;
            } else if ((ms > pm_ctx->ls_ms) && (ms <= pm_ctx->ds_ms)) {
                pm_state = SYS_POWER_STATE_WAIT;
            } else if (ms > pm_ctx->ds_ms) {
                // check device if busy first
                // ensure data transfer was completed before device is suspended
                if (device_is_busy()) {
                    pm_state = SYS_POWER_STATE_WAIT;
                } else {
                    pm_state = SYS_POWER_STATE_DOZE;
                }
            }
        }
    }

    //LOGI(TAG, "decide to %d", pm_state);

    return pm_state;
}

void setup_wake_irq(gpio_pin_handle_t handle, gpio_irq_mode_e mode)
{
    g_gpio_handle = handle;
    g_gpio_mode = mode;
}

/*
Mode    Ext Pad GPIO Interrupt  Other IP Interrupt  RTC Reset
Wait    Yes     Yes             Yes   Yes           Yes(reset)
Doze    Yes     Yes             No    Yes           Yes(reset)
Stop    Yes     No              No    Yes           Yes(wakeup)
Standby Yes     No              No    Yes           Yes(wakeup)
*/
static int setup_wake_event_and_sleep(pm_ctx_t *pm_ctx, int pm_state)
{
    int ret = 0;
    static int rtc_inited = 0;

    uint32_t flash_read_addr;
    partition_t eflash_part = -1;
    if (g_bootloadr_addr == 0xFFFFFFFF) {
        eflash_part = partition_open(BOOT_FLASH_NAME);
        hal_logic_partition_t *lp = hal_flash_get_info(eflash_part);
        g_bootloadr_addr = lp->start_addr + PART_ADDR_FLASH_BEGIN;
    }

    if ((pm_state >= SYS_POWER_STATE_DOZE) && (rtc_inited == 0)) {
        LOGI(TAG, "start rtc init");
        rtc_init();
        LOGI(TAG, "end of rtc init");
        rtc_inited = 1;
    }

    switch (pm_state) {
        case SYS_POWER_STATE_WAIT:
            //!was managed by rhino tickless
            //disable rhino tickless, no need to compensate tick
#if 0
            *(volatile unsigned int *)(0xe000e1c0) = 0xffffffff; // reload wakeup_IRQ
            //*(volatile unsigned int *)(0xe000e280) = 0xffffffff; // clear pend IRQ
            csi_vic_set_wakeup_irq(RTC_IRQn);
            csi_vic_set_wakeup_irq(UART2_IRQn);
            csi_vic_set_wakeup_irq(UART1_IRQn);
            csi_vic_set_wakeup_irq(UART0_IRQn);
            csi_vic_set_wakeup_irq(CORET_IRQn);
            csi_vic_set_wakeup_irq(GPIOA_IRQn);
            csi_vic_set_wakeup_irq(GPIOB_IRQn);
            csi_vic_set_wakeup_irq(TIMA0_IRQn);
            LOGI(TAG, "go to wait mode");
            csi_pmu_enter_sleep(pmu_handle, PMU_MODE_SLEEP);
#endif
            break;

        case SYS_POWER_STATE_DOZE:
            //can be waked up by GPIO(network) and RTC
            for (uint32_t i = GPIOA_IRQn; i <= SHA_IRQn; i++) {
                csi_vic_clear_wakeup_irq(i);
            }
            //LOGI(TAG, "1: %x", *(volatile uint32_t *)0x50006008);
            *(volatile uint32_t *)0x50006008 |= 0x100;
            //LOGI(TAG, "2: %x", *(volatile uint32_t *)0x50006008);
            csi_vic_set_wakeup_irq(RTC_IRQn);
            csi_vic_set_wakeup_irq(EXTWAK_IRQn);
            csi_vic_set_wakeup_irq(GPIOA_IRQn);
            rtc_timer_start(aos_kernel_tick2ms(pm_ctx->suspend_tick));
            LOGI(TAG, "go to doze mode\n");
            csi_pmu_enter_sleep(pmu_handle, PMU_MODE_DOZE);
            break;

        case SYS_POWER_STATE_STOP:
            //can be waked up by GPIO(network) and RTC
            csi_vic_set_wakeup_irq(RTC_IRQn);
            rtc_timer_start(aos_kernel_tick2ms(pm_ctx->suspend_tick));
            LOGI(TAG, "go to stop mode\n");
            csi_pmu_enter_sleep(pmu_handle, PMU_MODE_DORMANT);
            break;

        case SYS_POWER_STATE_STANDBY:
            eflash_part = partition_open(LPM_FLASH_NAME);
            partition_read(eflash_part, PMU_ADDROFF, &flash_read_addr, 4);
            if (g_bootloadr_addr != flash_read_addr) {
                partition_erase(eflash_part, PMU_ADDROFF, 1);
                partition_write(eflash_part, PMU_ADDROFF, &g_bootloadr_addr, 4);
                partition_read(eflash_part, PMU_ADDROFF, &flash_read_addr, 4);

                if (g_bootloadr_addr == flash_read_addr) {
                    LOGI(TAG, "LOWPOWEOR ADDR write 0x%x succuss\n", flash_read_addr);
                } else {
                    LOGE(TAG, "LOWPOWEOR ADDR write fail\n");
                }
            }

            if (pm_ctx->alarm_ms) {
                csi_vic_set_wakeup_irq(RTC_IRQn);
                LOGI(TAG, "setup rtc");
                rtc_timer_start(pm_ctx->alarm_ms);  // run about 4 second
                //csi_pmu_config_wakeup_source(pmu_handle, RTC_IRQn, PMU_WAKEUP_TYPE_PULSE, PMU_WAKEUP_POL_HIGH, 1);
            }

            LOGI(TAG, "go to standby mode with %ds\n", pm_ctx->alarm_ms/1000);
            csi_pmu_enter_sleep(pmu_handle, PMU_MODE_STANDBY);
            break;

        default:
            break;
    }

    return ret;
}
#endif

#ifdef CONFIG_WDT
extern void clk_gate_enable(MODULE_e module);
void wdt_wakeup_action()
{
    if (wdt_handle == NULL) {
        return;
    }

    clk_gate_enable(MOD_WDT);
    AP_WDT->EOI;

    AP_WDT->CRR = DW_WDT_CRR_RESET;
    AP_WDT->TORR = 0x00;
    AP_WDT->CR = 0x1F;
    AP_PCR->RESET2 |= 0x4;
}

void wdt_cb_handle(int32_t idx, wdt_event_e event)
{
    if (wdt_handle == NULL) {
        return;
    }

    csi_wdt_restart(wdt_handle);
}

void wdt_init(void)
{
    wdt_handle = csi_wdt_initialize(0, wdt_cb_handle);
    if (wdt_handle == NULL) {
        printf("WDT init Err!\n");
        return;
    }

    if (csi_wdt_set_timeout(wdt_handle, WDT_TIMEOUT) < 0) {
        printf("WDT set timeout Err!\n");
        return;
    }
    csi_wdt_start(wdt_handle);
}
#endif
extern void registers_save(uint32_t *mem, uint32_t *addr, int size);
static uint32_t usart_regs_saved[5];
static void usart_prepare_sleep_action(void)
{
    uint32_t addr = 0x40004000;
    uint32_t read_num=0;
    while (*(volatile uint32_t *)(addr + 0x14) & 0x1) {
        *(volatile uint32_t *)addr;
        if (read_num++ >= 16) {
            break;
        }
    }

    while (*(volatile uint32_t *)(addr + 0x7c) & 0x1);

    *(volatile uint32_t *)(addr + 0xc) |= 0x80;
    registers_save((uint32_t *)usart_regs_saved, (uint32_t *)addr, 2);
    *(volatile uint32_t *)(addr + 0xc) &= ~0x80;
    registers_save(&usart_regs_saved[2], (uint32_t *)addr + 1, 1);
    registers_save(&usart_regs_saved[3], (uint32_t *)addr + 3, 2);
}

static void usart_wakeup_action(void)
{
    uint32_t addr = 0x40004000;

    while (*(volatile uint32_t *)(addr + 0x7c) & 0x1);
    *(volatile uint32_t *)(addr + 0xc) |= 0x80;
    registers_save((uint32_t*)addr, usart_regs_saved, 2);
    *(volatile uint32_t *)(addr + 0xc) &= ~0x80;
    registers_save((uint32_t *)addr + 1, &usart_regs_saved[2], 1);
    registers_save((uint32_t *)addr + 3, &usart_regs_saved[3], 2);
}

__attribute((weak)) int pm_prepare_sleep_action()
{
    hal_ret_sram_enable(RET_SRAM0 | RET_SRAM1 | RET_SRAM2 | RET_SRAM3 | RET_SRAM4);
    usart_prepare_sleep_action();
    return 0;
}

 __attribute((weak)) int pm_after_sleep_action()
{
    drv_pinmux_config(P9, UART_TX);
    drv_pinmux_config(P10, UART_RX);
    usart_wakeup_action();

    return 0;
}

uint8 isSleepAllowInPM(void)
{
    return (bSleepDisable == 0)?TRUE:FALSE;
}

// enable sleep
void enableSleepInPM(uint8_t flag)
{
    bSleepDisable &= ~ flag;
}

// disable sleep
void disableSleepInPM(uint8_t flag)
{
    bSleepDisable |= flag;
}

// set sleep mode
void setSleepMode(Sleep_Mode mode)
{
    sleepMode = mode;
}

// get sleep mode configuration
Sleep_Mode getSleepMode(void)
{
    return sleepMode;
}

void set_sleep_flag(int flag)
{
	if(flag)
    {
		*(volatile uint32_t *) 0x4000f0a8 |= 1 ;
        sleep_flag = SLEEP_MAGIC ;
    }
	else
    {
		*(volatile uint32_t *) 0x4000f0a8 &= ~1;
        sleep_flag = 0 ;
    }
}

void config_RTC11(uint32 time)
{
//    *((volatile uint32_t *)(0xe000e100)) |= INT_BIT_RTC;   // remove, we don't use RTC interrupt

    // comparator configuration
    //sleep_tick = *(volatile uint32_t *) 0x4000f028;         // read current RTC counter

     //align to rtc clock edge
    WaitRTCCount(1);
    g_TIM2_IRQ_to_Sleep_DeltTick = (g_TIM2_IRQ_TIM3_CurrCount>(CP_TIM3->CurrentCount))
                                    ? (g_TIM2_IRQ_TIM3_CurrCount-(CP_TIM3->CurrentCount)): 0;

    sleep_tick = clock_time_rtc();
    if ((sleep_tick + time + 8) >= 0x01000000)              // comparator will overflow. 4: margin for wakeup HW/SW process delay  // update 4->8 to avoid too long app sleep process(2018-09-06)
        time -= (sleep_tick + time + 9) & 0x00ffffff;       // avoid RTC counter overflow during sleep
    *(volatile uint32_t *) 0x4000f02c = sleep_tick + time;  //set RTC comparatr0 value

//	*(volatile uint32_t *) 0x4000f024 |= 1 << 20;           //enable comparator0 envent
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 18;           //counter overflow interrupt
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 15;           //enable comparator0 inerrupt

    *(volatile uint32_t *) 0x4000f024 |= 0x148000;          // combine above 3 statement to save MCU time
}

void enter_sleep_off_mode11(Sleep_Mode mode)
{
    if (mode == SYSTEM_SLEEP_MODE) {
        subWriteReg(0x4000f014,26,26, 1);
        pm_prepare_sleep_action();
        csi_pmu_enter_sleep(pmu_handle, PMU_MODE_DORMANT);
#ifdef CONFIG_DEBUG
        *(volatile uint32_t *) 0x40003814 = 0;
#endif
    }
}

void enterSleepProcess11(uint32 time)
{
    uint32 delta, total, step, temp,error_delt;
    // if allow RC 32KHz tracking, adjust the time according to the bias
    if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
    {
        // 1. read RC 32KHz tracking counter, calculate 16MHz ticks number per RC32KHz cycle

        temp = *(volatile uint32_t *)0x4000f064 & 0x3fff;
        //====== assume the error cnt is (n+1/2) cycle,for this case, it should be 9 or 10

        error_delt =     (temp>STD_CRY32_8_CYCLE_16MHZ_CYCLE)
                    ?   temp- STD_CRY32_8_CYCLE_16MHZ_CYCLE : STD_CRY32_8_CYCLE_16MHZ_CYCLE-temp;


        if(error_delt<ERR_THD_RC32_CYCLE)
        {
           temp = temp;
        }
        else if(error_delt<((STD_CRY32_8_CYCLE_16MHZ_CYCLE>>3)+ERR_THD_RC32_CYCLE))
        {
            //temp = ((temp<<3)*455+2048)>>12;//*455/4096~=1/9
            temp = temp<<3;
            temp = ((temp<<9)-(temp<<6)+(temp<<3)-temp+2048)>>12;
        }
        else
        {
            //temp = ((temp<<3)*410+2048)>>12;//*410/4096~=1/10
            temp = temp<<3;
            temp = ((temp<<9)-(temp<<6)-(temp<<5)-(temp<<3)+(temp<<1)+2048)>>12;
        }

        //check for the abnormal temp value
        counter_tracking = (temp>CRY32_8_CYCLE_16MHZ_CYCLE_MAX) ? counter_tracking : temp;
        //20181204 filter the counter_tracking spur, due to the N+1 issue


        if(g_counter_traking_cnt<1000)
        {
            //before traking converage use hard limitation
            counter_tracking =  (counter_tracking>CRY32_8_CYCLE_16MHZ_CYCLE_MAX || counter_tracking<CRY32_8_CYCLE_16MHZ_CYCLE_MIN)
                                ? g_counter_traking_avg : counter_tracking;

            g_counter_traking_cnt++;
        }
        else
        {

            //after tracking converage use soft limitation
            counter_tracking =  (   counter_tracking > g_counter_traking_avg+(g_counter_traking_avg>>8)
                                ||  counter_tracking < g_counter_traking_avg-(g_counter_traking_avg>>8) )
                                ? g_counter_traking_avg : counter_tracking;

        }

        //one order filer to tracking the average counter_tracking
        g_counter_traking_avg = (7*g_counter_traking_avg+counter_tracking)>>3 ;

        // 2.  adjust the time according to the bias

        step = (counter_tracking) >> 3;           // accurate step = 500 for 32768Hz timer


        if (counter_tracking > STD_CRY32_8_CYCLE_16MHZ_CYCLE)           // RTC is slower, should sleep less RTC tick
        {
            delta = counter_tracking - STD_CRY32_8_CYCLE_16MHZ_CYCLE;   // delta 16MHz tick in 8 32KHz ticks

            total = (time * delta) >> 3;                               // total timer bias in 16MHz tick

            while (total > step)
            {
                total -= step;
                time --;
            }
        }
        else    // RTC is faster, should sleep more RTC tick
        {
            delta = STD_CRY32_8_CYCLE_16MHZ_CYCLE - counter_tracking;   // delta 16MHz tick in 8 32KHz ticks
            total = (time * delta) >> 3;                               // total timer bias in 16MHz tick
            while (total > step)
            {
                total -= step;
                time ++;
            }
        }
    }

    // backup registers         ------   none now

    // backup timers            ------   none now

    //===20180417 added by ZQ
    //   for P16,P17
    subWriteReg(0x4000f01c,6,6,0x00);   //disable software control

    //printf("sleep time %d\r\n", time);
    // 3. config wakeup timer
    config_RTC11(time);
//    config_RTC(327680);
//    *(volatile uint32_t *)0x4000f00C &= ~(1 << 12);
//    *(volatile uint32_t *)0x4000f00C |= (3 << 13);
//    *(volatile uint32_t *)0x4000f0A0 |= (1 << 14);

    // 4. app could add operation before sleep
//    app_sleep_process();
    //====== set sram retention
    //    hal_pwrmgr_RAM_retention_set();           // IMPORTANT: application should set retention in app_sleep_process

    ll_debug_output(DEBUG_ENTER_SYSTEM_SLEEP);

    // 5. set sleep flag(AON reg & retention SRAM variable)
    set_sleep_flag(1);

    // 6. trigger system sleep
    enter_sleep_off_mode11(SYSTEM_SLEEP_MODE);
}

/******************************************
// SOC interfaces
******************************************/
int sys_soc_init(pm_ctx_t *pm_ctx)
{
    int ret = 1;

    pmu_handle = csi_pmu_initialize(0, NULL);

   // ret = aos_kv_getint(KV_LPM_POLICY, &pm_ctx->policy);
    if (ret != 0) {
        pm_ctx->policy = 2;
    }
  //  ret = aos_kv_getint(KV_LPM_LS_THRES, (int *)&pm_ctx->ls_ms);
    if (ret != 0) {
        pm_ctx->ls_ms = MIN_TIME_TO_SLEEP;
    }
//    ret = aos_kv_getint(KV_LPM_DS_THRES, (int *)&pm_ctx->ds_ms);
    if (ret != 0) {
        pm_ctx->ds_ms = MIN_TIME_TO_SUSPEND;
    }

    bSleepDisable = 0;

    return 0;
}

int sys_soc_suspend(pm_ctx_t *pm_ctx)
{
#if 0
    int pm_state = SYS_POWER_STATE_RUN;
    //uint32_t ticks = pm_ctx->suspend_tick;

    pm_state = check_pm_policy(pm_ctx);

    if (pm_state < SYS_POWER_STATE_WAIT) {
        return pm_state;
    }

    if ((pm_state > SYS_POWER_STATE_WAIT) && (pm_state < SYS_POWER_STATE_STANDBY)) {
        device_manage_power(DEVICE_PM_SUSPEND_STATE);
    } else if (pm_state == SYS_POWER_STATE_STANDBY) {
        device_manage_power(DEVICE_PM_OFF_STATE);
    }

    setup_wake_event_and_sleep(pm_ctx, pm_state);

    return pm_state;
#else
  uint32        next;
  uint32        rtc_counter;
  uint32 temp1, temp2;

  // Hold off interrupts.
  HAL_ENTER_CRITICAL_SECTION( intState );

  if (isSleepAllowInPM()) {
            //printf("the next\r\n");
//    if ( (pwrmgr_attribute.pwrmgr_device != PWRMGR_ALWAYS_ON) ) {
        if (llWaitingIrq)       // bug correct 2017-7-5, osal sleep function may be interrupted by LL irq and LL timer could be changed
        {                       // Don't triggered System sleep in such case
            HAL_EXIT_CRITICAL_SECTION( intState );
            __WFI();
            return SYS_POWER_STATE_RUN;
        }

        //===============================================================================================================
        // ZQ: add 20180712. For 32K RC Tracking
        // Ensure wakup enough time before enter sleep
        // RTC counter_tracking is fatched at enter_sleep_process,
        // Need to reserve enough time for 16M Xtal settling, then fatch the RTC counter_traking.
        // When wakeup time is not enough, turn on the IRQ and _WFI,waiting for the timer_irq or peripheral_irq
        //
        if(pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW) {
            uint32 cnt1 = clock_time_rtc();
            uint32 delt = (cnt1>=g_wakeup_rtc_tick) ? cnt1-g_wakeup_rtc_tick : (0x00ffffff-g_wakeup_rtc_tick+cnt1);

            if(delt<(pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K])) {
                HAL_EXIT_CRITICAL_SECTION( intState );
                __WFI();
                return SYS_POWER_STATE_RUN;
            }
        }
        //===============================================================================================================
        // Hold off interrupts.
//        HAL_ENTER_CRITICAL_SECTION( intState );
        // the comparator of RTC not consider loop case, it will interrupt immediately if comparator value smaller then threshold
        // walkaround: sleep time will be decreased to avoid RTC counter overflow when sleep

        //rtc_counter = *(volatile uint32_t *)0x4000f028 & 0x00ffffff;     // read current RTC counter
        rtc_counter = clock_time_rtc();
        rtc_counter = 0x00ffffff - rtc_counter;

        if(pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW) {
            //when next>rtc_counter, use rc32k_rtc_counter 20180324
            //becase this time will be convert to rc32k in enterSleepProcess once more
//            rc32k_rtc_counter = (((rtc_counter << 7) - (rtc_counter << 2) - (rtc_counter << 1)) >>2 )   /* rtc_counter * (128-4-2)/4 */
//                    +(((rtc_counter << 3)+ rtc_counter ) >> 9 ) ; /* rtc_counter *9/512 */
//            //check for the abnormal counter_tracking value
            counter_tracking =  (counter_tracking>CRY32_8_CYCLE_16MHZ_CYCLE_MAX || counter_tracking<CRY32_8_CYCLE_16MHZ_CYCLE_MIN)
                            ? g_counter_traking_avg : counter_tracking;

            rtc_counter =   ((((rtc_counter&0xffff0000)>>16)*counter_tracking)<<9)
                            + (((rtc_counter&0xffff)*counter_tracking)>>7);

//            //rtc_counter = (rtc_counter << 5) - (rtc_counter << 1) + (rtc_counter >> 1);     // convert RTC tick to us, multiply 30.5
//            rtc_counter = (((rtc_counter << 7) - (rtc_counter << 2) - (rtc_counter << 1)) >>2 )   /* rtc_counter * (128-4-2)/4 */
//                    +(((rtc_counter << 3)+ rtc_counter ) >> 9 ) ; /* rtc_counter *9/512 */
        }
        else
        {

            //rtc_counter = (rtc_counter << 5) - (rtc_counter << 1) + (rtc_counter >> 1);     // convert RTC tick to us, multiply 30.5
            rtc_counter = (((rtc_counter << 7) - (rtc_counter << 2) - (rtc_counter << 1)) >>2 )   /* rtc_counter * (128-4-2)/4 */
                    +(((rtc_counter << 3)+ rtc_counter ) >> 9 ) ; /* rtc_counter *9/512 */
        }
        // Get next time-out
        next = osal_next_timeout() * 1000;         // convert from ms to us

        if (next == 0)                     // no OSAL timer running, only consider rtc_counter & ll timer
            next = rtc_counter;
        else
            next = (next > rtc_counter) ? rtc_counter : next;

        // if LL timer is not kick off, set remain time as max sleep time
        ll_remain_time = read_LL_remainder_time();
        // ZQ: add 20180514. When timer1 overrun happend, turn on the IRQ and return.
        // TIM1_IRQ_Handle will process the TIM1_IRQ evernt
        if((CP_TIM1->status &0x1)==1)
        {
            HAL_EXIT_CRITICAL_SECTION( intState );

            return SYS_POWER_STATE_RUN;
        }
        if (llState == LL_STATE_IDLE)   // (ll_remain_time == 0 || llState == LL_STATE_IDLE)
            ll_remain_time = pGlobal_config[MAX_SLEEP_TIME];
        // remove below decision 2018-04-04. In LL_STATE_IDLE, below statement will fail the sleep process
        // correct 17-09-11, timer IRQ during sleep process will cause read fault ll_remain_time value
        // walkaround here, it is better to modify timer driver to clear all register in ISR
//        if (CP_TIM1->CurrentCount > CP_TIM1->LoadCount)// && (CP_TIM1->ControlReg & 0x1) != 0)
//        {
//            HAL_EXIT_CRITICAL_SECTION( intState );
//            return;
//        }
        next = (next > ll_remain_time) ? ll_remain_time : next;

        uint32_t kernel_us = aos_kernel_tick2ms(pm_ctx->suspend_tick) * 1000;
        //printf("suspend tick is %d %d\n", kernel_us, next);
        next = (next > kernel_us) ? kernel_us : next;

        if (getSleepMode() == MCU_SLEEP_MODE
             || llWaitingIrq       // system sleep will not trigger when ll HW processing
             || next < (pGlobal_config[MIN_SLEEP_TIME] + pGlobal_config[WAKEUP_ADVANCE]))   // update 2018-09-06
//             || pwrmgr_attribute.pwrmgr_task_state != 0)    // Are all tasks in agreement to conserve
        {    // MCU sleep mode
            HAL_EXIT_CRITICAL_SECTION( intState );
            __WFI();
            return SYS_POWER_STATE_RUN;
        }
        else
        {
            next = next - pGlobal_config[WAKEUP_ADVANCE];    // wakeup advance: consider HW delay, SW re-init time, ..., etc.
            if (next > pGlobal_config[MAX_SLEEP_TIME]) // consider slave latency, we may sleep up to 16s between 2 connection events
                next = pGlobal_config[MAX_SLEEP_TIME];
            // covert time to RTC ticks
            // RTC timer clock is 32768HZ, about 30.5us per tick
            // time / 30.5 = time * ( 1/32 + 1 / 512 - 1 / 2048 + 1/16384 - 1/65536 + 1/128K - 1/512K...)
            //             = time * ( 1/32 + 1 / 512 - 1 / 2048 + 1/16384 - 1/128K - 1/512K...) = 32766.3 * 10e-6
            temp1 = next >> 9;
            temp2 = next >> 11;
            next = next >> 5;
            next = next + temp1 - temp2 + (temp2 >> 3) - (temp2 >> 6) - (temp2 >> 8);
            //next = next + temp1 - temp2 + (temp2 >> 4);

            //printf("the next %d\r\n", next);
            enterSleepProcess11(next);
            return SYS_POWER_STATE_STOP;
        }
//    }
  }
    // Re-enable interrupts.
    HAL_EXIT_CRITICAL_SECTION(intState);
    return SYS_POWER_STATE_RUN;
#endif
}

int sys_soc_resume(pm_ctx_t *pm_ctx, int pm_state)
{
#if 0
    uint32_t actual_suspend_ticks = 0;  //

    if (pm_state <= SYS_POWER_STATE_WAIT) {
        //!managed by rhino
        //support systick, no need to compensate tick
        actual_suspend_ticks = 0;
    } else if ((pm_state > SYS_POWER_STATE_WAIT) && (pm_state < SYS_POWER_STATE_STANDBY)) {
        device_manage_power(DEVICE_PM_ACTIVE_STATE);
        //get rtc value, may be waked up by network card
        //set actual_suspend_ticks;
        actual_suspend_ticks = pm_ctx->suspend_tick;//dummy

        LOGI(TAG, "Wake from Deep Sleep");
    }

    return actual_suspend_ticks;
#else

    if (pm_state == SYS_POWER_STATE_RUN) {
        return 0;
    }

    uint32_t current_RTC_tick;
    uint32_t wakeup_time, wakeup_time0, next_time;
    uint32_t sleep_total;
    uint32_t dlt_tick;

    subWriteReg(0x4000f01c,21,17,0);

    if (sleep_flag != SLEEP_MAGIC) {                 // enter this branch not in sleep/wakeup scenario
        set_sleep_flag(0);

        // software reset
        *(volatile uint32 *)0x40000010 &= ~0x2;    // bit 1: M0 cpu reset pulse, bit 0: M0 system reset pulse.
    }

    // restore HW registers
    wakeup_init();

#ifdef CONFIG_WDT
    wdt_wakeup_action();
#endif
    pm_after_sleep_action();

    HAL_EXIT_CRITICAL_SECTION(intState);
//    console_init(0, 115200, 128);
//    usart_handle_t console_handle = csi_usart_initialize(0, NULL);
//    csi_usart_config(console_handle, 115200, USART_MODE_ASYNCHRONOUS, USART_PARITY_NONE, USART_STOP_BITS_1, USART_DATA_BITS_8);

    //config the tx2rx timing according to the g_rfPhyPktFmt
    ll_hw_tx2rx_timing_config(g_rfPhyPktFmt);

    if (pGlobal_config[LL_SWITCH] & LL_RC32K_SEL) {
        subWriteReg(0x4000f01c,16,7,0x3fb);   //software control 32k_clk
        subWriteReg(0x4000f01c,6,6 ,0x01);    //enable software control

    } else {
        subWriteReg(0x4000f01c,9,8,0x03);   //software control 32k_clk
        subWriteReg(0x4000f01c,6,6,0x00);   //disable software control
    }

    //restart the TIM2 to align the RTC
    //----------------------------------------------------------
    //stop the 625 timer
    CP_TIM2->ControlReg=0x0;
    CP_TIM2->ControlReg=0x2;
    CP_TIM2->LoadCount = 2500;
    //----------------------------------------------------------
    //wait rtc cnt change
    WaitRTCCount(1);
    //----------------------------------------------------------
    //restart the 625 timer
    CP_TIM2->ControlReg=0x3;

    current_RTC_tick = clock_time_rtc();
    wakeup_time0 = read_current_fine_time();
    g_wakeup_rtc_tick = clock_time_rtc();

    // rf initial entry, will be set in app
    rf_phy_ini();

    //calculate the rtc ticks value between before sleep & wakeup
    if(current_RTC_tick > sleep_tick) {
        dlt_tick = current_RTC_tick - sleep_tick;
    } else {
        dlt_tick = current_RTC_tick+0x00ffffff - sleep_tick;

    }

    //convert the rtc ticks to timer ticks
    if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW) {
        //sleep_total = ((current_RTC_tick - sleep_tick) * counter_tracking) >> 7; // shift 4 for 16MHz -> 1MHz, shift 3 for we count 8 RTC tick
        sleep_total =   ((((dlt_tick &0xffff0000)>>16)*counter_tracking)<<9)
                        + (((dlt_tick &0xffff)*counter_tracking)>>7);
    } else {     // time = tick * 1000 0000 / f (us). f = 32000Hz for RC, f = 32768Hz for crystal. We also calibrate 32KHz RC to 32768Hz
        //sleep_total =  ((current_RTC_tick - sleep_tick) * TIMER_TO_32K_CRYSTAL) >> 2;

        //fix sleep timing error
        sleep_total = ( ( (dlt_tick<<7)-(dlt_tick<<2)-(dlt_tick<<1) +2)       >>2 )   /* dlt_tick * (128-4-2)/4 */
                     +( ( (dlt_tick<<3)+ dlt_tick +128)                       >>9 ) ; /* dlt_tick *9/512 */
                                                                                        //+2,+128 for zero-mean quanization noise

    }

    // caculate before sleep & wakeup sw delay
    g_osal_tick_trim = (pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM]+g_TIM2_IRQ_to_Sleep_DeltTick+2500-g_TIM2_IRQ_PendingTick) >> 2;        //16 is used to compensate the cal delay
    g_osalTickTrim_mod+=(pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM]+g_TIM2_IRQ_to_Sleep_DeltTick+2500-g_TIM2_IRQ_PendingTick) & 0x03;     //16 is used to compensate the cal delay
    if(g_osalTickTrim_mod > 4) {
        g_osal_tick_trim += 1;
        g_osalTickTrim_mod = g_osalTickTrim_mod % 4;
    }

    // calculate the systick
    osal_sys_tick += (sleep_total + g_osal_tick_trim) / 625;      // convert to 625us systick
    rtc_mod_value += ((sleep_total + g_osal_tick_trim) % 625);

    if(rtc_mod_value > 625) {
      osal_sys_tick += 1;
      rtc_mod_value = rtc_mod_value%625;
    }
    extern void osalTimeUpdate( void );
    osalTimeUpdate();

    // SW delay
    wakeup_time = read_current_fine_time() - wakeup_time0;

    //calculate the ll timer1 remain time
    next_time = 0;
    if (ll_remain_time > sleep_total + wakeup_time)
    {
        next_time = ll_remain_time - sleep_total - wakeup_time;
        // restore LL timer
        set_timer1(next_time);
    }
    else
    {
        // should not be here
        set_timer1(1000);
        //  next_time = 0xffff;
    }

    // app could add operation after wakeup
//    app_wakeup_process();

    ll_debug_output(DEBUG_WAKEUP);

    set_sleep_flag(0);
//    asm volatile("b .\n\r");
    return aos_kernel_ms2tick((sleep_total + g_osal_tick_trim) / 1000);
#endif
}


