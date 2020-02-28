/********************************************************************************************************
 * @file     pm.h
 *
 * @brief    This is the header file for TLSR8258
 *
 * @author	 junwei.lu@telink-semi.com;
 * @date     May 8, 2018
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

#pragma once

#include "bsp.h"
#include "gpio.h"

#pragma pack(1)

#ifndef PM_TIM_RECOVER_MODE
#define PM_TIM_RECOVER_MODE					1
#endif

#define 	BLT_RESET_WAKEUP_TIME_1000		1
#define 	BLT_RESET_WAKEUP_TIME_1500		0
#define 	BLT_RESET_WAKEUP_TIME_2000		0
#define 	BLT_RESET_WAKEUP_TIME_3000		0
#define 	BLT_RESET_WAKEUP_TIME_4000		0


#if (BLT_RESET_WAKEUP_TIME_1000)
	#define PM_XTAL_DELAY_DURATION    	   700
	#define PM_DCDC_DELAY_DURATION        1000
	#define EARLYWAKEUP_TIME_US_DEEP      1100
	#define EARLYWAKEUP_TIME_US_SUSPEND   1250
	#define EMPTYRUN_TIME_US       	      1500
#elif (BLT_RESET_WAKEUP_TIME_1500)
	#define PM_XTAL_DELAY_DURATION    	   750
	#define PM_DCDC_DELAY_DURATION        1500
	#define EARLYWAKEUP_TIME_US           1800
	#define EMPTYRUN_TIME_US       	      2100
#elif (BLT_RESET_WAKEUP_TIME_2000)
	#define PM_XTAL_DELAY_DURATION    	  1000
	#define PM_DCDC_DELAY_DURATION        2000
	#define EARLYWAKEUP_TIME_US           2300
	#define EMPTYRUN_TIME_US       	      2600
#elif(BLT_RESET_WAKEUP_TIME_3000)
	#define PM_XTAL_DELAY_DURATION    	  1000
	#define PM_DCDC_DELAY_DURATION        3000
	#define EARLYWAKEUP_TIME_US           3300
	#define EMPTYRUN_TIME_US       	      3600
#elif(BLT_RESET_WAKEUP_TIME_4000)
	#define PM_XTAL_DELAY_DURATION    	  2000
	#define PM_DCDC_DELAY_DURATION        4000
	#define EARLYWAKEUP_TIME_US           4300
	#define EMPTYRUN_TIME_US       	      4600

#else
#endif


static inline void usb_dp_pullup_en (int en)
{
	unsigned char dat = ReadAnalogReg(0x0b);
	if (en) {
		dat = dat | BIT(7);
	}
	else
	{
		dat = dat & 0x7f ;
	}

	WriteAnalogReg (0x0b, dat);
}





//analog register below can store infomation when MCU in deepsleep mode
//store your information in these ana_regs before deepsleep by calling analog_write function
//when MCU wakeup from deepsleep, read the information by by calling analog_read function

//these two below are stable
#define DEEP_ANA_REG0    				0x3a
#define DEEP_ANA_REG1    				0x3b


//ana3c system used, user can not use
#define SYS_DEEP_ANA_REG 				0x3c
#define SYS_NEED_REINIT_EXT32K			BIT(0)



//these analog register below may have some problem when user enter deepsleep but ERR wakeup
// for example, when set a GPIO PAD high wakeup deepsleep, but this gpio is high before
// you call func cpu_sleep_wakeup, then deepsleep will be ERR wakeup, these analog register
//   infomation loss.
#define DEEP_ANA_REG6    				0x35
#define DEEP_ANA_REG7    				0x36
#define DEEP_ANA_REG8    				0x37
#define DEEP_ANA_REG9    				0x38
#define DEEP_ANA_REG10   				0x39





typedef enum {
	SUSPEND_MODE						= 0,
	DEEPSLEEP_MODE						= 0x80,
	DEEPSLEEP_MODE_RET_SRAM_LOW16K  	= 0x43,
	DEEPSLEEP_MODE_RET_SRAM_LOW32K  	= 0x07,


	DEEPSLEEP_RETENTION_FLAG			= 0x7F,
}SleepMode_TypeDef;



//set wakeup source
typedef enum {
	 PM_WAKEUP_PAD   = BIT(4),    		SUSPENDWAKEUP_SRC_PAD = BIT(4),    DEEPWAKEUP_SRC_PAD   = BIT(4),
	 PM_WAKEUP_TIMER = BIT(6),	  		SUSPENDWAKEUP_SRC_TIMER = BIT(6),  DEEPWAKEUP_SRC_TIMER = BIT(6),

	 PM_TIM_RECOVER_START =	BIT(14),
	 PM_TIM_RECOVER_END   =	BIT(15),
}SleepWakeupSrc_TypeDef;




//wakeup status from return value of "cpu_sleep_wakeup"
enum {
	 WAKEUP_STATUS_COMP   = BIT(0),  //wakeup by comparator
	 WAKEUP_STATUS_TIMER  = BIT(1),
	 WAKEUP_STATUS_CORE   = BIT(2),
	 WAKEUP_STATUS_PAD    = BIT(3),

	 STATUS_GPIO_ERR_NO_ENTER_PM  = BIT(7),

	 STATUS_ENTER_SUSPEND  = BIT(30),
};

#define 	WAKEUP_STATUS_TIMER_CORE	( WAKEUP_STATUS_TIMER | WAKEUP_STATUS_CORE)
#define 	WAKEUP_STATUS_TIMER_PAD		( WAKEUP_STATUS_TIMER | WAKEUP_STATUS_PAD)



typedef struct{
	unsigned char is_deepRetn_back;
	unsigned char is_pad_wakeup;
	unsigned char wakeup_src;
}pm_para_t;

extern pm_para_t	pmParam;

typedef struct{
	unsigned char conn_mark;
	unsigned char ext_cap_en;
	unsigned char pad32k_en;
	unsigned char pm_enter_en;
}misc_para_t;

extern misc_para_t blt_miscParam;

#if (PM_TIM_RECOVER_MODE)
	typedef struct{
		unsigned int   tick_sysClk;
		unsigned int   tick_32k;
		unsigned char  recover_flag;
	}pm_tim_recover_t;

	extern pm_tim_recover_t	pm_timRecover;
#endif

typedef int (*suspend_handler_t)(void);

typedef int (*cpu_pm_handler_t)(SleepMode_TypeDef sleep_mode,  SleepWakeupSrc_TypeDef wakeup_src, unsigned int  wakeup_tick);

typedef void (*check_32k_clk_handler_t)(void);

typedef unsigned int (*get_32k_clk_handler_t)(void);

typedef unsigned int (*pm_tim_recover_handler_t)(unsigned int);

extern 	cpu_pm_handler_t  		 cpu_sleep_wakeup;
extern  pm_tim_recover_handler_t pm_tim_recover;
extern  check_32k_clk_handler_t  check_32k_clk_stable;
extern  get_32k_clk_handler_t  	 get_32k_tick;
extern  suspend_handler_t 		 func_before_suspend;
extern 	unsigned short 			 tick_32k_calib;
extern  unsigned int 			 tick_cur;
extern  unsigned int 			 tick_32k_cur;
extern  unsigned char       	 pm_long_suspend;

#define PM_Get32kTick		cpu_get_32k_tick
#define pm_start			sleep_start

void  sleep_start(void);
int  cpu_sleep_wakeup_32k_rc(SleepMode_TypeDef sleep_mode,  SleepWakeupSrc_TypeDef wakeup_src, unsigned int  wakeup_tick);
int  cpu_sleep_wakeup_32k_xtal(SleepMode_TypeDef sleep_mode,  SleepWakeupSrc_TypeDef wakeup_src, unsigned int  wakeup_tick);
void check_32k_clk_stable_32k_xtal(void);
unsigned int get_32k_tick_32k_xtal(void);
unsigned int pm_tim_recover_32k_rc(unsigned int now_tick_32k);
unsigned int pm_tim_recover_32k_xtal(unsigned int now_tick_32k);



/******************************* User Interface  ************************************/
void start_reboot(void);
void cpu_wakeup_init(void);
void cpu_stall_wakeup_by_timer0(unsigned int tick_stall);
void cpu_stall_wakeup_by_timer1(unsigned int tick_stall);
void cpu_stall_wakeup_by_timer2(unsigned int tick_stall);
void cpu_set_gpio_wakeup (GPIO_PinTypeDef pin, GPIO_LevelTypeDef pol, int en);
void bls_pm_registerFuncBeforeSuspend (suspend_handler_t func );


static inline int pm_is_MCU_deepRetentionWakeup(void)
{
	return pmParam.is_deepRetn_back;
}

static inline int pm_is_deepPadWakeup(void)
{
	return pmParam.is_pad_wakeup;
}


static inline void blc_pm_select_internal_32k_crystal(void)
{
	cpu_sleep_wakeup 	 	= cpu_sleep_wakeup_32k_rc;
	pm_tim_recover  	 	= pm_tim_recover_32k_rc;

	//blt_miscParam.pad32k_en 	= 0; // default: use internal 32k rc
	blt_miscParam.pm_enter_en 	= 1; // allow enter pm, 32k rc does not need to wait for 32k clk to be stable
}

static inline void blc_pm_select_external_32k_crystal(void)
{
	cpu_sleep_wakeup 	 	= cpu_sleep_wakeup_32k_xtal;
	pm_tim_recover		 	= pm_tim_recover_32k_xtal;
	check_32k_clk_stable 	= check_32k_clk_stable_32k_xtal;
	get_32k_tick         	= get_32k_tick_32k_xtal;

	blt_miscParam.pad32k_en 	= 1; // set '1': 32k clk src use external 32k crystal
	//blt_miscParam.pm_enter_en = 0; // default: not allow enter pm
}





/************************* Stack Interface, user can not use!!! ***************************/
unsigned int cpu_get_32k_tick(void);
void soft_reboot_dly13ms_use24mRC(void);

//only for debug below, will remove them later
void shutdown_gpio(void);  //for debug

extern unsigned int flash_rdid;
static inline unsigned int pm_get_flash_rdid(void)
{
	return flash_rdid;
}

#pragma pack()

