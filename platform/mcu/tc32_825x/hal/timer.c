/********************************************************************************************************
 * @file     timer.c 
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
#include "hal/soc/soc.h"
#include "hal/soc/timer.h"
#include "drivers/8258/compiler.h"
#include "drivers/8258/irq.h"
#include "k_api.h"
#include "common/ring_buffer.h"
#include "tl_common.h"
#include "drivers.h"

#define TIMER_MODE_SYSCLK          0
#define	TIMER_MODE_GPIO_TRIGGER	   1
#define	TIMER_MODE_GPIO_WIDTH	   2
#define	TIMER_MODE_TICK			   3

static hal_timer_cb_t hal_timer_cb = NULL;
static void *hal_timer_cb_arg = NULL;
static bool hal_timer_reload = true;

_attribute_ram_code_ void timer1_irq()
{
	u32 src = reg_irq_src;
	
	if(src & FLD_IRQ_TMR1_EN){
		reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status	

		if (!hal_timer_reload) {
			reg_tmr_ctrl &= ~FLD_TMR1_EN;
		}

		if (hal_timer_cb != NULL) {
			hal_timer_cb(hal_timer_cb_arg);
		}
	}
}


int32_t hal_timer_init(timer_dev_t *tim)
{
	// tim->port // only timer1
	hal_timer_cb = tim->config.cb;
	hal_timer_cb_arg = tim->config.arg;

	reg_irq_mask &= ~FLD_TMR1_EN;
	reg_tmr_sta = FLD_TMR_STA_TMR1;
	reg_tmr_ctrl &= (~FLD_TMR1_MODE);
	reg_tmr_ctrl |= (TIMER_MODE_SYSCLK<<4);

	reg_tmr1_tick = 0;
	reg_tmr1_capt = tim->config.period * CLOCK_SYS_CLOCK_1US;
	hal_timer_reload = tim->config.reload_mode == TIMER_RELOAD_AUTO ? true : false;
	
	return 0;
}


int32_t hal_timer_start(timer_dev_t *tim)
{
	reg_tmr_sta = FLD_TMR_STA_TMR1;

	reg_irq_mask |= FLD_TMR1_EN;
	reg_tmr_ctrl |= FLD_TMR1_EN;
	
	return 0;
}


void hal_timer_stop(timer_dev_t *tim)
{
	reg_tmr_ctrl &= ~FLD_TMR1_EN;
	reg_irq_mask &= ~FLD_TMR1_EN;
	
	return 0;
}


int32_t hal_timer_finalize(timer_dev_t *tim)
{
	reg_tmr_ctrl &= ~FLD_TMR1_EN;
	reg_tmr_sta = FLD_TMR_STA_TMR1;
	reg_irq_mask &= ~FLD_TMR1_EN;
	
	return 0;
}


