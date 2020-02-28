/********************************************************************************************************
 * @file     pwm.h 
 *
 * @brief    This is the header file for TLSR8258
 *
 * @author	 junyuan.zhang@telink-semi.com;
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

#ifndef PWM_H_
#define PWM_H_

#include "register.h"
#include "clock.h"

/**
 * @brief  enum variable, the number of PWM channels supported
 */
typedef enum {
	PWM0_ID = 0,
	PWM1_ID,
	PWM2_ID,
	PWM3_ID,
	PWM4_ID,
	PWM5_ID,
}pwm_id;

/**
 * @brief  enum variable used for PWM work mode setting
 */
typedef enum{
	PWM_NORMAL_MODE   = 0x00,
	PWM_COUNT_MODE    = 0x01,
	PWM_IR_MODE       = 0x03,
	PWM_IR_FIFO_MODE  = 0x07,
	PWM_IR_DMA_FIFO_MODE  = 0x0F,
}pwm_mode;

/**
 * pwm interrupt source
 */
typedef enum{
	PWM_IRQ_PWM0_PNUM =					BIT(0),
	PWM_IRQ_PWM0_IR_DMA_FIFO_DONE =		BIT(1),
	PWM_IRQ_PWM0_FRAME =				BIT(2),
	PWM_IRQ_PWM1_FRAME =				BIT(3),
	PWM_IRQ_PWM2_FRAME =				BIT(4),
	PWM_IRQ_PWM3_FRAME =				BIT(5),
	PWM_IRQ_PWM4_FRAME =				BIT(6),
	PWM_IRQ_PWM5_FRAME =				BIT(7),
}PWM_IRQ;

/**
 * @brief     This fuction servers to set pwm mode.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] mode - variable of enum to indicates the pwm mode.
 * @return	  none.
 */
static inline void pwm_set_mode(pwm_id id, pwm_mode mode){
	if(PWM0_ID == id){
		reg_pwm0_mode = mode;  //only PWM0 has count/IR/fifo IR mode
	}
}

/**
 * @brief     This fuction servers to set pwm clock frequency
 * @param[in] system_clock_hz - variable to set system clock hz.
 * @param[in] pwm_clk - variable of the pwm clock.
 * @return	  none.
 */
static inline void pwm_set_clk(int system_clock_hz, int pwm_clk){
	reg_pwm_clk = (int)system_clock_hz /pwm_clk - 1;
}

/**
 * @brief     This fuction servers to set pwm count status(CMP) time.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_cmp(pwm_id id, unsigned short cmp_tick){
	reg_pwm_cmp(id) = cmp_tick;
}

/**
 * @brief     This fuction servers to set pwm cycle time.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @return	  none.
 */
static inline void pwm_set_cycle(pwm_id id, unsigned short cycle_tick){
	reg_pwm_max(id) = cycle_tick;
}

/**
 * @brief     This fuction servers to set pwm cycle time & count status.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_cycle_and_duty(pwm_id id, unsigned short cycle_tick, unsigned short cmp_tick){
	reg_pwm_cycle(id) = MASK_VAL(FLD_PWM_CMP, cmp_tick, FLD_PWM_MAX, cycle_tick);
}

/**
 * @brief     This fuction servers to set pwm cycle time & count status.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_pwm0_shadow_cycle_and_duty(unsigned short cycle_tick, unsigned short cmp_tick)
{
	reg_pwm_tcmp0_shadow = cmp_tick;
	reg_pwm_tmax0_shadow = cycle_tick;
}

/**
 * @brief     This fuction servers to set the pwm phase.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] phase - variable of the pwm phase.
 * @return	  none.
 */
static inline void pwm_set_phase(pwm_id id, unsigned short phase){
	reg_pwm_phase(id) = phase;
}

/**
 * @brief     This fuction servers to set the pwm pulse number.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] pulse_num - variable of the pwm pulse number.
 * @return	  none.
 */
static inline void pwm_set_pulse_num(pwm_id id, unsigned short pulse_num){
	if(PWM0_ID == id){
		reg_pwm_pulse_num = pulse_num;
	}

}

/**
 * @brief     This fuction servers to start the pwm.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_start(pwm_id id){
	if(PWM0_ID == id){
		BM_SET(reg_pwm0_enable, BIT(0));
	}
	else{
		BM_SET(reg_pwm_enable, BIT(id));
	}
}

/**
 * @brief     This fuction servers to stop the pwm.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_stop(pwm_id id){
	if(PWM0_ID == id){
		BM_CLR(reg_pwm0_enable, BIT(0));
	}
	else{
		BM_CLR(reg_pwm_enable, BIT(id));
	}
}

/**
 * @brief     This fuction servers to revert the PWMx.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_revert(pwm_id id){
	reg_pwm_invert |= BIT(id);
}

/**
 * @brief     This fuction servers to revert the PWMx_N.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_n_revert(pwm_id id){
	reg_pwm_n_invert |= BIT(id);
}

/**
 * @brief     This fuction servers to enable the pwm polarity.
 * @param[in] pwm_id - variable of enum to select the pwm number.
 * @param[in] en: 1 enable. 0 disable.
 * @return	  none.
 */
static inline void pwm_polo_enable(pwm_id id, int en){
	if(en){
		BM_SET(reg_pwm_pol, BIT(id));
	}else{
		BM_CLR(reg_pwm_pol, BIT(id));
	}
}

/**
 * @brief     This fuction servers to enable the pwm interrupt.
 * @param[in] PWM_IRQ - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline void pwm_set_interrupt_enable(PWM_IRQ irq){
//	if(en){
//		BM_SET(reg_pwm_irq_mask, irq);
//	}else{
//		BM_CLR(reg_pwm_irq_mask, irq);
//	}
	BM_SET(reg_pwm_irq_mask, irq);
}

/**
 * @brief     This fuction servers to disable the pwm interrupt.
 * @param[in] PWM_IRQ - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline void pwm_set_interrupt_disable(PWM_IRQ irq){
	BM_CLR(reg_pwm_irq_mask, irq);
}

/**
 * @brief     This fuction servers to clear the pwm interrupt.
 * @param[in] PWM_IRQ - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline void pwm_clear_interrupt_status( PWM_IRQ irq){
	reg_pwm_irq_sta = irq;
}

/**********************************************************************************************
 *
 *     PWM FIFO DMA MODE
 *
 *********************************************************************************************/

typedef enum{
	PWM0_PULSE_NORMAL =		0,       // duty cycle and period from TCMP0/TMAX0 					 0x794~0x797
	PWM0_PULSE_SHADOW =		BIT(14), // duty cycle and period from TCMP0_SHADOW / TMAX0_SHADOW   0x7c4~0x7c7
}Pwm0Pulse_SelectDef;

/**
 * @brief     This fuction servers to config the pwm's dma wave form.
 * @param[in] carrier_en - must 1 or 0.
 * @param[in] Pwm0Pulse_SelectDef - type of pwm0's pulse.
 * @param[in] pulse_num - the number of pulse.
 * @return	  none.
 */
static inline unsigned short pwm_config_dma_fifo_waveform(int carrier_en, Pwm0Pulse_SelectDef pulse,  unsigned short pulse_num)
{
	return  ( carrier_en<<15 | pulse | (pulse_num & 0x3fff) );
}

/**
 * @brief     This fuction servers to set the pwm's dma address.
 * @param[in] pdat - variable of pointer to indicate the address.
 * @return	  none.
 */
static inline void pwm_set_dma_address(void * pdat)
{
	reg_dma_pwm_addr = (unsigned short)((unsigned int)pdat);
	reg_dma7_addrHi = 0x04;
	reg_dma_pwm_mode  &= ~FLD_DMA_WR_MEM;
}

/**
 * @brief     This fuction servers to start the pwm's IRQ sending.
 * @param[in] none.
 * @return	  none.
 */
static inline void pwm_start_dma_ir_sending(void)
{
	reg_dma_chn_en |= FLD_DMA_CHN_PWM;
	reg_dma_tx_rdy0 |= FLD_DMA_CHN_PWM;
}

/**
 * @brief     This fuction servers to stop the pwm's IRQ sending.
 * @param[in] none.
 * @return	  none.
 */
static inline void pwm_stop_dma_ir_sending(void)
{
//	reg_dma_tx_rdy0 &= ~FLD_DMA_PWM;

	reg_rst0 = FLD_RST0_PWM;
	sleep_us(20);  //1us <-> 4 byte
	reg_rst0 = 0;
}

#endif /* PWM_H_ */


/** \defgroup GP8  PWM Usage
 * 	This is the first Group
 * 	@{
 */

//-----------------------------------------------------------1-8
/*! \page pwm PWM Usage
This page is for ...
details.
*/


 /** @}*/ //end of GP8
