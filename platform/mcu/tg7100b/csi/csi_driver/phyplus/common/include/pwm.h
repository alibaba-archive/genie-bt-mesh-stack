/**************************************************************************************************

  Phyplus Microelectronics Limited. All rights reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

**************************************************************************************************/



/*******************************************************************************
* @file		pwm.h
* @brief	Contains all functions support for pwm driver
* @version	0.0
* @date		30. Oct. 2017
* @author	Ding
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __PWM__H__
#define __PWM__H__

#include "types.h"
#include "gpio.h"
#include "common.h"

#define    PWM_CH_BASE             	0x4000E004UL
#define    PWM_CTL0_ADDR(n)        	(PWM_CH_BASE + n*12)
#define    PWM_CTL1_ADDR(n)        	(PWM_CH_BASE + 4 + n*12)

#define    PWM_ENABLE_ALL          	do{\
        *(volatile unsigned int *) 0x4000e000 |= BIT(0);\
        *(volatile unsigned int *) 0x4000e000 |= BIT(4);\
    }while(0)
#define    PWM_DISABLE_ALL 			do{\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(0);\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(4);\
    }while(0)
#define    PWM_ENABLE_CH_012      	do{\
        *(volatile unsigned int *) 0x4000e000 |= BIT(8);\
        *(volatile unsigned int *) 0x4000e000 |= BIT(9);\
    }while(0)
#define    PWM_DISABLE_CH_012       do{\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(8);\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(9);\
    }while(0)
#define    PWM_ENABLE_CH_345    	do{\
        *(volatile unsigned int *) 0x4000e000 |= BIT(10);\
        *(volatile unsigned int *) 0x4000e000 |= BIT(11);\
    }while(0)
#define    PWM_DISABLE_CH_345      	do{\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(10);\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(11);\
    }while(0)
#define    PWM_ENABLE_CH_01        	do{\
        *(volatile unsigned int *) 0x4000e000 |= BIT(12);\
        *(volatile unsigned int *) 0x4000e000 |= BIT(13);\
    }while(0)
#define    PWM_DISABLE_CH_01       	do{\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(12);\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(13);\
    }while(0)
#define    PWM_ENABLE_CH_23        	do{\
        *(volatile unsigned int *) 0x4000e000 |= BIT(14);\
        *(volatile unsigned int *) 0x4000e000 |= BIT(15);\
    }while(0)
#define    PWM_DISABLE_CH_23       	do{\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(14);\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(15);\
    }while(0)
#define    PWM_ENABLE_CH_45        	do{\
        *(volatile unsigned int *) 0x4000e000 |= BIT(16);\
        *(volatile unsigned int *) 0x4000e000 |= BIT(17);\
    }while(0)
#define    PWM_DISABLE_CH_45    	do{\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(16);\
        *(volatile unsigned int *) 0x4000e000 &= ~BIT(17);\
    }while(0)


#define    PWM_INSTANT_LOAD_CH(n)  	 subWriteReg(PWM_CTL0_ADDR(n),31,31,1)
#define    PWM_NO_INSTANT_LOAD_CH(n) subWriteReg(PWM_CTL0_ADDR(n),31,31,0)
#define    PWM_LOAD_CH(n) 			     subWriteReg(PWM_CTL0_ADDR(n),16,16,1)
#define    PWM_NO_LOAD_CH(n) 		     subWriteReg(PWM_CTL0_ADDR(n),16,16,0)
#define    PWM_SET_DIV(n,v) 		     subWriteReg(PWM_CTL0_ADDR(n),14,12,v)
#define    PWM_SET_MODE(n,v) 	   	   subWriteReg(PWM_CTL0_ADDR(n),8,8,v)
#define    PWM_SET_POL(n,v) 		     subWriteReg(PWM_CTL0_ADDR(n),4,4,v)
#define    PWM_ENABLE_CH(n)        	 subWriteReg(PWM_CTL0_ADDR(n),0,0,1)
#define    PWM_DISABLE_CH(n)       	 subWriteReg(PWM_CTL0_ADDR(n),0,0,0)

#define    PWM_SET_CMP_VAL(n,v) 	   subWriteReg(PWM_CTL1_ADDR(n),31,16,v)
#define    PWM_SET_TOP_VAL(n,v) 	   subWriteReg(PWM_CTL1_ADDR(n),15,0,v)
#define    PWM_GET_CMP_VAL(n) 	   	 ((read_reg(PWM_CTL1_ADDR(n)) & 0xFFFF0000) >> 8)
#define    PWM_GET_TOP_VAL(n) 	   	 read_reg(PWM_CTL1_ADDR(n)) & 0x0000FFFF


/*************************************************************
*	@brief		enum variable, the number of PWM channels supported
*
*/
typedef enum {

    PWM_CH0 = 0,
    PWM_CH1 = 1,
    PWM_CH2 = 2,
    PWM_CH3 = 3,
    PWM_CH4 = 4,
    PWM_CH5 = 5

} PWMN_e;

/*************************************************************
*	@brief		enum variable used for PWM clock prescaler
*
*/
typedef enum {

    PWM_CLK_NO_DIV = 0,
    PWM_CLK_DIV_2 = 1,
    PWM_CLK_DIV_4 = 2,
    PWM_CLK_DIV_8 = 3,
    PWM_CLK_DIV_16 = 4,
    PWM_CLK_DIV_32 = 5,
    PWM_CLK_DIV_64 = 6,
    PWM_CLK_DIV_128 = 7

} PWM_CLK_DIV_e;

/*************************************************************
*	@brief		enum variable used for PWM work mode setting
*
*/
typedef enum {

    PWM_CNT_UP = 0,
    PWM_CNT_UP_AND_DOWN = 1

} PWM_CNT_MODE_e;

/*************************************************************
*	@brief		enum variable used for PWM output polarity setting
*
*/
typedef enum {

    PWM_POLARITY_RISING = 0,
    PWM_POLARITY_FALLING = 1

} PWM_POLARITY_e;


/******************************************************/
typedef void *pwm_handle_t;

pwm_handle_t csi_pwm_initialize(uint32_t idx);

void csi_pwm_uninitialize(pwm_handle_t handle);

int32_t csi_pwm_config(pwm_handle_t handle,
                       uint8_t channel,
                       uint32_t period_us,
                       uint32_t pulse_width_us);

void csi_pwm_start(pwm_handle_t handle, uint8_t channel);

void csi_pwm_stop(pwm_handle_t handle, uint8_t channel);
#endif
