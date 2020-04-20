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
* @file		adc.h
* @brief	Contains all functions support for adc driver
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __ADC__H__
#define __ADC__H__

#include "types.h"
#include "common.h"
#include "gpio.h"
#include "drv_adc.h"

#define ADC_BUF_CAPABILITY  16

#define ADC_PARAM_INVALID           1
#define ADC_USER_BUFFER_LACK        2
#define ADC_MODE_ERROR              3
#define ADC_CHANNEL_ERROR           4
#define ADC_CONVERSION_INFO_ERROR   5
#define ADC_DATA_LOST               6
#define ADC_DATA_OVERFLOW           7

#define ADC_DATA_COMPARE_RIGHT_INTERRUPT_BIT     2
#define ADC_DATA_OVERWRITE_INTERRUPT_BIT         1
#define ADC_CONVERSION_COMLETED_INTERRUPT_BIT    0

#define CK_ADC_SINGLE_MODE  0
#define CK_ADC_SCAN_MODE    1
#define CK_ADC_CONTINUOUS   2

#define CK_ADC_DATA_WIDTH   12

#define CK_ADC_CONTINOUS_MODE_MAX_CONVERT_TIMES    16
#define CK_ADC_SCAN_MODE_MAX_CONVERT_TIMES    1

#define    MAX_ADC_SAMPLE_SIZE     32
#define    MAX_ADC_SAMPLE_ID       (MAX_ADC_SAMPLE_SIZE-1)

//#define    BELOW_1V                 0
//#define    UP_1V                    1
//#define    ADC_INPUT_LEVEL          BELOW_1V
//#define    ADC_INPUT_LEVEL          UP_1V


#define    ADC_CH_BASE             (0x40050400UL)
#define    ADC_SAMPLE_AUTO              0
#define    ADC_SAMPLE_MANNUAL           1
#define    ADC_SAMPLE_MODE              SAMPLE_MANNUAL
//#define    ADC_SAMPLE_MODE              SAMPLE_AUTO

#define    ENABLE_ADC_INT      *(volatile unsigned int *)0x40050034 |= 0x000001ff
#define    MASK_ADC_INT        *(volatile unsigned int *)0x40050034 &= 0xfffffe00
#define    ADC_IRQ_ENABLE      *(volatile unsigned int *) 0xe000e100 = BIT(29)
#define    ADC_IRQ_DISABLE     *(volatile unsigned int *) 0xe000e100 = BIT(29)
#define    CLEAR_ADC_INT_CH0   *(volatile unsigned int *)0x40050038 |= BIT(0)
#define    CLEAR_ADC_INT_CH1   *(volatile unsigned int *)0x40050038 |= BIT(1)
#define    CLEAR_ADC_INT_CH2   *(volatile unsigned int *)0x40050038 |= BIT(2)
#define    CLEAR_ADC_INT_CH3   *(volatile unsigned int *)0x40050038 |= BIT(3)
#define    CLEAR_ADC_INT_CH4   *(volatile unsigned int *)0x40050038 |= BIT(4)
#define    CLEAR_ADC_INT_CH5   *(volatile unsigned int *)0x40050038 |= BIT(5)
#define    CLEAR_ADC_INT_CH6   *(volatile unsigned int *)0x40050038 |= BIT(6)
#define    CLEAR_ADC_INT_CH7   *(volatile unsigned int *)0x40050038 |= BIT(7)
#define    CLEAR_ADC_INT_VOICE *(volatile unsigned int *)0x40050038 |= BIT(8)
#define    CLEAR_ADC_INT(n)    *(volatile unsigned int *)0x40050038 |= BIT(n)

#define    IS_CLAER_ADC_INT_CH0 (*(volatile unsigned int *)0x4005003c) & BIT(0)
#define    IS_CLAER_ADC_INT_CH1 (*(volatile unsigned int *)0x4005003c) & BIT(1)
#define    IS_CLAER_ADC_INT_CH2 (*(volatile unsigned int *)0x4005003c) & BIT(2)
#define    IS_CLAER_ADC_INT_CH3 (*(volatile unsigned int *)0x4005003c) & BIT(3)
#define    IS_CLAER_ADC_INT_CH4 (*(volatile unsigned int *)0x4005003c) & BIT(4)
#define    IS_CLAER_ADC_INT_CH5 (*(volatile unsigned int *)0x4005003c) & BIT(5)
#define    IS_CLAER_ADC_INT_CH6 (*(volatile unsigned int *)0x4005003c) & BIT(6)
#define    IS_CLAER_ADC_INT_CH7 (*(volatile unsigned int *)0x4005003c) & BIT(7)
#define    IS_CLAER_ADC_INT_VOICE (*(volatile unsigned int *)0x4005003c) & BIT(8)
#define    IS_CLAER_ADC_INT(n)   (*(volatile unsigned int *)0x4005003c) & BIT(n)

#define    GET_IRQ_STATUS         ((*(volatile unsigned int *) 0x4005003c) & 0x3ff)
#define    ENABLE_ADC             (*(volatile unsigned int *)0x4000f048 |= BIT(3))
#define    DISABLE_ADC            (*(volatile unsigned int *)0x4000f048 &= ~BIT(3))
#define    ADC_CLOCK_ENABLE       (*(volatile unsigned int *)0x4000f044 |= BIT(13))
#define    ADC_CLOCK_DISABLE       (*(volatile unsigned int *)0x4000f044 &= ~BIT(13))
#define    ADC_DBLE_CLOCK_DISABLE (*(volatile unsigned int *)0x4000f044 &= ~BIT(21))
#define    POWER_DOWN_ADC         (*(volatile unsigned int *)0x4000f048 &= ~BIT(3))
#define    POWER_UP_TEMPSENSOR    (*(volatile unsigned int *)0x4000f048 |= BIT(29))
#define    REG_IO_CONTROL         ((volatile unsigned int  *)0x4000f020)



#define ADCC_REG_BASE   (0x4000F000)



/**************************************************************************************
 * @fn          hal_get_adc_int_source
 *
 * @brief       This function process for get adc interrupt source,such as adc channel NO
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      adc interrupt source bit loaction(uint8_t)
 **************************************************************************************/




typedef enum {
    ADC_CH0 = 0,
    ADC_CH1 = 1,
    ADC_CH1P = 2,  ADC_CH1DIFF = 2, ADC_CH1P_P12 = 2,
    ADC_CH1N = 3,  ADC_CH1N_P11 = 3,
    ADC_CH2P = 4,  ADC_CH2DIFF = 4, ADC_CH2P_P14 = 4,
    ADC_CH2N = 5,  ADC_CH2N_P13 = 5,
    ADC_CH3P = 6,  ADC_CH3DIFF = 6, ADC_CH3P_P20 = 6,
    ADC_CH3N = 7,  ADC_CH3N_P15 = 7,
    ADC_CH_VOICE = 8,
    ADC_CH_NUM = 9,
} adc_CH_t;

#define     ADC_CH0_AUTO    BIT(0)
#define     ADC_CH1_AUTO    BIT(1)
#define     ADC_CH1P_AUTO   BIT(2)//ADC_CH1P_P12
#define     ADC_CH1N_AUTO   BIT(3)//ADC_CH1N_P11
#define     ADC_CH2P_AUTO   BIT(4)//ADC_CH2P_P14
#define     ADC_CH2N_AUTO   BIT(5)//ADC_CH2N_P13
#define     ADC_CH3P_AUTO   BIT(6)//ADC_CH3P_P20
#define     ADC_CH3N_AUTO   BIT(7)//ADC_CH3N_P15
//typedef enum {
//  ADC_CH0_AUTO = BIT(0),
//  ADC_CH1_AUTO = BIT(1),
//  ADC_CH1P_AUTO = BIT(2),  ADC_CH1DIFF_AUTO = BIT(2), ADC_CH1P_P12_AUTO = BIT(2),
//  ADC_CH1N_AUTO = BIT(3),  ADC_CH1N_P11_AUTO = BIT(3),
//  ADC_CH2P_AUTO = BIT(4),  ADC_CH2DIFF_AUTO = BIT(4), ADC_CH2P_P14_AUTO = BIT(4),
//  ADC_CH2N_AUTO = BIT(5),  ADC_CH2N_P13_AUTO = BIT(5),
//  ADC_CH3P_AUTO = BIT(6),  ADC_CH3DIFF_AUTO = BIT(6), ADC_CH3P_P20_AUTO = BIT(6),
//  ADC_CH3N_AUTO = BIT(7),  ADC_CH3N_P15_AUTO = BIT(7),
//}adc_CH_AUTO_t;


enum {
    HAL_ADC_EVT_DATA = 1,
    HAL_ADC_EVT_FAIL = 0xff
};


typedef enum {
    HAL_ADC_CLOCK_80K = 0,
    HAL_ADC_CLOCK_160K = 1,
    HAL_ADC_CLOCK_320K = 2,
} adc_CLOCK_SEL_t;

typedef struct _adc_Cfg_t {
    bool  is_continue_mode;
    bool  is_differential_mode;
    bool  is_high_resolution; //high resolution for adc sampling
    bool  is_auto_mode; //manual|auto mode
} adc_Cfg_t;


typedef struct _adc_Evt_t {
    int       type;
    adc_CH_t  ch;
    uint16_t *data;
    uint8_t   size; //word size
} adc_Evt_t;

typedef void (*adc_Hdl_t)(adc_Evt_t *pev);

typedef struct _adc_Contex_t {
    bool        enable;
    bool        is_continue_mode;
    bool        is_differential_mode;
    uint8_t     all_channel;
    adc_event_cb_t   evt_handler[ADC_CH_NUM];
} adc_Ctx_t;

/**************************************************************************************
 * @fn          hal_adc_init
 *
 * @brief       This function process for adc initial
 *
 * input parameters
 *
 * @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *              ADC_SEMODE_e semode: single-end mode and diff mode select; 1:SINGLE_END(single-end mode) 0:DIFF(Diff mode)
 *              IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void phy_adc_init(void);

int phy_adc_config_channel(adc_CH_t channel, adc_Cfg_t cfg, adc_event_cb_t evt_handler);

int phy_adc_config_channel_auto(uint8_t channel, adc_Cfg_t cfg, adc_event_cb_t evt_handler);

int phy_adc_clock_config(adc_CLOCK_SEL_t clk);

int phy_adc_start(void);

int phy_adc_stop(void);


void __attribute__((weak)) phy_ADC_IRQHandler(void);

int phy_adc_value_cal(adc_CH_t ch, uint32_t *buf, uint32_t size, bool high_resol, bool diff_mode);

extern void phy_pad_ds_control(GPIO_Pin_e pin, BitAction_e value);

uint8_t get_adc_channel(void);
#endif
