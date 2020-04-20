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
* @file   adc.c
* @brief  Contains all functions support for adc driver
* @version  0.0
* @date   18. Oct. 2017
* @author qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include "error.h"
#include "ap_cp.h"
#include "common.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "clock.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>

extern void udelay(uint32_t us);

#define ADC_FIFO_LENGTH 64
//static uint8_t mAdc_init_flg = 0;
static adc_Ctx_t mAdc_Ctx;
uint8_t g_channel;
static uint8_t  adc_cal_read_flag = 0;
static uint16_t adc_cal_postive = 0x0fff;
static uint16_t adc_cal_negtive = 0x0fff;

const static GPIO_Pin_e s_pinmap[ADC_CH_NUM] = {
    GPIO_DUMMY, //ADC_CH0 =0,
    GPIO_DUMMY, //ADC_CH1 =1,
    P12, //ADC_CH1P =2,  ADC_CH1DIFF = 2,
    P11, //ADC_CH1N =3,
    P14, //ADC_CH2P =4,  ADC_CH2DIFF = 4,
    P13, //ADC_CH2N =5,
    P20, //ADC_CH3P =6,  ADC_CH3DIFF = 6,
    P15, //ADC_CH3N =7,
    GPIO_DUMMY,  //ADC_CH_VOICE =8,
};

static void phy_set_sampling_resolution(adc_CH_t channel, bool is_high_resolution, bool is_differential_mode)
{
    if (is_high_resolution) {
        if (is_differential_mode) {
            BM_CLR(REG_IO_CONTROL, BIT(7 + ((channel >> 1) << 1) - (channel & 0x01)));
            BM_SET(REG_IO_CONTROL, BIT(((channel >> 1) << 1) - (channel & 0x01) - 1));
            BM_CLR(REG_IO_CONTROL, BIT(7 + (((channel + 1) >> 1) << 1) - ((channel + 1) & 0x01)));
            BM_SET(REG_IO_CONTROL, BIT((((channel + 1) >> 1) << 1) - ((channel + 1) & 0x01) - 1));
        } else {
            BM_CLR(REG_IO_CONTROL, BIT(7 + ((channel >> 1) << 1) - (channel & 0x01)));
            BM_SET(REG_IO_CONTROL, BIT(((channel >> 1) << 1) - (channel & 0x01) - 1));
        }

    } else {
        if (is_differential_mode) {
            BM_CLR(REG_IO_CONTROL, BIT(((channel >> 1) << 1) - (channel & 0x01) - 1));
            BM_SET(REG_IO_CONTROL, BIT(7 + ((channel >> 1) << 1) - (channel & 0x01)));
            BM_CLR(REG_IO_CONTROL, BIT((((channel + 1) >> 1) << 1) - ((channel + 1) & 0x01) - 1));
            BM_SET(REG_IO_CONTROL, BIT(7 + (((channel + 1) >> 1) << 1) - ((channel + 1) & 0x01)));
        } else {
            BM_CLR(REG_IO_CONTROL, BIT(((channel >> 1) << 1) - (channel & 0x01) - 1));
            BM_SET(REG_IO_CONTROL, BIT(7 + ((channel >> 1) << 1) - (channel & 0x01)));
        }
    }
}

static void phy_set_sampling_resolution_auto(uint8_t channel, bool is_high_resolution, bool is_differential_mode)
{
    uint8_t i_channel;
    adc_CH_t a_channel;

    for (i_channel = 0; i_channel < (ADC_CH_NUM - 1); i_channel++) {
        if (channel & BIT(i_channel)) {
            a_channel = (adc_CH_t)i_channel;
            phy_set_sampling_resolution(a_channel, is_high_resolution, is_differential_mode);
        }
    }
}

static void phy_set_differential_mode(void)
{
    subWriteReg(0x4000f048, 8, 8, 0);
    subWriteReg(0x4000f048, 11, 11, 0);
}

static void phy_set_single_mode(adc_CH_t channel)
{
    if (channel & 1) {
        subWriteReg(0x4000f048, 11, 11, 1);
    } else {
        subWriteReg(0x4000f048, 8, 8, 1);
    }
}

static void phy_set_channel(adc_CH_t channel)
{
    int index = (int)channel;

    if (index == 1) {
        subWriteReg(0x4000f048, 7, 5, 1);
        POWER_UP_TEMPSENSOR;
        return;
    }

    GPIO_Pin_e pin = s_pinmap[index];

    if (pin == GPIO_DUMMY) {
        return;
    }

    subWriteReg(0x4000f048, 7, 5, ((channel >> 1) + 1));
}


static void phy_set_analog_pin(adc_CH_t channel)
{
    uint8_t index = (int)channel;
    GPIO_Pin_e pin = s_pinmap[index];

    if (pin == GPIO_DUMMY) {
        return;
    }

    phy_gpio_cfg_analog_io(pin, Bit_ENABLE);
}


static void phy_disable_analog_pin(adc_CH_t channel)
{
    int index = (int)channel;
    GPIO_Pin_e pin = s_pinmap[index];

    if (pin == GPIO_DUMMY) {
        return;
    }

    phy_gpio_cfg_analog_io(pin, Bit_DISABLE);
    phy_gpio_pin_init(pin, IE);      //ie=0,oen=1 set to imput
    phy_gpio_pull_set(pin, FLOATING);   //set pin pull up/down floating
}

static void phy_clear_adcc_cfg(void)
{
    memset(&mAdc_Ctx, 0, sizeof(mAdc_Ctx));
}


/////////////// adc ////////////////////////////
/**************************************************************************************
 * @fn          hal_ADC_IRQHandler
 *
 * @brief       This function process for adc interrupt
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
//uint32_t debug;

void __attribute__((used)) phy_ADC_IRQHandler(void)
{
    uint32_t status;
    status = GET_IRQ_STATUS;

    if (status == mAdc_Ctx.all_channel) {
        ;
    }
}
/*
static void adc_wakeup_hdl(void)
{
    NVIC_SetPriority((IRQn_Type)ADCC_IRQ, IRQ_PRIO_HAL);
}*/

/**************************************************************************************
 * @fn          hal_adc_init
 *
 * @brief       This function process for adc initial
 *
 * input parameters
 *
 * @param       ADC_MODE_e mode: adc sample mode select;1:SAM_MANNUAL(mannual mode),0:SAM_AUTO(auto mode)
 *              ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *              ADC_SEMODE_e semode: signle-ended mode negative side enable; 1:SINGLE_END(single-ended mode) 0:DIFF(Differentail mode)
 *              IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void phy_adc_init(void)
{
    phy_clear_adcc_cfg();
}

int phy_adc_config_channel(adc_CH_t channel, adc_Cfg_t cfg, adc_event_cb_t evt_handler)
{
    if (mAdc_Ctx.enable) {
        return PPlus_ERR_BUSY;
    }

    if (channel == ADC_CH_VOICE || channel == ADC_CH0 /*||channel == ADC_CH1*/) {
        return PPlus_ERR_NOT_SUPPORTED;
    }

    if ((channel != 1) && (cfg.is_differential_mode && (channel & 1))) {
        return PPlus_ERR_INVALID_PARAM;
    }

    mAdc_Ctx.is_differential_mode = cfg.is_differential_mode;

    mAdc_Ctx.evt_handler[channel] = evt_handler;

    clk_gate_enable(MOD_ADCC);//enable ADCC clk gated

    //CLK_1P28M_ENABLE;
    AP_PCRM->CLKSEL |= BIT(6);

    //ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
    AP_PCRM->CLKHF_CTL0 |= BIT(18);

    //ENABLE_DLL;                  //enable DLL
    AP_PCRM->CLKHF_CTL1 |= BIT(7);

    //ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
    AP_PCRM->CLKHF_CTL1 &= ~BIT(21);

    //ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
    AP_PCRM->CLKHF_CTL1 |= BIT(13);

    //subWriteReg(0x4000f07c,4,4,1);    //set adc mode,1:mannual,0:auto mode
    AP_PCRM->ADC_CTL4 |= BIT(4);

    phy_set_sampling_resolution(channel, cfg.is_high_resolution, cfg.is_differential_mode);

    if (cfg.is_differential_mode) {
        phy_set_channel(channel);
        phy_set_differential_mode();
        phy_set_analog_pin(channel);
        phy_set_analog_pin((adc_CH_t)(channel + 1));
    } else {
        phy_set_channel(channel);
        phy_set_single_mode(channel);
        phy_set_analog_pin(channel);
    }

    return PPlus_SUCCESS;
}

int phy_adc_clock_config(adc_CLOCK_SEL_t clk)
{
    subWriteReg(0x4000F000 + 0x7c, 2, 1, clk);
    return PPlus_SUCCESS;
}

int phy_adc_start(void)
{
    mAdc_Ctx.enable = TRUE;
    //hal_pwrmgr_lock(MOD_ADCC);
    //ENABLE_ADC;
    AP_PCRM->ANA_CTL |= BIT(3);

    //ADC_IRQ_ENABLE;
    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQ);
    //ENABLE_ADC_INT;
    AP_ADCC->intr_mask = 0x1ff;

    //disableSleep();
    return PPlus_SUCCESS;
}


int phy_adc_start_int_dis(void)
{
    mAdc_Ctx.enable = TRUE;
    AP_PCRM->ANA_CTL |= BIT(3);

    return PPlus_SUCCESS;

}

int phy_adc_config_channel_auto(uint8_t channel, adc_Cfg_t cfg, adc_event_cb_t evt_handler)
{
    uint8_t i;

    mAdc_Ctx.is_differential_mode = cfg.is_differential_mode;

    mAdc_Ctx.is_continue_mode = cfg.is_continue_mode;

    mAdc_Ctx.all_channel = channel;

    clk_gate_enable(MOD_ADCC);//enable ADCC clk gated

    //CLK_1P28M_ENABLE;
    AP_PCRM->CLKSEL |= BIT(6);

    //ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
    AP_PCRM->CLKHF_CTL0 |= BIT(18);

    //ENABLE_DLL;                  //enable DLL
    AP_PCRM->CLKHF_CTL1 |= BIT(7);

    //ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
    AP_PCRM->CLKHF_CTL1 &= ~BIT(21);

    //ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
    AP_PCRM->CLKHF_CTL1 |= BIT(13);

    //subWriteReg(0x4000f07c,4,4,1);    //set adc mode,1:mannual,0:auto mode
    AP_PCRM->ADC_CTL4 |= BIT(4);
    AP_PCRM->ADC_CTL4 |= BIT(0);

    phy_set_sampling_resolution_auto(channel, cfg.is_high_resolution, cfg.is_differential_mode);

    if (cfg.is_auto_mode) {
        AP_PCRM->ADC_CTL4 &= ~BIT(4); //enable auto mode

        for (i = 0; i < 8; i++) {
            if (channel & BIT(i)) {
                GPIO_Pin_e pin = s_pinmap[i];
                phy_pad_ds_control(pin, Bit_ENABLE);
                phy_gpio_cfg_analog_io(pin, Bit_ENABLE);

                switch (i) {
                    case 0:
                    case 1:
                        AP_PCRM->ADC_CTL0 |= BIT(4 + (i << 4));
                        mAdc_Ctx.evt_handler[i] = evt_handler;
                        break;

                    case 2:
                    case 3:
                        AP_PCRM->ADC_CTL1 |= BIT(4 + ((i - 2) << 4));
                        mAdc_Ctx.evt_handler[i] = evt_handler;
                        break;

                    case 4:
                    case 5:
                        AP_PCRM->ADC_CTL2 |= BIT(4 + ((i - 4) << 4));
                        mAdc_Ctx.evt_handler[i] = evt_handler;
                        break;

                    case 6:
                    case 7:
                        AP_PCRM->ADC_CTL3 |= BIT(4 + ((i - 6) << 4));
                        mAdc_Ctx.evt_handler[i] = evt_handler;
                        break;

                    default:
                        break;
                }
            }
        }
    }

    return PPlus_SUCCESS;
}


int phy_adc_stop(void)
{
    int i;

    //MASK_ADC_INT;
    AP_ADCC->intr_mask = 0x1ff;

    NVIC_DisableIRQ((IRQn_Type)ADCC_IRQ);

    //DISABLE_ADC;
    AP_PCRM->ANA_CTL &= ~BIT(3);

    //ADC_CLOCK_DISABLE;
    AP_PCRM->CLKHF_CTL1 &= ~BIT(13);

    for (i = 0; i < ADC_CH_NUM; i++) {
        if (mAdc_Ctx.evt_handler[i]) {
            phy_disable_analog_pin((adc_CH_t)i);
        }
    }

    clk_gate_disable(MOD_ADCC);//disable I2C clk gated
    phy_clear_adcc_cfg();
    //enableSleep();
//    hal_pwrmgr_unlock(MOD_ADCC);
    return PPlus_SUCCESS;
}



/**************************************************************************************
 * @fn          hal_adc_value
 *
 * @brief       This function process for get adc value
 *
 * input parameters
 *
 * @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      ADC value
 **************************************************************************************/
float phy_adc_value(uint16_t *buf, uint8_t size, bool high_resol, bool diff_mode)
{
    uint8_t i;
    int adc_sum = 0;
    float result = 0.0;

    for (i = 0; i < size; i++) {
        int value;

        if (diff_mode == TRUE) {
            value = (buf[i] & 0x0800) ? 0 - (int)(buf[i] & 0x7ff) : (int)(buf[i] & 0x7ff);
        } else {
            value = (int)(buf[i] & 0xfff);
        }

        adc_sum += value;
    }

    result = ((float)adc_sum) / size;

    result = (diff_mode) ? (result / 2048 - 1) : (result / 4096);

    if (high_resol == FALSE) {
        result = result * 4;
    }

    return result;
}

static void phy_adc_load_calibration_value(void)
{
    if (adc_cal_read_flag == FALSE) {
        adc_cal_read_flag = TRUE;
        adc_cal_negtive = read_reg(0x11001000) & 0x0fff;
        adc_cal_postive = (read_reg(0x11001000) >> 16) & 0x0fff;
    }

}

int phy_adc_value_cal(adc_CH_t ch, uint32_t *buf, uint32_t size, bool high_resol, bool diff_mode)
{
    uint32_t i;
    int adc_sum = 0;
    float result = 0.0;

    for (i = 0; i < size; i++) {
        adc_sum += (buf[i] & 0xfff);
    }

    phy_adc_load_calibration_value();

    result = (1000 * adc_sum) / size;

    if ((adc_cal_postive != 0xfff) && (adc_cal_negtive != 0xfff)) {
        float delta = ((int)(adc_cal_postive - adc_cal_negtive)) / 2.0;

        if (ch & 0x01) {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / (adc_cal_postive + adc_cal_negtive))
                     : ((result + delta) / (adc_cal_postive + adc_cal_negtive));
        } else {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / (adc_cal_postive + adc_cal_negtive))
                     : ((result - delta) / (adc_cal_postive + adc_cal_negtive));
        }

    } else {
        result = (diff_mode) ? (result / 2048 - 1) : (result / 4096);
    }

    if (high_resol == FALSE) {
        result = result * 4;
    }

    return result;
}


/*
------------------------------------------
*/


/*
*/
#include <csi_config.h>
#include <soc.h>
//#include <csi_core.h>
#include <drv_adc.h>
//#include <ck_adc.h>
#include <drv_gpio.h>
#include <io.h>
#include <string.h>
#include <drv_dmac.h>


#define ERR_ADC(errno) (CSI_DRV_ERRNO_ADC_BASE | errno)
#define ADC_NULL_PARAM_CHK(para) HANDLE_PARAM_CHK(para, ERR_ADC(DRV_ERROR_PARAMETER))

typedef struct {
    uint32_t base;

    adc_event_cb_t evt_handler;

    uint32_t  intrp_mode;
    uint8_t  channel;

    adc_Cfg_t cfg;
    uint32_t sampling_frequency;
    adc_mode_e mode;
} ck_adc_priv_t;

static ck_adc_priv_t adc_instance[CONFIG_ADC_NUM];
static const adc_capabilities_t adc_capabilities = {
    .single = 0,
    .continuous = 1,
    .scan = 1,
    .calibration = 0,
    .comparator = 0
};

adc_handle_t drv_adc_initialize(int32_t idx, adc_event_cb_t cb_event)
{
    if (idx != 0) {
        return NULL;
    }

    ck_adc_priv_t *adc_priv = &adc_instance[idx];

    adc_priv->evt_handler = cb_event;

    clk_gate_enable(MOD_ADCC);

    phy_adc_init();

    return (adc_handle_t)adc_priv;
}

/**
  \brief       De-initialize adc Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle   adc handle to operate.
  \return      error code
*/
int32_t drv_adc_uninitialize(adc_handle_t handle)
{
    ADC_NULL_PARAM_CHK(handle);
    //hal_pwrmgr_register(MOD_ADCC,NULL,NULL);
    phy_clear_adcc_cfg();
    //hal_adc_init();

    clk_gate_disable(MOD_ADCC);

    return 0;
}

/**
  \brief       Get driver capabilities.
  \param[in]   idx    adc index.
  \return      \ref adc_capabilities_t
*/
adc_capabilities_t drv_adc_get_capabilities(int32_t idx)
{
    if (idx > (CONFIG_ADC_NUM - 1) || idx < 0) {
        adc_capabilities_t ret;
        memset(&ret, 0, sizeof(adc_capabilities_t));
        return ret;
    }

    return adc_capabilities;
}


/**
  \brief       config adc mode.
  \param[in]   handle     adc handle to operate.
  \param[in]   config   adc_conf_t\ref  . pointer to adc configuration structure.
  \return      error code
*/
extern  uint8_t g_channel;
void phy_pad_ds_control(GPIO_Pin_e pin, BitAction_e value)
{
    uint32_t  bit = BIT(pin & 0x1f);

    if (value) {
        BM_SET(REG_PAD_DS(pin), bit);       //set bit
    } else {
        BM_CLR(REG_PAD_DS(pin), bit);       //clear bit
    }
}

static uint32_t set_channel_bit(uint32_t *channel_array, uint32_t channel_num)
{
    int i = 0;
    uint32_t channel_mask = 0;

    while (i < channel_num) {
        if (*channel_array > 7 || *channel_array < 2) {
            return ADC_CHANNEL_ERROR;
        }

        channel_mask |= BIT(*channel_array);
        channel_array++;
        i++;
    }

    return channel_mask;
}

int32_t drv_adc_config(adc_handle_t handle, adc_conf_t *config)
{
    ADC_NULL_PARAM_CHK(handle);
    ADC_NULL_PARAM_CHK(config);
    ADC_NULL_PARAM_CHK(config->channel_array);
    ADC_NULL_PARAM_CHK(config->channel_nbr);
    ADC_NULL_PARAM_CHK(config->conv_cnt);

    ck_adc_priv_t *adc_priv = handle;
    uint32_t *ch_arry = config->channel_array;

    uint32_t ch_num = config->channel_nbr;

    adc_priv->cfg.is_auto_mode = TRUE;
    adc_priv->cfg.is_differential_mode = FALSE;
    adc_priv->cfg.is_high_resolution = FALSE;
    adc_priv->sampling_frequency = config->sampling_time;
    adc_priv->mode = config->mode;
    adc_priv->intrp_mode = config->intrp_mode;

    if (config->mode == ADC_SINGLE) {
        return ADC_PARAM_INVALID;
    } else if (config->mode == ADC_CONTINUOUS) {
        if (ch_num > 1) {
            return ADC_PARAM_INVALID;
        }

        adc_priv->cfg.is_continue_mode = TRUE;
    } else if (config->mode == ADC_SCAN) {
        adc_priv->cfg.is_continue_mode = TRUE;
    }

    adc_CLOCK_SEL_t clk;

    if (adc_priv->sampling_frequency == 80000) {
        clk = HAL_ADC_CLOCK_80K;
    } else if (adc_priv->sampling_frequency == 160000) {
        clk = HAL_ADC_CLOCK_160K;
    } else if (adc_priv->sampling_frequency == 320000) {
        clk = HAL_ADC_CLOCK_320K;
    } else {
        adc_priv->sampling_frequency = 320000;
        clk = HAL_ADC_CLOCK_320K;
    }

    phy_adc_clock_config(clk);

    if (ch_num > 6 || ch_num == 0) {
        return -ADC_PARAM_INVALID;
    }

    //g_channel = adc_priv->channel;
    adc_priv->channel = set_channel_bit(ch_arry, ch_num);

    int ret = -1;

    if (!config->enable_link_internal_voltage) {
        ret = phy_adc_config_channel_auto(adc_priv->channel, adc_priv->cfg, adc_priv->evt_handler);
        if (ret != 0) {
            printf("%s, %d error, %d\n", __func__, __LINE__, ret);
            return -1;
        }
    } else {
        if(((1 << config->link_internal_voltage_channel) & adc_priv->channel) == 0) {
            return ADC_PARAM_INVALID;
        }

        GPIO_Pin_e pin;
        pin = s_pinmap[config->link_internal_voltage_channel];

        phy_gpio_cfg_analog_io(pin,Bit_DISABLE);
        phy_gpio_write(pin, 1);

        ret = phy_adc_config_channel_auto(adc_priv->channel, adc_priv->cfg, adc_priv->evt_handler);
        if (ret != 0) {
            printf("%s, %d error\n", __func__, __LINE__);
            return -1;
        }

        phy_gpio_cfg_analog_io(pin,Bit_DISABLE);
    }

    return 0;
}


/**
  \brief       start adc.
  \param[in]   handle adc handle to operate.
  \return      error code
*/
int32_t drv_adc_start(adc_handle_t handle)
{
    ADC_NULL_PARAM_CHK(handle);
    ck_adc_priv_t *adc_priv = handle;

    if (adc_priv->intrp_mode == 1) {
        phy_adc_start();
    } else {
        phy_adc_start_int_dis();
    }

    return 0;
}

/**
  \brief       stop adc.
  \param[in]   handle adc handle to operate.
  \return      error code
*/
int32_t drv_adc_stop(adc_handle_t handle)
{
    ADC_NULL_PARAM_CHK(handle);
    phy_adc_stop();
    return 0;
}

static void wait_read_end()
{
    ;
}

#define S_US 1000000 //us
static uint32_t wait_data_ready(uint32_t sampling_frequency, int data_num, int ch_num)
{
    uint32_t us = (S_US * data_num * ch_num + sampling_frequency - 1) / sampling_frequency;
    udelay(us + 10);
    return 0;
}

static int read_data_one_channel_n(adc_handle_t handle, uint32_t *data, uint32_t num)
{
    ck_adc_priv_t *adc_priv = handle;
    uint8_t channel_mask = adc_priv->channel;
    int ch = 2;

    while (ch < 8) {
        if (channel_mask & BIT(ch)) {
            break;
        }

        ch++;
    }

    int i = 0;

    while (i < num) {
        if ((i % ADC_FIFO_LENGTH) == 0) {
            wait_data_ready(adc_priv->sampling_frequency, ADC_FIFO_LENGTH, 1);
        }

        data[i] = (uint16_t)(read_reg(ADC_CH_BASE + (ch * 0x80) + ((ch + 2) * 4)) & 0xfff);

        i++;

        if (i >= num) {
            break;
        }

        data[i] = (uint16_t)((read_reg(ADC_CH_BASE + (ch * 0x80) + ((ch + 2) * 4)) >> 16) & 0xfff);
        i++;
    }

    return 0;
}

static int read_multiple_channel_once(uint32_t channel_mask, uint32_t *data, uint32_t *buf_len)
{
    int i = 0, j = 0;

    while (i < 8) {
        if (channel_mask & BIT(i)) {
            if (j % 2) {
                data[j] = (uint16_t)(read_reg(ADC_CH_BASE + (i * 0x80) + ((i + 2) * 4)) & 0xfff);
            } else {
                data[j] = (uint16_t)((read_reg(ADC_CH_BASE + (i * 0x80) + ((i + 2) * 4)) >> 16) & 0xfff);
            }

            j++;
            *buf_len -= 1;

            if (*buf_len == 0) {
                break;
            }
        }

        i++;
    }

    return 0;
}

static int read_multiple_channel_fifo_length(adc_handle_t handle,
        uint32_t *data,
        uint32_t *read_len,
        uint32_t ch_num)
{
    int i = 0;
    ck_adc_priv_t *adc_priv = handle;
    uint32_t channel_mask = adc_priv->channel;


    while (i < ADC_FIFO_LENGTH && *read_len) {
        read_multiple_channel_once(channel_mask, data, read_len);
        data += ch_num;
        i++;
    }

    return 0;
}

static int read_multiple_channel_n(adc_handle_t handle, uint32_t *data, uint32_t read_len)
{
    int i = 0;
    ck_adc_priv_t *adc_priv = handle;
    uint32_t channel_mask = adc_priv->channel;
    int ch_num = 0;

    while (i < 8) {
        if (channel_mask & BIT(i)) {
            ch_num++;
        }

        i++;
    }

    i = 0;

    while (read_len > 0) {
        if (i % ADC_FIFO_LENGTH == 0) {
            wait_data_ready(adc_priv->sampling_frequency, ADC_FIFO_LENGTH, ch_num);
        }

        read_multiple_channel_fifo_length(handle, data, &read_len, ch_num);
        i++;
    }

    return 0;
}

int32_t drv_adc_read(adc_handle_t handle, uint32_t *data, uint32_t num)
{
    ADC_NULL_PARAM_CHK(handle);
    ADC_NULL_PARAM_CHK(data);

    ck_adc_priv_t *adc_priv = handle;

    if (adc_priv->intrp_mode == TRUE) {
        wait_read_end();
        return 0;
    }

    if (adc_priv->mode == ADC_SINGLE) {
        read_data_one_channel_n(handle, data, num);
    } else if (adc_priv->mode == ADC_CONTINUOUS) {
        read_data_one_channel_n(handle, data, num);
    } else if (adc_priv->mode == ADC_SCAN) {
        read_multiple_channel_n(handle, data, num);
    }

    /*
    copy data from adc_data to data
    */
    return 0;
}

/**
  \brief       Get ADC status.
  \param[in]   handle   adc handle to operate.
  \return      ADC status \ref adc_status_t
*/
adc_status_t drv_adc_get_status(adc_handle_t handle)
{
    adc_status_t adc_status = {0};
    return adc_status;
}

int32_t drv_adc_comparator_config(adc_handle_t handle, adc_cmp_conf_t *config)
{
    return ERR_ADC(DRV_ERROR_UNSUPPORTED);
}

int32_t drv_adc_power_control(adc_handle_t handle, csi_power_stat_e state)
{
    ADC_NULL_PARAM_CHK(handle);

    if (state == DRV_POWER_FULL) {
        clk_gate_enable(MOD_ADCC);
        return 0;
    } else if (state == DRV_POWER_OFF) {
        clk_gate_disable(MOD_ADCC);
        return 0;
    } else {
        return ERR_ADC(DRV_ERROR_UNSUPPORTED);
    }
}
