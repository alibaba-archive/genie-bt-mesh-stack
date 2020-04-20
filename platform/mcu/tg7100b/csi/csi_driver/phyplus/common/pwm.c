/*******************************************************************************
* @file		pwm.c
* @brief	Contains all functions support for pwm driver
* @version	0.0
* @date		30. Oct. 2017
* @author	Ding
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include "ap_cp.h"
#include "clock.h"
#include "pwm.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "common.h"
#include <string.h>
#include "drv_common.h"

#define ERR_PWM(errno) (CSI_DRV_ERRNO_PWM_BASE | errno)

#define PWM_NULL_PARAM_CHK(para)          HANDLE_PARAM_CHK(para, ERR_PWM(DRV_ERROR_PARAMETER))
#define PWM_NULL_PARAM_CHK_NORETVAL(para) HANDLE_PARAM_CHK_NORETVAL(para, ERR_PWM(DRV_ERROR_PARAMETER))

#define PWM_BASE_FREQ 8000000
#define S_US 1000000
#define MAX_CNT_TIME_US (S_US * 65535)
#define MAX_DIV 7   /*MAV div val 1 << 7 = 128*/


void phy_pwm_init(PWMN_e pwmN, PWM_CLK_DIV_e pwmDiv, PWM_CNT_MODE_e pwmMode, PWM_POLARITY_e pwmPolarity)
{
    clk_gate_enable(MOD_PWM);
    PWM_DISABLE_CH(pwmN);

    PWM_SET_DIV(pwmN, pwmDiv);
    PWM_SET_MODE(pwmN, pwmMode);
    PWM_SET_POL(pwmN, pwmPolarity);

    //hal_pwrmgr_register(MOD_PWM, NULL, NULL);
}

void phy_pwm_open_channel(PWMN_e pwmN)
{
    PWM_ENABLE_CH(pwmN);
}

void phy_pwm_close_channel(PWMN_e pwmN)
{

    PWM_DISABLE_CH(pwmN);
}

void phy_pwm_destroy(PWMN_e pwmN)
{
    PWM_DISABLE_CH(pwmN);
    PWM_NO_LOAD_CH(pwmN);

    PWM_NO_INSTANT_LOAD_CH(pwmN);

    PWM_SET_DIV(pwmN, 0);
    PWM_SET_MODE(pwmN, 0);
    PWM_SET_POL(pwmN, 0);

    PWM_SET_TOP_VAL(pwmN, 0);
    PWM_SET_CMP_VAL(pwmN, 0);
}

void phy_pwm_set_count_val(PWMN_e pwmN, uint16_t cmpVal, uint16_t cntTopVal)
{
    PWM_NO_LOAD_CH(pwmN);

    PWM_SET_CMP_VAL(pwmN, cmpVal);
    PWM_SET_TOP_VAL(pwmN, cntTopVal);

    PWM_LOAD_CH(pwmN);
}

static int pwm_en = 0;

void phy_pwm_start(void)
{
    //hal_pwrmgr_lock(MOD_PWM);
    if (pwm_en == 0) {
        PWM_ENABLE_ALL;
        pwm_en = 1;
    }
}

void phy_pwm_stop(void)
{
    //hal_pwrmgr_unlock(MOD_PWM);
    if (pwm_en == 1) {
        PWM_DISABLE_ALL;
        pwm_en = 0;
    }
}

/**************************************/
typedef struct {
    PWMN_e pwmN;
    PWM_CLK_DIV_e pwmDiv;
    PWM_CNT_MODE_e pwmMode;
    PWM_POLARITY_e pwmPolarity;
    uint16_t cmpVal;
    uint16_t cntTopVal;
} ck_pwm_priv_t;

#define pwm_ch_t ck_pwm_priv_t

typedef struct {
    bool          	enable;
    bool            ch_en[6];
    ck_pwm_priv_t   ch[6];
} pwm_Ctx_t;

static pwm_Ctx_t pwmCtx = {
    .enable = FALSE,
    .ch_en = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE},
};

void phy_pwm_module_init(void)
{
    int i = 0;

    if (pwmCtx.enable == TRUE) {
        return;
    }

    pwmCtx.enable = TRUE;

    for (i = 0; i < 6; i++) {
        pwmCtx.ch_en[i] = FALSE;
        pwmCtx.ch[i].pwmN = (PWMN_e)i;
        pwmCtx.ch[i].pwmDiv = PWM_CLK_NO_DIV;
        pwmCtx.ch[i].pwmMode = PWM_CNT_UP;
        pwmCtx.ch[i].pwmPolarity = PWM_POLARITY_RISING;
        pwmCtx.ch[i].cmpVal = 0;
        pwmCtx.ch[i].cntTopVal = 0;
    }

    phy_pwm_stop();
}

void phy_pwm_module_deinit(void)
{
    int i = 0;

    if (pwmCtx.enable == FALSE) {
        return;
    }

    pwmCtx.enable = FALSE;

    for (i = 0; i < 6; i++) {
        pwmCtx.ch_en[i] = FALSE;

        pwmCtx.ch[i].pwmN = (PWMN_e)i;
        pwmCtx.ch[i].pwmDiv = PWM_CLK_NO_DIV;
        pwmCtx.ch[i].pwmMode = PWM_CNT_UP;
        pwmCtx.ch[i].pwmPolarity = PWM_POLARITY_RISING;
        pwmCtx.ch[i].cmpVal = 0;
        pwmCtx.ch[i].cntTopVal = 0;

        phy_pwm_close_channel((PWMN_e)i);
        phy_pwm_destroy((PWMN_e)i);
    }

    phy_pwm_stop();
}

void phy_pwm_ch_start(uint8_t ch)
{
    if (pwmCtx.enable == FALSE) {
        return;
    }

    phy_pwm_open_channel(ch);
    pwmCtx.ch_en[ch] = TRUE;
    phy_pwm_start();
}

void phy_pwm_ch_stop(uint8_t ch)
{
    ck_pwm_priv_t *p = &pwmCtx.ch[ch];

    if (pwmCtx.ch_en[p->pwmN] == FALSE) {
        return;
    } else {
        phy_pwm_stop();
        pwmCtx.ch_en[p->pwmN] = FALSE;
        phy_pwm_close_channel(p->pwmN);
        phy_pwm_destroy(p->pwmN);
    }
}

bool phy_pwm_ch_enable(PWMN_e pwmN)
{
    return pwmCtx.ch_en[pwmN];
}

/*
new api
*/
/**
  \brief       Initialize PWM Interface. 1. Initializes the resources needed for the PWM interface 2.registers event callback function
  \param[in]   idx pwm idx
  \return      handle pwm handle to operate.
*/
pwm_handle_t csi_pwm_initialize(uint32_t idx)
{
    if (idx > 0) {
        return NULL;
    }

    pwmCtx.enable = true;
    return &pwmCtx;
}

void csi_pwm_uninitialize(pwm_handle_t handle)
{
    phy_pwm_module_deinit();
}

int32_t csi_pwm_config(pwm_handle_t handle,
                       uint8_t channel,
                       uint32_t period_us,
                       uint32_t pulse_width_us)
{
    PWM_NULL_PARAM_CHK(handle);

    if (pulse_width_us > period_us) {
        return ERR_PWM(DRV_ERROR_PARAMETER);
    }

    pwm_Ctx_t *base = handle;
    ck_pwm_priv_t *p = &base->ch[channel];
    int ch_en = base->ch_en[p->pwmN];

    if (ch_en == TRUE) {
        csi_pwm_stop(handle, channel);
    }

    base->ch_en[channel] = false;
    p->pwmN = channel;
    p->pwmPolarity = PWM_POLARITY_FALLING;
    p->pwmMode = PWM_CNT_UP_AND_DOWN;

    uint32_t div = BIT(p->pwmDiv & 0XFF);
    p->cntTopVal = PWM_BASE_FREQ / div / (S_US / period_us);
    p->cmpVal = (p->cntTopVal * pulse_width_us) / period_us;

    if (p->cmpVal <= 0) {
        p->cmpVal = 1;
    }

    phy_pwm_init(p->pwmN, p->pwmDiv, p->pwmMode, p->pwmPolarity);
    phy_pwm_set_count_val(channel, p->cmpVal, p->cntTopVal);

    if (ch_en == TRUE) {
        csi_pwm_start(handle, channel);
    }

    return 0;
}

void csi_pwm_start(pwm_handle_t handle, uint8_t channel)
{
    if (handle !=  NULL) {
        phy_pwm_ch_start(channel);
    }
}

void csi_pwm_stop(pwm_handle_t handle, uint8_t channel)
{
    if (handle !=  NULL) {
        phy_pwm_ch_stop(channel);
    }
}

int32_t csi_pwm_power_control(pwm_handle_t handle, csi_power_stat_e state)
{
    if (state == DRV_POWER_FULL) {
        clk_gate_enable(MOD_PWM);
        return 0;
    } else if (state == DRV_POWER_OFF) {
        clk_gate_disable(MOD_PWM);
        return 0;
    } else {
        return ERR_PWM(DRV_ERROR_UNSUPPORTED);
    }
}
void drv_pwm_config_clockdiv(pwm_handle_t handle, uint8_t channel, uint32_t div)
{
    PWM_NULL_PARAM_CHK_NORETVAL(handle);

    pwm_Ctx_t *base = handle;
    ck_pwm_priv_t *p = &base->ch[channel];

    int i = 0;

    while (i < MAX_DIV + 1) {
        if (div == BIT(i)) {
            break;
        }

        i++;
    }

    if (i <= MAX_DIV) {
        p->pwmDiv = i;
    }
}

uint32_t drv_pwm_get_clockdiv(pwm_handle_t handle, uint8_t channel)
{
    PWM_NULL_PARAM_CHK(handle);

    pwm_Ctx_t *base = handle;
    ck_pwm_priv_t *p = &base->ch[channel];

    uint32_t div = p->pwmDiv & 0XFF;
    return BIT(div);
}
