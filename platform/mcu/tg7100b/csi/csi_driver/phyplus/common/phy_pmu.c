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
 * @file     ck_pmu.c
 * @brief    CSI Source File for PMU Driver
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/

#include <csi_config.h>
#include <stdio.h>
#include <string.h>
#include <drv_pmu.h>
#include <soc.h>
#include <power_manager.h>
#include <ll_sleep.h>

#define ERR_PMU(errno) (CSI_DRV_ERRNO_PMU_BASE | errno)
#define PMU_NULL_PARAM_CHK(para) HANDLE_PARAM_CHK(para, ERR_PMU(DRV_ERROR_PARAMETER))

#define      SRAM0_ADDRESS        0x1fff0f00
#define      SRAM1_ADDRESS        0x1fff9000
#define      SRAM2_ADDRESS        0x20001000
#define      SRAM3_ADDRESS        0x20010100
#define      SRAM4_ADDRESS        0x20012100

typedef struct {
    uint8_t idx;
    uint32_t base;
    uint32_t irq;
    pmu_event_cb_t cb;
} phy_pmu_priv_t;

typedef enum {
    WAIT_MODE = 0,
    DOZE_MODE,
    STOP_MODE,
    STANDBY_MODE,
    SLEEP_MODE
} lpm_mode_e;

extern int32_t target_get_pmu(int32_t idx, uint32_t *base, uint32_t *irq);
extern int32_t arch_do_cpu_save(void);
extern int32_t arch_do_cpu_resume(void);
extern int32_t arch_resume_context(void);

static uint8_t pmu_power_status;
static phy_pmu_priv_t pmu_handle[CONFIG_PMU_NUM];
#define CONFIG_CORETIM_REGISTER_NUM_SAVE    15
static uint32_t cortim_regs_saved[CONFIG_CORETIM_REGISTER_NUM_SAVE];
#define CONFIG_CPU_REGISTER_NUM_SAVE    16
uint32_t g_arch_cpu_saved[CONFIG_CPU_REGISTER_NUM_SAVE];
/* Driver Capabilities */
//
// Functions
//

static void do_prepare_sleep_action(int32_t idx)
{
    /* save the coretim register */
    cortim_regs_saved[0] = SysTick->LOAD;
    cortim_regs_saved[1] = SysTick->CTRL;
    cortim_regs_saved[2] = SCB->ICSR;
    cortim_regs_saved[3] = SCB->SHP[0];
    cortim_regs_saved[4] = SCB->SHP[1];
    cortim_regs_saved[5] = SCB->SHCSR;
    cortim_regs_saved[6] = NVIC->ISER[0];
    cortim_regs_saved[7] = NVIC->IP[0];
    cortim_regs_saved[8] = NVIC->IP[1];
    cortim_regs_saved[9] = NVIC->IP[2];
    cortim_regs_saved[10] = NVIC->IP[3];
    cortim_regs_saved[11] = NVIC->IP[4];
    cortim_regs_saved[12] = NVIC->IP[5];
    cortim_regs_saved[13] = NVIC->IP[6];
    cortim_regs_saved[14] = NVIC->IP[7];
}

volatile uint32_t forever_write;
static void do_wakeup_sleep_action(int32_t idx)
{
    /* resume the coretim register */
    SysTick->LOAD = cortim_regs_saved[0];
    SysTick->VAL   = 0UL;
    SysTick->CTRL = cortim_regs_saved[1];

    SCB->ICSR  = cortim_regs_saved[2];
    SCB->SHP[0] = cortim_regs_saved[3];
    SCB->SHP[1] = cortim_regs_saved[4];
    SCB->SHCSR  = cortim_regs_saved[5];
    NVIC->ISER[0] = cortim_regs_saved[6];
    NVIC->ICPR[0] = 0xffffffff;
    NVIC->IP[0] = cortim_regs_saved[7];
    NVIC->IP[1] = cortim_regs_saved[8];
    NVIC->IP[2] = cortim_regs_saved[9];
    NVIC->IP[3] = cortim_regs_saved[10];
    NVIC->IP[4] = cortim_regs_saved[11];
    NVIC->IP[5] = cortim_regs_saved[12];
    NVIC->IP[6] = cortim_regs_saved[13];
    NVIC->IP[7] = cortim_regs_saved[14];
}

void soc_sleep(pmu_handle_t handle, lpm_mode_e mode)
{
    uint32 sram0, sram1, sram2, sram3, sram4;

    // read data from sram0 - sram4
    sram0 = *(uint32 *)(SRAM0_ADDRESS);
    sram1 = *(uint32 *)(SRAM1_ADDRESS);
    sram2 = *(uint32 *)(SRAM2_ADDRESS);
    sram3 = *(uint32 *)(SRAM3_ADDRESS);
    sram4 = *(uint32 *)(SRAM4_ADDRESS);

    if (mode == STOP_MODE) {
        *(volatile uint32_t *) 0x4000f004 = 0xa5a55a5a; //enter system sleep mode
    } else if (mode == STANDBY_MODE) {
        *(volatile uint32_t *) 0x4000f000 = 0x5a5aa5a5; //enter system off mode
    }

    *(volatile uint32 *)(SRAM0_ADDRESS) = sram0;
    *(volatile uint32 *)(SRAM1_ADDRESS) = sram1;
    *(volatile uint32 *)(SRAM2_ADDRESS) = sram2;
    *(volatile uint32 *)(SRAM3_ADDRESS) = sram3;
    *(volatile uint32 *)(SRAM4_ADDRESS) = sram4;

    while (1) {
        forever_write = 0x12345678;
    }
}

/**
  \brief       Initialize PMU Interface. 1. Initializes the resources needed for the PMU interface 2.registers event callback function
  \param[in]   idx device id
  \param[in]   cb_event  Pointer to \ref pmu_event_cb_t
  \return      pointer to pmu handle
*/
pmu_handle_t csi_pmu_initialize(int32_t idx, pmu_event_cb_t cb_event)
{
    if (idx < 0 || idx >= CONFIG_PMU_NUM) {
        return NULL;
    }

    phy_pmu_priv_t *pmu_priv = &pmu_handle[idx];

    /* initialize the pmu context */
    pmu_priv->idx = idx;
    pmu_priv->cb = cb_event;

    return (pmu_handle_t)pmu_priv;
}

/**
  \brief       De-initialize PMU Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle  pmu handle to operate.
  \return      error code
*/
int32_t csi_pmu_uninitialize(pmu_handle_t handle)
{
    PMU_NULL_PARAM_CHK(handle);

    phy_pmu_priv_t *pmu_priv = handle;
    pmu_priv->cb = NULL;

    return 0;
}

int32_t csi_pmu_power_control(pmu_handle_t handle, csi_power_stat_e state)
{
    PMU_NULL_PARAM_CHK(handle);

    phy_pmu_priv_t *pmu_priv = handle;

    switch (state) {
        case DRV_POWER_OFF:
        case DRV_POWER_LOW:
            return ERR_PMU(DRV_ERROR_UNSUPPORTED);

        case DRV_POWER_FULL:
            if (pmu_power_status == DRV_POWER_SUSPEND) {
                do_wakeup_sleep_action(pmu_priv->idx);
            }

            break;

        case DRV_POWER_SUSPEND:
            do_prepare_sleep_action(pmu_priv->idx);
            break;

        default:
            return ERR_PMU(DRV_ERROR_PARAMETER);
    }

    pmu_power_status = state;

    return 0;
}

/**
  \brief       choose the pmu mode to enter
  \param[in]   handle  pmu handle to operate.
  \param[in]   mode    \ref pmu_mode_e
  \return      error code
*/
int32_t csi_pmu_enter_sleep(pmu_handle_t handle, pmu_mode_e mode)
{
    PMU_NULL_PARAM_CHK(handle);

    switch (mode) {
        case PMU_MODE_RUN:
            break;

        case PMU_MODE_SLEEP:
        case PMU_MODE_DOZE:
        case PMU_MODE_SHUTDOWN:
            return ERR_PMU(DRV_ERROR_UNSUPPORTED);
        case PMU_MODE_STANDBY:
             subWriteReg(0x4000f01c, 6, 6, 0x00); //disable software control
             enter_sleep_off_mode(2);
             break;
        case PMU_MODE_DORMANT:
            do_prepare_sleep_action(0);

            if (arch_do_cpu_save() == 0) {
                enter_sleep_off_mode(1);
            }

            do_wakeup_sleep_action(0);
            break;

        default:
            return ERR_PMU(DRV_ERROR_PARAMETER);
    }

    return 0;
}

/**
  \brief       Config the wakeup source.
  \param[in]   handle  pmu handle to operate
  \param[in]   type    \ref pmu_wakeup_type
  \param[in]   pol     \ref pmu_wakeup_pol
  \param[in]   enable  flag control the wakeup source is enable or not
  \return      error code
*/
int32_t csi_pmu_config_wakeup_source(pmu_handle_t handle, uint32_t wakeup_num, pmu_wakeup_type_e type, pmu_wakeup_pol_e pol, uint8_t enable)
{
    PMU_NULL_PARAM_CHK(handle);
#if 0

    if (wakeup_num >= 32 || (type != PMU_WAKEUP_TYPE_LEVEL) || (pol != PMU_WAKEUP_POL_HIGH)) {
        return ERR_PMU(DRV_ERROR_PARAMETER);
    }

    if (enable) {
        csi_vic_set_wakeup_irq(wakeup_num);
    } else {
        csi_vic_clear_wakeup_irq(wakeup_num);
    }

#endif
    return 0;
}
