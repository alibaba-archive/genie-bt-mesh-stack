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




#include "ap_cp.h"
#include "hal_defs.h"
#include "hal_mcu.h"
#include "clock.h"
#include "types.h"
#include "gpio.h"
#include "global_config.h"

extern uint32_t hclk,pclk;

//extern uint32_t osal_sys_tick;

void clk_gate_enable(MODULE_e module)
{
    if (module < 3) {
        return;
    }

    AP_PCR->CLKG |= BIT(module);
}

void clk_gate_disable(MODULE_e module)
{
    if (module < 3) {
        return;
    }

    AP_PCR->CLKG &= ~(BIT(module));
}

void clk_reset(MODULE_e module)
{
    AP_PCR->RESET &= ~(BIT(module));
    AP_PCR->RESET |= BIT(module);
}

uint32_t clk_hclk(void)
{
    return hclk;
}
uint32_t clk_pclk(void)
{
    return pclk;
}


/**************************************************************************************
 * @fn          hal_rtc_clock_config
 *
 * @brief       This function process for 32768Hz Clock configuation
 *
 * input parameters
 *
 * @param       CLK32K_e clk32Mode: CLK_32K_RC_LOWPOWER --> use 32K RC osc for extra low power
                                  : CLK_32K_RC_STABLE   --> use 32K RC osc for stable conectivity
                                  : CLK_32K_XTAL        --> use 32K Xtal
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None
 **************************************************************************************/

void hal_rtc_clock_config(uint8_t clk32Mode)
{
    if (clk32Mode == CLK_32K_RCOSC) {
        subWriteReg(0x4000f014, 31, 27, 0x05);
        subWriteReg(0x4000f01c, 16, 7, 0x3fb); //software control 32k_clk
        subWriteReg(0x4000f01c, 6, 6, 0x01);  //enable software control

        pGlobal_config[LL_SWITCH] |= RC32_TRACKINK_ALLOW | LL_RC32K_SEL;

//        //disable smart windwo for stable conectivity not higher power consumption
//        pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]=0;
    } else if (clk32Mode == CLK_32K_XTAL) {
        // P16 P17 for 32K XTAL input
        phy_gpio_pull_set(P16, FLOATING);
        phy_gpio_pull_set(P17, FLOATING);

        subWriteReg(0x4000f01c, 9, 8, 0x03); //software control 32k_clk
        subWriteReg(0x4000f01c, 6, 6, 0x00); //disable software control

        subWriteReg(0x4000f014, 31, 27, 0x16);
        pGlobal_config[LL_SWITCH] &= 0xffffffee;
    }

}

//uint32_t hal_systick(void)
//{
//  return osal_sys_tick;
//}


//uint32_t hal_ms_intv(uint32_t tick)
//{
//  uint32_t diff = 0;
//  if(osal_sys_tick < tick){
//    diff = 0xffffffff- tick;
//    diff = osal_sys_tick + diff;
//  }
//  else
//  {
//    diff = osal_sys_tick - tick;
//  }
//  return diff*625/1000;
//}

