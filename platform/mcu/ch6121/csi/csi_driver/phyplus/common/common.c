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
* @file		common.c
* @brief	Contains all functions support for common function driver£¬such as string function,you can use this driver for spi,adc,uart and so on
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include "types.h"
#include "common.h"
#include "ap_cp.h"

/**************************************************************************************
 * @fn          subWriteReg
 *
 * @brief       This function process for write register with sub bit
 *
 * input parameters
 *
 * @param       uint32_t addr: register address
 *              uint8_t hOff: high bit offset
 *              uint8_t lOff: low bit offset
 *              uint32_t value: write value
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void subWriteReg(uint32_t addr, uint8_t hOff, uint8_t lOff, uint32_t value)
{
    uint32_t temp = read_reg(addr);
    uint32_t temp2 = 0xffffffff;
    uint32_t temp3 = value << lOff;

    for (uint8_t i = lOff; i <= hOff; i++) {
        temp2 &= ~BIT(i);
    }

    temp = temp & temp2;
    temp = temp | temp3;
    write_reg(addr, temp);
}

