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
 * @file     drv_tipc.h
 * @brief    header file for tipc driver
 * @version  V1.0
 * @date     08. Mar 2019
 ******************************************************************************/

#ifndef _DRV_TIPC_H_
#define _DRV_TIPC_H_


#include <stdint.h>
#include <drv_common.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    TIPC_PROPERITY_UNTRUST          = 0,   ///<  untrust property in AHB/APB island
    TIPC_PROPERITY_TRUST                   ///<  trust property in AHB/APB island
} enum_tipc_properity_e;

/**
  \brief       Config the tipc module properity
  \param[in]   idx tipc module idx \ref ckenum_tipc_module_e
  \param[in]   properity module properity for trust \ref enum_tipc_properity_e
*/
void drv_tipc_set_module_properity(uint32_t idx, enum_tipc_properity_e properity);

/**
  \brief       set data to a module readable address
  \param[in]   idx tipc module idx \ref ckenum_tipc_module_e
  \param[in]   data  set data
*/
void drv_tipc_set_module_rw_addr(uint32_t idx, uint32_t data);

/**
  \brief       get data from a module readable address
  \param[in]   idx tipc module idx \ref ckenum_tipc_module_e
  \return      get data
*/
uint32_t drv_tipc_get_module_rw_addr(uint32_t idx);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_TIPC_H_ */

