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



/**************************************************************************************************
**************************************************************************************************/

#ifndef OSAL_NV_H
#define OSAL_NV_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Initialize NV service
 */
extern void osal_nv_init(void *p);

/*
 * Initialize an item in NV
 */
extern uint8 osal_nv_item_init(uint16 id, uint16 len, void *buf);

/*
 * Read an NV attribute
 */
extern uint8 osal_nv_read(uint16 id, uint16 offset, uint16 len, void *buf);

/*
 * Write an NV attribute
 */
extern uint8 osal_nv_write(uint16 id, uint16 offset, uint16 len, void *buf);

/*
 * Get the length of an NV item.
 */
extern uint16 osal_nv_item_len(uint16 id);

/*
 * Delete an NV item.
 */
extern uint8 osal_nv_delete(uint16 id, uint16 len);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_NV.H */
