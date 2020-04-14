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
  Filename:       osal_bufmgr.h
  Revised:
  Revision:

  Description:    This file contains the buffer management definitions.



**************************************************************************************************/

#ifndef OSAL_BUFMGR_H
#define OSAL_BUFMGR_H

#include "comdef.h"
#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * VARIABLES
 */


/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Allocate a block of memory.
 */
extern void *osal_bm_alloc(uint16 size);

/*
 * Add or remove header space for the payload pointer.
 */
extern void *osal_bm_adjust_header(void *payload_ptr, int16 size);

/*
 * Add or remove tail space for the payload pointer.
 */
extern void *osal_bm_adjust_tail(void *payload_ptr, int16 size);

/*
 * Free a block of memory.
 */
extern void osal_bm_free(void *payload_ptr);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_BUFMGR_H */
