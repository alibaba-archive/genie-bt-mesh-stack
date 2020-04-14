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
  Filename:       OSAL_Timers.h
  Revised:
  Revision:

  Description:    This file contains the OSAL Timer definition and manipulation functions.



**************************************************************************************************/

#ifndef OSAL_TIMERS_H
#define OSAL_TIMERS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 * the unit is chosen such that the 320us tick equivalent can fit in
 * 32 bits.
 */
#define OSAL_TIMERS_MAX_TIMEOUT 0x28f5c28e /* unit is ms*/

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
 * Initialization for the OSAL Timer System.
 */
extern void osalTimerInit(void);

/*
 * Set a Timer
 */
extern uint8 osal_start_timerEx(uint8 task_id, uint16 event_id, uint32 timeout_value);

/*
 * Set a timer that reloads itself.
 */
extern uint8 osal_start_reload_timer(uint8 taskID, uint16 event_id, uint32 timeout_value);

/*
 * Stop a Timer
 */
extern uint8 osal_stop_timerEx(uint8 task_id, uint16 event_id);

/*
 * Get the tick count of a Timer.
 */
extern uint32 osal_get_timeoutEx(uint8 task_id, uint16 event_id);

/*
 * Simulated Timer Interrupt Service Routine
 */

extern void osal_timer_ISR(void);

/*
 * Adjust timer tables
 */
extern void osal_adjust_timers(void);

/*
 * Update timer tables
 */
extern void osalTimerUpdate(uint32 updateTime);

/*
 * Count active timers
 */
extern uint8 osal_timer_num_active(void);

/*
 * Set the hardware timer interrupts for sleep mode.
 * These functions should only be called in OSAL_PwrMgr.c
 */
extern void osal_sleep_timers(void);
extern void osal_unsleep_timers(void);

/*
 * Read the system clock - returns milliseconds
 */
extern uint32 osal_GetSystemClock(void);

/*
 * Get the next OSAL timer expiration.
 * This function should only be called in OSAL_PwrMgr.c
 */
extern uint32 osal_next_timeout(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_TIMERS_H */
