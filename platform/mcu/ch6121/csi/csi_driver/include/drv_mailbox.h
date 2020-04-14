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
 * @file     drv_mailbox.h
 * @brief    header file for mailbox driver
 * @version  V1.0
 * @date     28. Jan 2019
 ******************************************************************************/

#ifndef _CSI_MAILBOX_H_
#define _CSI_MAILBOX_H_


#include <drv_common.h>

#ifdef __cplusplus
extern "C" {
#endif
/// definition for mailbox handle.
typedef void *mailbox_handle_t;

/****** MAILBOX Event *****/
typedef enum {
    MAILBOX_EVENT_SEND_COMPLETE       = 0,  ///< Send completed; however mailbox may still transmit data
    MAILBOX_EVENT_RECEIVED            = 1, ///< Data Received, only in mailbox buf, call memcpy() get the data
} mailbox_event_e;

typedef void (*mailbox_event_cb_t)(mailbox_handle_t handle, mailbox_event_e event);   ///< Pointer to \ref mailbox_event_cb_t : MAILBOX Event call back.

/**
  \brief       Initialize MAILBOX Interface. 1. Initializes the resources needed for the MAILBOX interface 2.registers event callback function
  \param[in]   dst_idx index of dest mailbox
  \param[in]   cb_event  event call back function \ref mailbox_event_cb_t
  \return      return mailbox handle if success
*/
mailbox_handle_t drv_mailbox_initialize(int32_t dst_idx, mailbox_event_cb_t cb_event);

/**
  \brief       De-initialize MAILBOX Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle  mailbox handle to operate.
  \return      error code
*/
int32_t drv_mailbox_uninitialize(mailbox_handle_t handle);

/**
  \brief       Start sending data to MAILBOX transmitter,(received data is ignored).
               This function is non-blocking,\ref mailbox_event_e is signaled when operation completes or error happens.
               \ref csi_mailbox_get_status can get operation status.
  \param[in]   handle  mailbox handle to operate.
  \param[in]   data  Pointer to buffer with data to send to MAILBOX transmitter.
  \param[in]   num Number of data items to send
  \return      sent number of data
*/
int32_t drv_mailbox_send_mail(mailbox_handle_t handle, const void *data, uint32_t num);

/**
  \brief       Start Receiving data from Mailbox receiver.
  \param[in]   handle  mailbox handle to operate.
  \param[in]   data  Pointer to buffer with data to receive from mailbox.
  \param[in]   num   Number of data items to receive
  \return      error code
*/
int32_t drv_mailbox_receive_query(mailbox_handle_t handle, void *data, uint32_t num);

#ifdef __cplusplus
}
#endif

#endif /* _CSI_MAILBOX_H_ */
