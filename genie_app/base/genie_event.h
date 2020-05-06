/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GENIE_EVENT_H_
#define _GENIE_EVENT_H_

#include <zephyr.h>

typedef enum {
/* !!!START!!! --- Don't add new ID before this one */
    GENIE_EVT_START = 0,

/* Reset Related Operation */
    GENIE_EVT_SW_RESET = GENIE_EVT_START, /* triggered from cloud */
    GENIE_EVT_HW_RESET_START,  /* triggered from user */
    GENIE_EVT_HW_RESET_DONE,   /*triggered by reset by repeat module */

/* SDK triggered event, with prefix of GENIE_EVT_SDK_MESH_ */
    GENIE_EVT_SDK_START,
    GENIE_EVT_SDK_MESH_INIT = GENIE_EVT_SDK_START,
    GENIE_EVT_SDK_MESH_PBADV_START,
    GENIE_EVT_SDK_MESH_PBADV_TIMEOUT,
    GENIE_EVT_SDK_MESH_SILENT_START,

    GENIE_EVT_SDK_MESH_PROV_START,
    GENIE_EVT_SDK_MESH_PROV_DATA,
    GENIE_EVT_SDK_MESH_PROV_TIMEOUT,
    GENIE_EVT_SDK_MESH_PROV_SUCCESS,
    GENIE_EVT_SDK_MESH_PROV_FAIL,

    GENIE_EVT_SDK_APPKEY_ADD,
    GENIE_EVT_SDK_APPKEY_DEL,
    GENIE_EVT_SDK_APPKEY_UPDATE,
    GENIE_EVT_SDK_NETKEY_ADD,
    GENIE_EVT_SDK_NETKEY_DEL,
    GENIE_EVT_SDK_NETKEY_UPDATE,
    GENIE_EVT_SDK_SUB_ADD,
    GENIE_EVT_SDK_SUB_DEL,
    GENIE_EVT_SDK_HB_SET,
    GENIE_EVT_SDK_SEQ_UPDATE,
    GENIE_EVT_SDK_STATE_SYNC, // Used to sync device's state with cloud
    //GENIE_EVT_SDK_IVI_UPDATE,
    //GENIE_EVENT_SUB_SET,
    //GENIE_EVENT_HB_SET,

    GENIE_EVT_SDK_ANALYZE_MSG,

    GENIE_EVT_SDK_AIS_DISCON,
#ifdef CONFIG_MESH_MODEL_TRANS
    GENIE_EVT_SDK_DELAY_START,
    GENIE_EVT_SDK_DELAY_END,
    GENIE_EVT_SDK_TRANS_START,
    GENIE_EVT_SDK_TRANS_CYCLE,
    GENIE_EVT_SDK_TRANS_END,
#endif
    GENIE_EVT_SDK_ACTION_DONE,

#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    GENIE_EVT_SDK_MESH_PWRON_INDC,
    GENIE_EVT_SDK_INDICATE,
    GENIE_EVT_SDK_VENDOR_MSG,
#endif

/* APP triggered event, with prefix of GENIE_EVT_APP_ */
    GENIE_EVT_APP_START,
    GENIE_EVT_TIME_OUT,

/* !!!END!!! --- Don't add new ID after this one */
    GENIE_EVT_END

} E_GENIE_EVENT;

extern const char *genie_event_str[];

 /**
 * @brief The handler for the underlying events. If necessary
 *        this handler dispatch the user events to applications.
 * @param[in] event refers to the event details.
 * @param[in] args refers to the additional information for the event.
 */
void genie_event(E_GENIE_EVENT event, void *args);

/**
* @brief Get the prov status of genie mesh stack.
* @return the prov status.
*/
bool genie_is_provisioned(void);

#endif // _GENIE_EVENT_H_
