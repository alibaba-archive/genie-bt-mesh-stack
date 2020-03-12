/**
 ****************************************************************************************
 *
 * @file app_task.h
 *
 * @brief Header file - APPTASK.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef APP_TASK_H_
#define APP_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APPTASK Task
 * @ingroup APP
 * @brief Routes ALL messages to/from APP block.
 *
 * The APPTASK is the block responsible for bridging the final application with the
 * RWBLE software host stack. It communicates with the different modules of the BLE host,
 * i.e. @ref SMP, @ref GAP and @ref GATT.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>         // Standard Integer
#include "rwip.h"
#include "rwip_task.h"      // Task definitions
#include "ke_task.h"        // Kernel Task

/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of APP Task Instances
#define APP_IDX_MAX                 (1)


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States of APP task
enum appm_state
{
    /// Initialization state
    APPM_INIT,
    /// Database create state
    APPM_CREATE_DB,
    /// Ready State
    APPM_READY,
    /// Advertising state
    APPM_ADVERTISING,
    /// Connected state
    APPM_CONNECTED,
	
	APPM_PRO_HCI_AHI_MSG,
    /// Number of defined states.
    APPM_STATE_MAX
};


/// APP Task messages
enum appm_msg
{
    APPM_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_APP),
    //chengyi change to MESH_PROFILE_PTS_TAG
    //from
    //#if(PTS_WITHOUT_MESH)
    //to
    #if(MESH_PROFILE_PTS_TEST)
    //end
    APP_SEND_SECURITY_REQ,//PTS test GAP layer

    APP_PARAM_UPDATE_REQ_IND,

    HCI_AHI_MSG_PRO_CMD,

	HCI_AHI_MSG_PRO_TMR,
    #endif
	
	APP_PERIOD_TIMER,

	//APP_HCI_SEND_HANDLER,

	APP_HCI_NO_CMD_HANDLER
};

struct app_hci_send_data
{
	void *param;
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler appm_default_handler;
extern ke_state_t appm_state[APP_IDX_MAX];


/// @} APPTASK

#endif //(BLE_APP_PRESENT)

#endif // APP_TASK_H_
