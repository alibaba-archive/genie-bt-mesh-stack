/**
 ****************************************************************************************
 *
 * @file app.h
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef APP_H_
#define APP_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Application entry point.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>          // Standard Integer Definition
#include <co_bt.h>           // Common BT Definitions
#include "arch.h"            // Platform Definitions
//#include "gapc.h"            // GAPC Definitions
//#include "nvds.h"


/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximal length of the Device Name value
#define APP_DEVICE_NAME_MAX_LEN      (18)

/// Default Advertising duration - 30s (in multiple of 10ms)
#define APP_DFLT_ADV_DURATION        (3000)




/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// List of Application NVDS TAG identifiers
enum app_nvds_tag
{
    /// BLE Application Advertising data
    NVDS_TAG_APP_BLE_ADV_DATA           = 0x0B,
    NVDS_LEN_APP_BLE_ADV_DATA           = 32,

    /// BLE Application Scan response data
    NVDS_TAG_APP_BLE_SCAN_RESP_DATA     = 0x0C,
    NVDS_LEN_APP_BLE_SCAN_RESP_DATA     = 32,

    /// Mouse Sample Rate
    NVDS_TAG_MOUSE_SAMPLE_RATE          = 0x38,
    NVDS_LEN_MOUSE_SAMPLE_RATE          = 1,
    /// Peripheral Bonded
    NVDS_TAG_PERIPH_BONDED              = 0x39,
    NVDS_LEN_PERIPH_BONDED              = 1,
    /// Mouse NTF Cfg
    NVDS_TAG_MOUSE_NTF_CFG              = 0x3A,
    NVDS_LEN_MOUSE_NTF_CFG              = 2,
    /// Mouse Timeout value
    NVDS_TAG_MOUSE_TIMEOUT              = 0x3B,
    NVDS_LEN_MOUSE_TIMEOUT              = 2,
    /// Peer Device BD Address
    NVDS_TAG_PEER_BD_ADDRESS            = 0x3C,
    NVDS_LEN_PEER_BD_ADDRESS            = 7,
    /// Mouse Energy Safe
    NVDS_TAG_MOUSE_ENERGY_SAFE          = 0x3D,
    NVDS_LEN_MOUSE_SAFE_ENERGY          = 2,
    /// EDIV (2bytes), RAND NB (8bytes),  LTK (16 bytes), Key Size (1 byte)
    NVDS_TAG_LTK                        = 0x3E,
    NVDS_LEN_LTK                        = 28,
    /// PAIRING
    NVDS_TAG_PAIRING                    = 0x3F,
    NVDS_LEN_PAIRING                    = 54,
};

#if 0
enum app_loc_nvds_tag
{
    /// Audio mode 0 task
    NVDS_TAG_AM0_FIRST                  = NVDS_TAG_APP_SPECIFIC_FIRST, // 0x90
    NVDS_TAG_AM0_LAST                   = NVDS_TAG_APP_SPECIFIC_FIRST+16, // 0xa0

    /// Local device Identity resolving key
    NVDS_TAG_LOC_IRK,
    /// Peer device Resolving identity key (+identity address)
    NVDS_TAG_PEER_IRK,
	
	NVDS_TAG_POWER_RESET_CNT,
	
	NVDS_TAG_MESH_OTA_INFO,
	
	NVDS_TAG_MESH_LED_INFO,

    /// size of local identity resolving key
    NVDS_LEN_LOC_IRK                    = KEY_LEN,
    /// size of Peer device identity resolving key (+identity address)
    NVDS_LEN_PEER_IRK                   = sizeof(struct gapc_irk),
};
#endif

/// Application environment structure
struct app_env_tag
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection Index
    uint8_t  conidx;

    /// Last initialized profile
    uint8_t next_svc;

    /// Bonding status
    bool bonded;

    /// Device Name length
    uint8_t dev_name_len;
    /// Device Name
    uint8_t dev_name[APP_DEVICE_NAME_MAX_LEN];

    /// Local device IRK
    uint8_t loc_irk[KEY_LEN];
    #if(PTS_WITHOUT_MESH)
    /// Peer device IRK
    struct gapc_irk peer_irk;
    #endif
	
};


///  
struct hci_event_msg_pro_t
{



};

struct hci_ahi_msg_pro_t
{
    /// call back function pointer
    uint8_t (*callback) (void*, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void* dummy;
};


/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Application environment
extern struct app_env_tag app_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the BLE demo application.
 ****************************************************************************************
 */
void appm_init(void);

#ifndef ALIOS_KERNEL
/**
 ****************************************************************************************
 * @brief Add a required service in the database
 ****************************************************************************************
 */
bool appm_add_svc(void);

/**
 ****************************************************************************************
 * @brief Put the device in general discoverable and connectable mode
 ****************************************************************************************
 */
void appm_start_advertising(void);

/**
 ****************************************************************************************
 * @brief Put the device in non discoverable and non connectable mode
 ****************************************************************************************
 */
void appm_stop_advertising(void);

/**
 ****************************************************************************************
 * @brief Send to request to update the connection parameters
 ****************************************************************************************
 */
void appm_update_param(struct gapc_conn_param *conn_param);

/**
 ****************************************************************************************
 * @brief Send a disconnection request
 ****************************************************************************************
 */
void appm_disconnect(void);

/**
 ****************************************************************************************
 * @brief Retrieve device name
 *
 * @param[out] device name
 *
 * @return name length
 ****************************************************************************************
 */
uint8_t appm_get_dev_name(uint8_t* name);

void hci_ahi_event_send(uint8_t (*callback) (void*, uint8_t), void* dummy);
#endif

void hci_event_recv_process(void *recv_buf);
void hci_event_process(uint8_t type, uint8_t *buf, uint16_t len);
void appm_hci_send(void *param);

/// @} APP

#endif //(BLE_APP_PRESENT)

#endif // APP_H_
