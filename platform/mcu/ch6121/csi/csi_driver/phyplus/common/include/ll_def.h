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




#ifndef LL_DEF_H_
#define LL_DEF_H_

#include "types.h"
#include "comdef.h"
#include "ll_buf.h"
#include "ll.h"

//chenlf
#undef MAX_NUM_LL_CONN
#define MAX_NUM_LL_CONN                          1

#define LL_PKT_PREAMBLE_LEN                      1
#define LL_PKT_SYNCH_LEN                         4
#define LL_PKT_LLID_LEN                          1
#define LL_PKT_HDR_LEN                           2
#define LL_PKT_MIC_LEN                           4
#define LL_PKT_CRC_LEN                           3

#define LL_DATA_PDU_HDR_LLID_RESERVED            0
#define LL_DATA_PDU_HDR_LLID_DATA_PKT_NEXT       1
#define LL_DATA_PDU_HDR_LLID_DATA_PKT_FIRST      2
#define LL_DATA_PDU_HDR_LLID_CONTROL_PKT         3


///adv header shift and mask
#define PDU_TYPE_SHIFT                           0
#define PDU_TYPE_MASK                            0xf
#define TX_ADD_SHIFT                             6
#define TX_ADD_MASK                              0x40
#define RX_ADD_SHIFT                             7
#define RX_ADD_MASK                              0x80
#define LENGTH_SHIFT                             8
#define LENGTH_MASK                              0x3f00

// macro for bit operations
#define SET_BITS(p,f,l,m)  p=(f<<l) | (p & (~m))
#define GET_BITS(p,l,m)     (p&m)>>l

// Receive Flow Control
#define LL_RX_FLOW_CONTROL_DISABLED              0
#define LL_RX_FLOW_CONTROL_ENABLED               1

//LL packet type
#define	ADV_IND                                  0               //Connectable Undirected Event 
#define	ADV_DIRECT_IND                           1               //Connectable  Directed Event
#define	ADV_NONCONN_IND                          2               //Non-connectable Undirected Event 
#define	ADV_SCAN_REQ                             3
#define	ADV_SCAN_RSP                             4
#define	ADV_CONN_REQ                             5
#define	ADV_SCAN_IND                             6	             //Scannable Undirected Event

// LL state defines
#define LL_STATE_IDLE                            0x00
#define LL_STATE_ADV_UNDIRECTED                  0x01
#define LL_STATE_ADV_DIRECTED                    0x02
#define LL_STATE_ADV_SCAN                        0x03
#define LL_STATE_ADV_NONCONN                     0x04
#define LL_STATE_SCAN                            0x05
#define LL_STATE_INIT                            0x06
#define LL_STATE_CONN_SLAVE                      0x07
#define LL_STATE_CONN_MASTER                     0x08
#define LL_STATE_DIRECT_TEST_MODE_TX             0x09
#define LL_STATE_DIRECT_TEST_MODE_RX             0x0A
#define LL_STATE_MODEM_TEST_TX                   0x0B
#define LL_STATE_MODEM_TEST_RX                   0x0C
#define LL_STATE_MODEM_TEST_TX_FREQ_HOPPING      0x0D

//  =============== add in A2 for simultaneous slave and adv/scan
#define LL_SEC_STATE_IDLE             0x00
#define LL_SEC_STATE_SCAN             0x01
#define LL_SEC_STATE_ADV              0x02
#define LL_SEC_STATE_SCAN_PENDING     0x03
#define LL_SEC_STATE_ADV_PENDING      0x04
#define LL_SEC_STATE_IDLE_PENDING     0x05


//  ===== A2 End

//LL connecction  control type
#define	    LL_CONNECTION_UPDATE_REQ             0
#define		LL_CHANNEL_MAP_REQ                   1
#define		LL_TERMINATE_IND                     2
#define		LL_ENC_REQ                           3
#define		LL_ENC_RSP                           4
#define		LL_START_ENC_REQ                     5
#define		LL_START_ENC_RSP                     6
#define		LL_UNKNOWN_RSP                       7
#define		LL_FEATURE_REQ                       8
#define		LL_FEATURE_RSP                       9
#define		LL_PAUSE_ENC_REQ                     10
#define		LL_PAUSE_ENC_RSP                     11
#define		LL_VERSION_IND                       12
#define		LL_REJECT_IND                        13
#define		LL_SLAVE_FEATURE_REQ                 14
#define	    LL_CONNECTION_PARAM_REQ              15
#define	    LL_CONNECTION_PARAM_RSP              16
#define	    LL_REJECT_IND_EXT                    17
#define	    LL_PING_REQ                          18
#define	    LL_PING_RSP                          19
#define	    LL_LENGTH_REQ                        20
#define	    LL_LENGTH_RSP                        21
#define	    LL_PHY_REQ                           22
#define	    LL_PHY_RSP                           23
#define	    LL_PHY_UPDATE_IND                    24


#define LL_CONNECT_REQ_PAYLOAD_LEN               18
#define LL_CONN_UPDATE_REQ_PAYLOAD_LEN           12
#define LL_CHAN_MAP_REQ_PAYLOAD_LEN              8
#define LL_TERM_IND_PAYLOAD_LEN                  2
#define LL_ENC_REQ_PAYLOAD_LEN                   23
#define LL_ENC_RSP_PAYLOAD_LEN                   13
#define LL_START_ENC_REQ_PAYLOAD_LEN             1
#define LL_START_ENC_RSP_PAYLOAD_LEN             1
#define LL_PAUSE_ENC_REQ_PAYLOAD_LEN             1
#define LL_PAUSE_ENC_RSP_PAYLOAD_LEN             1
#define LL_REJECT_IND_PAYLOAD_LEN                2
#define LL_REJECT_EXT_IND_PAYLOAD_LEN            3
#define LL_FEATURE_REQ_PAYLOAD_LEN               9
#define LL_FEATURE_RSP_PAYLOAD_LEN               9
#define LL_VERSION_IND_PAYLOAD_LEN               6
#define LL_UNKNOWN_RSP_PAYLOAD_LEN               2
#define LL_LENGTH_REQ_PAYLOAD_LEN                9
#define LL_LENGTH_RSP_PAYLOAD_LEN                9
#define LL_PHY_REQ_PAYLOAD_LEN                   3
#define LL_PHY_RSP_PAYLOAD_LEN                   3
#define LL_PHY_UPDATE_IND_PAYLOAD_LEN            5




#define LL_MAX_NUM_CTRL_PROC_PKTS                4
#define LL_CTRL_UNDEFINED_PKT                    0xFF

// LL Events
#define LL_EVT_POST_PROCESS_NR                   0x0001
#define LL_EVT_DIRECTED_ADV_FAILED               0x0002
#define LL_EVT_SLAVE_CONN_CREATED                0x0004
#define LL_EVT_NEXT_INTERVAL                     0x0008
#define LL_EVT_MASTER_CONN_CANCELLED             0x0010
#define LL_EVT_TASK_TIMER_FENCE_EXPIRED          0x0020
#define LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM      0x0040
#define LL_EVT_START_32KHZ_XOSC_DELAY            0x0080
#define LL_EVT_32KHZ_XOSC_DELAY                  0x0100
#define LL_EVT_RESET_SYSTEM_HARD                 0x0200
#define LL_EVT_RESET_SYSTEM_SOFT                 0x0400

#define LL_EVT_MASTER_CONN_CREATED               0x0800
#define LL_EVT_SECONDARY_SCAN                    0x1000
#define LL_EVT_SECONDARY_ADV                     0x2000


#define LL_ADV_NONCONN_STATE                     0x00
#define LL_ADV_DISCOV_STATE                      0x01
#define LL_ADV_UNDIRECT_STATE                    0x02
#define LL_ADV_HDC_DIRECT_STATE                  0x03
#define LL_SCAN_PASSIVE_STATE                    0x04
#define LL_SCAN_ACTIVE_STATE                     0x05
#define LL_INIT_STATE                            0x06 // connection state in master role also supported
#define LL_SLAVE_STATE                           0x07
//
#define LL_ADV_NONCONN_SCAN_PASSIVE_STATE        0x10
#define LL_ADV_DISCOV_SCAN_PASSIVE_STATE         0x11
#define LL_ADV_UNDIRECT_SCAN_PASSIVE_STATE       0x12
#define LL_ADV_HDC_DIRECT_SCAN_PASSIVE_STATE     0x13
#define LL_ADV_NONCONN_SCAN_ACTIVE_STATE         0x14
#define LL_ADV_DISCOV_SCAN_ACTIVE_STATE          0x15
#define LL_ADV_UNDIRECT_SCAN_ACTIVE_STATE        0x16
#define LL_ADV_HDC_DIRECT_SCAN_ACTIVE_STATE      0x17
//
#define LL_ADV_NONCONN_INIT_STATE                0x20
#define LL_ADV_DISCOV_INIT_STATE                 0x21
#define LL_ADV_NONCONN_MASTER_STATE              0x22
#define LL_ADV_DISCOV_MASTER_STATE               0x23
#define LL_ADV_NONCONN_SLAVE_STATE               0x24
#define LL_ADV_DISCOV_SLAVE_STATE                0x25
#define LL_SCAN_PASSIVE_INIT_STATE               0x26
#define LL_SCAN_ACTIVE_INIT_STATE                0x27
//
#define LL_SCAN_PASSIVE_MASTER_STATE             0x30
#define LL_SCAN_ACTIVE_MASTER_STATE              0x31
#define LL_SCAN_PASSIVE_SLAVE_STATE              0x32
#define LL_SCAN_ACTIVE_SLAVE_STATE               0x33
#define LL_INIT_MASTER_STATE                     0x34 // master role and master role combination also supported
//
#define LL_ADV_LDC_DIRECT_STATE                  0x35
#define LL_ADV_LDC_DIRECT_SCAN_PASSIVE_STATE     0x36
#define LL_ADV_LDC_DIRECT_SCAN_ACTIVE_STATE      0x37

#define HCI_RX_PKT_HDR_SIZE                      5
#define LL_NUM_BYTES_FOR_CHAN_MAP                5    //(LL_MAX_NUM_ADV_CHAN+LL_MAX_NUM_DATA_CHAN)/sizeof(uint8)

#define LL_CTRL_PROC_STATUS_SUCCESS              0
#define LL_CTRL_PROC_STATUS_TERMINATE            1

#define LL_TX_DATA_CONTEXT_POST_PROCESSING       2

#define LL_TX_DATA_CONTEXT_SEND_DATA             0

#define LL_LINK_SETUP_TIMEOUT                    5  // 6 connection intervals (i.e. 0..5)

// Setup Next Slave Procedure Actions
#define LL_SETUP_NEXT_LINK_STATUS_SUCCESS        0
#define LL_SETUP_NEXT_LINK_STATUS_TERMINATE      1


// Data PDU Control Packet Types
#define LL_CTRL_CONNECTION_UPDATE_REQ            0  // M
#define LL_CTRL_CHANNEL_MAP_REQ                  1  // M
#define LL_CTRL_TERMINATE_IND                    2  // M, S
#define LL_CTRL_ENC_REQ                          3  // M
#define LL_CTRL_ENC_RSP                          4  //  , S
#define LL_CTRL_START_ENC_REQ                    5  //  , S
#define LL_CTRL_START_ENC_RSP                    6  // M, S
#define LL_CTRL_UNKNOWN_RSP                      7  // M, S
#define LL_CTRL_FEATURE_REQ                      8  // M   
#define LL_CTRL_FEATURE_RSP                      9  //  , S  , also could be M in ver4.2 ... HZF
#define LL_CTRL_PAUSE_ENC_REQ                    10 // M
#define LL_CTRL_PAUSE_ENC_RSP                    11 //  , S
#define LL_CTRL_VERSION_IND                      12 // M, S
#define LL_CTRL_REJECT_IND                       13 //  , S

// BLE 4.2
#define  LL_CTRL_SLAVE_FEATURE_REQ               14
#define  LL_CTRL_CONNECTION_PARAM_REQ            15
#define  LL_CTRL_CONNECTION_PARAM_RSP            16
#define  LL_CTRL_REJECT_EXT_IND                  17
#define  LL_CTRL_PING_REQ                        18
#define  LL_CTRL_PING_RSP                        19
#define  LL_CTRL_LENGTH_REQ                      20
#define  LL_CTRL_LENGTH_RSP                      21
// BLE 5.0
#define  LL_CTRL_PHY_REQ                         22
#define  LL_CTRL_PHY_RSP                         23
#define  LL_CTRL_PHY_UPDATE_IND                  24
#define  LL_CTRL_MIN_USED_CHANNELS_IND           25

#define LL_CTRL_TERMINATE_RX_WAIT_FOR_TX_ACK     26 // M (internal to LL only)

// control procedure timeout in coarse timer ticks
#define LL_MAX_CTRL_PROC_TIMEOUT                 64000 // 40s

// Encryption Related
#define LL_ENC_RAND_LEN                          8
#define LL_ENC_EDIV_LEN                          2
#define LL_ENC_LTK_LEN                           16
#define LL_ENC_IV_M_LEN                          4
#define LL_ENC_IV_S_LEN                          4
#define LL_ENC_IV_LINK_LEN                       4
#define LL_ENC_IV_LEN                            (LL_ENC_IV_M_LEN + LL_ENC_IV_S_LEN)
#define LL_ENC_SKD_M_LEN                         8
#define LL_ENC_SKD_S_LEN                         8
#define LL_ENC_SKD_LINK_LEN                      8
#define LL_ENC_SKD_LEN                           (LL_ENC_SKD_M_LEN + LL_ENC_SKD_S_LEN)
#define LL_ENC_SK_LEN                            16
#define LL_ENC_NONCE_LEN                         13
#define LL_END_NONCE_IV_OFFSET                   5
#define LL_ENC_MIC_LEN                           LL_PKT_MIC_LEN
//
#define LL_ENC_IV_M_OFFSET                       LL_ENC_IV_S_LEN
#define LL_ENC_IV_S_OFFSET                       0
#define LL_ENC_SKD_M_OFFSET                      LL_ENC_SKD_S_LEN
#define LL_ENC_SKD_S_OFFSET                      0
//
#define LL_ENC_BLOCK_LEN                         16
#define LL_ENC_CCM_BLOCK_LEN                     LL_ENC_BLOCK_LEN
#define LL_ENC_BLOCK_B0_FLAGS                    0x49
#define LL_ENC_BLOCK_A0_FLAGS                    0x01



////////////////////  for scan
// Scanner Advertisment Channels
#define LL_SCAN_ADV_CHAN_37                      37
#define LL_SCAN_ADV_CHAN_38                      38
#define LL_SCAN_ADV_CHAN_39                      39

struct bd_addr {
    uint8_t  addr[6];
};


typedef struct {
    uint8_t     peerAddrType;                      // peer device address type of public or random
    uint8_t     peerAddr[ 6 ];                     // peer device address
} peerInfo_t;

/// Advertising parameters
typedef struct {
    uint8_t       active;

    uint16_t  advInterval;                       // the advertiser interval, based on advIntMin and advIntMax
    /// Advertising type
    uint16_t advMode;                            // flag to indicate if currently advertising

    uint8_t ownAddrType;                         // own device address type of public or random
    uint8_t ownAddr[LL_DEVICE_ADDR_LEN];         // own device address

    uint8_t  advChanMap;                         // saved Adv channel map; note, only lower three bits used

    uint8_t       advEvtType;                    //connectable directed, undirected, discoverable, or non-connectable

    uint8_t       wlPolicy;                      // white list policy for Adv
    uint16_t      scaValue;                      // Slave SCA in PPM

    uint8_t       advDataLen;                    // advertiser data length

    // Scan Repsonse Parameters
    uint8_t       scanRspLen;                    // scan response data length

    // add by HZF
    uint8          advNextChan;

} advInfo_t;

///////////////////////////////////////////////////////////
// Scanner Event Parameters
typedef struct {
//  taskInfo_t *llTask;                                // pointer to associated task block
    uint8       ownAddrType;                             // own device address type of public or random
    uint8       ownAddr[ LL_DEVICE_ADDR_LEN ];           // own device address
    uint8       initPending;                           // flag to indicate if Scan needs to be initialized
    uint8       scanMode;                              // flag to indicate if currently scanning
    uint8       scanType;                              // passive or active scan
    uint16      scanInterval;                          // the interval between scan events
    uint16      scanWindow;                            // the duration of a scan event
    uint8       wlPolicy;                              // white list policy for Scan
    uint8       filterReports;                         // flag to indicate if duplicate Adv packet reports are to be filtered
    uint16      scanBackoffUL;                         // backoff upper limit count
    uint8       nextScanChan;                          // advertising channel to be used by scanner
    uint8       numSuccess;                            // for adjusting backoff count by tracking successive successes
    uint8       numFailure;                            // for adjusting backoff count by tracking successive failures
    uint16      currentBackoff;                        // current back off count, uint16 because the upper limit is 256
} scanInfo_t;

/////////////////// Initiator Event Parameters
typedef struct {
    uint8       ownAddrType;                             // own device address type of public or random
    uint8       ownAddr[ LL_DEVICE_ADDR_LEN ];           // own device address
    //
    uint8       initPending;                           // flag to indicate if Scan needs to be initialized
    uint8       scanMode;                              // flag to indicate if currently scanning
    uint16      scanInterval;                          // the interval between scan events
    uint16      scanWindow;                            // the duration of a scan event
    uint8       nextScanChan;                          // advertising channel to be used by scanner
    uint8       wlPolicy;                              // white list policy for Init
    uint8       connId;                                // allocated connection ID
    uint8       scaValue;                              // Master SCA as an ordinal value for PPM
} initInfo_t;
/////////////////////////////////////////////////////////////////

typedef struct {
    uint8_t  winSize;                              // window size
    uint16_t winOffset;                            // window offset
    uint16_t connInterval;                         // connection interval
    uint16_t slaveLatency;                         // number of connection events the slave can ignore
    uint16_t connTimeout;                          // supervision connection timeout
} connParam_t;

typedef struct {
    uint8_t  verNum;                               // controller spec version
    uint16_t comId;                                // company identifier
    uint16_t subverNum;                            // implementation version
} verInfo_t;

typedef struct {
    uint8_t connId;                                // connection ID
    uint8_t termIndRcvd;                           // indicates a TERMINATE_IND was received
    uint8_t reason;                                // reason code to return to Host when connection finally ends
} termInfo_t;

// TX Data
typedef struct txData_t {
    struct txData_t *pNext;                        // pointer to next Tx data entry on queue
} txData_t;

// Data Packet Queue
typedef struct {
    txData_t *head;                                // pointer to head of queue
    txData_t *tail;                                // pointer to tail of queue
} llDataQ_t;


// Version Information Exchange
typedef struct {
    uint8_t peerInfoValid;                         // flag to indicate the peer's version information is valid
    uint8_t hostRequest;                           // flag to indicate the host has requested the peer's version information
    uint8_t verInfoSent;                           // flag to indicate this device's version information has been sent
} verExchange_t;

// Feature Set Data
typedef struct {
    uint8_t featureRspRcved;                       // flag to indicate the Feature Request has been responded to
    uint8_t featureSet[ 8 ];
} featureSet_t;

// Channel Map
typedef struct {
    uint8_t chanMap[ 5 ];                          // bit map corresponding to the data channels 0..39
} chanMap_t;

// Control Procedure Information
typedef struct {
    uint8_t  ctrlPktActive;                              // flag that indicates a control packet is being processed
    uint8_t  ctrlPkts[ LL_MAX_NUM_CTRL_PROC_PKTS ];      // queue of control packets to be processed
    uint8_t  ctrlPktCount;                               // number of queued control packets
    uint16_t ctrlTimeoutVal;                             // timeout in CI events for control procedure for this connection
    uint16_t ctrlTimeout;                                // timeout counter in CI events for control procedure
} ctrlPktInfo_t;

// for timer drift adjust
typedef struct {
    uint32 coarse;                                       // number of 625us ticks at SFD capture
    uint16 fine;                                         // number of 31.25ns ticks at SFD capture
} sysTime_t;

// Encryption
typedef struct {
    // Note: IV and SKD provide enough room for the full IV and SKD. When the
    //       Master and Slave values are provided, the result is one combined
    //       (concatenated) value.
    uint8  IV[ LL_ENC_IV_LEN ];                        // combined master and slave IV values concatenated
    uint8  SKD [ LL_ENC_SKD_LEN ];                     // combined master and slave SKD values concatenated
    uint8  RAND[ LL_ENC_RAND_LEN ];                    // random vector from Master
    uint8  EDIV[ LL_ENC_EDIV_LEN ];                    // encrypted diversifier from Master
    uint8  nonce[ LL_ENC_NONCE_LEN ];                  // current nonce with current IV value
    uint8  SK[ LL_ENC_SK_LEN ];                        // session key derived from LTK and SKD
    uint8  LTK[ LL_ENC_LTK_LEN ];                      // Long Term Key from Host
    uint8  SKValid;                                    // flag that indicates the Session Key is valid
    uint8  LTKValid;                                   // Long Term Key is valid
    uint32 txPktCount;                                 // used for nonce formation during encryption (Note: 39 bits!)??
    uint32 rxPktCount;                                 // used for nonce formation during encryption (Note: 39 bits!)??
    uint8  encRestart;                                 // flag to indicate if an encryption key change took place
    uint8  encRejectErrCode;                           // error code for rejecting encryption request
    // ALT: COULD USE ONE VARIABLE AND STATES FOR THESE FLAGS; IF SO, THE
    //      CONTROL PROCEDURE WOULD NEED TO BE REWORKED.
    uint8  startEncRspRcved;                           // flag to indicate the Start Request has been responded to
    uint8  pauseEncRspRcved;                           // flag to indicate the Pause Request has been responded to
    uint8  encReqRcved;                                // flag to indicate an Enc Req was received in a Enc Pause procedure


    uint8  startEncReqRcved;                           // flag to indicate the Start Request has been responded to
    uint8  rejectIndRcved;                             // flag to indicate the Start Encryption needs to be aborted

} encInfo_t;

// Packet Error Rate Information - General
typedef struct {
    uint16 numPkts;                                    // total number of packets
    uint16 numCrcErr;                                  // total number of packets with CRC error
    uint16 numEvents;                                  // total number of connection events
    uint16 numMissedEvts;                              // total number of missed connection events
} perInfo_t;

// Connection Data
typedef struct {
    uint8_t          rx_timeout;                     // ----- Phyplus
    uint8_t          rx_crcok;                       // ----- Phyplus
    uint8_t          active;                         // flag to indicate if this connection is active
    uint8_t          connId;                         // connection ID
    uint8_t          firstPacket;                    // flag to indicate when the first packet has been received. 0 means TURE, 1 means FALSE

    uint16_t         currentEvent;                   // current event number
    uint16_t         nextEvent;                      // next active event number
    uint16_t         lastCurrentEvent;
    uint16_t         expirationEvent;                // event at which the LSTO has expired
    uint16_t         expirationValue;                // number of events to a LSTO expiration


    uint16_t         scaFactor;                      // SCA factor for timer drift calculation
    uint32_t         timerDrift;                     // saved timer drift adjustment to avoid recalc
    uint32_t         accuTimerDrift;                 // accumulate timer drift
    // Connection Parameters
    uint32_t         lastTimeToNextEvt;              // the time to next event from the previous connection event
    uint8_t          slaveLatencyAllowed;            // flag to indicate slave latency is permitted
    uint16_t         slaveLatency;                   // current slave latency; 0 means inactive
    uint8_t          lastSlaveLatency;               // last slave latency value used
    uint16_t         slaveLatencyValue;              // current slave latency value (when enabled)

    uint32_t accessAddr;                             // saved synchronization word to be used by Slave
    uint32_t initCRC;                                // connection CRC initialization value (24 bits)

    uint8_t          sleepClkAccuracy;               // peer's sleep clock accurracy; used by own device to determine timer drift
    connParam_t     curParam;

    // current connection parameters
    // Channel Map
    uint8_t          nextChan;                       // the channel for the next active connection event
    uint8_t          currentChan;                    // the channel for the currently completed connection event

    uint8_t          numUsedChans;                   // count of the number of usable data channels
    // uint8_t          hopLength;                   // used for finding next data channel at next connection event
    uint8_t          chanMapTable[LL_MAX_NUM_DATA_CHAN]; // current chan map table that is in use for this connection

    uint8_t  chanMap[5];

    uint8_t  hop;

    // TX Related
    uint8_t          txDataEnabled;                  // flag that indicates whether data output is allowed
    llDataQ_t      txDataQ;                          // queue of Tx Data packets
    // RX Related
    uint8_t          rxDataEnabled;                  // flag that indicates whether data input is allowed
    uint8_t          lastRssi;                       // last data packet RSSI received on this connection

    uint16_t        foff;                            // A2 add, sync qualitiy indicator, estimated by rx BB
    uint8_t         carrSens;                        // A2 add, estimated freq offset by rx BB ,foff-512-->[-512 511]KHz

    // Control Packet Information
    ctrlPktInfo_t  ctrlPktInfo;                      // information for control procedure processing
    // Parameter Update Control Procedure
    uint8_t          pendingParamUpdate;             // flag to indicate connection parameter update is pending
    uint16_t         paramUpdateEvent;               // event count to indicate when to apply pending param update
    connParam_t    paramUpdate;                      // update parameters
    // Channel Map Update Control Procedure
    uint8_t          pendingChanUpdate;              // flag to indicate connection channel map update is pending
    uint16         chanMapUpdateEvent;               // event count to indicate when to apply pending chan map update
    // Encryption Data Control Procedure
    uint8          encEnabled;                       // flag to indicate that encryption is enabled for this connection
    encInfo_t      encInfo;                          // structure that holds encryption related data
    // Feature Set
    featureSet_t   featureSetInfo;                   // feature set for this connection
    // Version Information
    verExchange_t  verExchange;                      // version information exchange
    verInfo_t      verInfo;                          // peer version information
    // Termination Control Procedure
    termInfo_t     termInfo;                         // structure that holds connection termination data
    // Unknnown Control Packet
    uint8          unknownCtrlType;                  // value of unknown control type
    // Packet Error Rate
    perInfo_t      perInfo;                          // PER

    uint8_t        isCollision;
    uint8_t        rejectOpCode;

    ll_phy_update_ind_t phyUpdateInfo;               // ll_phy update
    // Parameter Update Control Procedure
    uint8_t          pendingPhyModeUpdate;             // flag to indicate connection ll phy update is pending
    uint16_t         phyModeUpdateEvent;

    uint8_t  sn_nesn;                                // use to save last sn/nesn in new IC

    // for new IC
    uint8_t llMode;                                  // for RTLP & TRLP loop, may need change the HW engine mode.

    int  anchor_point_base_time;    // do we need it?
    int  anchor_point_fine_time;    // do we need it?

    int  next_event_base_time;      // do we need it?
    int  next_event_fine_time;      // do we need it?
} llConnState_t;

typedef struct {
    // adv channel statistics
    int ll_send_undirect_adv_cnt;
    int ll_send_nonconn_adv_cnt;
    int ll_send_scan_adv_cnt;
    int ll_send_hdc_dir_adv_cnt;
    int ll_send_ldc_dir_adv_cnt;

    // adv in conn event
    int ll_send_conn_adv_cnt;
    int ll_conn_adv_pending_cnt;

    // scan in conn event
    int ll_conn_scan_pending_cnt;

    // slave counter
    int ll_recv_scan_req_cnt;
    int ll_send_scan_rsp_cnt;
    int ll_recv_conn_req_cnt;

    // whitelist
    int ll_filter_scan_req_cnt;
    int ll_filter_conn_req_cnt;

    // scan
    int ll_recv_adv_pkt_cnt;
    int ll_recv_scan_rsp_cnt;


    // connection event counters
    int ll_conn_succ_cnt;        // LL accept connect, but not always sync succ

    int ll_link_lost_cnt;
    int ll_link_estab_fail_cnt;

    // connection packet statistics
    int ll_recv_ctrl_pkt_cnt;
    int ll_recv_data_pkt_cnt;
    int ll_recv_invalid_pkt_cnt;

    int ll_recv_abnormal_cnt;

    int ll_send_data_pkt_cnt;

    int ll_conn_event_cnt;
    int ll_recv_crcerr_event_cnt;              // CRC error detected in the connection event
    int ll_conn_event_timeout_cnt;             // timeout connection event countt

    int ll_rx_peer_cnt;                        // scan/conn request counter, to consider whether we need it

    // LL <-> HCI packets statistics
    int ll_to_hci_pkt_cnt;
    int ll_hci_to_ll_pkt_cnt;

    int ll_hci_buffer_alloc_err_cnt;

    //ll_hw err cnt
    int ll_trigger_err;
    int ll_rfifo_rst_err;
    int ll_rfifo_rst_cnt;
    int ll_rfifo_read_err;

    // reserve counter
    int ll_tbd_cnt1;
    int ll_tbd_cnt2;
    int ll_tbd_cnt3;
    int ll_tbd_cnt4;

} llGlobalStatistics_t;

// Counters
typedef struct {
    uint8 numTxDone;                                   // TX pkts ACK'ed (auto-empty not counted)
    uint8 numTxAck;                                    // TX pkts ACK'ed (both auto-empty and TX FIFO packets)
    uint8 numTxCtrlAck;                                // TX control pkts ACK'ed
    uint8 numTxCtrl;                                   // TX control pkts TX'ed
    uint8 numTxRetrans;                                // retrans + auto-empty retrans
    uint8 numTx;                                       // trans (incl. auto-empty) + retrans (incl. auto-empty)
    uint8 numRxOk;                                     // non-empty correctly RX'ed and not ignored data and control pkts
    uint8 numRxCtrl;                                   // correctly RX'ed control pkts
    uint8 numRxNotOk;                                  // RX'ed with bad CRC
    uint8 numRxIgnored;                                // correctly RX'ed, but ignored
    uint8 numRxEmpty;                                  // correctly RX'ed empty packets
    uint8 numRxFifoFull;                               // correctly RX'ed but discarded due to full RX FIFO
} rfCounters_t;


// global variables
extern uint8_t     		   LL_TaskID;
extern uint8_t             llState;
extern uint8_t             connId;
extern peerInfo_t          peerInfo;
extern advInfo_t           adv_param;
extern scanInfo_t          scanInfo;                        // scan data
extern initInfo_t          initInfo;                        // Initiator info
extern llConnState_t       conn_param[];
extern chanMap_t           chanMapUpdate;
extern featureSet_t        deviceFeatureSet;

extern uint8               numComplPkts;
extern uint8               numComplPktsLimit;

extern verInfo_t         verInfo;

extern  rfCounters_t  rfCounters;

extern llLinkBuf_t  ll_buf;

extern llGlobalStatistics_t g_pmCounters;

extern llPduLenManagment_t g_llPduLen;
extern llPhyModeManagment_t g_llPhyModeCtrl;

#endif














