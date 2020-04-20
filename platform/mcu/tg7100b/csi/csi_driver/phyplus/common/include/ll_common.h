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



#ifndef _LL_H_
#define _LL_H_

#include "types.h"
#include "ll.h"
#include "ll_def.h"
#include "common.h"

#define LL_DATA_PDU( pktHdr )     ((pktHdr) != LL_DATA_PDU_HDR_LLID_CONTROL_PKT)
#define LL_CTRL_PDU( pktHdr )     ((pktHdr) == LL_DATA_PDU_HDR_LLID_CONTROL_PKT)
#define LL_INVALID_LLID( pktHdr ) ((pktHdr) == LL_DATA_PDU_HDR_LLID_RESERVED)


void LL_IRQHandler(void);

void move_to_slave_function(void);

void LL_slave_conn_event(void);

void LL_master_conn_event(void);

void LL_set_default_conn_params(void);

//void ll_setMem(uint8_t  *buf, uint8_t v, int n);

//void ll_cpyMem(uint8_t *dst, uint8_t *src, int n);

void LL_evt_schedule(void);

llStatus_t llSetupAdv(void);

void llConvertLstoToEvent(llConnState_t *connPtr,
                          connParam_t   *connParams);

void llSlaveEvt_TaskEndOk(void);

// for master process
void llMasterEvt_TaskEndOk(void);

// Connection Management
extern llConnState_t      *llAllocConnId(void);
extern void               llReleaseConnId(llConnState_t *connPtr);
extern void               llReleaseAllConnId(void);
extern uint16             llGetMinCI(uint16 connInterval);
extern uint8              llGetNextConn(void);
extern void               llConnCleanup(llConnState_t *connPtr);
extern void               llConnTerminate(llConnState_t *connPtr, uint8 reason);
extern uint8              llPendingUpdateParam(void);
extern void               llInitFeatureSet(void);
extern uint32             llGenerateValidAccessAddr(void);
extern uint32             llGenerateCRC(void);
extern  uint8             llEventInRange(uint16 curEvent, uint16 nextEvent, uint16 updateEvent);
extern  uint16            llEventDelta(uint16 eventA, uint16 eventB);
extern void               llConvertLstoToEvent(llConnState_t *connPtr, connParam_t *connParams);
extern void               llConvertCtrlProcTimeoutToEvent(llConnState_t *connPtr);
extern uint8              llAdjustForMissedEvent(llConnState_t *connPtr, uint32 timeToNextEvent);

// Task Setup
extern llStatus_t         llSetupAdv(void);
extern void               llSetupDirectedAdvEvt(void);
extern void               llSetupUndirectedAdvEvt(void);
extern void               llSetupNonConnectableAdvEvt(void);
extern void               llSetupScannableAdvEvt(void);
extern void               llSetupScan(uint8 chan);
extern void               llSetupScanInit(void);
extern void               llSetupInit(uint8 connId);
extern void               llSetupConn(void);
// A2 added
extern uint8              llSetupSecNonConnectableAdvEvt(void);
extern void               llSetupSecScan(uint8 chan);
extern uint32             llCalcMaxScanTime(void);
extern uint8              llSecAdvAllow(void);






// Data Management
extern uint8              llEnqueueDataQ(llDataQ_t *pDataQ, txData_t *pTxData);
extern uint8              llEnqueueHeadDataQ(llDataQ_t *pDataQ, txData_t *pTxData);
extern txData_t          *llDequeueDataQ(llDataQ_t *pDataQ);
extern uint8              llDataQEmpty(llDataQ_t *pDataQ);
extern uint8              llWriteTxData(llConnState_t *connPtr, uint8 pktHdr, uint8 pktLen, uint8 *pBuf);
extern uint8              *llMemCopySrc(uint8 *pDst, uint8 *pSrc, uint8 len);
extern uint8              *llMemCopyDst(uint8 *pDst, uint8 *pSrc, uint8 len);
extern void               llProcessMasterControlPacket(llConnState_t *connPtr, uint8 *pBuf);
extern void               llProcessSlaveControlPacket(llConnState_t *connPtr, uint8 *pBuf);
extern  void              llProcessTxData(llConnState_t *connPtr, uint8 context);
extern  uint8             llProcessRxData(void);

// Control Procedure Setup
extern uint8              llSetupUpdateParamReq(llConnState_t *connPtr);    // M
extern uint8              llSetupUpdateChanReq(llConnState_t *connPtr);     // M
extern uint8              llSetupEncReq(llConnState_t *connPtr);            // M
extern uint8              llSetupEncRsp(llConnState_t *connPtr);            // S
extern uint8              llSetupStartEncReq(llConnState_t *connPtr);       // S
extern uint8              llSetupStartEncRsp(llConnState_t *connPtr);       // M, S
extern uint8              llSetupPauseEncReq(llConnState_t *connPtr);       // M
extern uint8              llSetupPauseEncRsp(llConnState_t *connPtr);       // S
extern uint8              llSetupRejectInd(llConnState_t *connPtr, uint8 errCode);        // S
extern uint8              llSetupFeatureSetReq(llConnState_t *connPtr);     // M, S
extern uint8              llSetupFeatureSetRsp(llConnState_t *connPtr);     // M, S
extern uint8              llSetupVersionIndReq(llConnState_t *connPtr);     // M
extern uint8              llSetupTermInd(llConnState_t *connPtr);           // M, S
extern uint8              llSetupUnknownRsp(llConnState_t *connPtr);        // M, S

extern uint8              llSetupDataLenghtReq(llConnState_t *connPtr);  //M,S
extern uint8              llSetupDataLenghtRsp(llConnState_t *connPtr);  //M,S
extern uint8              llSetupPhyReq(llConnState_t *connPtr);            //M,S
extern uint8              llSetupPhyRsp(llConnState_t *connPtr);     //M,S
extern uint8              llSetupPhyUpdateInd(llConnState_t *connPtr);  //M
extern uint8              llSetupRejectExtInd(llConnState_t *connPtr, uint8 errCode);

// Control Procedure Management
extern void               llEnqueueCtrlPkt(llConnState_t *connPtr, uint8 ctrlType);
extern void               llDequeueCtrlPkt(llConnState_t *connPtr);
extern void               llReplaceCtrlPkt(llConnState_t *connPtr, uint8 ctrlType);


// Data Channel Management
extern void               llProcessChanMap(llConnState_t *connPtr, uint8 *chanMap);
extern  uint8    llGetNextDataChan(llConnState_t *connPtr, uint16 numEvents);
extern  void     llSetNextDataChan(llConnState_t *connPtr);
extern uint8              llAtLeastTwoChans(uint8 *chanMap);


uint8_t llTimeCompare(int base_time, int fine_time);
uint32_t calculateTimeDelta(int base_time, int  fine_time);

void llSetNextDataChan(llConnState_t *connPtr);

// White List Related
extern llStatus_t         llCheckWhiteListUsage(void);

// function add by HZF
void llResetConnId(void);
void llResetRfCounters(void);
extern void               llInitFeatureSet(void);


extern  uint16 llCalcScaFactor(uint8 masterSCA);


extern void llCalcTimerDrift(uint32    connInterval,
                             uint16    slaveLatency,
                             uint8     sleepClkAccuracy,
                             uint32 *timerDrift);


// add by HZF
uint8 llGetNextAdvChn(uint8 cur_chn);

// Tx loop buffer process
void update_tx_write_ptr(void);

void update_tx_read_ptr(void);

uint8_t getTxBufferSize(void);
uint8_t getTxBufferFree(void);

uint8_t get_tx_read_ptr(void);

uint8_t get_tx_write_ptr(void);

// Rx loop buffer process
void update_rx_write_ptr(void);

void update_rx_read_ptr(void);

uint8_t getRxBufferSize(void);
uint8_t getRxBufferFree(void);

uint8_t get_rx_read_ptr(void);

uint8_t get_rx_write_ptr(void);

// reset buffer
void reset_conn_buf(void);

void ll_schedule_next_event(int time);

uint16 ll_generateTxBuffer(int txFifo_vacancy, uint16 *pSave_ptr);

void ll_read_rxfifo(void);
void ll_hw_read_tfifo_rtlp(void);
int ll_hw_read_tfifo_packet(uint8 *pkt);

// function in ll_slaveEndCause.c
uint8 llSetupNextSlaveEvent(void);
uint8 llProcessSlaveControlProcedures(llConnState_t *connPtr);
uint8 llCheckForLstoDuringSL(llConnState_t *connPtr);

// function in ll_hwItf.c
void ll_hw_process_RTO(uint32 ack_num);
void ll_debug_output(uint32 state);

void llAdjSlaveLatencyValue(llConnState_t *connPtr);

//function for DLE add by ZQ
void llPduLengthManagmentReset(void);
void llTrxNumAdaptiveConfig(void);
void llPduLengthUpdate(uint16 connHandle);
uint8 LL_PLUS_GetLocalPduDataLength(ll_pdu_length_ctrl_t *pduLen);

//function for PHY UPDATE add by ZQ
void llPhyModeCtrlReset(void);
void llPhyModeCtrlUpdateNotify(llConnState_t *connPtr, uint8 status);
llStatus_t LL_PLUS_GetLocalPhyMode(ll_phy_ctrl_t *phyCtrl);
void llSetNextPhyMode(llConnState_t *connPtr);
extern void llInitFeatureSetDLE(uint8 enable);
extern void llInitFeatureSet2MPHY(uint8 enable);
extern void llInitFeatureSetCodedPHY(uint8 enable);

#endif

