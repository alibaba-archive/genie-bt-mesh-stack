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



/*******************************************************************************
* @file		rf_phy_driver.h
* @brief	Contains all functions support for PHYPLUS RF_PHY_DRIVER
* @version	1.0
* @date		24. Aug. 2017
* @author	Zhongqi Yang
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __RF_PHY_DRIVER_H_
#define __RF_PHY_DRIVER_H_




/*******************************************************************************
 * INCLUDES
 */


#include "ll_hw_drv.h"
#include "common.h"
#include "jump_function.h"




/*******************************************************************************
 * Global Var
 */
extern volatile uint8_t g_rfPhyTpCal0;            //** two point calibraion result0            **//
extern volatile uint8_t g_rfPhyTpCal1;            //** two point calibraion result1            **//
extern volatile uint8_t g_rfPhyTpCal0_2Mbps;            //** two point calibraion result0            **//
extern volatile uint8_t g_rfPhyTpCal1_2Mbps;            //** two point calibraion result1            **//
extern volatile uint8_t g_rfPhyTxPower;           //** rf pa output power setting [0x00 0x1f]  **//
extern volatile uint8_t g_rfPhyPktFmt;            //** rf_phy pkt format config                **//
extern volatile uint32  g_rfPhyRxDcIQ;            //** rx dc offset cal result                 **//
extern volatile uint8_t g_rfPhyXtalCap;           //** 16M Xtal trim cap for RF FreqOffset     **//
extern volatile int8_t  g_rfPhyFreqOffSet;

extern volatile uint8_t g_system_clk;
/*******************************************************************************
 * MACRO
 */


#define PHY_REG_RD(x)                               *(volatile uint32_t *)(x)
#define PHY_REG_WT(x,y)                             *(volatile uint32_t *)(x) = (y)
#define RF_CHN_TO_FREQ(x)
#define DCDC_CONFIG_SETTING(x)                      subWriteReg(0x4000f014,18,15, x)

#define GET_UART_WIDX                               *(volatile uint32_t *)(0x1fff1054)
#define CLR_UART_WIDX                               *(volatile uint32_t *)(0x1fff1054)=0

#define RF_PHY_TPCAL_CALC(tp0,tp1,chn)              ((tp0)>(tp1) ?(((tp0<<5)-(tp0-tp1)*(chn)+16)>>5) : tp0 )
//DTM STATE
#define RF_PHY_DTM_IDL                              0
#define RF_PHY_DTM_CMD                              1
#define RF_PHY_DTM_EVT                              2
#define RF_PHY_DTM_TEST                             3

#define RF_PHY_DTM_SYNC_WORD                        0x71764129
#define RF_PHY_DTM_PRBS9_SEED                       0xffffffff
#define RF_PHY_DTM_CRC_WT                           0x00555555

//DTM MODE TYPE
#define RF_PHY_DTM_MODE_RESET                       0
#define RF_PHY_DTM_MODE_TX_BURST                    2
#define RF_PHY_DTM_MODE_TX_CTMOD                    4
#define RF_PHY_DTM_MODE_TX_SINGLE                   6
#define RF_PHY_DTM_MODE_RX_PER                      8
#define RF_PHY_DTM_MODE_TEST_END                    10
#define RF_PHY_DTM_MODE_SET_LENGTH_UP2BIT           12

#define RF_PHY_DTM_MODE_SET_PHY_1M                  16
#define RF_PHY_DTM_MODE_SET_PHY_2M                  18
#define RF_PHY_DTM_MODE_SET_PHY_500K                20
#define RF_PHY_DTM_MODE_SET_PHY_125K                22
#define RF_PHY_DTM_MODE_SET_PHY_ZB                  24

#define RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STANDARD 32
#define RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STABLE   34
#define RF_PHY_DTM_MODE_READ_SUPPORTED_TEST_CASE    36
#define RF_PHY_DTM_MODE_READ_MAX_TX_OCTETS          38
#define RF_PHY_DTM_MODE_READ_MAX_TX_TIME            40
#define RF_PHY_DTM_MODE_READ_MAX_RX_OCTETS          42
#define RF_PHY_DTM_MODE_READ_MAX_RX_TIME            44

#define RF_PHY_DTM_MODE_SET_ACCCODE_0               114
#define RF_PHY_DTM_MODE_SET_ACCCODE_1               116
#define RF_PHY_DTM_MODE_SET_ACCCODE_2               118
#define RF_PHY_DTM_MODE_SET_ACCCODE_3               120

#define RF_PHY_DTM_MODE_SET_FREQ_FOFF               122
#define RF_PHY_DTM_MODE_SET_TPCAL_MANUAL            124
#define RF_PHY_DTM_MODE_SET_XTAL_CAP                126
#define RF_PHY_DTM_MODE_SET_TX_POWER                128
#define RF_PHY_DTM_MODE_GET_FOFF                    130
#define RF_PHY_DTM_MODE_GET_TPCAL                   132
#define RF_PHY_DTM_MODE_GET_RSSI                    134
#define RF_PHY_DTM_MODE_GET_CARR_SENS               136
#define RF_PHY_DTM_MODE_GET_PER_AUTO                138

#define RF_PHY_DTM_MODE_ATE_SET_PKTFMT              0xd0 //208
#define RF_PHY_DTM_MODE_ATE_SET_TXPOWER             0xd1

#define RF_PHY_DTM_MODE_ATE_TX_BURST                0xe0 //224
#define RF_PHY_DTM_MODE_ATE_TX_MOD                  0xe1
#define RF_PHY_DTM_MODE_ATE_TX_CARR                 0xe2
#define RF_PHY_DTM_MODE_ATE_RX_AUTOGAIN             0xe3
#define RF_PHY_DTM_MODE_ATE_RX_FIXGAIN              0xe4
#define RF_PHY_DTM_MODE_ATE_RX_DEMOD                0xe5
#define RF_PHY_DTM_MODE_ATE_RX2TX                   0xe6
#define RF_PHY_DTM_MODE_ATE_TX2RX                   0xe7

#define RF_PHY_DTM_MODE_ATE_RESET                   0xef

#define RF_PHY_DTM_MODE_ERROR                       254


/*******************************************************************************
 * CONSTANTS
 */
#define PKT_FMT_ZIGBEE                              0
#define PKT_FMT_BLE1M                               1
#define PKT_FMT_BLE2M                               2
#define PKT_FMT_BLR500K                             3
#define PKT_FMT_BLR125K                             4

#define RF_PHY_TX_POWER_EXTRA_MAX                   0x3f
#define RF_PHY_TX_POWER_MAX                         0x1f
#define RF_PHY_TX_POWER_MIN                         0x00

#define RF_PHY_TX_POWER_5DBM                        0x17
#define RF_PHY_TX_POWER_4DBM                        0x12
#define RF_PHY_TX_POWER_3DBM                        0x0f
#define RF_PHY_TX_POWER_0DBM                        0x0a

#define RF_PHY_TX_POWER_N3DBM                       0x06
#define RF_PHY_TX_POWER_N5DBM                       0x05
#define RF_PHY_TX_POWER_N6DBM                       0x04

#define RF_PHY_TX_POWER_N10DBM                      0x03
#define RF_PHY_TX_POWER_N15DBM                      0x02
#define RF_PHY_TX_POWER_N20DBM                      0x01

#define RF_PHY_FREQ_FOFF_00KHZ                      0
#define RF_PHY_FREQ_FOFF_20KHZ                      5
#define RF_PHY_FREQ_FOFF_40KHZ                      10
#define RF_PHY_FREQ_FOFF_60KHZ                      15
#define RF_PHY_FREQ_FOFF_80KHZ                      20
#define RF_PHY_FREQ_FOFF_100KHZ                     25
#define RF_PHY_FREQ_FOFF_120KHZ                     30
#define RF_PHY_FREQ_FOFF_140KHZ                     35
#define RF_PHY_FREQ_FOFF_160KHZ                     40
#define RF_PHY_FREQ_FOFF_180KHZ                     45
#define RF_PHY_FREQ_FOFF_200KHZ                     50
#define RF_PHY_FREQ_FOFF_N20KHZ                     -5
#define RF_PHY_FREQ_FOFF_N40KHZ                     -10
#define RF_PHY_FREQ_FOFF_N60KHZ                     -15
#define RF_PHY_FREQ_FOFF_N80KHZ                     -20
#define RF_PHY_FREQ_FOFF_N100KHZ                    -25
#define RF_PHY_FREQ_FOFF_N120KHZ                    -30
#define RF_PHY_FREQ_FOFF_N140KHZ                    -35
#define RF_PHY_FREQ_FOFF_N160KHZ                    -40
#define RF_PHY_FREQ_FOFF_N180KHZ                    -45
#define RF_PHY_FREQ_FOFF_N200KHZ                    -50



/*******************************************************************************
 * FUNCION DEFINE
 */
void        rf_phy_ini(void);
void        rf_phy_ana_cfg(void);
void        rf_phy_bb_cfg(uint8_t pktFmt);
void        rf_phy_change_cfg(uint8_t pktFmt);
void        rf_tpCal_cfg(uint8_t rfChn);
uint8_t     rf_tp_cal(uint8_t rfChn, uint8_t fDev);
void        rf_rxDcoc_cfg(uint8_t rfChn, uint8_t bwSet, volatile uint32 *dcCal);
void        rf_tpCal_gen_cap_arrary(void);

void        rf_phy_direct_test(void);
void        rf_phy_dtm_cmd_parse(void);
void        rf_phy_dtm_evt_send(uint8_t dtmType);
void        rf_phy_dtm_trigged(void);
void        rf_phy_get_pktFoot(uint8 *rssi, uint16 *foff, uint8 *carrSens);

void        rf_phy_set_txPower(uint8 txPower);

uint8_t     rf_phy_direct_test_ate(uint32_t cmdWord, uint8_t regPatchNum, uint32_t *regPatchAddr, uint32_t *regPatchVal, uint8_t *dOut);

void        rf_phy_dtm_zigbee_pkt_gen(void);

#endif
