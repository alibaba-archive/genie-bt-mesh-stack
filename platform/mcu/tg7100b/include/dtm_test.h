/**************************************************************************************************

  Phyplus Microelectronics Limited confidential and proprietary.
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics
  Limited ("Phyplus"). Your use of this Software is limited to those
  specific rights granted under  the terms of the business contract, the
  confidential agreement, the non-disclosure agreement and any other forms
  of agreements as a customer or a partner of Phyplus. You may not use this
  Software unless you agree to abide by the terms of these agreements.
  You acknowledge that the Software may not be modified, copied,
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
  (BLE) integrated circuit, either as a product or is integrated into your
  products.  Other than for the aforementioned purposes, you may not use,
  reproduce, copy, prepare derivative works of, modify, distribute, perform,
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

#ifndef __DTM_TEST_H_
#define __DTM_TEST_H_

typedef void (*dtm_event_func_t)(unsigned char data);

/**************************************************************************************
 * @fn          rf_phy_direct_print
 *
 * @brief       This function process for rf phy config print
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_direct_print(void);


/**************************************************************************************
 * @fn          rf_phy_sleep
 *
 * @brief       This function process for rf phy enter sleep
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_sleep(void);


/**************************************************************************************
 * @fn          rf_phy_dtm_init
 *
 * @brief       This function process for rf phy init for dtm test
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_dtm_init(dtm_event_func_t event_cb);


/**************************************************************************************
 * @fn          rf_phy_dtm_test_tx_mod_burst
 *
 * @brief       This function process for rf phy tx mode burst test
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_dtm_test_tx_mod_burst(void);

/**************************************************************************************
 * @fn          rf_phy_dtm_cmd_parse
 *
 * @brief       This function process for rf phy direct test,cmd parse
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_dtm_cmd_parse(void);


/**************************************************************************************
 * @fn          rf_phy_dtm_test_evt_send
 *
 * @brief       This function process for rf phy direct test, test mode trigged
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return
 *    -1: event_callback not register
 *     0: Success get event
 */
uint8_t rf_phy_dtm_test_evt_send(void);


/**************************************************************************************
 * @fn          rf_phy_dtm_trigged
 *
 * @brief       This function process for rf phy direct test, test mode trigged
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_dtm_trigged(void);


/**************************************************************************************
 * @fn          rf_phy_dtm_reset_pkg_count
 *
 * @brief       This function process for rf phy reset package counter in tx burst test
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_dtm_reset_pkg_count(void);

/**************************************************************************************
 * @fn          rf_phy_get_dtm_cmd
 *
 * @brief       This function process for rf phy get dtm cmd from buffer
 *
 * input parameters
 *
 * @param
 *     char dut_data[]  - data from uart, we need get dtm cmd from the data
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void rf_phy_set_dtm_cmd(char dut_data[]);

/**************************************************************************************
 * @fn          rf_phy_dtm_ext_tx_singleTone
 *
 * @brief       This function process for rf phy direct test, test mode interup
 *
 * input parameters
 *
 * @param       txPower     :   rf tx power
 *              rfChnIdx    :   rf channel = 2402+(rfChnIdx<<1)
 *              rfFoff      :   rf freq offset = rfFoff*4KHz
 *              testTimeUs  :   test loop active time(ms)
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void    rf_phy_dtm_ext_tx_singleTone(uint8_t txPower, uint8_t rfChnIdx, int8_t rfFoff, uint32 testTimeUs);

/**************************************************************************************
 * @fn          rf_phy_dtm_ext_tx_modulation
 *
 * @brief       This function process for rf phy direct test, test mode interup
 *
 * input parameters
 *
 * @param       txPower     :   rf tx power
 *              rfChnIdx    :   rf channel = 2402+(rfChnIdx<<1)
 *              rfFoff      :   rf freq offset = rfFoff*4KHz
 *              pktType     :   modulaiton data type, 0: prbs9, 1: 1111000: 2 10101010
 *              testTimeUs  :   test loop active time(ms)
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void    rf_phy_dtm_ext_tx_modulation(uint8_t txPower, uint8_t rfChnIdx, int8_t rfFoff, uint8_t pktType, uint32 testTimeUs);

/**************************************************************************************
 * @fn          rf_phy_dtm_ext_tx_mt_burst
 *
 * @brief       This function process for rf phy direct test, test mode interup
 *
 * input parameters
 *
 * @param       txPower     :   rf tx power
 *              rfChnIdx    :   rf channel = 2402+(rfChnIdx<<1)
 *              rfFoff      :   rf freq offset = rfFoff*4KHz
 *              pktType     :   modulaiton data type, 0: prbs9, 1: 1111000: 2 10101010
 *              pktLength   :   pkt length(Byte)
 *
 *              txPktNum    :   burst pkt tx number
 *              txPktIntv   :   txPkt intv,0 txPkt intv is pkt interval =  ceil((L+249)/625) * 625
 *
 * output parameters
 *
 * @param       none
 *
 * @return      none
 */
void    rf_phy_dtm_ext_tx_mod_burst(uint8_t txPower, uint8_t rfChnIdx, int8_t rfFoff,
                                    uint8_t pktType, uint8_t pktLength, uint32 txPktNum, uint32 txPktIntv);

/**************************************************************************************
 * @fn          rf_phy_dtm_ext_rx_demod_burst
 *
 * @brief       This function process for rf phy direct test, test mode interup
 *
 * input parameters
 *
 * @param       rfChnIdx        :   rf channel = 2402+(rfChnIdx<<1)
 *              rfFoff          :   rf freq offset = rfFoff*4KHz
 *              pktLength       :   pkt length(Byte)
 *              rxWindow        :   rx demod window length(us)
 *              rxTimeOut       :   rx on time (ms)
 *
 * output parameters
 *
 * @param       rxEstFoff       :   rx demod estimated frequency offset
 *              rxEstRssi       :   rx demod estimated rssi
 *              rxEstCarrSens   :   rx demod estimated carrier sense
 *              rxPktNum        :   rx demod received pkt number
 *
 * @return      none
 */
void    rf_phy_dtm_ext_rx_demod_burst(uint8_t rfChnIdx, int8_t rfFoff, uint8_t pktLength, uint32 rxTimeOut, uint32 rxWindow,
                                      uint16_t *rxEstFoff, uint8_t *rxEstRssi, uint8_t *rxEstCarrSens, uint16_t *rxPktNum);

/**************************************************************************************
* @fn          rf_phy_dtm_ext_acc_code_set
*
* @brief       config the acc_code in rf phy dtm
*
* input parameters
*
* @param       acc_code        :   sync word
*
* output parameters
* @return      none
*/
void    rf_phy_dtm_ext_acc_code_set(uint32 accCode);

void rf_phy_set_dtmModeType(uint8_t type);

void rf_phy_set_dtmChan(uint8_t dtmChan);

void rf_phy_set_dtmFreqOffSet(uint8_t FreqOffSet);

void rf_phy_set_dtmLength(uint8_t dtmLength);

void rf_phy_dtm_stop(void);

#endif
