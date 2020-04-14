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
* @file		spi.h
* @brief	Contains all functions support for spi driver
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef _SPI_H_
#define _SPI_H_

#include "types.h"
#include "gpio.h"
#include "ap_cp.h"
#include "gpio.h"
#include "common.h"

#define  SPI_MASTER_MODE      1    //define master mode,1:master mode,0:salve mode

#define  SPI_USE_TIMEOUT 0
#define  SPI_OP_TIMEOUT  100        //100ms for an Byte operation
#if(SPI_USE_TIMEOUT == 1)
#define SPI_INIT_TOUT(to)               int to = hal_systick()
#define SPI_CHECK_TOUT(to, timeout)   (hal_ms_intv(to)> timeout?true:false)
#else
#define SPI_INIT_TOUT(to)
#define SPI_CHECK_TOUT(to, timeout)     false
#endif


#define  ENABLE_SPI           Ssix->SSIEN = 1
#define  DISABLE_SPI          Ssix->SSIEN = 0
#define  SPI_BUSY             Ssix->SR & 0x1
#define  RX_FIFO_NOT_EMPTY    Ssix->SR & 0x8
#define  TX_FIFO_EMPTY        Ssix->SR & 0x4
#define  TX_FIFO_NOT_FULL     Ssix->SR & 0x2
#define  NUMBER_DATA_RX_FIFO  Ssix->RXFLR
#define  NUMBER_DATA_TX_FIFO  Ssix->TXFLR
#define  SPI_DATA             Ssix->DataReg[0]


typedef enum {
    SPI_MODE0 = 0,    //SCPOL=0,SCPH=0
    SPI_MODE1,        //SCPOL=0,SCPH=1
    SPI_MODE2,        //SCPOL=1,SCPH=0
    SPI_MODE3,        //SCPOL=1,SCPH=1
} SPI_SCMOD_e;

typedef enum {
    SPI_TRXD = 0,      //Transmit & Receive
    SPI_TXD,         //Transmit Only
    SPI_RXD,         //Receive Only
    SPI_EEPROM,      //EEPROM Read
} SPI_TMOD_e;

typedef enum {
    SPI0 = 0,        //use spi 0
    SPI1,          //use spi 1
} SPI_INDEX_e;

typedef enum {
    DFS_4BIT = 0x3,
    DFS_5BIT,
    DFS_6BIT,
    DFS_7BIT,
    DFS_8BIT,
    DFS_9BIT,
    DFS_10BIT,
    DFS_11BIT,
    DFS_12BIT,
    DFS_13BIT,
    DFS_14BIT,
    DFS_15BIT,
    DFS_16BIT,
} DATA_FRAME_SIZE_e;

typedef enum {
    TRANSMIT_COMPLETED = 0,
} spi_Type_t;

typedef struct _spi_Evt_t {
    uint8_t spi;
    spi_Type_t  type;
} spi_Evt_t;

typedef void (*spi_Hdl_t)(spi_Evt_t evt);

typedef struct _spi_Cfg_t {
    GPIO_Pin_e    sclk_pin;
    GPIO_Pin_e    ssn_pin;
    GPIO_Pin_e    MOSI;
    GPIO_Pin_e    MISO;
    uint32_t      baudrate;
    SPI_TMOD_e    spi_tmod;
    SPI_SCMOD_e   spi_scmod;
    bool int_mode;
    bool force_cs;
    spi_Hdl_t  evt_handler;
} spi_Cfg_t;

typedef enum {
    TRANSMIT_FIFO_EMPTY = 0x01,
    TRANSMIT_FIFO_OVERFLOW = 0x02,
    RECEIVE_FIFO_UNDERFLOW = 0x04,
    RECEIVE_FIFO_OVERFLOW = 0x08,
    RECEIVE_FIFO_FULL = 0x10,
} SPI_INT_STATUS_e;

typedef struct _hal_spi_t {
    SPI_INDEX_e spi_index;
} hal_spi_t;

typedef struct {
    bool idle;
    uint16_t tx_rx_len;
    uint16_t tx_index;
    uint16_t rx_index;
    uint16_t tx_buf_len;
    uint8_t *tx_ptr;
    uint8_t *rx_ptr;
} spi_data_t;

/**************************************************************************************
 * @fn          hal_SPI0_IRQHandler
 *
 * @brief       This function process for spi0 interrupt,when use int please consummate its callbackfunction
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((weak)) hal_SPI0_IRQHandler(void);

/**************************************************************************************
 * @fn          hal_SPI1_IRQHandler
 *
 * @brief       This function process for spi1 interrupt,when use int please consummate its callbackfunction
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((weak)) hal_SPI1_IRQHandler(void);

/**************************************************************************************
 * @fn          hal_spi_init
 *
 * @brief       This function will config the spi module.
 *
 * input parameters
 *
 * @param       	spi_Cfg_t cfg:there are two parameter you need config to use spi,one is cfg,another is the macro SPI_MASTER_MODE
 *                refer to struct spi_Cfg_t,you will config it right.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return
 * 							PPlus_SUCCESS:config success.
 * 							PPlus_ERR_BUSY:the spi you want to use is work,please use another or stop it and reuse it.
 * 							PPlus_ERR_INVALID_PARAM:there are two spi in mcu,you parameter is invalid.
 **************************************************************************************/
int hal_spi_bus_init(hal_spi_t *spi_ptr, spi_Cfg_t cfg);

/**************************************************************************************
 * @fn          hal_spi_deinit
 *
 * @brief       This function will deinit the spi you select.
 *
 * input parameters
 *
 * @param       	hal_spi_t* spi_ptr: spi module handle.

 *
 * output parameters
 *
 * @param       None.
 *
 * @return
 *              PPlus_SUCCESS
 *              PPlus_ERR_INVALID_PARAM
 **************************************************************************************/
int hal_spi_bus_deinit(hal_spi_t *spi_ptr);

/**************************************************************************************
 * @fn          hal_spi_init
 *
 * @brief       it is used to init spi module.
 *
 * input parameters
 * @param       None
 *
 * output parameters
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_spi_init(void);

/**************************************************************************************
 * @fn          hal_spi_transmit
 *
 * @brief       transmit data
 * please care polling mode or int mode,force cs manually or not.
 *
 * polling mode:this function will return just after all the transmit has been finished.
 *
 * int mode:the function just start the tranmsit,when tranmit finish the idle flag will set true in int.
 *              so after call this api,you can use a callback function to tell transmit result.
 *
 * force_cs=true:data will transmit in one time.
 *               cs is valid in the transmit.
 *
 * force_cs=false:data will transmit in several times.one time transmit one frame.frame length is 8 bytes.
 *							 cs is valid only when the frame transmit¡£
 *
 * input parameters
 * @param       spi_ptr:which spi module used,polling mode or int mode,force cs manually or not
 * @param       tx_buf:send data buffer start address
 * @param       rx_buf:receive data buffer start address
 * @param       len:tramsmit data len
 *
 * output parameters
 * @param       None.
 *
 * @return
 *              PPlus_SUCCESS
 *              parameter error
 **************************************************************************************/
int hal_spi_transmit(hal_spi_t *spi_ptr, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);

int hal_spi_int_set_tx_buf(hal_spi_t *spi_ptr, uint8_t *tx_buf, uint16_t len);
int hal_spi_set_int_mode(hal_spi_t *spi_ptr, bool en);
int hal_spi_set_force_cs(hal_spi_t *spi_ptr, bool en);

bool hal_spi_get_transmit_bus_state(hal_spi_t *spi_ptr);
void hal_spi_TxComplete(hal_spi_t *spi_ptr);
void hal_spi_send_byte(hal_spi_t *spi_ptr, uint8_t data);

#endif
