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



#ifndef _BURN_H_
#define _BURN_H_

#include "types.h"


#define M4_BIN_NUM_OFFSET           0x2000
#define M4_BIN_ADDR_OFFSET(n)       (M4_BIN_NUM_OFFSET+4+n*12)
#define M4_BIN_SIZE_OFFSET(n)       (M4_BIN_NUM_OFFSET+8+n*12)
#define M4_RUN_ADDR_OFFSET(n)       (M4_BIN_NUM_OFFSET+12+n*12)

#define M0_BIN_NUM_OFFSET           0x2100
#define M0_BIN_ADDR_OFFSET(n)       (M0_BIN_NUM_OFFSET+4+n*12)
#define M0_BIN_SIZE_OFFSET(n)       (M0_BIN_NUM_OFFSET+8+n*12)
#define M0_RUN_ADDR_OFFSET(n)       (M0_BIN_NUM_OFFSET+12+n*12)


#define USR_DATA_BASE                0x4000

#define IMAGE_BASE_OFFSET           0x5000


#define  M4_BIN          1
#define  M0_BIN          2


#define BURN_BUF_LEN     64

typedef struct {
    uint8_t  valid;
    uint8_t  length;
    uint8_t  data[BURN_BUF_LEN];
    uint8_t  end;

} burn_buf_st;

//void handle_uart_command(void);


#endif
