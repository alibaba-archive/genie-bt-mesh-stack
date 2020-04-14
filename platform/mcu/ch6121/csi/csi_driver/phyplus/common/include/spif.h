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
 * @file     spiflash_w25q64fv.h
 * @brief    head file for spiflash_w25q64fv
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/
#ifndef _SPIFLASH_W25Q64FV_H_
#define _SPIFLASH_W25Q64FV_H_

//#include "drv_spiflash.h"
//#include "soc.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

//todo
#ifndef 		__IOM
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
#endif

#define CONFIG_SPIFLASH_NUM 1
#define DEBUG_EN            1
//#define SPIF_ADDR_START     	0x11005000
//#define SPIF_ADDR_END         0x11010000
//#define SPIF_SECTOR_COUNT     11

#define SPIF_ADDR_START     	0x11000000
#define SPIF_ADDR_END         0x11080000//0x11010000
#define SPIF_SECTOR_COUNT     128

#define SPIF_SECTOR_SIZE      0x1000
#define SPIF_PAGE_SIZE        0x100

#ifdef __cplusplus
}
#endif

#endif
