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
 * @file     trng.h
 * @brief    header file for trng driver
 * @version  V1.0
 * @date     29. May 2019
 ******************************************************************************/
#ifndef __TRNG_H__
#define __TRNG_H__

#include "drv_trng.h"
#include "soc.h"

/*
 *  define the bits for TCR
 */
//offset 0x04
#define RNG_EN         0x02
//offset 0x08
#define RNG_INT_EN     0x01
#define RNG_IFLAG      0x01

//todo
#ifndef     __IOM
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
#define     __IM     volatile            /*! Defines 'read / write' structure member permissions */
#endif

typedef struct {
    volatile uint32_t RNG;
    __IOM uint32_t EN;
    __IM  uint32_t IFLAG;
} ck_trng_reg_t;

#endif
