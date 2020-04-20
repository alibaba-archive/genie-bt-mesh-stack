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
 * @file     phy_rtc.h
 * @brief    header file for rtc driver
 * @version  V1.0
 * @date     23. May 2019
 ******************************************************************************/
#ifndef __PHY_RTC_H
#define __PHY_RTC_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEC_PER_MIN  ((time_t)60)
#define SEC_PER_HOUR ((time_t)60 * SEC_PER_MIN)
#define SEC_PER_DAY  ((time_t)24 * SEC_PER_HOUR)

// base: 0x4000f024, 28, 2c, 30, 34, 38.

typedef struct {
    uint32_t    RTC_CTL;          /* Offset: 0x000 (R/ )  current count value register */
    uint32_t    RTC_CNT;          /* Offset: 0x004 (R/W)  count match register */
    uint32_t    RTC_CC0;          /* Offset: 0x008 (R/W)  count load register */
    uint32_t    RTC_CC1;          /* Offset: 0x00c (R/W)  count control register */
    uint32_t    RTC_CC2;          /* Offset: 0x010 (R/ )  interrupt status register */
    uint32_t    RTC_FLAG;         /* Offset: 0x014 (R/ )  interrupt raw status register */
} phy_rtc_reg_t;

#ifdef __cplusplus
}
#endif

#endif /* __CK_RTC_H */

