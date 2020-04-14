/*
 * Copyright (C) 2017-2019 C-SKY Microsystems Co., Ltd. All rights reserved.
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
 * @file     pin_name.h
 * @brief    header file for the pin_name
 * @version  V1.0
 * @date     18. May 2019
 ******************************************************************************/

#ifndef _PIN_NAME_H_
#define _PIN_NAME_H_

#include <gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef GPIO_Pin_e pin_name_e;

typedef Fmux_Type_e pin_func_e;

#define PIN_FUNC_GPIO 99

typedef enum {
    PORTA = 0,
    PORTB = 1,
} port_name_e;

#ifdef __cplusplus
}
#endif

#endif
