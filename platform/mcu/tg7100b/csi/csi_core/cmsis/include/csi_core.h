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
 * @file     csi_core.h
 * @brief    Header File for csi_core
 * @version  V1.0
 * @date     12. june 2019
 ******************************************************************************/
#ifndef _CSI_CORE_H_
#define _CSI_CORE_H_

#include <stddef.h>
#include <cmsis_gcc.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
__STATIC_INLINE size_t csi_irq_save(void)
{
    uint32_t result;
    result = __get_PRIMASK();
    __disable_irq();
    return (result);
}

__STATIC_INLINE void csi_irq_restore(size_t irq_state)
{
    __set_PRIMASK(irq_state);
}
#else
static inline __asm size_t csi_irq_save(void)
{
    MRS     R0, PRIMASK
    CPSID   I
    BX      LR

    return 0;
}

static inline __asm void csi_irq_restore(size_t irq_state)
{
    MSR     PRIMASK, R0
    BX      LR
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* _CSI_CORE_H_ */
