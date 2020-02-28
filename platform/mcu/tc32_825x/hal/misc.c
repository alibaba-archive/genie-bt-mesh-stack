/********************************************************************************************************
 * @file     misc.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/
     
#include <aos/types.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include "drivers/8258/irq.h"

void start_reboot(void);

void hal_reboot(void)
{
    printf("reboot!\n");
    start_reboot();
}

unsigned char tlk_irq_disable()
{
    return irq_disable();
}

void tlk_irq_resrote(unsigned char r)
{
    irq_restore(r);
}

