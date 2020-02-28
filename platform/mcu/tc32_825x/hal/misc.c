/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
     
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

