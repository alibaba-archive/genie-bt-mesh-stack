/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include <misc/printk.h>
#include <hal/hal.h>

#include "genie_app.h"

void genie_init(void)
{
    //add genie app init func
    genie_flash_init();

#ifdef CONFIG_GENIE_CLI
    genie_cmds_register();
#endif

#ifdef CONFIG_GENIE_RESET_BY_REPEAT
    genie_reset_by_repeat_init();
#endif

#ifdef CONFIG_BT_MESH
    genie_mesh_init();
#endif
}

