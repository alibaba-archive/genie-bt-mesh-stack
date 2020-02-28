/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "hal/soc/soc.h"

const hal_logic_partition_t hal_partitions[] = 
{
    // 0x00000 -- 0x3BFFF: firmware 1, refer to OTA_FIRMWARE_SIZE_MAX
	// 0x40000 -- 0x7BFFF: firmware 2, refer to OTA_FIRMWARE_SIZE_MAX
    // 0x3C000  // reserve
    // 0x7C000  // reserve

    [HAL_PARTITION_PARAMETER_4] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "seq",
        .partition_start_addr       = 0x3D000,
        .partition_length           = 0x1000, // 4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [HAL_PARTITION_PARAMETER_1] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "system",
        .partition_start_addr       = 0x7D000,
        .partition_length           = 0x1000, // 4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [HAL_PARTITION_PARAMETER_2] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "userdata",
        .partition_start_addr       = 0x7E000,
        .partition_length           = 0x1000, // 4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [HAL_PARTITION_PARAMETER_3] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "recycle",
        .partition_start_addr       = 0x3F000,
        .partition_length           = 0x1000, //4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [HAL_PARTITION_CUSTOM_1] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "kv",
        .partition_start_addr       = 0x3E000,
        .partition_length           = 0x1000, //4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [HAL_PARTITION_CUSTOM_2] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "freq",
        .partition_start_addr       = 0x7F000,
        .partition_length           = 0x1000, //4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
};

