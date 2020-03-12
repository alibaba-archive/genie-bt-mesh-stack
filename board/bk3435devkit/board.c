/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "hal/soc/soc.h"
#include <aos/kernel.h>
#include <aos/aos.h>

/* Logic partition on flash devices */
const hal_logic_partition_t hal_partitions_8M[] =
{
	[HAL_PARTITION_BOOTLOADER] =
	{
	    .partition_owner            = HAL_FLASH_EMBEDDED,
	    .partition_description      = "Bootloader",
	    .partition_start_addr       = 0x000000,
        .partition_length           = 0x003000, // 11.5KB = 0x2E00+CRC = (0x2E00/16)*17
	    .partition_options          = PAR_OPT_READ_EN,
	},
	[HAL_PARTITION_BT_FIRMWARE] =
	{
	    .partition_owner            = HAL_FLASH_EMBEDDED,
	    .partition_description      = "BLE FW",
	    .partition_start_addr       = 0x003000,
        .partition_length           = 0x014000, // 75KB = 0x13F00+CRC = (0x13F00/16)*17
	    .partition_options          = PAR_OPT_READ_EN,
	},
	[HAL_PARTITION_APPLICATION] =
	{
	    .partition_owner            = HAL_FLASH_EMBEDDED,
	    .partition_description      = "Application",
	    .partition_start_addr       = 0x017000,
        .partition_length           = 0x072000, // 429KB = 0x6B400+CRC = (0x6B400/16)*17
	    .partition_options          = PAR_OPT_READ_EN,
	},
	[HAL_PARTITION_OTA_TEMP] =
	{
		.partition_owner		   = HAL_FLASH_EMBEDDED,
		.partition_description	   = "OTA Storage",
		.partition_start_addr	   = 0x089000,
		.partition_length		   = 0x072000, // 429KB = 0x6B400+CRC = (0x6B400/16)*17
		.partition_options		   = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_PARAMETER_3] =
	{
		.partition_owner			= HAL_FLASH_EMBEDDED,
		.partition_description		= "PARAMETER_3",
		.partition_start_addr		= 0x0FC000,// boot information need protect
		.partition_length			= 0x1000, // 4k bytes
		.partition_options			= PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
    [HAL_PARTITION_CUSTOM_2] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "CUSTOM_2",
        .partition_start_addr       = 0x0FD000,
        .partition_length           = 0x1000, // 4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
	[HAL_PARTITION_PARAMETER_1] =
	{
		.partition_owner			= HAL_FLASH_EMBEDDED,
		.partition_description		= "PARAMETER1",
		.partition_start_addr		= 0x0FE000,
		.partition_length			= 0x1000, // 4k bytes
		.partition_options			= PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_CUSTOM_1] =
	{
		.partition_owner			= HAL_FLASH_EMBEDDED,
		.partition_description		= "CUSTOM_1",
		.partition_start_addr		= 0x0FF000,
		.partition_length			= 0x1000, // 4k bytes
		.partition_options			= PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
};

const hal_logic_partition_t hal_partitions_4M[] =
{
	[HAL_PARTITION_BOOTLOADER] =
	{
	    .partition_owner            = HAL_FLASH_EMBEDDED,
	    .partition_description      = "Bootloader",
	    .partition_start_addr       = 0x000000,
        .partition_length           = 0x003000, // 11.5KB = 0x2E00+CRC = (0x2E00/16)*17
	    .partition_options          = PAR_OPT_READ_EN,
	},
	[HAL_PARTITION_BT_FIRMWARE] =
	{
	    .partition_owner            = HAL_FLASH_EMBEDDED,
	    .partition_description      = "BLE FW",
	    .partition_start_addr       = 0x003000,
        .partition_length           = 0x012000, // 67KB = 0x10F00+CRC = (0x10F00/16)*17
	    .partition_options          = PAR_OPT_READ_EN,
	},
	[HAL_PARTITION_APPLICATION] =
	{
	    .partition_owner            = HAL_FLASH_EMBEDDED,
	    .partition_description      = "Application",
	    .partition_start_addr       = 0x015000,
        .partition_length           = 0x033000, // 191.5KB = 0x2FE00+CRC = (0x2FE00/16)*17
	    .partition_options          = PAR_OPT_READ_EN,
	},
	[HAL_PARTITION_OTA_TEMP] =
	{
		.partition_owner		   = HAL_FLASH_EMBEDDED,
		.partition_description	   = "OTA Storage",
		.partition_start_addr	   = 0x048000,
		.partition_length		   = 0x033000, // 191.5KB = 0x2FE00+CRC = (0x2FE00/16)*17
		.partition_options		   = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_PARAMETER_3] =
	{
		.partition_owner			= HAL_FLASH_EMBEDDED,
		.partition_description		= "PARAMETER_3",
		.partition_start_addr		= 0x07C000,// boot information need protect
		.partition_length			= 0x1000, // 4k bytes
		.partition_options			= PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
    [HAL_PARTITION_CUSTOM_2] =
    {
        .partition_owner            = HAL_FLASH_EMBEDDED,
        .partition_description      = "CUSTOM_2",
        .partition_start_addr       = 0x07D000,
        .partition_length           = 0x1000, // 4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
	[HAL_PARTITION_PARAMETER_1] =
	{
		.partition_owner			= HAL_FLASH_EMBEDDED,
		.partition_description		= "PARAMETER1",
		.partition_start_addr		= 0x07E000,
		.partition_length			= 0x1000, // 4k bytes
		.partition_options			= PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
	[HAL_PARTITION_CUSTOM_1] =
	{
		.partition_owner			= HAL_FLASH_EMBEDDED,
		.partition_description		= "CUSTOM_1",
		.partition_start_addr		= 0x07F000,
		.partition_length			= 0x1000, // 4k bytes
		.partition_options			= PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
	},
};

void board_init(void)
{
}

int board_cli_init(void)
{ 
}

