/********************************************************************************************************
 * @file     flash.c 
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

#include "k_api.h"
#include "hal/soc/soc.h"

#include "drivers/8258/flash.h"
#include "common/static_assert.h"


#define ROUND_DOWN(a,b) (((a) / (b)) * (b))
#define FLASH_ALIGN_MASK ~(sizeof(uint32_t) - 1)
#define FLASH_ALIGN sizeof(uint32_t)

#define SECTOR_SIZE     (4096)  // must equal to BLK_SIZE
//STATIC_ASSERT(SECTOR_SIZE == BLK_SIZE);

extern const hal_logic_partition_t hal_partitions[];

hal_logic_partition_t *hal_flash_get_info(hal_partition_t pno)
{
    hal_logic_partition_t *logic_partition;

    logic_partition = (hal_logic_partition_t *)&hal_partitions[ pno ];

    return logic_partition;
}

int32_t hal_flash_write(hal_partition_t pno, uint32_t* poff, const void* buf ,uint32_t buf_size)
{
#if 0
    return 0; // SQF:confirm later,   
#else
    uint32_t start_addr;

    hal_logic_partition_t *partition_info = hal_flash_get_info( pno );
    if(poff == NULL || buf == NULL || *poff + buf_size > partition_info->partition_length){
        return -1;
    }

    start_addr = partition_info->partition_start_addr + *poff;
    flash_write_page(start_addr, buf_size, buf);

    *poff += buf_size;
    return 0;
#endif
}

int32_t hal_flash_read(hal_partition_t pno, uint32_t* poff, void* buf, uint32_t buf_size)
{
    hal_logic_partition_t *partition_info = hal_flash_get_info( pno );
    if(poff == NULL || buf == NULL || *poff + buf_size > partition_info->partition_length){
        return -1;
    }
    
    uint32_t start_addr = partition_info->partition_start_addr + *poff;
    flash_read_page(start_addr, buf_size, buf);
    *poff += buf_size;

    return 0;
}

int32_t hal_flash_erase(hal_partition_t pno, uint32_t off_set,
                        uint32_t size)
{
    uint32_t addr;
    uint32_t start_addr, end_addr;
    //int32_t ret = 0;

    hal_logic_partition_t *partition_info = hal_flash_get_info( pno );
    if(size + off_set > partition_info->partition_length)
        return -1;

    start_addr = ROUND_DOWN((partition_info->partition_start_addr + off_set), SECTOR_SIZE);
    end_addr = ROUND_DOWN((partition_info->partition_start_addr + off_set + size - 1), SECTOR_SIZE);

    for (addr = start_addr; addr <= end_addr; addr += SECTOR_SIZE) {
        flash_erase_sector(addr);
    }

    return 0;
}

int32_t hal_flash_enable_secure(hal_partition_t partition, uint32_t off_set, uint32_t size)
{
    return 0;
}

int32_t hal_flash_dis_secure(hal_partition_t partition, uint32_t off_set, uint32_t size)
{
    return 0;
}


