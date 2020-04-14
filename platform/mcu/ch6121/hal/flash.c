/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "soc.h"
#include "drv_spiflash.h"
#include "spif.h"
#include "pin.h"
#include "hal/soc/flash.h"
#include "aos/kernel.h"

extern const hal_logic_partition_t hal_partitions[];
static spiflash_handle_t g_spiflash_handle; 
uint32_t g_erase_buf[SPIF_SECTOR_SIZE / 4];
#define ROUND_DOWN(a,b) (((a) / (b)) * (b))
#define FLASH_ALIGN_MASK ~(sizeof(uint32_t) - 1)
#define FLASH_ALIGN sizeof(uint32_t)
#define printk //printf
hal_logic_partition_t *hal_flash_get_info(hal_partition_t in_partition)
{
    hal_logic_partition_t *logic_partition;

    logic_partition = (hal_logic_partition_t *)&hal_partitions[ in_partition ];

    if (logic_partition == NULL || logic_partition->partition_description == NULL) {
        printk("pno %d err!\n", in_partition);
        return NULL;
    }
    
    if (g_spiflash_handle == NULL) {
        g_spiflash_handle = csi_spiflash_initialize(0, NULL);
        if (g_spiflash_handle == NULL) {
            return NULL;
        }
    }
    return logic_partition;
}

int32_t hal_flash_erase(hal_partition_t in_partition, uint32_t off_set, uint32_t size)
{
    int ret = -1;
    hal_logic_partition_t *node = hal_flash_get_info(in_partition);
    if (g_spiflash_handle != NULL && node != NULL && (off_set % SPIF_SECTOR_SIZE) == 0) {
        int blkcnt = 1, tmp_size;
        tmp_size = size % SPIF_SECTOR_SIZE;
        if (tmp_size == 0) {
            blkcnt = size / SPIF_SECTOR_SIZE;
        } else {
            blkcnt += (size / SPIF_SECTOR_SIZE);
        }

        for (int i = 0; i < blkcnt; i++) {
            ret = csi_spiflash_erase_sector(g_spiflash_handle, node->partition_start_addr + off_set + i * SPIF_SECTOR_SIZE);
            if (ret < 0) {
                printk("erase addr:%x\r\n", off_set);
                return -1;
            }
        }
    }

    return 0;
}

int32_t hal_flash_write(hal_partition_t in_partition, uint32_t *off_set, const void *in_buf , uint32_t in_buf_len)
{
    int ret = -1;
    uint32_t start_addr, len, left_off;
    uint8_t *buffer = (uint8_t *)g_erase_buf;
    hal_logic_partition_t *node = hal_flash_get_info(in_partition);
    
    if(g_spiflash_handle == NULL || node == NULL ||
       off_set == NULL || in_buf == NULL ||
       *off_set + in_buf_len > node->partition_length)
        return -1;
    start_addr = node->partition_start_addr + *off_set;

    left_off = start_addr % FLASH_ALIGN;
    len = ((in_buf_len + left_off) + ~FLASH_ALIGN_MASK) & FLASH_ALIGN_MASK;

    if (len > in_buf_len || left_off > 0) {
        memset(buffer, 0xFF, len);
        memcpy(buffer + left_off, in_buf, in_buf_len);
        csi_spiflash_program(g_spiflash_handle, start_addr - left_off, buffer, len);
    } else {
        csi_spiflash_program(g_spiflash_handle, start_addr, in_buf, len);
    }
#if 0
        printf("write[%x, %x, %d]: ", in_partition, start_addr, in_buf_len);
        uint8_t *point = (uint8_t *)in_buf;
        for (int i = 0; i < in_buf_len; i++) {
            printf("%x ", point[i]);
        }
        printf("\n\n");
#endif
    *off_set += in_buf_len;
    return 0;
}

int32_t hal_flash_read(hal_partition_t in_partition, uint32_t *off_set, void *out_buf, uint32_t out_buf_len)
{
    int ret = 0;
    uint32_t start_addr, len, left_off;
    uint8_t *buffer = (uint8_t *)g_erase_buf;
    hal_logic_partition_t *node = hal_flash_get_info(in_partition);

    if(g_spiflash_handle == NULL || node == NULL ||
       off_set == NULL || out_buf == NULL ||
       *off_set + out_buf_len > node->partition_length)
        return -1;
    start_addr = node->partition_start_addr + *off_set;

    memset(buffer, 0, sizeof(g_erase_buf));
    ret = csi_spiflash_read(g_spiflash_handle, start_addr, buffer, out_buf_len);
    
    memcpy(out_buf, buffer, out_buf_len);
#if 0
    printf("read[%x, %x, %d]: ", in_partition, start_addr, out_buf_len);
    buffer = out_buf;
    for (int i = 0; i < out_buf_len; i++) {
        printf("%x ", buffer[i]);
    }
    printf("\n\n");
#endif    
    (void)ret;
    *off_set += out_buf_len;
    return 0;
}

/**
 * Set security options on a logical partition
 *
 * @param[in]  partition  The target flash logical partition
 * @param[in]  offset     Point to the start address that the data is read, and
 *                        point to the last unread address after this function is
 *                        returned, so you can call this function serval times without
 *                        update this start address.
 * @param[in]  size       Size of enabled flash area
 *
 * @return  0 : On success, EIO : If an error occurred with any step
 */
int32_t hal_flash_enable_secure(hal_partition_t partition, uint32_t off_set, uint32_t size)
{
    return 0;
}

/**
 * Disable security options on a logical partition
 *
 * @param[in]  partition  The target flash logical partition
 * @param[in]  offset     Point to the start address that the data is read, and
 *                        point to the last unread address after this function is
 *                        returned, so you can call this function serval times without
 *                        update this start address.
 * @param[in]  size       Size of disabled flash area
 *
 * @return  0 : On success, EIO : If an error occurred with any step
 */
int32_t hal_flash_dis_secure(hal_partition_t partition, uint32_t off_set, uint32_t size)
{
    return 0;
}

