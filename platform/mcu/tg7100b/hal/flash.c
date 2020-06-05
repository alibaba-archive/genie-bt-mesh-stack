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
static aos_mutex_t g_spiflash_mutex;
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
        aos_mutex_new(&g_spiflash_mutex);
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
    uint32_t write_addr;
    uint8_t *pwrite_buf = in_buf;
    uint32_t write_len = in_buf_len;
    uint8_t tmp_write[FLASH_ALIGN] = {0};

    hal_logic_partition_t *node = hal_flash_get_info(in_partition);
    
    if(g_spiflash_handle == NULL || node == NULL ||
       off_set == NULL || in_buf == NULL ||
        *off_set + in_buf_len > node->partition_length) {
        return -1;
    }

    aos_mutex_lock(&g_spiflash_mutex, AOS_WAIT_FOREVER);
    start_addr = node->partition_start_addr + *off_set;
    write_addr = start_addr;

    ret = csi_spiflash_program(g_spiflash_handle, write_addr, pwrite_buf, write_len);

    if (ret != write_len) {
        return -1;
    }

    *off_set += in_buf_len;
    aos_mutex_unlock(&g_spiflash_mutex);
    return 0;
}

int32_t hal_flash_read(hal_partition_t in_partition, uint32_t *off_set, void *out_buf, uint32_t out_buf_len)
{
    int ret = 0;
    uint32_t start_addr, len, left_off;
    uint32_t read_addr = out_buf;
    uint32_t read_len = out_buf_len;
    uint8_t tmp_read[FLASH_ALIGN] = {0};
    uint8_t *pread_buf = out_buf;
    hal_logic_partition_t *node = hal_flash_get_info(in_partition);

    if (g_spiflash_handle == NULL || node == NULL ||
        off_set == NULL || out_buf == NULL ||
        *off_set + out_buf_len > node->partition_length) {
        return -1;
    }

    aos_mutex_lock(&g_spiflash_mutex, AOS_WAIT_FOREVER);
    start_addr = node->partition_start_addr + *off_set;

    read_addr = start_addr;

    ret = csi_spiflash_read(g_spiflash_handle, read_addr, pread_buf, read_len);

    if (ret != out_buf_len) {
        return -1;
    }

    (void)ret;
    *off_set += out_buf_len;
    aos_mutex_unlock(&g_spiflash_mutex);
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

void hal_flash_write_test()
{
    int ret = 0;
    uint32_t offset = 0;
    uint8_t data[32] = {0};
    uint8_t data_read[32] = {0};
    int i = 0;
    for (i = 0; i < sizeof(data);i++)
    {
        data[i] = i;
    }

    hal_flash_erase(HAL_PARTITION_PARAMETER_2, 0, 4096);

    offset = 0;

    printf("write test 32 by 32 %d, offset %d size %d, data %p\n", HAL_PARTITION_PARAMETER_2, offset, 4096, data);
    for (i = 0;i < 4096 / 32;i++)
    {
        ret = hal_flash_write(HAL_PARTITION_PARAMETER_2, &offset, (uint8_t *)(data), 32);
        if (ret) {
            printf("write 1 fail %d\n", ret);
            break;
        }
    }

    offset = 0;
    memset(data_read, 0, 32);
    for (i = 0;i < 4096 / 32;i++)
    {
        memset(data_read, 0, 32);
        ret = hal_flash_read(HAL_PARTITION_PARAMETER_2, &offset, (uint8_t *)(data_read), 32);
        if (ret) {
            printf("read 1 fail %d\n", ret);
            break;
        }
        if (memcmp(data, data_read, 32))
        {
            printf("write test fail, data missmatch\n");
            break;
        }
    }

    hal_flash_erase(HAL_PARTITION_PARAMETER_2, 0, 4096);

    offset = 0;

    printf("write test 1 by 1 %d, offset %d size %d, data %p\n", HAL_PARTITION_PARAMETER_2, offset, 4096, data);
    for (i = 0;i < 4096;i++)
    {
        ret = hal_flash_write(HAL_PARTITION_PARAMETER_2, &offset, (uint8_t *)(data + i % 32), 1);
        if (ret) {
            printf("write 2 fail %d\n", ret);
            break;
        }
    }

    offset = 0;
    memset(data_read, 0, 32);
    for (i = 0;i < 4096;i++)
    {
        ret = hal_flash_read(HAL_PARTITION_PARAMETER_2, &offset, (uint8_t *)(data_read + i % 32), 1);
        if (ret) {
            printf("read 2 fail %d\n", ret);
            break;
        }
        if ((i + 1) % 32 == 0)
        {
            if (memcmp(data, data_read, 32))
            {
                printf("write 2 1 by 1 test fail, data missmatch\n");
                break;
            }
            memset(data_read, 0, 32);
        }
    }

    hal_flash_erase(HAL_PARTITION_PARAMETER_2, 0, 4096);
}
