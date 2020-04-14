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

#ifndef YOC_PARTITION_H
#define YOC_PARTITION_H

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include <devices/device.h>

#ifndef MTB_IMAGE_NAME_SIZE
#define MTB_IMAGE_NAME_SIZE 8
#endif

#define MAX_FLASH_NUM   1

typedef int partition_t;

typedef struct {
    char     description[MTB_IMAGE_NAME_SIZE];
    uint32_t base_addr;
    uint32_t start_addr;
    uint32_t length;
    uint16_t sector_size;
    uint8_t  idx;

    dev_t   *g_eflash_dev;
} hal_logic_partition_t;


int partition_init(void);
partition_t partition_open(const char *name);
void partition_close(partition_t partition);


hal_logic_partition_t *hal_flash_get_info(partition_t partition);

/**
 * Read data from an area on a Flash to data buffer in RAM
 *
 * @param[in]  partition       The target flash logical partition which should be read
 * @param[in]  off_set         Point to the start address that the data is read, and
 *                             point to the last unread address after this function is
 *                             returned, so you can call this function serval times without
 *                             update this start address.
 * @param[in]  data            Point to the data buffer that stores the data read from flash
 * @param[in]  size            The length of the buffer
 *
 * @return  0 : On success, <0 If an error occurred with any step
 */
int partition_read(partition_t partition, off_t off_set, void *data, size_t size);
/**
 * Write data to an area on a flash logical partition without erase
 *
 * @param[in]  partition       The target flash logical partition which should be read which should be written
 * @param[in]  off_set         Point to the start address that the data is written to, and
 *                             point to the last unwritten address after this function is
 *                             returned, so you can call this function serval times without
 *                             update this start address.
 * @param[in]  data            point to the data buffer that will be written to flash
 * @param[in]  size            The length of the buffer
 *
 * @return  0 : On success, <0 If an error occurred with any step
 */
int partition_write(partition_t partition, off_t off_set, void *data, size_t size);

/**
 * Erase an area on a Flash logical partition
 *
 * @note  Erase on an address will erase all data on a sector that the
 *        address is belonged to, this function does not save data that
 *        beyond the address area but in the affected sector, the data
 *        will be lost.
 *
 * @param[in]  in_partition  The target flash logical partition which should be erased
 * @param[in]  off_set       Start address of the erased flash area
 * @param[in]  block_count           block_count  of the erased flash area
 *
 * @return  0 : On success, <0 If an error occurred with any step
 */
int32_t partition_erase(partition_t partition, off_t off_set, uint32_t block_count );


typedef enum {
    DIGEST_HASH_NONE   = 0,
    DIGEST_HASH_SHA1   = 1,
    DIGEST_HASH_MD5    = 2,
    DIGEST_HASH_SHA224 = 3,
    DIGEST_HASH_SHA256 = 4,
    DIGEST_HASH_SHA384 = 5,
    DIGEST_HASH_SHA512 = 6
} digest_sch_e;

int32_t get_partition_digest(partition_t partition, uint8_t *buffer, int32_t *buffer_size, digest_sch_e *digest_type);

#endif