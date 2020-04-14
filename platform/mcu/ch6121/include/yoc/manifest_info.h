/*
 * Copyright (C) 2018 C-SKY Microsystems Co., Ltd. All rights reserved.
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
#ifndef __MANIFEST_INFO_H__
#define __MANIFEST_INFO_H__

#include <yoc_config.h>

#ifndef CONFIG_TEE_CA

#define CID_MAX             0x20
#define MAX_VERSION_COUNT   0x20

#define BMTB_MAGIC  0x74427254
#define MTB_MAGIC  0x74427251

#define IMAGER_MANTB_ADDR_HW    0x11052000//0x10000000
#define BACKUP_MANTB_ADDR       0x11052800//0x10000800
#define MAX_MANTB_BYTE_SIZE     0x1000// spiflash 4K erase block

#define FLASH_SECTOR1           512
#define FLASH_IMG_SETCOR        FLASH_SECTOR1
#define CONFIG_ENABLE_MANTB_V1  1
#define MTB_IMAGE_NAME_SIZE     8
#define MTB_IMAGE_VERSION_SIZE  12
#define MTB_OS_VERSION1         32
#define MTB_OS_VERSION2         64
#define MTB_APP_VERSION_MAX_SIZE 64
#define SEACH_MODE_FIRST_TYPE   0
#define SEACH_MODE_SON_TYPE     1
#define SEACH_MODE_EXTEND_TYPE  2

typedef struct {
    uint16_t encrypted: 1;
    uint16_t reserve: 7;
    uint16_t update_flag: 1;
    uint16_t reserve2: 7;
} tb_flag;

typedef struct {
    uint32_t magic;
    uint16_t version;
    tb_flag  flag;
    uint16_t scn_count;
    uint16_t digest_sch;
    uint16_t signature_sch;
    uint16_t size;
} mhead_tb;

typedef struct {
    uint8_t extend_type: 4;
    uint8_t first_type: 4;
} scn_type_t;

typedef struct {
    scn_type_t father_type;
    uint8_t son_type;
} scn_info_t;

typedef struct {
    scn_info_t type;
    uint16_t index;
    uint32_t size;
} scn_head_t;

typedef struct {
    uint8_t value[MTB_OS_VERSION1];
} os_version_t1;

typedef struct {
    uint8_t value[MTB_OS_VERSION2];
} os_version_t2;

typedef struct {
    uint8_t  image_name[MTB_IMAGE_NAME_SIZE];
    uint64_t static_addr;
    uint64_t loading_addr;
    uint32_t image_size;
    uint8_t  image_version[MTB_IMAGE_VERSION_SIZE];
} scn_img_sig_info_t;

typedef enum {
    SCN_TYPE_IMG = 0,
    SCN_TYPE_KEY = 4,
    SCN_TYPE_PART = 5,
} scn_type_first_e;

typedef enum {
    SCN_TYPE_NORMAL = 0,
} scn_type_extend_e;

typedef enum {
    SCN_SUB_TYPE_IMG_PART = 0,
} scn_sub_type_part_e;

#define PUBLIC_KEY_NAME_SIZE 8
// imtb v4
typedef struct {
    uint32_t magic;
    uint16_t version;
    tb_flag  flag;
    uint16_t digest_sch;
    uint16_t signature_sch;
    char     pub_key_name[PUBLIC_KEY_NAME_SIZE];
    uint16_t partition_count;
    uint16_t size;
} imtb_head_v4_t;

typedef struct {
    char        name[MTB_IMAGE_NAME_SIZE];
    char        pub_key_name[PUBLIC_KEY_NAME_SIZE];
    scn_type_t  partition_type;
    uint16_t    block_count;
    uint32_t    block_offset;
    uint32_t    load_address;
    uint32_t    img_size;
} imtb_partition_info_v4_t;

typedef struct {
    uint32_t using_addr;    //当前使用的imtb地址(RAM、FLASH)
    uint32_t valid_addr;    //当前使用的imtb地址(FLASH)
    uint32_t backup_addr;   //备份的imtb地址(FLASH)
    uint32_t one_size;      //一份imtb表占用的分区size
    uint64_t flash_start;
    int version;
} mtb_t;

typedef struct {
  union {
        mhead_tb head;
        imtb_head_v4_t headv4;
  };
} mtb_head_t;

// for partition.c
struct partion_info_t {
    char     image_name[MTB_IMAGE_NAME_SIZE];
    uint64_t part_addr;
    uint64_t part_size;
};

int32_t get_sys_partition(uint8_t *out, uint32_t *out_len);
int32_t get_app_version(uint8_t *out, uint32_t *out_len);
#endif

#endif