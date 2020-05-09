/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GENIE_FLASH_H_
#define _GENIE_FLASH_H_

#include <bluetooth.h>

#include <api/mesh.h>
#include "net.h"
#include "flash.h"

enum {
    GENIE_FLASH_INDEX_START = 0,
    GFI_MESH_PARA,
    GFI_MESH_TRITUPLE,
    GFI_MESH_SADDR,
    GFI_MESH_SUB,
    GFI_MESH_SEQ,
    GFI_MESH_RESET_CNT,
    GFI_MESH_RESET_FLAG,
    GFI_MESH_POWERUP,
    GFI_MESH_HB,
    GFI_MESH_DEVKEY,
    GFI_MESH_NETKEY,
    GFI_MESH_APPKEY,
    GFI_OTA_INDICAT,
    GFI_OTA_IMAGE_CHANGE,
    GFI_USERDATA_START,
};

typedef enum
{
    GENIE_FLASH_SUCCESS = 0,
    GENIE_FLASH_INIT_FAIL,
    GENIE_FLASH_MALLOC_FAIL,
    GENIE_FLASH_EARSE_FAIL,
    GENIE_FLASH_DATA_INVALID,
    GENIE_FLASH_SEARCH_NONE,
    GENIE_FLASH_RECYCLE_FAIL,
    GENIE_FLASH_ERASE_FAIL,
    GENIE_FLASH_READ_FAIL,
    GENIE_FLASH_WRITE_FAIL,
    GENIE_FLASH_DELETE_FAIL,
    GENIE_FLASH_INVALID_INDEX,
    GENIE_FLASH_SIZE_ERROR,
    GENIE_FLASH_CHECK_CRC_FAIL,
} E_GENIE_FLASH_ERRCODE;

/**
 * @brief iniatialize the flash for further operation
 * @return the status of iniatializetion, 0 for successed.
 */
E_GENIE_FLASH_ERRCODE genie_flash_init(void);

/**
 * @brief read the date from flash and decrypt it.
 * @param[in] index refers to the flash partition to be read.
 * @param[out] p_buff: the data.
 * @param[in] data_size: the size of the data.
 * @return the status of reading operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_reliable(uint16_t index, uint8_t *p_buff, uint16_t data_size);

/**
 * @brief encrypt the data and write it into flash
 * @param[in] index refers to the flash partition to be read.
 * @param[out] p_buff: the data.
 * @param[in] data_size: the size of the data.
 * @return the status of writing operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_reliable(uint16_t index, uint8_t *p_buff, uint16_t data_size);

/**
 * @brief delete the contents saved at the partition refered by index.
 * @param[in] index refers to the partition
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_delete_reliable(uint16_t index);

/**
 * @brief erase the system partition.
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_erase_reliable(void);

/**
 * @brief read the user raw date from flash
 * @param[in] index refers to the flash partition to be read.
 * @param[out] p_buff: the data.
 * @param[in] data_size: the size of the data.
 * @return the status of reading operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_userdata(uint16_t index, uint8_t *p_buff, uint16_t data_size);

/**
 * @brief write the date into flash
 * @param[in] index refers to the flash partition to be read.
 * @param[out] p_buff: the data.
 * @param[in] data_size: the size of the data.
 * @return the status of writing operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_userdata(uint16_t index, uint8_t *p_buff, uint16_t data_size);

/**
 * @brief delete the contents saved at the partition refered by index.
 * @param[in] index refers to the partition
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_delete_userdata(uint16_t index);

/**
 * @brief erase the partition for user.
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_erase_userdata(void);

/**
 * @brief read the mesh outgoing sequence number from flash.
 * @param[out] p_seq: the sequence number
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_seq(uint32_t *p_seq);

/**
 * @brief write the mesh outgoing sequence number into flash.
 * @param[in] p_seq: the sequence number
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_seq(uint32_t *p_seq);

/**
 * @brief delete the mesh outgoing sequence number saved in flash.
 * @return the status of the operation.
 */
E_GENIE_FLASH_ERRCODE genie_flash_delete_seq(void);

#if defined(CONFIG_GENIE_DEBUG_CMD)
/**
 * @brief the command line for degging system patition
 * @param[in] pwbuf: no used
 * @param[in] blen: no used
 * @param[in] argc: the number of arguments
 * @param[in] argv: the argument list
 */
void cmd_handle_flash_sys(char *pwbuf, int blen, int argc, char **argv);

/**
 * @brief the command line for degging user data patition
 * @param[in] pwbuf: no used
 * @param[in] blen: no used
 * @param[in] argc: the number of arguments
 * @param[in] argv: the argument list
 */
void cmd_handle_flash_ud(char *pwbuf, int blen, int argc, char **argv);

/**
 * @brief the command line for degging sequence number
 * @param[in] pwbuf: no used
 * @param[in] blen: no used
 * @param[in] argc: the number of arguments
 * @param[in] argv: the argument list
 */
void cmd_handle_flash_seq(char *pwbuf, int blen, int argc, char **argv);
#endif

#endif/*--_GENIE_FLASH_H_--*/
