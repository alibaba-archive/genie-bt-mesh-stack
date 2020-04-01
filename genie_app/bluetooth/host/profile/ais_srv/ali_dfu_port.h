/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#ifndef _ALI_DFU_PORT_H
#define _ALI_DFU_PORT_H
#include <stdbool.h>

#define DFU_IMAGE_SIZE_MAX 240*1024
#define CONFIG_AIS_TOTAL_FRAME 16

enum {
    DFU_IMAGE_A = 0,
    DFU_IMAGE_B,
    DFU_IMAGE_TOTAL = DFU_IMAGE_B,
    DFU_IMAGE_ERR,
};

/**
 * @brief unlock whole flash.
 */
extern void unlock_flash_all();

/**
 * @brief lock whole flash.
 */
extern void lock_flash(void);

/**
 * @brief reboot the device.
 */
void hal_reboot(void);


/**
 * @brief finish ota and reboot the device.
 */
extern void dfu_reboot(void);

/**
 * @brief check the dfu image.
 * @param[in] the image type.
 * @param[out] the crc of image.
 * @return the result of checksum.
 */
extern unsigned char dfu_check_checksum(short image_id, unsigned short *crc16_output);

/**
 * @brief write dfu data.
 * @param[in] the image type.
 * @param[in] the offset of flash.
 * @param[in] the length of data.
 * @param[in] the writting data.
 * @return the current runing partition.
 */
extern int ali_dfu_image_update(short signature, int offset, int length, int *p_void);

#ifdef CONFIG_GENIE_OTA_PINGPONG
/**
 * @brief get the current runing partition.
 * @return the current runing partition.
 */
extern uint8_t get_program_image(void);

/**
 * @brief switch the running partition, without reboot.
 * @param[in] the partition which switch to.
 * @return the runing partition when next boot.
 */
extern uint8_t change_program_image(uint8_t dfu_image);
#endif

#endif
