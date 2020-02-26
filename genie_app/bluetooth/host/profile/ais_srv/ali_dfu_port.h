/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#ifndef _ALI_DFU_PORT_H
#define _ALI_DFU_PORT_H
#include <stdbool.h>

#define DFU_IMAGE_SIZE_MAX 240*1024
#define CONFIG_AIS_TOTAL_FRAME 4

extern void unlock_flash_all();
extern void flash_lock();
extern void dfu_reboot(void);
extern unsigned char dfu_check_checksum(short image_id, unsigned short *crc16_output);
extern int ali_dfu_image_update(short signature, int offset, int length, int *p_void);
extern void lock_flash(void);

#endif
