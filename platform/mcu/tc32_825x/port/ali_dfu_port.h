/********************************************************************************************************
 * @file     ali_dfu_port.h 
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

#ifndef _ALI_DFU_PORT_H
#define _ALI_DFU_PORT_H
#include <stdbool.h>

enum {

    DFU_IMAGE_A = 0,
    DFU_IMAGE_B,
    DFU_IMAGE_TOTAL = DFU_IMAGE_B,
    DFU_IMAGE_ERR,
};

bool dfu_check_checksum(short image_id, unsigned short *crc16_output);
int get_image_crc();
void unlock_flash_all();
void lock_flash();
int ali_dfu_image_update(short signature, int offset, int length, int/*void*/ *p_void);

#endif