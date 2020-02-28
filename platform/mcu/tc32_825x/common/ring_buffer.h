/********************************************************************************************************
 * @file     ring_buffer.h 
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
#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include "tl_common.h"

typedef struct{
	unsigned char *buf;
	int size;
	int wptr;
	int rptr;
} ring_buffer_t;


void ring_buffer_init(ring_buffer_t *rb, unsigned char *buf, int buffer_size);
bool ring_buffer_write_byte(ring_buffer_t *rb, unsigned char data);
int ring_buffer_write(ring_buffer_t *rb, const unsigned char *data, int length);
bool ring_buffer_read_byte(ring_buffer_t *rb, unsigned char *data);
bool ring_buffer_read(ring_buffer_t *rb, unsigned char *data, int length);
int ring_buffer_get_count(ring_buffer_t *rb);



#endif

