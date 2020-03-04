/********************************************************************************************************
 * @file     ring_buffer.c 
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
#include "ring_buffer.h"


void ring_buffer_init(ring_buffer_t *rb, unsigned char *buf, int size)
{
	rb->buf = buf;
	rb->size = size;
	rb->wptr = 0;
	rb->rptr = 0;
}


bool ring_buffer_write_byte(ring_buffer_t *rb, unsigned char data)
{
	if (   (rb->wptr + 1 != rb->rptr) 
		&& (rb->wptr + 1 != rb->rptr + rb->size) ){
		rb->buf[rb->wptr++] = data;
		if (rb->wptr >= rb->size){
			rb->wptr = 0;
		}
		return true;
	}
	return false;
}

int ring_buffer_write(ring_buffer_t *rb, const unsigned char *data, int length)
{
	for(int i=0;i<length;i++){
		int res = ring_buffer_write_byte(rb, data[i]);
		if ( res == false ){
		    static unsigned int RingBufOverFlowCnt;RingBufOverFlowCnt++;
			return i;
		}
	}
	return length;
}


bool ring_buffer_read_byte(ring_buffer_t *rb, unsigned char *data)
{
	if ( rb->rptr != rb->wptr ){
		*data = rb->buf[rb->rptr++];
		if ( rb->rptr >= rb->size ){
			rb->rptr = 0;
		}
		return true;
	}
	return false;
}


bool ring_buffer_read(ring_buffer_t *rb, unsigned char *data, int length)
{
	for(int i=0;i<length;i++){
		int res = ring_buffer_read_byte(rb, &data[i]);
		if ( res == false ){
			return i;
		}
	}
	return length;
}


int ring_buffer_get_count(ring_buffer_t *rb)
{
	if ( rb->wptr >= rb->rptr ){
		return rb->wptr - rb->rptr;
	}else{
		return rb->wptr + rb->size - rb->rptr;
	}
}



