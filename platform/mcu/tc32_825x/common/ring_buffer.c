
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



