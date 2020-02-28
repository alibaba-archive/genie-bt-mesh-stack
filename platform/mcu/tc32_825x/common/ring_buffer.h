
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

