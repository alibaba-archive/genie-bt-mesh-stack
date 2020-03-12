#ifndef _ALI_DFU_PORT_H
#define _ALI_DFU_PORT_H
#include <stdbool.h>

bool dfu_check_checksum(short image_id, unsigned short *crc16_output);
int get_image_crc();
void unlock_flash_all();
void lock_flash();
int ali_dfu_image_update(short signature, int offset, int length, int/*void*/ *p_void);

#endif
