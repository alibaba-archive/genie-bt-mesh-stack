#include <stdio.h>
#include "ali_dfu_port.h"
#include "uart_pub.h"

#define LOG		//bk_printf


typedef struct
{
    uint32_t crc;       // CRC must not be 0x0000 or 0xFFFF.
  	uint16_t ver;
	uint16_t len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).

	uint32_t  uid;       // User-defined Image Identification bytes.
	uint8_t  crc_status;     // cur image crc status
	uint8_t  sec_status;     // cur image sec status
	uint16_t  rom_ver;     // Rom ver.
}ota_hdr_t;


/**
 * @brief 写flash上锁
 * 
 * @param[in]  -
 * 
 * @return -
 */
void lock_flash()
{
	hal_flash_enable_secure(HAL_PARTITION_OTA_TEMP, 0, 0);
}

/**
 * @brief 写flash解锁
 * 
 * @param[in]  -
 * 
 * @return -
 */
void unlock_flash_all()
{
	hal_flash_dis_secure(HAL_PARTITION_OTA_TEMP, 0, 0);
}

/**
 * @brief 镜像更新
 * 
 * @param[in]  signature	暂时不使用
 * @param[in]  offset		当前buf代表的内容，从镜像bin文件从offset位置开始，比如为100，表示当前buffer是bin文件的第100字节开始
 * @param[in]  length		本次buffer的长度
 * @param[in]  buf			本次写入的具体内容
 * 
 * @return 0:success, otherwise is failed
 */
int ali_dfu_image_update(short signature, int offset, int length, int/*void*/ *buf)
{
	hal_logic_partition_t *partition_info;

	///get OTA temporary partition information
	partition_info = hal_flash_get_info( HAL_PARTITION_OTA_TEMP );

	if(partition_info->partition_length < (offset+length))
	{
		LOG("The write range is over OTA temporary!\r\n");
		return -1;
	}

	uint32_t wr_idx = offset;
	uint32_t wr_end = offset+length;
	uint8_t *wr_buf = buf;
	while(wr_idx != wr_end)
	{
		///check sector is clean
		if(!(wr_idx%0x1000))
		{
			uint8_t  i, chk_buf[16];
			uint32_t rd_idx = wr_idx;
			hal_flash_read(HAL_PARTITION_OTA_TEMP, &rd_idx, (void *)chk_buf, 16);
			for(i=0; i<16; i++) {
				if(chk_buf[i]!=0xFF) break;
			}
			if(i < 16) {
				///erase sector
				LOG("erase sector 0x%x\r\n", wr_idx+partition_info->partition_start_addr);
				hal_flash_erase(HAL_PARTITION_OTA_TEMP, wr_idx, 0x1000);
			}
		}
		uint32_t len = ((0x1000-(wr_idx%0x1000)) < (wr_end-wr_idx)) ?
						(0x1000-(wr_idx%0x1000)) : (wr_end-wr_idx);
		if(hal_flash_write(HAL_PARTITION_OTA_TEMP, &wr_idx, (void *)wr_buf, len))
		{
			LOG("write flash error!!\r\n");
			return -1;
		}
		wr_buf += len;
	}

	return 0;
}

/**
 * @brief 写入flash之前和之后checksum计算
 * 
 * @param[in]  image_id	暂时不使用
 * @param[in]  crc16_output	计算出来的crc返回给调用者
 * 
 * @return true:success false:failed
 */
bool dfu_check_checksum(short image_id, unsigned short *crc16_output)
{
	return true;
}

/**
 * @brief 升级结束后重启
 * 
 * @param[in]  - 
 * @return -
 * @说明： 比如在此函数里面call 切换镜像分区的业务逻辑
 */
void dfu_reboot()
{
	hal_reboot();
}

