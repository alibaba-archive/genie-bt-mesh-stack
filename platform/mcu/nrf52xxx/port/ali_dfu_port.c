#include <stdio.h>
#include <string.h>
#include <aos/aos.h>
#include <aos/kernel.h>
#include <hal/soc/flash.h>

#include "ali_dfu_port.h"

extern int 	ota_firmware_size_k;
extern unsigned int	ota_program_offset;

#define LOG_DFU_PORT    //bk_printf


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
    //printf("%s called\n", __FUNCTION__);
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
    //printf("%s called. offset=%d, length=%d, buf=%p\n", __FUNCTION__, offset, length, buf);
    //return 0;

    printf("offset=%d,len=%d\n", offset, length);

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
    printf("%s called\n", __FUNCTION__);
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
	printf("%s called\n", __FUNCTION__);

	hal_reboot();
}

