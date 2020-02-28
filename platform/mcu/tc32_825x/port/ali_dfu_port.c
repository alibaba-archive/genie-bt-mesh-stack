#include <stdio.h>
#include <string.h>
#include <aos/aos.h>
#include <aos/kernel.h>
#include <hal/soc/flash.h>

#include "ali_dfu_port.h"

void flash_write_page(unsigned long addr, unsigned long len, const unsigned char *buf);
void flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf);
unsigned short crc16 (unsigned char *pD, int len);
void ota_set_flag();
void show_ota_result(int result);
void hal_reboot(void);


extern int 	ota_firmware_size_k;
extern unsigned int	ota_program_offset;

#define LOG_DFU_PORT    //bk_printf


#if 1
enum{
    OTA_CHECK_TYPE_NONE             = 0,
    OTA_CHECK_TYPE_TELINK_MESH      = 1,
};

void mesh_ota_read_data(unsigned int adr, unsigned int len, unsigned char * buf){
	flash_read_page(ota_program_offset + adr, len, buf);
}

unsigned int get_fw_len()
{
	unsigned int fw_len = 0;
	mesh_ota_read_data(0x18, 4, (unsigned char *)&fw_len);	// use flash read should be better
	return fw_len;
}

unsigned char get_ota_check_type()
{
    unsigned char ota_type[2] = {0};
    mesh_ota_read_data(6, sizeof(ota_type), ota_type);
	if(ota_type[0] == 0x5D){
		return ota_type[1];
	}
	return OTA_CHECK_TYPE_NONE;
}

unsigned int get_total_crc_type1_new_fw()
{
	unsigned int crc = 0;
	unsigned int len = get_fw_len();
	mesh_ota_read_data(len - 4, 4, (unsigned char *)&crc);
    return crc;
}

#define OTA_DATA_LEN_1      (16)    

int is_valid_ota_check_type1()
{	
	unsigned int crc_org = 0;
	unsigned int len = get_fw_len();
	mesh_ota_read_data(len - 4, 4, (unsigned char *)&crc_org);

    unsigned char buf[2 + OTA_DATA_LEN_1];
    unsigned int num = (len - 4 + (OTA_DATA_LEN_1 - 1))/OTA_DATA_LEN_1;
	unsigned int crc_new = 0;
    for(unsigned int i = 0; i < num; ++i){
    	buf[0] = i & 0xff;
    	buf[1] = (i>>8) & 0xff;
        mesh_ota_read_data((i * OTA_DATA_LEN_1), OTA_DATA_LEN_1, buf+2);
        if(!i){     // i == 0
             buf[2+8] = 0x4b;	// must
        }
        
        crc_new += crc16(buf, sizeof(buf));
        if(0 == (i & 0x0fff)){
			// about take 88ms for 10k firmware;
			#if (MODULE_WATCHDOG_ENABLE)
			wd_clear();
			#endif
        }
    }
    
    return (crc_org == crc_new);
}

unsigned int get_blk_crc_tlk_type1(unsigned char *data, unsigned int len, unsigned int addr)
{	
    unsigned char buf[2 + OTA_DATA_LEN_1];
    unsigned int num = len / OTA_DATA_LEN_1; // sizeof firmware data which exclude crc, is always 16byte aligned.
    //int end_flag = ((len % OTA_DATA_LEN_1) != 0);
	unsigned int crc = 0;
    for(unsigned int i = 0; i < num; ++i){
        unsigned int line = (addr / 16) + i;
    	buf[0] = line & 0xff;
    	buf[1] = (line>>8) & 0xff;
    	memcpy(buf+2, data + (i * OTA_DATA_LEN_1), OTA_DATA_LEN_1);
        
        crc += crc16(buf, sizeof(buf));
    }
    return crc;
}

int is_valid_telink_fw_flag()
{
    unsigned char fw_flag_telink[4] = {0x4B,0x4E,0x4C,0x54};
    unsigned char fw_flag[4] = {0};
    mesh_ota_read_data(8, sizeof(fw_flag), fw_flag);
    fw_flag[0] = 0x4B;
	if(!memcmp(fw_flag,fw_flag_telink, 4)){
		return 1;
	}
	return 0;
}

int is_valid_ota_calibrate_val()
{
    // eclipse crc32 calibrate
    if(is_valid_telink_fw_flag()){
    	if(OTA_CHECK_TYPE_TELINK_MESH == get_ota_check_type()){
            return is_valid_ota_check_type1();
        }
        return 1;
    }else{
	    return 0;
	}
}
#endif

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

	if((offset+length) > (ota_firmware_size_k * 4096))
	{
		printf("The write range is over OTA temporary!\r\n");
		return -1;
	}
    printf("offset=%d,len=%d\n", offset, length);

    // read back and check  later.
    flash_write_page(ota_program_offset + offset, length, (unsigned char *)buf);

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
    return is_valid_ota_calibrate_val(); // return true;
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
	ota_set_flag ();
	printf("%s called\n", __FUNCTION__);
	show_ota_result(0); // success

	hal_reboot();
}

