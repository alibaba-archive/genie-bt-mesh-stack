/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

/*******************************************************************************
 * INCLUDES
 */
#include "log.h"
#include "clock.h"
#include "drv_usart.h"
#include "pinmux.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <drv_wdt.h>
#include <aos/aos.h>
#include <aos/kernel.h>
#include "dw_gpio.h"
#include "drv_gpio.h"
#include <gpio.h>
#include "pin_name.h"
#include "drv_spiflash.h"
#include "spif.h"
#include "pin.h"
#include "dtm_test.h"
#include "dut_rf_test.h"
#include "dut_uart_driver.h"
#include "dut_utility.h"
#include "rf_phy_driver.h"
#include "flash.h"
/***************************************************************/
#define FREQ_FLASH_OFFSET   0x20
#define FREQ_LEN            4
#define FLASH_RSV_LEN       96
#define OK                  0
#define ERR                 -1
#define FCDS_START          0x11004000
#define SECRET_OFFSET       0x10

#define USER_ADDR_OFFSET    0x1000

//gpio config
#define AUTO_MODE       0
#define WRITE_MODE      1
#define READ_MODE       2
#define ERASE_MODE      3

extern dut_uart_cfg_t g_dut_uart_cfg;
static volatile uint8_t rx_scan_flag = 0;

static spiflash_handle_t g_spiflash_handle; 

extern aos_queue_t  g_dut_queue;
extern volatile uint8_t g_dut_queue_num; 

static int ewrite_flash(uint32_t  startaddr,const uint8_t *buffer, int length)
{
    g_spiflash_handle = csi_spiflash_initialize(0, NULL);
    if (g_spiflash_handle == NULL) {
        return -1;
    }

    int ret = csi_spiflash_erase_sector(g_spiflash_handle, startaddr);
    if (ret < 0) {
        printk("erase addr:%x err\r\n", startaddr);
        return -1;
    }
    ret = csi_spiflash_program(g_spiflash_handle, startaddr, buffer, length);
    if (ret < 0) {
        printk("write addr:%x err\r\n", startaddr);
        return -1;
    }
    return 0;
}

static int eread_flash(uint32_t startaddr, int offset, const uint8_t *buffer,int length)
{
    int ret;
    g_spiflash_handle = csi_spiflash_initialize(0, NULL);
    if (g_spiflash_handle == NULL) {
        return -1;
    }
    ret = csi_spiflash_read(g_spiflash_handle, startaddr + offset, buffer, length);
    if (ret < 0) {
        return ret;
    }

    return 0;
}
static int user_write_flash(uint32_t addr, const uint8_t *buffer,int length)
{
    int useaddr;
    int ret;
    hal_logic_partition_t *partition_info;

    partition_info = hal_flash_get_info(HAL_PARTITION_PARAMETER_3);
    if(partition_info == NULL){
        printf("get hal info err\r\n");
        return ERR;
    }
    if(addr < partition_info->partition_start_addr){
        printf("input addr is not partition partitionaddr:%x\r\n",partition_info->partition_start_addr);
        return ERR;
    }
    //printf("partiaddr:%x\r\n",partition_info->partition_start_addr);
    //useaddr = (addr - partition_info->partition_start_addr);
    //return  hal_flash_write(HAL_PARTITION_PARAMETER_3, &useaddr, buffer, length);

    g_spiflash_handle = csi_spiflash_initialize(0, NULL);
    if (g_spiflash_handle == NULL) {
        return -1;
    }
    ret = csi_spiflash_program(g_spiflash_handle, addr, buffer, length);
    if (ret < 0) {
        printk("write addr:%x err\r\n", addr);
        return -1;
    }
    return 0;
}
static int user_read_flash(uint32_t addr, const uint8_t *buffer,int length)
{
    int ret;
    uint32_t startaddr;
    uint32_t offset;
    hal_logic_partition_t *partition_info;

    partition_info = hal_flash_get_info(HAL_PARTITION_PARAMETER_3);
    if(partition_info == NULL){
        printf("get hal info err\r\n");
        return ERR;
    }
    if(addr < partition_info->partition_start_addr){
        printf("input addr is not partition partitionaddr:%x\r\n",partition_info->partition_start_addr);
        return ERR;
    }
    startaddr = partition_info->partition_start_addr;
    offset = addr - partition_info->partition_start_addr;
    if(offset > USER_ADDR_OFFSET){
        printf("offset > USER_ADDR_OFFSET \r\n");
        return ERR;
    }

    return eread_flash(startaddr,offset,buffer,length);
}

static int gpio_passthrough_write(uint8_t wrgpio,uint8_t value)
{
    int ret;
    gpio_pin_handle_t *pin = NULL;

    drv_pinmux_config(wrgpio, PIN_FUNC_GPIO);
    pin = csi_gpio_pin_initialize(wrgpio, NULL);

    ret = csi_gpio_pin_config_direction(pin,GPIO_DIRECTION_OUTPUT);
    if(ret < 0){
        return ret;
    }
    csi_gpio_pin_write(pin,value);
    return 0;
}

static int gpio_passthrough_read(uint8_t wrgpio,uint8_t *value)
{
    int ret;
    gpio_pin_handle_t *pin = NULL;

    drv_pinmux_config(wrgpio, PIN_FUNC_GPIO);
    pin = csi_gpio_pin_initialize(wrgpio, NULL);

    ret = csi_gpio_pin_config_direction(pin,GPIO_DIRECTION_INPUT);
    if(ret < 0){
        return ret;
    }
    csi_gpio_pin_read(pin,value);
    return 0;
}

void dut_at_cmd_init(void)
{
    rx_scan_flag = 0;
}

int dut_cmd_rx_mode(int argc, char *argv[])
{
    int sleep_time = 0;

    if (argc > 2) {
        return -1;
    }
    
    sleep_time = atoi(argv[1]);

    if ((sleep_time > 1800) || (sleep_time < 0)) {
        return -1;
    }

    if (sleep_time < 10) {
        sleep_time = 10;
    }

    enableSleepInPM(0xFF);

    rf_phy_dtm_init(NULL);

    rf_phy_set_dtmModeType(RF_PHY_DTM_MODE_RESET);

    rf_phy_dtm_trigged();

    rf_phy_set_dtmModeType(RF_PHY_DTM_MODE_RX_PER);
    rf_phy_set_dtmChan(0);
    rf_phy_set_dtmFreqOffSet(0);
    rf_phy_set_dtmLength(0x10);
    rf_phy_dtm_trigged();
    printf("START OK\r\n");
    aos_msleep(sleep_time);

    rf_phy_dtm_stop();

    disableSleepInPM(0xFF);
    boot_wdt_close();
    dut_uart_init(&g_dut_uart_cfg);
    return OK;
}

int dut_cmd_ftest(int argc, char *argv[])
{
    // printf("cmd ftest ");
    return OK;
}

int dut_cmd_sleep(int argc, char *argv[])
{
    enableSleepInPM(0xFF);
    printf("OK\r\n");
    //for P16,P17
    subWriteReg(0x4000f01c, 6, 6, 0x00); //disable software control
    rf_phy_sleep();
    return OK;
}

int dut_cmd_ireboot(int argc, char *argv[])
{
    if (argc > 2) {
        return -1;
    }

    if (argc == 2) {
        if ((atoi(argv[1]) != 0) && (atoi(argv[1]) != 1)) {
            return -1;
        }
    }

    printf("OK\r\n");
    aos_reboot();
    return OK;
}

int  fdut_cmd_opt_mac(int argc, char *argv[])
{
    int ret;
    uint8_t addr[6];

    if (argc == 1) {
        ret = eread_flash(FCDS_START, 0, addr, sizeof(addr));
        if (ret < 0) {
            return ret;
        }
        printf("mac:%02x:%02x:%02x:%02x:%02x:%02x\r\n", addr[4], addr[5], addr[0], addr[1], addr[2], addr[3]);
    }
    return OK;
}

static void _get_tri_tuple(void)
{
    genie_tri_tuple_load();
    genie_tri_tuple_show();
}

static void _set_tri_tuple(int argc, char **argv)
{
    uint32_t pid;
    uint8_t mac[6];
    uint8_t temp_data[FLASH_RSV_LEN];
    uint8_t key[16];
    uint8_t ret;

    if (argc != 4){
        printk("para error!!!\n");
        return;
    }
    //pro_id
    pid = atol(argv[1]);
    //key
    ret = stringtohex(argv[2], key, 16);
    //addr
    ret = stringtohex(argv[3], mac, 6);
    //
    if (eread_flash(FCDS_START, 0, temp_data, sizeof(temp_data)) < 0) {
        return ;
    }

    temp_data[0] = mac[2];
    temp_data[1] = mac[3];
    temp_data[2] = mac[4];
    temp_data[3] = mac[5];
    temp_data[4] = mac[0];
    temp_data[5] = mac[1];

    memcpy((temp_data + SECRET_OFFSET),key,sizeof(key));
    if (ewrite_flash(FCDS_START, (uint8_t *)temp_data, sizeof(temp_data)) < 0) {
        return ;
    }
    ret = genie_flash_write_trituple(&pid, mac, key);

    _get_tri_tuple();
}

int  dut_cmd_trituple_component(int argc, char *argv[])
{
    int ret;

    genie_flash_init();

    if (argc == 1) {
        printf("+TURTUPLE:");
        _get_tri_tuple();
    }else if(argc == 4){
        _set_tri_tuple(argc,argv);
    }else{
        return -1;
    }
    return OK;
}

int  fdut_cmd_trituple_component(int argc, char *argv[])
{
    int ret;

    genie_flash_init();
    if (argc == 1) {
        printf("+TURTUPLE:");
        _get_tri_tuple();
    }
    return OK;
}

int  dut_cmd_flash(int argc, char *argv[])
{
    int ret;
    int mode;
    int len;
    uint8_t addr[4];
    uint32_t  useraddr;
    uint32_t  readdata;
    uint8_t data[128];

    if ((argc <= 1)||(argc > 5)) {
        return ERR;
    } 
    mode = atoi(argv[1]);
    if(mode > 3){
        return ERR;
    }

    if((argc == 3) && (mode != READ_MODE)){
        return ERR;
    }

    stringtohex(argv[2],addr, 4); 
    useraddr = (addr[0] << 24) | (addr[1] << 16) | (addr[2] << 8) | addr[3];

    if(AUTO_MODE == mode){
        return OK;
    }else if(WRITE_MODE == mode){
        len = atoi(argv[3]);
        stringtohex(argv[4],data, len);
        //printf("len:%d,addr:%x,\r\n",len,useraddr);
        return user_write_flash(useraddr,data,len);
    }else if(READ_MODE == mode){
        len = atoi(argv[3]);
        //printf("len:%x,addr:%x\r\n",len,useraddr);
        ret = user_read_flash(useraddr,data,sizeof(data));
        if(ret){
            return ret;
        }
        printf("+FLASH:");
        for(int i = 0 ;i < len;i++){
            printf("%x ", data[i]);
        }
        printf("\r\n");
    }else if(ERASE_MODE == mode){
        g_spiflash_handle = csi_spiflash_initialize(0, NULL);
        if (g_spiflash_handle == NULL) {
            return ERR;
        }
        int ret = csi_spiflash_erase_sector(g_spiflash_handle, useraddr);
        if (ret < 0) {
            printk("erase addr:%x err\r\n", useraddr);
            return ERR;
        }
        return 0;
    }else{
        return ERR;
    }
    return OK;
}

int  fdut_cmd_flash(int argc, char *argv[])
{
    return OK;
}

int  dut_cmd_gpio(int argc, char *argv[])
{
    int ret;
    int mode;
    uint8_t data;

    if ((argc <= 1)||(argc > 4)){
        return ERR;
    }
    mode = atoi(argv[1]);
    if (mode > 2) {
        return ERR;
    }
    if((argc == 3) && (mode != READ_MODE)){
        return ERR;
    }
    if(AUTO_MODE == mode){
        return OK;
    }else if(WRITE_MODE == mode){
        return gpio_passthrough_write(atoi(argv[2]),atoi(argv[3]));
    }else if(READ_MODE == mode){
        ret = gpio_passthrough_read(atoi(argv[2]), &data);
        if(ret){
            return ret;
        }
        printf("+GPIO:%d\r\n", data);
    }else{
        return ERR;
    }
    return OK;
}

int  fdut_cmd_gpio(int argc, char *argv[])
{
    int ret;
    uint8_t data;

    if ((argc == 3) && (READ_MODE == atoi(argv[1]))) {
        int ret = gpio_passthrough_read(atoi(argv[2]), &data);
        if(ret){
            return ret;
        }
        printf("+GPIO:%d\r\n", data);
    }
    return OK;
}

int  dut_cmd_freq_off(int argc, char *argv[])
{
    int ret;
    char  freq[FLASH_RSV_LEN];
    int16_t wfreq = 0;

    memset(freq, 0, sizeof(freq));
  
    ret = eread_flash(FCDS_START, 0, freq, sizeof(freq));
    if (ret < 0) {
        return ret;
    }

    if (argc == 1) {
        if (freq[FREQ_FLASH_OFFSET] == 0xff) {
            printf("Not Set Freq_Off\r\n");
            return -1;
        }
        printf("+FREQ_OFF=%d Khz\r\n", ((int8_t)(freq[FREQ_FLASH_OFFSET])) * 4);
    }

    if (argc == 2) {
        if ((strlen(argv[1]) > FREQ_LEN) && (int_num_check(argv[1]) == -1)) {
            return -1;
        }

        freq[FREQ_FLASH_OFFSET] = 0;
        wfreq = atoi((char *)(argv[1]));

        if (wfreq < -200) {
            wfreq = -200;
        } else if (wfreq > 200) {
            wfreq = 200;
        } else {}

        wfreq = wfreq - wfreq % 20;
        freq[FREQ_FLASH_OFFSET] = (char)(wfreq / 4);

        if (ewrite_flash(FCDS_START, (uint8_t *)(freq), sizeof(freq)) < 0) {
            return -1;
        }
    }

    return  OK;
}

int  fdut_cmd_freq_off(int argc, char *argv[])
{
    int ret;
    char  freq[FREQ_LEN];

    memset(freq, 0, sizeof(freq));
    ret = eread_flash(FCDS_START, FREQ_FLASH_OFFSET, freq, sizeof(freq));
    if (ret < 0) {
        return ret;
    }

    if (argc == 1) {
        printf("+FREQ_OFF=%d Khz\r\n", ((int8_t)(freq[0])) * 4);
    }

    return  0;
}

int  dut_cmd_opt_mac(int argc, char *argv[])
{
    int ret;
    int err;
    uint8_t addr[6];
    uint8_t  temp_data[FLASH_RSV_LEN];

    ret = eread_flash(FCDS_START, 0, temp_data, sizeof(temp_data));
    if (ret < 0) {
        return ret;
    }

    if (argc == 1) {
        printf("mac:%02x:%02x:%02x:%02x:%02x:%02x\r\n", temp_data[4], temp_data[5], temp_data[0], temp_data[1], temp_data[2], temp_data[3]);
    }

    if (argc == 2) {
        printf("%s\r\n", argv[1]);
        err = str2_char(argv[1], addr);

        if (err < 0) {
            return err;
        }

        if (err) {
            printf("Invalid address\n");
            return err;
        }

        temp_data[0] = addr[2];
        temp_data[1] = addr[3];
        temp_data[2] = addr[4];
        temp_data[3] = addr[5];
        temp_data[4] = addr[0];
        temp_data[5] = addr[1];

        if (ewrite_flash(FCDS_START, (uint8_t *)temp_data, sizeof(temp_data)) < 0) {
            return -1;
        }
    }

    return  0;
}

