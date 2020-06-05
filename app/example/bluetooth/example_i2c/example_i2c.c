/* helloworld.c - helloworld */

/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include <hal/soc/gpio.h>
#include <hal/soc/pwm.h>
#include <hal/soc/i2c.h>

#include <soc.h>

#define CK_IIC_SLAVE_ADDR         0x50
#define EEPROM_PAGE_SIZE          0x20

#define DEVICE_ZERO               0x0
#define DEVICE_ONE                0x1

static i2c_dev_t i2c_dev0 = {
    DEVICE_ZERO,
    {I2C_MEM_ADDR_SIZE_8BIT,I2C_BUS_SPEED_100K, I2C_MODE_MASTER, CK_IIC_SLAVE_ADDR},
    NULL
};

static i2c_dev_t i2c_dev1 = {
    DEVICE_ONE,
    {I2C_MEM_ADDR_SIZE_8BIT,I2C_BUS_SPEED_100K, I2C_MODE_MASTER, CK_IIC_SLAVE_ADDR},
    NULL
};

static int test_iic_eeprom()
{
    // write or read: ADDRHIGH ADDRLOW 
    uint8_t write_data[EEPROM_PAGE_SIZE + 2] = {0x00, 0x00, 0};
    uint8_t read_data[EEPROM_PAGE_SIZE + 2] = {0x00, 0x00, 0};
    uint8_t i = 0;
    int32_t ret;

    for (i = 2; i < sizeof(write_data); i++) {
        write_data[i] = 1;
    }

    ret = hal_i2c_init(&i2c_dev0);
    if (ret != 0) {
        printf("hal_i2c_init dev0 error\n");
        return -1;
    }

    ret = hal_i2c_init(&i2c_dev1);
    if (ret != 0) {
        printf("hal_i2c_init dev1 error\n");
        return -1;
    }

    ret = hal_i2c_master_send(&i2c_dev0, CK_IIC_SLAVE_ADDR, write_data, sizeof(write_data), 3000);
    if (ret < 0) {
        printf("hal_i2c_master_send1 error\n");
        return -1;
    }
    mdelay(5);
    ret = hal_i2c_master_send(&i2c_dev0, CK_IIC_SLAVE_ADDR, read_data, 2, 3000);
    if (ret < 0) {
        printf("hal_i2c_master_send2 error\n");
        return -1;
    }
    mdelay(5);
    ret = hal_i2c_master_recv(&i2c_dev0, CK_IIC_SLAVE_ADDR, read_data + 2, sizeof(read_data) - 2, 3000);
    if (ret < 0) {
        printf("hal_i2c_master_recv error\n");
        return -1;
    }

    printf("write_data0,read_data0:\n");
    for (i = 2; i < EEPROM_PAGE_SIZE + 2; i++) {
        printf("%x,%x \t", write_data[i], read_data[i]);
        if (((i + 3) % 4) == 0) {
            printf("\n");
        }
        if (write_data[i] != read_data[i]) {
            printf("\ntest at24c64 dev0 write and read failed\n");
            return -1;
        }
    }

    for (i = 2; i < sizeof(write_data); i++) {
        write_data[i] = 2;
    }

    ret = hal_i2c_master_send(&i2c_dev1, CK_IIC_SLAVE_ADDR, write_data, sizeof(write_data), 3000);
    if (ret < 0) {
        printf("hal_i2c_master_send1 error\n");
        return -1;
    }
    mdelay(5);
    ret = hal_i2c_master_send(&i2c_dev1, CK_IIC_SLAVE_ADDR, read_data, 2, 3000);
    if (ret < 0) {
        printf("hal_i2c_master_send2 error\n");
        return -1;
    }
    mdelay(5);
    ret = hal_i2c_master_recv(&i2c_dev1, CK_IIC_SLAVE_ADDR, read_data + 2, sizeof(read_data) - 2, 3000);
    if (ret < 0) {
        printf("hal_i2c_master_recv error\n");
        return -1;
    }

    printf("write_data1,read_data1:\n");
    for (i = 2; i < EEPROM_PAGE_SIZE + 2; i++) {
        printf("%x,%x \t", write_data[i], read_data[i]);
        if (((i + 3) % 4) == 0) {
            printf("\n");
        }
        if (write_data[i] != read_data[i]) {
            printf("\ntest at24c64 dev1 write and read failed\n");
            return -1;
        }
    }
    printf("\ntest at24c64 dev1 write and read successfully\n");

    ret = hal_i2c_finalize(&i2c_dev0);
    if (ret < 0) {
        printf("hal_i2c_finalize error\n");
        return -1;
    }

    ret = hal_i2c_finalize(&i2c_dev1);
    if (ret < 0) {
        printf("hal_i2c_finalize error\n");
        return -1;
    }
    printf("test_iic_eeprom successfully\n");
    return 0;
}

int application_start(int argc, char **argv)
{
    printk("BUILD TIME:%s\n", __DATE__","__TIME__);

    test_iic_eeprom();

    //aos init
#ifndef BOARD_TC825X    // telink confirm later: don't use this, because we will run while(1) of sys_init later.
    aos_loop_run();
#endif

    return 0;
}

