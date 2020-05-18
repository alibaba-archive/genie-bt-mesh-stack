/* helloworld.c - helloworld */

/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>
#include <pwrmgr.h>
#include <hal/soc/gpio.h>
#include <hal/soc/i2c.h>
#include <soc.h>

#define CK_IIC_SLAVE_ADDR          0x50
#define EEPROM_PAGE_SIZE          0x20
#define AOS_SLEEP 0XFF

static i2c_dev_t i2c_dev = {
    0,
    {I2C_MEM_ADDR_SIZE_8BIT, I2C_BUS_SPEED_100K, I2C_MODE_MASTER, CK_IIC_SLAVE_ADDR},
    NULL
};


int pm_prepare_sleep_action()
{
    hal_ret_sram_enable(RET_SRAM0 | RET_SRAM1 | RET_SRAM2 | RET_SRAM3 | RET_SRAM4);
    csi_gpio_prepare_sleep_action();
    csi_pinmux_prepare_sleep_action();
    csi_usart_prepare_sleep_action(0);
    csi_iic_prepare_sleep_action(0);
    return 0;
}

int pm_after_sleep_action()
{
    csi_gpio_wakeup_sleep_action();
    csi_pinmux_wakeup_sleep_action();
    csi_usart_wakeup_sleep_action(0);
    csi_iic_wakeup_sleep_action(0);
    return 0;
}


int application_start(int argc, char **argv)
{
    int32_t ret;
    printk("BUILD TIME:%s\n", __DATE__","__TIME__);

    uint8_t write_data[EEPROM_PAGE_SIZE + 2] = {0x0, EEPROM_PAGE_SIZE, 0};
    uint8_t read_data[EEPROM_PAGE_SIZE + 2] = {0x0, EEPROM_PAGE_SIZE, 0};

    pm_init();

    uint8_t i = 0;

    for (i = 2; i < sizeof(write_data); i++) {
        write_data[i] = i - 2;
    }

    ret = hal_i2c_init(&i2c_dev);

    if (ret != 0) {
        printk("hal_i2c_init error\n");
        return -1;
    }

    while (1) {
        aos_msleep(10000);

        disableSleepInPM(AOS_SLEEP);
        ret = hal_i2c_master_send(&i2c_dev, CK_IIC_SLAVE_ADDR, write_data, sizeof(write_data), 3000);

        if (ret < 0) {
            printf("hal_i2c_master_send1 error\n");
            continue;
        }

        mdelay(5);
        ret = hal_i2c_master_send(&i2c_dev, CK_IIC_SLAVE_ADDR, read_data, 2, 3500);

        if (ret < 0) {
            printf("hal_i2c_master_send2 error\n");
        }

        mdelay(5);
        ret = hal_i2c_master_recv(&i2c_dev, CK_IIC_SLAVE_ADDR, read_data + 2, sizeof(read_data) - 2, 3000);

        if (ret < 0) {
            printf("hal_i2c_master_recv error\n");
        }

        printf("write_data,read_data:\n");

        for (i = 2; i < EEPROM_PAGE_SIZE + 2; i++) {
            printf("%x,%x \t", write_data[i], read_data[i]);

            if (((i + 3) % 4) == 0) {
                printf("\n");
            }

            if (write_data[i] != read_data[i]) {
                printf("\ntest at24c64 write and read failed\n");
            }
        }

        enableSleepInPM(AOS_SLEEP);

    }

    return 0;
}

