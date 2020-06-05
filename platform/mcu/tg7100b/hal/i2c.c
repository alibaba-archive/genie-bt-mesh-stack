/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "soc.h"
#include "gpio.h"
#include "drv_iic.h"
#include "pin.h"
#include "ringbuffer.h"
#include "hal/soc/i2c.h"
#include "aos/kernel.h"

#define CHIP_I2C_NUM_BUILDIN    2
static iic_handle_t g_i2c_handle[CHIP_I2C_NUM_BUILDIN];

struct {
    aos_sem_t sem;
    uint8_t sda;
    uint8_t scl;
    uint8_t sdamux;
    uint8_t sclmux;
    uint8_t transfer_flag:1;
} i2c_config[CHIP_I2C_NUM_BUILDIN] = {
    {{0}, P31, P32, IIC0_SDA, IIC0_SCL, 0},
    {{0}, P20, P24, IIC1_SDA, IIC1_SCL, 0}
};

static void iic_event_cb_fun(int32_t idx, iic_event_e event)
{
    if (event == IIC_EVENT_TRANSFER_DONE) {
        aos_sem_signal(&i2c_config[idx].sem);
        i2c_config[idx].transfer_flag = 1;
    }
}

/**
 * Initialises an I2C interface
 * Prepares an I2C hardware interface for communication as a master or slave
 *
 * @param[in]  i2c  the device for which the i2c port should be initialised
 *
 * @return  0 : on success, EIO : if an error occurred during initialisation
 */
int32_t hal_i2c_init(i2c_dev_t *i2c)
{
    int ret;
	if (i2c == NULL || i2c->port >= CHIP_I2C_NUM_BUILDIN) {
		printf("i2c err param\n");
		return -1;
	}
    drv_pinmux_config(i2c_config[i2c->port].scl, i2c_config[i2c->port].sclmux);
    drv_pinmux_config(i2c_config[i2c->port].sda, i2c_config[i2c->port].sdamux);

	g_i2c_handle[i2c->port] = csi_iic_initialize(i2c->port, iic_event_cb_fun);
	if (g_i2c_handle[i2c->port] == NULL) {
		printf("i2c[%d] init fail\n", i2c->port);
		return -1;
	}
    iic_mode_e mode;
    iic_speed_e speed = i2c->config.freq;
    iic_address_mode_e addr_mode;
    int32_t slave_addr = i2c->config.dev_addr;

    if (i2c->config.mode == I2C_MODE_MASTER){
        mode = IIC_MODE_MASTER;
    } else {
        mode = IIC_MODE_SLAVE;
    }

    if (i2c->config.address_width == I2C_MEM_ADDR_SIZE_8BIT) {
        addr_mode = IIC_ADDRESS_7BIT;
    } else {
        addr_mode = IIC_ADDRESS_10BIT;
    }
    ret = csi_iic_config(g_i2c_handle[i2c->port], mode, speed, addr_mode, slave_addr);
    if (ret) {
        return ret;
    }
    i2c_config[i2c->port].transfer_flag = 0;
    aos_sem_new(&i2c_config[i2c->port].sem, 0);
	return 0;
}

/**
 * I2c master send
 *
 * @param[in]  i2c       the i2c device
 * @param[in]  dev_addr  device address
 * @param[in]  data      i2c send data
 * @param[in]  size      i2c send data size
 * @param[in]  timeout   timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                       if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred during initialisation
 */
int32_t hal_i2c_master_send(i2c_dev_t *i2c, uint16_t dev_addr, const uint8_t *data,
                            uint16_t size, uint32_t timeout)
{
    int ret;
	if (i2c == NULL || i2c->port >= 2 || (g_i2c_handle[i2c->port] == NULL)) {
		printf("i2c err param\n");
		return -EIO;
	}

    i2c_config[i2c->port].transfer_flag = 0;
    ret = csi_iic_master_send(g_i2c_handle[i2c->port], dev_addr, data, size, 1);
    if (ret < 0) {
        printf("csi_iic_master_send error\n");
        return -EIO;
    }

    ret = aos_sem_wait(&i2c_config[i2c->port].sem, timeout);
    if (ret != 0) {
        printf("timeout\n");
        return -EIO;
    }

    if (i2c_config[i2c->port].transfer_flag == 1) {
        ret = 0;
    } else {
        ret = -EIO;
    }
    i2c_config[i2c->port].transfer_flag = 0;
    return ret;
}

/**
 * I2c master recv
 *
 * @param[in]   i2c       the i2c device
 * @param[in]   dev_addr  device address
 * @param[out]  data      i2c receive data
 * @param[in]   size      i2c receive data size
 * @param[in]   timeout   timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                        if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred during initialisation
 */
int32_t hal_i2c_master_recv(i2c_dev_t *i2c, uint16_t dev_addr, uint8_t *data,
                            uint16_t size, uint32_t timeout)
{
    int ret;
	if (i2c == NULL || i2c->port >= 2) {
		printf("i2c err param\n");
		return -EIO;
	}

    i2c_config[i2c->port].transfer_flag = 0;
    ret = csi_iic_master_receive(g_i2c_handle[i2c->port], dev_addr, data, size, 0);
    if (ret < 0) {
        printf("csi_iic_master_recv error\n");
        return -EIO;
    }

    ret = aos_sem_wait(&i2c_config[i2c->port].sem, timeout);
    if (ret != 0) {
        printf("timeout\n");
        return -EIO;
    }

    if (i2c_config[i2c->port].transfer_flag == 1) {
        ret = 0;
    } else {
        ret = -EIO;
    }
    i2c_config[i2c->port].transfer_flag = 0;
    return ret;
}

/**
 * I2c slave send
 *
 * @param[in]  i2c      the i2c device
 * @param[in]  data     i2c slave send data
 * @param[in]  size     i2c slave send data size
 * @param[in]  timeout  timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                      if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred during initialisation
 */
int32_t hal_i2c_slave_send(i2c_dev_t *i2c, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    int ret;
	if (i2c == NULL || i2c->port >= 2) {
		printf("i2c err param\n");
		return -1;
	}

    i2c_config[i2c->port].transfer_flag = 0;
    ret = csi_iic_slave_send(g_i2c_handle[i2c->port], data, size);
    if (ret < 0) {
        printf("csi_iic_slave_send error\n");
        return -1;
    }

    ret = aos_sem_wait(&i2c_config[i2c->port].sem, timeout);
    if (ret != 0) {
        printf("timeout\n");
        return -1;
    }

    if (i2c_config[i2c->port].transfer_flag == 1) {
        ret = 0;
    } else {
        ret = -1;
    }
    i2c_config[i2c->port].transfer_flag = 0;
    return ret;
}

/**
 * I2c slave receive
 *
 * @param[in]   i2c      tthe i2c device
 * @param[out]  data     i2c slave receive data
 * @param[in]   size     i2c slave receive data size
 * @param[in]  timeout   timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                       if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred during initialisation
 */
int32_t hal_i2c_slave_recv(i2c_dev_t *i2c, uint8_t *data, uint16_t size, uint32_t timeout)
{
    int ret;
	if (i2c == NULL || i2c->port >= 2) {
		printf("i2c err param\n");
		return -1;
	}

    i2c_config[i2c->port].transfer_flag = 0;
    ret = csi_iic_slave_receive(g_i2c_handle[i2c->port], data, size);
    if (ret < 0) {
        printf("csi_iic_slave_send error\n");
        return -1;
    }

    ret = aos_sem_wait(&i2c_config[i2c->port].sem, timeout);
    if (ret != 0) {
        printf("timeout\n");
        return -1;
    }

    if (i2c_config[i2c->port].transfer_flag == 1) {
        ret = 0;
    } else {
        ret = -1;
    }
    i2c_config[i2c->port].transfer_flag = 0;
    return ret;

}

/**
 * Deinitialises an I2C device
 *
 * @param[in]  i2c  the i2c device
 *
 * @return  0 : on success, EIO : if an error occurred during deinitialisation
 */
int32_t hal_i2c_finalize(i2c_dev_t *i2c)
{
    int ret;
	if (i2c == NULL || i2c->port >= 2) {
		printf("i2c err param\n");
		return -1;
	}
    aos_sem_free(&i2c_config[i2c->port].sem);
    i2c_config[i2c->port].transfer_flag = 0;

    ret = csi_iic_uninitialize(g_i2c_handle[i2c->port]);

    if (ret < 0) {
        printf("csi_iic_uninitialize error\n");
        return -1;
    }
    
    return 0;
}

