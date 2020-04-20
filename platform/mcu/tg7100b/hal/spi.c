/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "soc.h"
#include "drv_spi.h"
#include "pin.h"
#include "hal/soc/spi.h"
#include "aos/kernel.h"


static spi_handle_t g_spi_handle[CONFIG_SPI_NUM];

/**
 * Initialises the SPI interface for a given SPI device
 *
 * @param[in]  spi  the spi device
 *
 * @return  0 : on success, EIO : if the SPI device could not be initialised
 */
int32_t hal_spi_init(spi_dev_t *spi)
{
	if (spi->port > CONFIG_SPI_NUM) {
		return -1;
	}

	g_spi_handle[spi->port] = csi_spi_initialize(spi->port, NULL);
	if (g_spi_handle[spi->port] == NULL) {
		return -1;
	}

	return csi_spi_config(g_spi_handle[spi->port], spi->config.freq, spi->config.mode, SPI_FORMAT_CPOL0_CPHA0, SPI_ORDER_MSB2LSB, SPI_SS_MASTER_SW, 8);
}

/**
 * Spi send
 *
 * @param[in]  spi      the spi device
 * @param[in]  data     spi send data
 * @param[in]  size     spi send data size
 * @param[in]  timeout  timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                      if you want to wait forever
 *
 * @return  0 : on success, EIO : if the SPI device could not be initialised
 */
int32_t hal_spi_send(spi_dev_t *spi, const uint8_t *data, uint16_t size, uint32_t timeout)
{
	if (g_spi_handle[spi->port] == NULL) {
		return -1;
	}

	return csi_spi_send(g_spi_handle[spi->port], data, size);
 }

/**
 * spi_recv
 *
 * @param[in]   spi      the spi device
 * @param[out]  data     spi recv data
 * @param[in]   size     spi recv data size
 * @param[in]  timeout   timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                       if you want to wait forever
 *
 * @return  0 : on success, EIO : if the SPI device could not be initialised
 */
int32_t hal_spi_recv(spi_dev_t *spi, uint8_t *data, uint16_t size, uint32_t timeout)
{
	if (g_spi_handle[spi->port] == NULL) {
		return -1;
	}

	return csi_spi_receive(g_spi_handle[spi->port], data, size);
}

/**
 * spi send data and recv
 *
 * @param[in]  spi      the spi device
 * @param[in]  tx_data  spi send data
 * @param[in]  rx_data  spi recv data
 * @param[in]  size     spi data to be sent and recived
 * @param[in]  timeout  timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                      if you want to wait forever
 *
 * @return  0, on success;  EIO : if the SPI device could not be initialised
 */
int32_t hal_spi_send_recv(spi_dev_t *spi, uint8_t *tx_data, uint8_t *rx_data,
                          uint16_t size, uint32_t timeout)
{
	if (g_spi_handle[spi->port] == NULL) {
		return -1;
	}

	return csi_spi_transfer(g_spi_handle[spi->port], tx_data, rx_data, size, size);
}

/**
 * De-initialises a SPI interface
 *
 *
 * @param[in]  spi the SPI device to be de-initialised
 *
 * @return  0 : on success, EIO : if an error occurred
 */
int32_t hal_spi_finalize(spi_dev_t *spi)
{
	if (g_spi_handle[spi->port] == NULL) {
		return -1;
	}
	
	return csi_spi_uninitialize(g_spi_handle[spi->port]);
}

