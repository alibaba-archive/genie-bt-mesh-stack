/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "soc.h"
#include "drv_adc.h"
#include "adc.h"
#include "pin.h"
#include "ringbuffer.h"
#include "hal/soc/adc.h"
#include "aos/kernel.h"

static adc_handle_t g_adc_handle;
#define BATTERY_ADC_CON_DATA_NUM        64
static uint32_t ch_array[1] = {0};

/**
 * Initialises an ADC interface, Prepares an ADC hardware interface for sampling
 *
 * @param[in]  adc  the interface which should be initialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_adc_init(adc_dev_t *adc)
{
	adc_conf_t sconfig = {0};
	if (adc == NULL) {
		printf("adc err param\n");
		return -1;
	}

	if (adc->port == P13) {
		ch_array[0] = ADC_CH2N_P13;
		sconfig.link_internal_voltage_channel = ADC_CH2N_P13;	
	    sconfig.enable_link_internal_voltage = 1;
	} else if (adc->port == P11) {
		ch_array[0] = ADC_CH1N_P11;
		sconfig.link_internal_voltage_channel = ADC_CH1N_P11;		
    	sconfig.enable_link_internal_voltage = 1;
	} else if (adc->port == P12) {
		ch_array[0] = ADC_CH1P_P12;
		sconfig.link_internal_voltage_channel = ADC_CH1P_P12;
		sconfig.enable_link_internal_voltage = 1;
	} else if (adc->port == P14) {
		ch_array[0] = ADC_CH2P_P14;
		sconfig.link_internal_voltage_channel = ADC_CH2P_P14;
		sconfig.enable_link_internal_voltage = 1;
	} else if (adc->port == P15) {
		ch_array[0] = ADC_CH3N_P15;
	} else if (adc->port == P20) {
		ch_array[0] = ADC_CH3P_P20;
	} else {
		printf("adc err param\n");
		return -1;
	}
	
	drv_pinmux_config(adc->port, ADCC);
	g_adc_handle = drv_adc_initialize(0, NULL);
	if (g_adc_handle == NULL) {
		printf("adc init fail\n");
		return -1;
	}

    sconfig.mode = ADC_CONTINUOUS;
    sconfig.trigger = 0;
    sconfig.intrp_mode = 0;
    sconfig.channel_array = ch_array;
    sconfig.channel_nbr = 1;
    sconfig.sampling_time = adc->config.sampling_cycle;//32000;
    sconfig.conv_cnt = BATTERY_ADC_CON_DATA_NUM; 

	if (drv_adc_config(g_adc_handle, &sconfig) != 0) {
		printf("adc config fail\n");
		return -1;
	}

	return 0;
}

/**
 * Takes a single sample from an ADC interface
 *
 * @param[in]   adc      the interface which should be sampled
 * @param[out]  output   pointer to a variable which will receive the sample
 * @param[in]   timeout  ms timeout
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_adc_value_get(adc_dev_t *adc, void *output, uint32_t timeout)
{
	int ret = -1;
	uint32_t recv_data[BATTERY_ADC_CON_DATA_NUM] = {0};
	int total = 0;
	
	if (adc == NULL || g_adc_handle == NULL) {
		return -1;
	}
	
    ret = drv_adc_start(g_adc_handle);
    if (ret < 0) {
        printf("adc start fail\n");
        return -1;
    }

    ret = drv_adc_read(g_adc_handle, recv_data, BATTERY_ADC_CON_DATA_NUM);
    if (ret < 0) {
        printf("adc read fail - %d\n", ret);
        return -1;
    }

    ret = drv_adc_stop(g_adc_handle);
    if (ret < 0) {
        printf("adc stop fail\n");
        return -1;
    }
    total = phy_adc_value_cal(ch_array[0], recv_data, BATTERY_ADC_CON_DATA_NUM, false, false);
	memcpy(output, &total, sizeof(total));
	return 0;
}

/**
 * De-initialises an ADC interface, Turns off an ADC hardware interface
 *
 * @param[in]  adc  the interface which should be de-initialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_adc_finalize(adc_dev_t *adc)
{
	if (adc == NULL || g_adc_handle == NULL) {
		return -1;
	}
	drv_adc_uninitialize(g_adc_handle);
	return 0;
}

