/* helloworld.c - helloworld */

/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <aos/aos.h>
#include <aos/kernel.h>
#include "soc.h"
#include "drv_adc.h"
#include "adc.h"
#include "pin.h"
#include <soc.h>

#define TEST_ADC_SCAN_CHANNEL_NUM     2

#define TEST_SINGLE_CONV_TIMES        1
#define TEST_CONTINOUS_CONV_TIMES     16
#define TEST_SCAN_CONV_TIMES          1

#define SINGE_DATA_NUM                1
#define CONTINOUS_DATA_NUM            (1*TEST_CONTINOUS_CONV_TIMES)

#define ADC_TEST_SUCCESS              0
#define ADC_TEST_FAIL                 -1
#define ADC_DELAY                     1000

extern void mdelay(uint32_t ms);

static int test_adc_continous(void)
{
    adc_conf_t sconfig;
    adc_handle_t hd;
    uint32_t data[CONTINOUS_DATA_NUM];
    uint32_t ch_array[1] = {ADC_CH3P_P20};
    int i = 0;
    int ret = 0;

    sconfig.mode = ADC_CONTINUOUS;
    sconfig.trigger = 0;
    sconfig.intrp_mode = 0;
    sconfig.channel_array = ch_array;
    sconfig.channel_nbr = 1;
    sconfig.conv_cnt = TEST_CONTINOUS_CONV_TIMES;
    sconfig.enable_link_internal_voltage = 0;

    drv_pinmux_config(P20, ADCC);

    for (i = 0; i < CONTINOUS_DATA_NUM; i++) {
        data[i] = 0x31415;
    }

    hd = drv_adc_initialize(0, NULL);

    if (!hd) {
        printf("adc initial failed\n\r");
        return ADC_TEST_FAIL;
    }

    ret = drv_adc_config(hd, &sconfig);

    if (ret == (CSI_DRV_ERRNO_ADC_BASE | DRV_ERROR_UNSUPPORTED)) {
        printf("continous mode unsupported\n");
        return ADC_TEST_SUCCESS;
    }

    if (ret != 0) {
        printf("adc config failed\n\r");
        return ADC_TEST_FAIL;
    }


    if (drv_adc_start(hd) != 0) {
        printf("adc start failed\n\r");
        return ADC_TEST_FAIL;
    }

    ret = drv_adc_read(hd, &data[0], CONTINOUS_DATA_NUM);
    printf("read ad ret = %d\n\r", ret);

    printf("read a data from adc :\n\r");

    for (i = 0; i < CONTINOUS_DATA_NUM; i++) {
        printf("0x%x\n\r", data[i]);
    }

    if (drv_adc_stop(hd) != 0) {
        printf("adc stop failed\n");
        return ADC_TEST_FAIL;
    }

    if (drv_adc_uninitialize(hd) != 0) {
        printf("adc uninitial failed\n");
        return ADC_TEST_FAIL;
    }

    return ADC_TEST_SUCCESS;
}

int example_adc(void)
{
    int ret;

    ret = test_adc_continous();

    if (ret < 0) {
        printf("test adc continous mode failed\n\r");
        return ADC_TEST_FAIL;
    }

    printf("test adc continous mode passed\n\r");

    printf("test adc successfully\n\r");
    return ADC_TEST_SUCCESS;
}

int application_start(int argc, char **argv)
{
    printk("BUILD TIME:%s\n", __DATE__","__TIME__);

    example_adc();

    //aos init
    aos_loop_run();
    return 0;
}

