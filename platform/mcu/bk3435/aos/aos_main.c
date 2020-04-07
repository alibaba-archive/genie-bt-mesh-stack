/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <aos_main.h>
#include <k_api.h>
#include <aos/kernel.h>
#include <aos/init.h>
#include "icu_pub.h"
#include "gpio.h"
#include "gpio_pub.h"
#include "fake_clock_pub.h"

#define AOS_START_STACK 320

extern void board_init(void);

static kinit_t kinit = {
    .argc = 0,
    .argv = NULL,
    .cli_enable = 1
};

static void sys_init(void)
{
    int i = 0;

    soc_system_init();

#ifdef BOOTLOADER
    main();
#else

    board_init();

#if 0
    ///initial LED gpio
    static gpio_dev_t led_pin = {GPIO_P10, OUTPUT_PUSH_PULL, NULL};
    hal_gpio_init(&led_pin);
    hal_gpio_output_high(&led_pin); /// LED off
#endif

#if 1
	printf("The APP code build at %s, %s\r\n\n", __TIME__, __DATE__);
#endif

    aos_components_init(&kinit);
    aos_kernel_init(&kinit);
#endif
}

static cpu_stack_t aos_app_stack[AOS_START_STACK];
static ktask_t aos_app_task_obj;

void sys_start(void)
{
    aos_init();

    soc_driver_init();

    UINT32 param = ICU_MCU_CLK_SEL_64M;
    sddev_control(ICU_DEV_NAME, CMD_ICU_MCU_CLK_SEL, &param);

    fclk_init(FCLK_PWM_ID, RHINO_CONFIG_TICKS_PER_SECOND);

    //krhino_task_dyn_create(&g_aos_init, "aos-app", 0, AOS_DEFAULT_APP_PRI, 0, AOS_START_STACK, sys_init, 1);
    krhino_task_create(&aos_app_task_obj, "aos-app", NULL, AOS_DEFAULT_APP_PRI, 0,
                       aos_app_stack, sizeof(aos_app_stack) / sizeof(cpu_stack_t),
                       (task_entry_t)sys_init, 1);

    aos_start();
}

