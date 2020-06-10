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
#include <soc.h>

#define WAKE_UP_GPIO  P15

aos_timer_t g_check_timer;
aos_sem_t   g_io_sem;

void io_irq_check()
{
  printf("wake i/o\r\n");

}
int pm_prepare_sleep_action()
{
    hal_ret_sram_enable(RET_SRAM0 | RET_SRAM1 | RET_SRAM2 | RET_SRAM3 | RET_SRAM4);
    csi_gpio_prepare_sleep_action();
    csi_pinmux_prepare_sleep_action();
    csi_usart_prepare_sleep_action(0);
    return 0;
}

int pm_after_sleep_action()
{
    csi_gpio_wakeup_sleep_action();
    csi_pinmux_wakeup_sleep_action();
    csi_usart_wakeup_sleep_action(0);
    if(!phy_gpio_read(WAKE_UP_GPIO))
    {
      aos_sem_signal(&g_io_sem);
    }
    return 0;
}

gpio_irq_handler_t io_irq_handler(void *arg)
{
  io_irq_check();
}

void wakeup_io_init(void)
{
    uint8_t ret;
    gpio_dev_t io;
    io.port = WAKE_UP_GPIO;
    io.config = INPUT_PULL_UP;
    hal_gpio_init(&io);
    hal_gpio_enable_irq(&io, IRQ_TRIGGER_FALLING_EDGE, io_irq_handler,&io);
    phy_gpio_wakeup_set(WAKE_UP_GPIO, NEGEDGE);
}

static void timer_irq_handler(void *p_timer,void * args)
{
  printf("wake t\r\n");
  aos_timer_start(&g_check_timer);
}

void wakeup_timer_init(void)
{
    aos_timer_new(&g_check_timer,timer_irq_handler,NULL,2000,0);
    aos_timer_stop(&g_check_timer);
}

int application_start(int argc, char **argv)
{
    int32_t ret;
    printk("BUILD TIME:%s\n", __DATE__","__TIME__);

    pm_init();
    wakeup_io_init();
    wakeup_timer_init();
    aos_timer_start(&g_check_timer);
    aos_sem_new(&g_io_sem, 0);
    while(1){
      aos_sem_wait(&g_io_sem, AOS_WAIT_FOREVER);
      io_irq_check();
    }
    return 0;
}