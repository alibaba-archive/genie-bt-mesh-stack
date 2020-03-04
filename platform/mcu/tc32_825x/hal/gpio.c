/********************************************************************************************************
 * @file     gpio.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/
#include "tl_common.h"
#include "hal/soc/soc.h"
#include "hal/soc/gpio.h"
#include "drivers/8258/gpio_8258.h"
#include "drivers/8258/compiler.h"
#include "common/types.h"



typedef struct {
	void *arg;
	gpio_irq_handler_t handler;
	gpio_irq_trigger_t trigger;
	uint8_t port;
} gpio_irq_handler_info_t;

_attribute_ram_code_ GPIO_PinTypeDef get_port_pin(unsigned int port)
{
    if (port > MAX_PINS_NUM){
        return GPIO_ERROR;
    }

    return (GPIO_GROUPB * (port / PINS_IN_GROUP)) | BIT(port % PINS_IN_GROUP);
}


#if GPIO_IRQ_ENABLE
static gpio_irq_handler_info_t gpio_irq_handler_info_tab[GPIO_IRQ_SUPPORTED_NUM] = {0};

// entry of gpio interrupt.
_attribute_ram_code_ void gpio_irq_handler()
{
	if(reg_irq_src & FLD_IRQ_GPIO_EN) {
		reg_irq_src = FLD_IRQ_GPIO_EN; // clear the relevant irq

		uint32_t gpio_irq_signal = REG_ADDR32(0x5E0);
		for(int i=0; i<GPIO_IRQ_SUPPORTED_NUM; i++){
			gpio_irq_handler_info_t *info = &gpio_irq_handler_info_tab[i];
			if (info->handler && (gpio_irq_signal & BIT(info->port))){
				GPIO_PinTypeDef port_pin = get_port_pin(info->port);
				unsigned int port_value = gpio_read(port_pin);
				gpio_set_interrupt_pol(port_pin, port_value?pol_falling:pol_rising);
				if (((info->trigger & IRQ_TRIGGER_FALLING_EDGE) && !port_value)
				  ||((info->trigger & IRQ_TRIGGER_RISING_EDGE) && port_value)){
					info->handler(info->arg);
				}
			}
		}
		
	}
}

gpio_irq_handler_info_t *get_gpio_irq_handler_info_by_port(int port)
{
	for(int i=0;i<GPIO_IRQ_SUPPORTED_NUM;i++){
		if(port == gpio_irq_handler_info_tab[i].port 
			&& NULL != gpio_irq_handler_info_tab[i].handler ){
			return &gpio_irq_handler_info_tab[i];
		}
	}
	return NULL;
}

gpio_irq_handler_info_t *get_available_gpio_irq_handler_info()
{
	// find new one.
	for(int i=0;i<GPIO_IRQ_SUPPORTED_NUM;i++){
		if (gpio_irq_handler_info_tab[i].handler == NULL){
			return &gpio_irq_handler_info_tab[i];
		}
	}
	return NULL;
}

#endif

int32_t hal_gpio_init(gpio_dev_t *gpio)
{
    GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin){
        return -1;
    }

    switch (gpio->config) {
    case INPUT_PULL_UP:
         gpio_set_func(port_pin, AS_GPIO);
	     gpio_set_output_en(port_pin, 0);//disable output
	     gpio_set_input_en(port_pin, 1); //enable input
         gpio_setup_up_down_resistor(port_pin,PM_PIN_PULLUP_10K);
         break;

    case INPUT_PULL_DOWN:
         gpio_set_func(port_pin, AS_GPIO);
	     gpio_set_output_en(port_pin, 0);//disable output
	     gpio_set_input_en(port_pin, 1); //enable input
         gpio_setup_up_down_resistor(port_pin,PM_PIN_PULLDOWN_100K);
         break;

    case INPUT_HIGH_IMPEDANCE:
         gpio_set_func(port_pin, AS_GPIO);
	     gpio_set_output_en(port_pin, 0);//disable output
	     gpio_set_input_en(port_pin, 1); //enable input
         gpio_setup_up_down_resistor(port_pin,PM_PIN_UP_DOWN_FLOAT);
         break;

    case OUTPUT_PUSH_PULL:
    case OUTPUT_OPEN_DRAIN_NO_PULL:
        gpio_set_func(port_pin, AS_GPIO);
	    gpio_set_output_en(port_pin, 1);//enable output
	    gpio_set_input_en(port_pin, 0); //disable input
        break;

    case OUTPUT_OPEN_DRAIN_PULL_UP:    
        gpio_set_func(port_pin, AS_GPIO);
	    gpio_set_output_en(port_pin, 1);//enable output
	    gpio_set_input_en(port_pin, 0); //disable input
        gpio_setup_up_down_resistor(port_pin,PM_PIN_PULLUP_10K);
        break;

    default:
          break;
    }

    return 0;
}


int32_t hal_gpio_finalize(gpio_dev_t *gpio)
{
    GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin){
        return -1;
    }

    //nrf_gpio_cfg_default(gpio->port);
    return 0;
}

int32_t hal_gpio_output_high(gpio_dev_t *gpio)
{    
    GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin)
        return -1;

    gpio_write(port_pin,1);
    //sim_printf("--------------------led_ctrl: port:%d, ON \r\n",gpio->port);

    return 0;
}

int32_t hal_gpio_output_low(gpio_dev_t *gpio)
{
    GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin){
        return -1;
    }

    gpio_write(port_pin,0);
    //sim_printf("--------------------led_ctrl: port:%d, OFF \r\n",gpio->port);

    return 0;
}

int32_t hal_gpio_output_toggle(gpio_dev_t *gpio)
{
    GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin){
        return -1;
    }

    gpio_toggle(port_pin);

    return 0;
}

int32_t hal_gpio_input_get(gpio_dev_t *gpio, uint32_t *value)
{
    GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin){
        return -1;
    }

    *value = gpio_read(port_pin);

    return 0;
}

int32_t hal_gpio_enable_irq(gpio_dev_t *gpio, gpio_irq_trigger_t trigger,gpio_irq_handler_t handler, void *arg)
{
#if GPIO_IRQ_ENABLE
	GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
    if(GPIO_ERROR == port_pin){
        return -1;
    }

	gpio_irq_handler_info_t *info = get_gpio_irq_handler_info_by_port(gpio->port);
	if (info == NULL){
		info = get_available_gpio_irq_handler_info();
	}
	if (info == NULL){
		return -2;
	}
	
	gpio_set_func(port_pin, AS_GPIO);
	gpio_set_input_en(port_pin, 1); //enable input
	gpio_set_output_en(port_pin, 0);//disable output

	info->port = gpio->port;
	info->trigger = trigger;
	info->arg = arg;
	info->handler = handler;

	GPIO_PolTypeDef pol_type;
	switch(trigger){
		case IRQ_TRIGGER_RISING_EDGE:
			pol_type = pol_rising;
			break;
		case IRQ_TRIGGER_FALLING_EDGE:
			pol_type = pol_falling;
			break;
		case IRQ_TRIGGER_BOTH_EDGES:
			pol_type = (gpio_read(port_pin)?pol_falling:pol_rising);
			break;
		default:
			return -2;
	}
	
	gpio_set_interrupt(port_pin, pol_type);
#endif

    return 0;
}

int32_t hal_gpio_disable_irq(gpio_dev_t *gpio)
{
#if GPIO_IRQ_ENABLE
	GPIO_PinTypeDef port_pin = get_port_pin(gpio->port);
	if(GPIO_ERROR == port_pin || gpio->port >= GPIO_IRQ_SUPPORTED_NUM){
		return -1;
	}

	gpio_irq_handler_info_t *info = get_gpio_irq_handler_info_by_port(gpio->port);
	if (info == NULL){
		return -2;
	}

	info->arg = NULL;
	info->handler = NULL;

	gpio_en_interrupt(port_pin, false);
#endif

    return 0;
}

int32_t hal_gpio_clear_irq(gpio_dev_t *gpio)
{
    return 0;
}

#if 0
void led_ctrl_test()
{
    static volatile int A_0led;
    if(A_0led){
        led_ctrl(&light_led, !(A_0led & BIT(0)));
        led_ctrl(&light_level_up, !(A_0led & BIT(1)));
        led_ctrl(&light_level_down, !(A_0led & BIT(2)));
        A_0led = 0;
    }
}
#endif

