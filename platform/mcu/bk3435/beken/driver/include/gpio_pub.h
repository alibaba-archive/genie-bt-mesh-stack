#ifndef _GPIO_PUB_H_
#define _GPIO_PUB_H_

#include "generic.h"
#include "drv_model_pub.h"

#define GPIO_FAILURE                (1)
#define GPIO_SUCCESS                (0)

#define GPIO_DEV_NAME                "gpio"

#define GPIO_CFG_PARAM(id, mode)           (id + ((mode & 0xff) << 8))
#define GPIO_CFG_PARAM_DEMUX_ID(param)      (param & 0xff)
#define GPIO_CFG_PARAM_DEMUX_MODE(param)    ((param >> 8) & 0xff)

#define GPIO_OUTPUT_PARAM(id, val)           (id + ((val & 0xff) << 8))
#define GPIO_OUTPUT_DEMUX_ID(param)          (param & 0xff)
#define GPIO_OUTPUT_DEMUX_VAL(param)         ((param >> 8) & 0xff)


#define GPIO_CMD_MAGIC              (0xcaa0000)
enum
{
    CMD_GPIO_CFG                        = GPIO_CMD_MAGIC + 0,
    CMD_GPIO_OUTPUT_REVERSE             = GPIO_CMD_MAGIC + 1,
    CMD_GPIO_ENABLE_SECOND              = GPIO_CMD_MAGIC + 2,
    CMD_GPIO_INPUT                      = GPIO_CMD_MAGIC + 3,
    CMD_GPIO_OUTPUT                     = GPIO_CMD_MAGIC + 4,
    CMD_GPIO_CLR_DPLL_UNLOOK_INT_BIT    = GPIO_CMD_MAGIC + 5,
    CMD_GPIO_EN_DPLL_UNLOOK_INT         = GPIO_CMD_MAGIC + 6,
    CMD_GPIO_INT_ENABLE 	            = GPIO_CMD_MAGIC + 7,
    CMD_GPIO_INT_DISABLE	            = GPIO_CMD_MAGIC + 8,
    CMD_GPIO_EN_USB_PLUG_IN_INT         = GPIO_CMD_MAGIC + 9,
    CMD_GPIO_EN_USB_PLUG_OUT_INT        = GPIO_CMD_MAGIC + 10,
};


enum
{
    GMODE_INPUT_PULLDOWN = 0,
    GMODE_OUTPUT,
    GMODE_SECOND_FUNC,
    GMODE_INPUT_PULLUP,
    GMODE_INPUT,
    GMODE_SECOND_FUNC_PULL_UP,//Special for uart1
    GMODE_OUTPUT_PULLUP,
    GMODE_HIGH_Z
};

typedef enum
{
    GPIO_P00 = 0x00,
    GPIO_P01,
    GPIO_P02,
    GPIO_P03,
    GPIO_P04,
    GPIO_P05,
    GPIO_P06,
    GPIO_P07,

    GPIO_P10 = 0x10,
    GPIO_P11,
    GPIO_P12,
    GPIO_P13,
    GPIO_P14,
    GPIO_P15,
    GPIO_P16,
    GPIO_P17,

    GPIO_P20 = 0x20,
    GPIO_P21,
    GPIO_P22,
    GPIO_P23,
    GPIO_P24,
    GPIO_P25,
    GPIO_P26,
    GPIO_P27,

    GPIO_P30 = 0x30,
    GPIO_P31,
    GPIO_P32,
    GPIO_P33,
    GPIO_P34,
    GPIO_P35,
    GPIO_P36,
    GPIO_P37,

    GPIONUM
} GPIO_INDEX ;
#define GPIO_SUM 	(4*8)

enum
{
    GFUNC_MODE_UART1 = 0,
    GFUNC_MODE_I2C,
    GFUNC_MODE_SPI_MST,
    GFUNC_MODE_SPI_SLV,
    GFUNC_MODE_PWM0,
    GFUNC_MODE_PWM1,
    GFUNC_MODE_PWM2,
    GFUNC_MODE_PWM3,
    GFUNC_MODE_PWM4,
    GFUNC_MODE_PWM5,
    GFUNC_MODE_UART2,
    GFUNC_MODE_I2S,
    GFUNC_MODE_ADC1,
    GFUNC_MODE_ADC2,
    GFUNC_MODE_ADC3,
    GFUNC_MODE_ADC4,
    GFUNC_MODE_ADC5,
    GFUNC_MODE_ADC6,
    GFUNC_MODE_ADC7,
};

enum
{
	GPIO_UART1_TX = GPIO_P00,
	GPIO_UART1_RX = GPIO_P01,

	GPIO_I2C_SCL  = GPIO_P02,
	GPIO_I2C_SDA  = GPIO_P03,

	GPIO_SPI_SCK  = GPIO_P04,
	GPIO_SPI_MOSI = GPIO_P05,
	GPIO_SPI_MOSO = GPIO_P06,
	GPIO_SPI_NSS  = GPIO_P07,

	GPIO_PWM_0    = GPIO_P10,
	GPIO_PWM_1    = GPIO_P11,
	GPIO_PWM_2    = GPIO_P12,
	GPIO_PWM_3    = GPIO_P13,
	GPIO_PWM_4    = GPIO_P14,

	GPIO_UART2_TX = GPIO_P16,
	GPIO_UART2_RX = GPIO_P16,

	GPIO_ADC_CH1  = GPIO_P31,
	GPIO_ADC_CH2  = GPIO_P32,
	GPIO_ADC_CH3  = GPIO_P33,
	GPIO_ADC_CH4  = GPIO_P34,
	GPIO_ADC_CH5  = GPIO_P35,
	GPIO_ADC_CH6  = GPIO_P36,
	GPIO_ADC_CH7  = GPIO_P37,
};

enum
{
    GPIO_INT_LEVEL_LOW = 0,
    GPIO_INT_LEVEL_HIGH = 1,
    GPIO_INT_LEVEL_RISING = 2,
    GPIO_INT_LEVEL_FALLING = 3
};

typedef struct gpio_int_st
{
	UINT32 id;
	UINT32 mode;
	void * phandler;
}GPIO_INT_ST;
				
__inline static void bk_gpio_config_input(GPIO_INDEX id)
{
    UINT32 ret;
    UINT32 param;
    
    param = GPIO_CFG_PARAM(id, GMODE_INPUT);
    ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
    
    ASSERT(GPIO_SUCCESS == ret);
}
								
__inline static void bk_gpio_config_input_pup(GPIO_INDEX id)
{
    UINT32 ret;
    UINT32 param;
    
    param = GPIO_CFG_PARAM(id, GMODE_INPUT_PULLUP);
    ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
    
    ASSERT(GPIO_SUCCESS == ret);
}
								
__inline static void bk_gpio_config_input_pdwn(GPIO_INDEX id)
{
    UINT32 ret;
	UINT32 param;
    
	param = GPIO_CFG_PARAM(id, GMODE_INPUT_PULLDOWN);
	ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
    
	ASSERT(GPIO_SUCCESS == ret);
}

__inline static uint32_t bk_gpio_input(GPIO_INDEX id)
{
    UINT32 ret;                                             
    UINT32 param = id;   
    
    ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_INPUT, &param); 
    
    return ret;                      
}

__inline static void bk_gpio_config_output(GPIO_INDEX id)
{
    UINT32 ret;
    
	UINT32 param;
    
	param = GPIO_CFG_PARAM(id, GMODE_OUTPUT);
	ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
	ASSERT(GPIO_SUCCESS == ret);  
}

__inline static void bk_gpio_output(GPIO_INDEX id,UINT32 val)
{
    UINT32 ret;                                           
    UINT32 param;
    
    param = GPIO_OUTPUT_PARAM(id, val);
    ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_OUTPUT, &param);
    ASSERT(GPIO_SUCCESS == ret);           
}

__inline static void bk_gpio_output_reverse(GPIO_INDEX id)
{
    UINT32 ret;
    UINT32 param = id;
    
    ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_OUTPUT_REVERSE, &param);
    ASSERT(GPIO_SUCCESS == ret);            
}
		
extern UINT32 gpio_ctrl(UINT32 cmd, void *param);
//extern UINT32 gpio_input(UINT32 id);
extern void gpio_init(void);
extern void gpio_exit(void);
//void gpio_int_disable(UINT32 index);
//void gpio_int_enable(UINT32 index, UINT32 mode, void (*p_Int_Handler)(unsigned char));
//void gpio_config( UINT32 index, UINT32 mode ) ;
//void gpio_output(UINT32 id, UINT32 val);

#endif // _GPIO_PUB_H_

// EOF

