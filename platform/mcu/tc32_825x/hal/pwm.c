/********************************************************************************************************
 * @file     pwm.c 
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
//#include "k_api.h"
#include "common/types.h"
#include "hal/soc/soc.h"
#include "hal/soc/pwm.h"
#include "drivers/8258/gpio_8258.h"
#include "drivers/8258/pwm.h"

GPIO_PinTypeDef get_port_pin(unsigned int port);

typedef struct 
{
    GPIO_FuncTypeDef pwm_ch;
    pwm_id pwmid;
    unsigned char invert;   // is PWM_N
}pwm_pram_t;

static u32 pwm_clock = 0;

static int32_t get_pwm_pram(u8 port, pwm_pram_t *pwm_pram, GPIO_PinTypeDef *pwm_pin_out)
{
    GPIO_PinTypeDef pwm_pin = get_port_pin(port);
    *pwm_pin_out = pwm_pin;
    if(GPIO_ERROR == pwm_pin){
        return -1;
    }

    pwm_pram->pwm_ch = AS_PWM; // (pwm_pin & 0x80) ? AS_PWM_SECOND : AS_PWM; confirm later
    pwm_pram->pwmid = GET_PWMID(pwm_pin, pwm_pram->pwm_ch);
    pwm_pram->invert = GET_PWM_INVERT_VAL(pwm_pin, pwm_pram->pwm_ch);
    
    return 0;    
}

static u32 set_max_pwm_clock(u32 freq)
{
    int div = reg_pwm_clk;
    u32 fs_clk = CLOCK_SYS_CLOCK_HZ;
    u32 pwm_clk = fs_clk;
    for(; div < (256); ++div){
        pwm_clk = (fs_clk/(div+1));
        //sim_printf("max freq: %d, tick: %d\r\n",pwm_clk, pwm_clk / freq);
        if(pwm_clk / freq < 65536){    // pwm is 16 bit
            break;
        }
    }

    if(div != reg_pwm_clk){
        reg_pwm_clk = div;  // pwm_set_clk(fs_clk, pwm_clk);
    }
    
    return pwm_clk;
}

static void pwm_set_cycle_and_duty_2(pwm_id id, u32 freq, float duty_cycle){
    u16 cycle_tick = (u16)(pwm_clock/(freq));
    u16 cmp_tick = (u16)(cycle_tick * duty_cycle);
	pwm_set_cycle_and_duty(id, cycle_tick, cmp_tick);
}

int32_t hal_pwm_init(pwm_dev_t *pwm)
{
    int32_t ret = 0;
    GPIO_PinTypeDef     pwm_pin;
    pwm_pram_t          pwm_pram = {0};
    
    ret = get_pwm_pram(pwm->port,&pwm_pram,&pwm_pin);
    if(ret != 0){
        return ret;
    }

    pwm_clock = set_max_pwm_clock(pwm->config.freq);

    gpio_set_func(pwm_pin, pwm_pram.pwm_ch);
	pwm_set_mode(pwm_pram.pwmid, PWM_NORMAL_MODE);
	pwm_set_phase(pwm_pram.pwmid, 0);   //no phase at pwm beginning
	if(pwm_pram.invert){
	    pwm_n_revert(pwm_pram.pwmid);
	}
	
    pwm_set_cycle_and_duty_2(pwm_pram.pwmid, pwm->config.freq, pwm->config.duty_cycle);
    
	return ret;
}

int32_t set_pwm_enable(pwm_dev_t *pwm, int enable)
{
    int32_t ret = 0;

    GPIO_PinTypeDef     pwm_pin;
    pwm_pram_t          pwm_pram = {0};
    
    ret = get_pwm_pram(pwm->port,&pwm_pram,&pwm_pin);
    if(ret != 0){
        return ret;
    }

    if(enable){
        pwm_start(pwm_pram.pwmid);
    }else{
        pwm_stop(pwm_pram.pwmid);
    }
    return 0;
}

int32_t hal_pwm_start(pwm_dev_t *pwm)
{
    return set_pwm_enable(pwm, 1);
}

int32_t hal_pwm_stop(pwm_dev_t *pwm)
{
    return set_pwm_enable(pwm, 0);
}

int32_t hal_pwm_para_chg(pwm_dev_t *pwm, pwm_config_t para)
{
    int32_t ret = 0;

    GPIO_PinTypeDef     pwm_pin;
    pwm_pram_t          pwm_pram = {0};
    
    ret = get_pwm_pram(pwm->port,&pwm_pram,&pwm_pin);
    if(ret != 0){
        return ret;
    }

    pwm_set_cycle_and_duty_2(pwm_pram.pwmid, para.freq, para.duty_cycle);
    return 0;
}

int32_t hal_pwm_finalize(pwm_dev_t *pwm)
{
    return 0;
}



#if 0
static uint8_t leds_test(void)
{
    tlk_irq_disable();
    int sw_flag = 0;
    static volatile int A_2freq = 1200;
    static volatile pwm_dev_t A_1tc_pwm = {0};
    A_1tc_pwm.port = LIGHT_LED;
    A_1tc_pwm.config.duty_cycle = 0.5f;
    A_1tc_pwm.config.freq = A_2freq;
    hal_pwm_init(&A_1tc_pwm);
    hal_pwm_start(&A_1tc_pwm);
    static pwm_config_t A_1pwm_cfg = {0.2f,10400};
    for(;;)
    {
        static volatile int A_2cnt;A_2cnt++;
        sleep_us(100*1000);
        if(A_2freq != A_1pwm_cfg.freq){
            printf("A_2freq: %d, org freq: %d\r\n", A_2freq, A_1pwm_cfg.freq);
            A_1pwm_cfg.freq = A_2freq;

            A_1tc_pwm.config.freq = A_1pwm_cfg.freq;
            hal_pwm_init(&A_1tc_pwm);
            hal_pwm_start(&A_1tc_pwm);
            extern unsigned int pwm_clock;
            static volatile int A_2pwm_clk; A_2pwm_clk = pwm_clock;
            static volatile int A_3cnt;A_3cnt++;
            static volatile int A_3ret;
            A_3ret = hal_pwm_para_chg(&A_1tc_pwm,A_1pwm_cfg);
        }
        
        static volatile int A_4stop;
        if(A_4stop){
            if(A_4stop == 2){
                printf("pwm start\r\n");
                hal_pwm_start(&A_1tc_pwm);
            }else{
                printf("pwm stop\r\n");
                hal_pwm_stop(&A_1tc_pwm);
            }
            A_4stop = 0;
        }
        //for(;;);
        #if 0
        for(volatile int i=0;i<100;i++)
        {
            for(volatile int j=0;j<15;j++)asm("tnop");
        }
        
        if(sw_flag==0)
            pwm_cfg.duty_cycle += 0.001f;
        else
            pwm_cfg.duty_cycle -= 0.001f;
        
        if(pwm_cfg.duty_cycle>=0.999f)
        {
            sw_flag = 1;
            for(volatile int i=0;i<100;i++)
            {
                for(volatile int j=0;j<7000;j++)asm("tnop");
            }
        }

        if(pwm_cfg.duty_cycle<=0.00f)   
        {
            sw_flag = 0;
            for(volatile int i=0;i<100;i++)
            {
                for(volatile int j=0;j<8000;j++)asm("tnop");
            }
        }
        #endif
    }

    return 0;
}
#endif

