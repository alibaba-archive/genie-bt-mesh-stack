/********************************************************************************************************
 * @file     clock.c
 *
 * @brief    This is the source file for TLSR8258
 *
 * @author	 junwei.lu@telink-semi.com;
 * @date     May 8, 2018
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

#include "register.h"
#include "clock.h"
#include "irq.h"
#include "analog.h"
#include "pm.h"

_attribute_data_retention_	unsigned char system_clk_type;




void clock_rc_set(SYS_CLK_TYPEDEF SYS_CLK)
{

	unsigned char  temp = analog_read(0x04)&0xfc;

	if(SYS_CLK==SYS_CLK_32M_RC)
	{
		analog_write(0x04, temp|0x01);
		//rc_32m_cal();
	}
	else if(SYS_CLK==SYS_CLK_48M_RC)
	{
		analog_write(0x04, temp|0x03);
		//rc_48m_cal();
	}
}


_attribute_ram_code_ void clock_init(SYS_CLK_TYPEDEF SYS_CLK)
{

	reg_clk_sel = (unsigned char)SYS_CLK;
	system_clk_type = (unsigned char)SYS_CLK;

#if (SYSCLK_RC_CLOCK_EN)
	if(SYS_CLK == SYS_CLK_48M_Crystal || SYS_CLK == SYS_CLK_48M_RC)
#else
	if(SYS_CLK == SYS_CLK_48M_Crystal)
#endif
	{
		analog_write(0x0C, 0xC6);   //default c4: dcdc 1.8V;  c6: dcdc 1.9V (GD flash 48M clock may error, need higher DCDC voltage)
	}


#if (SYSCLK_RC_CLOCK_EN)
	if(SYS_CLK < SYS_CLK_RC_THRES){
		clock_rc_set(SYS_CLK);
	}
#endif

#if (MODULE_WATCHDOG_ENABLE)
	reg_tmr_ctrl = MASK_VAL(
		FLD_TMR_WD_CAPT, (MODULE_WATCHDOG_ENABLE ? (WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF):0)
		, FLD_TMR_WD_EN, (MODULE_WATCHDOG_ENABLE?1:0));
#endif
}









void rc_24m_cal (void)
{
    analog_write(0xc8, 0x80);

//    sub_wr_ana(0x30, 1, 7, 7);
    analog_write(0x30, analog_read(0x30) | BIT(7) );

    analog_write(0xc7, 0x0e);
    analog_write(0xc7, 0x0f);
    while((analog_read(0xcf) & 0x80) == 0);
    unsigned char cap = analog_read(0xcb);
    analog_write(0x33, cap);		//write 24m cap into manual register

//	sub_wr_ana(0x30, 0, 7, 7);	//manual on
    analog_write(0x30, analog_read(0x30) & (~BIT(7)) );

	analog_write(0xc7, 0x0e);
}


void rc_32k_cal (void)
{
    analog_write(0x30, 0x60);
    analog_write(0xc6, 0xf6);
    analog_write(0xc6, 0xf7);
    while((analog_read(0xcf) & BIT(6)) == 0);
	unsigned char res1 = analog_read(0xc9); //read 32k res[13:6]
	analog_write(0x32, res1);  //write 32k res[13:6] into manual register
	unsigned char res2 = analog_read(0xca); //read 32k res[5:0]
	analog_write(0x31, res2);  //write 32k res[5:0] into manual register
	analog_write(0xc6, 0xf6);
	analog_write(0x30, 0x20);//manual on
}


void rc_48m_cal (void)
{
	analog_write(0x33, 0x80);
	analog_write(0x30, 0x20);
    analog_write(0xc7, 0x0e);
    WaitUs(1000);
    analog_write(0xc7, 0x0f);
    while((analog_read(0xcf) & 0x80) == 0);

    volatile unsigned int cal_cnt = analog_read(0xcf)&0x07 ;
    cal_cnt = (cal_cnt<<8) + analog_read(0xce);
    unsigned int f = 64;
    unsigned int temp_v = 0;
    unsigned int temp_d = 0;
    unsigned int temp_d2 = 100;
    unsigned char temp_cap = 0;
    unsigned int i=0;
    while(f>=1)
    {
		temp_v = analog_read(0x33);
    	if(cal_cnt>250)
    	{
    		temp_d =  cal_cnt - 250;
    	}
    	else
    	{
    		temp_d =  250 - cal_cnt;
    	}
    	if(cal_cnt>250)
    	{
    		analog_write(0x33, temp_v-f);
    	}
    	else
    	{
    		analog_write(0x33, temp_v+f);
    	}
    	f = f/2;
        analog_write(0xc7, 0x0e);
        analog_write(0xc7, 0x0f);
        while((analog_read(0xcf) & 0x80) == 0);
        cal_cnt = analog_read(0xcf)&0x07 ;
		cal_cnt = (cal_cnt<<8) + analog_read(0xce);
		i++;
    	if(temp_d2>temp_d)
    	{
    		temp_d2 = temp_d;
    		temp_cap = temp_v;
    	}
    }
    analog_write(0x33, temp_cap);
}

_attribute_ram_code_ void sleep_us (unsigned long us)
{
	unsigned long t = clock_time();
	while(!clock_time_exceed(t, us)){
	}
}

void doubler_calibration(void)
{
	 WriteAnalogReg(0x86,0xbb);//power on duty cycle cal moudle
	 WriteAnalogReg(0x82,ReadAnalogReg(0x82)&0x7f);
	 WriteAnalogReg(0x87,(ReadAnalogReg(0x87)&0xfc)|0x02);
	 WriteAnalogReg(0x87,ReadAnalogReg(0x87)|0x04);
	 unsigned char tmp=0;
	 unsigned char dcal=0;
	 tmp = ReadAnalogReg(0x88);
	 dcal= tmp & 0x1f;
	 WriteAnalogReg(0x87,(ReadAnalogReg(0x87)&0x07)|(dcal<<3));
	 WriteAnalogReg(0x82,ReadAnalogReg(0x82)|0x80);
	 WriteAnalogReg(0x87,ReadAnalogReg(0x87)&0xfd);
	 WriteAnalogReg(0x86,0xfb);
}

