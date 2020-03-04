/********************************************************************************************************
 * @file     emi.c
 *
 * @brief    This is the source file for TLSR8258
 *
 * @author	 liang.zhong@telink-semi.com;
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

#include "emi.h"
#include "clock.h"


#define STATE0		0x1234
#define STATE1		0x5678
#define STATE2		0xabcd
#define STATE3		0xef01

static unsigned char  emi_rx_packet[64] __attribute__ ((aligned (4)));
static unsigned char  emi_ble_tx_packet [48]  __attribute__ ((aligned (4))) = {39, 0, 0, 0,0, 37};
static unsigned int   emi_rx_cnt=0,emi_rssibuf=0;
static signed  char   rssi=0;
static unsigned int   state0,state1;

static void rf_single_tone_setting(void)
{
	// zigbee mode for rectify frequency offset
	write_reg8(0x1220, 0x04);
	write_reg8(0x1221, 0x2b);
	write_reg8(0x1222, 0x43);
	write_reg8(0x1223, 0x86);
	write_reg8(0x122a, 0x90);
}

static void rf_set_channel_singletone (signed char chn)//general
{
	unsigned short rf_chn =0;
	unsigned char ctrim;
	unsigned short chnl_freq;

	rf_chn = chn+2400;
	if (rf_chn >= 2550)
	    ctrim = 0;
	else if (rf_chn >= 2520)
	    ctrim = 1;
	else if (rf_chn >= 2495)
	    ctrim = 2;
	else if (rf_chn >= 2465)
	    ctrim = 3;
	else if (rf_chn >= 2435)
		ctrim = 4;
	else if (rf_chn >= 2405)
	    ctrim = 5;
	else if (rf_chn >= 2380)
	    ctrim = 6;
	else
	    ctrim = 7;

	chnl_freq = rf_chn * 2 +1;
	write_reg8(0x1244, ((chnl_freq & 0x7f)<<1) | 1  );   //CHNL_FREQ_DIRECT   CHNL_FREQ_L
	write_reg8(0x1245,  ((read_reg8(0x1245) & 0xc0)) | ((chnl_freq>>7)&0x3f) );  //CHNL_FREQ_H
	write_reg8(0x1229,  (read_reg8(0x1229) & 0xC3) | (ctrim<<2) );  //FE_CTRIM
}

static void rf_set_power_level_index_singletone (RF_PowerTypeDef level)
{
	unsigned char value;

	if(level & BIT(7)){    //VANT
		REG_ADDR8(0x1225) |= BIT(6);
	}
	else{  //VBAT
		REG_ADDR8(0x1225) &= ~BIT(6);
	}

	write_reg8 (0xf02, 0x55);  //tx_en
	value = (unsigned char)(level & 0x3F);

	write_reg8(0x1378, read_reg8(0x1378)|(1<<6));//<6>set 1
	write_reg8(0x137c, ((read_reg8(0x137c)&0x81) | ((value<<1)&0x7e)));//set value to <6:1>
}

static void rf_drv_init_dis_PN(RF_ModeTypeDef mode)
{
	rf_drv_init(mode);
	write_reg8(0x400,0x1f);
	write_reg8(0x401,0);

	if(mode==RF_MODE_BLE_1M)
	{
		write_reg8(0x1220,0X16);
		write_reg8(0x1221,0X0a);
		write_reg8(0x1222,0X20);
		write_reg8(0x1223,0X23);
		write_reg8(0x124e,0X09);
		write_reg8(0x124f,0X0f);

		write_reg8(0x404,0xd5);
	}
	else if(mode==RF_MODE_BLE_2M)
	{
		write_reg8(0x404,0xc5);
	}
}

void rf_emi_single_tone(RF_PowerTypeDef power_level,signed char rf_chn)
{
	rf_single_tone_setting();

	rf_set_channel_singletone(rf_chn);//set freq
	write_reg8 (0xf02, 0x45);  //tx_en
	rf_set_power_level_index_singletone(power_level);
}

void rf_emi_stop(void)
{
	write_reg8(0x1378, 0);
	write_reg8(0x137c, 0);//TX_PA_PWR

	rf_set_power_level_index (0);
	rf_set_tx_rx_off();
}

void rf_emi_rx(RF_ModeTypeDef mode,signed char rf_chn)
{
	write_reg32 (0x800408, 0x29417671);	//accesscode: 1001-0100 1000-0010 0110-1110 1000-1110   29 41 76 71
	write_reg8 (0x800405, read_reg8(0x405)|0x80); //trig accesscode

	rf_rx_buffer_set(emi_rx_packet,64,0);
	rf_drv_init_dis_PN(mode);
	rf_set_channel(rf_chn,0);//set freq
	rf_set_rxmode();
	WaitUs(150);
	rssi = 0;
	emi_rssibuf = 0;
	emi_rx_cnt = 0;
}

void rf_emi_rx_loop(void)
{
	if(read_reg8(0xf20)&BIT(0))
	{
		if((read_reg8(0x44f)&0x0f)==0)
		{
			emi_rssibuf +=  (read_reg8(0x449));

			if(emi_rx_cnt)
			{
				if(emi_rssibuf!=0)
					emi_rssibuf>>=1;
			}
			rssi = emi_rssibuf-110;
			emi_rx_cnt++;
		}
		write_reg8(0xf20, 1);
		write_reg8 (0xf00, 0x80);
	}
}

unsigned int rf_emi_get_rxpkt_cnt(void)
{
	return emi_rx_cnt;
}

char rf_emi_get_rssi_avg(void)
{
	return rssi;
}

unsigned char *rf_emi_get_rxpkt(void)
{
	return emi_rx_packet;
}

static unsigned int pnGen(unsigned int state)
{
	unsigned int feed = 0;
	feed = (state&0x4000) >> 1;
	state ^= feed;
	state <<= 1;
	state = (state&0xfffe) + ((state&0x8000)>>15);
	return state;
}

static void rf_continue_mode_setup(void)
{
	write_reg8(0x800400,0x0a);//0a for 2Mbps, 02 for Zigbee, 0f for New2M and BLE Nrd 1Mbps
	write_reg8(0x800408,0x00);//0 for random , 1 for 0xf0f0, 2 fro 0x5555

	write_reg8(0x800401,0x80);//kick tx controller to wait data
	write_reg32(0x80041c,0xa008030a);
	write_reg32(0x80041c,0xffffffff);
	state0 = STATE0;
	state1 = STATE1;
}

void rf_continue_mode_loop(void)
{

	while(read_reg8(0x80041c) & 0x1){

	}
	if(read_reg8(0x800408) == 1){
		write_reg32(0x80041c, 0x0f0f0f0f);
	}else if(read_reg8(0x800408)==2){
		write_reg32(0x80041c, 0x55555555);
	}else if(read_reg8(0x800408)==3){
		write_reg32(0x80041c, read_reg32(0x800409));
	}else{
		write_reg32(0x80041c, (state0<<16)+state1);
		state0 = pnGen(state0);
		state1 = pnGen(state1);
	}

}

void rf_emi_tx_continue_setup(RF_ModeTypeDef rf_mode,RF_PowerTypeDef power_level,signed char rf_chn,unsigned char pkt_type)
{
	rf_drv_init_dis_PN(rf_mode);//RF_MODE_BLE_1M

	rf_set_channel(rf_chn,0);
	write_reg8(0xf02, 0x45);  //tx manual off
	rf_set_power_level_index_singletone (power_level);
	rf_continue_mode_setup();
	write_reg8(0x408, pkt_type);//0:pbrs9 	1:0xf0	 2:0x55
}

static void rf_phy_test_prbs9 (unsigned char *p, int n)
{
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	unsigned short x = 0x1ff;
	int i;
	int j;
	for ( i=0; i<n; i++)
	{
		unsigned char d = 0;
		for (j=0; j<8; j++)
		{
			if (x & 1)
			{
				d |= BIT(j);
			}
			x = (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100);
		}
		*p++ = d;
	}
}

void rf_emi_tx_brust_setup(RF_ModeTypeDef rf_mode,unsigned char power_level,signed char rf_chn,unsigned char pkt_type)
{
	unsigned char i;
	unsigned char tx_data=0;

	write_reg32(0x408,0x29417671 );//access code  0xf8118ac9
	write_reg8 (0x405, read_reg8(0x405)|0x80);

	rf_set_channel(rf_chn,0);
	rf_drv_init_dis_PN(rf_mode);

	rf_set_power_level_index (power_level);
	if(pkt_type==1)
	{
		tx_data = 0x0f;
	}
	else if(pkt_type==2)
	{
		tx_data = 0x55;
	}


	switch(rf_mode)
	{
		case RF_MODE_BLE_1M:
		case RF_MODE_BLE_2M:
			emi_ble_tx_packet[4] = pkt_type;//type
			for( i=0;i<37;i++)
			{
				emi_ble_tx_packet[6+i]=tx_data;
			}
			break;

		default:
			break;
	}
}

void rf_emi_tx_brust_loop(RF_ModeTypeDef rf_mode,unsigned char pkt_type)
{
	write_reg8(0xf00, 0x80); // stop SM

	if((rf_mode==RF_MODE_BLE_1M)||(rf_mode==RF_MODE_BLE_2M))//ble
	{
		rf_start_stx ((void *)emi_ble_tx_packet, read_reg32(0x740) + 10);
		sleep_us(625);
		if(pkt_type==0)
			rf_phy_test_prbs9(&emi_ble_tx_packet[6],37);
	}
}
