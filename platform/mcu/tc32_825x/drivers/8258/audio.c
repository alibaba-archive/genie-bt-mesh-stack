/********************************************************************************************************
 * @file     audio.c
 *
 * @brief    This is the Audio driver file for TLSR8258
 *
 * @author   junyuan.zhang@telink-semi.com;junwei.lu@telink-semi.com
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

#include "audio.h"
#include "pga.h"
#include "adc.h"
//#include "usb8258.h"
#include "irq.h"
#include "register.h"


unsigned char AMIC_ADC_SampleLength[2] = {0xf0/*96K*/,0xab/*132K*/};

unsigned char DMIC_CLK_Mode[RATE_SIZE] = {47/*8	k	1021.277*/,	47/*16k	1021.277*/,
										  34/*22k	1411.765*/,	47/*32k	1021.277*/,
										  34/*44k	1411.765*/,	32/*48k	1500	*/,
										  32/*96k	1500	*/};

unsigned char DMIC_CIC_Rate[RATE_SIZE] = {0x23/*8k 7.978723		CIC_MODE 0*/,	0x22/*16k 15.95745	CIC_MODE 0*/,
										  0x32/*22k 22.05882	CIC_MODE 0*/,	0x11/*32k 31.91489	CIC_MODE 0*/,
										  0x11/*44k 44.11765	CIC_MODE 0*/,	0x11/*48k 46.875	CIC_MODE 0*/,
										  0x00/*96k 93.75 		CIC_MODE 0*/};

unsigned char AMIC_CIC_Rate[RATE_SIZE] = {0xab/*8k  96/12	*/,	0x32/*16k 96/6	*/,
										  0x85/*22k 132/6	*/,	0x42/*32k 96/3	*/,
										  0x42/*44k 132/3	*/,	0x31/*48k 96/2	*/,
										  0x20/*96k			*/};

/*  matrix used under condition: I2S = 2 MHz  */
unsigned long ASDM_Rate_Matching[RATE_SIZE] = {0x00832001/*8k */,0x01063001/*16k*/,
											   0x01687001/*22k*/,0x020C5001/*32k*/,
											   0x02D4D001/*44k*/,0x03127001/*48k*/,
											   0x0624d001/*96k*/};

/*  matrix used under condition: I2S = 2 MHz  */
unsigned long DSDM_Rate_Matching[RATE_SIZE] = {0x00820001/*8 k*/,0x01058001/*16k*/,
											   0x01697001/*22k*/,0x020AF001/*32k*/,
											   0x02D2E001/*44k*/,0x03000001/*48k*/,
											   0x06000001/*96k*/};


enum {
	AUD_DMIC,
	AUD_AMIC,
};

#define SET_PFM(v)     do{\
                           BM_CLR(reg_adc_period_chn0,FLD_ADC_CHNM_PERIOD);\
	              	   	   reg_adc_period_chn0 |= MASK_VAL(FLD_ADC_CHNM_PERIOD,v);\
                       }while(0)
#define SET_PFL(v)     do{\
                           BM_CLR(reg_adc_period_chn12,FLD_ADC_CHNLR_PERIOD);\
                           reg_adc_period_chn12 |= MASK_VAL(FLD_ADC_CHNLR_PERIOD,v);\
                       }while(0)

/**
 * @brief     configure the SDM buffer's address and size
 * @param[in] pbuff - the first address of buffer SDM read data from.
 * @param[in] size_buff - the size of pbuff.
 * @return    none
 */
void audio_config_sdm_buf(signed short* pbuff, unsigned char size_buff)
{

}

/****
* brief: audio amic initial function. configure ADC corresponding parameters. set hpf,lpf and decimation ratio.
* param[in] audio_Rate
* return none
*/
void audio_amic_init(AudioRate_Typedef Audio_Rate)
{
	/******power off sar adc********/
	adc_power_on_sar_adc(0);


	/*******1.ADC setting for analog audio sample**************************/
	adc_reset_adc_module();   //reset whole digital adc module


	adc_set_sample_clk(5); //adc sample clk= 24M/(1+5)=4M

	//adc state machine state cnt 2( "set" stage and "capture" state for left channel)
	adc_set_chn_enable_and_max_state_cnt(ADC_LEFT_CHN, 2);


	//set "capture state" length for misc channel: 240
	//set "set state" length for left channel: 10
	//adc state machine  period  = 24M/250 = 96K
	if((Audio_Rate == AUDIO_44K)||(Audio_Rate == AUDIO_22K))
	{
		adc_set_state_length(0, 172, 10);	//	132K
	}
	else
	{
		adc_set_state_length(0, 240, 10);	//	96K
	}



	adc_set_input_mode(ADC_LEFT_CHN, DIFFERENTIAL_MODE);  				//left channel differential mode
	adc_set_ain_channel_differential_mode(ADC_LEFT_CHN, PGA0P, PGA0N);  //left channel positive and negative data in

	adc_set_ref_voltage(ADC_LEFT_CHN, ADC_VREF_0P6V);					//left channel vref

	adc_set_resolution(ADC_LEFT_CHN, RES14);							//left channel resolution
	adc_set_tsample_cycle(ADC_LEFT_CHN, SAMPLING_CYCLES_6);				//left channel tsample

	adc_set_ain_pre_scaler(ADC_PRESCALER_1);                            //ain pre scaler none

	adc_set_itrim_preamp(ADC_CUR_TRIM_PER100);
	adc_set_itrim_vrefbuf(ADC_CUR_TRIM_PER125);
	adc_set_itrim_vcmbuf(ADC_CUR_TRIM_PER100);


	//PGA0 source: PC0/PC1
	SET_PGA_LEFT_P_AIN(PGA_AIN_C1);
	SET_PGA_LEFT_N_AIN(PGA_AIN_C1);
	SET_PGA_RIGHT_P_AIN(PGA_AIN_C0);
	SET_PGA_RIGHT_N_AIN(PGA_AIN_C0);


	//
	adc_set_left_boost_bias(GAIN_STAGE_BIAS_PER75);

	analog_write (anareg_adc_pga_ctrl, MASK_VAL( FLD_PGA_ITRIM_GAIN_L, GAIN_STAGE_BIAS_PER150, \
												  FLD_PGA_ITRIM_GAIN_R,GAIN_STAGE_BIAS_PER150, \
												  FLD_ADC_MODE, 0, \
												  FLD_SAR_ADC_POWER_DOWN, 1, \
												  FLD_POWER_DOWN_PGA_CHN_L, 0, \
												  FLD_POWER_DOWN_PGA_CHN_R, 1) );



	WriteAnalogReg(0x80+126,0x05);					//0x80+126  = 0x05
	SET_PGA_GAIN_FIX_VALUE(0x14);





	////////////////////////////// ALC HPF LPF setting /////////////////////////////////
	//enable hpf, enable lpf, anable alc, disable double_down_sampling
	reg_aud_alc_hpf_lpf_ctrl =   MASK_VAL( FLD_AUD_IN_HPF_SFT,  0x0b,   //different pcb may set different value.
										   FLD_AUD_IN_HPF_BYPASS, 0, \
										   FLD_AUD_IN_ALC_BYPASS, 0, \
										   FLD_AUD_IN_LPF_BYPASS, 0, \
										   FLD_DOUBLE_DOWN_SAMPLING_ON, 1);
	//alc mode select digital mode
	reg_aud_alc_cfg &= ~FLD_AUD_ALC_ANALOG_MODE_EN;
	//alc left channel select manual regulate, and set volume
	reg_aud_alc_vol_l_chn = MASK_VAL( FLD_AUD_ALC_MIN_VOLUME_IN_DIGITAL_MODE,  0x24, \
									  FLD_AUD_ALC_DIGITAL_MODE_AUTO_REGULATE_EN, 0);




	//2. Dfifo setting
	reg_clk_en2 |= FLD_CLK2_DFIFO_EN; //enable dfifo clock, this will be initialed in cpu_wakeup_int()
	reg_dfifo_mode = FLD_AUD_DFIFO0_IN;

	//amic input, mono mode, enable decimation filter
	reg_dfifo_ain = 	  MASK_VAL( FLD_AUD_DMIC0_DATA_IN_RISING_EDGE,AUDIO_DMIC_DATA_IN_FALLING_EDGE,\
									FLD_AUD_INPUT_SELECT, AUDIO_INPUT_AMIC, \
									FLD_AUD_INPUT_MONO_MODE, 1, \
									FLD_AUD_DECIMATION_FILTER_BYPASS, 0);


	reg_audio_dec_mode  |= FLD_AUD_LNR_VALID_SEL | FLD_AUD_CIC_MODE;
//	reg_dfifo_dec_ratio = AMIC_CIC_Rate[Audio_Rate];
	reg_dfifo_dec_ratio = 0x42;  // 96k/3 = 32k, down sampling to 16K by set core_b40<7>





	//note: this setting must be set after all other settings
	adc_power_on_sar_adc(1);

}

#if 0
static void SendDataToUSB(AudioInput_Typedef Input_Type,AudioRate_Typedef Audio_Rate)
{
	WRITE_REG8(0x800117, 0);//reset pointer of Endpoint7's buf

    int i=0;
    short md;
    unsigned char length = 0;
    short mdl,mdr;

    switch(Audio_Rate)
    {
    case 	AUDIO_8K:		length = 8;break;
    case	AUDIO_16K:		length = 16;break;
    case	AUDIO_22K:		length = 22;break;
    case	AUDIO_32K:		length = 32;break;
    case	AUDIO_44K:		length = 44;break;
    case	AUDIO_48K:		length = 48;break;
    case	AUDIO_96K:		length = 96;break;
    default:				length = 96;break;
    }

    if(Input_Type==DMIC)
    {
		for (i=0; (i<length)&&(!(READ_REG8(0x800b13)&0x10)); i++) {
			if(i%2) {
			md = READ_REG16(0x801802);
			WRITE_REG8(0x80011f, md);//write data to endpoint7's buf
			WRITE_REG8(0x80011f, md>>8);
			}
			else{
			md = READ_REG16(0x801800);
			WRITE_REG8(0x80011f, md);//write data to endpoint7's buf
			WRITE_REG8(0x80011f, md>>8);
			}
		}
    }
    else
    {
		for (i=0; (i<length); i++) {
			if(!(READ_REG8(0x800b13)&0x10))
			{
				if(i%2) {
				mdl = READ_REG16(0x801802);
				WRITE_REG8(0x80011f, mdl);//write data to endpoint7's buf
				WRITE_REG8(0x80011f, mdl>>8);
				}
				else{
				mdr = READ_REG16(0x801800);
				WRITE_REG8(0x80011f, mdr);//write data to endpoint7's buf
				WRITE_REG8(0x80011f, mdr>>8);
				}
			}
			else{
				if(i%2) {
				WRITE_REG8(0x80011f, mdl);//write data to endpoint7's buf
				WRITE_REG8(0x80011f, mdl>>8);
				}
				else{
				WRITE_REG8(0x80011f, mdr);//write data to endpoint7's buf
				WRITE_REG8(0x80011f, mdr>>8);
				}
			}
		}
    }
	WRITE_REG8(0x800127, 0x01);
}
#endif

/**
 * @brief     audio DMIC init function, config the speed of DMIC and downsample audio data to required speed.
 *            actually audio data is dmic_speed/d_samp.
 * @param[in] dmic_speed - set the DMIC speed. such as 1 indicate 1M and 2 indicate 2M.
 * @param[in] d_samp - set the decimation. ie div_speed.
 * @param[in] fhs_source -
 *
 * @return    none.
 */
void audio_dmic_init(unsigned char dmic_speed, enum audio_deci_t d_samp,unsigned char fhs_source)
{

}
/************************************************************************************
*
*	@brief	audio input set function, select analog audio input channel, start the filters
*
*	@param	adc_ch:	if audio input as signle end mode, should identify an analog audio signal input channel, enum variable of ADCINPUTCH
*
*	@return	none
*/
void audio_amic_input_set(enum audio_input_t adc_ch){

}
/**
*	@brief		reg0x30[1:0] 2 bits for fine tuning, divider for slow down sample rate
*	@param[in]	fine_tune - unsigned char fine_tune,range from 0 to 3
*	@return	    none
*/
void audio_finetune_sample_rate(unsigned char fine_tune)
{

}

/**
 *  @brief      tune decimation shift .i.e register 0xb04 in datasheet.
 *  @param[in]  deci_shift - range from 0 to 5.
 *  @return     none
 */
unsigned char audio_tune_deci_shift(unsigned char deci_shift)
{
	 return 0;
}
/**
 *   @brief       tune the HPF shift .i.e register 0xb05 in datasheet.
 *   @param[in]   hpf_shift - range from 0 to 0x0f
 *   @return      none
 */
 unsigned char audio_tune_hpf_shift(unsigned char hpf_shift)
 {
	 return 0;
 }
 /**
 *
 *	@brief	   sdm setting function, enable or disable the sdm output, configure SDM output paramaters
 *
 *	@param[in]	audio_out_en - audio output enable or disable set, '1' enable audio output; '0' disable output
 *	@param[in]	sample_rate - audio sampling rate, such as 16K,32k etc.
 *	@param[in]	sdm_clk -	  SDM clock, default to be 8Mhz
 *	@param[in]  fhs_source
 *
 *	@return	none
 */
void audio_sdm_output_set(unsigned char audio_out_en,int sample_rate,unsigned char sdm_clk,unsigned char fhs_source)
{

}

/**
*	@brief	    set audio volume level
*	@param[in]	input_out_sel - select the tune channel, '1' tune ALC volume; '0' tune SDM output volume
*	@param[in]	volume_level - volume level
*	@return	    none
*/
void audio_volume_tune(unsigned char input_out_sel, unsigned char volume_level)
{

}

/*************************************************************
*
*	@brief	automatically gradual change volume
*
*	@param[in]	vol_step - volume change step, the high part is decrease step while the low part is increase step
*			    gradual_interval - volume increase interval
*
*	@return	none
*/
void audio_volume_step_adjust(unsigned char vol_step,unsigned short gradual_interval)
{

}
