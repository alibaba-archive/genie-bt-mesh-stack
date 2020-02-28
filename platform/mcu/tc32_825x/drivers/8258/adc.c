/********************************************************************************************************
 * @file     adc.c
 *
 * @brief    This is the ADC driver file for TLSR8258
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

#include "adc.h"

#include "analog.h"
#include "clock.h"
#include "dfifo.h"


_attribute_data_retention_
adc_vref_ctr_t adc_vref_cfg = {
	.adc_vref 		= 1175, //default ADC ref voltage (unit:mV)
	.adc_calib_en	= 0, 	//default disable
};



/**
 * @brief This function serves to set the channel reference voltage.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC reference voltage.
 * @return none
 */
void adc_set_ref_voltage(ADC_ChTypeDef ch_n, ADC_RefVolTypeDef v_ref)
{
#if 0
	if(ch_n & ADC_LEFT_CHN)
	{
		adc_set_vref_chn_left(v_ref);
	}
	if(ch_n & ADC_RIGHT_CHN)
	{
		adc_set_vref_chn_right(v_ref);
	}
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_vref_chn_misc(v_ref);
	}
#else
	adc_set_vref(v_ref, v_ref, v_ref);
#endif


	if(v_ref == ADC_VREF_1P2V)
	{
		//Vref buffer bias current trimming: 		150%
		//Vref buffer bias current trimming: 		150%
		//Comparator preamp bias current trimming:  100%
		analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(0xC0)) | 0x3d );
	}
	else
	{
		//Vref buffer bias current trimming: 		100%
		//Vref buffer bias current trimming: 		100%
		//Comparator preamp bias current trimming:  100%
		analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(0xC0)) | 0x15 );
	}

}

/**
 * @brief This function serves to set resolution.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC resolution.
 * @return none
 */
void adc_set_resolution(ADC_ChTypeDef ch_n, ADC_ResTypeDef v_res)
{
	if(ch_n & ADC_LEFT_CHN)
	{
		adc_set_resolution_chn_left(v_res);
	}
	if(ch_n & ADC_RIGHT_CHN)
	{
		adc_set_resolution_chn_right(v_res);
	}
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_resolution_chn_misc(v_res);
	}
}

/**
 * @brief This function serves to set sample_cycle.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC Sampling cycles.
 * @return none
 */
void adc_set_tsample_cycle(ADC_ChTypeDef ch_n, ADC_SampCycTypeDef adcST)
{
	if(ch_n & ADC_LEFT_CHN)
	{
		adc_set_tsample_cycle_chn_left(adcST);
	}
	if(ch_n & ADC_RIGHT_CHN)
	{
		adc_set_tsample_cycle_chn_right(adcST);
	}
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_tsample_cycle_chn_misc(adcST);
	}
}

/**
 * @brief This function serves to set input_mode.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC channel input mode.
 * @return none
 */
void adc_set_input_mode(ADC_ChTypeDef ch_n,  ADC_InputModeTypeDef m_input)
{
	if(ch_n & ADC_LEFT_CHN)
	{
		adc_set_input_mode_chn_left(m_input);
	}
	if(ch_n & ADC_RIGHT_CHN)
	{
		adc_set_input_mode_chn_right(m_input);
	}
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_input_mode_chn_misc(m_input);
	}
}

/**
 * @brief This function serves to set input channel in single_ended_input_mode.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC analog positive input channel.
 * @return none
 */
#if 0
void adc_set_ain_channel_single_ended_input_mode(ADC_ChTypeDef ch_n, ADC_InputPchTypeDef InPCH)
{

	if(ch_n & ADC_LEFT_CHN)
	{
		adc_set_ain_negative_chn_left(GND);
		adc_set_ain_positive_chn_left(InPCH);
		adc_set_input_mode_chn_left(SINGLE_ENDED_MODE);
	}
	if(ch_n & ADC_RIGHT_CHN)
	{
		adc_set_ain_negative_chn_right(GND);
		adc_set_ain_positive_chn_right(InPCH);
		adc_set_input_mode_chn_right(SINGLE_ENDED_MODE);
	}
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_ain_negative_chn_misc(GND);
		adc_set_ain_positive_chn_misc(InPCH);
		adc_set_input_mode_chn_misc(SINGLE_ENDED_MODE);
	}
}
#endif

/**
 * @brief This function serves to set input channel in differential_mode.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC analog positive input channel.
 * @param[in]   enum variable of ADC analog negative input channel.
 * @return none
 */
void adc_set_ain_channel_differential_mode(ADC_ChTypeDef ch_n, ADC_InputPchTypeDef InPCH,ADC_InputNchTypeDef InNCH)
{

	if(ch_n & ADC_LEFT_CHN)
	{
		adc_set_ain_chn_left(InPCH, InNCH);
		adc_set_input_mode_chn_left(DIFFERENTIAL_MODE);
	}
	if(ch_n & ADC_RIGHT_CHN)
	{
		adc_set_ain_chn_right(InPCH, InNCH);
		adc_set_input_mode_chn_right(DIFFERENTIAL_MODE);
	}
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_ain_chn_misc(InPCH, InNCH);
		adc_set_input_mode_chn_misc(DIFFERENTIAL_MODE);
	}
}


/**
 * @brief This function serves to set pre_scaling.
 * @param[in]   enum variable of ADC pre_scaling factor.
 * @return none
 */
void adc_set_ain_pre_scaler(ADC_PreScalingTypeDef v_scl)
{


	analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(~FLD_SEL_AIN_SCALE)) | (v_scl<<6) );

	//setting adc_sel_atb ,if stat is 0,clear adc_sel_atb,else set adc_sel_atb[0]if(stat)
	unsigned char tmp;
	if(v_scl)
	{
		//ana_F9<4> must be 1
		tmp = ReadAnalogReg(0xF9);
		tmp = tmp|0x10;                    //open tmp = tmp|0x10;
		WriteAnalogReg (0xF9, tmp);
	}
	else
	{
		//ana_F9 <4> <5> must be 0
		tmp = ReadAnalogReg(0xF9);
		tmp = tmp&0xcf;
		WriteAnalogReg (0xF9, tmp);
	}


}




#define DBG_ADC_SAMPLE_DAT			0

#define ADC_SAMPLE_NUM				4    //4, 8



_attribute_data_retention_  unsigned short     adc_vol_mv;


#if (DBG_ADC_SAMPLE_DAT)
	_attribute_data_retention_	unsigned short	adc_dat_min = 0xffff;
	_attribute_data_retention_	unsigned short	adc_dat_max = 0

	_attribute_data_retention_	volatile int * adc_data_buf;
	_attribute_data_retention_	volatile signed int adc_dat_raw[ADC_SAMPLE_NUM*128];

	_attribute_data_retention_	unsigned char	adc_index = 0;

	_attribute_data_retention_	unsigned short adc_sample[ADC_SAMPLE_NUM] = {0};

	_attribute_data_retention_	unsigned int adc_result;
#else

	_attribute_data_retention_	volatile signed int adc_data_buf[ADC_SAMPLE_NUM];  //size must 16 byte aligned(16/32/64...)

#endif


/**********************************************************************************
 * adc_sample_and_get_result
 *
 * return:   mV unit
 * defalut for config:  differential mode, 1.2 Vref, 14 bit resolution, 1/8 scaler
 *
 *********************************************************************************/
unsigned int adc_sample_and_get_result(void)
{
	unsigned short temp;
	int i,j;


	adc_reset_adc_module();
	unsigned int t0 = clock_time();


#if (DBG_ADC_SAMPLE_DAT)
	adc_data_buf = (int *)&adc_dat_raw[ADC_SAMPLE_NUM*adc_index];
#else
	unsigned short adc_sample[ADC_SAMPLE_NUM] = {0};
	unsigned int adc_result;
#endif



	for(i=0;i<ADC_SAMPLE_NUM;i++){   	//dfifo data clear
		adc_data_buf[i] = 0;
	}
	while(!clock_time_exceed(t0, 25));  //wait at least 2 sample cycle(f = 96K, T = 10.4us)

	//dfifo setting will lose in suspend/deep, so we need config it every time
	adc_config_misc_channel_buf((unsigned short *)adc_data_buf, ADC_SAMPLE_NUM<<2);  //size: ADC_SAMPLE_NUM*4
	dfifo_enable_dfifo2();





//////////////// get adc sample data and sort these data ////////////////
	for(i=0;i<ADC_SAMPLE_NUM;i++){
		unsigned int tick_now = clock_time();
		//while(!adc_data_buf[i]);  //wait for new adc sample data
		while(!adc_data_buf[i] && (unsigned int)(clock_time() - tick_now) < 100*CLOCK_16M_SYS_TIMER_CLK_1US);

		if(adc_data_buf[i] & BIT(13)){  //14 bit resolution, BIT(13) is sign bit, 1 means negative voltage in differential_mode
			adc_sample[i] = 0;
		}
		else{
			adc_sample[i] = ((unsigned short)adc_data_buf[i] & 0x1FFF);  //BIT(12..0) is valid adc result
		}



#if (DBG_ADC_SAMPLE_DAT) //debug
		if(adc_sample[i] < adc_dat_min){
			adc_dat_min = adc_sample[i];
		}
		if(adc_sample[i] > adc_dat_max){
			adc_dat_max = adc_sample[i];
		}
#endif


		//insert sort
		if(i){
			if(adc_sample[i] < adc_sample[i-1]){
				temp = adc_sample[i];
				adc_sample[i] = adc_sample[i-1];
				for(j=i-1;j>=0 && adc_sample[j] > temp;j--){
					adc_sample[j+1] = adc_sample[j];
				}
				adc_sample[j+1] = temp;
			}
		}
	}
//////////////////////////////////////////////////////////////////////////////


	dfifo_disable_dfifo2();   //misc channel data dfifo disable





///// get average value from raw data(abandon some small and big data ), then filter with history data //////
#if (ADC_SAMPLE_NUM == 4)  	//use middle 2 data (index: 1,2)
	unsigned int adc_average = (adc_sample[1] + adc_sample[2])/2;
#elif(ADC_SAMPLE_NUM == 8) 	//use middle 4 data (index: 2,3,4,5)
	unsigned int adc_average = (adc_sample[2] + adc_sample[3] + adc_sample[4] + adc_sample[5])/4;
#endif




	adc_result = adc_average;


//////////////// adc sample data convert to voltage(mv) ////////////////
	//                          (Vref, 1/8 scaler)   (BIT<12~0> valid data)
	//			 =  adc_result * Vref * 8 / 0x2000
	//           =  adc_result * Vref >>10
	adc_vol_mv  = (adc_result * adc_vref_cfg.adc_vref)>>10;



#if (DBG_ADC_SAMPLE_DAT) //debug
	adc_index ++;
	if(adc_index >=128){
		adc_index = 0;
	}
#endif



	return adc_vol_mv;
}
