/********************************************************************************************************
 * @file     adc.h
 *
 * @brief    This is the ADC driver header file for TLSR8258
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

#pragma once

#include "bsp.h"
#include "analog.h"
#include "register.h"

//ADC reference voltage cfg
typedef struct {
	unsigned short adc_vref; //default: 11750mV
	unsigned short adc_calib_en;
}adc_vref_ctr_t;

extern adc_vref_ctr_t adc_vref_cfg;

/**
 * @brief      This function enable/disable adc vref calibration
 * @param[in]  none.
 * @return     none.
 */
static inline void	adc_calib_vref_enable(unsigned char en)
{
	adc_vref_cfg.adc_calib_en = en;
}


/**
 * @brief      This function reset adc module
 * @param[in]  none.
 * @return     none.
 */
static inline void	adc_reset_adc_module(void)
{
	reg_rst1 = FLD_RST1_ADC;
	reg_rst1 = 0;
}

/**
 * @brief      This function enable adc source clock: external 24M
 * @param[in]  variable of source clock state 1: enable;  0: disable.
 * @return     none.
 */
//ana_82 set 0x64(FLD_CLK_24M_TO_SAR_EN is on) in cpu_wakeup_init(), so we do not set it enable in application
static inline void adc_enable_clk_24m_to_sar_adc(unsigned int en)
{
	if(en){
		analog_write(anareg_82, analog_read(anareg_82) | FLD_CLK_24M_TO_SAR_EN);
	}
	else{
		analog_write(anareg_82, analog_read(anareg_82) & ~FLD_CLK_24M_TO_SAR_EN);
	}
}


/**************************************************************************************
afe_0xE7
    BIT<1:0>  adc_vrefl, Select VREF for left channel
    BIT<3:2>  adc_vrefr, Select VREF for right channel
    BIT<5:4>  adc_vrefm, Select VREF for Misc channel
			0x0: 0.6V
			0x1: 0.9V
 	 	 	0x2: 1.2V
 	 	 	0x3: VBAT/N (N=2/3/4, when afe_0xF9<3:2> is set as 0x3/0x2/0x1).

	notice that: when choose ADC_VREF_VBAT_N, must use adc_set_vref_vbat_divider to config N(1/2/3/4)

afe_0xF9
    BIT<3:2>  adc_vbat_div
      	  	  Vbat divider select (see adc_vref)
					0x0: OFF
					0x1: VBAT/4
					0x2: VBAT/3
					0x3: VBAT/2
 *************************************************************************************/
#define anareg_adc_vref			0xE7            //poweron_dft: 0x0b
enum{
	FLD_ADC_VREF_CHN_L = 		BIT_RNG(0,1),
	FLD_ADC_VREF_CHN_R = 		BIT_RNG(2,3),
	FLD_ADC_VREF_CHN_M = 		BIT_RNG(4,5),
};

/**
 *  ADC reference voltage
 */
typedef enum{
	ADC_VREF_0P6V,
	ADC_VREF_0P9V,
	ADC_VREF_1P2V,
	ADC_VREF_VBAT_N,  //here N(1/2/3/4) is configed by interface adc_set_vref_vbat_divider
}ADC_RefVolTypeDef;

/**
 *
 */
#define anareg_adc_vref_vbat_div		0xF9   //poweron_dft: 0x00
enum{
	FLD_ADC_VREF_VBAT_DIV = 		BIT_RNG(2,3),
};

/**
 *  Vbat divider select
 */
typedef enum{
	ADC_VBAT_DIVIDER_OFF = 0,
	ADC_VBAT_DIVIDER_1F4,
	ADC_VBAT_DIVIDER_1F3,
	ADC_VBAT_DIVIDER_1F2
}ADC_VbatDivTypeDef;


static inline void adc_set_vref(ADC_RefVolTypeDef vRef_L, ADC_RefVolTypeDef vRef_R, ADC_RefVolTypeDef vRef_M)
{
	analog_write(anareg_adc_vref, vRef_L | vRef_R<<2 | vRef_M<<4);
}


/**
 * @brief      This function sets ADC reference voltage for the L channel
 * @param[in]  v_ref - enum variable of adc reference voltage.
 * @return     none
 */
static inline void adc_set_vref_chn_left(ADC_RefVolTypeDef v_ref)
{
	analog_write(anareg_adc_vref, ((analog_read(anareg_adc_vref)&(~FLD_ADC_VREF_CHN_L)) | (v_ref)) );
}

/**
 * @brief      This function sets ADC reference voltage for the R channel
 * @param[in]  v_ref - enum variable of adc reference voltage.
 * @return     none
 */
static inline void adc_set_vref_chn_right(ADC_RefVolTypeDef v_ref)
{
	analog_write(anareg_adc_vref, ((analog_read(anareg_adc_vref)&(~FLD_ADC_VREF_CHN_R)) | (v_ref<<2) ));
}

/**
 * @brief      This function sets ADC reference voltage for the MISC channel
 * @param[in]  v_ref - enum variable of adc reference voltage.
 * @return     none
 */
static inline void adc_set_vref_chn_misc(ADC_RefVolTypeDef v_ref)
{
	analog_write(anareg_adc_vref, ((analog_read(anareg_adc_vref)&(~FLD_ADC_VREF_CHN_M)) | (v_ref<<4)) );
}

/**
 * @brief      This function select Vbat voltage divider
 * @param[in]  vbat_div - enum variable of Vbat voltage divider.
 * @return     none
 */
static inline void adc_set_vref_vbat_divider(ADC_VbatDivTypeDef vbat_div)
{
	analog_write (anareg_adc_vref_vbat_div, (analog_read(anareg_adc_vref_vbat_div)&(~FLD_ADC_VREF_VBAT_DIV)) | (vbat_div<<2) );
}

/**************************************************************************************
afe_0xE8
    BIT<3:0>  Select negative input for Misc channel
    BIT<7:4>  Select positive input for Misc channel

afe_0xE9
    BIT<3:0>  Select negative input for left channel
    BIT<7:4>  Select positive input for left channel

afe_0xEA
    BIT<3:0>  Select negative input for right channel
    BIT<7:4>  Select positive input for right channel

	negative input:
		0x0: No input
		0x1: B<0>
		0x2: B<1>
		...
		0x8: B<7>
		0x9: C<4>
		0xa: C<5>
		0xb: pga_n<0> (PGA left-channel negative output)
		0xc: pga_n<1> (PGA right-channel negative output)
		0xd: tempsensor_n (Temperature sensor negative output)
		0xe: rsvd  //spec "GND" is wrong
		0xf: Ground

	positive input:
		0x0: No input
		0x1: B<0>
		0x2: B<1>
		...
		0x8: B<7>
		0x9: C<4>
		0xa: C<5>
		0xb: pga_p<0> (PGA left-channel positive output)
		0xc: pga_p<1> (PGA right-channel positive output)
		0xd: tempsensor_p (Temperature sensor positive output)
		0xe: rsvd
		0xf: Vbat (Battery voltage)
 *************************************************************************************/
#define anareg_adc_ain_chn_misc			0xE8	//poweron_dft: 0x02
#define anareg_adc_ain_chn_left			0xE9	//poweron_dft: 0x00
#define anareg_adc_ain_chn_right		0xEA	//poweron_dft: 0x01

enum{
	FLD_ADC_AIN_NEGATIVE = 		BIT_RNG(0,3),
	FLD_ADC_AIN_POSITIVE = 		BIT_RNG(4,7),
};


/**
 *ADC analog input channel
 */
/*ADC analog negative input channel selection enum*/
typedef enum {
	NOINPUTN,
	B0N,
	B1N,
	B2N,
	B3N,
	B4N,
	B5N,
	B6N,
	B7N,
	C4N,
	C5N,
	PGA0N,
	PGA1N,
	TEMSENSORN,
	RSVD_N,
	GND,
}ADC_InputNchTypeDef;

/*ADC analog positive input channel selection enum*/
typedef enum {
	NOINPUTP,
	B0P,
	B1P,
	B2P,
	B3P,
	B4P,
	B5P,
	B6P,
	B7P,
	C4P,
	C5P,
	PGA0P,
	PGA1P,
	TEMSENSORP,
	RSVD_P,
	VBAT,
}ADC_InputPchTypeDef;

static inline void adc_set_ain_chn_misc(ADC_InputPchTypeDef p_ain, ADC_InputNchTypeDef n_ain)
{
	analog_write (anareg_adc_ain_chn_misc, n_ain | p_ain<<4 );
}

static inline void adc_set_ain_chn_left(ADC_InputPchTypeDef p_ain, ADC_InputNchTypeDef n_ain)
{
	analog_write (anareg_adc_ain_chn_left, n_ain | p_ain<<4 );
}

static inline void adc_set_ain_chn_right(ADC_InputPchTypeDef p_ain, ADC_InputNchTypeDef n_ain)
{
	analog_write (anareg_adc_ain_chn_right, n_ain | p_ain<<4 );
}


/**************************************************************************************
afe_0xEB
    BIT<1:0>  adc_resl, Set resolution for left channel
    BIT<5:4>  adc_resr, Set resolution for right channel

afe_0xEC
    BIT<1:0>  adc_resm, Set resolution for Misc channel

			 If RNS mode is set, resolution is set to 1. Otherwise ADC mode setting is:
				0x0: 8bits
				0x1: 10bits
				0x2: 12bits
				0x3: 14bits

	BIT<3:2>  Reserved
	BIT<4>    adc_en_diffl,  Select single-end or differential input mode for left channel.
	BIT<5>    adc_en_diffr,  Select single-end or differential input mode for right channel.
	BIT<6>    adc_en_diffm,  Select single-end or differential input mode for Misc channel.
	BIT<7>    Reserved
 *************************************************************************************/
#define anareg_adc_res_l_r			0xEB	//poweron_dft: 0x33
enum{
	FLD_ADC_RES_L = 		BIT_RNG(0,1),
	FLD_ADC_RES_R = 		BIT_RNG(4,5),
};

#define anareg_adc_res_m			0xEC	//poweron_dft: 0x03
enum{
	FLD_ADC_RES_M 		 = 	BIT_RNG(0,1),
	FLD_ADC_EN_DIFF_CHN_L	 =  BIT(4),
	FLD_ADC_EN_DIFF_CHN_R	 =  BIT(5),
	FLD_ADC_EN_DIFF_CHN_M	 =  BIT(6),
};

/**
 *ADC resolution
 */
typedef enum{
	RES8,
	RES10,
	RES12,
	RES14
}ADC_ResTypeDef;

/**
 * @brief      This function sets ADC resolution for the L channel
 * @param[in]  v_res - enum variable of ADC resolution.
 * @return     none
 */
static inline void adc_set_resolution_chn_left(ADC_ResTypeDef v_res)
{
	analog_write(anareg_adc_res_l_r, (analog_read(anareg_adc_res_l_r)&(~FLD_ADC_RES_L)) | (v_res) );
}

/**
 * @brief      This function sets ADC resolution for the R channel
 * @param[in]  v_res - enum variable of ADC resolution.
 * @return     none
 */
static inline void adc_set_resolution_chn_right(ADC_ResTypeDef v_res)
{
	analog_write(anareg_adc_res_l_r, (analog_read(anareg_adc_res_l_r)&(~FLD_ADC_RES_R)) | (v_res<<4) );
}

/**
 * @brief      This function sets ADC resolution for the MISC channel
 * @param[in]  v_res - enum variable of ADC resolution.
 * @return     none
 */
static inline void adc_set_resolution_chn_misc(ADC_ResTypeDef v_res)
{
	analog_write(anareg_adc_res_m, (analog_read(anareg_adc_res_m)&(~FLD_ADC_RES_M)) | (v_res) );
}

/**
 *ADC channel input mode
 */
typedef enum{
	SINGLE_ENDED_MODE = 0,  //single-ended mode
	DIFFERENTIAL_MODE = 1,  //differential mode
}ADC_InputModeTypeDef;

/**
 * @brief      This function sets ADC input mode for the L channel
 * @param[in]  m_input - enum variable of ADC channel input mode.
 * @return     none
 */
static inline void adc_set_input_mode_chn_left(ADC_InputModeTypeDef m_input)
{
	if(m_input){  //differential mode
		analog_write(anareg_adc_res_m, (analog_read(anareg_adc_res_m) | FLD_ADC_EN_DIFF_CHN_L ));
	}
	else{  //single-ended mode
		analog_write(anareg_adc_res_m, (analog_read(anareg_adc_res_m) & (~FLD_ADC_EN_DIFF_CHN_L)));
	}
}

/**
 * @brief      This function sets ADC input mode for the R channel
 * @param[in]  m_input - enum variable of ADC channel input mode.
 * @return     none
 */
static inline void adc_set_input_mode_chn_right(ADC_InputModeTypeDef m_input)
{
	if(m_input){  //differential mode
		analog_write(anareg_adc_res_m, analog_read(anareg_adc_res_m) | FLD_ADC_EN_DIFF_CHN_R );
	}
	else{  //single-ended mode
		analog_write(anareg_adc_res_m, analog_read(anareg_adc_res_m) & (~FLD_ADC_EN_DIFF_CHN_R) );
	}
}

/**
 * @brief      This function sets ADC input mode for the MISC channel
 * @param[in]  m_input - enum variable of ADC channel input mode.
 * @return     none
 */
static inline void adc_set_input_mode_chn_misc(ADC_InputModeTypeDef m_input)
{
	if(m_input){  //differential mode
		analog_write(anareg_adc_res_m, analog_read(anareg_adc_res_m) | FLD_ADC_EN_DIFF_CHN_M );
	}
	else{  //single-ended mode
		analog_write(anareg_adc_res_m, analog_read(anareg_adc_res_m) & (~FLD_ADC_EN_DIFF_CHN_M) );
	}
}


/**************************************************************************************
afe_0xED
    BIT<3:0>  adc_tsampl, Number of ADC clock cycles in sampling phase for left channel to stabilize the input before sampling
    BIT<7:4>  adc_tsampr, Number of ADC clock cycles in sampling phase for right channel to stabilize the input before sampling

afe_0xEE
    BIT<3:0>  adc_tsampm, Number of ADC clock cycles in sampling phase for Misc channel to stabilize the input before sampling

			0x0: 3 cycles (RNS mode default)
			0x1: 6 cycles
			0x2: 9 cycles
			0x3: 12 cycles
			......
			0xf: 48 cycles
 *************************************************************************************/
#define anareg_adc_tsmaple_l_r			0xED	//poweron_dft: 0x11
enum{
	FLD_ADC_TSAMPLE_CYCLE_CHN_L = 		BIT_RNG(0,3),
	FLD_ADC_TSAMPLE_CYCLE_CHN_R = 		BIT_RNG(4,7),
};

#define anareg_adc_tsmaple_m			0xEE	//poweron_dft: 0x01
enum{
	FLD_ADC_TSAMPLE_CYCLE_CHN_M = 		BIT_RNG(0,3),
};

/**
 * ADC Sampling cycles
 */
typedef enum{
	SAMPLING_CYCLES_3,
	SAMPLING_CYCLES_6,
	SAMPLING_CYCLES_9,
	SAMPLING_CYCLES_12,
	SAMPLING_CYCLES_15,
	SAMPLING_CYCLES_18,
	SAMPLING_CYCLES_21,
	SAMPLING_CYCLES_24,
	SAMPLING_CYCLES_27,
	SAMPLING_CYCLES_30,
	SAMPLING_CYCLES_33,
	SAMPLING_CYCLES_36,
	SAMPLING_CYCLES_39,
	SAMPLING_CYCLES_42,
	SAMPLING_CYCLES_45,
	SAMPLING_CYCLES_48,
}ADC_SampCycTypeDef;


/**
 * @brief      This function sets ADC sample time(the number of adc clocks for sample cycles) for the L channel.
 * @param[in]  adcST - enum variable of adc sample cycles.
 * @return     none
 */
static inline void adc_set_tsample_cycle_chn_left(ADC_SampCycTypeDef adcST)
{
	analog_write(anareg_adc_tsmaple_l_r, (analog_read(anareg_adc_tsmaple_l_r)&(~FLD_ADC_TSAMPLE_CYCLE_CHN_L)) | (adcST) );
}

/**
 * @brief      This function sets ADC sample time(the number of adc clocks for sample cycles) for the R channel.
 * @param[in]  adcST - enum variable of adc sample cycles.
 * @return     none
 */
static inline void adc_set_tsample_cycle_chn_right(ADC_SampCycTypeDef adcST)
{
	analog_write(anareg_adc_tsmaple_l_r, (analog_read(anareg_adc_tsmaple_l_r)&(~FLD_ADC_TSAMPLE_CYCLE_CHN_R)) | (adcST<<4) );
}

/**
 * @brief      This function sets ADC sample time(the number of adc clocks for sample cycles) for the MISC channel.
 * @param[in]  adcST - enum variable of adc sample cycles.
 * @return     none
 */
static inline void adc_set_tsample_cycle_chn_misc(ADC_SampCycTypeDef adcST)
{
	//analog_write(anareg_adc_tsmaple_m, (analog_read(anareg_adc_tsmaple_m)&(~FLD_ADC_TSAMPLE_CYCLE_CHN_M)) | (adcST) );
	analog_write(anareg_adc_tsmaple_m, adcST );
}


/**************************************************************************************
afe_0xEF<7:0>  	r_max_mc[7:0]
afe_0xF0<7:0>	r_max_c[7:0]
afe_0xF1
		<3:0>   r_max_s
		<5:4>   r_max_c[9:8]
		<7:6>   r_max_mc[9:8]

	r_max_mc[9:0]: serves to set length of ��capture�� state for RNS and Misc channel.
	r_max_c[9:0]:  serves to set length of ��capture�� state for left and right channel.
	r_max_s:       serves to set length of ��set�� state for left, right and Misc channel.

	Note: State length indicates number of 24M clock cycles occupied by the state.
 *************************************************************************************/
#define    anareg_r_max_mc	        	0xEF	//poweron_dft: 0x0f
#define    anareg_r_max_c       		0xF0	//poweron_dft: 0x60
#define    anareg_r_max_s			    0xF1	//poweron_dft: 0x06

enum{								 //ana_EF
	FLD_R_MAX_MC0	= BIT_RNG(0,7),
};

enum{								 //ana_F0
	FLD_R_MAX_C0	= BIT_RNG(0,7),
};

enum{                                //ana_F1
	FLD_R_MAX_S		= BIT_RNG(0,3),
	FLD_R_MAX_C1	= BIT_RNG(4,5),
	FLD_R_MAX_MC1	= BIT_RNG(6,7),
};


//state length indicates number of 24M clock cycles occupied by the state
//R_max_mc[9:0] serves to set length of "capture" state for RNS and Misc channel
//R_max_c[9:0]  serves to set length of "capture" state for left and right channel
//R_max_s[9:0]  serves to set length of "set" state for left, right and Misc channel
static inline void adc_set_state_length(unsigned short R_max_mc, unsigned short R_max_c,unsigned char R_max_s)
{
	WriteAnalogReg(anareg_r_max_mc, R_max_mc);
	WriteAnalogReg(anareg_r_max_c, 	R_max_c);
	WriteAnalogReg(anareg_r_max_s,  ((R_max_mc>>8)<<6) | ((R_max_c>>8)<<4)  | (R_max_s & FLD_R_MAX_S)   );
}



/***************************************************************************************
afe_0xF2
    BIT<0>   r_en_left    Enable left  channel. 1: enable;  0: disable
    BIT<1>   r_en_right   Enable right channel. 1: enable;  0: disable
	BIT<2>   r_en_Misc    Enable Misc  channel. 1: enable;  0: disable
	BIT<3>   r_en_rns	  Enable RNS sampling.  1: enable;  0: disable
	BIT<6:4> r_max_scnt   Set total length for sampling state machine (i.e. max state index)
 *************************************************************************************/
#define anareg_adc_chn_en			0xF2	//poweron_dft: 0x27
enum{
	FLD_ADC_CHN_EN_L	= BIT(0),
	FLD_ADC_CHN_EN_R	= BIT(1),
	FLD_ADC_CHN_EN_M	= BIT(2),
	FLD_ADC_CHN_EN_RNS	= BIT(3),
	FLD_ADC_MAX_SCNT	= BIT_RNG(4,6),
};

/**
 * ADC input channel selection enum
 */
typedef enum{
	ADC_LEFT_CHN 	= BIT(0),
	ADC_RIGHT_CHN	= BIT(1),
	ADC_MISC_CHN	= BIT(2),
	ADC_RNS_CHN 	= BIT(3),
}ADC_ChTypeDef;


static inline void adc_set_chn_enable_and_max_state_cnt(ADC_ChTypeDef ad_ch, unsigned char s_cnt)
{
	analog_write(anareg_adc_chn_en, ad_ch | ((s_cnt&0x07)<<4) );
}



/**************************************************************************************
afe_0xF4
    BIT<2:0>  adc_clk_div
      	  	  ADC clock(derive from external 24M crystal)
      	  	  ADC clk frequency = 24M/(adc_clk_div + 1)
    BIT<7:3>  reserved
 *************************************************************************************/
#define anareg_adc_clk_div				0xF4	//poweron_dft: 0x03
enum{
	FLD_ADC_CLK_DIV = BIT_RNG(0,2)
};

/**
 * @brief      This function sets adc sample clk. adc sample clk = 24M/(1+div)  div: 0~7.
 * @param[in]  div - variable plus one to divided by ADC source clock.
 * @return     none
 */
static inline void adc_set_sample_clk(unsigned char div)
{
	analog_write(anareg_adc_clk_div,  div & 0x07 );
}


/**************************************************************************************
afe_0xF5<7:0>  	rng[7:0]   		Read only, random number [7:0]
afe_0xF6<7:0>	rng[15:8]  		Read only, random number [15:8]
afe_0xF7<7:0>   adc_dat[7:0]  	Read only, Misc adc dat[7:0]
afe_0xF8<7:0>   adc_dat[15:8]  	Read only
								[7]: vld, ADC data valid status bit (This bit will be set as 1 at the end of capture state to indicate the ADC data is valid, and will be cleared when set state starts.)
								[6:0]:  Misc adc_dat[14:8]

 *************************************************************************************/
#define    anareg_adc_rng_l        		0xF5
#define    anareg_adc_rng_h        		0xF6
#define    anareg_adc_misc_l        	0xF7
#define    anareg_adc_misc_h        	0xF8
/**
 *  Check MISC channel capture status,return 1 indicates MISC capture data is valid.
 */
enum{
	FLD_ADC_MISC_DATA   = BIT_RNG(0,6),
	FLD_ADC_MISC_VLD    = BIT(7),
};

#define		CHECK_ADC_MISC_STATUS		((analog_read(anareg_adc_misc_h) & FLD_ADC_MISC_VLD)?1:0)


/**************************************************************************************
afe_0xFA
		BIT<1:0>  adc_itrim_preamp, Comparator preamp bias current trimming
		BIT<3:2>  adc_itrim_vrefbuf, Vref buffer bias current trimming
		BIT<5:4>  adc_itrim_vcmbuf, Vref buffer bias current trimming

		BIT<7:6>  Analog input pre-scaling select
					sel_ai_scale[1:0]: scaling factor
							0x0: 1
							0x1: 1/2
							0x2: 1/4
							0x3: 1/8

 *************************************************************************************/
#define    anareg_ain_scale        		0xFA	//poweron_dft: 0x00
enum{
	FLD_ADC_ITRIM_PREAMP 	= BIT_RNG(0,1),
	FLD_ADC_ITRIM_VREFBUF	= BIT_RNG(2,3),
	FLD_ADC_ITRIM_VCMBUF	= BIT_RNG(4,5),

	FLD_SEL_AIN_SCALE 		= BIT_RNG(6, 7),
};

typedef enum{
	ADC_PRESCALER_1,
	ADC_PRESCALER_1F2,
	ADC_PRESCALER_1F4,
	ADC_PRESCALER_1F8
}ADC_PreScalingTypeDef;

/**
 *
 */
typedef enum{
	 ADC_CUR_TRIM_PER75,
	 ADC_CUR_TRIM_PER100,
	 ADC_CUR_TRIM_PER125,
	 ADC_CUR_TRIM_PER150
}CUrrent_TrimTypeDef;

/**
 * @brief      This function sets pre-scaling for comparator preamp bias current trimming.
 * @param[in]  bias - enum variable of current trimming.
 * @return     none
 */
static inline void adc_set_itrim_preamp(CUrrent_TrimTypeDef bias)
{
	analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(~FLD_ADC_ITRIM_PREAMP)) | (bias<<0) );
}

/**
 * @brief      This function sets pre-scaling for Vref buffer bias current trimming.
 * @param[in]  bias - enum variable of current trimming.
 * @return     none
 */
static inline void adc_set_itrim_vrefbuf(CUrrent_TrimTypeDef bias)
{
	analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(~FLD_ADC_ITRIM_VREFBUF)) | (bias<<2) );
}

/**
 * @brief      This function sets pre-scaling for Vref(Vcmbuf) buffer bias current trimming.
 * @param[in]  bias - enum variable of current trimming.
 * @return     none
 */
static inline void adc_set_itrim_vcmbuf(CUrrent_TrimTypeDef bias)
{
	analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(~FLD_ADC_ITRIM_VCMBUF)) | (bias<<4) );
}


/**************************************************************************************
afe_0xFB
    BIT<5:4>  pga_itrim_boost_l
    BIT<7:6>  pga_itrim_boost_r
    			0x0 :   75%
				0x1 :   100%
				0x2 :   125%
				0x3 :   150%

afe_0xFC
    BIT<1:0>  pga_itrim_gain_l
    BIT<3:2>  pga_itrim_gain_r
    			0x0 :   75%
				0x1 :   100%
				0x2 :   125%
				0x3 :   150%

    BIT<4>	  adc_mode
    					0: normal mode
						1: rsvd (RNS mode)
	BIT<5>	  adc_pd, Power down ADC
						1: Power down
						0: Power up
	BIT<6>    pga_pd_l, power down left channel pga
						1: Power down
						0: Power up
	BIT<7>    pga_pd_r, power down right channel pga
						1: Power down
						0: Power up
 *************************************************************************************/
enum{                                              //ana_0xFB	//poweron_dft: 0xf0
	FLD_PGA_CAP_TRIM_EN_L		= BIT(0),
	FLD_PGA_CAP_TRIM_EN_R		= BIT(1),
	FLD_PGA_ITRIM_BOOST_L		= BIT_RNG(4,5),
	FLD_PGA_ITRIM_BOOST_R		= BIT_RNG(6,7),
};

#define anareg_adc_pga_ctrl				0xFC	//poweron_dft: 0xe0
enum{                                              //ana_0xFC
	FLD_PGA_ITRIM_GAIN_L		= BIT_RNG(0,1),
	FLD_PGA_ITRIM_GAIN_R		= BIT_RNG(2,3),
	FLD_ADC_MODE				= BIT(4),     //0 for  normal mode
	FLD_SAR_ADC_POWER_DOWN 		= BIT(5),
	FLD_POWER_DOWN_PGA_CHN_L 	= BIT(6),
	FLD_POWER_DOWN_PGA_CHN_R 	= BIT(7),
};

#define anareg_adc_pga_sel_vin				0xFD

enum{
	PGA_AIN_OFF = 0,
	PGA_AIN_ON = 1,
};

enum{                                              //ana_0xFC
	FLD_PGA_SEL_VIN_LEFT_P		= BIT_RNG(0,1),
	FLD_PGA_SEL_VIN_LEFT_N		= BIT_RNG(2,3),
	FLD_PGA_SEL_VIN_RIGHT_P		= BIT_RNG(4,5),
	FLD_PGA_SEL_VIN_RIGHT_N		= BIT_RNG(6,7),
};


typedef enum{
	NORMAL_MODE      = 0,
	RNS_MODE         = BIT(4),
}ADCModeTypeDef;


/**
 * Name     :adc_set_mode
 * Function :Set the source and mode of the random number generator
 * Input    :unsigned char stat
 *          :RNG_SrcTypeDef|RNG_UpdataTypeDef
 * return   :None
 */
static inline void adc_set_mode(ADCModeTypeDef adc_m)
{
	analog_write (anareg_adc_pga_ctrl, (analog_read(anareg_adc_pga_ctrl)&(~FLD_ADC_MODE)) | adc_m);

}

/**
 * @brief      This function sets sar_adc power.
 * @param[in]  on_off - on _off = 1 : power on. on _off = 0 : power off.
 * @return     none
 */
static inline void adc_power_on_sar_adc(unsigned char on_off)
{
	analog_write (anareg_adc_pga_ctrl, (analog_read(anareg_adc_pga_ctrl)&(~FLD_SAR_ADC_POWER_DOWN)) | (!on_off)<<5  );
}

/**
 * @brief      This function sets PGA-Left-channel power.
 * @param[in]  on_off - on _off = 1 : power on. on _off = 0 : power off.
 * @return     none
 */
static inline void pga_left_chn_power_on(unsigned char on_off)
{
	analog_write (anareg_adc_pga_ctrl, (analog_read(anareg_adc_pga_ctrl)&(~FLD_POWER_DOWN_PGA_CHN_L)) | (!on_off)<<6 );
}

/**
 * @brief      This function sets PGA-Right-channel power.
 * @param[in]  on_off - on _off = 1 : power on. on _off = 0 : power off.
 * @return     none
 */
static inline void pga_right_chn_power_on(unsigned char on_off)
{
	analog_write (anareg_adc_pga_ctrl, (analog_read(anareg_adc_pga_ctrl)&(~FLD_POWER_DOWN_PGA_CHN_R)) | (!on_off)<<7 );
}

typedef enum{
	GAIN_STAGE_BIAS_PER75 = 0,
	GAIN_STAGE_BIAS_PER100,
	GAIN_STAGE_BIAS_PER125,
	GAIN_STAGE_BIAS_PER150,
}Gain_BiasTypeDef;

/**
 * @brief      This function sets left_boost_bias with PGA enable.
 * @param[in]  bias - Value of gain_stage.
 * @return     none
 */
static inline void adc_set_left_boost_bias(Gain_BiasTypeDef bias)
{
	analog_write(0xFB, (analog_read(0xFB)&(~FLD_PGA_ITRIM_BOOST_L)) | (bias<<4) | FLD_PGA_CAP_TRIM_EN_L );
}

/**
 * @brief      This function sets right_boost_bias with PGA enable.
 * @param[in]  bias - Value of gain_stage.
 * @return     none
 */
static inline void adc_set_right_boost_bias(Gain_BiasTypeDef bias)
{
	analog_write(0xFB, (analog_read(0xFB)&(~FLD_PGA_ITRIM_BOOST_R)) | (bias<<6) |  FLD_PGA_CAP_TRIM_EN_R);
}



static inline void adc_set_left_right_gain_bias(Gain_BiasTypeDef bias_L, Gain_BiasTypeDef bias_R)
{
	analog_write(0xFC, (analog_read(0xFC) & 0xF0) | (bias_L | bias_R<<2) );
}

/**
 * @brief      This function gets left_gain_bias with PGA enable.
 * @param[in]  bias - Value of gain_stage.
 * @return     none
 */
static inline void adc_set_left_gain_bias(Gain_BiasTypeDef bias)
{
	analog_write(0xFC, (analog_read(0xFC)&(~FLD_PGA_ITRIM_GAIN_L)) | (bias) );
}

/**
 * @brief      This function gets right_gain_bias with PGA enable.
 * @param[in]  bias - Value of gain_stage.
 * @return     none
 */
static inline void adc_set_right_gain_bias(Gain_BiasTypeDef bias)
{
	analog_write(0xFC, (analog_read(0xFC)&(~FLD_PGA_ITRIM_GAIN_R)) | (bias<<2) );
}



//ana_0xFD	//poweron_dft: 0x05
//ana_0xFE	//poweron_dft: 0xe5
//ana_0xFF	//poweron_dft: 0x00



/********************************	configure set state	  ****************************/


/********************************	configure capture state	  ****************************/

/**
 * ADC RNG channel source
 */
typedef enum {
	SAR_ADC_RNG_MODE   = 0,
	R_RNG_MODE_0       = 2,
	R_RNG_MODE_1       = 3,
	ADC_DAT12_RNG_MODE = 4,
	ADC_DAT5_RNG_MODE  = 6,
}RNG_SrcTypeDef;

typedef enum {
	READ_UPDATA        = BIT(3),
	CLOCLK_UPDATA      = BIT(4),
}RNG_UpdataTypeDef;








/**
 * @brief This function serves to set the channel reference voltage.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC reference voltage.
 * @return none
 */
void adc_set_ref_voltage(ADC_ChTypeDef ch_n, ADC_RefVolTypeDef v_ref);

/**
 * @brief This function serves to set resolution.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC resolution.
 * @return none
 */
void adc_set_resolution(ADC_ChTypeDef ch_n, ADC_ResTypeDef v_res);

/**
 * @brief This function serves to set sample_cycle.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC Sampling cycles.
 * @return none
 */
void adc_set_tsample_cycle(ADC_ChTypeDef ch_n, ADC_SampCycTypeDef adcST);

/**
 * @brief This function serves to set input_mode.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC channel input mode.
 * @return none
 */
void adc_set_input_mode(ADC_ChTypeDef ch_n,  ADC_InputModeTypeDef m_input);

/**
 * @brief This function serves to set input channel in single_ended_input_mode.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC analog positive input channel.
 * @return none
 */
void adc_set_ain_channel_single_ended_mode(ADC_ChTypeDef ch_n, ADC_InputPchTypeDef InPCH);

/**
 * @brief This function serves to set input channel in differential_mode.
 * @param[in]   enum variable of ADC input channel.
 * @param[in]   enum variable of ADC analog positive input channel.
 * @param[in]   enum variable of ADC analog negative input channel.
 * @return none
 */
void adc_set_ain_channel_differential_mode(ADC_ChTypeDef ch_n, ADC_InputPchTypeDef InPCH,ADC_InputNchTypeDef InNCH);

/**
 * @brief This function serves to set pre_scaling.
 * @param[in]   enum variable of ADC pre_scaling factor.
 * @return none
 */
void adc_set_ain_pre_scaler(ADC_PreScalingTypeDef v_scl);


/**
 * @brief This function serves to set adc sampling and get results.
 * @param[in]  none.
 * @return the result of sampling.
 */
unsigned int adc_sample_and_get_result(void);

/**
 * @brief This function serves to set the source and mode of the random number generator.
 * @param[in]  RNG_SrcTypeDef src.
 * @param[in]  RNG_UpdataTypeDef update_type.
 * @return none.
 */
static inline void RNG_Set(RNG_SrcTypeDef src,RNG_UpdataTypeDef update_type)
{
	WriteAnalogReg(0xfe, src | update_type);			//Set
}


/**
 * @brief This function serves to read the value of the random number generator.
 * @param[in]  none.
 * @return unsigned short RngValue random number.
 */
static inline unsigned short RNG_Read(void)
{
	return ( ReadAnalogReg(0xf6)<<8 |  ReadAnalogReg(0xf5) );
}
