/**************************************************************************************************

  Phyplus Microelectronics Limited. All rights reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

**************************************************************************************************/



/*******************************************************************************
* @file		common.h
* @brief	Contains all functions support for common function driver£¬such as string function,you can use this driver for spi,adc,uart and so on
* @version	0.0
* @date		19. Oct. 2017
* @author	qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __COMMON__H__
#define __COMMON__H__
#include "types.h"

#define   CLAER_RTC_COUNT   *(volatile unsigned int *)0x4000f024 |= BIT(1)
#define   RUN_RTC           *(volatile unsigned int *)0x4000f024 |= BIT(0)
#define   STOP_RTC          *(volatile unsigned int *)0x4000f024 &= ~BIT(0)

#define TIME_BASE               (1000000) // 24bit count shift 2 bit as 1us/bit
#define TIME_DELTA(x,y)         ( (x>=y) ? x-y : TIME_BASE-y+x )

/**************************************************************************************
 * @fn          hal_setMem
 *
 * @brief       This function process for set some memory addr with a value
 *
 * input parameters
 *
 * @param       uint8_t *buf: set memory buffer
 *              uint8_t value: memory value
 *              uint32_t length: set memory length
 *
 * output parameters
 *
 * @param       uint8_t *buf: set memory buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_setMem(uint8_t  *buf, uint8_t value, uint32_t length);

/**************************************************************************************
 * @fn          hal_cpyMem
 *
 * @brief       This function process for copying data from source addr to dest addr,once copy one byte
 *
 * input parameters
 *
 * @param       uint8_t *dst: copy destnation buffer
 *              uint8_t *src: copy source buffer
 *              uint32_t length: copy length
 *
 * output parameters
 *
 * @param       uint8_t *dst: copy destnation buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_cpyMem(uint8_t *dst, uint8_t *src, uint32_t length);

/**************************************************************************************
 * @fn          hal_cpyMem32
 *
 * @brief       This function process for copying data from source addr to dest addr,once copy 4 bytes
 *
 * input parameters
 *
 * @param       uint32_t *dst: copy destnation buffer
 *              uint32_t *src: copy source buffer
 *              uint32_t length: copy length
 *
 * output parameters
 *
 * @param       uint32_t *dst: copy destnation buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_cpyMem32(uint32_t *dst, uint32_t *src, uint32_t length);

/**************************************************************************************
 * @fn          hal_my_strcmp
 *
 * @brief       This function process for compare two strings, return  1 means same, 0 means different
 *
 * input parameters
 *
 * @param       const uint8_t *str: the first string
 *              const uint8_t *ptr: the second string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      1:the same
 *              0:the different
 **************************************************************************************/
int hal_my_strcmp(const uint8_t *str,  const uint8_t *ptr);

/**************************************************************************************
 * @fn          hal_copy_bin_from_flash_to_sram
 *
 * @brief       This function process for copy bin from flash to sram
 *
 * input parameters
 *
 * @param       int toAddr: destnation address
 *              int fromAddr: source address
 *              int length: copy length
 *
 * output parameters
 *
 * @param       (uint8_t *) toAddr: destnation buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_copy_bin_from_flash_to_sram(int toAddr, int fromAddr, int length);

/**************************************************************************************
 * @fn          hal_my_sizeof
 *
 * @brief       This function process for calculate the string length
 *
 * input parameters
 *
 * @param       const uint8_t *str: the source string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      the string length(int)
 **************************************************************************************/
int hal_my_sizeof(const uint8_t *str);

/**************************************************************************************
 * @fn          hal_my_strlen
 *
 * @brief       This function process for calculate the string length,PS:the char[] must give the '\0'
 *
 * input parameters
 *
 * @param       const uint8_t *str: the source string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      the string length(int)
 **************************************************************************************/
int hal_my_strlen(const uint8_t *str);

/**************************************************************************************
 * @fn          hal_convert_hex_to_char
 *
 * @brief       This function process for convert a hex data to ASCII code type,include the 0x symbol
 *
 * input parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *              unsigned int data: the source hex data
 *
 * output parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_convert_hex_to_char(unsigned char *ch,  unsigned int data);

/**************************************************************************************
 * @fn          hal_convert_hex_to_char_wo_x
 *
 * @brief       This function process for convert a hex data to ASCII code type,without the 0x symbol
 *
 * input parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *              unsigned int data: the source hex data
 *
 * output parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_convert_hex_to_char_wo_x(unsigned char *ch,  unsigned int data);

/**************************************************************************************
 * @fn          hal_convert_char_to_hex
 *
 * @brief       This function process for convert a data from ASCII code to hex type
 *
 * input parameters
 *
 * @param       const unsigned char *ch: the source char string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hex data
 **************************************************************************************/
uint32_t hal_convert_char_to_hex(const unsigned char *ch);

/**************************************************************************************
 * @fn          hal_convert_char_to_dec
 *
 * @brief       This function process for convert a data from ASCII code to decimal type
 *
 * input parameters
 *
 * @param       const unsigned char *ch: the source char string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      decimal data
 **************************************************************************************/
uint32_t hal_convert_char_to_dec(const unsigned char *ch);

/**************************************************************************************
 * @fn          hal_is_an_valid_number
 *
 * @brief       This function process for judge if a char is hex number or not, return  1 means yes, 0 means no
 *
 * input parameters
 *
 * @param       uint8_t ch: the source data
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      1: yes
 *              0: no
 **************************************************************************************/
int hal_is_an_valid_number(uint8_t ch);

/**************************************************************************************
 * @fn          clock_time_rtc
 *
 * @brief       This function process for return rtc count
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      RTC count
 **************************************************************************************/
//static inline uint32_t clock_time_rtc(void){
//	return (*(volatile unsigned int *)0x4000f028) & 0xffffff;
//}
//
extern uint32_t clock_time_rtc(void);

/**************************************************************************************
 * @fn          subWriteReg
 *
 * @brief       This function process for write register with sub bit
 *
 * input parameters
 *
 * @param       uint32_t addr: register address
 *              uint8_t hOff: high bit offset
 *              uint8_t lOff: low bit offset
 *              uint32_t value: write value
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void subWriteReg(uint32_t addr, uint8_t hOff, uint8_t lOff, uint32_t value);

/**************************************************************************************
 * @fn          WaitMs
 *
 * @brief       This function process for wait program msecond,use RTC
 *
 * input parameters
 *
 * @param       uint32_t msecond: the msecond value
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void WaitMs(uint32_t msecond);

#define WR_REG(a,b)    *(volatile unsigned int *)a=b
#define RD_REG(a)      *(volatile unsigned int *)a
#define ADD_IDX(a,b)   (a==b)? a=0:a++;

//below is system initial defination
#define    ENABLE_XTAL_TRIGGER   *(volatile unsigned int *)0x4000f040 |= BIT(16)
#define    ENABLE_XTAL_OUTPUT    *(volatile unsigned int *)0x4000f040 |= BIT(18)
#define    ENABLE_DLL            *(volatile unsigned int *)0x4000f044 |= BIT(7)
#define    DLL32M_CLOCK_ENABLE   *(volatile unsigned int *)0x4000f044 |= BIT(13)
#define    DLLn_CLOCK_ENABLE(n)  *(volatile unsigned int *)0x4000f044 |= BIT(n+11)
#define    DBLE_CLOCK_DISABLE    *(volatile unsigned int *)0x4000f044 &= ~BIT(21)
#define    CLK_1P28M_ENABLE      *(volatile unsigned int *)0x4000f03c |= BIT(6)       //the source is 32M dll
#define    HCLK_DIV2_ENABLE      *(volatile unsigned int *)0x4000f03c |= BIT(15)      //use carefully
#define    PCLK_DIV_ENABLE       *(volatile unsigned int *)0x4000001c & 0x02


enum  H_SYSCLK_SEL {
    SYS_CLK_RC_32M      = 0,
    SYS_CLK_DLL_32M     = 1,
    SYS_CLK_XTAL_16M    = 2,//RF OK
    SYS_CLK_DLL_48M     = 3,//RF OK
    SYS_CLK_DLL_64M     = 4,
    SYS_CLK_DLL_96M     = 5,
    SYS_CLK_DBL_32M     = 6
};

enum  LOWCLK_SEL {
    RC_32K,
    XTAL_32K
};

/**************************************************************************************
 * @fn          wdt_clk_sel
 *
 * @brief       This function process for low clock select; 0----RC 32k   1----XTAL 32k
 *
 * input parameters
 *
 * @param       enum LOWCLK_SEL clk:  0----RC 32k   1----XTAL 32k
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static inline void wdt_clk_sel(enum LOWCLK_SEL clk)
{
    if (clk) {
        BM_SET((volatile unsigned int *)0x4000f03c, BIT(16));  //use xtal 32k
    } else {
        BM_CLR((volatile unsigned int *)0x4000f03c, BIT(16));  //use rc 32k
    }
}

/**************************************************************************************
 * @fn          hal_system_init
 *
 * @brief       This function process for system initial,you can select diff source,such as RC_32M XTAL_16M and so on
 *
 * input parameters
 *
 * @param       uint8_t h_system_clk_sel: system clock select  SYS_CLK_RC_32M rc32M
 * 																																			 SYS_CLK_DLL_32M dll32M
 *																																			 SYS_CLK_XTAL_16M xtal16M
 *																																			 SYS_CLK_DLL_48M dll48M
 *																																		   SYS_CLK_DLL_64M dll64M
 *																																	     SYS_CLK_DLL_96M dll96M
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_system_init(uint8_t h_system_clk_sel);

void WaitUs(uint32_t wtTime);

void WaitRTCCount(uint32_t rtcDelyCnt);

uint8 char_array_cmp(uint8 *s1, uint8 *s2, uint8 len);

void hal_system_soft_reset(void);

void hal_ret_sram_enable(uint8_t sram_index);

void hal_ret_sram_enable_all(void);
#endif
