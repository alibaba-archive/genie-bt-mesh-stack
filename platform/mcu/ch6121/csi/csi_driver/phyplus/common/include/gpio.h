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
* @file		gpio.h
* @brief	Contains all functions support for gpio and iomux driver
* @version	0.0
* @date		19. Oct. 2017
* @author	qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __GPIO_H__
#define __GPIO_H__
#include <types.h>
#include "ap_cp.h"

#define NUMBER_OF_PINS      35
#define NUMBER_OF_IRQ_PINS  18


#define gpio_write_reg(addr,data)      (*(volatile unsigned long  *)(addr)=data)
#define gpio_read_reg(addr)            (*(volatile unsigned long  *)(addr))

#define reg_gpio_ioe_porta			                 (volatile unsigned int  *)0x40008004
#define reg_gpio_ioe_portb			                 (volatile unsigned int  *)0x40008010
#define reg_gpio_swporta_dr			                 (volatile unsigned int  *)0x40008000
#define reg_gpio_swportb_dr			                 (volatile unsigned int  *)0x4000800c
#define reg_gpio_ext_porta			                 (volatile unsigned int  *)0x40008050
#define reg_gpio_ext_portb			                 (volatile unsigned int  *)0x40008054
#define FMUX_EN_ADD_BASE                (AP_IOMUX_BASE+full_mux_en)
#define DMUX_EN_ADD_BASE                (AP_IOMUX_BASE+debug_mux_en)
#define FMUX_FUNIO_ADD_BASE             (AP_IOMUX_BASE+gpio_base_sel)
#define IO_PULL_ADD_BASE                0x4000f008
#define IO_WAKEUP_ADD_BASE              0x4000f0a0
#define PAD_PE_ADD_BASE                 (AP_IOMUX_BASE+pad_pe)
#define PAD_DS_ADD_BASE                 (AP_IOMUX_BASE+pad_ds)
#define FMUX_FUNIO_SELECT(Pin,Type)     gpio_write_reg(FMUX_FUNIO_ADD_BASE+((Pin >>2) << 2),((gpio_read_reg(FMUX_FUNIO_ADD_BASE+((Pin >> 2) <<  2)))& (~(0xFF << ((Pin & 0x03) << 3))))|(Type << ((Pin & 0x03) << 3)))
//#define reg_gpio_oen	                   0x40008004
//#define GPIO_REG_ADDR_8(addr)                 (volatile unsigned char  *)(GPIO_REG_BASE+addr)

//AP_IOMUX->full_mux0_en |= BIT(pin);
#define REG_FMUX_EN_FUC(i)    (volatile unsigned int  *)(FMUX_EN_ADD_BASE + ((i >> 5) << 2))
#define REG_DMUX_EN_FUC       *(volatile unsigned int  *)(DMUX_EN_ADD_BASE)

#define REG_IO_WAKEUP_EN(i)   (volatile unsigned int  *)(IO_WAKEUP_ADD_BASE + ((i >> 5) << 2))
#define REG_IO_Wakeuo_Pol(i)  (volatile unsigned int  *)(IO_PULL_ADD_BASE + ((i/10) << 2))
#define REG_ANALOG_IO         (volatile unsigned int  *)(AP_IOMUX_BASE)
#define REG_IOPULL_IO(i)      (volatile unsigned int  *)(IO_PULL_ADD_BASE + ((i/10) << 2))
#define REG_PAD_EN(i)         (volatile unsigned int  *)(PAD_PE_ADD_BASE + ((i >> 5) << 2))
#define REG_PAD_DS(i)         (volatile unsigned int  *)(PAD_DS_ADD_BASE + ((i >> 5) << 2))
#define REG_SWPORT_DR(i)      (volatile unsigned int  *)(AP_GPIOA_BASE + ((i >> 5) << 2)*3)


typedef enum {
    IE, //input
    OEN, //output
} GPIO_ioe;


typedef enum {
    GPIO_P00   =   0,    P0  =  0,
    GPIO_P01   =   1,    P1  =  1,
    GPIO_P02   =   2,    P2  =  2,
    GPIO_P03   =   3,    P3  =  3,
    GPIO_P04   =   4,    P4  =  4,
    GPIO_P05   =   5,    P5  =  5,
    GPIO_P06   =   6,    P6  =  6,
    GPIO_P07   =   7,    P7  =  7,
    TEST_MODE  =   8,    P8  =  8,
    GPIO_P09   =   9,    P9  =  9,
    GPIO_P10   =   10,   P10  =  10,
    GPIO_P11   =   11,   P11  =  11,   Analog_IO_0 = 11,
    GPIO_P12   =   12,   P12  =  12,   Analog_IO_1 = 12,
    GPIO_P13   =   13,   P13  =  13,   Analog_IO_2 = 13,
    GPIO_P14   =   14,   P14  =  14,   Analog_IO_3 = 14,
    GPIO_P15   =   15,   P15  =  15,   Analog_IO_4 = 15,
    GPIO_P16   =   16,   P16  =  16,   XTALI = 16,
    GPIO_P17   =   17,   P17  =  17,   XTALO = 17,
    GPIO_P18   =   18,   P18  =  18,   Analog_IO_7 = 18,
    GPIO_P19   =   19,   P19  =  19,   Analog_IO_8 = 19,
    GPIO_P20   =   20,   P20  =  20,   Analog_IO_9 = 20,
    GPIO_P21   =   21,   P21  =  21,
    GPIO_P22   =   22,   P22  =  22,
    GPIO_P23   =   23,   P23  =  23,
    GPIO_P24   =   24,   P24  =  24,
    GPIO_P25   =   25,   P25  =  25,
    GPIO_P26   =   26,   P26  =  26,
    GPIO_P27   =   27,   P27  =  27,
    GPIO_P28   =   28,   P28  =  28,
    GPIO_P29   =   29,   P29  =  29,
    GPIO_P30   =   30,   P30  =  30,
    GPIO_P31   =   31,   P31  =  31,
    GPIO_P32   =   32,   P32  =  32,
    GPIO_P33   =   33,   P33  =  33,
    GPIO_P34   =   34,   P34  =  34,
    GPIO_DUMMY  =  0xff,
} GPIO_Pin_e;

typedef enum {
    Bit_DISABLE = 0,
    Bit_ENABLE,
} BitAction_e;

typedef enum {
    FLOATING = 0x00,
    WEAK_PULL_UP = 0x01,
    STRONG_PULL_UP = 0x02,
    PULL_DOWN = 0x03,
} IO_Pull_Type_e;

typedef enum {
    POSEDGE,
    NEGEDGE
} IO_Wakeup_Pol_e;

typedef enum {

    IIC0_SCL = 0,
    IIC0_SDA = 1,
    IIC1_SCL = 2,
    IIC1_SDA = 3,
    I2S_SCK = 4,
    I2S_WS = 5,
    I2S_SDO_0 = 6,
    I2S_SDI_0 = 7,
    UART_TX = 8,
    UART_RX = 9,
    PWM0,
    PWM1,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    SPI_0_SCK,
    SPI_0_SSN,
    SPI_0_TX,
    SPI_0_RX,
    SPI_1_SCK,
    SPI_1_SSN,
    SPI_1_TX,
    SPI_1_RX,
    CHAX,
    CHBX,
    CHIX,
    CHAY,
    CHBY,
    CHIY,
    CHAZ,
    CHBZ,
    CHIZ,
    CLK1P28M,
    ADCC,

    I2S_SDO_1 = 35,
    I2S_SDO_2 = 36,
    I2S_SDO_3 = 37,
    I2S_SDI_1 = 38,
    I2S_SDI_2 = 39,
    I2S_SDI_3 = 40

} Fmux_Type_e;

#define Analog_IO_en  0x00
#define debug_mux_en  0x08
#define full_mux_en   0x0c  // 0xc
#define gpio_pad_en   0x14
#define gpio_base_sel 0x18
#define pad_pe        0x3c
#define pad_ds        0x44

typedef void (*gpioin_Hdl_t)(GPIO_Pin_e pin, IO_Wakeup_Pol_e type);

bool phy_gpio_read(GPIO_Pin_e pin);
int phy_gpio_write(GPIO_Pin_e pin, uint8_t en);
int phy_gpio_toggle(GPIO_Pin_e pin);
int phy_gpio_pin_init(GPIO_Pin_e pin, GPIO_ioe type);
int phy_gpio_cfg_analog_io(GPIO_Pin_e pin, BitAction_e value);
int phy_gpio_pin0to3_pin31to34_control(GPIO_Pin_e pin, uint8_t en);
int phy_gpio_DS_control(GPIO_Pin_e pin, BitAction_e value);
int phy_gpio_fmux(GPIO_Pin_e pin, BitAction_e value);
int phy_gpio_fmux_set(GPIO_Pin_e pin, Fmux_Type_e type);
int phy_gpio_pull_set(GPIO_Pin_e pin, IO_Pull_Type_e type);
int phy_gpio_wakeup_set(GPIO_Pin_e pin, IO_Wakeup_Pol_e type);
int phy_gpioin_enable(GPIO_Pin_e pin);
int phy_gpioin_disable(GPIO_Pin_e pin);
int phy_gpioin_register(GPIO_Pin_e pin, gpioin_Hdl_t posedgeHdl, gpioin_Hdl_t negedgeHdl);
int phy_gpioin_unregister(GPIO_Pin_e pin);

int phy_gpio_init(void);

void __attribute__((weak)) phy_GPIO_IRQHandler(void);

void phy_gpio_p00_to_hclk_div8_enable(void);
void phy_gpio_p00_to_hclk_div8_disable(void);
void phy_gpio_p01_to_pclk_div4_enable(void);
void phy_gpio_p01_to_pclk_div4_disable(void);
void phy_gpio_p24_to_rc32k_enable(void);
void phy_gpio_p24_to_rc32k_disable(void);
void phy_gpio_p25_to_xtal_clk32k_enable(void);
void phy_gpio_p25_to_xtal_clk32k_disable(void);

#endif
