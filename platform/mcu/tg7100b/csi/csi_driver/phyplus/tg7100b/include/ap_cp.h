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



/**************************************************************************************************
  Filename:       ap_cp.h
  Revised:
  Revision:

  Description:    This file contains the SoC MCU relate definitions

 **************************************************************************************************/

#ifndef __AP_CP_H__
#define __AP_CP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "soc.h" //chenlf
//#include "core_cm0.h" //chenlf

#ifdef CFG_CP
#include "ARMCM0.h"
#endif

#ifdef CFG_AP
#if defined (ARMCM4)
#include "ARMCM4.h"
#elif defined (ARMCM4_FP)
#include "ARMCM4_FP.h"
#else
#error device not specified!
#endif
#endif

/*******************************************************************************
 * TYPEDEFS
 */
/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
typedef struct {
    __IO uint32_t  CH0_AP_MBOX;
    __IO uint32_t  CH0_CP_MBOX;
    __IO uint32_t  CH1_AP_MBOX;
    __IO uint32_t  CH1_CP_MBOX;
    __IO uint32_t  AP_STATUS;
    __IO uint32_t  CP_STATUS;
    __IO uint32_t  AP_INTEN;
    __IO uint32_t  CP_INTEN;
    __IO uint32_t   remap;
    __IO uint32_t   RXEV_EN;
    __IO uint32_t   STCALIB;
    __IO uint32_t  PERI_MASTER_SELECT;
} AP_COM_TypeDef;

typedef struct {
    __IO uint8_t  CR;   //0x0
    uint8_t  RESERVED0[3];
    __IO uint32_t TORR;  //0x4
    __O  uint32_t CCVR;  //0x8
    __IO uint32_t CRR;    //0xc
    uint8_t  STAT;   //0x10
    uint8_t  reserverd1[3];
    __IO uint8_t  EOI;    //0x14
    uint8_t  reserverd2[3];
} AP_WDT_TypeDef;

typedef struct {
    __IO uint32_t  RESET;   //0x0
    __IO uint32_t  RESET1;   //0x4
    __IO uint32_t  CLKG;   //0x8
    __IO uint32_t RESET2;   //0xc
    __IO uint32_t RESET3;   //0x10
    __IO uint32_t CLKG1;   //0x14
    __IO uint32_t APB_CLK;   //0x18
    __IO uint32_t APB_CLK_U;   //0x1c

} AP_PCR_TypeDef;

typedef struct {
    __IO uint32_t LoadCount;  //0x0
    __IO uint32_t CurrentCount;  //0x4
    __IO uint32_t ControlReg;  //0x8
    __IO uint32_t EOI;  //0xc
    __IO uint32_t status;   //0x10

} AP_TIM_TypeDef;

typedef struct {
    __IO uint32_t IntStatus;
    __IO uint32_t EOI;
    __IO uint32_t unMaskIntStatus;
    __IO uint32_t version;
} AP_TIM_SYS_TypeDef;


#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- Universal Asynchronous Receiver Transmitter (UARTx) -----------*/

typedef struct {
    union {
        __I  uint8_t  RBR;
        __IO  uint8_t  THR;
        __IO uint8_t  DLL;
        uint32_t RESERVED0;    //0x0
    };
    union {
        __IO uint8_t  DLM;
        __IO uint32_t IER;   //0x4
    };
    union {
        __I  uint32_t IIR;   //0x8
        __IO  uint8_t  FCR;
    };
    __IO uint8_t  LCR;   //0xc
    uint8_t  RESERVED1[3];//Reserved
    __IO uint32_t MCR; //0x10

    __I  uint8_t  LSR;  //0x14
    uint8_t  RESERVED2[3];//Reserved
    __IO uint32_t MSR;   //0x18

    __IO uint8_t  SCR;   //0x1c
    uint8_t  RESERVED3[3];//Reserved

    __IO uint32_t LPDLL;  //0x20

    __IO uint32_t LPDLH;  //0x24

    __IO uint32_t  recerved[2];

    union {
        __IO uint32_t SRBR[16];  // 0x30~60xc
        __IO uint32_t STHR[16];
    };
    __IO uint32_t  FAR;  //0x70

    __IO uint32_t  TFR;  //0x74

    __IO uint32_t  RFW;  // 0x78

    __IO uint32_t  USR;  // 0x7c

    __IO uint32_t  TFL;

    __IO uint32_t  RFL;

    __IO uint32_t  SRR;

    __IO uint32_t  SRTS;

    __IO uint32_t  SBCR;

    __IO uint32_t  SDMAM;

    __IO uint32_t  SFE;

    __IO uint32_t  SRT;

    __IO uint32_t  STET;  //0xa0

    __IO uint32_t  HTX;

    __IO uint32_t  DMASA;  //0xa8

    __IO uint32_t  reserved[18];

    __IO uint32_t  CPR;  //0xf4

    __IO uint32_t  UCV;

    __IO uint32_t  CTR;

} AP_UART_TypeDef;


/*------------- Inter-Integrated Circuit (I2C)  setup by zjp-------------------------------*/
typedef struct {
    __IO uint32_t  IC_CON;
    __IO uint32_t  IC_TAR;
    __IO uint32_t  IC_SAR;
    __IO uint32_t  IC_HS_MADDR;
    __IO uint32_t  IC_DATA_CMD; //0x10
    __IO uint32_t  IC_SS_SCL_HCNT;
    __IO uint32_t  IC_SS_SCL_LCNT;
    __IO uint32_t  IC_FS_SCL_HCNT;
    __IO uint32_t  IC_FS_SCL_LCNT; //0x20
    __IO uint32_t  IC_HS_SCL_HCNT;
    __IO uint32_t  IC_HS_SCL_LCNT;
    __IO uint32_t  IC_INTR_STAT;
    __IO uint32_t  IC_INTR_MASK;  //0x30
    __IO uint32_t  IC_RAW_INTR_STAT;
    __IO uint32_t  IC_RX_TL;
    __IO uint32_t  IC_TX_TL;
    __IO uint32_t  IC_CLR_INTR;  //0x40
    __IO uint32_t  IC_CLR_UNDER;
    __IO uint32_t  IC_CLR_RX_OVER;
    __IO uint32_t  IC_CLR_TX_OVER;
    __IO uint32_t  IC_CLR_RD_REG;  //0x50
    __IO uint32_t  IC_CLR_TX_ABRT;
    __IO uint32_t  IC_CLR_RX_DONE;
    __IO uint32_t  IC_CLR_ACTIVITY;
    __IO uint32_t  IC_CLR_STOP_DET;  //0x60
    __IO uint32_t  IC_CLR_START_DET;
    __IO uint32_t  IC_CLR_GEN_CALL;
    __IO uint32_t  IC_ENABLE;
    __IO uint32_t  IC_STATUS;  //0x70
    __IO uint32_t  IC_TXFLR;
    __IO uint32_t  IC_RXFLR;
    __IO uint32_t  IC_SDA_HOLD;
    __IO uint32_t  IC_TX_ABRT_SOURCE;  //0x80
    __IO uint32_t  IC_SLV_DATA_NACK_ONLY;
    __IO uint32_t  IC_DMA_CR;
    __IO uint32_t  IC_DMA_TDLR;
    __IO uint32_t  IC_DMA_RDLR; //0x90
    __IO uint32_t  IC_SDA_SETUP;
    __IO uint32_t  IC_ACK_GENERAL_CALL;
    __IO uint32_t  IC_ENABLE_STATUS;
    __IO uint32_t  IC_FS_SPKLEN;  //0xa0
    __IO uint32_t  IC_HS_SPKLEN;

} AP_I2C_TypeDef;


/*------------- Inter IC Sound (I2S) -----------------------------------------*/
typedef struct {
    __IO uint32_t IER;
    __IO uint32_t IRER;
    __IO  uint32_t ITER;
    __IO  uint32_t CER;
    __IO  uint32_t CCR;
    __IO uint32_t RXFFR;
    __IO uint32_t TXFFR;

} AP_I2S_BLOCK_TypeDef;

typedef struct {
    union {
        __IO uint32_t LRBR;       //0x20
        __IO uint32_t LTHR;       //0x20
    };
    union {
        __IO uint32_t RRBR;       // 0x24
        __IO uint32_t RTHR;       //0x24
    };
    __IO uint32_t RER;            //0x28
    __IO uint32_t TER;            //0x2c
    __IO uint32_t RCR;            //0x30
    __IO uint32_t TCR;            //0x34
    __IO uint32_t ISR;            //0x38
    __IO uint32_t IMR;            //0x3c
    __IO uint32_t ROR;            //0x40
    __IO uint32_t TOR;            //0x44
    __IO uint32_t  RFCR;          //0x48
    __IO uint32_t  TFCR;          //0x4c
    __IO uint32_t  RFF;           //0x50
    __IO uint32_t  TFF;           //0x54

} AP_I2S_TypeDef;


/*------------- General Purpose Input/Output (GPIO) --------------------------*/
typedef struct {
    __IO uint32_t swporta_dr;                              //0x00
    __IO uint32_t swporta_ddr;                             //0x04
    __IO uint32_t swporta_ctl;                             //0x08
    __IO uint32_t swportb_dr;                              //0x0c
    __IO uint32_t swportb_ddr;                             //0x10
    __IO uint32_t swportb_ctl;                             //0x14
    uint32_t reserved8[6];                            //0x18-0x2c portC&D
    __IO uint32_t inten;		                           //0x30
    __IO uint32_t intmask;                                 //0x34
    __IO uint32_t inttype_level;                           //0x38
    __IO uint32_t int_polarity;                            //0x3c
    __I  uint32_t int_status;                              //0x40
    __IO uint32_t raw_instatus;                            //0x44
    __IO uint32_t debounce;                                //0x48
    __O  uint32_t porta_eoi;                               //0x4c
    __I  uint32_t ext_porta;                               //0x50
    __I  uint32_t ext_portb;                               //0x54
    uint32_t reserved9[2];                            //0x58 0x5c
    __IO uint32_t ls_sync;                                 //0x60
    __I  uint32_t id_code;                                 //0x64
    uint32_t reserved10[1];                           //0x68
    __I  uint32_t ver_id_code;                             //0x6c
    __I  uint32_t config_reg2;                             //0x70
    __I  uint32_t config_reg1;                             //0x74
} AP_GPIO_TypeDef;


/*--------------------  (SPI) --------------------------------*/
typedef struct {
    __IO uint16_t     CR0;           //0x0                /*!< Offset: 0x000 Control Register 0 (R/W) */
    uint16_t     reserved1;
    __IO uint16_t     CR1;           //0x04                 /*!< Offset: 0x004 Control Register 1 (R/W) */
    uint16_t     reserved2;
    __IO uint8_t      SSIEN;         //0x08
    uint8_t      reserved3[3];
    __IO uint8_t      MWCR;          // 0x0c
    uint8_t      reserved4[3];
    __IO uint8_t      SER;           //0x10
    uint8_t      reserved5[3];
    __IO uint16_t     BAUDR;         //0x14
    uint16_t     reserved6;
    __IO uint32_t     TXFTLR;        //0x18
    __IO uint32_t     RXFTLR;        //0x1c
    __O  uint32_t     TXFLR;         //0x20
    __O  uint32_t     RXFLR;         //0x24

    __IO uint8_t      SR;            //0x28
    uint8_t      reserved7[3];
    __IO uint32_t     IMR;           //0x2c
    __IO uint32_t     ISR;           //0x30
    __IO uint32_t     RISR;          //0x34
    __IO uint32_t     TXOICR;        //0x38
    __IO uint32_t     RXOICR;        //0x3c
    __IO uint32_t     RXUICR;        //0x40
    __IO uint32_t     MSTICR;        //0x44
    __IO uint32_t     ICR;           //0x48
    __IO uint32_t     DMACR;         //0x4c
    __IO uint32_t     DMATDLR;       //0x50
    __IO uint32_t     DMARDLR;       //0x54
    __IO uint32_t     IDR;           //0x5c
    __IO uint32_t     SSI_COM_VER;   //0x5c
    __IO uint16_t     DataReg[72];
    __IO uint8_t      RX_SAMPLE_DLY;

} AP_SSI_TypeDef;

typedef struct {

    __IO uint32_t    Analog_IO_en;
    __IO uint32_t    SPI_debug_en;
    __IO uint32_t    debug_mux_en;
    __IO uint32_t    full_mux0_en;         // 0xc
    __IO uint32_t    full_mux1_en;         // 0x10
    __IO uint32_t    gpio_pad_en;
    __IO uint32_t    gpio_0_3_sel;
    __IO uint32_t    gpio_4_7_sel;
    __IO uint32_t    gpio_8_11_sel;         //0x20
    __IO uint32_t    gpio_12_15_sel;
    __IO uint32_t    gpio_16_19_sel;
    __IO uint32_t    gpio_20_23_sel;
    __IO uint32_t    gpio_24_27_sel;        //0x30
    __IO uint32_t    gpio_28_31_sel;
    __IO uint32_t    gpio_32_34_sel;
    __IO uint32_t    pad_pe0;
    __IO uint32_t    pad_pe1;               //0x40
    __IO uint32_t    pad_ps0;
    __IO uint32_t    pad_ps1;
    __IO uint32_t    keyscan_in_en;
    __IO uint32_t    keyscan_out_en;        //0x50

} IOMUX_TypeDef;

typedef struct {

    __IO uint32_t    PWROFF;         //0x00
    __IO uint32_t    PWRSLP;         //0x04
    __IO uint32_t    IOCTL0;         //0x08
    __IO uint32_t    IOCTL1;         // 0x0c
    __IO uint32_t    IOCTL2;         // 0x10
    __IO uint32_t    PMCTL0;         // 0x14
    __IO uint32_t    PMCTL1;         //0x18
    __IO uint32_t    PMCTL2_0;         //0x1c
    __IO uint32_t    PMCTL2_1;         //0x20
} AP_AON_TypeDef;

typedef struct {
    __IO uint32_t    RTCCTL;          //0x24
    __IO uint32_t    RTCCNT;          //0x28
    __IO uint32_t    RTCCC0;          //0x2c
    __IO uint32_t    RTCCC1;          //0x30
    __IO uint32_t    RTCCC2;          //0x34
    __IO uint32_t    RTCFLAG;         //0x38
} AP_RTC_TypeDef;

typedef struct {
    __IO uint32_t    io_wu_mask_31_0;          //0xa0
    __IO uint32_t    io_wu_mask_34_32;          //0xa4
} AP_Wakeup_TypeDef;

typedef struct {
    __IO uint32_t    CLKSEL;          //0x3c
    __IO uint32_t    CLKHF_CTL0;          //0x40
    __IO uint32_t    CLKHF_CTL1;          //0x44
    __IO uint32_t    ANA_CTL;          //0x48
    __IO uint32_t    mem_0_1_dvs;          //0x4c
    __IO uint32_t    mem_2_3_4_dvs;          //0x50
    __IO uint32_t    efuse_cfg;          //0x54
    __IO uint32_t    chip_state;          //0x58
    __IO uint32_t    cal_rw;          //0x58
    __IO uint32_t    cal_ro0;          //0x5c
    __IO uint32_t    cal_ro1;          //0x60
    __IO uint32_t    cal_ro2;          //0x64
    __IO uint32_t    ADC_CTL0;          //0x68
    __IO uint32_t    ADC_CTL1;          //0x6c
    __IO uint32_t    ADC_CTL2;          //0x70
    __IO uint32_t    ADC_CTL3;          //0x74
    __IO uint32_t    ADC_CTL4;          //0x7c
} AP_PCRM_TypeDef;

typedef struct {
    __IO uint32_t    enable;          //0
    __IO uint32_t    reserve[2];          //4~8
    __IO uint32_t    control_1;          //c
    __IO uint32_t    control_2;          //10
    __IO uint32_t    control_3;          //14
    __IO uint32_t    control_4;         //18
    __IO uint32_t    reserve1[6];         //1c~30
    __IO uint32_t    intr_mask;          //34
    __IO uint32_t    intr_clear;         //38
    __IO uint32_t    intr_status;        //3c
} AP_ADCC_TypeDef;


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif


/******************************************************************************/
/*                         Peripheral memory map(AP)                          */
/******************************************************************************/
/* Base addresses                                                             */

#define AP_APB0_BASE         (0x40000000UL)
#define AP_APB1_BASE         (0x40020000UL)
#define AP_APB2_BASE         (0x4000F000UL)


/* APB0 peripherals   */
#define AP_PCR_BASE          (AP_APB0_BASE + 0x0000)
#define AP_TIM1_BASE         (AP_APB0_BASE + 0x1000)
#define AP_TIM2_BASE         (AP_APB0_BASE + 0x1014)
#define AP_TIM3_BASE         (AP_APB0_BASE + 0x1028)
#define AP_TIM4_BASE         (AP_APB0_BASE + 0x103c)
#define AP_TIM_SYS_BASE      (AP_APB0_BASE + 0x10a0)

#define AP_WDT_BASE          (AP_APB0_BASE + 0x2000)
#define AP_COM_BASE          (AP_APB0_BASE + 0x3000)
#define AP_IOMUX_BASE        (AP_APB0_BASE + 0x3800)
#define AP_UART0_BASE        (AP_APB0_BASE + 0x4000)
#define AP_I2C0_BASE         (AP_APB0_BASE + 0x5000)
#define AP_I2C1_BASE         (AP_APB0_BASE + 0x5800)
#define AP_SPI0_BASE         (AP_APB0_BASE + 0x6000)
#define AP_SPI1_BASE         (AP_APB0_BASE + 0x7000)
#define AP_GPIOA_BASE         (AP_APB0_BASE + 0x8000)
#define AP_GPIOB_BASE         (AP_APB0_BASE + 0x800C)
#define AP_I2S_BASE           (AP_APB0_BASE + 0x9000)
#define AP_DMIC_BASE          (AP_APB0_BASE + 0xA000)
#define AP_QDEC_BASE          (AP_APB0_BASE + 0xB000)
#define AP_RNG_BASE           (AP_APB0_BASE + 0xC000)
#define AP_PWM_BASE           (AP_APB0_BASE + 0xE000)
#define AP_AON_BASE           (AP_APB0_BASE + 0xF000)
#define AP_RTC_BASE           (AP_APB0_BASE + 0xF024)
#define AP_PCRM_BASE          (AP_APB0_BASE + 0xF03c)
#define AP_WAKEUP_BASE        (AP_APB0_BASE + 0xF0a0)



#define SRAM0_BASE_ADDRESS     0x1FFF0000
#define SRAM1_BASE_ADDRESS     0x1FFF8000
#define SRAM2_BASE_ADDRESS     0x20000000
#define SRAM3_BASE_ADDRESS     0x20010000
#define SRAM4_BASE_ADDRESS     0x20012000

#define FLASH_BASE_ADDR        0x11000000

#define PCR_BASE_ADDR          0x40000000
#define PCRM_BASE_ADDR         0x4000F000
#define IOMUX_BASE_ADDR        0x40003800
#define AON_BASE_ADDR          0x4000F000
#define COM_BASE_ADDR          0x40003000
#define AES_BASE_ADDR          0x40040000
#define ADCC_BASE_ADDR         0x40050000
#define BBTOP_BASE_ADDR        0x40030000
#define LL_BASE_ADDR           0x40031000
#define PWM_BASE_ADDR          0x400E0000
#define KSCAN_BASE_ADDR        0x40024000
#define QDEC_BASE_ADDR         0x4000B000
#define RNG_BASE_ADDR          0x4000C000

/////////////////////////////////////////////////////////////

#define AP_PCR               ((AP_PCR_TypeDef *) AP_PCR_BASE )

#define AP_WDT               ((AP_WDT_TypeDef *) AP_WDT_BASE )

#define AP_COM               ((AP_COM_TypeDef *) AP_COM_BASE)

#define AP_TIM1              ((AP_TIM_TypeDef *) AP_TIM1_BASE )
#define AP_TIM2              ((AP_TIM_TypeDef *) AP_TIM2_BASE )
#define AP_TIM3              ((AP_TIM_TypeDef *) AP_TIM3_BASE )
#define AP_TIM4              ((AP_TIM_TypeDef *) AP_TIM4_BASE )
#define AP_TIMS              ((AP_TIM_SYS_TypeDef *) AP_TIM_SYS_BASE )

#define AP_IOMUX             ((IOMUX_TypeDef *) AP_IOMUX_BASE)

#define AP_SPI0              ((AP_SSI_TypeDef *) AP_SPI0_BASE)
#define AP_SPI1              ((AP_SSI_TypeDef *) AP_SPI1_BASE)

#define AP_UART0             ((AP_UART_TypeDef *) AP_UART0_BASE)

#define AP_I2C0              ((AP_I2C_TypeDef  *) AP_I2C0_BASE)
#define AP_I2C1              ((AP_I2C_TypeDef  *) AP_I2C1_BASE)

#define AP_I2S_BLOCK         ((AP_I2S_BLOCK_TypeDef  *) AP_I2S_BASE)

#define AP_I2S0              ((AP_I2S_TypeDef  *) (AP_I2S_BASE+0x20))


#define AP_RTC                ((AP_RTC_TypeDef  *) AP_RTC_BASE)


#define AP_GPIOA              ((AP_GPIO_TypeDef *) AP_GPIOA_BASE )

#define AP_AON                ((AP_AON_TypeDef  *) AP_AON_BASE)
#define AP_PCRM               ((AP_PCRM_TypeDef  *) AP_PCRM_BASE)


#define AP_WAKEUP             ((AP_Wakeup_TypeDef*) AP_WAKEUP_BASE)

#define AP_ADCC               ((AP_ADCC_TypeDef  *) ADCC_BASE_ADDR)

#define I2S_COMP_VER          (unsigned int *)(AP_I2S_BASE+0x1f8)
#define I2S_COMP_TYPE         (unsigned int *)(AP_I2S_BASE+0x1fc)
#define I2S_COM_PARA1         (unsigned int *)(AP_I2S_BASE+0x1f4)
#define I2S_COM_PARA2         (unsigned int *)(AP_I2S_BASE+0x1f0)



#define AP_STATUS1_REG          (unsigned int *) 0x40003010
#define AP_STATUS2_REG          (unsigned int *) 0x40003014
#define AP_EVENT_REC_REG        (unsigned int *) 0x40003024
#define AP_STCALIB              (unsigned int *) 0x40003028
#define AP_PERI_MASTER_SELECT   *(volatile unsigned int  *)0x4000302C



/******************************************************************************/
/*                         Peripheral memory map(CP)                          */
/******************************************************************************/
/* Base addresses                                                             */


#define CP_PCR_BASE          (0x40020000UL)           // change from 0x40020000UL to 0x40000000UL by HZF
#define CP_TIM1_BASE         (0x40021000UL)
#define CP_TIM2_BASE         (0x40021014UL)
#define CP_TIM3_BASE         (0x40021028UL)
#define CP_TIM4_BASE         (0x4002103cUL)
#define CP_TIM_SYS_BASE      (0x400210a0UL)
#define CP_WDT_BASE          (0x40022000UL)
#define CP_COM_BASE          (0x40023000UL)
#define CP_BASEBAND_BASE     (0x40030000UL)

#define CP_PCR               ((AP_PCR_TypeDef *) CP_PCR_BASE )
#define CP_WDT               ((AP_WDT_TypeDef *) CP_WDT_BASE )
#define CP_TIM1              ((AP_TIM_TypeDef *) CP_TIM1_BASE )
#define CP_TIM2              ((AP_TIM_TypeDef *) CP_TIM2_BASE )
#define CP_TIM3              ((AP_TIM_TypeDef *) CP_TIM3_BASE )
#define CP_TIM4              ((AP_TIM_TypeDef *) CP_TIM4_BASE )
#define CP_TIMS              ((AP_TIM_SYS_TypeDef *) CP_TIM_SYS_BASE )
#define CP_COM               ((AP_COM_TypeDef *) CP_COM_BASE)


#define CP_STATUS1_REG            (unsigned int *) 0x40023010
#define CP_STATUS2_REG            (unsigned int *) 0x40023014
#define CP_EVENT_REC_REG          (unsigned int *) 0x40023024
#define CP_STCALIB                (unsigned int *) 0x40023028
#define CP_PERI_MASTER_SELECT     (unsigned int *) 0x4002302C


#define PM_BASE  0x4000F000


//////////////////////////////////////////////////////////


#define M0_ONLY                1
#define M4_ONLY                2
#define COMBO                  3





#define IRQ_PRIO_REALTIME     0
#define IRQ_PRIO_HIGH         1
#define IRQ_PRIO_HAL          2
#define IRQ_PRIO_THREAD       3
#define IRQ_PRIO_APP          3


enum {
    APCOM_CP_IPC_IRQ  = 0,
    CPCOM_CP_IPC_IRQ  = 1,
    CP_TIMER_IRQ      = 2,
    CP_WDT_IRQ        = 3,
    BB_IRQ            = 4,
    KSCAN_IRQ         = 5,
    RTC_IRQ           = 6,
    CPCOM_AP_IPC_IRQ  = 7,
    APCOM_AP_IPC_IRQ  = 8,
    TIMER_IRQ         = 9,
    WDT_IRQ           = 10,
    UART_IRQ          = 11,
    I2C0_IRQ          = 12,
    I2C1_IRQ          = 13,
    SPI0_IRQ          = 14,
    SPI1_IRQ          = 15,
    GPIO_IRQ          = 16,
    I2S_IRQ           = 17,
    SPIF_IRQ          = 18,
    DMAC_INTR_IRQ     = 19,
    DMAC_INTTC_IRQ    = 20,
    DMAC_INTERR_IRQ   = 21,
    FPIDC_IRQ         = 22,
    FPDZC_IRQ         = 23,
    FPIOC_IRQ         = 24,
    FPUFC_IRQ         = 25,
    FPOFC_IRQ         = 26,
    FPIXC_IRQ         = 27,
    AES_IRQ           = 28,
    ADCC_IRQ          = 29,
    QDEC_IRQ          = 30,
    RNG_IRQ           = 31
};


#endif






