/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     dw_spi.c
 * @brief    CSI Source File for SPI Driver
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/
#include <csi_config.h>
#include <string.h>
#include <drv_irq.h>
#include <drv_spi.h>
#include <dw_spi.h>
#include <drv_dmac.h>
#include <soc.h>   /*CONFIG_SPI_NUM*/
#include <stdio.h>
#include "clock.h"

#define ERR_SPI(errno) (CSI_DRV_ERRNO_SPI_BASE | errno)
#define SPI_BUSY_TIMEOUT    0x1000000
#define SPI_NULL_PARAM_CHK(para)  HANDLE_PARAM_CHK(para, ERR_SPI(DRV_ERROR_PARAMETER))

typedef struct {
#ifdef CONFIG_LPM
    uint8_t spi_power_status;
    uint32_t spi_regs_saved[12];
#endif
    uint32_t base;
    uint32_t irq;
    int32_t ssel;
    spi_event_cb_t cb_event;
    uint32_t send_num;
    uint32_t recv_num;
    uint8_t *send_buf;
    uint8_t *recv_buf;
    uint8_t enable_slave;
    uint32_t transfer_num;
    uint32_t clk_num;            //clock number with a process of communication
    uint8_t state;               //Current SPI state
    uint32_t mode;               //Current SPI mode
    uint8_t ss_mode;
    spi_status_t status;
    int32_t block_mode;
    int32_t idx;

#define TRANSFER_STAT_IDLE      0
#define TRANSFER_STAT_SEND      1
#define TRANSFER_STAT_RCV       2
#define TRANSFER_STAT_TRAN      3

    uint8_t  transfer_stat;     //TRANSFER_STAT_* : 0 - idle, 1 - send , 2 -receive , 3 - transceive
    uint32_t tot_num;
} dw_spi_priv_t;
#define SPI_DMA_BLOCK_SIZE  1024
extern int32_t target_spi_init(int32_t idx, uint32_t *base, uint32_t *irq, void **handler, uint32_t *ssel);
static int32_t dw_spi_set_mode(spi_handle_t handle, DWENUM_SPI_MODE mode);
extern int32_t target_get_addr_space(uint32_t addr);

static dw_spi_priv_t spi_instance[CONFIG_SPI_NUM];

static const spi_capabilities_t spi_capabilities = {
    .simplex = 1,           /* Simplex Mode (Master and Slave) */
    .ti_ssi = 1,            /* TI Synchronous Serial Interface */
    .microwire = 1,         /* Microwire Interface */
    .event_mode_fault = 0   /* Signal Mode Fault event: \ref CSKY_SPI_EVENT_MODE_FAULT */
};

/**
  \brief       Set the SPI datawidth.
  \param[in]   handle     spi handle
  \param[in]   datawidth  date frame size in bits
  \return      error code
*/
int32_t csi_spi_config_datawidth(spi_handle_t handle, uint32_t datawidth)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    addr->SPIENR    = DW_SPI_DISABLE;
    addr->IMR       = DW_SPI_INT_DISABLE;

    if ((datawidth >= (DWENUM_SPI_DATASIZE_4 + 1)) && (datawidth <= (DWENUM_SPI_DATASIZE_16  + 1))) {
        uint16_t temp = addr->CTRLR0;
        temp &= 0xfff0;         /* temp has the value of CTRLR0 with DFS being cleared.*/
        temp |= (datawidth - 1);    /* get the final CTRLR0 after datawidth config. */
        addr->CTRLR0 = temp;    /* write CTRLR0 */

        spi_priv->state |= SPI_CONFIGURED;
        return 0;
    }

    return ERR_SPI(SPI_ERROR_DATA_BITS);
}

/**
  \brief       Set the SPI clock divider.
  \param[in]   handle   spi handle
  \param[in]   baud     spi baud rate
  \return      error code
*/
int32_t csi_spi_config_baudrate(spi_handle_t handle, uint32_t baud)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    addr->SPIENR    = DW_SPI_DISABLE;
    addr->IMR       = DW_SPI_INT_DISABLE;

    int32_t sckdv = drv_get_apb_freq() / baud;

    if (sckdv < 0x10000) {
        addr->BAUDR =  sckdv;
    } else {
        return ERR_SPI(DRV_ERROR_PARAMETER);
    }

    spi_priv->state |= SPI_CONFIGURED;
    return 0;
}

/**
  \brief       Set the SPI polarity.
  \param[in]   addr  pointer to register address
  \param[in]   polarity spi polarity
  \return      error code
*/
static int32_t dw_spi_set_polarity(dw_spi_reg_t *addr, DWENUM_SPI_POLARITY polarity)
{
    /* To config the polarity, we can set the SCPOL bit(CTRLR0[7]) as below:
     *     0 - inactive state of serial clock is low
     *     1 - inactive state of serial clock is high
     */
    switch (polarity) {
        case DWENUM_SPI_CLOCK_POLARITY_LOW:
            addr->CTRLR0 &= (~DW_SPI_POLARITY);
            break;

        case DWENUM_SPI_CLOCK_POLARITY_HIGH:
            addr->CTRLR0 |= DW_SPI_POLARITY;
            break;

        default:
            return -1;
    }

    return 0;
}

/**
  \brief       Set the SPI Phase.
  \param[in]   addr  pointer to register address
  \param[in]   phase    Serial clock phase
  \return      error code
*/
static int32_t dw_spi_set_phase(dw_spi_reg_t *addr, DWENUM_SPI_PHASE phase)
{
    switch (phase) {
        case DWENUM_SPI_CLOCK_PHASE_MIDDLE:
            addr->CTRLR0 &= (~DW_SPI_PHASE);
            break;

        case DWENUM_SPI_CLOCK_PHASE_START:
            addr->CTRLR0 |= DW_SPI_PHASE;
            break;

        default:
            return -1;
    }

    return 0;
}

/**
  \brief       config the SPI format.
  \param[in]   handle   spi handle
  \param[in]   format   spi format. \ref spi_format_e
  \return      error code
*/
int32_t csi_spi_config_format(spi_handle_t handle, spi_format_e format)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    addr->SPIENR    = DW_SPI_DISABLE;
    addr->IMR       = DW_SPI_INT_DISABLE;

    switch (format) {
        case SPI_FORMAT_CPOL0_CPHA0:
            dw_spi_set_polarity(addr, DWENUM_SPI_CLOCK_POLARITY_LOW);
            dw_spi_set_phase(addr, DWENUM_SPI_CLOCK_PHASE_MIDDLE);
            break;

        case SPI_FORMAT_CPOL0_CPHA1:
            dw_spi_set_polarity(addr, DWENUM_SPI_CLOCK_POLARITY_LOW);
            dw_spi_set_phase(addr, DWENUM_SPI_CLOCK_PHASE_START);
            break;

        case SPI_FORMAT_CPOL1_CPHA0:
            dw_spi_set_polarity(addr, DWENUM_SPI_CLOCK_POLARITY_HIGH);
            dw_spi_set_phase(addr, DWENUM_SPI_CLOCK_PHASE_MIDDLE);
            break;

        case SPI_FORMAT_CPOL1_CPHA1:
            dw_spi_set_polarity(addr, DWENUM_SPI_CLOCK_POLARITY_HIGH);
            dw_spi_set_phase(addr, DWENUM_SPI_CLOCK_PHASE_START);
            break;

        default:
            return ERR_SPI(SPI_ERROR_FRAME_FORMAT);
    }

    spi_priv->state |= SPI_CONFIGURED;

    return 0;
}

/**
  \brief       config the SPI mode.
  \param[in]   handle   spi handle
  \param[in]   mode     spi mode. \ref spi_mode_e
  \return      error code
*/
int32_t csi_spi_config_mode(spi_handle_t handle, spi_mode_e  mode)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    addr->SPIENR    = DW_SPI_DISABLE;
    addr->IMR       = DW_SPI_INT_DISABLE;

    //AP_PERI_MASTER_SELECT |= (BIT(shift)|BIT(shift+4));

    if (mode == SPI_MODE_MASTER) {
        if (spi_priv->idx == 0) {
            AP_PERI_MASTER_SELECT |= SPI0_MASTER_MODE | (1 << 4);
        } else {
            AP_PERI_MASTER_SELECT |= SPI1_MASTER_MODE | (1 << 5);
        }

        spi_priv->state |= SPI_CONFIGURED;
        return 0;
    }

    if (mode == SPI_MODE_SLAVE) {
        if (spi_priv->idx == 0) {
            AP_PERI_MASTER_SELECT &= ~SPI0_MASTER_MODE;
        } else {
            AP_PERI_MASTER_SELECT &= ~SPI1_MASTER_MODE;
        }

        spi_priv->state |= SPI_CONFIGURED;
        return 0;
    }

    if (mode == SPI_MODE_SLAVE_SIMPLEX || mode == SPI_MODE_MASTER_SIMPLEX) {
        return ERR_SPI(DRV_ERROR_UNSUPPORTED);
    }

    return ERR_SPI(SPI_ERROR_MODE);
}
/**
  \brief       config the SPI block mode.
  \param[in]   handle   spi handle
  \param[in]   flag 1 - enbale the block mode. 0 - disable the block mode
  \return      error code
*/
int32_t csi_spi_config_block_mode(spi_handle_t handle, int32_t flag)
{
    SPI_NULL_PARAM_CHK(handle);
    dw_spi_priv_t *spi_priv = handle;

    if (flag == 1) {
        spi_priv->block_mode = 1;
    } else if (flag == 0) {
        spi_priv->block_mode = 0;
    } else {
        return ERR_SPI(DRV_ERROR_PARAMETER);
    }

    return 0;
}

/**
  \brief       config the SPI mode.
  \param[in]   handle   spi handle
  \param[in]   order    spi bit order.\ref spi_bit_order_e
  \return      error code
*/
int32_t csi_spi_config_bit_order(spi_handle_t handle, spi_bit_order_e order)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    if (order == SPI_ORDER_MSB2LSB) {
        spi_priv->state |= SPI_CONFIGURED;
        return 0;
    }

    return ERR_SPI(SPI_ERROR_BIT_ORDER);;
}

/**
  \brief       config the SPI slave select mode.
  \param[in]   handle   spi handle
  \param[in]   ss_mode  spi slave select mode. \ref spi_ss_mode_e
  \return      error code
*/
int32_t csi_spi_config_ss_mode(spi_handle_t handle, spi_ss_mode_e ss_mode)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    switch (ss_mode) {
        case SPI_SS_MASTER_SW:
            spi_priv->ss_mode = SPI_SS_MASTER_SW;
            break;

        case SPI_SS_MASTER_HW_OUTPUT:
            spi_priv->ss_mode = SPI_SS_MASTER_HW_OUTPUT;
            break;

        case SPI_SS_MASTER_HW_INPUT:
            spi_priv->ss_mode = SPI_SS_MASTER_HW_INPUT;
            break;

        case SPI_SS_SLAVE_HW:
            spi_priv->ss_mode = SPI_SS_SLAVE_HW;
            break;

        case SPI_SS_SLAVE_SW:
            spi_priv->ss_mode = SPI_SS_SLAVE_SW;
            break;

        default:
            return ERR_SPI(SPI_ERROR_SS_MODE);
    }

    spi_priv->state |= SPI_CONFIGURED;

    return 0;
}

/**
  \brief       Set the SPI mode.
  \param[in]   addr  pointer to register address
  \param[in]   mode     SPI_Mode
  \return      error code
*/
static int32_t dw_spi_set_mode(spi_handle_t handle, DWENUM_SPI_MODE mode)
{
    dw_spi_priv_t *spi_priv = handle;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    /* It is impossible to write to this register when the SSI is enabled.*/
    /* we can set the TMOD to config transfer mode as below:
     *     TMOD_BIT9  TMOD_BIT8      transfer mode
     *         0          0         transmit & receive
     *         0          1           transmit only
     *         1          0           receive only
     *         1          1             reserved
     */
    switch (mode) {
        case DWENUM_SPI_TXRX:
            addr->CTRLR0 &= (~DW_SPI_TMOD_BIT8);
            addr->CTRLR0 &= (~DW_SPI_TMOD_BIT9);
            break;

        case DWENUM_SPI_TX:
            addr->CTRLR0 |= DW_SPI_TMOD_BIT8;
            addr->CTRLR0 &= (~DW_SPI_TMOD_BIT9);
            break;

        case DWENUM_SPI_RX:
            addr->CTRLR0 &= (~DW_SPI_TMOD_BIT8);
            addr->CTRLR0 |= DW_SPI_TMOD_BIT9;
            break;

        default:
            addr->CTRLR0 |= DW_SPI_TMOD_BIT8;
            addr->CTRLR0 |= DW_SPI_TMOD_BIT9;
            break;
    }

    spi_priv->mode = mode;
    return 0;
}

/**
  \brief       interrupt service function for receive FIFO full interrupt .
  \param[in]   spi_priv pointer to spi private.
*/
static void dw_spi_intr_rx_full(int32_t idx, dw_spi_priv_t *spi_priv)
{
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    uint8_t temp = addr->ICR;
    uint8_t *pbuffer = spi_priv->recv_buf;
    uint32_t length = spi_priv->recv_num;


    uint8_t rxnum;
    rxnum = addr->RXFLR;
    uint32_t i = 0u;

    if (rxnum > length) {
        rxnum = length;
    }

    for (i = 0; i < rxnum; i++) {
        *pbuffer = addr->DR;
        pbuffer++;
    }

    length -= rxnum;

    if (length < DW_SPI_FIFO_MAX_LV) {
        addr->RXFTLR = length - 1;
    }

    if (length <= 0) {
        temp = addr->IMR;
        temp &= 0x2f;
        addr->IMR = temp;

        addr->SER = 0;
        addr->SPIENR = DW_SPI_DISABLE;
        spi_priv->status.busy = 0U;

        spi_priv->recv_num = 0;

        if (spi_priv->cb_event) {
            spi_priv->cb_event(idx, SPI_EVENT_RX_COMPLETE);
            return;
        }
    } else {
        if ((idx == 0 && !SPI0_IS_SLAVE_MODE) || (idx == 1 && !SPI1_IS_SLAVE_MODE)) {
            addr->SPIENR = DW_SPI_DISABLE;   /* disable SPI */
        }

        dw_spi_set_mode(spi_priv, DWENUM_SPI_RX);
        addr->SER = spi_priv->enable_slave; /* enable all cs */
        spi_priv->recv_buf = pbuffer;
        spi_priv->recv_num = length;

        if (spi_priv->recv_num > DW_SPI_FIFO_MAX_LV) {
            addr->RXFTLR = DW_SPI_FIFO_MAX_LV - 1;
            addr->CTRLR1 = DW_SPI_FIFO_MAX_LV - 1;
        } else {
            addr->RXFTLR = spi_priv->recv_num - 1;
            addr->CTRLR1 = spi_priv->recv_num - 1;
        }

        if ((idx == 0 && !SPI0_IS_SLAVE_MODE) || (idx == 1 && !SPI1_IS_SLAVE_MODE)) {
            addr->SPIENR = DW_SPI_DISABLE;   /* disable SPI */
        }

        addr->DR = DW_SPI_START_RX ;
    }

}

/**
  \brief       interrupt service function for transmit FIFO empty interrupt.
  \param[in]   spi_priv pointer to spi private.
*/
static void dw_spi_intr_tx_empty(int32_t idx, dw_spi_priv_t *spi_priv)
{
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);
    uint8_t temp = addr->ICR;

    /* transfer mode:transmit & receive */
    uint32_t i = 0u;

    if (spi_priv->
        mode == DWENUM_SPI_TXRX) {
        /* read data out from rx FIFO */
        while (spi_priv->transfer_num) {
            if (!(addr->RXFLR)) {
                return;
            }

            *spi_priv->recv_buf = addr->DR;
            spi_priv->recv_buf++;
            spi_priv->transfer_num--;
        }

        if (spi_priv->clk_num >= DW_SPI_FIFO_MAX_LV) {
            spi_priv->transfer_num = DW_SPI_FIFO_MAX_LV;
        } else {
            spi_priv->transfer_num = spi_priv->clk_num;
        }

        for (i = 0; i < spi_priv->transfer_num; i++) {
            if (spi_priv->send_num == 0) {
                addr->DR = 0x0;
            } else {
                addr->DR = *spi_priv->send_buf;
                spi_priv->send_buf++;
            }

            spi_priv->send_num--;
        }
    } else {    //transfer mode :transmit only
        if (spi_priv->clk_num >= DW_SPI_FIFO_MAX_LV) {
            spi_priv->transfer_num = DW_SPI_FIFO_MAX_LV;
        } else {
            spi_priv->transfer_num = spi_priv->clk_num;
        }

        for (i = 0; i < spi_priv->transfer_num; i++) {
            addr->DR = *spi_priv->send_buf;
            spi_priv->send_buf++;
            spi_priv->send_num--;
        }

    }

    if (spi_priv->clk_num == 0) {
        if (addr->SR & DW_SPI_BUSY) {
            return;
        }

        temp = addr->IMR;
        temp &= ~DW_SPI_IMR_TXEIM;
        addr->IMR = temp;

        addr->SER = 0;
        addr->SPIENR = DW_SPI_DISABLE;
        spi_priv->status.busy = 0U;

        spi_priv->send_num = 0;

        if (spi_priv->mode == DWENUM_SPI_TXRX) {
            if (spi_priv->cb_event) {
                spi_priv->cb_event(idx, SPI_EVENT_TRANSFER_COMPLETE);
                return;
            }
        } else {
            if (spi_priv->cb_event) {
                spi_priv->cb_event(idx, SPI_EVENT_TX_COMPLETE);
                return;
            }
        }
    }

    spi_priv->clk_num -= spi_priv->transfer_num;

}
/**
  \brief       handler the interrupt.
  \param[in]   spi      Pointer to \ref SPI_RESOURCES
*/
void dw_spi_irqhandler(int32_t idx)
{
    dw_spi_priv_t *spi_priv = &spi_instance[idx];
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    uint32_t intr = addr->ISR;

    /* deal with receive FIFO full interrupt */
    if (intr & DW_SPI_RXFIFO_FULL) {
        dw_spi_intr_rx_full(idx, spi_priv);
    }
    /* deal with transmit FIFO empty interrupt */
    else if (intr & DW_SPI_TXFIFO_EMPTY) {
        dw_spi_intr_tx_empty(idx, spi_priv);
    }
    /* deal with transmit FIFO overflow interrupt */
    else if (intr & (DW_SPI_RXFIFO_OVERFLOW | DW_SPI_TXFIFO_OVERFLOW)) {
        dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);
        /*clear interrupt*/
        addr->ICR;
        /*clear overflow*/
        addr->RXOICR;
        addr->TXOICR;
        spi_priv->cb_event(idx, SPI_EVENT_DATA_LOST);
    }
}

#ifdef CONFIG_LPM
static void manage_clock(spi_handle_t handle, uint8_t enable)
{
    dw_spi_priv_t *spi_priv = handle;
    uint8_t device[] = {CLOCK_MANAGER_SPI0, CLOCK_MANAGER_SPI1};

    drv_clock_manager_config(device[spi_priv->idx], enable);
}

static void do_prepare_sleep_action(spi_handle_t handle)
{
    dw_spi_priv_t *spi_priv = handle;
    uint32_t *sbase = (uint32_t *)(spi_priv->base);
    registers_save(spi_priv->spi_regs_saved, sbase, 7);
    registers_save(&spi_priv->spi_regs_saved[7], sbase + 11, 1);
    registers_save(&spi_priv->spi_regs_saved[8], sbase + 19, 3);
    registers_save(&spi_priv->spi_regs_saved[11], sbase + 40, 1);
}

static void do_wakeup_sleep_action(spi_handle_t handle)
{
    dw_spi_priv_t *spi_priv = handle;
    uint32_t *sbase = (uint32_t *)(spi_priv->base);
    registers_restore(sbase, spi_priv->spi_regs_saved, 2);
    registers_restore(sbase + 3, &spi_priv->spi_regs_saved[3], 4);
    registers_restore(sbase + 11, &spi_priv->spi_regs_saved[7], 1);
    registers_restore(sbase + 19, &spi_priv->spi_regs_saved[8], 3);
    registers_restore(sbase + 40, &spi_priv->spi_regs_saved[11], 1);
    registers_restore(sbase + 2, &spi_priv->spi_regs_saved[2], 1);
}
#endif

/**
  \brief       Initialize SPI Interface. 1. Initializes the resources needed for the SPI interface 2.registers event callback function
  \param[in]   idx spi index
  \param[in]   cb_event  event call back function \ref spi_event_cb_t
  \return      return spi handle if success
*/
spi_handle_t csi_spi_initialize(int32_t idx, spi_event_cb_t cb_event)
{
    uint32_t base = 0u;
    uint32_t irq = 0u;
    void *handler;
    uint32_t ssel = 0u;

    if (idx == 0) {
        clk_gate_enable(MOD_SPI0);
    } else {
        clk_gate_enable(MOD_SPI1);
    }

    int32_t ret = target_spi_init(idx, &base, &irq, &handler, &ssel);

    if (ret < 0 || ret >= CONFIG_SPI_NUM) {
        return NULL;
    }

    dw_spi_priv_t *spi_priv = &spi_instance[idx];

    spi_priv->base = base;
    spi_priv->irq  = irq;
    spi_priv->ssel = ssel;

    spi_priv->cb_event          = cb_event;
    spi_priv->status.busy       = 0U;
    spi_priv->status.data_lost  = 0U;
    spi_priv->status.mode_fault = 0U;
    spi_priv->enable_slave      = 1U;
    spi_priv->state             = SPI_INITIALIZED;

    spi_priv->idx  = idx;

    drv_irq_register(spi_priv->irq, handler);
    drv_irq_enable(spi_priv->irq);
#ifdef CONFIG_LPM
    csi_spi_power_control(spi_priv, DRV_POWER_FULL);
#endif

    return (spi_handle_t)spi_priv;
}

/**
  \brief       De-initialize SPI Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle spi handle to operate.
  \return      error code
*/
int32_t csi_spi_uninitialize(spi_handle_t handle)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    drv_irq_disable(spi_priv->irq);
    drv_irq_unregister(spi_priv->irq);

    spi_priv->cb_event          = NULL;
    spi_priv->state             = 0U;
    spi_priv->status.busy       = 0U;
    spi_priv->status.data_lost  = 0U;
    spi_priv->status.mode_fault = 0U;

#ifdef CONFIG_LPM
    csi_spi_power_control(spi_priv, DRV_POWER_OFF);
#endif
    return 0;
}

/**
  \brief       control spi power.
  \param[in]   idx     spi index.
  \param[in]   state   power state.\ref csi_power_stat_e.
  \return      error code
*/
int32_t csi_spi_power_control(spi_handle_t handle, csi_power_stat_e state)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    MODULE_e mod = MOD_SPI0 + spi_priv->idx;

    if (state == DRV_POWER_FULL) {
        clk_gate_enable(mod);
        return 0;
    } else if (state == DRV_POWER_OFF) {
        clk_gate_disable(mod);
        return 0;
    } else {
#ifdef CONFIG_LPM
        power_cb_t callback = {
            .wakeup = do_wakeup_sleep_action,
            .sleep = do_prepare_sleep_action,
            .manage_clock = manage_clock
        };
        return drv_soc_power_control(handle, state, &callback);
#else
        return ERR_SPI(DRV_ERROR_UNSUPPORTED);
#endif
    }
}

/**
  \brief       Get driver capabilities.
  \param[in]   idx spi index.
  \return      \ref spi_capabilities_t
*/
spi_capabilities_t csi_spi_get_capabilities(int32_t idx)
{
    if (idx < 0 || idx >= CONFIG_SPI_NUM) {
        spi_capabilities_t ret;
        memset(&ret, 0, sizeof(spi_capabilities_t));
        return ret;
    }

    return spi_capabilities;
}

/**
  \brief       config spi mode.
  \param[in]   handle spi handle to operate.
  \param[in]   baud      spi baud rate. if negative, then this attribute not changed
  \param[in]   mode      \ref spi_mode_e . if negative, then this attribute not changed
  \param[in]   format    \ref spi_format_e . if negative, then this attribute not changed
  \param[in]   order     \ref spi_bit_order_e . if negative, then this attribute not changed
  \param[in]   ss_mode   \ref spi_ss_mode_e . if negative, then this attribute not changed
  \param[in]   bit_width spi data bitwidth: (1 ~ SPI_DATAWIDTH_MAX) . if negative, then this attribute not changed
  \return      error code
*/
int32_t csi_spi_config(spi_handle_t handle,
                       int32_t          baud,
                       spi_mode_e       mode,
                       spi_format_e     format,
                       spi_bit_order_e  order,
                       spi_ss_mode_e    ss_mode,
                       int32_t          bit_width)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;

    if ((spi_priv->state & SPI_INITIALIZED) == 0U) {
        return ERR_SPI(DRV_ERROR_UNSUPPORTED);
    }

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    spi_priv->status.busy       = 0U;
    spi_priv->status.data_lost  = 0U;
    spi_priv->status.mode_fault = 0U;

    int32_t ret = 0;

    if (mode >= 0) {
        ret = csi_spi_config_mode(handle, mode);

        if (ret < 0) {
            return ret;
        }
    }

    if (baud >= 0) {
        ret = csi_spi_config_baudrate(handle, baud);

        if (ret < 0) {
            return ret;
        }
    }

    if (format >= 0) {
        ret = csi_spi_config_format(handle, format);

        if (ret < 0) {
            return ret;
        }
    }

    if (order >= 0) {
        ret = csi_spi_config_bit_order(handle, order);

        if (ret < 0) {
            return ret;
        }
    }

    if (ss_mode >= 0) {
        ret = csi_spi_config_ss_mode(handle, ss_mode);

        if (ret < 0) {
            return ret;
        }
    }

    if (bit_width >= 0) {
        ret = csi_spi_config_datawidth(handle, bit_width);

        if (ret < 0) {
            return ret;
        }
    }

    spi_priv->state |= SPI_CONFIGURED;

    return 0;
}


/**
  \brief       sending data to SPI transmitter,(received data is ignored).
               if non-blocking mode, this function only start the sending,
               \ref spi_event_e is signaled when operation completes or error happens.
               \ref csi_spi_get_status can indicates operation status.
               if blocking mode, this function return after operation completes or error happens.
  \param[in]   handle spi handle to operate.
  \param[in]   data  Pointer to buffer with data to send to SPI transmitter. data_type is : uint8_t for 1..8 data bits, uint16_t for 9..16 data bits,uint32_t for 17..32 data bits,
  \param[in]   num   Number of data items to send.
  \return      error code
*/
int32_t csi_spi_send(spi_handle_t handle, const void *data, uint32_t num)

{
    if (handle == NULL || data == NULL || num == 0) {
        return ERR_SPI(DRV_ERROR_PARAMETER);
    }

    dw_spi_priv_t *spi_priv = handle;

    if ((spi_priv->state & SPI_CONFIGURED) == 0U) {
        return ERR_SPI(DRV_ERROR_UNSUPPORTED);
    }

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    spi_priv->status.busy       = 1U;
    spi_priv->status.data_lost  = 0U;
    spi_priv->status.mode_fault = 0U;
    spi_priv->tot_num           = num;
    spi_priv->transfer_stat     = TRANSFER_STAT_SEND;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);
    addr->SPIENR    = DW_SPI_DISABLE; /* disable SPI */
    dw_spi_set_mode(spi_priv, DWENUM_SPI_TX);
    uint8_t *ptxbuffer = (uint8_t *)data;

    if (spi_priv->block_mode) {
        addr->SPIENR = DW_SPI_ENABLE; /* enable SPI */
        addr->SER = spi_priv->enable_slave;
        addr->TXFTLR            = DW_SPI_TXFIFO_LV;

        spi_priv->send_num      = num;

        uint32_t once_len, i;

        while (spi_priv->send_num) {
            once_len = (spi_priv->send_num >= DW_SPI_FIFO_MAX_LV) ?
                       DW_SPI_FIFO_MAX_LV : spi_priv->send_num;

            for (i = 0; i < once_len; i++) {
                addr->DR = *ptxbuffer++;
            }

            uint32_t timecount = 0;

            while (!(addr->SR & DW_SPI_TFE)) {
                timecount++;

                if (timecount >= SPI_BUSY_TIMEOUT) {
                    return ERR_SPI(DRV_ERROR_TIMEOUT);
                }

            }

            timecount = 0;

            while (addr->SR & DW_SPI_BUSY) {
                timecount++;

                if (timecount >= SPI_BUSY_TIMEOUT) {
                    return ERR_SPI(DRV_ERROR_TIMEOUT);
                }
            }

            spi_priv->send_num -= once_len;
        }

        spi_priv->status.busy   = 0U;
    } else {
        spi_priv->send_num      = num;
        spi_priv->clk_num       = num;
        spi_priv->send_buf      = ptxbuffer;
        spi_priv->transfer_num  = 0;
        addr->SPIENR            = DW_SPI_ENABLE; /* enable SPI */
        addr->TXFTLR            = DW_SPI_TXFIFO_LV;
        addr->SER               = spi_priv->enable_slave;
        addr->IMR               = DW_SPI_IMR_TXEIM | DW_SPI_IMR_TXOIM;

    }

    return 0;
}

/**
\brief      receiving data from SPI receiver. if non-blocking mode, this function only start the receiving,
            \ref spi_event_e is signaled when operation completes or error happens.
            \ref csi_spi_get_status can indicates operation status.
            if blocking mode, this function return after operation completes or error happens.
\param[in]  handle spi handle to operate.
\param[out] data  Pointer to buffer for data to receive from SPI receiver
\param[in]  num   Number of data items to receive
\return     error code
*/
int32_t csi_spi_receive(spi_handle_t handle, void *data, uint32_t num)
{
    if (handle == NULL || data == NULL || num == 0) {
        return ERR_SPI(DRV_ERROR_PARAMETER);
    }

    dw_spi_priv_t *spi_priv = handle;

    if ((spi_priv->state & SPI_CONFIGURED) == 0U) {
        return ERR_SPI(DRV_ERROR_UNSUPPORTED);
    }

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    spi_priv->status.busy = 1U;
    spi_priv->status.data_lost = 0U;
    spi_priv->status.mode_fault = 0U;
    spi_priv->tot_num           = num;
    spi_priv->transfer_stat     = TRANSFER_STAT_RCV;

    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);
    uint8_t *prx_buffer = data;

    if (spi_priv->block_mode) {

        spi_priv->recv_buf = prx_buffer;
        spi_priv->recv_num = num;


        while (spi_priv->recv_num) {
            addr->SPIENR    = DW_SPI_DISABLE;   /* disable SPI */
            dw_spi_set_mode(spi_priv, DWENUM_SPI_RX);
            addr->SER = spi_priv->enable_slave;   /* enable all cs */

            uint32_t once_len = (spi_priv->recv_num >= DW_SPI_FIFO_MAX_LV) ?
                                DW_SPI_FIFO_MAX_LV : spi_priv->recv_num;
            addr->CTRLR1 = once_len - 1;
            addr->RXFTLR = once_len - 1;
            addr->SPIENR    = DW_SPI_ENABLE;
            addr->DR     = 0;

            uint32_t timecount = 0;

            while (addr->RXFLR < once_len) {
                timecount++;

                if (timecount >= SPI_BUSY_TIMEOUT) {
                    return ERR_SPI(DRV_ERROR_TIMEOUT);
                }
            }

            int i = 0;

            for (i = 0; i < once_len; i++) {
                *spi_priv->recv_buf++ = addr->DR;
            }

            spi_priv->recv_num -= once_len;
        }

        spi_priv->status.busy   = 0U;
    } else {
        addr->SPIENR    = DW_SPI_DISABLE;   /* disable SPI */
        spi_priv->recv_buf = prx_buffer;
        spi_priv->recv_num = num;
        dw_spi_set_mode(spi_priv, DWENUM_SPI_RX);
        addr->SER = spi_priv->enable_slave;   /* enable all cs */

        if (num > DW_SPI_FIFO_MAX_LV) {
            addr->RXFTLR = DW_SPI_FIFO_MAX_LV - 1;
            addr->CTRLR1 = DW_SPI_FIFO_MAX_LV - 1;
        } else {
            addr->RXFTLR = num - 1;
            addr->CTRLR1 = num - 1;
        }

        addr->IMR    = DW_SPI_IMR_RXFIM | DW_SPI_IMR_RXOIM;
        addr->SPIENR = DW_SPI_ENABLE; /* enable SPI */
        addr->DR     = DW_SPI_START_RX ;
    }

    return 0;
}

/**
  \brief       sending/receiving data to/from SPI transmitter/receiver.
               if non-blocking mode, this function only start the transfer,
               \ref spi_event_e is signaled when operation completes or error happens.
               \ref csi_spi_get_status can indicates operation status.
               if blocking mode, this function return after operation completes or error happens.
  \param[in]   handle spi handle to operate.
  \param[in]   data_out  Pointer to buffer with data to send to SPI transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SPI receiver
  \param[in]   num_out      Number of data items to send
  \param[in]   num_in       Number of data items to receive
  \param[in]   block_mode   blocking and non_blocking to selcect
  \return      error code
*/
int32_t csi_spi_transfer(spi_handle_t handle, const void *data_out, void *data_in, uint32_t num_out, uint32_t num_in)
{
    if (handle == NULL || data_in == NULL || num_out == 0 || num_in == 0 || data_out == NULL) {
        return ERR_SPI(DRV_ERROR_PARAMETER);
    }

    dw_spi_priv_t *spi_priv = handle;

    if ((spi_priv->state & SPI_CONFIGURED) == 0U) {
        return ERR_SPI(DRV_ERROR_UNSUPPORTED);
    }

    if (spi_priv->status.busy) {
        return ERR_SPI(DRV_ERROR_BUSY);
    }

    spi_priv->status.busy = 1U;
    spi_priv->status.data_lost = 0U;
    spi_priv->status.mode_fault = 0U;

    spi_priv->tot_num = (num_out > num_in) ? num_out : num_in;
    spi_priv->transfer_stat     = TRANSFER_STAT_TRAN;

    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    addr->SPIENR    = DW_SPI_DISABLE;   /* disable SPI */
    dw_spi_set_mode(spi_priv, DWENUM_SPI_TXRX);
    addr->SER = spi_priv->enable_slave;   /* enable all cs */
    uint8_t *ptx_buffer = (uint8_t *)data_out;
    uint8_t *prx_buffer = (uint8_t *)data_in;
    uint32_t i = 0u;

    if (spi_priv->block_mode) {
        addr->SPIENR            = DW_SPI_ENABLE;

        while (spi_priv->tot_num) {
            uint32_t transfer_num = (spi_priv->tot_num > DW_SPI_FIFO_MAX_LV) ? DW_SPI_FIFO_MAX_LV : spi_priv->tot_num;

            for (i = 0; i < transfer_num; i++) { /* transmit datas in transmit-buffer */
                addr->DR = *ptx_buffer;
                ptx_buffer++;
            }

            uint32_t timecount = 0;

            while ((addr->SR & DW_SPI_BUSY)) {
                timecount++;

                if (timecount >= SPI_BUSY_TIMEOUT) {
                    return ERR_SPI(DRV_ERROR_TIMEOUT);
                }
            }

            timecount = 0;

            while ((addr->SR & DW_SPI_TFE) == 0) {
                timecount++;

                if (timecount >= SPI_BUSY_TIMEOUT) {
                    return ERR_SPI(DRV_ERROR_TIMEOUT);
                }
            }

            timecount = 0;

            while ((addr->SR & DW_SPI_RFNE) == 0) {
                timecount++;

                if (timecount >= SPI_BUSY_TIMEOUT) {
                    return ERR_SPI(DRV_ERROR_TIMEOUT);
                }
            }

            for (i = 0; i < transfer_num; i++) {
                while (!(addr->RXFLR));

                *prx_buffer = addr->DR;
                prx_buffer++;
            }

            spi_priv->tot_num -= transfer_num;
        }

        addr->SER = 0;

        addr->SPIENR = DW_SPI_DISABLE;
        spi_priv->status.busy = 0U;
    } else {
        spi_priv->send_buf      = ptx_buffer;
        spi_priv->recv_buf      = prx_buffer;
        spi_priv->send_num      = num_out;
        spi_priv->recv_num      = num_in;
        spi_priv->clk_num       = num_in;
        spi_priv->transfer_num  = 0;

        addr->TXFTLR            = DW_SPI_TXFIFO_LV;
        addr->SPIENR            = DW_SPI_ENABLE;
        /* enable transmit FIFO empty interrupt */
        addr->IMR               |= DW_SPI_IMR_TXEIM;
    }

    return 0;
}

/**
  \brief       abort spi transfer.
  \param[in]   handle spi handle to operate.
  \return      error code
*/
int32_t csi_spi_abort_transfer(spi_handle_t handle)
{
    SPI_NULL_PARAM_CHK(handle);

    dw_spi_priv_t *spi_priv = handle;
    dw_spi_reg_t *addr = (dw_spi_reg_t *)(spi_priv->base);

    addr->SPIENR = DW_SPI_DISABLE;
    spi_priv->status.busy = 0U;
    spi_priv->recv_buf = NULL;
    spi_priv->recv_num = 0;

    return 0;
}

/**
  \brief       Get SPI status.
  \param[in]   handle spi handle to operate.
  \return      SPI status \ref ARM_SPI_STATUS
*/
spi_status_t csi_spi_get_status(spi_handle_t handle)
{
    spi_status_t spi_status = {0};

    if (handle == NULL) {
        return spi_status;
    }

    dw_spi_priv_t *spi_priv = handle;

    return spi_priv->status;
}

/**
  \brief       Get spi transferred data count.
  \param[in]   handle  spi handle to operate.
  \return      number of data bytes transferred
*/
uint32_t csi_spi_get_data_count(spi_handle_t handle)
{
    uint32_t cnt = 0;

    if (handle == NULL) {
        return 0;
    }

    dw_spi_priv_t *spi_priv = handle;

    if (spi_priv->transfer_stat == TRANSFER_STAT_SEND) {
        cnt = spi_priv->tot_num - spi_priv->send_num;
    } else if (spi_priv->transfer_stat == TRANSFER_STAT_RCV) {
        cnt = spi_priv->tot_num - spi_priv->recv_num;
    } else if (spi_priv->transfer_stat == TRANSFER_STAT_TRAN) {
        cnt = spi_priv->tot_num - (spi_priv->recv_num > spi_priv->send_num ? spi_priv->recv_num : spi_priv->send_num);
    }

    return cnt;
}

/**
  \brief       Control the Slave Select signal (SS).
  \param[in]   handle  spi handle to operate.
  \param[in]   stat    SS state. \ref spi_ss_stat_e.
  \return      error code
*/
int32_t csi_spi_ss_control(spi_handle_t handle, spi_ss_stat_e stat)
{
    SPI_NULL_PARAM_CHK(handle);
    return ERR_SPI(DRV_ERROR_UNSUPPORTED);
}
