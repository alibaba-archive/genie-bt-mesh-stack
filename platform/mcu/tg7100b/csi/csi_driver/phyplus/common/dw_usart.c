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
 * @file     dw_usart.c
 * @brief    CSI Source File for usart Driver
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/

#include <csi_config.h>
#include <stdbool.h>
#include <string.h>
#include <drv_irq.h>
#include <drv_usart.h>
#include <drv_pmu.h>
#include <dw_usart.h>
#include <soc.h>
#include <drv_dmac.h>
#include <sys_freq.h>
#include <clock.h>

#define ERR_USART(errno) (CSI_DRV_ERRNO_USART_BASE | errno)

/*
 * setting config may be accessed when the USART is not
 * busy(USR[0]=0) and the DLAB bit(LCR[7]) is set.
 */

#define WAIT_USART_IDLE(addr)\
    do {                       \
        int32_t timecount = 0;  \
        while ((addr->USR & USR_UART_BUSY) && (timecount < UART_BUSY_TIMEOUT)) {\
            timecount++;\
        }\
        if (timecount >= UART_BUSY_TIMEOUT) {\
            return ERR_USART(DRV_ERROR_TIMEOUT);\
        }                                   \
    } while(0)

#define WAIT_USART_IDLE_VOID(addr)\
    do {                       \
        int32_t timecount = 0;  \
        while ((addr->USR & USR_UART_BUSY) && (timecount < UART_BUSY_TIMEOUT)) {\
            timecount++;\
        }\
        if (timecount >= UART_BUSY_TIMEOUT) {\
            return;\
        }                                   \
    } while(0)

#define USART_NULL_PARAM_CHK(para)    HANDLE_PARAM_CHK(para, ERR_USART(DRV_ERROR_PARAMETER))

typedef struct {
#ifdef CONFIG_LPM
    uint8_t usart_power_status;
    uint32_t usart_regs_saved[5];
#endif
    uint32_t base;
    uint32_t irq;
    usart_event_cb_t cb_event;           ///< Event callback
    uint32_t rx_total_num;
    uint32_t tx_total_num;
    uint8_t *rx_buf;
    uint8_t *tx_buf;
    volatile uint32_t rx_cnt;
    volatile uint32_t tx_cnt;
    volatile uint32_t tx_busy;
    volatile uint32_t rx_busy;
    //for get data count
    uint32_t last_tx_num;
    uint32_t last_rx_num;
    int32_t idx;
    uint32_t fcr_reg;
} dw_usart_priv_t;

extern int32_t target_usart_init(int32_t idx, uint32_t *base, uint32_t *irq, void **handler);
extern int32_t target_get_addr_space(uint32_t addr);

static uint32_t usart_regs_saved[CONFIG_USART_NUM][5];

static dw_usart_priv_t usart_instance[CONFIG_USART_NUM];

static const usart_capabilities_t usart_capabilities = {
    .asynchronous = 1,          /* supports USART (Asynchronous) mode */
    .synchronous_master = 0,    /* supports Synchronous Master mode */
    .synchronous_slave = 0,     /* supports Synchronous Slave mode */
    .single_wire = 0,           /* supports USART Single-wire mode */
    .event_tx_complete = 1,     /* Transmit completed event */
    .event_rx_timeout = 0,      /* Signal receive character timeout event */
};

static int32_t usart_wait_timeout(usart_handle_t handle, dw_usart_reg_t *addr)
{
    uint32_t timecount = 0;

    while (addr->LSR & 0x1 || addr->USR & 0x1) {
        addr->RBR;
        timecount++;

        if (timecount >= UART_BUSY_TIMEOUT) {
            return -1;
        }
    }

    WAIT_USART_IDLE(addr);

    return 0;
}

/**
  \brief       set the baut drate of usart.
  \param[in]   addr  usart base to operate.
  \return      error code
*/
int32_t csi_usart_config_baudrate(usart_handle_t handle, uint32_t baud)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    if (usart_wait_timeout(handle, addr) != 0) {
        return ERR_USART(USART_ERROR_BAUDRATE);
    }

    /* baudrate=(seriak clock freq)/(16*divisor); algorithm :rounding*/
    uint32_t divisor = ((drv_get_apb_freq()  * 10) / baud) >> 4;

    if ((divisor % 10) >= 5) {
        divisor = (divisor / 10) + 1;
    } else {
        divisor = divisor / 10;
    }

    addr->LCR |= LCR_SET_DLAB;
    /* DLL and DLH is lower 8-bits and higher 8-bits of divisor.*/
    addr->DLL = divisor & 0xff;
    addr->DLH = (divisor >> 8) & 0xff;
    /*
     * The DLAB must be cleared after the baudrate is setted
     * to access other registers.
     */
    addr->LCR &= (~LCR_SET_DLAB);

    return 0;
}

/**
  \brief       config usart mode.
  \param[in]   handle  usart handle to operate.
  \param[in]   mode    \ref usart_mode_e
  \return      error code
*/
int32_t csi_usart_config_mode(usart_handle_t handle, usart_mode_e mode)
{
    USART_NULL_PARAM_CHK(handle);

    if (mode == USART_MODE_ASYNCHRONOUS) {
        return 0;
    }

    return ERR_USART(USART_ERROR_MODE);
}


/**
  \brief       config usart parity.
  \param[in]   handle  usart handle to operate.
  \param[in]   parity    \ref usart_parity_e
  \return      error code
*/
int32_t csi_usart_config_parity(usart_handle_t handle, usart_parity_e parity)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    if (usart_wait_timeout(handle, addr) != 0) {
        return ERR_USART(USART_ERROR_PARITY);
    }

    switch (parity) {
        case USART_PARITY_NONE:
            /*CLear the PEN bit(LCR[3]) to disable parity.*/
            addr->LCR &= (~LCR_PARITY_ENABLE);
            break;

        case USART_PARITY_ODD:
            /* Set PEN and clear EPS(LCR[4]) to set the ODD parity. */
            addr->LCR |= LCR_PARITY_ENABLE;
            addr->LCR &= LCR_PARITY_ODD;
            break;

        case USART_PARITY_EVEN:
            /* Set PEN and EPS(LCR[4]) to set the EVEN parity.*/
            addr->LCR |= LCR_PARITY_ENABLE;
            addr->LCR |= LCR_PARITY_EVEN;
            break;

        default:
            return ERR_USART(USART_ERROR_PARITY);
    }

    return 0;
}

/**
  \brief       config usart stop bit number.
  \param[in]   handle  usart handle to operate.
  \param[in]   stopbits  \ref usart_stop_bits_e
  \return      error code
*/
int32_t csi_usart_config_stopbits(usart_handle_t handle, usart_stop_bits_e stopbit)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    if (usart_wait_timeout(handle, addr) != 0) {
        return ERR_USART(USART_ERROR_STOP_BITS);
    }

    switch (stopbit) {
        case USART_STOP_BITS_1:
            /* Clear the STOP bit to set 1 stop bit*/
            addr->LCR &= LCR_STOP_BIT1;
            break;

        case USART_STOP_BITS_2:
            /*
            * If the STOP bit is set "1",we'd gotten 1.5 stop
            * bits when DLS(LCR[1:0]) is zero, else 2 stop bits.
            */
            addr->LCR |= LCR_STOP_BIT2;
            break;

        default:

            return ERR_USART(USART_ERROR_STOP_BITS);
    }

    return 0;
}

/**
  \brief       config usart data length.
  \param[in]   handle  usart handle to operate.
  \param[in]   databits      \ref usart_data_bits_e
  \return      error code
*/
int32_t csi_usart_config_databits(usart_handle_t handle, usart_data_bits_e databits)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    if (usart_wait_timeout(handle, addr) != 0) {
        return ERR_USART(USART_ERROR_DATA_BITS);
    }

    /* The word size decides by the DLS bits(LCR[1:0]), and the
     * corresponding relationship between them is:
     *   DLS   word size
     *       00 -- 5 bits
     *       01 -- 6 bits
     *       10 -- 7 bits
     *       11 -- 8 bits
     */

    switch (databits) {
        case USART_DATA_BITS_5:
            addr->LCR &= LCR_WORD_SIZE_5;
            break;

        case USART_DATA_BITS_6:
            addr->LCR &= 0xfd;
            addr->LCR |= LCR_WORD_SIZE_6;
            break;

        case USART_DATA_BITS_7:
            addr->LCR &= 0xfe;
            addr->LCR |= LCR_WORD_SIZE_7;
            break;

        case USART_DATA_BITS_8:
            addr->LCR |= LCR_WORD_SIZE_8;
            break;

        default:
            return ERR_USART(USART_ERROR_DATA_BITS);
    }

    return 0;
}

/**
  \brief       get character in query mode.
  \param[in]   handle  usart handle to operate.
  \param[in]   the pointer to the received character if return 0.
  \return      error code
*/
int32_t csi_usart_getchar(usart_handle_t handle, uint8_t *ch)
{
    USART_NULL_PARAM_CHK(handle);
    USART_NULL_PARAM_CHK(ch);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    while (!(addr->LSR & LSR_DATA_READY));

    *ch = addr->RBR;

    return 0;
}

/**
  \brief       transmit character in query mode.
  \param[in]   handle  usart handle to operate.
  \param[in]   ch  the input character
  \return      error code
*/
int32_t csi_usart_putchar(usart_handle_t handle, uint8_t ch)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    uint32_t timecount = 0;

    while ((!(addr->LSR & DW_LSR_TRANS_EMPTY))) {
        timecount++;

        if (timecount >= UART_BUSY_TIMEOUT) {
            return ERR_USART(DRV_ERROR_TIMEOUT);
        }
    }

    addr->THR = ch;

    return 0;
}

/**
  \brief       interrupt service function for transmitter holding register empty.
  \param[in]   usart_priv usart private to operate.
*/
static void dw_usart_intr_threshold_empty(int32_t idx, dw_usart_priv_t *usart_priv)
{
    if (usart_priv->tx_total_num == 0) {
        return;
    }

    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    if (usart_priv->tx_cnt >= usart_priv->tx_total_num) {
        addr->IER &= (~IER_THRE_INT_ENABLE);
        usart_priv->last_tx_num = usart_priv->tx_total_num;

        usart_priv->tx_cnt = 0;
        usart_priv->tx_busy = 0;
        usart_priv->tx_buf = NULL;
        usart_priv->tx_total_num = 0;

        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_SEND_COMPLETE);
        }
    } else {
        uint32_t remain_txdata  = usart_priv->tx_total_num - usart_priv->tx_cnt;
        uint32_t txdata_num = (remain_txdata > (UART_MAX_FIFO - 1)) ? (UART_MAX_FIFO - 1) : remain_txdata;
        uint32_t i = 0u;

        for (i = 0; i < txdata_num; i++) {
            addr->THR = *((uint8_t *)usart_priv->tx_buf);
            usart_priv->tx_cnt++;
            usart_priv->tx_buf++;
        }
    }
}

/**
  \brief        interrupt service function for receiver data available.
  \param[in]   usart_priv usart private to operate.
*/
static void dw_usart_intr_recv_data(int32_t idx, dw_usart_priv_t *usart_priv)
{
    if ((usart_priv->rx_total_num == 0) || (usart_priv->rx_buf == NULL)) {
        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_RECEIVED);
        } else {
            uint8_t data;
            csi_usart_receive_query(usart_priv, &data, 1);
        }

        return;
    }

    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    uint32_t rxfifo_num = 8;//addr->RFL; //chenlf: hardware bug?
    uint32_t rxdata_num = (rxfifo_num > usart_priv->rx_total_num) ? usart_priv->rx_total_num : rxfifo_num;
    uint32_t i;

    for (i = 0; i < rxdata_num; i++) {
        *((uint8_t *)usart_priv->rx_buf) = addr->RBR;;
        usart_priv->rx_cnt++;
        usart_priv->rx_buf++;
    }

    if (usart_priv->rx_cnt >= usart_priv->rx_total_num) {
        usart_priv->last_rx_num = usart_priv->rx_total_num;
        usart_priv->rx_cnt = 0;
        usart_priv->rx_buf = NULL;
        usart_priv->rx_busy = 0;
        usart_priv->rx_total_num = 0;

        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_RECEIVE_COMPLETE);
        }
    }

}

/**
  \brief        interrupt service function for receiver line.
  \param[in]   usart_priv usart private to operate.
*/
static void dw_usart_intr_recv_line(int32_t idx, dw_usart_priv_t *usart_priv)
{
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    uint32_t lsr_stat = addr->LSR;

    uint32_t timecount = 0;

    while (addr->LSR & 0x1) {
        addr->RBR;
        timecount++;

        if (timecount >= UART_BUSY_TIMEOUT) {
            if (usart_priv->cb_event) {
                usart_priv->cb_event(idx, USART_EVENT_RX_TIMEOUT);
            }

            return;
        }
    }

    /** Break Interrupt bit. This is used to indicate the detection of a
      * break sequence on the serial input data.
      */
    if (lsr_stat & DW_LSR_BI) {
        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_RX_BREAK);
        }

        return;
    }

    /** Framing Error bit. This is used to indicate the occurrence of a
      * framing error in the receiver. A framing error occurs when the receiver
      * does not detect a valid STOP bit in the received data.
      */
    if (lsr_stat & DW_LSR_FE) {
        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_RX_FRAMING_ERROR);
        }

        return;

    }

    /** Framing Error bit. This is used to indicate the occurrence of a
      * framing error in the receiver. A framing error occurs when the
      * receiver does not detect a valid STOP bit in the received data.
      */
    if (lsr_stat & DW_LSR_PE) {
        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_RX_PARITY_ERROR);
        }

        return;

    }

    /** Overrun error bit. This is used to indicate the occurrence of an overrun error.
      * This occurs if a new data character was received before the previous data was read.
      */
    if (lsr_stat & DW_LSR_OE) {
        if (usart_priv->cb_event) {
            usart_priv->cb_event(idx, USART_EVENT_RX_OVERFLOW);
        }

        return;

    }

}
/**
  \brief        interrupt service function for character timeout.
  \param[in]   usart_priv usart private to operate.
*/
static void dw_usart_intr_char_timeout(int32_t idx, dw_usart_priv_t *usart_priv)
{
    if ((usart_priv->rx_total_num != 0) && (usart_priv->rx_buf != NULL)) {
        dw_usart_intr_recv_data(idx, usart_priv);
        return;
    }

    if (usart_priv->cb_event) {
        usart_priv->cb_event(idx, USART_EVENT_RECEIVED);
    } else {
        dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

        uint32_t timecount = 0;

        while (addr->LSR & 0x1) {
            addr->RBR;
            timecount++;

            if (timecount >= UART_BUSY_TIMEOUT) {
                if (usart_priv->cb_event) {
                    usart_priv->cb_event(idx, USART_EVENT_RX_TIMEOUT);
                }

                return;
            }
        }
    }
}

/**
  \brief       the interrupt service function.
  \param[in]   index of usart instance.
*/
void dw_usart_irqhandler(int32_t idx)
{
    dw_usart_priv_t *usart_priv = &usart_instance[idx];
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    uint8_t intr_state = addr->IIR & 0xf;

    switch (intr_state) {
        case DW_IIR_THR_EMPTY:       /* interrupt source:transmitter holding register empty */
            dw_usart_intr_threshold_empty(idx, usart_priv);
            break;

        case DW_IIR_RECV_DATA:       /* interrupt source:receiver data available or receiver fifo trigger level reached */
            dw_usart_intr_recv_data(idx, usart_priv);
            break;

        case DW_IIR_RECV_LINE:
            dw_usart_intr_recv_line(idx, usart_priv);
            break;

        case DW_IIR_CHAR_TIMEOUT:
            dw_usart_intr_char_timeout(idx, usart_priv);
            break;

        default:
            break;
    }
}

#ifdef CONFIG_LPM
static void manage_clock(usart_handle_t handle, uint8_t enable)
{
    dw_usart_priv_t *usart_priv = handle;
    uint8_t device[] = {CLOCK_MANAGER_UART0, CLOCK_MANAGER_UART1, CLOCK_MANAGER_UART2};
    drv_clock_manager_config(device[usart_priv->idx], enable);
}

static void do_prepare_sleep_action(usart_handle_t handle)
{
    dw_usart_priv_t *usart_priv = handle;
    volatile dw_usart_reg_t *ubase = (dw_usart_reg_t *)(usart_priv->base);
    uint8_t data[16];
    csi_usart_receive_query(handle, data, 16);
    WAIT_USART_IDLE_VOID(ubase);
    ubase->LCR |= LCR_SET_DLAB;
    registers_save(usart_priv->usart_regs_saved, (uint32_t *)ubase, 2);
    ubase->LCR &= ~LCR_SET_DLAB;
    registers_save(&usart_priv->usart_regs_saved[2], (uint32_t *)ubase + 1, 1);
    registers_save(&usart_priv->usart_regs_saved[3], (uint32_t *)ubase + 3, 2);
}

static void do_wakeup_sleep_action(usart_handle_t handle)
{
    dw_usart_priv_t *usart_priv = handle;
    volatile dw_usart_reg_t *ubase = (dw_usart_reg_t *)(usart_priv->base);
    WAIT_USART_IDLE_VOID(ubase);
    ubase->LCR |= LCR_SET_DLAB;
    registers_restore((uint32_t *)ubase, usart_priv->usart_regs_saved, 2);
    ubase->LCR &= ~LCR_SET_DLAB;
    registers_restore((uint32_t *)ubase + 1, &usart_priv->usart_regs_saved[2], 1);
    registers_restore((uint32_t *)ubase + 3, &usart_priv->usart_regs_saved[3], 2);
}
#endif

/**
  \brief       Get driver capabilities.
  \param[in]   idx usart index
  \return      \ref usart_capabilities_t
*/
usart_capabilities_t csi_usart_get_capabilities(int32_t idx)
{
    if (idx < 0 || idx >= CONFIG_USART_NUM) {
        usart_capabilities_t ret;
        memset(&ret, 0, sizeof(usart_capabilities_t));
        return ret;
    }

    return usart_capabilities;
}

/**
  \brief       Initialize USART Interface. 1. Initializes the resources needed for the USART interface 2.registers event callback function
  \param[in]   idx usart index
  \param[in]   cb_event  Pointer to \ref usart_event_cb_t
  \return      return usart handle if success
*/
usart_handle_t csi_usart_initialize(int32_t idx, usart_event_cb_t cb_event)
{
    uint32_t base = 0u;
    uint32_t irq = 0u;
    void *handler;
    int32_t ret = target_usart_init(idx, &base, &irq, &handler);

    if (ret < 0 || ret >= CONFIG_USART_NUM) {
        return NULL;
    }

    dw_usart_priv_t *usart_priv = &usart_instance[idx];
    usart_priv->base = base;
    usart_priv->irq = irq;
    usart_priv->cb_event = cb_event;
    usart_priv->idx = idx;

#ifdef CONFIG_LPM
    csi_usart_power_control(usart_priv, DRV_POWER_FULL);
#endif
    clk_gate_enable(MOD_UART);
    clk_reset(MOD_UART);

    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    /* FIFO enable */
    addr->FCR = DW_FCR_FIFOE | DW_FCR_RT_FIFO_QUARTER;
    usart_priv->fcr_reg = DW_FCR_FIFOE | DW_FCR_RT_FIFO_SINGLE;
    /* enable received data available */
    addr->IER = IER_RDA_INT_ENABLE | IIR_RECV_LINE_ENABLE;

#ifndef CONFIG_DISABLE_IRQ
    drv_irq_register(usart_priv->irq, handler);
    drv_irq_enable(usart_priv->irq);
#endif

    return usart_priv;
}

/**
  \brief       De-initialize UART Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle  usart handle to operate.
  \return      error code
*/
int32_t csi_usart_uninitialize(usart_handle_t handle)
{
    USART_NULL_PARAM_CHK(handle);

    dw_usart_priv_t *usart_priv = handle;
    drv_irq_disable(usart_priv->irq);
    drv_irq_unregister(usart_priv->irq);
    usart_priv->cb_event   = NULL;
    clk_gate_disable(MOD_UART);

#ifdef CONFIG_LPM
    csi_usart_power_control(usart_priv, DRV_POWER_OFF);
#endif

    return 0;
}

/**
  \brief       config usart mode.
  \param[in]   handle  usart handle to operate.
  \param[in]   baud      baud rate
  \param[in]   mode      \ref usart_mode_e
  \param[in]   parity    \ref usart_parity_e
  \param[in]   stopbits  \ref usart_stop_bits_e
  \param[in]   bits      \ref usart_data_bits_e
  \return      error code
*/
int32_t csi_usart_config(usart_handle_t handle,
                         uint32_t baud,
                         usart_mode_e mode,
                         usart_parity_e parity,
                         usart_stop_bits_e stopbits,
                         usart_data_bits_e bits)
{
    int32_t ret;
    USART_NULL_PARAM_CHK(handle);

    /* control the data_bit of the usart*/
    ret = csi_usart_config_baudrate(handle, baud);

    if (ret < 0) {
        return ret;
    }

    /* control mode of the usart*/
    ret = csi_usart_config_mode(handle, mode);

    if (ret < 0) {
        return ret;
    }

    /* control the parity of the usart*/
    ret = csi_usart_config_parity(handle, parity);

    if (ret < 0) {
        return ret;
    }

    /* control the stopbit of the usart*/
    ret = csi_usart_config_stopbits(handle, stopbits);

    if (ret < 0) {
        return ret;
    }

    ret = csi_usart_config_databits(handle, bits);

    if (ret < 0) {
        return ret;
    }

    return 0;
}

/**
  \brief       Start sending data to UART transmitter,(received data is ignored).
               The function is non-blocking,UART_EVENT_TRANSFER_COMPLETE is signaled when transfer completes.
               csi_usart_get_status can indicates if transmission is still in progress or pending
  \param[in]   handle  usart handle to operate.
  \param[in]   data  Pointer to buffer with data to send to UART transmitter. data_type is : uint8_t for 1..8 data bits, uint16_t for 9..16 data bits,uint32_t for 17..32 data bits,
  \param[in]   num   Number of data items to send
  \return      error code
*/
int32_t csi_usart_send(usart_handle_t handle, const void *data, uint32_t num)
{
    USART_NULL_PARAM_CHK(handle);
    USART_NULL_PARAM_CHK(data);

    int i;
    uint8_t *buff = (uint8_t *)data;

    if (num == 0) {
        return ERR_USART(DRV_ERROR_PARAMETER);
    }

    dw_usart_priv_t *usart_priv = handle;

    for (i = 0; i < num; i++) {
        csi_usart_putchar(handle, buff[i]);
    }
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    int ret = usart_wait_timeout(handle, addr);
    if (ret)
    {
        return ERR_USART(DRV_ERROR_TIMEOUT);
    }

    if (usart_priv->cb_event) {
        usart_priv->cb_event(usart_priv->idx, USART_EVENT_SEND_COMPLETE);
    }

    return 0;
}

/**
  \brief       Abort Send data to UART transmitter
  \param[in]   handle  usart handle to operate.
  \return      error code
*/
int32_t csi_usart_abort_send(usart_handle_t handle)
{
    USART_NULL_PARAM_CHK(handle);
    return ERR_USART(DRV_ERROR_UNSUPPORTED);
}

/**
  \brief       Start receiving data from UART receiver.
  \param[in]   handle  usart handle to operate.
  \param[out]  data  Pointer to buffer for data to receive from UART receiver
  \param[in]   num   Number of data items to receive
  \return      error code
*/
int32_t csi_usart_receive(usart_handle_t handle, void *data, uint32_t num)
{
    USART_NULL_PARAM_CHK(handle);
    USART_NULL_PARAM_CHK(data);

    dw_usart_priv_t *usart_priv = handle;

    usart_priv->rx_buf = (uint8_t *)data;   // Save receive buffer usart
    usart_priv->rx_total_num = num;         // Save number of data to be received
    usart_priv->rx_cnt = 0;
    usart_priv->rx_busy = 1;
    usart_priv->last_rx_num = 0;

    return 0;

}

/**
  \brief       query data from UART receiver FIFO.
  \param[in]   handle  usart handle to operate.
  \param[out]  data  Pointer to buffer for data to receive from UART receiver
  \param[in]   num   Number of data items to receive
  \return      receive fifo data num
*/
int32_t csi_usart_receive_query(usart_handle_t handle, void *data, uint32_t num)
{
    USART_NULL_PARAM_CHK(handle);
    USART_NULL_PARAM_CHK(data);

    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    int32_t recv_num = 0;
    uint8_t *dest = (uint8_t *)data;

    while (addr->LSR & 0x1) {
        *dest++ = addr->RBR;
        recv_num++;

        if (recv_num >= num) {
            break;
        }
    }

    return recv_num;

}

/**
  \brief       Abort Receive data from UART receiver
  \param[in]   handle  usart handle to operate.
  \return      error code
*/
int32_t csi_usart_abort_receive(usart_handle_t handle)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;

    usart_priv->rx_cnt = usart_priv->rx_total_num;
    return 0;
}

/**
  \brief       Start sending/receiving data to/from UART transmitter/receiver.
  \param[in]   handle  usart handle to operate.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \return      error code
*/
int32_t csi_usart_transfer(usart_handle_t handle, const void *data_out, void *data_in, uint32_t num)
{
    USART_NULL_PARAM_CHK(handle);
    return ERR_USART(DRV_ERROR_UNSUPPORTED);
}

/**
  \brief       abort sending/receiving data to/from USART transmitter/receiver.
  \param[in]   handle  usart handle to operate.
  \return      error code
*/
int32_t csi_usart_abort_transfer(usart_handle_t handle)
{
    USART_NULL_PARAM_CHK(handle);
    return ERR_USART(DRV_ERROR_UNSUPPORTED);
}

/**
  \brief       Get USART status.
  \param[in]   handle  usart handle to operate.
  \return      USART status \ref usart_status_t
*/
usart_status_t csi_usart_get_status(usart_handle_t handle)
{
    if (handle == NULL) {
        usart_status_t ret;
        memset(&ret, 0, sizeof(usart_status_t));
        return ret;
    }

    usart_status_t usart_status;
    memset(&usart_status, 0, sizeof(usart_status_t));
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);
    uint32_t line_status_reg    = addr->LSR;

    usart_status.tx_busy = usart_priv->tx_busy;
    usart_status.rx_busy = usart_priv->rx_busy;

    if (line_status_reg & DW_LSR_BI) {
        usart_status.rx_break = 1;
    }

    if (line_status_reg & DW_LSR_FE) {
        usart_status.rx_framing_error = 1;
    }

    if (line_status_reg & DW_LSR_PE) {
        usart_status.rx_parity_error = 1;
    }

    usart_status.tx_enable  = 1;
    usart_status.rx_enable  = 1;

    return usart_status;
}

/**
  \brief       control the transmit.
  \param[in]   handle  usart handle to operate.
  \param[in]   1 - enable the transmitter. 0 - disable the transmitter
  \return      error code
*/
int32_t csi_usart_control_tx(usart_handle_t handle, uint32_t enable)
{
    USART_NULL_PARAM_CHK(handle);
    return 0;
}

/**
  \brief       control the receive.
  \param[in]   handle  usart handle to operate.
  \param[in]   1 - enable the receiver. 0 - disable the receiver
  \return      error code
*/
int32_t csi_usart_control_rx(usart_handle_t handle, uint32_t enable)
{
    USART_NULL_PARAM_CHK(handle);
    return 0;
}

/**
  \brief       control the break.
  \param[in]   handle  usart handle to operate.
  \param[in]   1- Enable continuous Break transmission,0 - disable continuous Break transmission
  \return      error code
*/
int32_t csi_usart_control_break(usart_handle_t handle, uint32_t enable)
{
    USART_NULL_PARAM_CHK(handle);
    return ERR_USART(DRV_ERROR_UNSUPPORTED);
}

/**
  \brief       flush receive/send data.
  \param[in]   handle usart handle to operate.
  \param[in]   type \ref usart_flush_type_e.
  \return      error code
*/
int32_t csi_usart_flush(usart_handle_t handle, usart_flush_type_e type)
{
    USART_NULL_PARAM_CHK(handle);

    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    uint32_t timecount = 0;

    if (type == USART_FLUSH_WRITE) {
        usart_priv->fcr_reg  |= DW_FCR_XFIFOR;
        addr->FCR = usart_priv->fcr_reg;
        usart_priv->fcr_reg  &= ~DW_FCR_XFIFOR;

        while (addr->FCR & DW_FCR_XFIFOR) {
            timecount++;

            if (timecount >= UART_BUSY_TIMEOUT) {
                return ERR_USART(DRV_ERROR_TIMEOUT);
            }
        }
    } else if (type == USART_FLUSH_READ) {
        usart_priv->fcr_reg |= DW_FCR_RFIFOR;
        addr->FCR = usart_priv->fcr_reg;
        usart_priv->fcr_reg  &= DW_FCR_RFIFOR;

        while (addr->FCR & DW_FCR_RFIFOR) {
            timecount++;

            if (timecount >= UART_BUSY_TIMEOUT) {
                return ERR_USART(DRV_ERROR_TIMEOUT);
            }
        }
    } else {
        return ERR_USART(DRV_ERROR_PARAMETER);
    }

    return 0;
}

/**
  \brief       set interrupt mode.
  \param[in]   handle usart handle to operate.
  \param[in]   type \ref usart_intr_type_e.
  \param[in]   flag 0-OFF, 1-ON.
  \return      error code
*/
int32_t csi_usart_set_interrupt(usart_handle_t handle, usart_intr_type_e type, int32_t flag)
{
    USART_NULL_PARAM_CHK(handle);

    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    switch (type) {
        case USART_INTR_WRITE:
            if (flag == 0) {
                addr->IER &= ~IER_THRE_INT_ENABLE;
            } else if (flag == 1) {
                addr->IER |= IER_THRE_INT_ENABLE;
            } else {
                return ERR_USART(DRV_ERROR_PARAMETER);
            }

            break;

        case USART_INTR_READ:
            if (flag == 0) {
                addr->IER &= ~IER_RDA_INT_ENABLE;
            } else if (flag == 1) {
                addr->IER |= IER_RDA_INT_ENABLE;
            } else {
                return ERR_USART(DRV_ERROR_PARAMETER);
            }

            break;

        default:
            return ERR_USART(DRV_ERROR_PARAMETER);

    }

    return 0;
}

void csi_usart_prepare_sleep_action(int32_t idx)
{
    dw_usart_priv_t *usart_priv = NULL;
    usart_handle_t handle = NULL;
    usart_priv = &usart_instance[idx];

    if (!usart_priv || idx >= CONFIG_USART_NUM) {
        return;
    }

    handle = usart_priv;
    volatile dw_usart_reg_t *ubase = (dw_usart_reg_t *)(usart_priv->base);
    uint8_t data[16];
    csi_usart_receive_query(handle, data, 16);
    WAIT_USART_IDLE_VOID(ubase);
    ubase->LCR |= LCR_SET_DLAB;
    registers_save(usart_regs_saved[idx], (uint32_t *)ubase, 2);
    ubase->LCR &= ~LCR_SET_DLAB;
    registers_save(&usart_regs_saved[idx][2], (uint32_t *)ubase + 1, 1);
    registers_save(&usart_regs_saved[idx][3], (uint32_t *)ubase + 3, 2);

}

void csi_usart_wakeup_sleep_action(int32_t idx)
{
    dw_usart_priv_t *usart_priv = NULL;
    usart_priv = &usart_instance[idx];

    if (!usart_priv || idx >= CONFIG_USART_NUM) {
        return;
    }

    volatile dw_usart_reg_t *ubase = (dw_usart_reg_t *)(usart_priv->base);
    WAIT_USART_IDLE_VOID(ubase);
    ubase->LCR |= LCR_SET_DLAB;
    registers_restore((uint32_t *)ubase, usart_regs_saved[idx], 2);
    ubase->LCR &= ~LCR_SET_DLAB;
    registers_restore((uint32_t *)ubase + 1, &usart_regs_saved[idx][2], 1);
    registers_restore((uint32_t *)ubase + 3, &usart_regs_saved[idx][3], 2);
    ubase->FCR |= 0x41;
}


/**
  \brief       Get usart send data count.
  \param[in]   handle  usart handle to operate.
  \return      number of currently transmitted data bytes
*/
uint32_t csi_usart_get_tx_count(usart_handle_t handle)
{
    USART_NULL_PARAM_CHK(handle);

    dw_usart_priv_t *usart_priv = handle;

    if (usart_priv->tx_busy) {
        return usart_priv->tx_cnt;
    } else {
        return usart_priv->last_tx_num;
    }
}

/**
  \brief       Get usart receive data count.
  \param[in]   handle  usart handle to operate.
  \return      number of currently received data bytes
*/
uint32_t csi_usart_get_rx_count(usart_handle_t handle)
{
    USART_NULL_PARAM_CHK(handle);

    dw_usart_priv_t *usart_priv = handle;

    if (usart_priv->rx_busy) {
        return usart_priv->rx_cnt;
    } else {
        return usart_priv->last_rx_num;
    }
}

/**
  \brief       control usart power.
  \param[in]   handle  usart handle to operate.
  \param[in]   state   power state.\ref csi_power_stat_e.
  \return      error code
*/
int32_t csi_usart_power_control(usart_handle_t handle, csi_power_stat_e state)
{
    USART_NULL_PARAM_CHK(handle);

    if (state == DRV_POWER_FULL) {
        clk_gate_enable(MOD_UART);
        return 0;
    } else if (state == DRV_POWER_OFF) {
        clk_gate_disable(MOD_UART);
        return 0;
    } else {
#ifdef CONFIG_LPM
        power_cb_t callback;
        callback.wakeup = do_wakeup_sleep_action;
        callback.sleep = do_prepare_sleep_action;
        callback.manage_clock = manage_clock;
        return drv_soc_power_control(handle, state, &callback);
#else
        return ERR_USART(DRV_ERROR_UNSUPPORTED);
#endif
    }

}

/**
  \brief       config usart flow control type.
  \param[in]   handle  usart handle to operate.
  \param[in]   flowctrl_type   flow control type.\ref usart_flowctrl_type_e.
  \return      error code
*/
int32_t csi_usart_config_flowctrl(usart_handle_t handle,
                                  usart_flowctrl_type_e flowctrl_type)
{
    USART_NULL_PARAM_CHK(handle);
    dw_usart_priv_t *usart_priv = handle;
    dw_usart_reg_t *addr = (dw_usart_reg_t *)(usart_priv->base);

    switch (flowctrl_type) {
        case USART_FLOWCTRL_CTS:
            return ERR_USART(DRV_ERROR_UNSUPPORTED);

        case USART_FLOWCTRL_RTS:
            return ERR_USART(DRV_ERROR_UNSUPPORTED);

        case USART_FLOWCTRL_CTS_RTS:
            WAIT_USART_IDLE(addr);
            addr->MCR |= DW_MCR_AFCE | DW_MCR_RTS;
            break;

        case USART_FLOWCTRL_NONE:
            WAIT_USART_IDLE(addr);
            addr->MCR = 0;
            break;

        default:
            return ERR_USART(DRV_ERROR_UNSUPPORTED);
    }

    return 0;
}


/**
  \brief       config usart clock Polarity and Phase.
  \param[in]   handle  usart handle to operate.
  \param[in]   cpol    Clock Polarity.\ref usart_cpol_e.
  \param[in]   cpha    Clock Phase.\ref usart_cpha_e.
  \return      error code
*/
int32_t csi_usart_config_clock(usart_handle_t handle, usart_cpol_e cpol, usart_cpha_e cpha)
{
    USART_NULL_PARAM_CHK(handle);
    return ERR_USART(DRV_ERROR_UNSUPPORTED);
}
