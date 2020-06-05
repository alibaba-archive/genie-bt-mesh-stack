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
 * @file     spiflash_w25q64fv.c
 * @brief    CSI Source File for Embedded Flash Driver
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/
#include <string.h>
#include <drv_spiflash.h>
#include "spif.h"
#include "hal_mcu.h"
#include <ap_cp.h>
#if defined CONFIG_CHIP_DANICA || CONFIG_CHIP_HOBBIT4 || defined(CONFIG_CHIP_YUNVOICE2)
#define ATTRIBUTE_DATA __attribute__((section(".ram.code")))
#else
#define ATTRIBUTE_DATA
#endif

#define ERR_SPIFLASH(errno) (CSI_DRV_ERRNO_SPIFLASH_BASE | errno)
#define SPIFLASH_NULL_PARAM_CHK(para) HANDLE_PARAM_CHK(para, ERR_SPIFLASH(DRV_ERROR_PARAMETER))

static uint8_t spiflash_init_flag = 0;

typedef struct {
    spiflash_info_t spiflashinfo;
    spiflash_event_cb_t cb;
    spiflash_status_t status;
} ck_spiflash_priv_t;

static ck_spiflash_priv_t s_spiflash_handle[CONFIG_SPIFLASH_NUM];

static const spiflash_capabilities_t driver_capabilities = {
    1, /* event_ready */
    2, /* data_width = 0:8-bit, 1:16-bit, 2:32-bit */
    1  /* erase_chip */
};

/*
basic spif driver
*/
#define SPIF_CONFIG_BASE   0x11080000
#define SPIF_RLEN         (SPIF_CONFIG_BASE+0x88)
#define RFIFO_DEPTH       (SPIF_CONFIG_BASE+0x84)

// commands defines
#define PAGE_PROGRAM_CMD      0x2
#define FAST_PROGRAM_CMD      0xF2

#define WRITE_ENABLE_CMD      0x6
#define WRITE_DISABLE_CMD     0x4

#define READ_STATUS_CMD       0x5
#define WRITE_STATUS_CMD      0x1

#define SECT_ERASE_CMD        0x20
#define BLOCK32_ERASE_CMD     0x52
#define BLOCK64_ERASE_CMD     0xD8
#define CHIP_ERASE_CMD        0xC7

#define DEEP_DOWN_CMD         0xB9
#define RELEASE_DOWN_CMD      0xAB
#define MANUF_ID_CMD          0x90
#define READ_ID_CMD           0x9F

#define FAST_READ_CMD         0x0B
#define READ_DATA_CMD         0x3

#define WAIT_FOR_SPIF()     ((*(volatile uint32_t *)(SPIF_CONFIG_BASE+0x90)) & 0x4)
                            //{while(!( (*(volatile uint32_t *)(SPIF_CONFIG_BASE+0x90)) & 0x4)) ;}
#define PHY_SPIF_TIMEROUT 10000000

static void phy_wait_for_spif(uint32_t timeout)
{
    while(timeout--) {
        if (WAIT_FOR_SPIF()) {
            return;
        }
    }
}

void phy_init_spif(void)
{
    *(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x88) = 0;

    *(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c) = 0;

    *(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c) = 0x32;

    *(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x90) = 0x3;

    *(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c) = 0x7;  //enable spif
}

// disable spif interface
void phy_close_spif(void)
{
    *(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c) = 0;  //enable spif
}

//enable flash write
void phy_enable_spif_flash_write()
{
    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 0;
    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = WRITE_ENABLE_CMD;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);
}

// disable spif write
void phy_disable_spif_flash_write()
{

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 0;
    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = WRITE_DISABLE_CMD;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

}

// check flash internal bus busy or notfor erase and program operation
int phy_check_internal_bus_busy()
{
    int  state1 = 0;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 1;

    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = READ_STATUS_CMD;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);   //不能去掉

    state1 = *(volatile  uint8_t *)(SPIF_CONFIG_BASE);

    return (state1 & 0x1);
}

// read flash internal status regiser
int phy_read_status_register(void)
{
    unsigned int  state;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 1;

    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = READ_STATUS_CMD;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    state = *(volatile uint8_t *)(SPIF_CONFIG_BASE);

    return (state & 0xff);

}

static void phy_wait_internal_bus_busy(uint32_t time_out)
{
    while(time_out--) {
        if (phy_check_internal_bus_busy() == 0) {
            return;
        }
    }
}

// write flash internal status register
void phy_write_status_register(int value)
{

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();

    *(volatile unsigned int *) SPIF_RLEN = 0;

    *(volatile unsigned short *)(SPIF_CONFIG_BASE) = WRITE_STATUS_CMD | (value << 8);

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();

}

static uint32_t phy_ReadWord(unsigned int addr)
{
    uint32_t value;
    uint32_t little_addr;
    uint8_t *p_val = (uint8_t *)&value;

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);;

    HAL_DISABLE_INTERRUPTS();

    *(volatile unsigned int *) SPIF_RLEN = 4;

    little_addr = (addr & 0xff) << 16 | (addr & 0xff0000) >> 16 | (addr & 0xff00);

    *(volatile unsigned int *)SPIF_CONFIG_BASE = READ_DATA_CMD | (little_addr << 8);

    HAL_ENABLE_INTERRUPTS();
    phy_wait_for_spif(PHY_SPIF_TIMEROUT);   //不能去掉


    p_val[0] = *(volatile uint8_t *)(SPIF_CONFIG_BASE);
    p_val[1] = *(volatile uint8_t *)(SPIF_CONFIG_BASE);
    p_val[2] = *(volatile uint8_t *)(SPIF_CONFIG_BASE);
    p_val[3] = *(volatile uint8_t *)(SPIF_CONFIG_BASE);

    return value;
}

int phy_ProgramWord(unsigned long offset, const unsigned char *buf, uint8_t size)   // size must be 4
{
    int    i = 0;
    int    little_addr;
    int    data = 0;
    halIntState_t cs;

    while (i < 4) {
        data |= buf[i] << ((i & 0x03) << 3);
        i++;
    }

    HAL_ENTER_CRITICAL_SECTION(cs);

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();   // enable write

    little_addr = (offset & 0xff) << 16 | (offset & 0xff0000) >> 16 | (offset & 0xff00);


    *(volatile unsigned int *) SPIF_RLEN = 0;   // write  0x0 to RLEN

    *(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
    *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data;

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();   // disable  write

    if (phy_ReadWord(0x11000000 + offset) != data) {
        __enable_irq();
        return 0;
    }

    HAL_EXIT_CRITICAL_SECTION(cs);

    return 1;
}

// erase total chip of flash
void phy_flash_chip_erase(void)
{
    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();   // enable write
    *(volatile unsigned int *) SPIF_RLEN = 0;
    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = CHIP_ERASE_CMD;

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();   // diasble write
}

// erase a block whose size is 32KB
void phy_flash_block32_erase(unsigned int addr)
{
    int little_addr;

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();   // enable write
    *(volatile unsigned int *) SPIF_RLEN = 0;
    little_addr = (addr & 0xff) << 16 | (addr & 0xff0000) >> 16 | (addr & 0xff00);
    *(volatile unsigned int *)(SPIF_CONFIG_BASE) = BLOCK32_ERASE_CMD | (little_addr << 8);

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();   // diasble write
}

// write a Word to flash
int phy_WriteFlash(unsigned int offset, uint32_t  value)
{
    uint32_t temp = value;
    offset &= 0x00ffffff;

    if (phy_ProgramWord(offset, (uint8_t *) &temp, 4) == 0) {
        return 0;
    }

    return 1;

}

// program 64 bytes into flash
int phy_ProgramPage64(unsigned long offset, const unsigned char *buf,  int size)    // size must be <=256
{
    int	i = 0, j;
    int little_addr;
    int data[16];
    //int baseAddr;

    if (size > 64) {
        return 0;
    }

    if (size == 0) {
        return 0;
    }


    for (i = 0; i < 16; i++) {
        data[i] = 0xffffffff;
    }

    i = 0;

    while (i < size) {
        data[i >> 2] &= (~(0xffUL << ((i & 0x03) << 3)));
        data[i >> 2] |= buf[i] << ((i & 0x03) << 3);
        i++;
    }

    j = (size & 0x03) ? ((size >> 2) + 1) : (size >> 2);

    i = 0;
    halIntState_t cs;
    HAL_ENTER_CRITICAL_SECTION(cs);

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();   // enable write

    little_addr = (offset & 0xff) << 16 | (offset & 0xff0000) >> 16 | (offset & 0xff00);

#if 1
    if (size <= 4) {
        *(volatile unsigned int *) SPIF_RLEN = 0;   // write  0x0 to RLEN

        *(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[0];
    } else if (size < 64) {
        *(volatile unsigned int *) SPIF_RLEN = 0;   // write  0x0 to RLEN

        *(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));

        while (i < j) {
            *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[i++];
        }

    } else if (size == 64) {
        *(volatile unsigned int *) SPIF_RLEN = 0;   // write  0x0 to RLEN

        *(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[0];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[1];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[2];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[3];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[4];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[5];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[6];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[7];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[8];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[9];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[10];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[11];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[12];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[13];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[14];

        *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[15];
    }


#endif

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();   // disable  write

    // verify program contents
/*
    i = 0;
    baseAddr = 0x11000000 + offset;
    while (i < j) {
        if (*(volatile int *)(baseAddr + i * 4) != data[i]) {
            HAL_EXIT_CRITICAL_SECTION(cs);
            return 0;
        }

        i++;
    }
*/
    HAL_EXIT_CRITICAL_SECTION(cs);

    return size;
}

// set flash to deep sleep mode
void phy_set_flash_deep_sleep(void)
{

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 0; //set rlen

    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = 0xB9;    // command

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);   //不能去掉

}

// wake up flash from sleep state
void phy_release_flash_deep_sleep(void)
{

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 0; //set rlen

    *(volatile uint8_t *)(SPIF_CONFIG_BASE) = 0xAB;    // command

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);   //不能去掉


}

// read one byte from flash
uint32_t phy_ReadFlash(unsigned int addr)
{
    uint32_t value;
    uint32_t little_addr;

    halIntState_t cs;
    HAL_ENTER_CRITICAL_SECTION(cs);

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);

    *(volatile unsigned int *) SPIF_RLEN = 4;

    little_addr = (addr & 0xff) << 16 | (addr & 0xff0000) >> 16 | (addr & 0xff00);

    *(volatile unsigned int *)SPIF_CONFIG_BASE = READ_DATA_CMD | (little_addr << 8);

    phy_wait_for_spif(PHY_SPIF_TIMEROUT);   //不能去掉

    value = *(volatile uint32_t *)(SPIF_CONFIG_BASE);

    HAL_EXIT_CRITICAL_SECTION(cs);


    return value;
}

// erase a sector of flash, size is 4KB
void phy_flash_sector_erase(unsigned int addr)
{

    int little_addr;
    halIntState_t cs;
    HAL_ENTER_CRITICAL_SECTION(cs);

    if ((addr & 0xffffff) < 0x2000) {
        return;
    }

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();   // enable write
    *(volatile unsigned int *) SPIF_RLEN = 0;
    little_addr = (addr & 0xff) << 16 | (addr & 0xff0000) >> 16 | (addr & 0xff00);
    *(volatile unsigned int *)(SPIF_CONFIG_BASE) = SECT_ERASE_CMD | (little_addr << 8);

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();   // diasble write
    HAL_EXIT_CRITICAL_SECTION(cs);
}

// erase a block whose size is 64KB
void phy_flash_block64_erase(unsigned int addr)
{
    int little_addr;

    if ((addr & 0xffffff) < 0x10000) {
        return;
    }

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_enable_spif_flash_write();   // enable write
    *(volatile unsigned int *) SPIF_RLEN = 0;
    little_addr = (addr & 0xff) << 16 | (addr & 0xff0000) >> 16 | (addr & 0xff00);
    *(volatile uint32_t *)(SPIF_CONFIG_BASE) = BLOCK64_ERASE_CMD | (little_addr << 8);

    phy_wait_internal_bus_busy(PHY_SPIF_TIMEROUT);

    phy_disable_spif_flash_write();   // diasble write
}

/*
cpi api
*/
int ATTRIBUTE_DATA spiflash_read_status_register(uint8_t *data)
{
    *data = phy_read_status_register();
    return 0;
}

/**
  \brief       Get driver capabilities.
  \param[in]   spiflash handle to operate.
  \return      \ref spiflash_capabilities_t
*/
spiflash_capabilities_t csi_spiflash_get_capabilities(int32_t idx)
{
    if (idx < 0 || idx >= CONFIG_SPIFLASH_NUM) {
        spiflash_capabilities_t ret;
        memset(&ret, 0, sizeof(spiflash_capabilities_t));
        return ret;
    }

    return driver_capabilities;
}

/**
  \brief       Initialize SPIFLASH Interface. 1. Initializes the resources needed for the SPIFLASH interface 2.registers event callback function
  \param[in]   handle  spiflash handle to operate.
  \param[in]   cb_event  Pointer to \ref spiflash_event_cb_t
  \return      \ref execution_status
*/
spiflash_handle_t csi_spiflash_initialize(int32_t idx, spiflash_event_cb_t cb_event)
{
    if (idx < 0 || idx >= CONFIG_SPIFLASH_NUM) {
        return NULL;
    }

    ck_spiflash_priv_t *spiflash_priv = &s_spiflash_handle[idx];

    spiflash_priv->spiflashinfo.start = SPIF_ADDR_START;
    spiflash_priv->spiflashinfo.end = SPIF_ADDR_END;
    spiflash_priv->spiflashinfo.sector_count = SPIF_SECTOR_COUNT;
    spiflash_priv->spiflashinfo.sector_size = SPIF_SECTOR_SIZE;
    spiflash_priv->spiflashinfo.page_size = SPIF_PAGE_SIZE;
    spiflash_priv->spiflashinfo.program_unit = 1;
    spiflash_priv->spiflashinfo.erased_value = 0xff;

    spiflash_priv->cb = cb_event;

    spiflash_priv->status.busy = 0;
    spiflash_priv->status.error = 0U;

    spiflash_init_flag = 1;

    phy_init_spif();

    return (spiflash_handle_t)spiflash_priv;
}

/**
  \brief       De-initialize SPIFLASH Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle  spiflash handle to operate.
  \return      \ref execution_status
*/
int32_t csi_spiflash_uninitialize(spiflash_handle_t handle)
{

#if(DEBUG_EN == 1)
    SPIFLASH_NULL_PARAM_CHK(handle);
#endif

    ck_spiflash_priv_t *spiflash_priv = (ck_spiflash_priv_t *)handle;
    spiflash_init_flag = 0;
    spiflash_priv->cb = NULL;

    //phy_close_spif();

    return 0;
}

/**
  \brief       Read data from Flash.
  \param[in]   handle  spiflash handle to operate.
  \param[in]   addr  Data address.
  \param[in]   data  Pointer to a buffer storing the data read from Flash.
  \param[in]   cnt   Number of data items to read.
  \return      number of data items read or \ref execution_status
*/
int32_t csi_spiflash_read(spiflash_handle_t handle, uint32_t addr, void *data, uint32_t cnt)
{
    int i;
    uint32_t rd = 0;
    uint32_t *p_rd = data;
    uint8_t *pdata = data;
    uint32_t len_rd = 0;
    int8_t len_left;
    ck_spiflash_priv_t *spiflash_priv = handle;

#if(DEBUG_EN == 1)
    SPIFLASH_NULL_PARAM_CHK(handle);
    SPIFLASH_NULL_PARAM_CHK(data);
    SPIFLASH_NULL_PARAM_CHK(cnt);

    if (spiflash_priv->spiflashinfo.start > addr
        || spiflash_priv->spiflashinfo.end < addr
        || spiflash_priv->spiflashinfo.start > (addr + cnt - 1)
        || spiflash_priv->spiflashinfo.end < (addr + cnt - 1)) {
        return ERR_SPIFLASH(DRV_ERROR_PARAMETER);
    }

    if (spiflash_priv->status.busy) {
        return ERR_SPIFLASH(DRV_ERROR_BUSY);
    }

    if (!spiflash_init_flag) {
        return ERR_SPIFLASH(DRV_ERROR);
    }

#endif
    addr &= 0xffffff;
    spiflash_priv->status.error = 0U;

    len_rd = (cnt >> 2);

    if ((((uint32_t)p_rd >> 2) << 2) == (uint32_t)p_rd) {
    for (i = 0; i < len_rd; i++) {
        *(unsigned int *)(p_rd + i) = phy_ReadFlash(addr + (i << 2));
        }
    } else {
        for (i = 0; i < len_rd; i++) {
            rd = phy_ReadFlash(addr + (i << 2));
            pdata[0 + (i << 2)] = rd & 0xff;
            pdata[1 + (i << 2)] = (rd >> 8) & 0xff;
            pdata[2 + (i << 2)] = (rd >> 16) & 0xff;
            pdata[3 + (i << 2)] = (rd >> 24) & 0xff;
        }
    }

    len_left = cnt - (len_rd << 2);
    if (len_left > 0) {
        rd = phy_ReadFlash(addr + (len_rd << 2));
        i = 0;

        while (i < len_left) {
            pdata[i + (len_rd << 2)] = (rd >> (i << 3)) & 0xff;
            i++;
        }
    }

    return cnt;
}

//#include "log.h"
/**
  \brief       Program data to Flash.
  \param[in]   handle  spiflash handle to operate.
  \param[in]   addr  Data address.
  \param[in]   data  Pointer to a buffer containing the data to be programmed to Flash..
  \param[in]   cnt   Number of data items to program.
  \return      number of data items programmed or \ref execution_status
*/
int32_t csi_spiflash_program(spiflash_handle_t handle, uint32_t addr, const void *data, uint32_t cnt)
{

    ck_spiflash_priv_t *spiflash_priv = handle;

#if (DEBUG_EN == 1)
    SPIFLASH_NULL_PARAM_CHK(handle);
    SPIFLASH_NULL_PARAM_CHK(data);
    SPIFLASH_NULL_PARAM_CHK(cnt);

    if (spiflash_priv->spiflashinfo.start > addr
        || spiflash_priv->spiflashinfo.end < addr
        || spiflash_priv->spiflashinfo.start > (addr + cnt - 1)
        || spiflash_priv->spiflashinfo.end < (addr + cnt - 1)) {
        return ERR_SPIFLASH(DRV_ERROR_PARAMETER);
    }

    if (spiflash_priv->status.busy) {
        return ERR_SPIFLASH(DRV_ERROR_BUSY);
    }

    if (!spiflash_init_flag) {
        return ERR_SPIFLASH(DRV_ERROR);
    }

#endif

    uint32_t len = 0;
    uint32_t num = cnt;
    uint8_t *p_wr;
    addr &= 0xffffff;

    spiflash_priv->status.busy = 1U;
    spiflash_priv->status.error = 0U;

    p_wr = (uint8_t *)data;
    len = 64 - (addr % 64);

    if (len >= cnt) {
        phy_ProgramPage64(addr, p_wr, cnt);
    } else {
        phy_ProgramPage64(addr, p_wr, len);

        addr += len;
        p_wr += len;
        cnt -= len;

        if (cnt <= 64) {
            phy_ProgramPage64(addr, p_wr, cnt);
        } else {
            while (cnt >= 64) {
                phy_ProgramPage64(addr, p_wr, 64);
                addr += 64;
                p_wr += 64;
                cnt -= 64;
            }

            if (cnt > 0) {
                phy_ProgramPage64(addr, p_wr, cnt);
            }
        }
    }

    spiflash_priv->status.busy = 0U;

    return num;
}

/**
  \brief       Erase Flash Sector.
  \param[in]   handle  spiflash handle to operate.
  \param[in]   addr  Sector address
  \return      \ref execution_status
*/
int32_t csi_spiflash_erase_sector(spiflash_handle_t handle, uint32_t addr)
{
    ck_spiflash_priv_t *spiflash_priv = handle;

#if (DEBUG_EN == 1)

    if (spiflash_priv->spiflashinfo.start > addr || spiflash_priv->spiflashinfo.end < addr) {
        return ERR_SPIFLASH(DRV_ERROR_PARAMETER);
    }

    SPIFLASH_NULL_PARAM_CHK(handle);

    if (spiflash_priv->status.busy) {
        return ERR_SPIFLASH(DRV_ERROR_BUSY);
    }

    if (!spiflash_init_flag) {
        return ERR_SPIFLASH(DRV_ERROR);
    }

#endif

    spiflash_priv->status.busy = 1U;
    spiflash_priv->status.error = 0U;
    addr &= 0xffffff;

    phy_flash_sector_erase(addr);

    spiflash_priv->status.busy = 0U;

    if (spiflash_priv->cb) {
        spiflash_priv->cb(0, SPIFLASH_EVENT_READY);
    }

    return 0;
}

/**
  \brief       Erase complete Flash.
  \param[in]   handle  spiflash handle to operate.
  \return      \ref execution_status
*/
int32_t csi_spiflash_erase_chip(spiflash_handle_t handle)
{
    ck_spiflash_priv_t *spiflash_priv = handle;
#if (DEBUG_EN == 1)
    SPIFLASH_NULL_PARAM_CHK(handle);

    if (!spiflash_init_flag) {
        return ERR_SPIFLASH(DRV_ERROR);
    }

#endif
    spiflash_priv->status.busy = 1U;

    phy_flash_chip_erase();

    spiflash_priv->status.busy = 0U;

    if (spiflash_priv->cb) {
        spiflash_priv->cb(0, SPIFLASH_EVENT_READY);
    }

    return 0;
}

/**
  \brief       Get Flash information.
  \param[in]   handle  spiflash handle to operate.
  \return      Pointer to Flash information \ref spiflash_info_t
*/
spiflash_info_t *csi_spiflash_get_info(spiflash_handle_t handle)//todo
{
    if (handle == NULL || (!spiflash_init_flag)) {
        return NULL;
    }

    ck_spiflash_priv_t *spiflash_priv = handle;
    spiflash_info_t *spiflash_info = &(spiflash_priv->spiflashinfo);

    return spiflash_info;
}

/**
  \brief       Get SPIFLASH status.
  \param[in]   handle  spiflash handle to operate.
  \return      SPIFLASH status \ref spiflash_status_t
*/
spiflash_status_t csi_spiflash_get_status(spiflash_handle_t handle)
{
    if (handle == NULL || (!spiflash_init_flag)) {
        spiflash_status_t ret;
        memset(&ret, 0, sizeof(spiflash_status_t));
        return ret;
    }

    ck_spiflash_priv_t *spiflash_priv = handle;

    uint8_t status = 0;
    spiflash_read_status_register(&status);
    spiflash_priv->status.busy = status;

    return spiflash_priv->status;
}

int32_t csi_spiflash_power_control(spiflash_handle_t handle, /*csi_power_stat_e*/ uint8_t state)
{
    switch (state) {
        case 0:
            phy_set_flash_deep_sleep();// set flash to deep sleep mode
            break;

        case 1:
            phy_release_flash_deep_sleep();// wake up flash from sleep state
            break;

        default:
            break;
    }

    return state;
}
