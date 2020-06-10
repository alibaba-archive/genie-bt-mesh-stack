/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include "genie_app.h"

#include <stdlib.h>
#include "flash.h"
#include <aos/aos.h>
#include "crc16.h"
#include "mesh_crypto.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_FLASH)
#include "common/log.h"

/***********************************************************************
reliable map
------------------------------------------------------------------------
inited_flag(4B) | reliable_data(4092B)
------------------------------------------------------------------------

reliable data
------------------------------------------------------------------------
flag(1B) | align(1B) | index(2B) | len(2B) | CRC(2B) | data(nB)
------------------------------------------------------------------------
***********************************************************************/

#define GENIE_FLASH_PARTITION_SYSTEM   HAL_PARTITION_PARAMETER_1
#define GENIE_FLASH_PARTITION_USERDATA HAL_PARTITION_PARAMETER_2
#define GENIE_FLASH_PARTITION_RECYCLE  HAL_PARTITION_PARAMETER_3
#define GENIE_FLASH_PARTITION_SEQ      HAL_PARTITION_PARAMETER_4

#define GENIE_FLASH_SIZE_SYSTEM_PART 0x1000
#define GENIE_FLASH_START_INITED_FLAG 0
#define GENIE_FLASH_SIZE_INITED_FLAG 4
#define GENIE_FLASH_SIZE_RELIABLE (GENIE_FLASH_SIZE_SYSTEM_PART - GENIE_FLASH_SIZE_INITED_FLAG)

#define GENIE_FLASH_SIZE_SEQ_PART 0x1000
#define GENIE_FLASH_START_SEQ_BASE 0
#define GENIE_FLASH_SIZE_SEQ_BASE 4
#define GENIE_FLASH_START_SEQ_COUNT (GENIE_FLASH_START_SEQ_BASE + GENIE_FLASH_SIZE_SEQ_BASE)
#define GENIE_FLASH_SIZE_SEQ  (GENIE_FLASH_SIZE_SEQ_PART - GENIE_FLASH_SIZE_SEQ_BASE)

#define GENIE_FLASH_SIZE_USERDATA_PART 0x1000
#define GENIE_FLASH_SIZE_USERDATA      (GENIE_FLASH_SIZE_USERDATA_PART - GENIE_FLASH_SIZE_INITED_FLAG)

#define GENIE_FLASH_START_RECYCLE 0
#define GENIE_FLASH_SIZE_RECYCLE 0x1000

#define GENIE_FLASH_FLAG_INITED_BASE    0xA5A500A8
#define GENIE_FLASH_FLAG_INITED_INVALID 0xAFAFFFA8
#define GENIE_FLASH_FLAG_INITED_ORIGIN  0xFFFFFFFF

typedef enum {
    GENIE_FLASH_FLAG_INITED_SYS = (GENIE_FLASH_FLAG_INITED_BASE | (0x01 << 8)),
    GENIE_FLASH_FLAG_INITED_UD = (GENIE_FLASH_FLAG_INITED_BASE | (0x02 << 8)),
    GENIE_FLASH_FLAG_INITED_SEQ = (GENIE_FLASH_FLAG_INITED_BASE | (0x03 << 8)),
    GENIE_FLASH_FLAG_INITED_RECYCLE = (GENIE_FLASH_FLAG_INITED_BASE | (0x04 << 8)),
    /* Append more partition Here */
};

#define GENIE_FLASH_FLAG_UNSED   0xFF
#define GENIE_FLASH_FLAG_WRITING 0x03
#define GENIE_FLASH_FLAG_ACTIVE  0x01
#define GENIE_FLASH_FLAG_INVALID 0x00

#define SEQ_COUNT_BUFFER_SIZE 4

#define ALIGN_ERROR(data) ((uint32_t)(data) & 0x00000003)

#define CELL_PREPEAR(cell, id, pn, start, size) cell.index = id;                \
                                                cell.pno = pn;                  \
                                                cell.offset = start;            \
                                                cell.end = start+size;          \
                                                memset(&cell.header, 0, sizeof(flash_header_t));

#define CELL_PREPEAR_RELIABLE(cell, id) CELL_PREPEAR(cell, id, GENIE_FLASH_PARTITION_SYSTEM,  \
                                            GENIE_FLASH_SIZE_INITED_FLAG, GENIE_FLASH_SIZE_RELIABLE)

#define CELL_PREPEAR_USERDATA(cell, id) CELL_PREPEAR(cell, id, GENIE_FLASH_PARTITION_USERDATA,  \
                                            GENIE_FLASH_SIZE_INITED_FLAG, GENIE_FLASH_SIZE_USERDATA)

#define CELL_PREPEAR_RECYCLE(cell) CELL_PREPEAR(cell, 0, GENIE_FLASH_PARTITION_RECYCLE,  \
                                            GENIE_FLASH_START_RECYCLE, GENIE_FLASH_SIZE_RECYCLE)

#define CELL_PREPEAR_SEQ(cell) CELL_PREPEAR(cell, 0, GENIE_FLASH_PARTITION_SEQ,  \
                                            GENIE_FLASH_SIZE_INITED_FLAG, GENIE_FLASH_SIZE_SEQ)

typedef struct
{
    uint8_t flag;
    uint8_t align;
    uint16_t index;
    uint16_t length;
    uint16_t crc;
} flash_header_t;

typedef struct{
    uint16_t index;
    hal_partition_t pno;
    uint32_t offset;
    uint32_t end;
    flash_header_t header;
} genie_flash_cell_t;

typedef struct{
    uint16_t remain;
    uint32_t free_offset;
} genie_flash_info_t;

static genie_flash_info_t g_info_system;
static genie_flash_info_t g_info_userdata;
static genie_flash_info_t g_info_recycle;

#define CHECK_RET(ret) { if(ret != 0) BT_ERR("error ret(%d)", ret); }
#if(BT_DBG_ENABLED == 1)
#define RETURN_WHEN_ERR(result, err_code) { if(result != 0) { BT_ERR("%s[%d] error ret(%d)", __func__, __LINE__, result); return err_code; } }
#else
#define RETURN_WHEN_ERR(result, err_code) { if(result != 0) return err_code; }
#endif

#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
typedef struct {
    uint8_t  init_flag;
    int16_t read_offset;
    int16_t write_offset;
} flash_seq_ctl_t;

static flash_seq_ctl_t g_flash_seq_ctl;

E_GENIE_FLASH_ERRCODE genie_flash_seq_init();
static E_GENIE_FLASH_ERRCODE _genie_flash_copy_seq(genie_flash_cell_t *p_src, genie_flash_cell_t *p_dst, uint32_t seq);
#endif

static E_GENIE_FLASH_ERRCODE _genie_flash_copy(genie_flash_cell_t *p_src, genie_flash_cell_t *p_dst, uint8_t *p_buff, uint16_t size);
static E_GENIE_FLASH_ERRCODE genie_flash_init_partition(hal_partition_t in_partition, uint32_t part_size);

static uint16_t _genie_get_crc(uint8_t *p_buff, uint16_t size)
{
    uint16_t crc;
    crc = util_crc16_compute(p_buff, size, NULL);
    BT_DBG("crc(0x%02X)", crc);
    return crc;
}

static uint8_t g_enc_key[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static E_GENIE_FLASH_ERRCODE _genie_flash_encrypt(uint8_t *p_buff, uint16_t size)
{
    uint8_t data_temp[16];
    uint8_t enc_temp[16];
    uint16_t i = 0;

    while(i < size >> 4) {
        memcpy(data_temp, p_buff+(i<<4), 16);
        BT_DBG("data: %s", bt_hex(data_temp, 16));
        bt_mesh_aes_encrypt(g_enc_key, data_temp, enc_temp);
        memcpy(p_buff+(i<<4), enc_temp, 16);
        i++;
    }

    return 0;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_decrypt(uint8_t *p_buff, uint16_t size)
{
    uint8_t data_temp[16];
    uint8_t dec_temp[16];
    uint16_t i = 0;

    while(i < size >> 4) {
        memcpy(data_temp, p_buff+(i<<4), 16);
        bt_mesh_aes_decrypt(g_enc_key, data_temp, dec_temp);
        BT_DBG("dec: %s", bt_hex(dec_temp, 16));
        memcpy(p_buff+(i<<4), dec_temp, 16);
        i++;
    }

    return 0;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_checkflag(hal_partition_t in_partition)
{
    int32_t ret = 0;
    uint32_t offset = 0;
    uint32_t flash_flag = 0;

    ret = hal_flash_read(in_partition, &offset, (uint8_t *)(&flash_flag), sizeof(flash_flag));
    BT_DBG("ret(%d) flash_flag(0x%08X)", ret, flash_flag);

    RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);

    if (flash_flag == GENIE_FLASH_FLAG_INITED_ORIGIN) {
        return GENIE_FLASH_DATA_ORIGIN;
    }

    if ((in_partition == GENIE_FLASH_PARTITION_SYSTEM && flash_flag != GENIE_FLASH_FLAG_INITED_SYS) ||
        (in_partition == GENIE_FLASH_PARTITION_USERDATA && flash_flag != GENIE_FLASH_FLAG_INITED_UD) ||
        (in_partition == GENIE_FLASH_PARTITION_SEQ && flash_flag != GENIE_FLASH_FLAG_INITED_SEQ)) {
        return GENIE_FLASH_DATA_INVALID;
    }

    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_get_header(genie_flash_cell_t *p_cell)
{
    int32_t ret = 0;

    //BT_DBG("");

    if(p_cell->offset < p_cell->end) {

        ret = hal_flash_read(p_cell->pno, &p_cell->offset, (uint8_t *)(&p_cell->header),
                                sizeof(flash_header_t));

        RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);
#if 0
        BT_DBG("offset(0x%04X) flag(0x%02X) align(%d) index(0x%04X) length(%d) crc(0x%04X)",
                p_cell->offset-sizeof(flash_header_t), p_cell->header.flag, p_cell->header.align,
                p_cell->header.index, p_cell->header.length, p_cell->header.crc);
#endif
        /* move offset to header*/
        p_cell->offset -= sizeof(flash_header_t);

        if(p_cell->header.flag != GENIE_FLASH_FLAG_UNSED) {
            return GENIE_FLASH_SUCCESS;
        }
    }
    return GENIE_FLASH_SEARCH_NONE;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_check_remain(void)
{
    E_GENIE_FLASH_ERRCODE ret;
    genie_flash_cell_t cell;

    /* system */
    g_info_system.remain = GENIE_FLASH_SIZE_RELIABLE;
    CELL_PREPEAR_RELIABLE(cell, 0);
    do {
        ret = _genie_flash_get_header(&cell);
        if(ret == GENIE_FLASH_SUCCESS) {
            if(cell.header.flag == GENIE_FLASH_FLAG_ACTIVE) {
                g_info_system.remain -= sizeof(flash_header_t) + cell.header.length + cell.header.align;
            }
            cell.offset += sizeof(flash_header_t) + cell.header.length + cell.header.align;
        }
    } while(ret == GENIE_FLASH_SUCCESS);
    if(g_info_system.remain > GENIE_FLASH_SIZE_RELIABLE) {
        g_info_system.remain = 0;
    }
    if(cell.offset <= GENIE_FLASH_SIZE_RELIABLE) {
        g_info_system.free_offset = cell.offset;
    }

    /* userdata */
    g_info_userdata.remain = GENIE_FLASH_SIZE_USERDATA;
    CELL_PREPEAR_USERDATA(cell, 0);
    do {
        ret = _genie_flash_get_header(&cell);
        if(ret == GENIE_FLASH_SUCCESS) {
            if(cell.header.flag == GENIE_FLASH_FLAG_ACTIVE) {
                g_info_userdata.remain -= sizeof(flash_header_t) + cell.header.length + cell.header.align;
            }
            cell.offset += sizeof(flash_header_t) + cell.header.length + cell.header.align;
        }
    } while(ret == GENIE_FLASH_SUCCESS);
    if(g_info_userdata.remain > GENIE_FLASH_SIZE_USERDATA) {
        g_info_userdata.remain = 0;
    }
    if(cell.offset <= GENIE_FLASH_SIZE_USERDATA) {
        g_info_userdata.free_offset = cell.offset;
    }

    g_info_recycle.remain = GENIE_FLASH_SIZE_RECYCLE;
    g_info_recycle.free_offset = GENIE_FLASH_START_RECYCLE;
    BT_DBG("remain sys(%d) ud(%d)", g_info_system.remain, g_info_userdata.remain);

    return GENIE_FLASH_SUCCESS;
}

void _ginie_flash_get_enc_key(uint8_t project_key[16])
{
    uint32_t offset = 100;
    uint8_t random[16];
    int32_t ret;

    BT_DBG("project key: %s", bt_hex(project_key, 16));

    memset(random, 0, 16);
    ret = hal_flash_read(HAL_PARTITION_CUSTOM_2, &offset, random, 16);
    BT_DBG("random: %s[%d]", bt_hex(random, 16), ret);


    bt_mesh_aes_encrypt(random, project_key, g_enc_key);
    BT_DBG("enc key: %s", bt_hex(g_enc_key, 16));
}

E_GENIE_FLASH_ERRCODE genie_flash_check_recycle(void)
{
    uint32_t part_offset = 0;
    uint32_t part_flag = 0;
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    genie_flash_cell_t recycle;
    genie_flash_cell_t cell;

    CELL_PREPEAR_RECYCLE(recycle);

    ret = hal_flash_read(GENIE_FLASH_PARTITION_RECYCLE, &part_offset, (uint8_t *)(&part_flag), sizeof(part_flag));
    BT_DBG("ret(%d) flash_flag(0x%08X)", ret, part_flag);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_RECYCLE_FAIL);

    if (part_flag == GENIE_FLASH_FLAG_INITED_SYS) {
        CELL_PREPEAR_RELIABLE(cell, 0);
    } else if (part_flag == GENIE_FLASH_FLAG_INITED_UD) {
        CELL_PREPEAR_USERDATA(cell, 0);
    }
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    else if (part_flag == GENIE_FLASH_FLAG_INITED_SEQ) {
        uint32_t data =0;
        uint32_t seq_write = GENIE_FLASH_SIZE_INITED_FLAG;

        /* Prepare Seq partition */
        CELL_PREPEAR_SEQ(cell);

        /* Read Seq value from Recycle */
        ret = hal_flash_read(recycle.pno, &seq_write, &data, sizeof(uint32_t));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);

        if ((data >> 24) != GENIE_FLASH_FLAG_ACTIVE) {
            genie_flash_init_partition(recycle.pno, recycle.end);
            return GENIE_FLASH_DATA_INVALID;
        }

        return _genie_flash_copy_seq(&recycle, &cell, data);
    }
#endif
    else {
        /* part_flag == GENIE_FLASH_FLAG_INITED_INVALID, means data in ud/sys have not been erased, wait for recycle,
           part_flag == GENIE_FLASH_FLAG_INITED_ORIGIN, means recycle have not data yet,
           part_flag == GENIE_FLASH_FLAG_INITED_BASE, means recycle have been done,
           else, not support
        */
        return GENIE_FLASH_SUCCESS;
    }

    /* restore data from recycle to using partition */
    return _genie_flash_copy(&recycle, &cell, NULL, 0);
}

E_GENIE_FLASH_ERRCODE genie_flash_init(void)
{
    static uint8_t flash_already_inited = 0;
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;

    if(flash_already_inited == 1) {
        return GENIE_FLASH_SUCCESS;
    }

    flash_already_inited = 1;

#ifdef PROJECT_SECRET_KEY
    char key_char[] = PROJECT_SECRET_KEY;
    uint8_t prj_key[16];

    memset(prj_key, 0, 16);
    stringtohex(key_char, prj_key, 16);

    _ginie_flash_get_enc_key(prj_key);
#endif

    /* check if any valid data in Recycle, if true ,recovery it */
    ret = genie_flash_check_recycle();
    RETURN_WHEN_ERR(ret, GENIE_FLASH_INIT_FAIL);

    /* check if dirty data in UD, if true ,erase it */
    ret = _genie_flash_checkflag(GENIE_FLASH_PARTITION_USERDATA);
    if (ret != GENIE_FLASH_SUCCESS) {
        ret = genie_flash_init_partition(GENIE_FLASH_PARTITION_USERDATA, GENIE_FLASH_SIZE_USERDATA_PART);
    }

    /* check if dirty data in SYS, if true ,erase it */
    ret = _genie_flash_checkflag(GENIE_FLASH_PARTITION_SYSTEM);
    if (ret != GENIE_FLASH_SUCCESS) {
        ret = genie_flash_init_partition(GENIE_FLASH_PARTITION_SYSTEM, GENIE_FLASH_SIZE_SYSTEM_PART);
    }
    RETURN_WHEN_ERR(ret, GENIE_FLASH_INIT_FAIL);

    /* check if dirty data in SEQ, if true ,erase it */
    ret = _genie_flash_checkflag(GENIE_FLASH_PARTITION_SEQ);
    if (ret != GENIE_FLASH_SUCCESS) {
        ret = genie_flash_init_partition(GENIE_FLASH_PARTITION_SEQ, GENIE_FLASH_SIZE_SEQ_PART);
    }
    RETURN_WHEN_ERR(ret, GENIE_FLASH_INIT_FAIL);
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
    ret = genie_flash_seq_init();
    RETURN_WHEN_ERR(ret, GENIE_FLASH_INIT_FAIL); 
#endif

    ret = _genie_flash_check_remain();

    return ret;
}

#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
static E_GENIE_FLASH_ERRCODE _genie_flash_search(genie_flash_cell_t *p_cell)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;

    BT_DBG("search index(0x%04X)", p_cell->index);

    static uint32_t read_buf[4096 / 4] = {0};
    uint32_t offset = 0;
    flash_header_t *flash_head = NULL;

    if (p_cell->offset >= p_cell->end) {
        return GENIE_FLASH_SEARCH_NONE;
    }

    if (hal_flash_read(p_cell->pno, &offset, read_buf, sizeof(read_buf))) {
        return GENIE_FLASH_SEARCH_NONE;
    }

    flash_head = (flash_header_t *)((uint8_t *)read_buf + p_cell->offset);
    do {
        if (flash_head->flag == GENIE_FLASH_FLAG_UNSED) {
            return GENIE_FLASH_SEARCH_NONE;
        }

        if (flash_head->flag == GENIE_FLASH_FLAG_ACTIVE && flash_head->index == p_cell->index) {
            BT_DBG("bingo");
            p_cell->header = *flash_head; // copy header
            return GENIE_FLASH_SUCCESS;
        } else {
            /* found next cell */
            uint32_t len = sizeof(flash_header_t) + flash_head->length + flash_head->align;
            p_cell->offset += len;
            flash_head = (flash_header_t *)((uint8_t *)flash_head + len);
        }
    } while (p_cell->offset < p_cell->end);

    BT_DBG("can not find index(0x%04X), ret(%d)", p_cell->index, ret);
    return ret;
}
#else
static E_GENIE_FLASH_ERRCODE _genie_flash_search(genie_flash_cell_t *p_cell)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;

    BT_DBG("search index(0x%04X)", p_cell->index);

    do {
        ret = _genie_flash_get_header(p_cell);

        if(p_cell->header.flag == GENIE_FLASH_FLAG_ACTIVE && p_cell->header.index == p_cell->index) {
            BT_DBG("bingo");
            return GENIE_FLASH_SUCCESS;
        } else {
            /* found next cell */
            p_cell->offset += sizeof(flash_header_t) + p_cell->header.length + p_cell->header.align;
        }
    } while(ret == GENIE_FLASH_SUCCESS);

    BT_DBG("can not find index(0x%04X), ret(%d)", p_cell->index, ret);
    return ret;
}
#endif

static E_GENIE_FLASH_ERRCODE _genie_flash_read(genie_flash_cell_t *p_cell, uint8_t *p_buff, uint16_t size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;

    ret = _genie_flash_search(p_cell);
    RETURN_WHEN_ERR(ret, ret);

    if(p_cell->header.length < size || size == 0) {
        size = p_cell->header.length;
    }

    /* move offset */
    p_cell->offset += sizeof(flash_header_t);

    ret = hal_flash_read(p_cell->pno, &p_cell->offset, p_buff, size);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);

    if(_genie_get_crc(p_buff, size) == p_cell->header.crc) {
        return GENIE_FLASH_SUCCESS;
    } else {
        BT_ERR("check crc fail(%x)", p_cell->header.crc);
        return GENIE_FLASH_CHECK_CRC_FAIL;
    }
}

static E_GENIE_FLASH_ERRCODE _genie_flash_copy(genie_flash_cell_t *p_src, genie_flash_cell_t *p_dst, uint8_t *p_buff, uint16_t buf_size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint8_t *p_data;
    uint32_t flag_offset = 0xFFFFFFFF;
    uint16_t size = 0x1000;
    uint32_t part_offset = 0;
    uint32_t part_flag = 0;
    flash_header_t new_data_head = {0};

    /* step 1, erase destination */
    BT_DBG("1.1 erase pno(%d)", p_dst->pno);
    if (p_dst->pno == GENIE_FLASH_PARTITION_SYSTEM ||
        p_dst->pno == GENIE_FLASH_PARTITION_USERDATA ||
        /* write AFAFA8FF to head for recycling flags to avoid powerdown unexpected */
        p_dst->pno == GENIE_FLASH_PARTITION_RECYCLE) {
        genie_flash_init_partition(p_dst->pno, p_dst->end);
        p_dst->offset = 4;
    } else {
        hal_flash_erase(p_dst->pno, 0, size);
        p_dst->offset = 0;
    }

    if (p_src->pno == GENIE_FLASH_PARTITION_SYSTEM ||
        p_src->pno == GENIE_FLASH_PARTITION_USERDATA ||
        p_src->pno == GENIE_FLASH_PARTITION_RECYCLE) {
        p_src->offset = 4;
    } else {
        p_src->offset = 0;
    }

    BT_DBG("1.2 src(0x%04X) dst(0x%04X)", p_src->offset, p_dst->offset);

    /* Save new data to recycle */
    if (p_dst->pno == GENIE_FLASH_PARTITION_RECYCLE && p_buff != NULL && buf_size !=0 ) {
        uint8_t align_count = 0;
        if(ALIGN_ERROR(buf_size)) {
            align_count = 4 - (buf_size & 0x0003);
        }
        new_data_head.flag = GENIE_FLASH_FLAG_ACTIVE;
        new_data_head.align = align_count;
        new_data_head.index = p_src->index;
        new_data_head.length = buf_size;
        new_data_head.crc = _genie_get_crc(p_buff, buf_size);

        ret = hal_flash_write(p_dst->pno, &p_dst->offset, &new_data_head, sizeof(flash_header_t));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

        ret = hal_flash_write(p_dst->pno, &p_dst->offset, p_buff, buf_size);
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
        p_dst->offset += new_data_head.align;
    }

    /* step 2, move old data to destination */
    do {
        ret = _genie_flash_get_header(p_src);

        if (ret == GENIE_FLASH_SUCCESS &&
            p_src->header.flag == GENIE_FLASH_FLAG_ACTIVE &&
            /* skip data if data index is duplicated */
            p_src->index != p_src->header.index) {

            /* step 2.1 write header */
            BT_DBG("2.1 src(0x%04X) dst(0x%04X)", p_src->offset, p_dst->offset);
            ret = hal_flash_write(p_dst->pno, &p_dst->offset, &p_src->header, sizeof(flash_header_t));
            RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

            /* step 2.2 read data */
            BT_DBG("2.2 read data");
            p_src->offset += sizeof(flash_header_t);
            p_data = k_malloc(p_src->header.length);
            if (p_data == NULL) {
                BT_ERR("p_data is null!!!");
                return GENIE_FLASH_DATA_INVALID;
            }
            ret = hal_flash_read(p_src->pno, &p_src->offset, p_data, p_src->header.length);

            /* step 2.3 write data */
            BT_DBG("2.3 write data(0x%08X) dst(0x%04X)", p_data, p_dst->offset);
            ret = hal_flash_write(p_dst->pno, &p_dst->offset, p_data, p_src->header.length);
            k_free(p_data);
            RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

            p_src->offset += p_src->header.align;
            p_dst->offset += p_src->header.align;
        } else {
            p_src->offset += sizeof(flash_header_t) + p_src->header.length + p_src->header.align;
        }
    } while (ret == GENIE_FLASH_SUCCESS);

    /* write partition flags to recycle to avoid powerdown unexpected */
    if (p_dst->pno == GENIE_FLASH_PARTITION_RECYCLE) {
        part_offset = 0;
        ret = hal_flash_read(p_src->pno, &part_offset, (uint8_t *)(&part_flag), sizeof(part_flag));
        BT_DBG("ret(%d) flash_flag(0x%08X)", ret, part_flag);
        RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);

        part_offset = 0;
        ret = hal_flash_write(p_dst->pno, &part_offset, &part_flag, sizeof(part_flag));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
    }

    if (p_src->pno == GENIE_FLASH_PARTITION_RECYCLE) {
        /* Here, means data recovery finished , so erase recycle flag only */
        part_offset = 0;
        part_flag = GENIE_FLASH_FLAG_INITED_BASE;
        ret = hal_flash_write(p_src->pno, &part_offset, &part_flag, sizeof(part_flag));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
    }
    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_recycle(genie_flash_cell_t *p_cell, uint8_t *p_buff, uint16_t size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    genie_flash_cell_t recycle;

    CELL_PREPEAR_RECYCLE(recycle);

    /* backup new data and valid data to recycle */
    ret = _genie_flash_copy(p_cell, &recycle, p_buff, size);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_RECYCLE_FAIL);

    /* restore data from recycle to using partition */
    ret = _genie_flash_copy(&recycle, p_cell, NULL, 0);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_RECYCLE_FAIL);

    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_delete_flag(hal_partition_t pno, uint32_t *p_offset)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint8_t flag = GENIE_FLASH_FLAG_INVALID;

    BT_DBG("pno(%d) offset(0x%04X)", pno, *p_offset);
    ret = hal_flash_write(pno, p_offset, &flag, sizeof(flag));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_delete(genie_flash_cell_t *p_cell)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint32_t flag_offset;

    ret = _genie_flash_search(p_cell);
    RETURN_WHEN_ERR(ret, ret);

    flag_offset = (uint32_t)&p_cell->header.flag - (uint32_t)&p_cell->header + p_cell->offset;
    if(flag_offset < p_cell->end) {
        return _genie_flash_delete_flag(p_cell->pno, &flag_offset);
    } else {
        BT_ERR("flag_offset error");
        return GENIE_FLASH_DELETE_FAIL;
    }
}

static E_GENIE_FLASH_ERRCODE _genie_flash_write(genie_flash_cell_t *p_cell, uint8_t *p_buff, uint16_t size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint8_t align_count = 0;
    genie_flash_info_t *p_info;
    uint32_t old_flag_offset = 0xFFFFFFFF;
    uint32_t new_flag_offset = 0xFFFFFFFF;
    uint16_t old_size = 0;
    uint16_t end;
    genie_flash_cell_t recycle;

    if(size == 0) {
        BT_ERR("size is zero!!!");
        return GENIE_FLASH_WRITE_FAIL;
    }

    if(p_buff == NULL) {
        BT_ERR("p_buff is null!!!");
        return GENIE_FLASH_DATA_INVALID;
    }

    /* get info */
    switch(p_cell->pno) {
        case GENIE_FLASH_PARTITION_SYSTEM:
            p_info = &g_info_system;
            end = GENIE_FLASH_SIZE_INITED_FLAG + GENIE_FLASH_SIZE_RELIABLE;
            break;
        case GENIE_FLASH_PARTITION_USERDATA:
            p_info = &g_info_userdata;
            end = GENIE_FLASH_SIZE_INITED_FLAG + GENIE_FLASH_SIZE_USERDATA;
            break;
        default:
            BT_ERR("pno error!!!");
            return GENIE_FLASH_WRITE_FAIL;
    }

    /* check size */
    if(p_info->remain < size + sizeof(flash_header_t)) {
        BT_ERR("no space");
        return GENIE_FLASH_WRITE_FAIL;
    }

    /* check recycle */
    if(p_info->free_offset + size + sizeof(flash_header_t) > end) {
        memcpy(&recycle, p_cell, sizeof(genie_flash_cell_t));
        ret = _genie_flash_recycle(&recycle, p_buff, size);
        RETURN_WHEN_ERR(ret, ret);
        _genie_flash_check_remain();
        return GENIE_FLASH_SUCCESS;
    }

    /* find old */
    if(_genie_flash_search(p_cell) == GENIE_FLASH_SUCCESS) {
        old_flag_offset = (uint32_t)&p_cell->header.flag - (uint32_t)&p_cell->header + p_cell->offset;
        old_size = sizeof(flash_header_t) +p_cell->header.length + p_cell->header.align;
    }

    //step 1 write new data
    p_cell->offset = new_flag_offset = p_info->free_offset;
    BT_DBG("new_flag_offset(0x%04X) cell.offset(0x%04X)", new_flag_offset, p_cell->offset);

    if(ALIGN_ERROR(size)) {
        align_count = 4 - (size & 0x0003);
    }
    p_cell->header.flag = GENIE_FLASH_FLAG_WRITING;
    p_cell->header.align = align_count;
    p_cell->header.index = p_cell->index;
    p_cell->header.length = size;
    p_cell->header.crc = _genie_get_crc(p_buff, size);

    BT_DBG("step1.1 write header offset(0x%04X)", p_cell->offset);
    ret = hal_flash_write(p_cell->pno, &p_cell->offset, &p_cell->header, sizeof(flash_header_t));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

    BT_DBG("step1.2 write data offset(0x%04X)", p_cell->offset);
    ret = hal_flash_write(p_cell->pno, &p_cell->offset, p_buff, size);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

    //step 2 update new flag
    p_cell->header.flag = GENIE_FLASH_FLAG_ACTIVE;
    BT_DBG("step2 update flag offset(0x%04X)", new_flag_offset);
    ret = hal_flash_write(p_cell->pno, &new_flag_offset, &p_cell->header.flag, 1);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

    /* update remain & free*/
    p_info->remain -= size + sizeof(flash_header_t) + p_cell->header.align;
    p_info->free_offset = p_cell->offset + p_cell->header.align;

    //modify old flag
    if(old_flag_offset != 0xFFFFFFFF) {
        BT_DBG("old offset(0x%04X)", old_flag_offset);
        ret = _genie_flash_delete_flag(p_cell->pno, &old_flag_offset);
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
        p_info->remain += old_size;
    }

    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE genie_flash_init_partition(hal_partition_t in_partition, uint32_t part_size)
{
    int32_t ret = 0;
    uint32_t offset = 0;
    uint32_t flash_flag = 0;

    BT_DBG("");
    switch (in_partition) {
    case GENIE_FLASH_PARTITION_SYSTEM:
        flash_flag = GENIE_FLASH_FLAG_INITED_SYS;
        g_info_system.remain = GENIE_FLASH_SIZE_RELIABLE;
        g_info_system.free_offset = GENIE_FLASH_SIZE_INITED_FLAG;
        break;
    case GENIE_FLASH_PARTITION_USERDATA:
        flash_flag = GENIE_FLASH_FLAG_INITED_UD;
        g_info_userdata.remain = GENIE_FLASH_SIZE_USERDATA;
        g_info_userdata.free_offset = GENIE_FLASH_SIZE_INITED_FLAG;
        break;
    case GENIE_FLASH_PARTITION_RECYCLE:
        flash_flag = GENIE_FLASH_FLAG_INITED_INVALID;
        g_info_recycle.remain = GENIE_FLASH_SIZE_RECYCLE;
        g_info_recycle.free_offset = GENIE_FLASH_START_RECYCLE;
        break;
    case GENIE_FLASH_PARTITION_SEQ:
#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
        flash_flag = GENIE_FLASH_FLAG_INITED_SEQ;
        g_flash_seq_ctl.write_offset = GENIE_FLASH_SIZE_INITED_FLAG;
        g_flash_seq_ctl.read_offset = 0;
        break;
#else
        return hal_flash_erase(in_partition, 0, part_size);
#endif
    default:
        return GENIE_FLASH_EARSE_FAIL;
    }

    ret = hal_flash_erase(in_partition, 0, part_size);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_EARSE_FAIL);

    ret = hal_flash_write(in_partition, &offset, &flash_flag, sizeof(flash_flag));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

    return GENIE_FLASH_SUCCESS;
}

static uint16_t _genie_flash_get_reliable_size(uint16_t data_size)
{
    if((data_size & 0x0F) == 0){
        return data_size;
    } else {
        return ((data_size>>4)+1)<<4;
    }
}

E_GENIE_FLASH_ERRCODE genie_flash_read_reliable(uint16_t index, uint8_t *p_data, uint16_t data_size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint16_t buff_size = _genie_flash_get_reliable_size(data_size);
    uint8_t *p_buff = NULL;
    genie_flash_cell_t cell;

    if(p_data == NULL) {
        BT_ERR("p_data is null!!!");
        return GENIE_FLASH_DATA_INVALID;
    }

    CELL_PREPEAR_RELIABLE(cell, index);
    p_buff = k_malloc(buff_size);
    if(p_buff == NULL) {
        BT_ERR("p_buff is null!!!");
        return GENIE_FLASH_DATA_INVALID;
    }

    ret = _genie_flash_read(&cell, p_buff, buff_size);
    BT_DBG("read ret %d", ret);
    if(ret == 0) {
        _genie_flash_decrypt(p_buff, buff_size);
        memcpy(p_data, p_buff, data_size);
    }
    k_free(p_buff);

    return ret;
}

E_GENIE_FLASH_ERRCODE genie_flash_write_reliable(uint16_t index, uint8_t *p_data, uint16_t data_size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint16_t buff_size = _genie_flash_get_reliable_size(data_size);
    uint8_t *p_buff = NULL;
    genie_flash_cell_t cell;

    if(p_data == NULL) {
        BT_ERR("p_data is null!!!");
        return GENIE_FLASH_DATA_INVALID;
    }

    CELL_PREPEAR_RELIABLE(cell, index);
    p_buff = k_malloc(buff_size);
    if(p_buff == NULL) {
        BT_ERR("p_buff is null!!!");
        return GENIE_FLASH_DATA_INVALID;
    }

    memset(p_buff, 0, buff_size);
    memcpy(p_buff, p_data, data_size);
    BT_DBG("data:%d buff:%d\n", data_size, buff_size);
    _genie_flash_encrypt(p_buff, buff_size);

    ret = _genie_flash_write(&cell, p_buff, buff_size);
    k_free(p_buff);

    return ret;
}

E_GENIE_FLASH_ERRCODE genie_flash_delete_reliable(uint16_t index)
{
    genie_flash_cell_t cell;

    CELL_PREPEAR_RELIABLE(cell, index);

    return _genie_flash_delete(&cell);
}

E_GENIE_FLASH_ERRCODE genie_flash_erase_reliable(void)
{
    BT_DBG("");
    return genie_flash_init_partition(GENIE_FLASH_PARTITION_SYSTEM, GENIE_FLASH_SIZE_SYSTEM_PART);
}

E_GENIE_FLASH_ERRCODE genie_flash_read_userdata(uint16_t index, uint8_t *p_buff, uint16_t size)
{
    genie_flash_cell_t cell;

    CELL_PREPEAR_USERDATA(cell, index);

    return _genie_flash_read(&cell, p_buff, size);
}

E_GENIE_FLASH_ERRCODE genie_flash_write_userdata(uint16_t index, uint8_t *p_buff, uint16_t size)
{
    genie_flash_cell_t cell;

    CELL_PREPEAR_USERDATA(cell, index);

    return _genie_flash_write(&cell, p_buff, size);
}

E_GENIE_FLASH_ERRCODE genie_flash_delete_userdata(uint16_t index)
{
    genie_flash_cell_t cell;

    CELL_PREPEAR_USERDATA(cell, index);

    return _genie_flash_delete(&cell);
}

E_GENIE_FLASH_ERRCODE genie_flash_erase_userdata(void)
{
    return genie_flash_init_partition(GENIE_FLASH_PARTITION_USERDATA, GENIE_FLASH_SIZE_USERDATA);
}

#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
E_GENIE_FLASH_ERRCODE genie_flash_seq_init()
{
    uint32_t offset = GENIE_FLASH_SIZE_INITED_FLAG, p_data;
    int ret = 0;

    if(g_flash_seq_ctl.init_flag) {
        return GENIE_FLASH_SUCCESS;
    }

    while (offset < GENIE_FLASH_SIZE_SEQ){
        ret = hal_flash_read(GENIE_FLASH_PARTITION_SEQ, &offset, &p_data, sizeof(uint32_t));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);
        if(p_data >> 24 == 0XFF){
            break;
        }
    }
    g_flash_seq_ctl.write_offset = offset - sizeof(uint32_t);
    g_flash_seq_ctl.read_offset = g_flash_seq_ctl.write_offset - sizeof(uint32_t);
    g_flash_seq_ctl.init_flag =1;

    BT_DBG("w_o:%d r_o:%d\n", g_flash_seq_ctl.write_offset, g_flash_seq_ctl.read_offset);
    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_copy_seq(genie_flash_cell_t *p_src, genie_flash_cell_t *p_dst, uint32_t seq)
{
    uint32_t part_offset = 0;
    uint32_t part_flag = 0;
    uint32_t flag_offset = GENIE_FLASH_SIZE_INITED_FLAG;
    int ret = -1;
    uint32_t data =0;

    /* erase destination */
    BT_DBG("erase pno(%d)", p_dst->pno);
    genie_flash_init_partition(p_dst->pno, p_dst->end);
    p_dst->offset = GENIE_FLASH_SIZE_INITED_FLAG;

    BT_DBG("data:%x", seq);

    /* Set writing flag to avoid powerdowm case */
    flag_offset = GENIE_FLASH_SIZE_INITED_FLAG;
    data = (seq & 0x00FFFFFF) | (GENIE_FLASH_FLAG_ACTIVE << 24);
    ret = hal_flash_write(p_dst->pno, &flag_offset, &data, sizeof(uint32_t));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

    /* write partition flags to recycle to avoid powerdown unexpected */
    if (p_dst->pno == GENIE_FLASH_PARTITION_RECYCLE) {
        part_flag = GENIE_FLASH_FLAG_INITED_SEQ;
        part_offset = 0;
        ret = hal_flash_write(p_dst->pno, &part_offset, &part_flag, sizeof(part_flag));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
    }

    if (p_src->pno == GENIE_FLASH_PARTITION_RECYCLE) {
        /* Here, means data recovery finished , so erase recycle flag only */
        part_offset = 0;
        part_flag = GENIE_FLASH_FLAG_INITED_BASE;
        ret = hal_flash_write(p_src->pno, &part_offset, &part_flag, sizeof(part_flag));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
    }
    return GENIE_FLASH_SUCCESS;
}

E_GENIE_FLASH_ERRCODE genie_flash_read_seq(uint32_t *p_seq)
{
    int ret = -1;
    uint32_t seq_read;
    if (g_flash_seq_ctl.init_flag == 0) {
        return -1;
    }

    seq_read = g_flash_seq_ctl.read_offset;
    BT_DBG("r_o:%x", seq_read);

    if (seq_read < GENIE_FLASH_SIZE_INITED_FLAG) {
        return GENIE_FLASH_DATA_INVALID;
    }

    ret = hal_flash_read(GENIE_FLASH_PARTITION_SEQ, &seq_read, p_seq, sizeof(uint32_t));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);

    if (((*p_seq) >> 24) != GENIE_FLASH_FLAG_ACTIVE) {
        return GENIE_FLASH_DATA_INVALID;
    }

    *p_seq = *p_seq & 0x00FFFFFF;
    BT_DBG("seq:%x", *p_seq);

    return ret;
}

E_GENIE_FLASH_ERRCODE genie_flash_write_seq(uint32_t *p_seq)
{
    int ret = -1;
    uint32_t data =0;
    uint32_t seq_write =0;
    if(g_flash_seq_ctl.init_flag == 0){
        return -1;
    }

    data = *p_seq;
    BT_DBG("w_o:%x", g_flash_seq_ctl.write_offset);

    if ((g_flash_seq_ctl.write_offset + sizeof(uint32_t)) > GENIE_FLASH_SIZE_SEQ) {
        // Start to recycle
        genie_flash_cell_t recycle;
        genie_flash_cell_t seq;

        CELL_PREPEAR_RECYCLE(recycle);
        CELL_PREPEAR_SEQ(seq);

        //recycle seq
        ret = _genie_flash_copy_seq(&seq, &recycle, data);
        RETURN_WHEN_ERR(ret, GENIE_FLASH_RECYCLE_FAIL);

        //recovery seq
        ret = _genie_flash_copy_seq(&recycle, &seq, data);
        RETURN_WHEN_ERR(ret, GENIE_FLASH_RECYCLE_FAIL);
        return GENIE_FLASH_SUCCESS;
    }

    seq_write = g_flash_seq_ctl.write_offset;

    /* Set writing flag to avoid powerdowm case */
    data = (data & 0x00FFFFFF) | (GENIE_FLASH_FLAG_ACTIVE << 24);
    ret = hal_flash_write(GENIE_FLASH_PARTITION_SEQ, &seq_write, &data, sizeof(uint32_t));
    if(ret == GENIE_FLASH_SUCCESS) {
        g_flash_seq_ctl.read_offset = g_flash_seq_ctl.write_offset;
        g_flash_seq_ctl.write_offset = seq_write;
        BT_DBG("seq:%x", data);
    }

    return ret;
}

#else
static E_GENIE_FLASH_ERRCODE _genie_flash_read_seqbase(uint32_t *p_base)
{
    int32_t ret = 0;
    uint32_t offset = GENIE_FLASH_START_SEQ_BASE;
    uint8_t flag;

    BT_DBG("");

    ret = hal_flash_read(GENIE_FLASH_PARTITION_SEQ, &offset, p_base, sizeof(uint32_t));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);

    BT_DBG("read raw base(0x%02X)", *p_base);
    flag = *p_base >> 24;

    if(flag != GENIE_FLASH_FLAG_ACTIVE) {
        BT_DBG("base seq is invalid(0x%02X)", flag);
        return GENIE_FLASH_DATA_INVALID;
    }

    *p_base &= 0x00FFFFFF;
    return GENIE_FLASH_SUCCESS;
}


static E_GENIE_FLASH_ERRCODE _genie_flash_read_seqcount(uint16_t *p_count)
{
    int32_t ret = 0;
    uint32_t offset = GENIE_FLASH_START_SEQ_COUNT;
    uint8_t data[SEQ_COUNT_BUFFER_SIZE];
    uint8_t i, j;

    *p_count = 0;
    while(offset < GENIE_FLASH_START_SEQ_COUNT + GENIE_FLASH_SIZE_SEQ) {
        ret = hal_flash_read(GENIE_FLASH_PARTITION_SEQ, &offset, data, SEQ_COUNT_BUFFER_SIZE);
        RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);
        for(i = 0; i < SEQ_COUNT_BUFFER_SIZE; i++) {
            //BT_DBG("read byte(0x%02X)", data[i])
            if(data[i] != 0) {
                //do sth then break
                for(j = 0; j < 8; j++) {
                    //BT_DBG("0x%02X 0x%02X 0x%02X", data[i], 0x01<<j, (data[i] & (0x01<<j)))
                    if((data[i] & (0x01<<j)) == 0) {
                        (*p_count)++;
                    }
                }
                //BT_DBG("get p_count(0x%02X)", *p_count)
                return GENIE_FLASH_SUCCESS;
            } else {
                *p_count += 8;
            }
        }
    }
    return GENIE_FLASH_SUCCESS;
}

E_GENIE_FLASH_ERRCODE genie_flash_read_seq(uint32_t *p_seq)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint32_t base;
    uint16_t count;

    BT_DBG("");

    ret = _genie_flash_read_seqbase(&base);
    RETURN_WHEN_ERR(ret, ret);
    BT_DBG("read base(0x%02X)", base);

    ret = _genie_flash_read_seqcount(&count);
    RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);
    BT_DBG("read count(0x%02X)", count);

    *p_seq = base+count;
    return GENIE_FLASH_SUCCESS;
}

static E_GENIE_FLASH_ERRCODE _genie_flash_write_seqbase(uint32_t base)
{
    int32_t ret = 0;
    uint32_t offset = GENIE_FLASH_START_SEQ_BASE;

    base |= GENIE_FLASH_FLAG_ACTIVE<<24;
    BT_DBG("base(0x%04X)", base);

    //_genie_flash_delete_seq();

    ret = hal_flash_write(GENIE_FLASH_PARTITION_SEQ, &offset, &base, sizeof(base));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);

    return GENIE_FLASH_SUCCESS;
}


static E_GENIE_FLASH_ERRCODE _genie_flash_write_seqcount(uint16_t count)
{
    int32_t ret = 0;
    uint32_t offset = GENIE_FLASH_START_SEQ_COUNT + (count>>3);
    uint8_t write_byte = count % 8, zero = 0;
    uint32_t start = GENIE_FLASH_START_SEQ_COUNT;

    if(count && write_byte == 0) {
        offset -= 1;
    } else {
        write_byte = 0xFF >> write_byte;
    }

    BT_DBG("offset(0x%04X) byte(0x%02X)", offset, write_byte);

    //TODO
    //clear all bytes before offset
    while(start < offset) {
        ret = hal_flash_read(GENIE_FLASH_PARTITION_SEQ, &start, &zero, sizeof(zero));
        RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
        if(zero != 0) {
            zero = 0;
            start--;
            ret = hal_flash_write(GENIE_FLASH_PARTITION_SEQ, &start, &zero, sizeof(zero));
            RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
        }
    }

    ret = hal_flash_write(GENIE_FLASH_PARTITION_SEQ, &offset, &write_byte, sizeof(write_byte));
    RETURN_WHEN_ERR(ret, GENIE_FLASH_WRITE_FAIL);
    return GENIE_FLASH_SUCCESS;
}

E_GENIE_FLASH_ERRCODE genie_flash_write_seq(uint32_t *p_seq)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint32_t base = 0;
    uint16_t count = 0;
    uint32_t seq = *p_seq;

    BT_DBG("%s %d", __func__, seq);

    ret = _genie_flash_read_seqbase(&base);

    if(ret == GENIE_FLASH_DATA_INVALID) {
        if(base != 0xFFFFFFFF) {
            goto CLEAN;
        }
        goto WRITE_BASE;
    }

    if(ret == GENIE_FLASH_SUCCESS) {
        if(base >= seq) {
            BT_ERR("base(%d) > seq(%d)", base, seq);
            return GENIE_FLASH_WRITE_FAIL;
        } else {
            if(seq-base <= (GENIE_FLASH_SIZE_SEQ<<3)) {
                //write new count
                ret = _genie_flash_read_seqcount(&count);
                RETURN_WHEN_ERR(ret, GENIE_FLASH_READ_FAIL);
                BT_DBG("read count(%d)", count);

                if(count < seq - base) {
                    count = seq - base;
                    return _genie_flash_write_seqcount(count);
                } else {
                    BT_ERR("base(%d)+count(%d) >= seq(%d)", base, count, seq);
                    return GENIE_FLASH_WRITE_FAIL;
                }
            } else {
                goto CLEAN;
            }
        }
    } else {
        BT_ERR("flash error");
        return GENIE_FLASH_WRITE_FAIL;
    }

CLEAN:
    BT_DBG("del seq\n");
    genie_flash_delete_seq();
WRITE_BASE:
    return _genie_flash_write_seqbase(seq);
}

#endif
E_GENIE_FLASH_ERRCODE genie_flash_delete_seq(void)
{
    return genie_flash_init_partition(GENIE_FLASH_PARTITION_SEQ, GENIE_FLASH_SIZE_SEQ_PART);
}

#if defined(CONFIG_GENIE_DEBUG_CMD)
extern const hal_logic_partition_t hal_partitions[];
static void _genie_print_data(hal_partition_t pno)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    genie_flash_cell_t cell;

    if(pno == GENIE_FLASH_PARTITION_SYSTEM) {
        CELL_PREPEAR_RELIABLE(cell, 0);
    } else {
        CELL_PREPEAR_USERDATA(cell, 0);
    }

    printk(F_RED "offset\t flag\t align\t index\t length\t\t crc\n" F_END);
    do {
        ret = _genie_flash_get_header(&cell);

        if(ret != GENIE_FLASH_SEARCH_NONE) {
            if(cell.header.flag == GENIE_FLASH_FLAG_ACTIVE) {
                printk("%04X\t %02X\t %d\t %04X\t %04X|%04d\t %04X\n",
                    cell.offset, cell.header.flag, cell.header.align, cell.header.index, cell.header.length,
                    cell.header.length, cell.header.crc);
                cell.offset += sizeof(flash_header_t) + cell.header.length + cell.header.align;
            } else {
                printk(F_RED "%04X\t %02X\t %d\t %04X\t %04X|%04d\t %04X" F_END "\n",
                    cell.offset, cell.header.flag, cell.header.align, cell.header.index, cell.header.length,
                    cell.header.length, cell.header.crc);
                cell.offset += sizeof(flash_header_t) + cell.header.length + cell.header.align;
            }
        }
    } while(ret == GENIE_FLASH_SUCCESS);
}

static E_GENIE_FLASH_ERRCODE _genie_flash_write_data(hal_partition_t pno, uint16_t size)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint8_t *p_data = k_malloc(size);

    if(p_data == NULL) {
        return GENIE_FLASH_MALLOC_FAIL;
    }

    memset(p_data, 0xA5, size);
    BT_DBG("set data(0x%08X) size(%d)\n", p_data, size);
    if(pno == GENIE_FLASH_PARTITION_SYSTEM) {
        ret = genie_flash_write_reliable(GFI_MESH_PARA, p_data, size);
    } else {
        ret = genie_flash_write_userdata(GFI_MESH_POWERUP, p_data, size);
    }
    aos_free(p_data);
    return ret;
}

static void _genie_flash_dump(hal_partition_t pno, uint16_t size)
{
    uint32_t offset = 0;
    uint16_t i = 0;
    uint8_t data[16];

    if(size == 0) {
        size = 0x1000;
    }

    printk(F_RED "dump(%d)(%d)(%p)\n" F_END, pno, size, hal_partitions[pno].partition_start_addr);

    offset = 0;
    while(offset < size) {
        if(hal_flash_read(pno, &offset, data, 16))
            break;
        printf("[%04X]", offset-16);
        for(i = 0; i < 16; i++)
            printf(" %02X", data[i]);
        printf("\n");
    }
}

void cmd_handle_flash_sys(char *pwbuf, int blen, int argc, char **argv)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint32_t tmp = 0;

    if(!strcmp(argv[1], "get")) {
        _genie_print_data(GENIE_FLASH_PARTITION_SYSTEM);
    } else if(!strcmp(argv[1], "set")) {
        tmp = atol(argv[2]);
        ret = _genie_flash_write_data(GENIE_FLASH_PARTITION_SYSTEM, tmp);
        BT_INFO("set count(%d), ret(%d)", tmp, ret);
    } else if(!strcmp(argv[1], "del")) {
        tmp = atol(argv[2]);
        ret = genie_flash_delete_reliable(tmp);
        BT_INFO("del index(%d), ret(%d)", tmp, ret);
    } else if(!strcmp(argv[1], "dump")) {
        _genie_flash_dump(GENIE_FLASH_PARTITION_SYSTEM, atol(argv[2]));
    } else if(!strcmp(argv[1], "rst")) {
        BT_INFO("delete mesh");
        genie_flash_reset_system();
        aos_reboot();
    } else {
        BT_INFO("sys cmd error");
    }
}

void cmd_handle_flash_ud(char *pwbuf, int blen, int argc, char **argv)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint32_t tmp = 0;

    if(!strcmp(argv[1], "get")) {
        _genie_print_data(GENIE_FLASH_PARTITION_USERDATA);
    } else if(!strcmp(argv[1], "set")) {
        tmp = atol(argv[2]);
        ret = _genie_flash_write_data(GENIE_FLASH_PARTITION_USERDATA, tmp);
        BT_INFO("set count(%d), ret(%d)", tmp, ret);
    } else if(!strcmp(argv[1], "del")) {
        tmp = atol(argv[2]);
        ret = genie_flash_delete_userdata(tmp);
        BT_INFO("del index(%d), ret(%d)", tmp, ret);
    } else if(!strcmp(argv[1], "dump")) {
        _genie_flash_dump(GENIE_FLASH_PARTITION_USERDATA, atol(argv[2]));
    } else if(!strcmp(argv[1], "rst")) {
        BT_INFO("erase userdata");
        genie_flash_erase_userdata();
    } else {
        BT_INFO("ud cmd error");
    }
}

void cmd_handle_flash_seq(char *pwbuf, int blen, int argc, char **argv)
{
    E_GENIE_FLASH_ERRCODE ret = GENIE_FLASH_SUCCESS;
    uint32_t seq = 0;

    if(!strcmp(argv[1], "get")) {
        ret = genie_flash_read_seq(&seq);
        BT_INFO("get seq (%x), ret(%d)", seq, ret);
    } else if(!strcmp(argv[1], "set")) {
        seq = atol(argv[2]);
        BT_INFO("set seq (%x), ret(%d)", seq, genie_flash_write_seq(&seq));
    } else if(!strcmp(argv[1], "del")) {
        BT_INFO("del seq, ret(%d)", genie_flash_delete_seq());
    } else if(!strcmp(argv[1], "dump")) {
        _genie_flash_dump(GENIE_FLASH_PARTITION_SEQ, atol(argv[2]));
    } else {
        BT_INFO("seq cmd error");
    }
}

#endif

