/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include "genie_mesh_flash.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_FLASH)
#include "common/log.h"

#if(BT_DBG_ENABLED == 1)
#define RETURN_WHEN_ERR(result, err_code) { if(result != 0) { BT_ERR("%s[%d] error ret(%d)", __func__, __LINE__, result); return err_code; } }
#else
#define RETURN_WHEN_ERR(result, err_code) { if(result != 0) return err_code; }
#endif

E_GENIE_FLASH_ERRCODE genie_flash_write_sub(uint16_t *p_sub)
{
    return genie_flash_write_reliable(GFI_MESH_SUB, (uint8_t *)p_sub, CONFIG_BT_MESH_MODEL_GROUP_COUNT<<1);
}

E_GENIE_FLASH_ERRCODE genie_flash_read_sub(uint16_t *p_sub)
{
    return genie_flash_read_reliable(GFI_MESH_SUB, (uint8_t *)p_sub, CONFIG_BT_MESH_MODEL_GROUP_COUNT<<1);
}

E_GENIE_FLASH_ERRCODE genie_flash_write_para(struct bt_mesh_net *mesh_data)
{
    return genie_flash_write_reliable(GFI_MESH_PARA, (uint8_t *)mesh_data, sizeof(struct bt_mesh_net));
}

E_GENIE_FLASH_ERRCODE genie_flash_read_para(struct bt_mesh_net *mesh_data)
{
    return genie_flash_read_reliable(GFI_MESH_PARA, (uint8_t *)mesh_data, sizeof(struct bt_mesh_net));
}

E_GENIE_FLASH_ERRCODE genie_flash_write_devkey(uint8_t *p_devkey)
{
    return genie_flash_write_reliable(GFI_MESH_DEVKEY, p_devkey, 16);
}

E_GENIE_FLASH_ERRCODE genie_flash_read_devkey(uint8_t *p_devkey)
{
    return genie_flash_read_reliable(GFI_MESH_DEVKEY, p_devkey, 16);
}

E_GENIE_FLASH_ERRCODE genie_flash_write_netkey(mesh_netkey_para_t *p_netkey)
{
    return genie_flash_write_reliable(GFI_MESH_NETKEY, (uint8_t *)p_netkey, sizeof(mesh_netkey_para_t));
}

E_GENIE_FLASH_ERRCODE genie_flash_read_netkey(mesh_netkey_para_t *p_netkey)
{
    return genie_flash_read_reliable(GFI_MESH_NETKEY, (uint8_t *)p_netkey, sizeof(mesh_netkey_para_t));
}

E_GENIE_FLASH_ERRCODE genie_flash_write_appkey(mesh_appkey_para_t *p_appkey)
{
    return genie_flash_write_reliable(GFI_MESH_APPKEY, (uint8_t *)p_appkey, sizeof(mesh_appkey_para_t));
}

E_GENIE_FLASH_ERRCODE genie_flash_read_appkey(mesh_appkey_para_t *p_appkey)
{
    return genie_flash_read_reliable(GFI_MESH_APPKEY, (uint8_t *)p_appkey, sizeof(mesh_appkey_para_t));
}

E_GENIE_FLASH_ERRCODE genie_flash_write_addr(uint16_t *p_addr)
{
    BT_DBG("0x%04x", *p_addr);

    return genie_flash_write_reliable(GFI_MESH_SADDR, (uint8_t *)p_addr, 2);
}

E_GENIE_FLASH_ERRCODE genie_flash_read_addr(uint16_t *p_addr)
{
    return genie_flash_read_reliable(GFI_MESH_SADDR, (uint8_t *)p_addr, 2);
}

E_GENIE_FLASH_ERRCODE genie_flash_write_hb(mesh_hb_para_t *p_para)
{
    return genie_flash_write_reliable(GFI_MESH_HB, (uint8_t *)p_para, sizeof(mesh_hb_para_t));
}

E_GENIE_FLASH_ERRCODE genie_flash_read_hb(mesh_hb_para_t *p_para)
{
    return genie_flash_read_reliable(GFI_MESH_HB, (uint8_t *)p_para, sizeof(mesh_hb_para_t));
}

E_GENIE_FLASH_ERRCODE genie_flash_reset_system(void)
{
    E_GENIE_FLASH_ERRCODE ret;
    uint32_t pid = 0;
    uint8_t mac[6];
    uint8_t key[32];

    /* read tri truple */
    ret = genie_flash_read_trituple(&pid, mac, key);
    if(ret == GENIE_FLASH_SUCCESS || ret == GENIE_FLASH_SEARCH_NONE) {
        /* erase system and seq */
        ret = genie_flash_delete_seq();
        ret |= genie_flash_erase_reliable();
        RETURN_WHEN_ERR(ret, ret);
        //_genie_flash_check_remain();

        /* write tri truple */
        if(pid != 0) {
            ret = genie_flash_write_trituple(&pid, mac, key);
            RETURN_WHEN_ERR(ret, ret);
        }

        /* delete seq */
        return genie_flash_delete_seq();
    }
    RETURN_WHEN_ERR(ret, ret);

    return GENIE_FLASH_SUCCESS;
}

