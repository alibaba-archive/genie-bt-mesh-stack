/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#include <aos/aos.h>

#include <stdlib.h> //for atol
#include "flash.h"
#include "common/log.h"
#include "mesh/access.h"
#include "net.h"
#include "access.h"

#include "genie_app.h"
#include "tri_tuple_default.h"

static void _get_tri_tuple(char *pwbuf, int blen, int argc, char **argv)
{
    genie_tri_tuple_load();
    genie_tri_tuple_show();
}

static void _set_tri_tuple(char *pwbuf, int blen, int argc, char **argv)
{
    uint32_t pid;
    uint8_t mac[6];
    uint8_t key[16];
    uint8_t ret;

    if (argc != 4)
    {
        printk("para error!!!\n");
        return;
    }

    //pro_id
    pid = atol(argv[1]);

    //key
    ret = stringtohex(argv[2], key, 16);

    //addr
    ret = stringtohex(argv[3], mac, 6);

    genie_flash_write_trituple(&pid, mac, key);

    _get_tri_tuple(pwbuf, blen, argc, argv);

}

static void _reboot_handle(char *pwbuf, int blen, int argc, char **argv)
{
    aos_reboot();
}

void print_sw_info(void)
{
#if defined(BOARD_CH6121EVB)
    printf("DEVICE:%s\n", CONFIG_BT_DEVICE_NAME);
    printf("SW VER:%08x\n", PROJECT_SW_VERSION);
    printf("SDK:v%s\n", APP_SDK_VERSION);
    printf("OS:v%d\n", SYSINFO_OS_VERSION);
    printf("PROUDUCT:%s\n", SYSINFO_PRODUCT_MODEL);
#else
    BT_INFO("DEVICE:%s", CONFIG_BT_DEVICE_NAME);
    BT_INFO("SW VER:%08x", PROJECT_SW_VERSION);
    BT_INFO("SDK:v%s", APP_SDK_VERSION);
    BT_INFO("OS:v%d", SYSINFO_OS_VERSION);
    BT_INFO("PROUDUCT:%s", SYSINFO_PRODUCT_MODEL);
#endif
}

static void _get_sw_info(char *pwbuf, int blen, int argc, char **argv)
{
    print_sw_info();
}

extern uint32_t dump_mm_info_used(void);
static void _get_mm_info(char *pwbuf, int blen, int argc, char **argv)
{
#if RHINO_CONFIG_MM_DEBUG
    dump_mm_info_used();
#endif
}

extern struct bt_mesh_elem elements[];
static void _send_msg(char *pwbuf, int blen, int argc, char **argv)
{
    uint8_t count;
    uint8_t msg_b[32];
    uint8_t ret;
    uint8_t i = 0;
    struct bt_mesh_model *p_model = NULL;
    struct bt_mesh_msg_ctx ctx;
    struct net_buf_simple *msg;

    ctx.app_idx = 0;
    ctx.net_idx = 0;
    ctx.addr = 0xF000;
    ctx.send_ttl = 3;
    ctx.send_rel = 0;

    count = strlen(argv[1])>>1;
    ret = stringtohex(argv[1], msg_b, count);

    if(ret == 0) {
        return;
    }

    msg = NET_BUF_SIMPLE(32);

    if(msg == NULL) {
        printk("no buff\n");
        return;
    }

    net_buf_simple_init(msg, 0);
    while(i < count) {
        net_buf_simple_add_u8(msg, msg_b[i]);
        i++;
    }
#ifdef CONFIG_MESH_MODEL_VENDOR_SRV
    p_model = bt_mesh_model_find_vnd(elements, BT_MESH_MODEL_VND_COMPANY_ID, BT_MESH_MODEL_VND_MODEL_SRV);
#endif
    if(p_model == NULL) {
        printk("no model\n");
        return;
    }

    if (bt_mesh_model_send(p_model, &ctx, msg, NULL, NULL)) {
        printk("Unable to send\n");
    }
}

static const struct cli_command genie_cmds[] = {
    {"get_tt", "get tri truple", _get_tri_tuple},
    {"set_tt", "set_tt pid mac key", _set_tri_tuple},
    {"reboot", "reboot", _reboot_handle},

#if defined(CONFIG_GENIE_DEBUG_CMD_FLASH)
    {"seq", "seq test", cmd_handle_flash_seq},
    {"sys", "sys flash", cmd_handle_flash_sys},
    {"ud", "userdata teest", cmd_handle_flash_ud},
#endif

    {"get_info", "get sw info", _get_sw_info},
    {"mm_info", "get mm info", _get_mm_info},
    {"msg", "send mesh msg", _send_msg},
};

void genie_cmds_register(void)
{
#ifdef CONFIG_AOS_CLI
    aos_cli_register_commands(&genie_cmds[0], sizeof(genie_cmds) / sizeof(genie_cmds[0]));
#endif
}

