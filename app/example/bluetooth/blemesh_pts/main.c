/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#if 1
#include <stdio.h>
#include <string.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include <misc/printk.h>

#include <bluetooth.h>
#include <api/mesh.h>

#if 1
#define CID_INTEL            0x0002
#define ID_TEMP_CELSIUS            0x2A1F

#define BT_MESH_MODEL_OP_SENSOR_STATUS    BT_MESH_MODEL_OP_1(0x52)
#define BT_MESH_MODEL_OP_SENSOR_GET    BT_MESH_MODEL_OP_2(0x82, 0x31)

static struct k_work temp_work;
static struct k_timer temp_timer;

static u16_t node_addr = BT_MESH_ADDR_UNASSIGNED;



#define CUR_FAULTS_MAX 4

static u8_t cur_faults[CUR_FAULTS_MAX];
static u8_t reg_faults[CUR_FAULTS_MAX * 2];

static void get_faults(u8_t *faults, u8_t faults_size, u8_t *dst, u8_t *count)
{
	u8_t i, limit = *count;

	for (i = 0, *count = 0; i < faults_size && *count < limit; i++) {
		if (faults[i]) {
			*dst++ = faults[i];
			(*count)++;
		}
	}
}

static int fault_get_cur(struct bt_mesh_model *model, u8_t *test_id,
			 u16_t *company_id, u8_t *faults, u8_t *fault_count)
{
	printk("Sending current faults\n");

	*test_id = 0x00;
	*company_id = CID_INTEL;

	get_faults(cur_faults, sizeof(cur_faults), faults, fault_count);

	return 0;
}

static int fault_get_reg(struct bt_mesh_model *model, u16_t cid,
			 u8_t *test_id, u8_t *faults, u8_t *fault_count)
{
	if (cid != CID_INTEL) {
		printk("Faults requested for unknown Company ID 0x%04x\n", cid);
		return -EINVAL;
	}

	printk("Sending registered faults\n");

	*test_id = 0x00;

	get_faults(reg_faults, sizeof(reg_faults), faults, fault_count);

	return 0;
}

static int fault_clear(struct bt_mesh_model *model, uint16_t cid)
{
	if (cid != CID_INTEL) {
		return -EINVAL;
	}

	memset(reg_faults, 0, sizeof(reg_faults));

	return 0;
}

static int fault_test(struct bt_mesh_model *model, uint8_t test_id,
		      uint16_t cid)
{
	if (cid != CID_INTEL) {
		return -EINVAL;
	}

	if (test_id != 0x00) {
		return -EINVAL;
	}

	return 0;
}

static void show_faults(u8_t test_id, u16_t cid, u8_t *faults, size_t fault_count)
{
        size_t i;

        if (!fault_count) {
                printk("Health Test ID 0x%02x Company ID 0x%04x: no faults\n",
                       test_id, cid);
                return;
        }

        printk("Health Test ID 0x%02x Company ID 0x%04x Fault Count %zu:\n",
               test_id, cid, fault_count);

        for (i = 0; i < fault_count; i++) {
                printk("\t0x%02x\n", faults[i]);
        }
}

static void health_current_status(struct bt_mesh_health_cli *cli, u16_t addr,
                                  u8_t test_id, u16_t cid, u8_t *faults,
                                  size_t fault_count)
{
        printk("Health Current Status from 0x%04x\n", addr);
        show_faults(test_id, cid, faults, fault_count);
}

#ifdef CONFIG_BT_MESH_CFG_CLI
static struct bt_mesh_cfg_cli cfg_cli = {
};
#endif



#ifdef CONFIG_BT_MESH_HEALTH_CLI
static struct bt_mesh_health_cli health_cli = {
        .current_status = health_current_status,
};
#endif


static struct bt_mesh_model root_models[] = {
        /* Mandatory Configuration Server model. Should be the first model
         * of root element */
#ifdef CONFIG_BT_MESH_CFG_SRV
        BT_MESH_MODEL_CFG_SRV(),
#endif

#ifdef CONFIG_BT_MESH_CFG_CLI
        BT_MESH_MODEL_CFG_CLI(&cfg_cli),
#endif
#ifdef CONFIG_BT_MESH_HEALTH_SRV
        BT_MESH_MODEL_HEALTH_SRV(),
#endif
#ifdef CONFIG_BT_MESH_HEALTH_CLI
        BT_MESH_MODEL_HEALTH_CLI(&health_cli),
#endif
};

static struct bt_mesh_elem elements[] = {
        BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE, 0),
};

/* Node composition data used to configure a node while provisioning */
static const struct bt_mesh_comp comp = {
        .cid = CID_INTEL,
        .elem = elements,
        .elem_count = ARRAY_SIZE(elements),
};

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
        printk("OOB Number: %u\n", number);
        
    return 0;
}

static void temp_work_thread(struct k_work *work)
{
    struct bt_mesh_model *model = &root_models[2];
           struct net_buf_simple *msg = model->pub->msg;
    int ret;

    if (node_addr == BT_MESH_ADDR_UNASSIGNED) {
            goto exit;
        }

    /* sensor status */
        bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENSOR_GET);
    net_buf_simple_add_le16(msg, ID_TEMP_CELSIUS);

    ret = bt_mesh_model_publish(model);
    if (ret) {
        printk("ERR: Unable to send sensor status get request: %d\n", ret);
        goto exit;
    }

        printk("Sensor status Get request sent with OpCode 0x%08x\n", BT_MESH_MODEL_OP_SENSOR_GET);
exit:
        k_timer_start(&temp_timer, K_SECONDS(5));
}

static void temp_timer_thread(void *work, void *args)
{
    k_work_submit(&temp_work);
}

static void prov_complete(u16_t net_idx, u16_t addr)
{
        printk("Provisioning completed!\n");
    printk("Net ID: %u\n", net_idx);
    printk("Unicast addr: 0x%04x\n", addr);
    
    node_addr = addr;

        k_work_init(&temp_work, temp_work_thread);
        k_timer_init(&temp_timer, temp_timer_thread, NULL);
        k_timer_start(&temp_timer, K_SECONDS(50));
}

/* UUID for identifying the unprovisioned node */
static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

/* Only displaying the number while provisioning is supported */
static const struct bt_mesh_prov prov = {
        .uuid = dev_uuid,
        .complete = prov_complete,
};

static void bt_ready(int err)
{
    int ret;

        if (err) {
                printk("Bluetooth init failed (err %d)\n", err);
                return;
        }

        printk("Bluetooth initialized\n");

        ret = bt_mesh_init(&prov, &comp);
        if (ret) {
                printk("Initializing mesh failed (err %d)\n", ret);
                return;
        }

    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    
        printk("Mesh initialized\n");
}

extern int hci_driver_init();
void blemesh_sample(void)
{
        int ret;

        printk("Initializing...\n");

        hci_driver_init();
        ais_ota_bt_storage_init();
        /* Initialize the Bluetooth Subsystem */
        ret = bt_enable(bt_ready);
        if (ret) {
                printk("Bluetooth init failed (err %d)\n", ret);
        }
}
#endif
static void app_delayed_action(void *arg)
{
    //blemesh_sample();
}

#ifdef CONFIG_BT_MESH_SHELL
static void handle_bt_mesh_cmd(char *pwbuf, int blen, int argc, char **argv)
{
    struct mesh_shell_cmd *mesh_cmds = NULL, *p;
    char *cmd_str, no_match = 1;;

    if (strcmp(argv[0], "bt-mesh") != 0) {
        return;
    }

    if (argc <= 1) {
        cmd_str = "help";
    } else {
        cmd_str = argv[1];
    }

    mesh_cmds = bt_mesh_get_shell_cmd_list();
    if (mesh_cmds) {
        p = mesh_cmds;
        while (p->cmd_name != NULL) {
            if (strcmp(p->cmd_name, cmd_str) != 0) {
                p++;
                continue;
            }
            if (p->cb) {
                no_match = 0;
                p->cb(argc - 1, &(argv[1]));
                return;
            }
        }
        printf("cmd error\n");
    }
}

static uint8_t char2u8(char *c)
{
    uint8_t ret = 0;

    if (isdigit(*c)) {
        ret = *c - '0';
    } else if (*c >= 'A' && *c <= 'F') {
        ret = *c - 'A' + 10;
    } else if (*c >= 'a' && *c <= 'f') {
        ret = *c - 'a' + 10;
    }

    return ret;
}

static void handle_set_mac(char *pwbuf, int blen, int argc, char **argv)
{
    char *p;
    uint8_t mac[6] = {0}, i;

    if (argc < 2) {
        printf("Invalid argument.\r\n");
        printf("Usage:\n");
        return;
    }

    for (p = argv[1], i = 0; *p != '\0'; p += 2, i += 2) {
        if (!isxdigit(*p) || !isxdigit(*(p+1))) {
            printf("Invalid format, MAC not set!!!\r\n");
            return;
        }

        mac[i / 2] = ((char2u8(p) & 0x0f) << 4) | (char2u8(p+1) & 0x0f);
    }

    ais_set_mac(mac);
}

static struct cli_command ncmd[] = {
                                    {
                                        .name     = "set_mac",
                                        .help     = "set_mac <MAC address in xxxxxxxxxxxx format>",
                                        .function = handle_set_mac
                                    },
                                    {   .name     = "bt-mesh",
                                        .help     = "bt-mesh [cmd] [options]",
                                        .function = handle_bt_mesh_cmd 
                                    },
                                 }

;
#endif

int application_start(int argc, char **argv)
{
    printf("Hello, PTS node.\r\n");
#ifdef CONFIG_BT_MESH_SHELL
    aos_cli_register_commands(&ncmd, sizeof(ncmd)/sizeof(ncmd[0]));
#endif
    aos_post_delayed_action(1000, app_delayed_action, NULL);
    aos_loop_run();
    return 0;
}
#endif