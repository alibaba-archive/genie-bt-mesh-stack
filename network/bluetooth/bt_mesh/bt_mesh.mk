NAME := bt_mesh

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := BLE Mesh stack.

ifeq ($(bt_mesh_standalone_deploy),1)
GLOBAL_DEFINES += CONFIG_MESH_STACK_ALONE
$(NAME)_COMPONENTS += bluetooth.bt_mesh.util
else
$(NAME)_COMPONENTS += bluetooth.bt_common 
endif

$(NAME)_COMPONENTS += bluetooth.bt_mesh.ref_impl

$(NAME)_INCLUDES += ./inc/ \
                    ./inc/api/ \
                    ./inc/api/mesh/ \
                    ../../../genie_app/ \
					../../../genie_app/base \
					../../../genie_app/bluetooth/host \
					../../../genie_app/bluetooth/mesh

                    
$(NAME)_SOURCES  :=  ./src/access.c \
                     ./src/adv.c \
                     ./src/beacon.c \
                     ./src/crypto.c \
                     ./src/cfg_srv.c \
                     ./src/cfg_cli.c \
                     ./src/health_srv.c \
                     ./src/health_cli.c \
                     ./src/main.c \
                     ./src/net.c \
                     ./src/prov.c \
                     ./src/proxy.c \
                     ./src/transport.c \
                     ./src/friend.c \
                     ./src/lpn.c \
                     ./src/shell.c 


GLOBAL_INCLUDES += ./inc/ \
                   ./inc/api


GLOBAL_DEFINES += CONFIG_BT_MESH_MULTIADV

GLOBAL_DEFINES += CRC16_ENABLED

## MESH debug log control macro
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_ADV
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_BEACON
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_PROXY
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_PROV
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_NET
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_CRYPTO
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_TRANS
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_FRIEND
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_LOW_POWER
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_ACCESS
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_FLASH
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_MODEL
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_VENDOR_MODEL
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_FACTORY
#GLOBAL_DEFINES += CONFIG_BT_MESH_DEBUG_OTA
GLOBAL_DEFINES += CONFIG_GENIE_DEBUG_CMD_FLASH

