NAME := genie_app

ble = 1

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION := 1.1.0
$(NAME)_SUMMARY := Genie mesh application.

GLOBAL_CFLAGS += -DSYSINFO_BUILD_TIME=\"$(CURRENT_TIME)\"

$(NAME)_COMPONENTS += bluetooth.bt_mesh yloop cli bluetooth.bt_host crc

$(NAME)_INCLUDES += ../network/bluetooth/bt_mesh/inc \
					../network/bluetooth/bt_mesh/inc/api \
					../network/bluetooth/bt_mesh/inc/port \
					../network/bluetooth/bt_mesh/mesh_model \
					../network/bluetooth/bt_mesh/vendor_model \
					../network/bluetooth/bt_mesh/inc \
					../utility/crc \
					../genie_app

$(NAME)_SOURCES  += genie_app.c \
					base/genie_cmds.c \
					base/genie_reset.c \
					base/genie_event.c \
					base/genie_flash.c \
					base/tri_tuple.c \
					bluetooth/mesh/genie_mesh.c \
					bluetooth/mesh/genie_mesh_flash.c

ifeq ($(genie_vendor_timer),1)
GLOBAL_DEFINES += MESH_MODEL_VENDOR_TIMER
$(NAME)_SOURCES += base/vendor_timers.c
endif
GLOBAL_DEFINES += CONFIG_BT_MESH
GLOBAL_DEFINES += CONFIG_GENIE_CLI
GLOBAL_DEFINES += APP_SDK_VERSION=\"$($(NAME)_VERSION)\"
GLOBAL_DEFINES += CONFIG_GENIE_RESET_BY_REPEAT
#GLOBAL_DEFINES += CONFIG_GENIE_RESET_BY_PRESS
#GLOBAL_DEFINES += AES_CMAC_DECRYPT


####### ota config #######
ifeq ($(genie_ota),1)
GLOBAL_DEFINES += CONFIG_GENIE_OTA
$(NAME)_SOURCES  += bluetooth/host/profile/ais_srv/ais_service.c
ifeq ($(HOST_ARCH), tc32)
GLOBAL_DEFINES += CONFIG_GENIE_OTA_PINGPONG
endif
endif


####### model config #######

#check config
ifeq ($(MESH_MODEL_CTL_SRV),1)
MESH_MODEL_GEN_ONOFF_SRV = 1
MESH_MODEL_LIGHTNESS_SRV = 1
ifneq ($(ALI_SIMPLE_MODLE),1)
MESH_MODEL_GEN_LEVEL_SRV = 1
endif
endif

ifeq ($(MESH_MODEL_LIGHTNESS_SRV),1)
MESH_MODEL_GEN_ONOFF_SRV = 1
ifneq ($(ALI_SIMPLE_MODLE),1)
MESH_MODEL_GEN_LEVEL_SRV = 1
endif
endif

ifneq ($(MESH_MODEL_DIABLE_TRANS),1)
GLOBAL_DEFINES += CONFIG_MESH_MODEL_TRANS
endif

ifeq ($(MESH_MODEL_GEN_ONOFF_SRV),1)
GLOBAL_DEFINES += CONFIG_MESH_MODEL_GEN_ONOFF_SRV
$(NAME)_SOURCES += bluetooth/mesh/mesh_model/src/gen_onoff_srv.c
endif

ifeq ($(MESH_MODEL_GEN_LEVEL_SRV),1)
GLOBAL_DEFINES += CONFIG_MESH_MODEL_GEN_LEVEL_SRV
$(NAME)_SOURCES += bluetooth/mesh/mesh_model/src/gen_level_srv.c
endif

ifeq ($(MESH_MODEL_LIGHTNESS_SRV),1)
GLOBAL_DEFINES += CONFIG_MESH_MODEL_LIGHTNESS_SRV
GLOBAL_DEFINES += CONFIG_MESH_MODEL_LIGHT_CMD
$(NAME)_SOURCES += bluetooth/mesh/mesh_model/src/lightness_srv.c
endif

ifeq ($(MESH_MODEL_CTL_SRV),1)
GLOBAL_DEFINES += CONFIG_MESH_MODEL_CTL_SRV
GLOBAL_DEFINES += CONFIG_MESH_MODEL_CTL_SETUP_SRV
#GLOBAL_DEFINES += CONFIG_MESH_MODEL_CTL_TEMPERATURE_SRV
$(NAME)_SOURCES += bluetooth/mesh/mesh_model/src/light_ctl_srv.c
endif

ifeq ($(MESH_MODEL_VENDOR_SRV),1)
GLOBAL_DEFINES += CONFIG_MESH_MODEL_VENDOR_SRV
$(NAME)_SOURCES += bluetooth/mesh/mesh_model/src/vendor_model_srv.c
endif

ifeq ($(ALI_SIMPLE_MODLE),1)
GLOBAL_DEFINES += CONFIG_ALI_SIMPLE_MODLE
else
ifeq ($(MESH_MODEL_DIABLE_TRANS),1)
$(error "MESH_MODEL_DIABLE_TRANS must set to 0")
endif
endif

$(NAME)_SOURCES += bluetooth/mesh/mesh_model/src/model_bind_ops.c

