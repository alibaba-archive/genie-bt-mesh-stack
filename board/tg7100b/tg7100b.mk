NAME := board_tg7100b


$(NAME)_TYPE := kernel
$(NAME)_MBINS_TYPE := kernel
SUPPORT_MBINS      := yes

MODULE               := 1062
HOST_ARCH            := Cortex-M0
HOST_MCU_FAMILY      := tg7100b
SUPPORT_BINS         := no
HOST_MCU_NAME        := tg7100b

GLOBAL_INCLUDES += . \
                   include \
                   $(SOURCE_ROOT)/kernel/rhino/core/include \
                   $(SOURCE_ROOT)/platform/mcu/tg7100b/include

bt_controller := 1
bt_host_tinycrypt := 0

GLOBAL_DEFINES += STDIO_UART=0  CONFIG_NO_TCPIP BOARD_TG7100B

CONFIG_SYSINFO_PRODUCT_MODEL := ALI_AOS_TG7100B
CONFIG_SYSINFO_DEVICE_NAME := TG7100B

$(NAME)_SOURCES := init/board_init.c \
                   init/base_init.c

GLOBAL_CFLAGS  += -DADV_NCONN_CFG=0x01 -DADV_CONN_CFG=0x02 -DSCAN_CFG=0x04 -DINIT_CFG=0x08 -DBROADCASTER_CFG=0x01 -DTG7100B
GLOBAL_CFLAGS  += -DOBSERVER_CFG=0x02 -DPERIPHERAL_CFG=0x04 -DCENTRAL_CFG=0x08 -DHOST_CONFIG=0x4  -DSYSINFO_DEVICE_NAME=\"$(CONFIG_SYSINFO_DEVICE_NAME)\" -DSYSINFO_OS_VERSION=\"$(CONFIG_SYSINFO_OS_VERSION)\" -DSYSINFO_PRODUCT_MODEL=\"$(CONFIG_SYSINFO_PRODUCT_MODEL)\"

# Extra build target in mico_standard_targets.mk, include bootloader, and copy output file to eclipse debug file (copy_output_for_eclipse)
EXTRA_TARGET_MAKEFILES +=  $(MAKEFILES_PATH)/aos_standard_targets.mk
EXTRA_TARGET_MAKEFILES +=  $(SOURCE_ROOT)/board/tg7100b/gen_bin.mk

# Define default component testcase set
ifeq (, $(findstring yts, $(BUILD_STRING)))
GLOBAL_DEFINES += RHINO_CONFIG_WORKQUEUE=1
TEST_COMPONENTS += basic api wifi_hal rhino kv yloop alicrypto cjson hashtable
else
GLOBAL_DEFINES += RHINO_CONFIG_WORKQUEUE=0
endif
