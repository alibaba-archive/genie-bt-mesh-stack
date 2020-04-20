NAME := bt_host

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := BLE stack.

GLOBAL_DEFINES += AOS_BT

GLOBAL_INCLUDES += include \
                   include/drivers

$(NAME)_COMPONENTS += yloop

$(NAME)_COMPONENTS += bluetooth.bt_common

$(NAME)_SOURCES := host/uuid.c \
                     host/hci_core.c \
                     host/conn.c \
                     host/l2cap.c \
                     host/att.c \
                     host/gatt.c \
                     host/keys.c \
                     host/rpa.c \
                     host/multi_adv.c

ifeq ($(HOST_MCU_FAMILY),ch6121)
$(NAME)_SOURCES += hci_driver/ch6121_driver.c

#GLOBAL_DEFINES += CONFIG_BT_DEBUG_LOG
#GLOBAL_DEFINES += CONFIG_BT_DEBUG
else ifeq ($(HOST_MCU_FAMILY),tg7100b)
$(NAME)_SOURCES += hci_driver/tg7100b_driver.c
else
ifeq ($(hci_h4),1)
$(NAME)_SOURCES += hci_driver/h4.c
endif
endif
bt_host_tinycrypt ?= 1
ifeq ($(bt_host_tinycrypt),1)
$(NAME)_SOURCES += host/crypto.c \
                   host/hci_ecc.c
else
$(NAME)_SOURCES += host/crypto_ctrl.c \
                   host/hci_ecc_ctrl.c
endif

ifeq ($(COMPILER),)
$(NAME)_CFLAGS      += -Wall
else ifeq ($(COMPILER),gcc)
$(NAME)_CFLAGS      += -Wall
endif

en_bt_smp ?= 1
ifeq ($(en_bt_smp),1)
GLOBAL_DEFINES += CONFIG_BT_SMP
$(NAME)_SOURCES += host/smp.c
else
$(NAME)_SOURCES += host/smp_null.c
endif

## BLE debug log general control macro (Note: still to be affected by DEBUG)
## Enable below macros if BLE stack debug needed
ifeq ($(HOST_MCU_FAMILY),ch6121)
else ifeq ($(HOST_MCU_FAMILY),tg7100b)
else
GLOBAL_DEFINES += CONFIG_BT_DEBUG_LOG
GLOBAL_DEFINES += CONFIG_BT_DEBUG
endif

## BLE subsystem debug log control macro
## Enable below macros if component-specific debug needed
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_L2CAP
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_CONN
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_ATT
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_GATT
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_HCI_DRIVER
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_HCI_CORE
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_CORE
#GLOBAL_DEFINES += CONFIG_BT_DEBUG_MULTI_ADV