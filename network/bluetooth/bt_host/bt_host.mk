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
hci_h4 = 0
$(NAME)_SOURCES += hci_driver/ch6121_driver.c
else ifeq ($(HOST_MCU_FAMILY),tg7100b)
hci_h4 = 0
$(NAME)_SOURCES += hci_driver/tg7100b_driver.c
endif

ifeq ($(hci_h4),1)
$(NAME)_SOURCES += hci_driver/h4.c
endif

bt_host_tinycrypt ?= 1
ifeq ($(bt_host_tinycrypt),1)
GLOBAL_DEFINES += CONFIG_BT_TINYCRYPT_ECC
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

