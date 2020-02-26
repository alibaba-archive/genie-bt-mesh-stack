NAME := bt_profile

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := BLE commonly used profiles.

$(NAME)_SOURCES := bas.c cts.c dis.c hrs.c

GLOBAL_INCLUDES += .
