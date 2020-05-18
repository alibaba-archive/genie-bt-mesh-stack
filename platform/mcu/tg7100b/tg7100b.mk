HOST_OPENOCD := tg7100b
NAME := mcu_tg7100b
bt_host_crypto = 1
$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 0.0.1
$(NAME)_SUMMARY    := driver & sdk &ble controller for platform/mcu tg7100b

$(NAME)_COMPONENTS += platform/arch/arm/armv6m
$(NAME)_COMPONENTS += libc rhino hal 
#network.bluetooth.bt_host rhino.fs.kv 
#$(NAME)_LIBSUFFIX := $(PLATFORM_MCU_BOARD)

$(NAME)_INCLUDE := csi/csi_driver/include \
                   ecdh

GLOBAL_INCLUDES += csi/csi_core/cmsis/include  \
                   include                     \
                   csi/csi_driver/include      \
                   hal \
                   port \
                   osal/include

GLOBAL_LDFLAGS += -T board/tg7100b/configs/gcc_eflash.ld
GLOBAL_INCLUDES += csi/csi_driver/phyplus/tg7100b/include \
                   csi/csi_driver/phyplus/common/include \
                   modules/ble_dut

$(NAME)_SOURCES += csi/csi_driver/phyplus/common/adc.c           \
                   csi/csi_driver/phyplus/common/ck_irq.c        \
                   csi/csi_driver/phyplus/common/clock.c         \
                   csi/csi_driver/phyplus/common/common.c        \
                   csi/csi_driver/phyplus/common/gpio.c          \
                   csi/csi_driver/phyplus/common/pwm.c           \
                   csi/csi_driver/phyplus/common/trng.c          \
                   csi/csi_driver/phyplus/common/spif.c          \
                   csi/csi_driver/phyplus/common/dw_spi.c        \
                   csi/csi_driver/phyplus/common/dw_usart.c      \
                   csi/csi_driver/phyplus/common/dw_timer.c      \
                   csi/csi_driver/phyplus/common/dw_gpio.c       \
                   csi/csi_driver/phyplus/common/dw_iic.c        \
                   csi/csi_driver/phyplus/common/dw_wdt.c        \
                   csi/csi_driver/phyplus/common/phy_aes.c        \
                   csi/csi_driver/phyplus/common/phy_pmu.c        \
                   csi/csi_driver/phyplus/common/phy_rtc.c        \
                   csi/csi_driver/phyplus/common/gpio_usart.c     \
                   csi/csi_driver/phyplus/tg7100b/startup.S       \
                   csi/csi_driver/phyplus/tg7100b/lpm_arch_reg_save.S  \
                   csi/csi_driver/phyplus/tg7100b/system.c        \
                   csi/csi_driver/phyplus/tg7100b/isr.c           \
                   csi/csi_driver/phyplus/tg7100b/lib.c           \
                   csi/csi_driver/phyplus/tg7100b/device.c        \
                   csi/csi_driver/phyplus/tg7100b/pinmux.c        \
                   csi/csi_driver/phyplus/tg7100b/jump_table.c    \
                   csi/csi_driver/phyplus/tg7100b/lib_printf.c    \
                   csi/csi_driver/phyplus/tg7100b/power_manager.c \
                   csi/csi_driver/phyplus/tg7100b/reboot.c    \
                   csi/csi_driver/phyplus/tg7100b/systick.c    \
                   csi/csi_driver/phyplus/tg7100b/sys_freq.c

$(NAME)_SOURCES += aos/aos.c \
                   aos/hook_impl.c
$(NAME)_SOURCES += hal/pm.c \
                   modules/lpm/lpm.c \
                   modules/ble_dut/dut_uart_driver.c \
                   modules/ble_dut/ble_dut_test.c \
                   modules/ble_dut/dut_at_cmd.c \
                   modules/ble_dut/dut_rf_test.c \
                   modules/ble_dut/dut_utility.c \
                   hal/ringbuffer.c \
                   hal/uart.c \
                   hal/spi.c  \
                   hal/adc.c \
                   hal/flash.c \
                   hal/reboot.c \
                   hal/gpio.c  \
                   hal/timer.c \
                   hal/pwm.c  \
                   hal/i2c.c  \
                   hal/common/device.c \
                   port/ali_dfu_port.c

$(NAME)_SOURCES += ecdh/P256-cortex-m0-ecdh-gcc.S

#include ./platform/mcu/tg7100b/hal/drivers/bt/build.mk
$(NAME)_PREBUILT_LIBRARY := ./hal/drivers/libbt/driver_bt.a
include ./board/tg7100b/rom1Sym.mk

GLOBAL_DEFINES += CONFIG_BT_OBSERVER
#GLOBAL_DEFINES += CONFIG_USING_ADV_SCAN_CTROL_WORK
GLOBAL_DEFINES += CONFIG_BT_USING_HCI_API

ifeq ($(COMPILER),armcc)
GLOBAL_CFLAGS   += --c99 --cpu=7E-M -D__MICROLIB -g --apcs=interwork --split_sections
else ifeq ($(COMPILER),iar)
GLOBAL_CFLAGS += --cpu=Cortex-M0 \
                 --cpu_mode=thumb \
                 --endian=little
else
GLOBAL_CFLAGS +=  -MP -MMD -mcpu=cortex-m0 \
                 -mlittle-endian \
                 -mthumb -mthumb-interwork \
                 -specs=nosys.specs \
                 --specs=rdimon.specs \
                 -w
endif

ifeq ($(COMPILER),armcc)
GLOBAL_ASMFLAGS += --cpu=7E-M -g --apcs=interwork --library_type=microlib --pd "__MICROLIB SETA 1"
else ifeq ($(COMPILER),iar)
GLOBAL_ASMFLAGS += --cpu Cortex-M0 \
                   --cpu_mode thumb \
                   --endian little
else
GLOBAL_ASMFLAGS += -mcpu=cortex-m0 \
                   -mlittle-endian \
                   -mthumb -mthumb-interwork \
                   -w
endif

ifeq ($(COMPILER),armcc)
GLOBAL_LDFLAGS += -L --cpu=7E-M   \
                  -L --strict \
                  -L --xref -L --callgraph -L --symbols \
                  -L --info=sizes -L --info=totals -L --info=unused -L --info=veneers -L --info=summarysizes
else ifeq ($(COMPILER),iar)
GLOBAL_LDFLAGS += --silent --cpu=Cortex-M0.vfp

else
GLOBAL_LDFLAGS += -mcpu=cortex-m0  \
                  -mlittle-endian  \
                  -mthumb -mthumb-interwork \
                  --specs=nosys.specs \
                  $(CLIB_LDFLAGS_NANO_FLOAT)
endif

ifneq ($(CONFIG_BT_PERIPHERAL), y)
GLOBAL_LDFLAGS += -DALI_OPT=1
endif

GLOBAL_CFLAGS += -D_RTE_ -DARMCM0 -DCFG_CP -DCFG_QFN32 -DCFG_SLEEP_MODE="PWR_MODE_NO_SLEEP" -DDEBUG_INFO="1" -DCONFIG_AOS_KV_MULTIPTN_MODE -DCONFIG_AOS_KV_PTN=6 -DCONFIG_AOS_KV_SECOND_PTN=7 -DCONFIG_AOS_KV_PTN_SIZE=4096 -DCONFIG_AOS_KV_BUFFER_SIZE=8192
GLOBAL_DEFINES += CONFIG_BT_MESH_ADV_BUF_COUNT=11
GLOBAL_DEFINES += CONFIG_BT_HARD_ECC
