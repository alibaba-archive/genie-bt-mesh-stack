NAME := tc32_825x
HOST_OPENOCD := tc32_825x

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 0.0.1
$(NAME)_COMPONENTS += platform/arch/tc32
#$(NAME)_COMPONENTS += libc rhino hal rhino.vfs network.bluetooth.bt_host network.bluetooth.bt_mesh rhino.fs.kv
$(NAME)_COMPONENTS += libc rhino hal rhino.vfs network.bluetooth.bt_host rhino.fs.kv

GLOBAL_DEFINES    += CONFIG_NO_TCPIP
GLOBAL_LDFLAGS    += -nostdlib -nostartfiles -nodefaultlibs
#GLOBAL_LDFLAGS   += -nostartfiles -nodefaultlibs

# MODULE 参数在board的组件中定义
ifeq ($(MODULE), tc8258)
#GLOBAL_DEFINES   += MCU_CORE_8258

$(NAME)_PREBUILT_LIBRARY := proj_lib/liblt_8258.a
$(info ========liblt_8258.a========)
endif
$(NAME)_PREBUILT_LIBRARY += proj_lib/libsoft-fp.a

# RAM_RET_TYPE 参数在board的组件中定义
ifeq ($(RAM_RET_TYPE), MCU_STARTUP_8258_RET_16K)
$(NAME)_SOURCES +=  boot/8258/cstartup_8258_RET_16K.S
$(info ========8258 32k ret start========)
else ifeq ($(RAM_RET_TYPE), MCU_STARTUP_8258_RET_32K)
$(NAME)_SOURCES +=  boot/8258/cstartup_8258_RET_32K.S
$(info ========8258 32k ret start========)
else ifeq ($(RAM_RET_TYPE), MCU_STARTUP_8258)
$(NAME)_SOURCES +=  boot/8258/cstartup_8258.S
$(info ========8258 def ret start========)
endif

$(NAME)_SOURCES +=  div_mod.S

# RAM_RET_TYPE 参数在board的组件中定义
ifeq ($(RAM_RET_TYPE), MCU_STARTUP_8258_RET_16K)
#GLOBAL_DEFINES  += -DMCU_STARTUP_8258_RET_16K
#.s中不起作用
GLOBAL_LDFLAGS  += -T platform/mcu/tc32_825x/boot.link
$(info ========8258 32k ret boot link========)
else ifeq ($(RAM_RET_TYPE), MCU_STARTUP_8258_RET_32K)
GLOBAL_LDFLAGS  += -T platform/mcu/tc32_825x/boot_ret_32k.link
#需要在初始化的时候添加blc_pm_setDeepsleepRetentionType(DEEPSLEEP_MODE_RET_SRAM_LOW32K);
$(info =需要在初始化的时候添加blc_pm_setDeepsleepRetentionType(DEEPSLEEP_MODE_RET_SRAM_LOW32K)=)
$(info ========8258 32k ret boot link========)
else ifeq ($(RAM_RET_TYPE), MCU_STARTUP_8258)
GLOBAL_LDFLAGS  += -T platform/mcu/tc32_825x/boot.link
$(info =理论上编译时不应该到这里=)
$(info ========8258 def ret boot link========)
endif

#GLOBAL_INCLUDES += application/common\
                   application/app\
                   application/keyboard\
				   application/print\
				   application/usbstd\
				   application

#GLOBAL_INCLUDES += common/config\
				   common

#GLOBAL_INCLUDES += drivers/8258\

#GLOBAL_INCLUDES += stack/ble/attr\
                   stack/ble/crypt\
				   stack/ble/crypt/aes\
                   stack/ble/gap\
				   stack/ble/hci\
				   stack/ble/ll\
				   stack/ble/phy\
				   stack/ble/service\
				   stack/ble/smp\
				   stack/ble
#GLOBAL_INCLUDES += vendor/common
GLOBAL_INCLUDES += osal/include \
		   kernel/rhino/hal/soc

GLOBAL_INCLUDES += ./				   
						   

$(NAME)_SOURCES += application/app/usbaud.c\
				   application/app/usbcdc.c\
				   application/app/usbkb.c\
				   application/app/usbmouse.c\

$(NAME)_SOURCES += application/keyboard/keyboard.c\

$(NAME)_SOURCES += application/print/putchar_sim.c\
				   application/print/u_printf.c
				   
$(NAME)_SOURCES += application/usbstd/usb.c\
				   application/usbstd/usbhw.c\
				   application/usbstd/usbdesc.c\
				   
$(NAME)_SOURCES += common/breakpoint.c\
				   common/selection_sort.c\
				   common/string.c\
                   common/utility.c\
                   common/ring_buffer.c\

$(NAME)_SOURCES += \
        drivers/8258/adc.c \
        drivers/8258/analog.c \
        drivers/8258/audio.c \
        drivers/8258/bsp.c \
        drivers/8258/clock.c \
        drivers/8258/emi.c \
        drivers/8258/flash.c \
        drivers/8258/gpio_8258.c \
        drivers/8258/i2c.c \
        drivers/8258/rf_pa.c \
        drivers/8258/spi.c \
        drivers/8258/uart.c \
        drivers/8258/watchdog.c 
				   						   
$(NAME)_SOURCES += vendor/common/blt_common.c\
				   vendor/common/blt_led.c\
				   vendor/common/blt_soft_timer.c\
				   vendor/common/tl_audio.c

#$(NAME)_SOURCES +=  hal/hal_uart.c

$(NAME)_SOURCES +=  aos/soc_impl.c \
                   aos/aos.c \
                   aos/mod_div.c \
                   aos/alios_std_lib_api.c \
                   aos/ctype.c \
                   aos/errno/errno.c

ifeq ($(ble),1)
$(NAME)_SOURCES  += hal/ais_ota_port.c
$(NAME)_SOURCES  += hal/ble_port.c
endif

$(NAME)_SOURCES  += hal/misc.c
#$(NAME)_SOURCES  += hal/pwrmgmt_hal/board_cpu_pwr_rtc.c
#$(NAME)_SOURCES  += hal/pwrmgmt_hal/board_cpu_pwr_systick.c
#$(NAME)_SOURCES  += hal/pwrmgmt_hal/board_cpu_pwr.c
$(NAME)_SOURCES  += hal/flash.c
$(NAME)_SOURCES  += hal/gpio.c
$(NAME)_SOURCES  += hal/pwm.c
$(NAME)_SOURCES  += hal/uart.c

$(NAME)_SOURCES  += port/ali_dfu_port.c

include $(SOURCE_ROOT)platform/mcu/$(PLATFORM_MCU_BOARD)/bt_controller/bt_controller.mk
GLOBAL_DEFINES += CONFIG_BT_CTLR

