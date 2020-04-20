##
 # Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #   http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
##
###############################################################################
# @file     makefile
# @brief    the makefile for the whole project
# @version  V1.0
# @date     02. June 2017
###############################################################################

ABSROOTDIR = $(shell pwd)
ROOTDIR = .

TARGETS_ROOT_PATH ?=
CONFIG_FILE ?=
CONFIG_INC_PATH ?=

ifneq ($(CONFIG_FILE),)
CONFIG_FILE_CHECK:= $(shell ls $(CONFIG_FILE) 2>/dev/null)
endif

ifeq ($(CONFIG_FILE_CHECK),)
$(info "CONFIG_FILE not set")
else

include $(CONFIG_FILE)

ifeq ($(CONFIG_CPU_CK610), y)
CC      = $(TOOL_PATH)csky-elf-gcc
CPP     = $(TOOL_PATH)csky-elf-g++
AS      = $(TOOL_PATH)csky-elf-as
AR      = $(TOOL_PATH)csky-elf-ar
LD      = $(TOOL_PATH)csky-elf-ld
DUMP    = $(TOOL_PATH)csky-elf-objdump
OBJCOPY = $(TOOL_PATH)csky-elf-objcopy
else

ifeq ($(CONFIG_ARCH_ARM), y)

TOOLCHAIN_PREFIX:=arm-none-eabi-

else

TOOLCHAIN_PREFIX_CHECK:= $(shell which csky-elfabiv2-gcc)
ifeq ($(TOOLCHAIN_PREFIX_CHECK),)
TOOLCHAIN_PREFIX:=csky-abiv2-elf-
else
TOOLCHAIN_PREFIX:=csky-elfabiv2-
endif

endif

CC      = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)gcc
CPP     = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)g++
AS      = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)as
AR      = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)ar
LD      = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)ld
DUMP    = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)objdump
OBJCOPY = $(TOOL_PATH)$(TOOLCHAIN_PREFIX)objcopy
endif

CONFIG_CHIP_VENDOR_NAME := $(patsubst "%",%,$(strip $(CONFIG_CHIP_VENDOR_STR)))

CONFIG_CHIP_NAME  := $(patsubst "%",%,$(strip $(CONFIG_CHIP_NAME_STR)))
CONFIG_BOARD_NAME := $(patsubst "%",%,$(strip $(CONFIG_BOARD_NAME_STR)))

CONFIG_DEBUG_LEVEL := $(patsubst "%", %, $(strip $(CONFIG_DEBUG_LEVEL)))
CONFIG_OPTIMIZE_LEVEL := $(patsubst "%", %, $(strip $(CONFIG_OPTIMIZE_LEVEL)))

ifeq ($(CONFIG_BIG_ENDIAN),y)
ENDIAN_MODE = -mbig-endian
else
ENDIAN_MODE = -mlittle-endian
endif

OUTDIR = out
OBJDIR = $(OUTDIR)/obj
CSIDRIVERDIR = $(ROOTDIR)/csi_driver

ifeq ($(CONFIG_ARCH_ARM), y)
COREDIR = $(ROOTDIR)/csi_core/cmsis
else
COREDIR = $(ROOTDIR)/csi_core
endif

OFFCHIPDRIVERDIR = $(ROOTDIR)/drivers
DRIVERDIR = $(ROOTDIR)/csi_driver/$(CONFIG_CHIP_VENDOR_NAME)/common
LIBSDIR = $(ROOTDIR)/libs
CHIPDIR = $(ROOTDIR)/csi_driver/$(CONFIG_CHIP_VENDOR_NAME)/$(CONFIG_CHIP_NAME)
KERNELDIR = $(ROOTDIR)/csi_kernel
INCDIR =
CSRC =
LDDIR = $(ROOTDIR)/csi_driver/$(CONFIG_CHIP_VENDOR_NAME)/$(CONFIG_CHIP_NAME)
NAME = $(CONFIG_BOARD_NAME)
PACKNAME = libcsi.a

export CC AS AR LD DUMP OBJCOPY CFLAGS LDFLAGS ASFLAGS INCLUDEDIRS ROOTDIR TARGERDIR DRIVERDIR COREDIR TESTDIR KERNELDIR OFFCHIPDRIVERDIR

$(shell [ -d ${OUTDIR} ] || mkdir -p ${OUTDIR} && mkdir -p ${OBJDIR})

include $(CHIPDIR)/csi.mk
-include $(OFFCHIPDRIVERDIR)/csi.mk
include $(LIBSDIR)/csi.mk
ifneq ($(CONFIG_KERNEL_NONE), y)
include $(KERNELDIR)/csi.mk
endif

INCLUDEDIRS = -I$(ROOTDIR)/include -I$(COREDIR)/include -I$(CSIDRIVERDIR)/include -I$(TESTDIR)/include -I$(KERNELDIR)/include -I$(LIBSDIR)/include
INCLUDEDIRS += $(INCDIR)

ifneq ($(TARGETS_ROOT_PATH),)
INCLUDEDIRS += -I$(TARGETS_ROOT_PATH)/include
endif

ifneq ($(CONFIG_INC_PATH),)
INCLUDEDIRS += -I$(CONFIG_INC_PATH)
endif

ifneq ($(BOARD_INCDIR),)
INCLUDEDIRS += -I$(BOARD_INCDIR)
endif

ifeq ($(CONFIG_CPU_E801), y)
CPU += -mcpu=e801
endif

ifeq ($(CONFIG_CPU_E802), y)
CPU += -mcpu=e802
endif

ifeq ($(CONFIG_CPU_E802T), y)
CPU += -mcpu=e802t
endif

ifeq ($(CONFIG_CPU_S802), y)
CPU += -mcpu=s802
endif

ifeq ($(CONFIG_CPU_S802T), y)
CPU += -mcpu=s802t
endif

ifeq ($(CONFIG_CPU_E803), y)
CPU += -mcpu=e803
endif

ifeq ($(CONFIG_CPU_E803T), y)
CPU += -mcpu=e803t
endif

ifeq ($(CONFIG_CPU_S803), y)
CPU += -mcpu=s803
endif

ifeq ($(CONFIG_CPU_S803T), y)
CPU += -mcpu=s803t
endif

ifeq ($(CONFIG_CPU_E804D), y)
CPU += -mcpu=e804d
endif

ifeq ($(CONFIG_CPU_E804DT), y)
CPU += -mcpu=e804dt
endif

ifeq ($(CONFIG_CPU_E804F), y)
CPU += -mcpu=e804f
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_E804FT), y)
CPU += -mcpu=e804ft
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_E804DF), y)
CPU += -mcpu=e804df
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_E804DFT), y)
CPU += -mcpu=e804dft
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_I805), y)
CPU += -mcpu=i805
endif

ifeq ($(CONFIG_CPU_I805F), y)
CPU += -mcpu=i805f
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK801), y)
CPU += -mcpu=ck801
endif

ifeq ($(CONFIG_CPU_CK802), y)
CPU += -mcpu=ck802
endif

ifeq ($(CONFIG_CPU_CK803), y)
CPU += -mcpu=ck803
endif

ifeq ($(CONFIG_CPU_CK803F), y)
CPU += -mcpu=ck803f
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK803EF), y)
CPU += -mcpu=ck803ef -mdsp
CFLAGS += -mhard-float -mhigh-registers
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK803EFR1), y)
CPU += -mcpu=ck803efr1
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK803EFR2), y)
CPU += -mcpu=ck803efr2
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK803EFR3), y)
CPU += -mcpu=ck803efr3
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK804EF), y)
CPU += -mcpu=ck804ef
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK805), y)
CPU += -mcpu=ck805
endif

ifeq ($(CONFIG_CPU_CK805F), y)
CPU += -mcpu=ck805f
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_CK610E), y)
CPU += -mcpu=ck610e
CFLAGS += -mdsp
endif

ifeq ($(CONFIG_CPU_CK610EF), y)
CPU += -mcpu=ck610ef
CFLAGS += -mdsp
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_C810), y)
CPU += -mcpu=c810
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_C810T), y)
CPU += -mcpu=c810t
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_C810V), y)
CPU += -mcpu=c810v
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_C810TV), y)
CPU += -mcpu=c810tv
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_C807), y)
CPU += -mcpu=c807
endif

ifeq ($(CONFIG_CPU_C807F), y)
CPU += -mcpu=c807f
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_C807FV), y)
CPU += -mcpu=c807fv
CFLAGS += -mhard-float
LDFLAGS += -mhard-float
endif

ifeq ($(CONFIG_CPU_R807), y)
CPU += -mcpu=ck807
endif

ifeq ($(CONFIG_CPU_R807F), y)
CPU += -mcpu=ck807f -mhard-float
endif

ifeq ($(CONFIG_CPU_RV32EMC), y)
CPU += -march=rv32emc -mabi=ilp32e
CKMAP=
endif

ifeq ($(CONFIG_CPU_RV32EC), y)
CPU += -march=rv32ec -mabi=ilp32e
CKMAP=
endif

ifeq ($(CONFIG_CPU_CM0), y)
CPU += -mcpu=cortex-m0 -mthumb
ASFLAGS = -Wa,-mthumb
CKMAP=

CFLAGS += $(CPU) -c
else
CFLAGS += $(CPU) -c -Wa,-melrw
endif

ifeq ($(CONFIG_HARD_FLOAT), y)
CFLAGS += -mhard-float
endif

CFLAGS += $(CONFIG_DEBUG_LEVEL) $(CONFIG_OPTIMIZE_LEVEL) $(CONFIG_ENDIAN_MODE)
CFLAGS += -Wall
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += $(INCLUDEDIRS)

ifeq ($(CONFIG_BENCHMARK_COREMARK), y)
CFLAGS += -DCOMPILER_FLAGS=\"-Os\"
endif

LDFLAGS = -EL $(CPU)
ASFLAGS += $(CFLAGS)
ARFLAGS = r
CPPLDFLAG_END += -lstdc++ -lsupc++


LDLIBS	=  -lm -lc -lgcc  -Wl,--gc-section

ifeq ($(V),1)
Q =
else
Q = @
endif

export Q

.PHONY: all

all: lib
lib: lib_sub

S_SRC = $(wildcard $(SSRC))
C_SRC = $(wildcard $(CSRC))
CPP_SRC      = $(wildcard $(CPPSRC))
CORE_C_SRC   = $(wildcard $(CORE_CSRC))
DRIVER_S_SRC = $(wildcard $(DRIVER_SSRC))
DRIVER_C_SRC = $(wildcard $(DRIVER_CSRC))
KERNEL_S_SRC = $(wildcard $(KERNEL_SSRC))
KERNEL_C_SRC = $(wildcard $(KERNEL_CSRC))
LIB_S_SRC    = $(wildcard $(LIB_SSRC))
LIB_C_SRC    = $(wildcard $(LIB_CSRC))
CMSIS_S_SRC  = $(wildcard $(CMSIS_SSRC))
CMSIS_C_SRC  = $(wildcard $(CMSIS_CSRC))
MBEDOS_S_SRC = $(wildcard $(MBEDOS_SSRC))
MBEDOS_C_SRC = $(wildcard $(MBEDOS_CSRC))
MBEDOS_CPP_SRC = $(wildcard $(MBEDOS_CPPSRC))

DOBJECTS        = $(C_SRC:%.c=%.o) $(S_SRC:%.S=%.o) $(CPP_SRC:%.cpp=%.o)
CORE_DOBJECTS   = $(CORE_C_SRC:%.c=%.o)
DRIVER_DOBJECTS = $(DRIVER_C_SRC:%.c=%.o) $(DRIVER_S_SRC:%.S=%.o)
KERNEL_DOBJECTS = $(KERNEL_C_SRC:%.c=%.o) $(KERNEL_S_SRC:%.S=%.o)
LIB_DOBJECTS    = $(LIB_C_SRC:%.c=%.o) $(LIB_S_SRC:%.S=%.o)
CMSIS_DOBJECTS  = $(CMSIS_C_SRC:%.c=%.o) $(CMSIS_S_SRC:%.S=%.o)
MBEDOS_DOBJECTS = $(MBEDOS_C_SRC:%.c=%.o) $(MBEDOS_S_SRC:%.S=%.o) $(MBEDOS_CPP_SRC:%.cpp=%.o)

L_DEPS := $(DOBJECTS:%o=%d) $(CORE_DOBJECTS:%o=%d) $(DRIVER_DOBJECTS:%o=%d) $(KERNEL_DOBJECTS:%o=%d) $(LIB_DOBJECTS:%o=%d) \
          $(CMSIS_DOBJECTS:%o=%d) $(MBEDOS_DOBJECTS:%o=%d)

lib_sub: $(CORE_DOBJECTS) $(DRIVER_DOBJECTS) $(KERNEL_DOBJECTS) $(LIB_DOBJECTS) $(CMSIS_DOBJECTS) $(MBEDOS_DOBJECTS)
	$(Q)$(AR) $(ARFLAGS) $(OUTDIR)/$(PACKNAME) $(CORE_DOBJECTS) $(DRIVER_DOBJECTS) $(KERNEL_DOBJECTS) $(LIB_DOBJECTS)
	$(Q)$(AR) $(ARFLAGS) $(OUTDIR)/libcmsis.a $(CMSIS_DOBJECTS)
	$(Q)$(AR) $(ARFLAGS) $(OUTDIR)/libmbedos.a $(MBEDOS_DOBJECTS)

main_objs: $(DOBJECTS)

%.o:%.c
	@echo CC $<
	$(Q)$(CC) -MP -MMD $(CFLAGS)  -o $@ $<

%.o:%.S
	@echo AS $<
	$(Q)$(CC) -MP -MMD $(ASFLAGS) -o $@ $<

%.o:%.cpp
	@echo CPP $<
	$(Q)$(CPP) -MP -MMD $(CFLAGS)  -o $@ $<

sinclude $(L_DEPS)

$(NAME).elf: lib_sub main_objs $(LDDIR)/gcc_csky.ld
	@echo LINK $@
	$(Q)$(CC)  -T $(LDDIR)/gcc_csky.ld -o $(OUTDIR)/$(NAME).elf $(CPPLDFLAG_BEGIN) $(LDFLAGS) \
		$(DOBJECTS) $(EOBJECTS) $(CORE_DOBJECTS) $(DRIVER_DOBJECTS) $(KERNEL_DOBJECTS) $(LIB_DOBJECTS) \
		$(CMSIS_DOBJECTS) $(MBEDOS_DOBJECTS) \
		$(SUBDIR_FILES) $(LDLIBS) $(CPPLDFLAG_END) -nostartfiles
	@-cp $(DOBJECTS) $(OBJDIR)
	@echo DUMP $@ to $(NAME).asm
	$(Q)$(DUMP) -S $(OUTDIR)/$(NAME).elf > $(OUTDIR)/$(NAME).asm
	@echo OBJCOPY $@ to $(NAME).bin
	$(Q)$(OBJCOPY) -O binary $(OUTDIR)/$(NAME).elf $(OUTDIR)/$(NAME).bin
	$(Q)$(OBJCOPY) -O srec $(OUTDIR)/$(NAME).elf $(OUTDIR)/$(NAME).hex

.PHONY : debug-make

DEBUG_VARS = 	SUB_DIRS \
		SUBDIR_FILES\
		INCLUDEDIRS \
		CSRC \
		SSRC \
		EOBJECTS \
		DOBJECTS \
		ASFLAGS \
		CFLAGS
#:
#: debug-make -- Print a list of Makefile variables

debug-make:
	@$(foreach var, $(DEBUG_VARS), echo $(var)=$($(var)) ; )

test_core:
	sh ./products/unit_test/core_test/testsuite/core_test.sh

endif

###############
clean:
	rm -rf $(OUTDIR)
#	rm -f  $(DOBJECTS)
#	rm -f  $(L_DEPS)
	find . -name "*.[od]" -delete
	rm out -fr
#	rm -f include/csi_config.h
	find . -name tee.ld.S | xargs -i  dirname {} | xargs -I {} make -C {} clean

tee_os:
	@make -C $(ROOTDIR)/csi_driver/$(CONFIG_CHIP_VENDOR_NAME)/common/tee \
	TEEOS_LIB_PATH=$(ABSROOTDIR)/$(CHIPDIR)/tee/ \
	TEEOS_LIB=libtee_os_$(CONFIG_CHIP_NAME).a \
	TARGETS_ROOT_PATH=$(TARGETS_ROOT_PATH)


