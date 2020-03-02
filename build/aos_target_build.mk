include $(MAKEFILES_PATH)/aos_host_cmd.mk

CONFIG_FILE := $(OUTPUT_DIR)/config.mk

include $(CONFIG_FILE)

# Include all toolchain makefiles - one of them will handle the architecture
# default gcc
ifeq ($(COMPILER),)
include $(MAKEFILES_PATH)/aos_toolchain_gcc.mk
else ifeq ($(COMPILER),gcc)
include $(MAKEFILES_PATH)/aos_toolchain_gcc.mk
else ifeq ($(COMPILER),armcc)
include $(MAKEFILES_PATH)/aos_toolchain_armcc.mk
else ifeq ($(COMPILER),rvct)
include $(MAKEFILES_PATH)/aos_toolchain_rvct.mk
else ifeq ($(COMPILER),iar)
include $(MAKEFILES_PATH)/aos_toolchain_iar.mk
endif

.PHONY: display_map_summary build_done  

##################################
# Filenames
##################################

LINK_OUTPUT_FILE          :=$(OUTPUT_DIR)/binary/$(CLEANED_BUILD_STRING)$(RADIXPOINT)$(BINSTYPE_LOWER)$(LINK_OUTPUT_SUFFIX)
# out/helloworld@xx/binary/helloworld@xx.elf
STRIPPED_LINK_OUTPUT_FILE :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.stripped$(LINK_OUTPUT_SUFFIX))
# out/helloworld@xx/binary/helloworld@xx.stripped.elf
BIN_OUTPUT_FILE           :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=$(BIN_OUTPUT_SUFFIX))
# out/helloworld@xx/binary/helloworld@xx.bin
HEX_OUTPUT_FILE           :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=$(HEX_OUTPUT_SUFFIX))
# out/helloworld@xx/binary/helloworld@xx.bin
MAP_OUTPUT_FILE           :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.map)
# out/helloworld@xx/binary/helloworld@xx.map
MAP_CSV_OUTPUT_FILE       :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=_map.csv)
# out/helloworld@xx/binary/helloworld@xx_map.csv
LST_OUTPUT_FILE           :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.lst)
# out/helloworld@xx/binary/helloworld@xx.lst

ifeq ($(PING_PONG_OTA),1)
LINK_OUTPUT_FILE_XIP2     :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.xip2$(LINK_OUTPUT_SUFFIX))
STRIPPED_LINK_OUTPUT_FILE_XIP2 :=$(LINK_OUTPUT_FILE_XIP2:$(LINK_OUTPUT_SUFFIX)=.stripped$(LINK_OUTPUT_SUFFIX))
BIN_OUTPUT_FILE_XIP2      :=$(BIN_OUTPUT_FILE:$(BIN_OUTPUT_SUFFIX)=.xip2$(BIN_OUTPUT_SUFFIX))
endif

OPENOCD_LOG_FILE          ?= $(OUTPUT_DIR)/openocd_log.txt

LIBS_DIR                  := $(OUTPUT_DIR)/libraries
LINK_OPTS_FILE            := $(OUTPUT_DIR)/binary/link$(UNDERLINE)$(BINSTYPE_LOWER).opts

LINT_OPTS_FILE            := $(OUTPUT_DIR)/binary/lint$(UNDERLINE)$(BINSTYPE_LOWER).opts

LDS_FILE_DIR              := $(OUTPUT_DIR)/ld

ifeq (,$(SUB_BUILD))
ifneq (,$(EXTRA_TARGET_MAKEFILES))
$(foreach makefile_name,$(EXTRA_TARGET_MAKEFILES),$(eval include $(makefile_name)))
endif
endif

ifneq (,$(BINS))
CREATE_SYSCALLFILE :=$(MAKEFILES_PATH)/scripts/gen_syscalls.py
PARSE_RESOURSE_TO_SYSCALL_FILE = $(PYTHON) $(CREATE_SYSCALLFILE) $(1) $(2)
PROCESS_PRECOMPILED_FILES := $(OUTPUT_DIR)/precompile/mark.i
endif

include $(MAKEFILES_PATH)/aos_resources.mk
include $(MAKEFILES_PATH)/aos_images_download.mk

##################################
# Macros
##################################

###############################################################################
# MACRO: GET_BARE_LOCATION
# Returns a the location of the given component relative to source-tree-root
# rather than from the cwd
# $(1) is component
GET_BARE_LOCATION =$(patsubst $(call ESCAPE_BACKSLASHES,$(SOURCE_ROOT))%,%,$(strip $(subst :,/,$($(1)_LOCATION))))

define SELF_BUILD_RULE
$(LIBS_DIR)/$(notdir $($(1)_SELF_BUIlD_COMP_targets)): $(OUTPUT_DIR)/config.mk
	echo CONFIG_ENV_CFLAGS += $(RESOURCE_CFLAGS) > $($(1)_LOCATION)iotx-sdk-c_clone/aos_board_conf.mk
	echo CROSS_PREFIX := $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX) >> $($(1)_LOCATION)iotx-sdk-c_clone/aos_board_conf.mk
	sh $($(1)_LOCATION)$($(1)_SELF_BUIlD_COMP_scripts) $(LIBS_DIR)  $(SOURCE_ROOT)app/example/$(APP_FULL)
endef


###############################################################################
# MACRO: BUILD_C_RULE
# Creates a target for building C language files (*.c)
# $(1) is component, $(2) is the source file
define BUILD_C_RULE
ifeq ($(COMPILER),)
-include $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(2:.c=.d)
endif
$(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(2:.c=.o): $(strip $($(1)_LOCATION))$(2) $(CONFIG_FILE) $$(dir $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(2)).d $(RESOURCES_DEPENDENCY) $(LIBS_DIR)/$(1).c_opts $(PROCESS_PRECOMPILED_FILES) | $(EXTRA_PRE_BUILD_TARGETS)
	$$(if $($(1)_START_PRINT),,$(eval $(1)_START_PRINT:=1) $(QUIET)$(ECHO) Compiling $(1) )
	$(QUIET)$(CC) $($(1)_C_OPTS) -D__FILENAME__='"$$(notdir $$<)"' $(call COMPILER_SPECIFIC_DEPS_FILE,$(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(2:.c=.d)) -o $$@ $$< $(COMPILER_SPECIFIC_STDOUT_REDIRECT)
endef

###############################################################################
# MACRO: CHECK_HEADER_RULE
# Compiles a C language header file to ensure it is stand alone complete
# $(1) is component, $(2) is the source header file
define CHECK_HEADER_RULE
$(eval $(1)_CHECK_HEADER_LIST+=$(OUTPUT_DIR)/Modules/$(strip $($(1)_LOCATION))$(2:.h=.chk) )
.PHONY: $(OUTPUT_DIR)/Modules/$(strip $($(1)_LOCATION))$(2:.h=.chk)
$(OUTPUT_DIR)/Modules/$(strip $($(1)_LOCATION))$(2:.h=.chk): $(strip $($(1)_LOCATION))$(2) $(CONFIG_FILE) $$(dir $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(2)).d
	$(QUIET)$(ECHO) Checking header  $(2)
	$(QUIET)$(CC) -c $(AOS_SDK_CFLAGS) $(filter-out -pedantic -Werror, $($(1)_CFLAGS) $(C_BUILD_OPTIONS) ) $($(1)_INCLUDES) $($(1)_DEFINES) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES) -o $$@ $$<
endef

###############################################################################
# MACRO: BUILD_CPP_RULE
# Creates a target for building C++ language files (*.cpp)
# $(1) is component name, $(2) is the source file
define BUILD_CPP_RULE
-include $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(patsubst %.cc,%.d,$(2:.cpp=.d))
$(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(patsubst %.cc,%.o,$(2:.cpp=.o)): $(strip $($(1)_LOCATION))$(2) $(CONFIG_FILE) $$(dir $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(2)).d $(RESOURCES_DEPENDENCY) $(LIBS_DIR)/$(1).cpp_opts | $(EXTRA_PRE_BUILD_TARGETS)
	$$(if $($(1)_START_PRINT),,$(eval $(1)_START_PRINT:=1) $(ECHO) Compiling $(1))
	$(QUIET)$(CXX) $($(1)_CPP_OPTS) -o $$@ $$< $(COMPILER_SPECIFIC_STDOUT_REDIRECT)
endef

###############################################################################
# MACRO: BUILD_S_RULE
# Creates a target for building Assembly language files (*.s & *.S)
# $(1) is component name, $(2) is the source file
define BUILD_S_RULE
$(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(strip $(patsubst %.S,%.o, $(2:.s=.o) )): $(strip $($(1)_LOCATION))$(2) $($(1)_PRE_BUILD_TARGETS) $(CONFIG_FILE) $$(dir $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))$(strip $(patsubst %.S, %.o, $(2)))).d $(RESOURCES_DEPENDENCY) $(LIBS_DIR)/$(1).as_opts $(PROCESS_PRECOMPILED_FILES) | $(EXTRA_PRE_BUILD_TARGETS)
	$$(if $($(1)_START_PRINT),,$(eval $(1)_START_PRINT:=1) $(ECHO) Compiling $(1))
	$(QUIET)$(AS) $($(1)_S_OPTS) -o $$@ $$< $(COMPILER_SPECIFIC_STDOUT_REDIRECT)
endef

IDE_IAR_FLAG :=
IDE_KEIL_FLAG :=

ifeq ($(IDE),iar)
IDE_IAR_FLAG := 1
else ifeq ($(IDE),keil)
IDE_KEIL_FLAG := 1
endif

###############################################################################
# MACRO: BUILD_COMPONENT_RULES
# Creates targets for building an entire component
# Target for the component static library is created in this macro
# Targets for source files are created by calling the macros defined above
# $(1) is component name
define BUILD_COMPONENT_RULES

$(eval LINK_LIBS +=$(if $($(1)_SOURCES),$(LIBS_DIR)/$(1).a))
$(eval LINK_LIBS +=$(if $($(1)_SELF_BUIlD_COMP_targets),$(LIBS_DIR)/$(notdir $($(1)_SELF_BUIlD_COMP_targets) )))

ifneq ($($(1)_PRE_BUILD_TARGETS),)
include $($(1)_MAKEFILE)
endif

# Make a list of the object files that will be used to build the static library
$(eval $(1)_LIB_OBJS := $(addprefix $(strip $(OUTPUT_DIR)/Modules/$(call GET_BARE_LOCATION,$(1))),  $(filter %.o, $($(1)_SOURCES:.cc=.o) $($(1)_SOURCES:.cpp=.o) $($(1)_SOURCES:.c=.o) $($(1)_SOURCES:.s=.o) $($(1)_SOURCES:.S=.o)))  $(patsubst %.c,%.o,$(call RESOURCE_FILENAME, $($(1)_RESOURCES))))


$(LIBS_DIR)/$(1).c_opts: $($(1)_PRE_BUILD_TARGETS) $(CONFIG_FILE) | $(LIBS_DIR)
	$(eval $(1)_C_OPTS:=$(subst $(COMMA),$$(COMMA), $(COMPILER_SPECIFIC_COMP_ONLY_FLAG) $(COMPILER_SPECIFIC_DEPS_FLAG) $(COMPILER_UNI_CFLAGS) $($(1)_CFLAGS) $($(1)_INCLUDES) $($(1)_DEFINES) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES)))
	$(eval C_OPTS_IAR := $(subst =\",="\",$($(1)_C_OPTS)) ) 
	$(eval C_OPTS_IAR := $(subst \" ,\"" ,$(C_OPTS_IAR) ) )
	$(eval C_OPTS_IAR := $(filter-out -I% --cpu=% --endian% --dlib_config%,$(C_OPTS_IAR)) )
	$(eval C_OPTS_KEIL := $(subst -I.,-I../../../../.,$($(1)_C_OPTS)) )
	$(eval C_OPTS_FILE := $($(1)_C_OPTS) )
	$(if $(IDE_IAR_FLAG),$(eval C_OPTS_FILE:=$(C_OPTS_IAR)),)
	$(if $(IDE_KEIL_FLAG),$(eval C_OPTS_FILE:=$(C_OPTS_KEIL)),)
	$$(call WRITE_FILE_CREATE, $$@, $(C_OPTS_FILE))
	$$(file >$$@, $(C_OPTS_FILE) )

$(LIBS_DIR)/$(1).cpp_opts: $($(1)_PRE_BUILD_TARGETS) $(CONFIG_FILE) | $(LIBS_DIR)
	$(eval $(1)_CPP_OPTS:=$(COMPILER_SPECIFIC_COMP_ONLY_FLAG) $(COMPILER_SPECIFIC_DEPS_FLAG) $($(1)_CXXFLAGS)  $($(1)_INCLUDES) $($(1)_DEFINES) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES))
	$$(file >$$@, $($(1)_CPP_OPTS) )

$(LIBS_DIR)/$(1).as_opts: $(CONFIG_FILE) | $(LIBS_DIR)
	$(eval $(1)_S_OPTS:=$(CPU_ASMFLAGS) $(COMPILER_SPECIFIC_COMP_ONLY_FLAG) $(COMPILER_UNI_SFLAGS) $($(1)_ASMFLAGS) $($(1)_INCLUDES) $(AOS_SDK_INCLUDES))
	$(eval S_OPTS_KEIL := $(subst -I.,-I../../../../., $($(1)_S_OPTS) ) )
	$(eval S_OPTS_IAR := $(filter-out --cpu Cortex-M4, $($(1)_S_OPTS) ) )
	$(eval S_OPTS_FILE := $($(1)_S_OPTS) )    
	$(if $(IDE_KEIL_FLAG),$(eval S_OPTS_FILE:=$(S_OPTS_KEIL)),)
	$(if $(IDE_IAR_FLAG),$(eval S_OPTS_FILE:=$(S_OPTS_IAR)),)
	$$(file >$$@, $(S_OPTS_FILE) )

$(LIBS_DIR)/$(1).ar_opts: $(CONFIG_FILE) | $(LIBS_DIR)
	$(QUIET)$$(call WRITE_FILE_CREATE, $$@ ,$($(1)_LIB_OBJS))


# Allow checking of completeness of headers
$(foreach src, $(if $(findstring 1,$(CHECK_HEADERS)), $(filter %.h, $($(1)_CHECK_HEADERS)), ),$(eval $(call CHECK_HEADER_RULE,$(1),$(src))))

# Target for build-from-source
#$(OUTPUT_DIR)/libraries/$(1).a: $$($(1)_LIB_OBJS) $($(1)_CHECK_HEADER_LIST) $(OUTPUT_DIR)/libraries/$(1).ar_opts $$(if $(AOS_BUILT_WITH_ROM_SYMBOLS),$(ROMOBJCOPY_OPTS_FILE))
$(LIBS_DIR)/$(1).a: $$($(1)_LIB_OBJS) $($(1)_CHECK_HEADER_LIST) $(OUTPUT_DIR)/libraries/$(1).ar_opts
	$(ECHO) Making $$@
	$(QUIET)$(AR) $(AOS_SDK_ARFLAGS) $(COMPILER_SPECIFIC_ARFLAGS_CREATE) $$@ $(OPTIONS_IN_FILE_OPTION_PREFIX)$(OPTIONS_IN_FILE_OPTION)$(OUTPUT_DIR)/libraries/$(1).ar_opts$(OPTIONS_IN_FILE_OPTION_SUFFIX)
ifeq ($(COMPILER),)
	$(QUIET)$(STRIP) -g -o $(OUTPUT_DIR)/libraries/$(1).stripped.a $(OUTPUT_DIR)/libraries/$(1).a
endif
# Create targets to built the component's source files into object files
$(if $($(1)_SELF_BUIlD_COMP_scripts), $(eval $(call SELF_BUILD_RULE,$(1))) )
$(foreach src, $(filter %.c, $($(1)_SOURCES)),$(eval $(call BUILD_C_RULE,$(1),$(src))))
$(foreach src, $(filter %.cpp, $($(1)_SOURCES)) $(filter %.cc, $($(1)_SOURCES)),$(eval $(call BUILD_CPP_RULE,$(1),$(src))))
$(foreach src, $(filter %.s %.S, $($(1)_SOURCES)),$(eval $(call BUILD_S_RULE,$(1),$(src))))


$(eval $(1)_LINT_FLAGS +=  $(filter -D% -I%, $($(1)_CFLAGS) $($(1)_INCLUDES) $($(1)_DEFINES) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES) ) )
$(eval LINT_FLAGS +=  $($(1)_LINT_FLAGS) )
$(eval LINT_FILES +=  $(addprefix $(strip $($(1)_LOCATION)), $(filter %.c, $($(1)_SOURCES))) )
endef

define PROCESS_C_FILE
$(OUTPUT_DIR)/precompile/$(3)/$(call GET_BARE_LOCATION,$(1))$(2:.c=.i): $(strip $($(1)_LOCATION))$(2) $(CONFIG_FILE) $$(dir $(OUTPUT_DIR)/precompile/$(3)/$(call GET_BARE_LOCATION,$(1))$(2)).i
	$(QUIET)$(CPP) -P $(subst $(COMMA),$$(COMMA), $($(1)_CFLAGS) $($(1)_INCLUDES) $($(1)_DEFINES) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES)) -DAOS_EXPORTX -o $$@ $$<
$(eval PRECOMPILED_FILES += $(OUTPUT_DIR)/precompile/$(3)/$(call GET_BARE_LOCATION,$(1))$(2:.c=.i))
endef

define PROCESS_S_FILE
$(OUTPUT_DIR)/precompile/$(3)/$(call GET_BARE_LOCATION,$(1))$(strip $(patsubst %.S,%.i, $(2:.s=.i))): $(strip $($(1)_LOCATION))$(2) $(CONFIG_FILE) $$(dir $(OUTPUT_DIR)/precompile/$(3)/$(call GET_BARE_LOCATION,$(1))$(strip $(patsubst %.S, %.i, $(2:.s=.i)))).i
	$(QUIET)$(CPP) -P $(subst $(COMMA),$$(COMMA), $($(1)_CFLAGS) $($(1)_INCLUDES) $($(1)_DEFINES) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES)) -DAOS_EXPORTX -o $$@ $$<
$(eval PRECOMPILED_FILES += $(OUTPUT_DIR)/precompile/$(3)/$(call GET_BARE_LOCATION,$(1))$(strip $(patsubst %.S,%.i, $(2:.s=.i))))
endef

define PRECOMPILED_RESOURCE_FILE
$(foreach src, $(filter %.c, $($(1)_SOURCES)),$(eval $(call PROCESS_C_FILE,$(1),$(src),$(2))))
$(foreach src, $(filter %.s %.S, $($(1)_SOURCES)),$(eval $(call PROCESS_S_FILE,$(1),$(src),$(2))))
endef

define PROCESS_LDS_FILE
$(LDS_FILE_DIR)/$(notdir $(1:.ld.S=.ld)): $(LDS_FILE_DIR)
	$(ECHO) Making $$@
	$(QUIET)$(CPP) -P $(AOS_SDK_CFLAGS) $(AOS_SDK_INCLUDES) $(AOS_SDK_DEFINES) $(1) -o $$@

$(eval LDS_FILES += $(LDS_FILE_DIR)/$(notdir $(1:.ld.S=.ld)))
endef

##################################
# Processing
##################################

# Create targets for resource files
# $(info Resources: $(ALL_RESOURCES))
$(eval $(if $(ALL_RESOURCES),$(call CREATE_ALL_RESOURCE_TARGETS,$(ALL_RESOURCES))))
LINK_LIBS += $(RESOURCES_LIBRARY)

# $(info Components: $(COMPONENTS))
# Create targets for components
ifeq (app, $(BINS))
# precompile kernel/framework file
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter kernel, $($(comp)_TYPE)), $(call PRECOMPILED_RESOURCE_FILE,$(comp),kernel))))
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter framework, $($(comp)_TYPE)), $(call PRECOMPILED_RESOURCE_FILE,$(comp),framework))))
# Create targets for components
$(foreach comp,$(COMPONENTS),$(eval $(if $($(comp)_TYPE), $(if $(filter app app&framework app&kernel share, $($(comp)_TYPE)), $(call BUILD_COMPONENT_RULES,$(comp))), $(call BUILD_COMPONENT_RULES,$(comp)))))
else ifeq (framework, $(BINS))
# precompile kernel/framework file
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter kernel, $($(comp)_TYPE)), $(call PRECOMPILED_RESOURCE_FILE,$(comp),kernel))))
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter framework, $($(comp)_TYPE)), $(call PRECOMPILED_RESOURCE_FILE,$(comp),framework))))
# Create targets for components
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter framework app&framework framework&kernel share, $($(comp)_TYPE)), $(call BUILD_COMPONENT_RULES,$(comp)))))
else ifeq (kernel, $(BINS))
# precompile kernel file
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter kernel, $($(comp)_TYPE)), $(call PRECOMPILED_RESOURCE_FILE,$(comp),kernel))))
# Create targets for components
$(foreach comp,$(COMPONENTS),$(eval $(if $(filter kernel app&kernel framework&kernel share, $($(comp)_TYPE)), $(call BUILD_COMPONENT_RULES,$(comp)))))
else ifeq (,$(BINS))
$(foreach comp,$(COMPONENTS),$(eval $(call BUILD_COMPONENT_RULES,$(comp))))
endif

# handle lds file, lds -> ld
$(foreach ldsfile,$(AOS_SDK_LDS_FILES),$(eval $(call PROCESS_LDS_FILE,$(ldsfile))))
$(foreach ldsfile,$(AOS_SDK_LDS_INCLUDES),$(eval $(call PROCESS_LDS_FILE,$(ldsfile))))
$(foreach ldsfile,$(AOS_SDK_LDS_FILES),$(eval AOS_SDK_LDFLAGS += -T $(notdir $(ldsfile:.ld.S=.ld))))
$(if $(AOS_SDK_LDS_FILES),$(eval AOS_SDK_LDFLAGS += -L $(LDS_FILE_DIR)))

ifneq (,$(BINS))
$(PROCESS_PRECOMPILED_FILES): $(PRECOMPILED_FILES)
	$(QUIET)$(TOUCH) $@
	$(QUIET)$(if $(PARSE_RESOURSE_TO_SYSCALL_FILE), $(call PARSE_RESOURSE_TO_SYSCALL_FILE, $(OUTPUT_DIR), create))
endif

# Add pre-built libraries
LINK_LIBS += $(AOS_SDK_PREBUILT_LIBRARIES)

##################################
# Build rules
##################################

$(LIBS_DIR):
	$(QUIET)$(call MKDIR, $@)

$(LDS_FILE_DIR):
	$(QUIET)$(call MKDIR, $@)

# Directory dependency - causes mkdir to be called once for each directory.
%/.d:
	$(QUIET)$(call MKDIR, $(dir $@))
	$(QUIET)$(TOUCH) $(@)

%/.i:
	$(QUIET)$(call MKDIR, $(dir $@))

LINK_OPTS := $(AOS_SDK_LINK_SCRIPT_CMD) $(call COMPILER_SPECIFIC_LINK_MAP,$(MAP_OUTPUT_FILE))  $(call COMPILER_SPECIFIC_LINK_FILES, $(AOS_SDK_LINK_FILES) $(filter %.a,$^) $(LINK_LIBS)) $(AOS_SDK_LDFLAGS)
	
# FIXME GCC Whole archive not ready in all platform
$(LINK_OPTS_FILE): $(OUTPUT_DIR)/config.mk $(LDS_FILES)
ifeq ($(COMPILER),armcc)
	$(QUIET)$(call WRITE_FILE_CREATE, $@ ,$(LINK_OPTS))
else
	$(QUIET)$(call WRITE_FILE_CREATE, $@ ,$(LINK_OPTS) )
endif

$(LINT_OPTS_FILE): $(LINK_LIBS)
	$(QUIET)$(call WRITE_FILE_CREATE, $@ , )
	$(QUIET)$(foreach opt,$(sort $(subst \",",$(LINT_FLAGS))) $(sort $(LINT_FILES)),$(call WRITE_FILE_APPEND, $@ ,$(opt)))

define 	LINK_OUTPUT_FILE_OPTIONS_MACRO
LINK_OUTPUT_FILE_OPTIONS = $(OPTIONS_IN_FILE_OPTION_PREFIX)$(OPTIONS_IN_FILE_OPTION)$1$(OPTIONS_IN_FILE_OPTION_SUFFIX)
endef

ifeq ($(PING_PONG_OTA),1)
$(LINK_OUTPUT_FILE): $(LINK_LIBS) $(AOS_SDK_LINK_SCRIPT) $(LINK_OPTS_FILE) $(LINT_DEPENDENCY) | $(EXTRA_PRE_LINK_TARGETS)
	$(QUIET)$(ECHO) Making $(notdir $@)
	$(QUIET)$(LINKER) $(LINK_OPTS) $(COMPILER_SPECIFIC_STDOUT_REDIRECT) -T $(SOURCE_ROOT)/platform/mcu/rtl8710bn/script/rlx8711B-symbol-v02-img2_xip1.ld -o $@
	$(QUIET)$(ECHO_BLANK_LINE)
	$(QUIET)$(call COMPILER_SPECIFIC_MAPFILE_TO_CSV,$(MAP_OUTPUT_FILE),$(MAP_CSV_OUTPUT_FILE))

$(LINK_OUTPUT_FILE_XIP2): $(LINK_LIBS) $(AOS_SDK_LINK_SCRIPT) $(LINK_OPTS_FILE) $(LINT_DEPENDENCY) | $(EXTRA_PRE_LINK_TARGETS)
	$(QUIET)$(ECHO) Making $(notdir $@)
	$(QUIET)$(LINKER) $(LINK_OPTS) $(COMPILER_SPECIFIC_STDOUT_REDIRECT) -T $(SOURCE_ROOT)/platform/mcu/rtl8710bn/script/rlx8711B-symbol-v02-img2_xip2.ld -o $@
	$(QUIET)$(ECHO_BLANK_LINE)
	$(QUIET)$(call COMPILER_SPECIFIC_MAPFILE_TO_CSV,$(MAP_OUTPUT_FILE),$(MAP_CSV_OUTPUT_FILE))
else
$(LINK_OUTPUT_FILE): $(LINK_LIBS) $(AOS_SDK_LINK_SCRIPT) $(LINK_OPTS_FILE) $(LINT_DEPENDENCY) | $(EXTRA_PRE_LINK_TARGETS)
	$(QUIET)$(ECHO) Making $(notdir $@)
	$(QUIET)$(LINKER) $(LINK_OPTS) $(COMPILER_SPECIFIC_STDOUT_REDIRECT) -o $@
	$(QUIET)$(ECHO_BLANK_LINE)
	$(QUIET)$(call COMPILER_SPECIFIC_MAPFILE_TO_CSV,$(MAP_OUTPUT_FILE),$(MAP_CSV_OUTPUT_FILE))
endif

# Stripped elf file target - Strips the full elf file and outputs to a new .stripped.elf file
$(STRIPPED_LINK_OUTPUT_FILE): $(LINK_OUTPUT_FILE)
ifeq ($(COMPILER),iar)
	$(QUIET)$(STRIP) $(STRIPFLAGS) $< $(STRIP_OUTPUT_PREFIX)$@
else
	$(QUIET)$(STRIP) $(STRIP_OUTPUT_PREFIX)$@ $(STRIPFLAGS) $<
endif

PROJ_GEN_DIR   := projects/autogen/$(CLEANED_BUILD_STRING)

# Bin file target - uses objcopy to convert the stripped elf into a binary file
$(BIN_OUTPUT_FILE): $(STRIPPED_LINK_OUTPUT_FILE)
	$(QUIET)$(ECHO) Making $(notdir $@)
	$(QUIET)$(OBJCOPY) $(OBJCOPY_BIN_FLAGS) $< $(OBJCOPY_OUTPUT_PREFIX)$@ 
ifeq ($(TOOLCHAIN_DEFAULT_FOLDER), tc32)
	$(QUIET)$(ECHO) Making $(notdir $(LST_OUTPUT_FILE)) 
	$(OBJDUMP) $(OBJDUMP_LST_FLAGS) ${LINK_OUTPUT_FILE} > $(OBJCOPY_OUTPUT_PREFIX)$(LST_OUTPUT_FILE) 
	$(QUIET)$(ECHO) Adding Check $(notdir $(BIN_OUTPUT_FILE)) 
	$(CHECK_FW_TOOL) $(BIN_OUTPUT_FILE)
endif
ifeq ($(IDE),iar)
	echo copy iar opt files..
	$(QUIET)$(call MKDIR, $(PROJ_GEN_DIR)/iar_project/opts)
	$(QUIET)cp -rf $(OUTPUT_DIR)/libraries/*_opts $(PROJ_GEN_DIR)/iar_project/opts
else ifeq ($(IDE),keil)
	echo copy keil opt files..
	$(QUIET)$(call MKDIR, $(PROJ_GEN_DIR)/keil_project/opts)
	$(QUIET)cp -rf $(OUTPUT_DIR)/libraries/*_opts $(PROJ_GEN_DIR)/keil_project/opts
endif	
	
ifeq ($(PING_PONG_OTA),1)
$(STRIPPED_LINK_OUTPUT_FILE_XIP2): $(LINK_OUTPUT_FILE_XIP2)
	$(QUIET)$(STRIP) $(STRIP_OUTPUT_PREFIX)$@ $(STRIPFLAGS) $<

$(BIN_OUTPUT_FILE_XIP2): $(STRIPPED_LINK_OUTPUT_FILE_XIP2)
	$(QUIET)$(ECHO) Making $(notdir $@)
	$(QUIET)$(OBJCOPY) $(OBJCOPY_BIN_FLAGS) $< $(OBJCOPY_OUTPUT_PREFIX)$@ 
endif

$(HEX_OUTPUT_FILE): $(STRIPPED_LINK_OUTPUT_FILE)
	$(QUIET)$(ECHO) Making $(notdir $@)
	$(QUIET)$(OBJCOPY) $(OBJCOPY_HEX_FLAGS) $< $(OBJCOPY_OUTPUT_PREFIX)$@

# Linker output target - This links all component & resource libraries and objects into an output executable
# CXX is used for compatibility with C++
#$(AOS_SDK_CONVERTER_OUTPUT_FILE): $(LINK_OUTPUT_FILE)
#	$(QUIET)$(ECHO) Making $(notdir $@)
#	$(QUIET)$(CONVERTER) "--ihex" "--verbose" $(LINK_OUTPUT_FILE) $@

#$(AOS_SDK_FINAL_OUTPUT_FILE): $(AOS_SDK_CONVERTER_OUTPUT_FILE)
#	$(QUIET)$(ECHO) Making $(PYTHON_FULL_NAME) $(AOS_SDK_CHIP_SPECIFIC_SCRIPT) -i $(AOS_SDK_CONVERTER_OUTPUT_FILE) -o $(AOS_SDK_FINAL_OUTPUT_FILE)
#	$(QUIET)$(PYTHON_FULL_NAME) $(AOS_SDK_CHIP_SPECIFIC_SCRIPT) -i $(AOS_SDK_CONVERTER_OUTPUT_FILE) -o $(AOS_SDK_FINAL_OUTPUT_FILE)


ifeq ($(PING_PONG_OTA),1)
$(LINK_OUTPUT_FILE_XIP2): $(LINK_OUTPUT_FILE)
display_map_summary: $(LINK_OUTPUT_FILE_XIP2) $(AOS_SDK_CONVERTER_OUTPUT_FILE) $(AOS_SDK_FINAL_OUTPUT_FILE)
	$(QUIET) $(call COMPILER_SPECIFIC_MAPFILE_DISPLAY_SUMMARY,$(MAP_OUTPUT_FILE))
else
display_map_summary: $(LINK_OUTPUT_FILE) $(AOS_SDK_CONVERTER_OUTPUT_FILE) $(AOS_SDK_FINAL_OUTPUT_FILE)
	$(QUIET) $(call COMPILER_SPECIFIC_MAPFILE_DISPLAY_SUMMARY,$(MAP_OUTPUT_FILE))
endif

# Main Target - Ensures the required parts get built
# $(info Prebuild targets:$(EXTRA_PRE_BUILD_TARGETS))
# $(info $(BIN_OUTPUT_FILE))
ifeq ($(PING_PONG_OTA),1)
$(BIN_OUTPUT_FILE_XIP2): $(BIN_OUTPUT_FILE)
build_done: $(EXTRA_PRE_BUILD_TARGETS) $(BIN_OUTPUT_FILE_XIP2) $(HEX_OUTPUT_FILE) display_map_summary
else
build_done: $(EXTRA_PRE_BUILD_TARGETS) $(BIN_OUTPUT_FILE) $(HEX_OUTPUT_FILE) display_map_summary
endif

ifeq ($(post_run),1)
ifeq ($(HOST_OS),Win32)
FILE_SCRIPT := post_run.bat
POST_CMD := board\\${PLATFORM}\\$(FILE_SCRIPT) $(APP) $(PLATFORM) $(HOST_MCU_FAMILY)
endif

build_post_run: build_done
	$(QUIET)$(ECHO) Running $(POST_CMD)
	$(POST_CMD)
endif

$(EXTRA_POST_BUILD_TARGETS): build_done

ifeq ($(post_run),1)
$(BUILD_STRING): $(if $(EXTRA_POST_BUILD_TARGETS),$(EXTRA_POST_BUILD_TARGETS),build_done) build_post_run
else
$(BUILD_STRING): $(if $(EXTRA_POST_BUILD_TARGETS),$(EXTRA_POST_BUILD_TARGETS),build_done)
endif
