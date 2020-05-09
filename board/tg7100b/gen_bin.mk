
# MCU_FILE := $(PWD)/platform/mcu/$(HOST_MCU_FAMILY)

EXTRA_POST_BUILD_TARGETS += gen_bin

MK_GENERATED_IMGS_PATH =  $(OUTPUT_DIR)/binary

CPRE := @
ifeq ($(V),1)
CPRE :=
endif
EXE_EXT:=$(shell which ls | grep -o .exe)

gen_bin:
	@echo [INFO] Create bin files	
	@mkdir $(MK_GENERATED_IMGS_PATH) $(MK_GENERATED_IMGS_PATH)/data -p
	$(CPRE) mkdir -p $(MK_GENERATED_IMGS_PATH)/data/
	$(CPRE) cp $(SOURCE_ROOT)/board/tg7100b/bootimgs/boot $(MK_GENERATED_IMGS_PATH)/data/
	$(CPRE) cp $(SOURCE_ROOT)/board/tg7100b/bootimgs/bomtb $(MK_GENERATED_IMGS_PATH)/data/
	$(CPRE) cp $(SOURCE_ROOT)/board/tg7100b/script/genbin.sh $(MK_GENERATED_IMGS_PATH)/data/
	$(CPRE) sh $(MK_GENERATED_IMGS_PATH)/data/genbin.sh $(MK_GENERATED_IMGS_PATH)/data $(LINK_OUTPUT_FILE)
	$(CPRE) cp $(SOURCE_ROOT)/board/tg7100b/configs/config.yaml $(MK_GENERATED_IMGS_PATH)/data
	$(CPRE) $(SOURCE_ROOT)/board/tg7100b/product$(EXE_EXT) image $(MK_GENERATED_IMGS_PATH)/images.zip -i $(MK_GENERATED_IMGS_PATH)/data -p -l -v "v1.0"
	$(CPRE) $(SOURCE_ROOT)/board/tg7100b/product$(EXE_EXT) image $(MK_GENERATED_IMGS_PATH)/images.zip -e $(MK_GENERATED_IMGS_PATH) -x
	$(CPRE) cp $(MK_GENERATED_IMGS_PATH)/total_image.hex $(MK_GENERATED_IMGS_PATH)/total_image.hexf
	$(CPRE) rm -fr $(MK_GENERATED_IMGS_PATH)/data
	$(CPRE) $(SOURCE_ROOT)/board/tg7100b/product$(EXE_EXT) diff -f $(MK_GENERATED_IMGS_PATH)/images.zip $(MK_GENERATED_IMGS_PATH)/images.zip -r -v "v1.0" -o $(MK_GENERATED_IMGS_PATH)/fota.bin

