#!/bin/sh
BASE_PWD=`pwd`

MK_GENERATED_PATH=generated

echo "[INFO] Generated output files ..."

rm -fr $MK_GENERATED_PATH
mkdir -p $MK_GENERATED_PATH/data/

OBJCOPY=csky-elfabiv2-objcopy

#output yoc.bin
ELF_NAME=`ls Obj/*.elf`
$OBJCOPY -O binary $ELF_NAME $MK_GENERATED_PATH/data/prim

#create mtb
cp "$CDKPath/CSKY/SDK/Components/yoc7.0#yoc_kernel_ch2201/$yoc7_0_yoc_kernel_ch2201/com/boards/csky/cb2201/bootimgs/boot" $MK_GENERATED_PATH/data/
cp "$CDKPath/CSKY/SDK/Components/yoc7.0#yoc_kernel_ch2201/$yoc7_0_yoc_kernel_ch2201/com/boards/csky/cb2201/bootimgs/tee" $MK_GENERATED_PATH/data/
#cp tools/kv_tool/kv.bin $MK_GENERATED_PATH/data/kv

cp boards/cb2201/configs/config.yaml $MK_GENERATED_PATH/data/
"$CDKPath/CSKY/SDK/Components/yoc7.0#yoc_kernel_ch2201/$yoc7_0_yoc_kernel_ch2201/com/tools/build/product.exe" image $MK_GENERATED_PATH/images.zip -i $MK_GENERATED_PATH/data
"$CDKPath/CSKY/SDK/Components/yoc7.0#yoc_kernel_ch2201/$yoc7_0_yoc_kernel_ch2201/com/tools/build/product.exe" image $MK_GENERATED_PATH/images.zip -l
"$CDKPath/CSKY/SDK/Components/yoc7.0#yoc_kernel_ch2201/$yoc7_0_yoc_kernel_ch2201/com/tools/build/product.exe" image $MK_GENERATED_PATH/images.zip -e $MK_GENERATED_PATH -x
rm -fr $MK_GENERATED_PATH/data

#rm some tmp files
#rm gcc_eflash.ld

