#!/bin/sh

MK_GENERATED_PATH=generated

"$CDKPath/CSKY/SDK/Components/yoc7.0#yoc_kernel_ch2201/$yoc7_0_yoc_kernel_ch2201/com/tools/build/gen_ldfile.sh" boards/cb2201/configs/config.yaml boards/cb2201/configs/gcc_eflash.ld.S gcc_eflash.ld

mkdir -p $MK_GENERATED_PATH
cp gcc_eflash.ld $MK_GENERATED_PATH -af