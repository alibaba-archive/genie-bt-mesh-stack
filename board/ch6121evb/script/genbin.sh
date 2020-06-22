#flash
WORK_PATH=$1
ELF_FILE=$2
rm -f $WORK_PATH/*.hex
#arm-none-eabi-objcopy -j .text -j .rodata -j .ARM.exidx -O binary $ELF_FILE $WORK_PATH/xprim
#arm-none-eabi-objcopy -j .data_text -j .data -O binary $ELF_FILE $WORK_PATH/prim
#arm-none-eabi-objcopy -j .jmp_table -O binary $ELF_FILE $WORK_PATH/jumptb


TOPDIR=`pwd`
TOOLCHAIN_DIR=$TOPDIR/build/compiler/gcc-arm-none-eabi/Linux64/bin
$TOOLCHAIN_DIR/arm-none-eabi-objcopy -j .text -j .rodata -j .ARM.exidx -O binary $ELF_FILE $WORK_PATH/xprim
$TOOLCHAIN_DIR/arm-none-eabi-objcopy -j .data_text -j .data -O binary $ELF_FILE $WORK_PATH/prim
$TOOLCHAIN_DIR/arm-none-eabi-objcopy -j .jmp_table -O binary $ELF_FILE $WORK_PATH/jumptb

