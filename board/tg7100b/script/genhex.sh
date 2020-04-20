#flash
WORK_PATH=$1
rm -f $WORK_PATH/*.hex
arm-none-eabi-objcopy -j .text -j .rodata -j .ARM.exidx -O binary $WORK_PATH/yoc.elf $WORK_PATH/one.bin
arm-none-eabi-objcopy -j .data_text  -j .data -O binary $WORK_PATH/yoc.elf $WORK_PATH/two.bin
arm-none-eabi-objcopy -j .data_text1 -O binary $WORK_PATH/yoc.elf $WORK_PATH/two1.bin
arm-none-eabi-objcopy -j .jmp_table -O binary $WORK_PATH/yoc.elf $WORK_PATH/three.bin
arm-none-eabi-objcopy -I binary -O ihex $WORK_PATH/one.bin $WORK_PATH/one.hex --change-address=0x11032000
arm-none-eabi-objcopy -I binary -O ihex $WORK_PATH/two.bin $WORK_PATH/two.hex --change-address=0x1fff4800
arm-none-eabi-objcopy -I binary -O ihex $WORK_PATH/two1.bin $WORK_PATH/two1.hex --change-address=0x20000000
arm-none-eabi-objcopy -I binary -O ihex $WORK_PATH/three.bin $WORK_PATH/three.hex --change-address=0x1fff0800
sed -i '/:00000001FF/d' $WORK_PATH/one.hex
sed -i '/:00000001FF/d' $WORK_PATH/two.hex
sed -i '/:00000001FF/d' $WORK_PATH/two1.hex
cat $WORK_PATH/one.hex > $WORK_PATH/yoc.hex
cat $WORK_PATH/two.hex >> $WORK_PATH/yoc.hex
cat $WORK_PATH/two1.hex >> $WORK_PATH/yoc.hex
cat $WORK_PATH/three.hex >> $WORK_PATH/yoc.hex
