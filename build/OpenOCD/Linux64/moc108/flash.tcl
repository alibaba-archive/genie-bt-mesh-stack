proc memread32 {address} {
    mem2array memar 32 $address 1
    return $memar(0)
}

proc load_image_bin {fname foffset address length } {
    # Load data from fname filename at foffset offset to
    # target at address. Load at most length bytes.
    puts "loadimage address $address foffset $foffset $length"
    load_image $fname [expr $address - $foffset] bin $address $length
}

proc flash_boot_check { } {
    halt
    set boot [memread32 0x20]
    if { $boot == 0x2E302E31 } {
        error "Bootloader version is too low, v1.1.0 or higher required."
        exit -1;
    }
}

proc flash_program { file_name dest_addr } {

    soft_reset_halt
    mww 0x400018 0xAAAAAAAA
    mww 0x40001C 3 

    set updata_ram     0x00400020
    set buffer_size     0x800

    set update_dest_addr       [expr $updata_ram + 0x00]
    set update_file_size       [expr $updata_ram + 0x04]
    set update_stream_size     [expr $updata_ram + 0x08]
    set update_status          [expr $updata_ram + 0x0C]
    set update_data_buffer     [expr $updata_ram + 0x10]

    set UPDATE_STATUS_READY      1
    set UPDATE_STATUS_SUCCESS    2
    set UPDATE_STATUS_ERROR      3

    set pos 0
    set bin_file_size [file size $file_name]

    puts "Write $file_name to flash, total size is $bin_file_size"
    mww $update_dest_addr $dest_addr
    mww $update_file_size $bin_file_size
    while { $pos < $bin_file_size } {

        if { ($bin_file_size - $pos) <  $buffer_size } {
            set writesize [expr ($bin_file_size - $pos)]
        } else {
            set writesize $buffer_size
        }

        puts "Writing $writesize bytes at [expr $dest_addr + $pos]"
        # Load the binary data into the RAM
        load_image_bin $file_name $pos $update_data_buffer $writesize
        mww $update_stream_size $writesize
        mww $update_status $UPDATE_STATUS_READY

        set loops  0
        set resultval $UPDATE_STATUS_READY
        while { ($resultval == $UPDATE_STATUS_READY) && ( $loops < 100 ) } {
            resume
            sleep 10
            halt
            set resultval [memread32 $update_status]
            incr loops
        }

        puts "**************** Result: $resultval"

        if { $resultval != $UPDATE_STATUS_SUCCESS } {
            reg
            error "program_sflash error, Result: $resultval"
            exit -1;
        }

        set pos [expr $pos + $writesize]
    }
}