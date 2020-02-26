set MODE_STD     0
set MODE_DUAL    1
set MODE_QUAD    2

set FLASH_BASE                           (0x00803000)

set REG_FLASH_OPERATE_SW                 [expr $FLASH_BASE + 0 * 4]
set ADDR_SW_REG_POSI                     (0)
set ADDR_SW_REG_MASK                     (0xFFFFFF)
set OP_TYPE_SW_POSI                      (24)
set OP_TYPE_SW_MASK                      (0x1F)
set OP_SW                                [expr 0x01 << 29]
set WP_VALUE                             [expr 0x01 << 30]
set BUSY_SW                              [expr 0x01 << 31]

set REG_FLASH_DATA_SW_FLASH              [expr $FLASH_BASE + 1 * 4]

set REG_FLASH_DATA_FLASH_SW              [expr $FLASH_BASE + 2 * 4]

set REG_FLASH_RDID_DATA_FLASH            [expr $FLASH_BASE + 4 * 4]

set REG_FLASH_SR_DATA_CRC_CNT            [expr $FLASH_BASE + 5 * 4]
set SR_DATA_FLASH_POSI                   (0)
set SR_DATA_FLASH_MASK                   (0xFF)
set CRC_ERROR_COUNT_POSI                 (8)
set CRC_ERROR_COUNT_MASK                 (0xFF)
set DATA_FLASH_SW_SEL_POSI               (16)
set DATA_FLASH_SW_SEL_MASK               (0x07)
set DATA_SW_FLASH_SEL_POSI               (19)
set DATA_SW_FLASH_SEL_MASK               (0x07)

set REG_FLASH_CONF                       [expr $FLASH_BASE + 7 * 4]
set FLASH_CLK_CONF_POSI                  (0)
set FLASH_CLK_CONF_MASK                  (0x0F)
set MODEL_SEL_POSI                       (4)
set MODEL_SEL_MASK                       (0x1F)
set FWREN_FLASH_CPU                      [expr 0x01 << 9]
set WRSR_DATA_POSI                       (10)
set WRSR_DATA_MASK                       (0xFFFF)
set CRC_EN                               [expr 0x01 << 26]

set FLASH_OPCODE_WREN    1
set FLASH_OPCODE_WRDI    2
set FLASH_OPCODE_RDSR    3
set FLASH_OPCODE_WRSR    4
set FLASH_OPCODE_READ    5
set FLASH_OPCODE_RDSR2   6
set FLASH_OPCODE_WRSR2   7
set FLASH_OPCODE_PP      12
set FLASH_OPCODE_SE      13
set FLASH_OPCODE_BE1     14
set FLASH_OPCODE_BE2     15
set FLASH_OPCODE_CE      16
set FLASH_OPCODE_DP      17
set FLASH_OPCODE_RFDP    18
set FLASH_OPCODE_RDID    20
set FLASH_OPCODE_HPM     1
set FLASH_OPCODE_CRMR    22
set FLASH_OPCODE_CRMR2   23

proc read_reg { addr } {
    mem2array memar 32 $addr 1
    return $memar(0)
}

proc write_reg { addr val } {
    set memar(0) $val
    array2mem memar 32 $addr 1
}

proc flash_init { } {
    while { [expr [read_reg $::REG_FLASH_OPERATE_SW] & $::BUSY_SW] } { }

    write_reg $::REG_FLASH_CONF 0x00000215
}

proc flash_erase_sector { addr } {
    while { [expr [read_reg $::REG_FLASH_OPERATE_SW] & $::BUSY_SW] } { }

    set value [read_reg $::REG_FLASH_OPERATE_SW]
    set value [expr (($addr << $::ADDR_SW_REG_POSI)| ($::FLASH_OPCODE_SE << $::OP_TYPE_SW_POSI)| $::OP_SW | ($value & $::WP_VALUE))]
    write_reg $::REG_FLASH_OPERATE_SW $value
    while { [expr [read_reg $::REG_FLASH_OPERATE_SW] & $::BUSY_SW] } { }
}

proc flash_erase { addr size } {
    if { [expr $addr % 0x1000] || [expr $size % 0x1000] } {
         error "erase address $addr or size $size not aligned to 4K bytes"
    } else {
        while { $size } {
            flash_erase_sector $addr
            set addr [expr $addr + 0x1000]
            set size [expr $size - 0x1000]
        }
    }
}
