#include <yoc_config.h>
#include <stdio.h>
#include <stdint.h>
#include <hal/soc/uart.h>

void board_base_init(void)
{
    uart_dev_t uart_0;
    uart_0.port                = STDIO_UART;
    uart_0.config.baud_rate    = 512000;
    uart_0.config.data_width   = DATA_WIDTH_8BIT;
    uart_0.config.parity       = NO_PARITY;
    uart_0.config.stop_bits    = STOP_BITS_1;
    uart_0.config.flow_control = FLOW_CONTROL_DISABLED;
    uart_0.config.mode = MODE_TX_RX;
    hal_uart_init(&uart_0);
}
