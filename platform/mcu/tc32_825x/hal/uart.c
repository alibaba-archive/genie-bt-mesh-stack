#include "hal/soc/soc.h"
#include "hal/soc/uart.h"
#include "drivers/8258/compiler.h"
#include "drivers/8258/uart.h"
#include "drivers/8258/dma.h"
#include "drivers/8258/irq.h"
#include "k_api.h"
#include "common/ring_buffer.h"
#include "tl_common.h"
#include "drivers.h"


#define UART_DMA  		1     //uart use dma
#define UART_NDMA  		2     //uart not use dma
#define UART_MODE		UART_DMA

volatile unsigned char uart_rx_flag         =   0;
volatile unsigned char uart_dmairq_tx_cnt   =   0;
volatile unsigned char uart_dmairq_rx_cnt   =   0;
volatile unsigned int  uart_ndmairq_cnt     =   0;
volatile unsigned char uart_ndmairq_index   =   0;

#if (UART_MODE==UART_DMA)
	#define UART_DATA_LEN_RX    (160-4)      //data max ?    (UART_DATA_LEN+4) must 16 byte aligned,if UART_DATA_LEN<140 cli have error!!!
    #define UART_DATA_LEN_TX    (160-4)      //data max ?    (UART_DATA_LEN+4) must 16 byte aligned,if UART_DATA_LEN<140 cli have error!!!

	typedef struct{
		unsigned int dma_len;        // dma len must be 4 byte
		unsigned char data[UART_DATA_LEN_RX];
	}uart_data_rx_t;
	
	typedef struct{
		unsigned int dma_len;        // dma len must be 4 byte
		unsigned char data[UART_DATA_LEN_TX];
	}uart_data_tx_t;

	_attribute_aligned_(16) uart_data_rx_t rec_buff =   {0,  {0, } };
	_attribute_aligned_(16) uart_data_tx_t send_buff =  {0,  {0, } };
	
	#if HW_UART_RING_BUF_EN
	unsigned char uart_send_rb_buffer[512]; // at least 256; // 0x75
	ring_buffer_t uart_send_rb = {
		.buf = uart_send_rb_buffer,
		.size = sizeof(uart_send_rb_buffer),
		.wptr = 0,
		.rptr = 0,
	};
	#endif
#elif(UART_MODE==UART_NDMA)
	
	#define rec_buff_Len    16
	#define trans_buff_Len  16
	__attribute__((aligned(4))) unsigned char rec_buff[rec_buff_Len]        ={0};
	__attribute__((aligned(4))) unsigned char trans_buff[trans_buff_Len]    
	= {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, \
	   0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};
#endif

//typedef void (* uart_rx_cb_t)(void);

void hal_uart_send_print(u16 len);
void uart_rx_proc(void);                   //fixme Registering Callbacks at irq funtion!!!!
//uart_rx_cb_t  uart_rx_handler_cb = NULL; //fixme Registering Callbacks at irq funtion!!!!
//extern uart_rx_cb_t  uart_rx_handler_cb;

#define USE_AOS_RING_BUF_FLOW_EN		0

#if USE_AOS_RING_BUF_FLOW_EN
#define MAX_BUF_UART_BYTES  256
kbuf_queue_t g_buf_queue_uart;
char g_buf_uart[MAX_BUF_UART_BYTES];
#else
#define MAX_BUF_UART_BYTES  128
unsigned char uart_recv_rb_buffer[MAX_BUF_UART_BYTES]; // at least 256; // 0x75
ring_buffer_t uart_recv_rb = {
	.buf = uart_recv_rb_buffer,
	.size = sizeof(uart_recv_rb_buffer),
	.wptr = 0,
	.rptr = 0,
};
#endif

int32_t hal_uart_init(uart_dev_t *uart){
	//note: dma addr must be set first before any other uart initialization! (confirmed by sihui)
	uart_recbuff_init( (unsigned short *)&rec_buff, sizeof(rec_buff));

	uart_gpio_set(DMA_UART_TX, DMA_UART_RX);// uart tx/rx pin set

	uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	//	uart_init(118, 13, PARITY_NONE, STOP_BIT_ONE);	//baud rate: 9600
	//	uart_init(9, 13, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 115200
	//	uart_init(6, 8, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 256000
		uart_init(0, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 1024000
	//uart_init(0, 7, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 2048000
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
    //	uart_init(12, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 115200
    //	uart_init(5, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 256000
    uart_init(1, 11, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 1024000
    //uart_init(0, 11, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 2048000
#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
	//	uart_init(30, 8, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 115200
	//	uart_init(24, 4, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 256000
		uart_init(1, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 1024000
	//uart_init(0, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 2048000
#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
	//	uart_init(25, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 115200
	//	uart_init(12, 15, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 230400
    //  uart_init(12, 7, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 460800
		uart_init(12, 3, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 921600
	//uart_init(1, 11, PARITY_NONE, STOP_BIT_ONE);    //baud rate: 2048000
#endif

#if (UART_MODE==UART_DMA)
	uart_dma_enable(1, 1); 	//uart data in hardware buffer moved by dma, so we need enable them first
	irq_set_mask(FLD_IRQ_DMA_EN);
	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 1);   	//uart Rx/Tx dma irq enable
	uart_irq_enable(0, 0);  	//uart Rx/Tx irq no need, disable them
#elif(UART_MODE==UART_NDMA)
	uart_dma_enable(0, 0);
	irq_clr_mask(FLD_IRQ_DMA_EN);
	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 0);
	uart_irq_enable(1,0);   //uart RX irq enable
	uart_ndma_irq_triglevel(1,0);   //set the trig level. 1 indicate one byte will occur interrupt
#endif
#if USE_AOS_RING_BUF_FLOW_EN
	krhino_buf_queue_create(&g_buf_queue_uart, "buf_queue_uart",g_buf_uart, MAX_BUF_UART_BYTES, 1);
#endif
	//uart_rx_handler_cb =  uart_rx_proc;
	//irq_enable();
    return 0;
}

_attribute_ram_code_ int32_t hal_uart_send(uart_dev_t *uart, const void *data, uint32_t size, uint32_t timeout_us)
{
    //int i = 0;
    //uint8_t  byte;
	int8_t  send_flag = 0;
#if (UART_MODE==UART_DMA)
#if HW_UART_RING_BUF_EN
	uint8_t r = irq_disable();
	send_flag = ring_buffer_write(&uart_send_rb, data, size);
	if((size > 1) // aos may call hal uart send directly in cli_putstr
	|| ring_buffer_get_count(&uart_send_rb) > min(sizeof(uart_send_rb_buffer)/2,UART_DATA_LEN_TX)){
	    hal_uart_send_print(size);
	}
	irq_restore(r);
#else
	send_buff.dma_len = size;
	memcpy(&send_buff.data,(unsigned short*)data,size);
	
	while(1){
		send_flag = uart_dma_send( (unsigned short*)&send_buff);
		if((0 == send_flag) && timeout_us){
            sleep_us(1);
            timeout_us --;
		}else{
		    break;
		}
	};
#endif
	
#else
    for(i = 0; i < size; i++)
    {
        byte = *((uint8_t *)data + i);
        uart_ndma_send_byte(byte);
    }
#endif
	if(send_flag==0)
		send_flag = -1;
    return send_flag;
}


#if HW_UART_RING_BUF_EN
void hal_uart_send_loop(int wait_flag)
{
    if(wait_flag){
        unsigned int tick = clock_time();
        while(!(reg_uart_status1 & FLD_UART_TX_DONE ) && !clock_time_exceed(tick, 2000)){ // busy
        }
    }
    
    uint8_t r = irq_disable();    
	if (reg_uart_status1 & FLD_UART_TX_DONE ){
		if ( ring_buffer_get_count(&uart_send_rb) > 0 ){
			send_buff.dma_len = ring_buffer_read(&uart_send_rb, send_buff.data, sizeof(send_buff.data));
			uart_dma_send( (unsigned short*)&send_buff);
		}
	}else{
	    // static unsigned short RingBufUartBusy;RingBufUartBusy++;
	}
    irq_restore(r);
}

void hal_uart_send_print(u16 len)
{
    static unsigned short RingBufSimPrintfMax;
    if(len > RingBufSimPrintfMax){RingBufSimPrintfMax = len;}
    hal_uart_send_loop(0);  // no need to wait.
}
#endif


int32_t hal_uart_recv(uart_dev_t *uart, void *data, uint32_t expect_size, uint32_t timeout){
    uint8_t *pdata = (uint8_t *)data;
    //int i = 0;
    uint32_t rx_count = 0;
    int32_t ret = -1;
    //int32_t rev_size;

    if (data == NULL) {
        return -1;
    }
	
#if USE_AOS_RING_BUF_FLOW_EN
    for (i = 0; i < expect_size; i++)
    {
        ret = krhino_buf_queue_recv(&g_buf_queue_uart, RHINO_NO_WAIT, &pdata[i], &rev_size);
        if((ret == 0) && (rev_size == 1))
        {
            rx_count++;
        }else {
            break;
        }

    }
#else
	rx_count = ring_buffer_read(&uart_recv_rb, pdata, expect_size);
#endif


    if(rx_count != 0)
    {
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

int32_t hal_uart_recv_II(uart_dev_t *uart, void *data, uint32_t expect_size,uint32_t *recv_size, uint32_t timeout){
	uint8_t *pdata = (uint8_t *)data;
    //int i = 0;
    uint32_t rx_count = 0;
    int32_t ret = -1;
    //int32_t rev_size;

    if (data == NULL) {
        return -1;
    }

#if USE_AOS_RING_BUF_FLOW_EN
    for (i = 0; i < expect_size; i++)
    {
        ret = krhino_buf_queue_recv(&g_buf_queue_uart, RHINO_NO_WAIT, &pdata[i], &rev_size);
        if((ret == 0) && (rev_size == 1))
        {
            rx_count++;
        }else {
            break;
        }

    }
#else
	rx_count = ring_buffer_read(&uart_recv_rb, pdata, expect_size);
#endif
    
    if (recv_size)
        *recv_size = rx_count;

    if(rx_count != 0)
    {
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

int32_t hal_uart_finalize(uart_dev_t *uart){
    return 0;
}

_attribute_ram_code_ void uart_rx_proc(void)
{
#if (UART_MODE==UART_DMA)

	unsigned char uart_dma_irqsrc;
	//1. UART irq
	uart_dma_irqsrc = dma_chn_irq_status_get();///in function,interrupt flag have already been cleared,so need not to clear DMA interrupt flag here
	if(uart_dma_irqsrc & FLD_DMA_CHN_UART_RX){
		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
		uart_dmairq_rx_cnt++;
	#if USE_AOS_RING_BUF_FLOW_EN
		for(int i=0;i<rec_buff.dma_len;i++)
			krhino_buf_queue_send(&g_buf_queue_uart, &(rec_buff.data[i]), 1);
	#else
		ring_buffer_write(&uart_recv_rb, rec_buff.data, rec_buff.dma_len);
	#endif
	}
	
	if(uart_dma_irqsrc & FLD_DMA_CHN_UART_TX){
		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_TX);
		uart_dmairq_tx_cnt++;
	}
#elif(UART_MODE==UART_NDMA)
    uint8_t bytedata[1] = {0x0a};
	static unsigned char uart_ndma_irqsrc;
	uart_ndma_irqsrc = uart_ndmairq_get();  ///get the status of uart irq.
	if(uart_ndma_irqsrc){
	//cycle the four registers 0x90 0x91 0x92 0x93,in addition reading will clear the irq.
		if(uart_rx_flag==0)
		{
			bytedata[0]=rec_buff[uart_ndmairq_cnt++] = reg_uart_data_buf(uart_ndmairq_index);
			_printf("uart_ndmairq_index %d\t bytedata:%x\n",uart_ndmairq_index,rec_buff[uart_ndmairq_cnt]);
			//bytedata = rec_buff[uart_ndmairq_cnt];
			//gpio_toggle(GPIO_PA3);
			//krhino_buf_queue_send(&g_buf_queue_uart, bytedata, 1);

			uart_ndmairq_index++;
			uart_ndmairq_index &= 0x03;// cycle the four registers 0x90 0x91 0x92 0x93, it must be done like this for the design of SOC.
			if(uart_ndmairq_cnt%4==0 && uart_ndmairq_cnt!=0){
				uart_rx_flag=1;
			#if USE_AOS_RING_BUF_FLOW_EN
				krhino_buf_queue_send(&g_buf_queue_uart, rec_buff, 16);
			#else
				ring_buffer_write(&uart_recv_rb, rec_buff, 16);
			#endif
			}
		}
		 else{
			read_reg8(0x90 + uart_ndmairq_index);
			uart_ndmairq_index++;
			uart_ndmairq_index &= 0x03;
			uart_ndmairq_cnt=0; //Clear uart_ndmairq_cnt
			uart_rx_flag=0;
		}
	}
#endif
}
