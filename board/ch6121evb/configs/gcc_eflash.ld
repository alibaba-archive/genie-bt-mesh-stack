/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     gcc_csky.ld
 * @brief    csky linker file
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/
MEMORY
{
    LR_IROM0   : ORIGIN = 0x1105D000, LENGTH = 0x1D000    /* FLASH 120KB*/
    LR_IRAM0   : ORIGIN = 0x1fff0000, LENGTH = 0x00800    /* on-chip SRAM0 2KB*/
    LR_IRAM1   : ORIGIN = 0x1fff0800, LENGTH = 0x00400    /* on-chip SRAM0 1KB*/
    LR_IRAM2   : ORIGIN = 0x1fff0c00, LENGTH = 0x00400    /* on-chip SRAM0 1KB*/
    LR_IRAM3   : ORIGIN = 0x1fff1000, LENGTH = 0x03800   /* on-chip SRAM0 14KB*/
    LR_IRAM4   : ORIGIN = 0x1fff4800, LENGTH = 0x1E000   /* on-chip SRAM0 120KB*/
}

__min_heap_size = 0x200;
PROVIDE (__ram_end  = 0x1fff4800 + 0x1E000);
PROVIDE (__heap_end = __ram_end);

REGION_ALIAS("REGION_TEXT",    LR_IROM0);
REGION_ALIAS("REGION_RODATA",  LR_IROM0);
REGION_ALIAS("REGION_RETENTION0",    LR_IRAM4);
REGION_ALIAS("REGION_RETENTION1",    LR_IRAM4);

ENTRY(Reset_Handler)
SECTIONS
{
 .data_text : {
  *startup.o(*.startup)
  KEEP(*startup.o(.vectors))
  *pm.o(.text.lpm_handle)
  *pm.o(.text.lpm_idle_hook)
  *pm.o(.text.pm_prepare_sleep_action)
  *pm.o(.text.enter_sleep_off_mode11.part.1)
  *pm.o(.text.pm_after_sleep_action)
  *pm.o(.text.config_RTC11)
  *pm.o(.text.enterSleepProcess11)
  *pm.o(.text.sys_soc_suspend)
  *pm.o(.text.sys_soc_resume)
  *mcu_ch6121.a:lpm_arch_reg_save.o(.text)
  *k_obj.o(.text.*)
  *mcu_ch6121.a:power_manager.o(.text.*)
  *mcu_ch6121.a:spif.o(.text.*)
  *mcu_ch6121.a:isr.o(.text.*)
  *mcu_ch6121.a:phy_pmu.o(.text.*)
  *mcu_ch6121.a:systick.o(.text.*)
  *mcu_ch6121.a:hook_impl.o(.text.*)
  *mcu_ch6121.a:hook_weak.o(.text.*)
  *mcu_ch6121.a:gpio.o(.text.*)
  *mcu_ch6121.a:dw_usart.o(.text.csi_usart_receive_query .text.csi_usart_send .text.csi_usart_putchar .text.dw_usart_intr_recv_data .text.dw_usart_irqhandler)
  *mcu_ch6121.a:P256-cortex-m0-ecdh-gcc.o(*)
  *mcu_ch6121.a:ringbuffer.o(.text.*)
  *mcu_ch6121.a:flash.o(.text.hal_flash_erase)
  *mcu_ch6121.a:uart.o(.text.hal_uart_recv_II)
  *rhino.a:k_buf_queue.o(.text.*)
  *rhino.a:k_idle.o(.text.*)
  *rhino.a:k_mm.o(.text.*)
  *rhino.a:k_ringbuf.o(.text.*)
  *rhino.a:k_mutex.o(.text.*)
  *rhino.a:k_pend.o(.text.*)
  *rhino.a:k_sched.o(.text.*)
  *rhino.a:k_sem.o(.text.*)
  *rhino.a:k_tick.o(.text.*)
  *rhino.a:k_timer.o(.text.*)
  *rhino.a:k_time.o(.text.*)
  *mcu_ch6121.a:common.o(.text.*)
  *rhino.a:k_sys.o(.text.*)
  *armv6m.a:port_s.o(.text*)
  *prov.o(.text*)
  *ch6121_driver.o(.text.*)
  *bt_host.a:hci_core.o(.text.report_completed_packet                  )
  *bt_host.a:hci_core.o(.text.bt_addr_le_copy                          )
  *bt_host.a:hci_core.o(.text.atomic_test_bit                          )
  *bt_host.a:hci_core.o( .text.hci_api_num_complete_packets            )
  *bt_host.a:hci_core.o( .text.bt_le_conn_params_valid                 )
  *bt_host.a:hci_core.o( .text.bt_le_scan_update                       )
  *buf.o(.text.*)
  *aos_port.o(.text.*)
  *poll.o(.text.*)
  *conn.o(.text.bt_conn_recv)
  *l2cap.o(.text.bt_l2cap_recv)
  *att.o(.text.bt_att_recv .text.att_notify)
  *gatt.o(.text.bt_gatt_notification)
  *crypto.o(.text.*)
  *crypto_ctrl.o(.text.*)
  *net.o(.text.*)
  *adv.o(.text.*)
  *stack_main.o(.text.wakeup_init .text.rf_calibrate)
  *ll_patch.o(.text.* .rodata.*)
  *hci_event.o(.text.* .rodata.*)
  *hci.o(.text.* .rodata.*)
  *hci_tl.o(.text.* .rodata.*)
  *hci_data.o(.text.* .rodata.*)
  *ch6121_hci.o(.text.ble_stack_main .rodata.*)
  *ch6121_hci.o(.text.ch6121_hci_send)
  *ch6121_hci.o(.text.ch6121_hci_sync)
  *ch6121_hci.o(.text.control2host_send_message)
  *aes_decrypt.o(.text.*)
  *bt_common.a:utils.o(.text.*)
  *hci_api.o(.text.*)
  *libc_nano.a:(.text.memcpy .text.memset)
  *cli.a:cli.o(.text.cli_getchar)
  *libgcc.a:_aeabi_uldivmod.o(.text.__udivmoddi4)
  *(.retention_mem_area0 retention_mem_area0)
  *ch6121_driver.o(.bss.rx_thread_stack)
 } > REGION_RETENTION1
 .text : {
  . = ALIGN(0x4) ;
  __stext = . ;
  *(.text)
  *(.text*)
  *(.text.*)
  *(.gnu.warning)
  *(.stub)
  *(.gnu.linkonce.t*)
  *(.glue_7t)
  *(.glue_7)
  *(.jcr)
  KEEP (*(.init))
  KEEP (*(.fini))
  . = ALIGN (4) ;
  PROVIDE(__ctbp = .);
  *(.call_table_data)
  *(.call_table_text)
  . = ALIGN(0x10) ;
  __etext = . ;
 } > REGION_TEXT
 .eh_frame_hdr : {
  *(.eh_frame_hdr)
 } > REGION_TEXT
 .eh_frame : ONLY_IF_RO {
  KEEP (*(.eh_frame))
 } > REGION_TEXT
 .gcc_except_table : ONLY_IF_RO {
  *(.gcc_except_table .gcc_except_table.*)
 } > REGION_TEXT
 .rodata : {
  . = ALIGN(0x4) ;
  __srodata = .;
  *(.rdata)
  *(.rdata*)
  *(.rdata1)
  *(.rdata.*)
  *(.rodata)
  *(.rodata1)
  *(.rodata*)
  *(.rodata.*)
  *(.rodata.str1.4)
  . = ALIGN(0x4) ;
  __ctor_start__ = .;
  KEEP (*(SORT(.ctors.*)))
  KEEP (*(.ctors))
  __ctor_end__ = .;
  KEEP (*(SORT(.dtors.*)))
  KEEP (*(.dtors))
  __dtor_end__ = .;
   _bt_settings_start = .;
   KEEP(*(._bt_settings.static.*))
   _bt_settings_end = .;
   . = ALIGN(0x4) ;
  __erodata = .;
  __rodata_end__ = .;
 } > REGION_RODATA
 .data : {
  . = ALIGN(0x4) ;
  __sdata = . ;
  __data_start__ = . ;
  __data_copy_start__ = .;
  data_start = . ;
  *(.got.plt)
  *(.got)
  *(.gnu.linkonce.r*)
  *(.data)
  *(.data*)
  *(.data1)
  *(.data.*)
  *(.gnu.linkonce.d*)
  *(.data1)
  *(.gcc_except_table)
  *(.gcc_except_table*)
  __start_init_call = .;
  *(.initcall.init)
  __stop_init_call = .;
  __start_cmd = .;
  *(.bootloaddata.cmd)
  . = ALIGN(4) ;
  __stop_cmd = .;
  *(.sdata)
  *(.sdata.*)
  *(.gnu.linkonce.s.*)
  *(__libc_atexit)
  *(__libc_subinit)
  *(__libc_subfreeres)
  *(.note.ABI-tag)
  . = ALIGN(0x4) ;
  _net_buf_pool_list = .;
  KEEP(*(SORT_BY_NAME("._net_buf_pool.static.*")))
  _net_buf_pool_list_end = .;
  . = ALIGN(0x4) ;
  __edata = .;
  __data_end__ = .;
  . = ALIGN(0x4) ;
 } > REGION_RETENTION1
 .eh_frame : ONLY_IF_RW {
  KEEP (*(.eh_frame))
 } > REGION_RETENTION1
 .gcc_except_table : ONLY_IF_RW {
  *(.gcc_except_table .gcc_except_table.*)
  __edata = .;
  __data_end__ = .;
  __data_copy_end__ = .;
 } > REGION_RETENTION1
 .jmp_table : {
  KEEP (*(jump_table_mem_area))
 } > LR_IRAM1
 .global_config : {
  KEEP (*(global_config_area))
 } > LR_IRAM2
 .data_noretention : {
  . = ALIGN(0x4) ;
  *(noretention_mem_area0)
 } > REGION_RETENTION0
 .bss : {
  . = ALIGN(0x4) ;
  __sbss = ALIGN(0x4) ;
  __bss_start__ = . ;
  *(.dynsbss)
  *(.sbss)
  *(.sbss.*)
  *(.scommon)
  *(.dynbss)
  *(.bss)
  *(.bss.*)
  *(COMMON)
  . = ALIGN(0x4) ;
  __ebss = . ;
  __bss_end__ = .;
  __end = . ;
  end = . ;
 } > REGION_RETENTION0
 ._user_heap : {
  . += 4;
  . = ALIGN(0x4) ;
  __heap_start = .;
  . += __min_heap_size;
  . = ALIGN(0x4) ;
 } > REGION_RETENTION0
}
