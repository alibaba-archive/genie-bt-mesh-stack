##
 # Copyright (C) 2017-2019 C-SKY Microsystems Co., Ltd. All rights reserved.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #   http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
##

INCDIR += -I$(CHIPDIR)/include
INCDIR += -I$(DRIVERDIR)/include

CFLAGS += -DCFG_CP -DCFG_QFN32 -DCFG_SLEEP_MODE=PWR_MODE_NO_SLEEP -DDEBUG_INFO=1

DRIVER_SSRC += $(CHIPDIR)/startup.S
DRIVER_SSRC += $(CHIPDIR)/lpm_arch_reg_save.S
DRIVER_CSRC += $(CHIPDIR)/power_manager.c
DRIVER_CSRC += $(CHIPDIR)/system.c
DRIVER_CSRC += $(CHIPDIR)/jump_table.c
DRIVER_CSRC += $(CHIPDIR)/isr.c
DRIVER_CSRC += $(CHIPDIR)/device.c
DRIVER_CSRC += $(CHIPDIR)/sys_freq.c
DRIVER_CSRC += $(CHIPDIR)/pinmux.c
DRIVER_CSRC += $(CHIPDIR)/lib.c
DRIVER_CSRC += $(CHIPDIR)/reboot.c
DRIVER_CSRC += $(CHIPDIR)/lib_printf.c

DRIVER_CSRC += $(DRIVERDIR)/clock.c
DRIVER_CSRC += $(DRIVERDIR)/common.c
DRIVER_CSRC += $(DRIVERDIR)/dw_usart.c
DRIVER_CSRC += $(DRIVERDIR)/ck_irq.c
DRIVER_CSRC += $(DRIVERDIR)/gpio.c
DRIVER_CSRC += $(DRIVERDIR)/dw_gpio.c
DRIVER_CSRC += $(DRIVERDIR)/dw_timer.c
DRIVER_CSRC += $(DRIVERDIR)/dw_spi.c
DRIVER_CSRC += $(DRIVERDIR)/phy_rtc.c
DRIVER_CSRC += $(DRIVERDIR)/phy_pmu.c
DRIVER_CSRC += $(DRIVERDIR)/dw_wdt.c
DRIVER_CSRC += $(DRIVERDIR)/spif.c
DRIVER_CSRC += $(DRIVERDIR)/dw_iic.c
DRIVER_CSRC += $(DRIVERDIR)/trng.c
DRIVER_CSRC += $(DRIVERDIR)/pwm.c
DRIVER_CSRC += $(DRIVERDIR)/phy_aes.c
DRIVER_CSRC += $(DRIVERDIR)/adc.c
