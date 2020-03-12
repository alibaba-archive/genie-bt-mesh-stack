#ifndef _INTC_PUB_H_
#define _INTC_PUB_H_

#include "generic.h"

#define FIQ_IRQ_END                      (18)

#if 1
#define FIQ_BLE                          (15)
#define FIQ_PWM0                         (0)

#define IRQ_UART2                        (16)

#define IRQ_PWM5                         (13)
#define IRQ_RTC                          (12)

#define IRQ_GPIO                         (9)
#define IRQ_ADC                          (8)
#define IRQ_I2S_PCM                      (7)
#define IRQ_SPI                          (6)
#define IRQ_UART1                        (5)

#define IRQ_PWM4                         (4)
#define IRQ_PWM3                         (3)
#define IRQ_PWM2                         (2)
#define IRQ_PWM1                         (1)
#endif

#if 2
#define PRI_FIQ_BLE                      (31)
#define PRI_FIQ_PWM0                     (30)

#define PRI_IRQ_RTC                      (11)

#define PRI_IRQ_GPIO                     (12)
#define PRI_IRQ_ADC                      (29)
#define PRI_IRQ_I2S_PCM                  (28)
#define PRI_IRQ_SPI                      (13)

#define PRI_IRQ_UART2                    (26)
#define PRI_IRQ_UART1                    (27)

#define PRI_IRQ_PWM5                     (25)
#define PRI_IRQ_PWM4                     (24)
#define PRI_IRQ_PWM3                     (23)
#define PRI_IRQ_PWM2                     (22)
#define PRI_IRQ_PWM1                     (21)
#endif 

extern void intc_service_register(UINT8 int_num, UINT8 int_pri, FUNCPTR isr);
extern void intc_spurious(void);
extern void intc_enable(int index);
extern void intc_disable(int index);

#endif // _INTC_PUB_H_
// eof
