#ifndef _FAKE_CLOCK_PUB_H_
#define _FAKE_CLOCK_PUB_H_

#include "include.h"
#include "pwm_pub.h"

#define FCLK_PWM_ID           PWM5 ///pwm timer

#if (RHINO_CONFIG_TICKS_PER_SECOND >= 250)
#define FCLK_SECOND           (16000000 / RHINO_CONFIG_TICKS_PER_SECOND)
#else
#define FCLK_SECOND           (32000 / RHINO_CONFIG_TICKS_PER_SECOND)
#endif

extern UINT32 fclk_get_tick(void);
extern UINT32 fclk_get_second(void);
extern void fclk_reset_count(void);
extern void fclk_init(UINT8 pwm_id, UINT16 ticks_per_sec);
extern UINT32 fclk_from_sec_to_tick(UINT32 sec);
extern UINT32 fclk_tick_a_second(void);
extern UINT32 fclk_cal_endvalue(UINT32 mode);

#endif // _FAKE_CLOCK_PUB_H_
// eof

