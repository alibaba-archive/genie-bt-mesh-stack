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
 * @file     phy_rtc.c
 * @brief    CSI Source File for RTC Driver
 * @version  V1.0
 * @date     24. May 2019
 ******************************************************************************/

#include <csi_config.h>
#include <stdbool.h>
#include <string.h>
#include <phy_rtc.h>
#include "phy_rtc.h"
#include <drv_irq.h>
#include <drv_rtc.h>
#include <soc.h>
#include <clock.h>

#include "ap_cp.h"
#include "common.h"
#include "OSAL.h"
#include "clock.h"
#include "pwrmgr.h"
#include "error.h"
#include "hal_defs.h"
#include "hal_mcu.h"
#include "rf_phy_driver.h"


#define ERR_RTC(errno) (CSI_DRV_ERRNO_RTC_BASE | errno)
#define RTC_BUSY_TIMEOUT        0x10000000
#define RTC_NULL_PARAM_CHK(para)    HANDLE_PARAM_CHK(para, ERR_RTC(DRV_ERROR_PARAMETER))

typedef struct {
#ifdef CONFIG_LPM
    uint8_t rtc_power_status;
    uint32_t rtc_regs_saved[3];
#endif
    uint32_t base;
    uint32_t irq;
    rtc_event_cb_t cb_event;
    struct tm rtc_base;
} phy_rtc_priv_t;

extern void mdelay(uint32_t ms);
extern int32_t target_get_rtc_count(void);
extern int32_t target_get_rtc(int32_t idx, uint32_t *base, uint32_t *irq, void **handler);

static phy_rtc_priv_t rtc_instance[CONFIG_RTC_NUM];
static uint8_t leap_year[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static uint8_t noleap_year[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const uint16_t g_noleap_daysbeforemonth[13] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};
//static const uint16_t g_leap_daysbeforemonth[13] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366};

static const rtc_capabilities_t rtc_capabilities = {
    .interrupt_mode = 1,    /* supports Interrupt mode */
    .wrap_mode = 0          /* supports wrap mode */
};


extern void WaitRTCCount(uint32_t rtcDelyCnt);

static uint8 rc32k_calibration(void)
{
    int delay = 15000;//4000;//ZQ 20181203 for HCLK=48M
    uint32_t  temp = 0;

    *(volatile uint32_t *) 0x4000f05c &= 0xfffffffe;                  // disable RC32K calibration
    WaitRTCCount(6);
    // calibrate RC32K clock
    *(volatile uint32_t *) 0x4000f018 |= 0x80;                  // set capbank controlled by calibration
    *(volatile uint32_t *) 0x4000f05c |= 0x01;                  // enable RC32K calibration

    while (!(*(volatile uint32_t *) 0x4000f068 & 0x200)         // check RC32K calibration OK flag, normally need >200us
           && delay -- > 0)
        ;


    if (delay >= 0) {
        temp = (*(volatile uint32_t *) 0x4000f060 & 0x3f0000) >> 15;        // read 6bit calibration result
        *(volatile uint32_t *)0x4000f018 = (*(volatile uint32_t *) 0x4000f018 & 0xffffff81) | temp;   // write the result
    }

    *(volatile uint32_t *) 0x4000f018 &= 0xffffff7f;            // set capbank controlled by AON

    return (uint8)(0x7f & (temp >> 1));
}

static void arry_sort(uint32 *arr, uint32 len)
{
    uint32 i, j, temp;

    for (i = 0; i < len - 1; i++)
        for (j = 0; j < len - 1 - i; j++)
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
}
static uint32 rc32k_tracking_check(uint8 rdCnt, uint8 ckCnt)
{
    uint32 cnt[16] = {0};
    uint32 error_delt;
    uint32 sum_cnt = 0;

    for (uint8 i = 0; i < rdCnt; i++) {
        WaitRTCCount(16);
        cnt[i] = (*(volatile uint32_t *)0x4000f064 & 0x3fff);
    }

    arry_sort(cnt, rdCnt);


    for (uint8 i = 0; i < ckCnt; i++) {

        error_delt = (cnt[i] > STD_CRY32_8_CYCLE_16MHZ_CYCLE)
                     ?   cnt[i] - STD_CRY32_8_CYCLE_16MHZ_CYCLE : STD_CRY32_8_CYCLE_16MHZ_CYCLE - cnt[i];

        if (error_delt > (ERR_THD_RC32_CYCLE >> 1)) {
            return 0;
        } else {
            sum_cnt += cnt[i];
        }
    }

    return (sum_cnt / ckCnt);

}

#define CAL_32KRC_RETRY_CNT_LIMT        0x10
static uint8 rc32k_calibration_check(uint8 hLimt, uint8 lLimt, uint8 rdCnt, uint8 ckCnt)
{
    uint8 calCnt = CAL_32KRC_RETRY_CNT_LIMT;
    uint8 rc0;
    uint32 temp;

    while (calCnt--) {
        rc0 = rc32k_calibration();

        if (rc0 < hLimt && rc0 > lLimt) {
            temp = rc32k_tracking_check(rdCnt, ckCnt);

            //jugment of the rc32k calibration result according to 16MXtal tracking count
            if ((temp > STD_CRY32_8_CYCLE_16MHZ_CYCLE - (ERR_THD_RC32_CYCLE >> 1))
                && (temp < STD_CRY32_8_CYCLE_16MHZ_CYCLE + (ERR_THD_RC32_CYCLE >> 1))) {
                break;
            }
        }
    }

    return (CAL_32KRC_RETRY_CNT_LIMT - calCnt);
}

static inline int clock_isleapyear(int year)
{
    return (year % 400) ? ((year % 100) ? ((year % 4) ? 0 : 1) : 0) : 1;
}

static uint64_t phy_rtc_readtime(phy_rtc_reg_t *addr)
{
    return (addr->RTC_CNT + (uint64_t)(addr->RTC_CC2 & 0xffffff) * (0xffffff + 1)) / 8;
}

static int ck_check_tm_ok(struct tm *rtctime)
{
    int32_t leap = 1;

    if (rtctime->tm_year < 70 || rtctime->tm_year >= 200) {
        goto error_time;
    }

    leap = clock_isleapyear(rtctime->tm_year + 1900);

    if (rtctime->tm_sec < 0 || rtctime->tm_sec >= 60) {
        goto error_time;
    }

    if (rtctime->tm_min < 0 || rtctime->tm_min >= 60) {
        goto error_time;
    }

    if (rtctime->tm_hour < 0 || rtctime->tm_hour >= 24) {
        goto error_time;
    }

    if (rtctime->tm_mon < 0 || rtctime->tm_mon >= 12) {
        goto error_time;
    }

    if (leap) {
        if (rtctime->tm_mday < 1 || rtctime->tm_mday > leap_year[rtctime->tm_mon]) {
            goto error_time;
        }
    } else {
        if (rtctime->tm_mday < 1 || rtctime->tm_mday > noleap_year[rtctime->tm_mon]) {
            goto error_time;
        }
    }

    return 0;
error_time:
    return ERR_RTC(RTC_ERROR_TIME);

}

static int clock_daysbeforemonth(int month, bool leapyear)
{
    int retval = g_noleap_daysbeforemonth[month];

    if (month >= 2 && leapyear) {
        retval++;
    }

    return retval;
}

static time_t clock_calendar2utc(int year, int month, int day)
{
    time_t days;

    /* Years since epoch in units of days (ignoring leap years). */

    days = (year - 1970) * 365;

    /* Add in the extra days for the leap years prior to the current year. */

    days += (year - 1969) >> 2;

    /* Add in the days up to the beginning of this month. */

    days += (time_t)clock_daysbeforemonth(month, clock_isleapyear(year));

    /* Add in the days since the beginning of this month (days are 1-based). */

    days += day - 1;

    /* Then convert the seconds and add in hours, minutes, and seconds */

    return days;
}


time_t mktime(struct tm *tp)
{
    time_t ret;
    time_t jdn;

    /* Get the EPOCH-relative julian date from the calendar year,
     * month, and date
     */

    ret = ck_check_tm_ok(tp);

    if (ret < 0) {
        return -1;
    }

    jdn = clock_calendar2utc(tp->tm_year + 1900, tp->tm_mon, tp->tm_mday);

    /* Return the seconds into the julian day. */

    ret = ((jdn * 24 + tp->tm_hour) * 60 + tp->tm_min) * 60 + tp->tm_sec;

    return ret;
}
static void clock_utc2calendar(time_t days, int *year, int *month,
                               int *day)
{

    /* There is one leap year every four years, so we can get close with the
     * following:
     */

    int value   = days  / (4 * 365 + 1); /* Number of 4-years periods since the epoch */
    days   -= value * (4 * 365 + 1); /* Remaining days */
    value <<= 2;                     /* Years since the epoch */

    /* Then we will brute force the next 0-3 years */
    bool leapyear;
    int  tmp;

    for (; ;) {
        /* Is this year a leap year (we'll need this later too) */

        leapyear = clock_isleapyear(value + 1970);

        /* Get the number of days in the year */

        tmp = (leapyear ? 366 : 365);

        /* Do we have that many days? */

        if (days >= tmp) {
            /* Yes.. bump up the year */

            value++;
            days -= tmp;
        } else {
            /* Nope... then go handle months */

            break;
        }
    }

    /* At this point, value has the year and days has number days into this year */

    *year = 1970 + value;

    /* Handle the month (zero based) */
    int  min = 0;
    int  max = 11;

    do {
        /* Get the midpoint */

        value = (min + max) >> 1;

        /* Get the number of days that occurred before the beginning of the month
         * following the midpoint.
         */

        tmp = clock_daysbeforemonth(value + 1, leapyear);

        /* Does the number of days before this month that equal or exceed the
         * number of days we have remaining?
         */

        if (tmp > days) {
            /* Yes.. then the month we want is somewhere from 'min' and to the
             * midpoint, 'value'.  Could it be the midpoint?
             */

            tmp = clock_daysbeforemonth(value, leapyear);

            if (tmp > days) {
                /* No... The one we want is somewhere between min and value-1 */

                max = value - 1;
            } else {
                /* Yes.. 'value' contains the month that we want */

                break;
            }
        } else {
            /* No... The one we want is somwhere between value+1 and max */

            min = value + 1;
        }

        /* If we break out of the loop because min == max, then we want value
         * to be equal to min == max.
         */

        value = min;
    } while (min < max);

    /* The selected month number is in value. Subtract the number of days in the
     * selected month
     */

    days -= clock_daysbeforemonth(value, leapyear);

    /* At this point, value has the month into this year (zero based) and days has
     * number of days into this month (zero based)
     */

    *month = value + 1; /* 1-based */
    *day   = days + 1;  /* 1-based */
}


struct tm *gmtime_r(const time_t *timer, struct tm *result)
{
    time_t epoch;
    time_t jdn;
    int    year;
    int    month;
    int    day;
    int    hour;
    int    min;
    int    sec;

    /* Get the seconds since the EPOCH */

    epoch = *timer;

    /* Convert to days, hours, minutes, and seconds since the EPOCH */

    jdn    = epoch / SEC_PER_DAY;
    epoch -= SEC_PER_DAY * jdn;

    hour   = epoch / SEC_PER_HOUR;
    epoch -= SEC_PER_HOUR * hour;

    min    = epoch / SEC_PER_MIN;
    epoch -= SEC_PER_MIN * min;

    sec    = epoch;

    /* Convert the days since the EPOCH to calendar day */

    clock_utc2calendar(jdn, &year, &month, &day);


    /* Then return the struct tm contents */

    result->tm_year  = (int)year - 1900; /* Relative to 1900 */
    result->tm_mon   = (int)month - 1;   /* zero-based */
    result->tm_mday  = (int)day;         /* one-based */
    result->tm_hour  = (int)hour;
    result->tm_min   = (int)min;
    result->tm_sec   = (int)sec;
    return result;
}
static int phy_rtc_setmarchtime(phy_rtc_reg_t *addr, int64_t settime)
{
    if (settime < 0 || (settime * 8) >= 0x1000000) {
        return ERR_RTC(DRV_ERROR_PARAMETER);
    }

    addr->RTC_CC0 = (uint32_t)(settime * 8 + addr->RTC_CNT);
    return 0;
}

void phy_rtc_irqhandler(int32_t idx)
{
    phy_rtc_priv_t *rtc_priv = &rtc_instance[idx];
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    if ((addr->RTC_FLAG & 1u) && rtc_priv->cb_event) {
        rtc_priv->cb_event(idx, RTC_EVENT_TIMER_INTRERRUPT);
    }

    if (addr->RTC_FLAG & 1u << 3) {
        addr->RTC_CC2 += 1u;
    }
}

#ifdef CONFIG_LPM
static void manage_clock(rtc_handle_t handle, uint8_t enable)
{
    if (handle == &rtc_instance[0]) {
        drv_clock_manager_config(CLOCK_MANAGER_RTC, enable);
    } else if (handle == &rtc_instance[1]) {
        drv_clock_manager_config(CLOCK_MANAGER_RTC1, enable);
    }
}

static void do_prepare_sleep_action(rtc_handle_t handle)
{
    phy_rtc_priv_t *rtc_priv = handle;
    uint32_t *rbase = (uint32_t *)(rtc_priv->base);
    registers_save(rtc_priv->rtc_regs_saved, rbase + 1, 3);
}

static void do_wakeup_sleep_action(rtc_handle_t handle)
{
    phy_rtc_priv_t *rtc_priv = handle;
    uint32_t *rbase = (uint32_t *)(rtc_priv->base);
    registers_restore(rbase + 1, rtc_priv->rtc_regs_saved, 3);
}
#endif

/**
  \brief       Initialize RTC Interface. 1. Initializes the resources needed for the RTC interface 2.registers event callback function
  \param[in]   idx  rtc index
  \param[in]   cb_event  Pointer to \ref rtc_event_cb_t
  \return      pointer to rtc instance
*/
rtc_handle_t csi_rtc_initialize(int32_t idx, rtc_event_cb_t cb_event)
{
    if (idx < 0 || idx >= CONFIG_RTC_NUM) {
        return NULL;
    }

    int32_t real_idx;
    uint32_t base = 0u;
    uint32_t irq;
    void *handler;

    real_idx = target_get_rtc(idx, &base, &irq, &handler);

    if (real_idx != idx) {
        return NULL;
    }

    phy_rtc_priv_t *rtc_priv;

    rtc_priv = &rtc_instance[idx];
    rtc_priv->base = base;
    rtc_priv->irq  = irq;
    rtc_priv->cb_event = cb_event;

#ifdef CONFIG_LPM
    csi_rtc_power_control(rtc_priv, DRV_POWER_FULL);
#endif

    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    hal_rtc_clock_config(CLK_32K_RCOSC);

    addr->RTC_CC0 = 0;
    addr->RTC_CC1 = 0;
    addr->RTC_CC2 = 0;
    addr->RTC_CTL = 0x2;

    drv_irq_register(RTC_IRQ, handler);
    drv_irq_enable(RTC_IRQ);

    addr->RTC_CTL |= 1;
    rc32k_calibration_check(/*hLimt*/0x38,/*lLimt*/0x04,/*rdCnt*/8,/*cdCnt*/4);
    addr->RTC_CTL &= ~1;
    addr->RTC_CTL |= 2u;

    volatile uint32_t get_reg = 0x2;
    volatile uint32_t timeout_count = 0;
    addr->RTC_CTL |= 1u << 1;

    while (get_reg & 0x2) {
        get_reg = addr->RTC_CTL;

        if (timeout_count++ > 0xff) {
            break;
        }
    }

    addr->RTC_CTL &= ~(0xfff << 2);
    addr->RTC_CTL |= 4095 << 2;

    addr->RTC_CTL |= 1u << 14 | 1u << 18;

    return (rtc_handle_t)rtc_priv;
}

/**
  \brief       De-initialize RTC Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle rtc handle to operate.
  \return      \ref execution_status
*/
int32_t csi_rtc_uninitialize(rtc_handle_t handle)
{
    RTC_NULL_PARAM_CHK(handle);

    phy_rtc_priv_t *rtc_priv = handle;

    rtc_priv->cb_event = NULL;

    drv_irq_disable(RTC_IRQ);
    drv_irq_unregister(RTC_IRQ);

#ifdef CONFIG_LPM
    csi_rtc_power_control(rtc_priv, DRV_POWER_OFF);
#endif
    return 0;
}

int32_t csi_rtc_power_control(rtc_handle_t handle, csi_power_stat_e state)
{
    RTC_NULL_PARAM_CHK(handle);

    if (state == DRV_POWER_FULL) {
        /* Always on module, not support */
        return ERR_RTC(DRV_ERROR_UNSUPPORTED);
        /* Always on module, not support */
    } else if (state == DRV_POWER_OFF) {
        return ERR_RTC(DRV_ERROR_UNSUPPORTED);
    } else {
#ifdef CONFIG_LPM
        power_cb_t callback = {
            .wakeup = do_wakeup_sleep_action,
            .sleep = do_prepare_sleep_action,
            .manage_clock = manage_clock
        };
        return drv_soc_power_control(handle, state, &callback);
#else
        return ERR_RTC(DRV_ERROR_UNSUPPORTED);
#endif
    }
}

/**
  \brief       Get driver capabilities.
  \param[in]   idx  rtc index
  \return      \ref rtc_capabilities_t
*/
rtc_capabilities_t csi_rtc_get_capabilities(int32_t idx)
{
    if (idx < 0 || idx >= CONFIG_RTC_NUM) {
        rtc_capabilities_t ret;
        memset(&ret, 0, sizeof(rtc_capabilities_t));
        return ret;
    }

    return rtc_capabilities;
}

/**
  \brief       Set RTC timer.
  \param[in]   handle rtc handle to operate.
  \param[in]   rtctime \ref struct tm
  \return      \ref execution_status
*/

int32_t csi_rtc_set_time(rtc_handle_t handle, const struct tm *rtctime)
{
    RTC_NULL_PARAM_CHK(handle);
    RTC_NULL_PARAM_CHK(rtctime);

    phy_rtc_priv_t *rtc_priv = handle;

    int32_t ret = ck_check_tm_ok((struct tm *)rtctime);
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    if (ret < 0) {
        return ret;
    }

    volatile uint32_t get_reg = 0x2;
    volatile uint32_t timeout_count = 0;
    addr->RTC_CTL |= 1u << 1;

    while (get_reg & 0x2) {
        get_reg = addr->RTC_CTL;

        if (timeout_count++ > 0xff) {
            break;
        }
    }

    rtc_priv->rtc_base.tm_sec  = rtctime->tm_sec;
    rtc_priv->rtc_base.tm_min  = rtctime->tm_min;
    rtc_priv->rtc_base.tm_hour = rtctime->tm_hour;
    rtc_priv->rtc_base.tm_mday = rtctime->tm_mday;
    rtc_priv->rtc_base.tm_mon  = rtctime->tm_mon;
    rtc_priv->rtc_base.tm_year = rtctime->tm_year;

    return 0;
}

/**
  \brief       Get RTC timer.
  \param[in]   handle rtc handle to operate.
  \param[in]   rtctime \ref struct tm
  \return      \ref execution_status
*/
int32_t csi_rtc_get_time(rtc_handle_t handle, struct tm *rtctime)
{
    RTC_NULL_PARAM_CHK(handle);
    RTC_NULL_PARAM_CHK(rtctime);

    phy_rtc_priv_t *rtc_priv = handle;
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    time_t time = phy_rtc_readtime(addr);
    time += mktime(&(rtc_priv->rtc_base));
    gmtime_r(&time, rtctime);

    return 0;

}

/**
  \brief       Start RTC timer.
  \param[in]   handle rtc handle to operate.
  \return      \ref execution_status
*/
int32_t csi_rtc_start(rtc_handle_t handle)
{
    RTC_NULL_PARAM_CHK(handle);

    phy_rtc_priv_t *rtc_priv = handle;
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    addr->RTC_CTL |= 1u;
    return 0;
}

/**
  \brief       Stop RTC timer.
  \param[in]   handle rtc handle to operate.
  \return      \ref execution_status
*/
int32_t csi_rtc_stop(rtc_handle_t handle)
{
    RTC_NULL_PARAM_CHK(handle);

    phy_rtc_priv_t *rtc_priv = handle;
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    addr->RTC_CTL &= ~1u;
    return 0;
}

/**
  \brief       Get RTC status.
  \param[in]   handle rtc handle to operate.
  \return      RTC status \ref rtc_status_t
*/
rtc_status_t csi_rtc_get_status(rtc_handle_t handle)
{
    rtc_status_t rtc_status = {0};

    if (handle == NULL) {
        return rtc_status;
    }

    phy_rtc_priv_t *rtc_priv = handle;
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    if (addr->RTC_CTL & 0x1) {
        rtc_status.active = 1;
    }

    return rtc_status;
}

/**
  \brief       config RTC timer.
  \param[in]   handle rtc handle to operate.
  \param[in]   rtctime time to wake up
  \return      error code
*/
int32_t csi_rtc_set_alarm(rtc_handle_t handle, const struct tm *rtctime)
{
    int32_t ret;
    struct tm current_time;
    int64_t settime = 0;

    RTC_NULL_PARAM_CHK(handle);
    RTC_NULL_PARAM_CHK(rtctime);

    phy_rtc_priv_t *rtc_priv = (phy_rtc_priv_t *)handle;

    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);
    ret = ck_check_tm_ok((struct tm *)rtctime);

    if (ret < 0) {
        goto error_time;
    }

    ret = csi_rtc_get_time(handle, &current_time);
    settime = (mktime((struct tm *)rtctime) - (int64_t)mktime(&current_time));

    if (settime < 0) {
        goto error_time;
    }

    phy_rtc_setmarchtime(addr, settime);

    return 0;
error_time:
    return ERR_RTC(RTC_ERROR_TIME);

}

/**
  \brief       disable or enable RTC timer.
  \param[in]   handle rtc handle to operate.
  \param[in]   flag  1 - enable rtc alarm 0 - disable rtc alarm
  \return      error code
*/
int32_t csi_rtc_enable_alarm(rtc_handle_t handle, uint8_t en)
{
    RTC_NULL_PARAM_CHK(handle);

    phy_rtc_priv_t *rtc_priv = handle;
    phy_rtc_reg_t *addr = (phy_rtc_reg_t *)(rtc_priv->base);

    if (en == 1) {
        addr->RTC_CTL |= 1 << 20u;
        addr->RTC_CTL |= 1 << 15u;
    } else if (en == 0) {
        addr->RTC_CTL &= ~(1 << 20u);
        addr->RTC_CTL &= ~(1 << 15u);
        phy_rtc_setmarchtime(addr, phy_rtc_readtime(addr));
    } else {
        return ERR_RTC(DRV_ERROR_PARAMETER);
    }

    return 0;
}

