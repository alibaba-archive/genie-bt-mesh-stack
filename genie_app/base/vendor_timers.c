//
// Created by sanyi on 2019/12/25.
//

#include <stdint.h>
#include "vendor_timers.h"
#include <api/mesh.h>
#include <misc/slist.h>
#include <hal/soc/timer.h>
#include <base/genie_flash.h>

#define VT_DEBUG(...)  //printf
//#define VT_STORE

#define DAY                         86400
#define HOUR                        3600
#define MINU                        60

#define VT_NUM (40)

#define VT_LOCK    k_sem_take(&g_vendor_timer.lock, -1)
#define VT_UNLOCK  k_sem_give(&g_vendor_timer.lock)

static UTCTimeStruct local_time = {0};

typedef enum {
    TIMER_OFF = 0,
    TIMER_ON = 1,
    TIMER_INVAILD = 0xf,
} vt_state;

struct vendor_timer_t {
    sys_snode_t  next;
    uint8_t  index;
    uint8_t state: 4;
    uint8_t periodic: 1;
    uint16_t periodic_time;
    uint8_t  schedule;
    uint32_t unixtime_match;
    vendor_attr_data_t attr_data;
};

struct unixtime_sync_t {
    uint16_t period_time;
    uint8_t retry_delay;
    uint8_t retry_times;
};

struct {
    uint16_t magic;
    int8_t  timezone;
    struct unixtime_sync_t timing_sync_config;
    struct vendor_timer_t timer_data[VT_NUM];
} g_timing_data;

struct {
    uint32_t init: 1;
    uint32_t update: 1;
    timer_dev_t timer;
    struct k_work work;
    struct k_sem lock;
    sys_slist_t timer_list_active;
    sys_slist_t timer_list_idle;
    uint32_t unix_time;
    uint32_t unix_time_sync_match;
    uint8_t  unix_time_sync_retry_times;
    vendor_timer_event_func_t cb;
} g_vendor_timer;

static inline uint8_t is_leap_year(uint16_t year);

UTCTimeStruct vendor_timer_local_time_get()
{
    return local_time;
}

uint32_t vendor_timer_local_unixtime_get()
{
    return g_vendor_timer.unix_time;
}

static inline UTCTimeStruct unix2UTC(uint32_t unix_time)
{
    UTCTimeStruct utc;

    uint16_t g_noleap_daysbeforemonth[13] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};
    uint32_t epoch;
    uint32_t jdn;
    int      year;
    int      month;
    int      day;
    int      hour;
    int      minutes;
    int      sec;
    int      weekday;

    epoch = unix_time;

    jdn    = epoch / DAY;
    epoch -= DAY * jdn;

    weekday = (jdn + 4) % 7; // 1970/1/1 is thursday

    hour   = epoch / HOUR;
    epoch -= HOUR * hour;

    minutes    = epoch / MINU;
    epoch -= MINU * minutes;

    sec    = epoch;

    year    = jdn   / (4 * 365 + 1); /* Number of 4-years periods since the epoch */
    jdn   -= year * (4 * 365 + 1);  /* Remaining days */
    year <<= 2;                  /* Years since the epoch */

    /* Then we will brute force the next 0-3 years */
    bool leapyear;
    int  tmp;

    for (; ;) {
        /* Is this year a leap year (we'll need this later too) */

        leapyear = is_leap_year(year + 1970);

        /* Get the number of days in the year */

        tmp = (leapyear ? 366 : 365);

        /* Do we have that many days? */

        if (jdn >= tmp) {
            /* Yes.. bump up the year */

            year++;
            jdn -= tmp;
        } else {
            /* Nope... then go handle months */

            break;
        }
    }

    /* At this point, value has the year and days has number days into this year */

    year += 1970;

    /* Handle the month (zero based) */
    int  min = 0;
    int  max = 11;
    int value = 0;

    do {
        /* Get the midpoint */

        value = (min + max) >> 1;

        /* Get the number of days that occurred before the beginning of the month
         * following the midpoint.
         */

        tmp = g_noleap_daysbeforemonth[value + 1];

        if (value + 1 >= 2 && is_leap_year(leapyear)) {
            tmp++;
        }

        /* Does the number of days before this month that equal or exceed the
         * number of days we have remaining?
         */

        if (tmp > jdn) {
            /* Yes.. then the month we want is somewhere from 'min' and to the
             * midpoint, 'value'.  Could it be the midpoint?
             */

            tmp = g_noleap_daysbeforemonth[value];

            if (value >= 2 && is_leap_year(leapyear)) {
                tmp++;
            }

            if (tmp > jdn) {
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

    tmp = g_noleap_daysbeforemonth[value];

    if (value >= 2 && is_leap_year(leapyear)) {
        tmp++;
    }

    jdn -= tmp;

    /* At this point, value has the month into this year (zero based) and days has
     * number of days into this month (zero based)
     */

    month = value;   // zero based
    day   = jdn + 1; // one based

    utc.year = year;
    utc.month = month;
    utc.day = day;
    utc.weekday = weekday;
    utc.hour = hour;
    utc.minutes = minutes;
    utc.seconds = sec;

    return utc;
}

static inline uint32_t UTC2unix(UTCTimeStruct *utc_time)
{
    uint32_t days;
    uint16_t g_noleap_daysbeforemonth[13] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};

    days = (utc_time->year - 1970) * 365;
    days += (utc_time->year - 1969) >> 2;

    days += g_noleap_daysbeforemonth[utc_time->month];

    if (utc_time->month >= 2 && is_leap_year(utc_time->year)) {
        days++;
    }

    days += utc_time->day - 1;

    return ((days * 24 + utc_time->hour) * 60 + utc_time->minutes) * 60 + utc_time->seconds;
}

static inline uint8_t is_leap_year(uint16_t year)
{
    if (((year % 4) == 0) && ((year % 100) != 0)) {
        return 1;
    } else if ((year % 400) == 0) {
        return 1;
    } else {
        return 0;
    }
}

static inline void month_update()
{
    local_time.month++;

    if (local_time.month >= 12) {
        local_time.month = 0;
        local_time.year++;
    }
}

static inline void days_update()
{
    uint8_t month_days_list[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    };

    local_time.day++;

    if (is_leap_year(local_time.year)) {
        month_days_list[1] = 29;
    }

    uint8_t month_day = month_days_list[local_time.month];

    if (local_time.day > month_day) {
        local_time.day = 0;
        month_update();
    }

    local_time.weekday = (local_time.weekday + 1) % 7;
}

static inline void hours_update()
{
    local_time.hour++;

    if (local_time.hour == 24) {
        local_time.hour = 0;
        days_update();
    }
}

static inline void minutes_update()
{
    local_time.minutes++;

    if (local_time.minutes == 60) {
        local_time.minutes = 0;
        hours_update();
    }
}

static inline void seconds_update()
{
    local_time.seconds++;

    if (local_time.seconds == 60) {
        local_time.seconds = 0;
        minutes_update();
    }
}

static inline uint8_t next_weekday_diff_get(uint8_t weekday_now, uint8_t schedule)
{
    uint16_t schedule_tmp = 0;

    if (weekday_now == 0) {
        weekday_now = 7;
    }

    schedule_tmp = ((schedule | ((uint16_t) schedule << 7)) >> (weekday_now - 1)) & 0x7f;

    uint8_t day_diff = 0;

    while (day_diff < 7) {
        if ((schedule_tmp >> day_diff)  & 0x0001) {
            break;
        }

        day_diff++;
    }

    return day_diff;
}

static inline uint8_t is_weekday_match(uint8_t weekday_now, uint8_t schedule)
{
    uint8_t weekday_mask = weekday_now ? (uint8_t)(1 << (weekday_now - 1)) : (uint8_t)(1 << 6);
    return (weekday_mask == (schedule & weekday_mask));
}

static inline uint8_t next_weekday(uint8_t weekday_now)
{
    return (weekday_now + 1) % 7;
}

static int vendor_timer_save()
{
#ifdef VT_STORE
    int ret;

    ret = genie_flash_write_userdata(GFI_MESH_VND_TIMER, &g_timing_data, sizeof(g_timing_data));

    if (ret) {
        VT_DEBUG("vendor timers save fail %d\n", ret);
    }

    return ret;
#else
    return -1;
#endif
}

static int vendor_timer_restore()
{
#ifdef VT_STORE
    int ret;

    uint16_t len = sizeof(g_timing_data);

    memset(&g_timing_data, 0, sizeof(g_timing_data));

    ret = genie_flash_read_userdata(GFI_MESH_VND_TIMER, &g_timing_data, &len);

    if (ret) {
        VT_DEBUG("vendor timers restore fail %d\n", ret);
        return ret;
    }

    if (g_timing_data.magic != 0xABCD
        || g_timing_data.timezone < -12
        || g_timing_data.timezone > 12) {
        VT_DEBUG("vendor timers restore missmatch\n");
        return -1;
    }

    int i = 0;

    for (i = 0; i < VT_NUM; i++) {
        if (g_timing_data.timer_data[i].state != TIMER_INVAILD) {
            sys_slist_append(&g_vendor_timer.timer_list_active, &g_timing_data.timer_data[i].next);
        } else {
            sys_slist_append(&g_vendor_timer.timer_list_idle, &g_timing_data.timer_data[i].next);
        }

        VT_DEBUG("restore vendor timer index %d state %d periodic %d periodic_time %d schedule %d unixtime_match %d\n",
                 g_timing_data.timer_data[i].index, g_timing_data.timer_data[i].state,
                 g_timing_data.timer_data[i].periodic, g_timing_data.timer_data[i].periodic_time,
                 g_timing_data.timer_data[i].schedule, g_timing_data.timer_data[i].unixtime_match);
    }

    return 0;
#else
    return -1;
#endif
}

static uint8_t is_vendor_timer_timeout(struct vendor_timer_t *vendor_timer)
{
    if (vendor_timer->state == TIMER_INVAILD) {
        return 0;
    }

    if (vendor_timer->periodic) {
        if (is_weekday_match(local_time.weekday, vendor_timer->schedule)
            && vendor_timer->unixtime_match < g_vendor_timer.unix_time) {
            vendor_timer->unixtime_match += (1 + next_weekday_diff_get(next_weekday(local_time.weekday), vendor_timer->schedule)) * DAY;
        }
    }

    return vendor_timer->unixtime_match == g_vendor_timer.unix_time;
}

static void vendor_timer_check()
{
    struct vendor_timer_t *tmp, *node;
    SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&g_vendor_timer.timer_list_active, node,  tmp, next) {
        if (is_vendor_timer_timeout(node)) {
            if (g_vendor_timer.cb) {
                g_vendor_timer.cb(VT_TIMEOUT, node->index, &node->attr_data);
            }

            VT_LOCK;

            if (!node->periodic) {
                node->unixtime_match = 0xffffffff;
                node->state = TIMER_INVAILD;
                sys_slist_find_and_remove(&g_vendor_timer.timer_list_active, &node->next);
                sys_slist_append(&g_vendor_timer.timer_list_idle, &node->next);
            } else {
                node->unixtime_match += 24 * HOUR;
            }

            VT_UNLOCK;
            vendor_timer_save();
        }
    }

    if (g_vendor_timer.unix_time_sync_match
        && g_vendor_timer.unix_time_sync_match <= g_vendor_timer.unix_time) {
        if (g_vendor_timer.cb) {
            int ret = g_vendor_timer.cb(VT_TIMING_SYNC, 0, NULL);

            if (ret) {
                if (g_vendor_timer.unix_time_sync_retry_times > 0) {
                    g_vendor_timer.unix_time_sync_match += g_timing_data.timing_sync_config.retry_delay * MINU;
                    g_vendor_timer.unix_time_sync_retry_times--;
                    return;
                }
            }
        }

        g_vendor_timer.unix_time_sync_retry_times = g_timing_data.timing_sync_config.retry_times;
        g_vendor_timer.unix_time_sync_match == g_vendor_timer.unix_time + g_timing_data.timing_sync_config.period_time *MINU;
    }
}

static void timer_update(void *args)
{
    if (!g_vendor_timer.update) {
        return;
    }

    g_vendor_timer.unix_time += 1;
    seconds_update();

    k_work_submit(&g_vendor_timer.work);

    if (g_vendor_timer.unix_time % 60 == 0) {
        VT_DEBUG("timer_update %d\n", g_vendor_timer.unix_time);
    }
}

void timer_check_work(struct k_work *work)
{
    vendor_timer_check();
}

vendor_timer_handle_t vendor_timer_find(uint8_t index)
{
    if (index >= VT_NUM) {
        return NULL;
    }

    VT_LOCK;
    struct vendor_timer_t *tmp, *node;
    sys_slist_t *list;
    list = &g_vendor_timer.timer_list_active;

    SYS_SLIST_FOR_EACH_CONTAINER_SAFE(list, node,  tmp, next) {
        if (node->index == index) {
            VT_UNLOCK;
            return node;
        }
    }
    VT_UNLOCK;

    return NULL;
}

static struct vendor_timer_t *vendor_timer_new()
{
    struct vendor_timer_t *free_timer;
    VT_LOCK;
    free_timer = sys_slist_get(&g_vendor_timer.timer_list_idle);
    VT_UNLOCK;
    VT_DEBUG("timer new %p\n", free_timer);
    return free_timer;
}

int vendor_timer_start(uint8_t index, uint32_t unix_time, vendor_attr_data_t *attr_data)
{
    struct vendor_timer_t *vendor_timer;

    if (!attr_data) {
        return -VT_E_PARAM;
    }

    VT_DEBUG("timer start index %d unix_time %d on_off %d\n",
             index,  unix_time, attr_data->type);

    if (!g_vendor_timer.init) {
        return -VT_E_INIT;
    }

    if (!g_vendor_timer.update) {
        return -VT_E_LOCALTIME_NOTSET;
    }

    if (index >= VT_NUM) {
        //return -VT_E_INDEX;
    }

    if (unix_time <= g_vendor_timer.unix_time) {
        return -VT_E_PARAM;
    }

    vendor_timer = vendor_timer_find(index);

    if (vendor_timer == NULL) {
        vendor_timer = vendor_timer_new();

        if (vendor_timer == NULL) {
            return -VT_E_NORESOURCE;
        }
    } else {
        VT_LOCK;
        sys_slist_find_and_remove(&g_vendor_timer.timer_list_active, &vendor_timer->next);
        VT_UNLOCK;
    }

    vendor_timer->index = index;
    vendor_timer->unixtime_match = unix_time;// + g_vendor_timer.timezone * HOUR;
    vendor_timer->state = TIMER_ON;
    vendor_timer->attr_data.type = attr_data->type;
    vendor_timer->attr_data.para = attr_data->para;

    VT_LOCK;
    sys_slist_append(&g_vendor_timer.timer_list_active, &vendor_timer->next);
    VT_UNLOCK;

    vendor_timer_save();

    return 0;
}

int vendor_timer_utc_start(uint8_t index, UTCTimeStruct utc_time, vendor_attr_data_t *attr_data)
{
    VT_DEBUG("utc timer start index %d on_off %d %4d/%2d/%2d %2d:%2d:%d weekday %2d %04d\n",
             index, attr_data->para,
             utc_time.year, utc_time.month + 1, utc_time.day,
             utc_time.hour, utc_time.minutes, utc_time.seconds,
             utc_time.weekday, g_timing_data.timezone);

    return vendor_timer_start(index, UTC2unix(&utc_time), attr_data);
}

int vendor_timer_periodic_start(uint8_t index, uint16_t periodic_time, uint8_t schedule, vendor_attr_data_t *attr_data)
{
    struct vendor_timer_t *vendor_timer;

    VT_DEBUG("periodic timer start index %d periodic_time %d schedule %d on_off %d\n",
             index,  periodic_time,  schedule, attr_data->para);

    if (!g_vendor_timer.init) {
        return -VT_E_INIT;
    }

    if (!g_vendor_timer.update) {
        return -VT_E_LOCALTIME_NOTSET;
    }

    if (index >= VT_NUM) {
        //return -VT_E_INDEX;
    }

    if (schedule == 0) {
        return -VT_E_PARAM;
    }

    vendor_timer = vendor_timer_find(index);

    if (vendor_timer == NULL) {
        vendor_timer = vendor_timer_new();

        if (vendor_timer == NULL) {
            return -VT_E_NORESOURCE;
        }
    } else {
        VT_LOCK;
        sys_slist_find_and_remove(&g_vendor_timer.timer_list_active, &vendor_timer->next);
        VT_UNLOCK;
    }

    vendor_timer->index = index;
    vendor_timer->periodic = 1;
    vendor_timer->periodic_time = periodic_time;
    vendor_timer->schedule = schedule;
    vendor_timer->state = TIMER_ON;

    UTCTimeStruct utc = local_time;
    utc.hour = 0;
    utc.minutes = 0;
    utc.seconds = 0;
    utc.day = utc.day + next_weekday_diff_get(local_time.weekday, schedule);

    vendor_timer->unixtime_match = UTC2unix(&utc) + periodic_time - g_timing_data.timezone * HOUR;

    VT_DEBUG("periodic timer unixtime_match %d\n", vendor_timer->unixtime_match);

    VT_LOCK;
    sys_slist_append(&g_vendor_timer.timer_list_active, &vendor_timer->next);
    VT_UNLOCK;
    vendor_timer_save();
    return 0;
}

int vendor_timer_stop(int8_t index)
{
    VT_DEBUG("timer stop %d\n", index);

    if (!g_vendor_timer.init) {
        return -VT_E_INIT;
    }

    if (!g_vendor_timer.update) {
        return -VT_E_LOCALTIME_NOTSET;
    }

    if (index >= VT_NUM) {
        return -VT_E_INDEX;
    }

    struct vendor_timer_t *vendor_timer = vendor_timer_find(index);

    if (vendor_timer == NULL) {
        return -VT_E_INDEX;
    } else {
        VT_LOCK;
        vendor_timer->index = 0xFF;
        vendor_timer->state = TIMER_INVAILD;
        vendor_timer->unixtime_match = 0;
        sys_slist_find_and_remove(&g_vendor_timer.timer_list_active, &vendor_timer->next);
        sys_slist_append(&g_vendor_timer.timer_list_idle, &vendor_timer->next);
        VT_UNLOCK;
    }

    return 0;
}

int vendor_timer_remove(int8_t index)
{
    int i;

    VT_DEBUG("timer remove %d\n", index);

    /* remove alll timers */
    if (index == 0xFF) {
        for (i = 0; i < VT_NUM; i++) {
            vendor_timer_stop(i);
        }

        return 0;
    }

    int ret = vendor_timer_stop(index);

    vendor_timer_save();

    return ret;
}

void vendor_timer_local_time_show()
{
    printk("%4d/%2d/%2d %2d:%2d:%d weekday %2d %04d\n",
           local_time.year, local_time.month + 1, local_time.day,
           local_time.hour, local_time.minutes, local_time.seconds,
           local_time.weekday, g_timing_data.timezone);
}

int vendor_timer_timezone_update(int8_t timezone)
{
    VT_DEBUG("timezone update %d\n", timezone);

    if (timezone < -12 || timezone > 12) {
        return -VT_E_PARAM;
    }

    if (!g_vendor_timer.init) {
        return -VT_E_INIT;
    }

    g_timing_data.timezone = timezone;

    return 0;
}

int8_t vendor_timer_timezone_get()
{
    return g_timing_data.timezone;
}

int vendor_timer_time_sync_set(uint16_t period_time, uint8_t retry_delay,   uint8_t retry_times)
{
    VT_DEBUG("timing sync set period_time %d retry_delay %d retry_times %d\n",
             period_time, retry_delay, retry_times);

    if (period_time == 0 || retry_delay == 0 || retry_times) {
        return -VT_E_PARAM;
    }

    g_timing_data.timing_sync_config.period_time = period_time;
    g_timing_data.timing_sync_config.retry_delay = retry_delay;
    g_timing_data.timing_sync_config.retry_times = retry_times;

    g_vendor_timer.unix_time_sync_match = g_vendor_timer.unix_time +  g_timing_data.timing_sync_config.period_time * MINU;
    g_vendor_timer.unix_time_sync_retry_times = retry_times;

    return 0;
}

int vendor_timer_time_sync_get(uint16_t *period_time, uint8_t *retry_delay, uint8_t *retry_times)
{
    *period_time = g_timing_data.timing_sync_config.period_time;
    *retry_delay = g_timing_data.timing_sync_config.retry_delay;
    *retry_times = g_timing_data.timing_sync_config.retry_times;

    return 0;
}

int vendor_timer_local_time_update(uint32_t unix_time)
{
    if (!g_vendor_timer.init) {
        return -VT_E_INIT;
    }

    g_vendor_timer.update = 1;
    g_vendor_timer.unix_time = unix_time;

    local_time = unix2UTC(unix_time + g_timing_data.timezone * HOUR);

    VT_DEBUG("unix_time %d\n", unix_time);
    VT_DEBUG("localtime update %4d/%2d/%2d %2d:%2d:%d weekday %2d\n",
             local_time.year, local_time.month + 1, local_time.day,
             local_time.hour, local_time.minutes, local_time.seconds,
             local_time.weekday);
    VT_DEBUG("unix_time revert %d\n", UTC2unix(&local_time));

    return 0;
}

int vendor_timer_init(vendor_timer_event_func_t cb)
{
    int i;

    if (g_vendor_timer.init) {
        return 0;
    }

    if (cb == NULL) {
        return -VT_E_INIT;
    }

    memset(&g_vendor_timer, 0, sizeof(g_vendor_timer));
    memset(&local_time, 0, sizeof(local_time));

    g_vendor_timer.cb = cb;

    sys_slist_init(&g_vendor_timer.timer_list_active);
    sys_slist_init(&g_vendor_timer.timer_list_idle);

    k_sem_init(&g_vendor_timer.lock, 1, 1);

    k_work_init(&g_vendor_timer.work, timer_check_work);
    g_vendor_timer.timer.port = 0;
    g_vendor_timer.timer.config.period = 1000000;
    g_vendor_timer.timer.config.reload_mode = TIMER_RELOAD_AUTO;
    g_vendor_timer.timer.config.cb = timer_update;
    g_vendor_timer.timer.config.arg = NULL;
    hal_timer_init(&g_vendor_timer.timer);
    hal_timer_start(&g_vendor_timer.timer);

    g_vendor_timer.init = 1;

    if (vendor_timer_restore()) {
        memset(&g_timing_data, 0, sizeof(g_timing_data));
        g_timing_data.timezone = 8;
        g_timing_data.magic = 0xABCD;

        for (i = 0; i < VT_NUM; i++) {
            g_timing_data.timer_data[i].unixtime_match = 0xffffffff;
            g_timing_data.timer_data[i].index = 0xFF;
            g_timing_data.timer_data[i].state = TIMER_INVAILD;
            sys_slist_append(&g_vendor_timer.timer_list_idle, &g_timing_data.timer_data[i].next);
        }
    }

    //sync timing
    g_vendor_timer.cb(VT_TIMING_SYNC, 0, NULL);

    VT_DEBUG("vendor_timer_init\n");

    return 0;
}
