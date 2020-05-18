/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _VENDOR_TIMER_H
#define _VENDOR_TIMER_H

#pragma pack(1)
typedef struct {
    uint16_t type;
    uint8_t para;
} vendor_attr_data_t;
#pragma pack()

#define DEF_SYNC_PERIOD 180
#define DEF_SYNC_DELAY 10
#define DEF_SYNC_DELAY_RETRY 10

typedef struct {
    volatile uint16_t year;    // 2019+
    volatile uint8_t month;    // 0-11
    volatile uint8_t day;      // 1-31
    volatile uint8_t seconds;  // 0-59
    volatile uint8_t minutes;  // 0-59
    volatile uint8_t hour;     // 0-23
    volatile uint8_t weekday; // 0 means sunday
} UTCTimeStruct;

typedef enum {
    VT_OK = 0,
    VT_E_INIT,
    VT_E_LOCALTIME_NOTSET,
    VT_E_INDEX,
    VT_E_PARAM,
    VT_E_NORESOURCE,
    VT_E_OTHER,
} vendor_timer_errno;

enum {
    VT_TIMEOUT = 0,
    VT_TIMING_SYNC = 1,
};

typedef void *vendor_timer_handle_t;
typedef int (*vendor_timer_event_func_t)(uint8_t event, uint8_t index, vendor_attr_data_t *data);

uint32_t vendor_timer_local_unixtime_get();
int8_t vendor_timer_timezone_get();
int vendor_timer_time_sync_get(uint16_t *period_time, uint8_t *retry_delay, uint8_t *retry_times);
int vendor_timer_start(uint8_t index, uint32_t unix_time, vendor_attr_data_t *attr_data);
int vendor_timer_periodic_start(uint8_t index, uint16_t periodic_time, uint8_t schedule, vendor_attr_data_t *attr_data);
int vendor_timer_remove(int8_t index);
int vendor_timer_local_time_update(uint32_t unix_time);
int vendor_timer_timezone_update(int8_t timezone);
void vendor_timer_local_time_show();
int vendor_timer_init(vendor_timer_event_func_t cb);
int vendor_timer_time_sync_set(uint16_t period_time, uint8_t retry_delay,   uint8_t retry_times);



#endif //_VENDOR_MODEL_SRV_H
