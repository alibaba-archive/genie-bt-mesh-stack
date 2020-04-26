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

#endif //_VENDOR_MODEL_SRV_H
