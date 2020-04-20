/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef TRI_TUPLE_DEFAULT_H_
#define TRI_TUPLE_DEFAULT_H_


#if defined(BOARD_TG7100B) || defined(BOARD_CH6121EVB)
/* default UUID for identifying the unprovisioned node */
#define DEFAULT_PID 10857
#define DEFAULT_SECRET "7f5a348ad47baac74e48b8d6e980cb83"
#define DEFAULT_MAC "f8a7638ca646"
#else
/* default UUID for identifying the unprovisioned node */
// 602 1565d0497400c999e984cffa4da4fdf9 78da07c11ca8
#define DEFAULT_PID 602
#define DEFAULT_SECRET "1565d0497400c999e984cffa4da4fdf9"
#define DEFAULT_MAC "78da07c11ca8"

#endif
#if 0
#define DEFAULT_PID 761
#define DEFAULT_SECRET "8a99315d87da5d24db777cb7a0f9d687"
#define DEFAULT_MAC { 0xd2, 0x0a, 0x02, 0x3a, 0x9e, 0x10 }
#endif

#endif // TRI_TUPLE_DEFAULT_H_
