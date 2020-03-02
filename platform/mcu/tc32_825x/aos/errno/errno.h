/********************************************************************************************************
 * @file     errno.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#ifndef TLK_ERRNO_H__
#define TLK_ERRNO_H__

#ifdef __cplusplus
extern "C" {
#endif

int * __error(void);
void  set_errno(int);

#undef errno
#define errno (* __error())

#ifdef __cplusplus
}
#endif

#endif // ERRNO_H__
