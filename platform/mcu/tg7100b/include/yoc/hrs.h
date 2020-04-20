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

#ifndef _BT_HRS_H_
#define _BT_HRS_H_

typedef struct _hrs_t {
    uint16_t conn_handle;
    uint16_t hrs_svc_handle;
    int16_t mea_ccc;
    uint8_t hrs_mea_flag;
    uint8_t hrs_mea_level;
    slist_t next;
} hrs_t;

typedef hrs_t *hrs_handle_t;

hrs_handle_t hrs_init(hrs_t *hrs);
int hrs_mea_level_update(hrs_handle_t handle, uint8_t *data, uint8_t length);

#endif

