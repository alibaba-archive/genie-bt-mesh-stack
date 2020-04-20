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

#ifndef YOC_NVRAM_H
#define YOC_NVRAM_H

#include <stdint.h>

/**
 * This function will get data from the factory setting area.
 *
 * @param[in]   key   the data pair of the key, less than 64 bytes
 * @param[in]   size  the size of the buffer
 * @param[out]  buf   the buffer that will store the data
 * @return  the length of the data value, error code otherwise
 */
int nvram_get_val(const char *key, void *buf ,int size);

/**
 * This function will set data to the factory setting area.
 *
 * @param[in]   key   the data pair of the key, less than 64 bytes
 * @param[in]   value the data pair of the value, delete the pair if value == NULL
 * @return  the length of the data value, error code otherwise
 */
int nvram_set_val(const char *key, char *value);


/**
 * This function will get ali iot info from factory setting area.
 *
 * @param[out] product_key output ali iot product key
 * @param[in&out] product_key_len in buffer len, out real len
 * @param[out] device_name output ali iot device name
 * @param[in&out] device_name_len in buffer len, out real len
 * @param[out] device_secret output ali iot device device secret
 * @param[in&out] device_secret_len in buffer len, out real len
 */
int nvram_get_iot_info(char *product_key, uint32_t *product_key_len, 
                            char *device_name, uint32_t *device_name_len, 
                            char *device_secret, uint32_t *device_secret_len);

#endif

