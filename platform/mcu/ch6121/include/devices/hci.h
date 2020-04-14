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

#ifndef DEVICES_HCI_H_
#define DEVICES_HCI_H_

#include "hal/hci_impl.h"

#define hci_open(name) device_open(name)
#define hci_open_id(name, id) device_open_id(name, id)
#define hci_close(dev) device_close(dev)

/**
  \brief       send hci format data
  \param[in]   dev      Pointer to device object.
  \param[out]  data     data address to store data read.
  \param[in]   size     data length expected to read.
  \return      0 on success, else on fail.
*/
int hci_send(yoc_dev_t *dev, void *data, uint32_t size);

#endif
