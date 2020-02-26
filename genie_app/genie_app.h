/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GENIE_APP_H_
#define _GENIE_APP_H_

#include "base/genie_cmds.h"
#include "base/genie_event.h"
#include "base/genie_flash.h"
#include "base/genie_reset.h"
#include "base/tri_tuple.h"

#include "bluetooth/mesh/genie_mesh.h"

/**
 * @brief The initialization api for genie sdk. The application always
 *        involks this api to initialize AIS/Mesh procedure for bussiness.
 */
void genie_init(void);

#endif  //_GENIE_APP_H_
