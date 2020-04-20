/*
 * Copyright (C) 2017 C-SKY Microsystems Co., All rights reserved.
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

#ifndef __NETWORK_H_
#define __NETWORK_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>


typedef struct network {
    int fd;
    struct sockaddr_in address;
    int (*net_connect)(struct network *n, char *addr, int port, int net_type);
    int (*net_read)(struct network *, unsigned char *, int, int);
    int (*net_write)(struct network *, unsigned char *, int, int);
    void (*net_disconncet)(struct network *n);
} network_t;

void network_init_http(network_t *n);

int ntp_sync_time(char *server);

#endif

