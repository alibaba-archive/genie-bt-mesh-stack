#ifndef YOC_NETIO_H
#define YOC_NETIO_H

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <aos/list.h>

#include <aos/kernel.h>
#include <yoc/network.h>

typedef struct netio_cls netio_cls_t;

typedef struct {
    const netio_cls_t *cls;
    size_t offset;
    size_t size;
    size_t block_size;

    void *private;
} netio_t;

struct netio_cls {
    const char *name;
    int (*open)(netio_t *io, const char *path);
    int (*close)(netio_t *io);

    int (*read)(netio_t *io, uint8_t *buffer, int length, int timeoutms);
    int (*write)(netio_t *io, uint8_t *buffer, int length, int timeoutms);
    int (*remove)(netio_t *io);
    int (*seek)(netio_t *io, size_t offset, int whence);
    // int (*getinfo)(netio_t *io, fota_info_t *info);
};

int netio_register(const netio_cls_t *cls);

int netio_register_http(void);
int netio_register_flash(void);
int netio_register_coap(void);

netio_t *netio_open(const char *path);
int netio_close(netio_t *io);
int netio_read(netio_t *io, uint8_t *buffer, size_t lenght, int timeoutms);
int netio_write(netio_t *io, uint8_t *buffer, size_t lenght, int timeoutms);
int netio_seek(netio_t *io, size_t offset, int whence);
// int netio_getinfo(netio_t *io, fota_info_t *info);

#endif