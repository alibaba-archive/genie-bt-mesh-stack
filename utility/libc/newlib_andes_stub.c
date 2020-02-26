/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <reent.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include <stdarg.h>
#include <k_api.h>
#include <aos/aos.h>
#include "hal/soc/soc.h"
#include "vfs_conf.h"
#include "aos/network.h"
#ifdef WITH_LWIP_TELNETD
#include "lwip/apps/telnetserver.h"
#endif

#define FD_VFS_START AOS_CONFIG_VFS_FD_OFFSET
#define FD_VFS_END   (FD_VFS_START + MAX_FILE_NUM - 1)

#ifdef POSIX_DEVICE_IO_NEED
#ifdef WITH_LWIP
#define FD_SOCKET_START FD_AOS_SOCKET_OFFSET
#define FD_SOCKET_END   (FD_AOS_SOCKET_OFFSET + FD_AOS_NUM_SOCKETS - 1)
#define FD_EVENT_START  FD_AOS_EVENT_OFFSET
#define FD_EVENT_END    (FD_AOS_EVENT_OFFSET + FD_AOS_NUM_EVENTS - 1)
#endif
#endif

int _execve_r(struct _reent *ptr, const char *name, char *const *argv,
              char *const *env)
{
    return -1;
}

int _fcntl_r(struct _reent *ptr, int fd, int cmd, int arg)
{
    if ((fd >= FD_VFS_START) && (fd <= FD_VFS_END)) {
        return aos_fcntl(fd, cmd, arg);
#ifdef POSIX_DEVICE_IO_NEED
#ifdef WITH_LWIP
    } else if ((fd >= FD_SOCKET_START) && (fd <= FD_EVENT_END)) {
        return lwip_fcntl(fd, cmd, arg);
#endif
#endif
    } else {
        return -1;
    }
}

int _fork_r(struct _reent *ptr)
{
    return -1;
}

int _getpid_r(struct _reent *ptr)
{
    return 0;
}

int _isatty_r(struct _reent *ptr, int fd)
{
    if (fd >= 0 && fd < 3) {
        return 1;
    }

    return -1;
}

int _kill_r(struct _reent *ptr, int pid, int sig)
{
    return -1;
}

int _link_r(struct _reent *ptr, const char *old, const char *new)
{
    return -1;
}

_off_t _lseek_r(struct _reent *ptr, int fd, _off_t pos, int whence)
{
    return aos_lseek(fd, pos, whence);
}

int _mkdir_r(struct _reent *ptr, const char *name, int mode)
{
    return aos_mkdir(name);
}

int _open_r(struct _reent *ptr, const char *file, int flags, int mode)
{
    return aos_open(file, flags);
}

int _close_r(struct _reent *ptr, int fd)
{
    if ((fd >= FD_VFS_START) && (fd <= FD_VFS_END)) {
        return aos_close(fd);
#ifdef POSIX_DEVICE_IO_NEED
#ifdef WITH_LWIP
    } else if ((fd >= FD_SOCKET_START) && (fd <= FD_EVENT_END)) {
        return lwip_close(fd);
#endif
#endif
    } else {
        return -1;
    }
}

_ssize_t _read_r(struct _reent *ptr, int fd, void *buf, size_t nbytes)
{
    if ((fd >= FD_VFS_START) && (fd <= FD_VFS_END)) {
        return aos_read(fd, buf, nbytes);
#ifdef POSIX_DEVICE_IO_NEED
#ifdef WITH_LWIP
    } else if ((fd >= FD_SOCKET_START) && (fd <= FD_EVENT_END)) {
        return lwip_read(fd, buf, nbytes);
#endif
#endif
    } else {
        return -1;
    }
}

/*
 * implement _write_r here
 */
_ssize_t _write_r(struct _reent *ptr, int fd, const void *buf, size_t nbytes)
{
    const char *tmp = buf;
    int         i   = 0;
    uart_dev_t  uart_stdio;

    memset(&uart_stdio, 0, sizeof(uart_stdio));
    uart_stdio.port = 0;

    if ((fd >= FD_VFS_START) && (fd <= FD_VFS_END)) {
        return aos_write(fd, buf, nbytes);
#ifdef POSIX_DEVICE_IO_NEED
#ifdef WITH_LWIP
    } else if ((fd >= FD_SOCKET_START) && (fd <= FD_EVENT_END)) {
        return lwip_write(fd, buf, nbytes);
#endif
#endif
    } else if ((fd == STDOUT_FILENO) || (fd == STDERR_FILENO)) {
        for (i = 0; i < nbytes; i++) {
            if (*tmp == '\n') {
#ifdef WITH_LWIP_TELNETD
                TelnetWrite('\r');
#endif
                hal_uart_send(&uart_stdio, (void *)"\r", 1, 0);
            }

#ifdef WITH_LWIP_TELNETD
            TelnetWrite(*tmp);
#endif
            hal_uart_send(&uart_stdio, (void *)tmp, 1, 0);
            tmp++;
        }

        return nbytes;
    } else {
        return -1;
    }
}

#ifdef POSIX_DEVICE_IO_NEED
int ioctl(int fildes, int request, ... /* arg */)
{
    long    arg  = 0;
    void   *argp = NULL;
    va_list args;

    va_start(args, request);

    if ((fildes >= AOS_CONFIG_VFS_FD_OFFSET) &&
        (fildes <= (AOS_CONFIG_VFS_FD_OFFSET + MAX_FILE_NUM - 1))) {
        arg = va_arg(args, int);
        return aos_ioctl(fildes, request, arg);
#ifdef WITH_LWIP
    } else if ((fildes >= FD_AOS_SOCKET_OFFSET) &&
               (fildes <= (FD_AOS_EVENT_OFFSET + FD_AOS_NUM_EVENTS - 1))) {
        argp = va_arg(args, void *);
        return lwip_ioctl(fildes, request, argp);
#endif
    } else {
        return -1;
    }
}
#endif

int _fstat_r(struct _reent *ptr, int fd, struct stat *pstat)
{
    return -1;
}

int _rename_r(struct _reent *ptr, const char *old, const char *new)
{
    return 0;
}

void *_sbrk_r(struct _reent *ptr, ptrdiff_t incr)
{
    return NULL;
}

int _stat_r(struct _reent *ptr, const char *file, struct stat *pstat)
{
    return aos_stat(file, pstat);
}

_CLOCK_T_ _times_r(struct _reent *ptr, struct tms *ptms)
{
    return -1;
}

int _unlink_r(struct _reent *ptr, const char *file)
{
    return aos_unlink(file);
}

int _wait_r(struct _reent *ptr, int *status)
{
    return -1;
}

int _gettimeofday_r(struct _reent *ptr, struct timeval *tv, void *__tzp)
{
    uint64_t t  = aos_now_ms();
    tv->tv_sec  = t / 1000;
    tv->tv_usec = (t % 1000) * 1000;
    return 0;
}

#if (RHINO_CONFIG_MM_TLF > 0)
#define AOS_UNSIGNED_INT_MSB (1u << (sizeof(unsigned int) * 8 - 1))
extern void *aos_malloc(unsigned int size);
extern void aos_alloc_trace(void *addr, size_t allocator);
extern void aos_free(void *mem);
extern void *aos_realloc(void *mem, unsigned int size);

void *malloc(size_t size)
{
    void *mem;

#if (RHINO_CONFIG_MM_DEBUG > 0u && RHINO_CONFIG_GCC_RETADDR > 0u)
    mem = aos_malloc(size | AOS_UNSIGNED_INT_MSB);
#else
    mem = aos_malloc(size);
#endif

    return mem;
}

void free(void *mem)
{
    aos_free(mem);
}

void *realloc(void *old, size_t newlen)
{
    void *mem;

#if (RHINO_CONFIG_MM_DEBUG > 0u && RHINO_CONFIG_GCC_RETADDR > 0u)
    mem = aos_realloc(old, newlen | AOS_UNSIGNED_INT_MSB);
#else
    mem = aos_realloc(old, newlen);
#endif

    return mem;
}

void *calloc(size_t len, size_t elsize)
{
    void *mem;

#if (RHINO_CONFIG_MM_DEBUG > 0u && RHINO_CONFIG_GCC_RETADDR > 0u)
    mem = aos_malloc((elsize * len) | AOS_UNSIGNED_INT_MSB);
#else
    mem = aos_malloc(elsize * len);
#endif

    if (mem) {
        memset(mem, 0, elsize * len);
    }

    return mem;
}

char * strdup(const char *s)
{
    size_t  len = strlen(s) +1;
    void *dup_str = aos_malloc(len);
    if (dup_str == NULL)
        return NULL;
    return (char *)memcpy(dup_str, s, len);
}

#else /* (RHINO_CONFIG_MM_TLF > 0) */

void *_malloc_r(struct _reent *ptr, size_t size)
{
    void *mem;

#if (RHINO_CONFIG_MM_DEBUG > 0u && RHINO_CONFIG_GCC_RETADDR > 0u)
    mem = aos_malloc(size | AOS_UNSIGNED_INT_MSB);
    aos_alloc_trace(mem, (size_t)__builtin_return_address(0));
#else
    mem = aos_malloc(size);
#endif

    return mem;
}

void *_realloc_r(struct _reent *ptr, void *old, size_t newlen)
{
    void *mem;

#if (RHINO_CONFIG_MM_DEBUG > 0u && RHINO_CONFIG_GCC_RETADDR > 0u)
    mem = aos_realloc(old, newlen | AOS_UNSIGNED_INT_MSB);
    aos_alloc_trace(mem, (size_t)__builtin_return_address(0));
#else
    mem = aos_realloc(old, newlen);
#endif

    return mem;
}

void *_calloc_r(struct _reent *ptr, size_t size, size_t len)
{
    void *mem;

#if (RHINO_CONFIG_MM_DEBUG > 0u && RHINO_CONFIG_GCC_RETADDR > 0u)
    mem = aos_malloc((size * len) | AOS_UNSIGNED_INT_MSB);
    aos_alloc_trace(mem, (size_t)__builtin_return_address(0));
#else
    mem = aos_malloc(size * len);
#endif

    if (mem) {
        bzero(mem, size * len);
    }

    return mem;
}

void _free_r(struct _reent *ptr, void *addr)
{
    aos_free(addr);
}

#endif /* (RHINO_CONFIG_MM_TLF > 0) */

void _exit(int status)
{
    while (1)
        ;
}

void _system(const char *s)
{
    return;
}

void abort(void)
{
    while (1)
        ;
}
