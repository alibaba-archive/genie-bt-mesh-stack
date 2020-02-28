
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
