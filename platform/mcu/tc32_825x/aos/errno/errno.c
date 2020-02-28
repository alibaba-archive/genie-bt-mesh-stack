

#include "errno.h"

// TODO: Support multiple contexts
static int m_errno_main;

int *
__error(void)
{
    return &m_errno_main;
}

void
set_errno(int err_code)
{
    m_errno_main = err_code;
}
