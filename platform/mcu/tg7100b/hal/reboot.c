

#include <csi_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <hal/soc/flash.h>
extern void drv_reboot(void);

void hal_boot(hal_partition_t partition)
{


}

void hal_reboot(void)
{
	drv_reboot();
}

