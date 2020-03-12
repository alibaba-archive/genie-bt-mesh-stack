#ifndef __ROM_CALL_FLASH_H_
#define __ROM_CALL_FLASH_H_

#include "ke_msg.h"
#include "ke_event.h"
#include "co_bt.h"            // common bt definitions
#include "ke_task.h"
//#include "gapm_task.h"

//#include "rwble_hl.h"
//#include "prf.h"
//#include "lld_sleep.h"
//#include "rwip.h"
//#include "ea.h"
#include "app.h"
#include <string.h>
#include "k_type.h"


struct rom_env_tag
{
	//void (*rwip_reset)(void);

	void (*rwip_prevent_sleep_set)(uint16_t prv_slp_bit);
	void (*rwip_prevent_sleep_clear)(uint16_t prv_slp_bit);
	uint32_t (*rwip_sleep_lpcycles_2_us)(uint32_t lpcycles);
	uint32_t (*rwip_us_2_lpcycles)(uint32_t us);
	void (*rwip_wakeup_delay_set)(uint16_t wakeup_delay);
	
	void (*appm_init)(void);
	
	void (*platform_reset)(uint32_t error);
	
	void(*assert_err) (const char *condition, const char * file, int line);
	
	void(*assert_param)(int param0, int param1, const char * file, int line);

	void (*assert_warn)(int param0, int param1, const char * file, int line);
		
	void (*hci_event_process)(uint8_t type, uint8_t *buf, uint16_t len);

#ifdef ALIOS_KERNEL
	kstat_t (*krhino_sem_create)(ksem_t *sem, const name_t *name, sem_count_t count);
	kstat_t (*krhino_sem_give)(ksem_t *sem);
#endif
};
 

void rom_env_init(struct rom_env_tag *api);

extern struct rom_env_tag rom_env;

#endif // __ROM_CALL_FLASH_H_

