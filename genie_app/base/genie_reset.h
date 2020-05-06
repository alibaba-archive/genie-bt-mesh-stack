/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _GENIE_RESET_H_
#define _GENIE_RESET_H_

#define GENIE_RESET_BY_REPEAT_COUNTER		4
#define GENIE_RESET_BY_REPEAT_TIMEOUT		(3*1000)
#define GENIE_RESET_BY_REPEAT_FLASH_COUNTER		5
#define GENIE_RESET_BY_REPEAT_FLASH_DURATION    500

#define GENIE_RESET_WAIT_TIMEOUT            (5*1000)

uint8_t genie_reset_get_flag(void);

void genie_reset_done(void);

/**
 * @brief initialize the function that resets whole system by multiple clicks
 */
void genie_reset_by_repeat_init(void);
/**
 * @brief cleans the recored count for clicks.
 */
void genie_reset_clean_count(void);

#endif // _GENIE_RESET_H_
