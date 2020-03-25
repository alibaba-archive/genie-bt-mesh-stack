/*
 * Copyright (C) 2018-2020 Alibaba Group Holding Limited
 */

#ifndef _TRI_TUPLE_H_
#define _TRI_TUPLE_H_

/**
 * @brief writes the trituple info into flash.
 * @param[in] p_pid: product ID
 * @param[in] p_mac: device address
 * @param[in] p_key: secret key
 * @return the status of the writing operation, 0 for success.
 */
E_GENIE_FLASH_ERRCODE genie_flash_write_trituple(uint32_t *p_pid, uint8_t *p_mac,  uint8_t *p_key);

/**
 * @brief get the trituple info from flash.
 * @param[in] p_pid
 * @param[in] p_mac
 * @param[in] p_key
 * @return the status of the reading operation, 0 for success.
 */
E_GENIE_FLASH_ERRCODE genie_flash_read_trituple(uint32_t *p_pid, uint8_t *p_mac,  uint8_t *p_key);

/**
 * @brief get the trituple info formatted in uuid
 * @return the string refering to uuid
 */
uint8_t *genie_tri_tuple_get_uuid(void);
#ifdef GENIE_OLD_AUTH
/**
 * @brief get the authentication info
 * @return the authenticated sequence
 */
uint8_t *genie_tri_tuple_get_auth(void);
#else
/**
 * @brief get the authentication info
 * @param[in] random: the random sequence used for calculating.
 * @return the authenticated sequence
 */
uint8_t *genie_tri_tuple_get_auth(const uint8_t random[16]);
#endif
/**
 * @brief check is the pid is correct or not
 * @param[in] pid: the pid to be checked
 * @return 0 for incorrect, otherwise correct.
 */
uint8_t genie_tri_tuple_check_pid(uint32_t pid);
/**
 * @brief load the trituple info from flash.
 * @return 0 for success, otherwise failed.
 */
int8_t genie_tri_tuple_load(void);

/**
 * @brief dump the trituple info.
 */
void genie_tri_tuple_show(void);

#ifdef CONFIG_GENIE_OTA
/**
 * @brief encrypt ais data.
 * @param[in] raw data
 * @param[out] encryped data
 */
void genie_ais_encrypt(const uint8_t data_in[16], uint8_t data_out[16]);

/**
 * @brief decrypt ais data.
 * @param[in] encryped data
 * @param[out] raw data
 */
void genie_ais_decrypt(const uint8_t data_in[16], uint8_t data_out[16]);

/**
 * @brief get the cipher for ble auth.
 * @param[in] random, 16 Byte
 * @param[out] cipher, 1 6Byte
 */
void genie_ais_get_cipher(const uint8_t random[16], uint8_t *cipher);

/**
 * @brief clear the keys of ais service.
 */
void genie_ais_reset(void);

/**
 * @brief init the ais advertising.
 * @param[inout] the phoint of advertising data
 */
void genie_ais_adv_init(uint8_t ad_structure[14]);
#endif

/**
 * @brief set the flag of the mesh advertising.
 */
void genie_tri_tuple_set_silent_adv(void);

#endif/*--_TRI_TUPLE_H_--*/
