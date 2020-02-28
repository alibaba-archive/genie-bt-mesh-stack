/********************************************************************************************************
 * @file     spi_i.h 
 *
 * @brief    This is the header file for TLSR8258
 *
 * @author	 xxxx@telink-semi.com;
 * @date     May 8, 2018
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#include "register.h"

// use static inline, because, spi flash code must reside in memory..
// these code may be embedd in flash code


#if  0

 /**
  * @brief     This function servers to set the spi wait.
  * @param[in] none
  * @return    none
  */
 static inline void mspi_wait(void){
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY)
		;
}

/**
 * @brief     This function servers to set the spi high level.
 * @param[in] none
 * @return    none
 */
 static inline void mspi_high(void){
	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
}

/**
 * @brief     This function servers to set the spi low level.
 * @param[in] none
 * @return    none
 */
 static inline void mspi_low(void){
	reg_master_spi_ctrl = 0;
}

/**
 * @brief     This function servers to gets the spi data.
 * @param[in] none.
 * @return    the spi data.
 */
 static inline unsigned char mspi_get(void){
	return reg_master_spi_data;
}

/**
  * @brief     This function servers to write the spi.
  * @param[in] c - the char need to be write.
  * @return    none
  */
 static inline void mspi_write(unsigned char c){
	reg_master_spi_data = c;
}

 /**
  * @brief     This function servers to control the write.
  * @param[in] c - need to be write.
  * @return    none
  */
 static inline void mspi_ctrl_write(unsigned char c){
	reg_master_spi_ctrl = c;
}

 /**
  * @brief     This function servers to spi read.
  * @param[in] none.
  * @return    read reault.
  */
 static inline unsigned char mspi_read(void){
	mspi_write(0);		// dummy, issue clock
	mspi_wait();
	return mspi_get();
}



#else

 /**
  * @brief     This function servers to set the spi wait.
  * @param[in] none
  * @return    none
  */
_attribute_ram_code_ static inline void mspi_wait(void){
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY)
		;
}

/**
 * @brief     This function servers to set the spi high level.
 * @param[in] none
 * @return    none
 */
_attribute_ram_code_ static inline void mspi_high(void){
	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
}

/**
 * @brief     This function servers to set the spi low level.
 * @param[in] none
 * @return    none
 */
_attribute_ram_code_ static inline void mspi_low(void){
	reg_master_spi_ctrl = 0;
}

/**
 * @brief     This function servers to gets the spi data.
 * @param[in] none.
 * @return    the spi data.
 */
_attribute_ram_code_ static inline unsigned char mspi_get(void){
	return reg_master_spi_data;
}

/**
 * @brief     This function servers to write the spi.
 * @param[in] c - the char need to be write.
 * @return    none
 */
_attribute_ram_code_ static inline void mspi_write(unsigned char c){
	reg_master_spi_data = c;
}

/**
 * @brief     This function servers to control the write.
 * @param[in] c - need to be write.
 * @return    none
 */
_attribute_ram_code_ static inline void mspi_ctrl_write(unsigned char c){
	reg_master_spi_ctrl = c;
}

/**
 * @brief     This function servers to spi read.
 * @param[in] none.
 * @return    read reault.
 */
_attribute_ram_code_ static inline unsigned char mspi_read(void){
	mspi_write(0);		// dummy, issue clock
	mspi_wait();
	return mspi_get();
}

#endif




