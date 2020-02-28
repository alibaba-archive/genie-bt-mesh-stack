/********************************************************************************************************
 * @file     phy.h
 *
 * @brief    for TLSR chips
 *
 * @author	 public@telink-semi.com;
 * @date     Feb. 1, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and proprietary property of Telink
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/

#ifndef PHY_H_
#define PHY_H_








typedef enum {
	BLE_PHY_MASK_1M 	= BIT(0),
	BLE_PHY_MASK_2M		= BIT(1),
	BLE_PHY_MASK_CODED 	= BIT(2),
} le_phy_mask_t;











/********************* Macro & Enumeration variables for Stack, user can not use!!!!  **********************/
#define BLE_LL_EXT_ADV_ADVA_BIT         						BIT(0)
#define BLE_LL_EXT_ADV_TARGETA_BIT      						BIT(1)
#define BLE_LL_EXT_ADV_RFU_BIT          						BIT(2)
#define BLE_LL_EXT_ADV_DATA_INFO_BIT    						BIT(3)
#define BLE_LL_EXT_ADV_AUX_PTR_BIT      						BIT(4)
#define BLE_LL_EXT_ADV_SYNC_INFO_BIT    						BIT(5)
#define BLE_LL_EXT_ADV_TX_POWER_BIT     						BIT(6)


#define BLE_LL_EXT_ADV_MODE_NON_CONN_NON_SCAN    				(0x00)
#define BLE_LL_EXT_ADV_MODE_CONN        						(0x01)		//connectable, none_scannable
#define BLE_LL_EXT_ADV_MODE_SCAN        						(0x02)      //scannable,   none_connectable


#define PHY_USED_AUXPTR_LE_1M									0
#define PHY_USED_AUXPTR_LE_2M									1
#define PHY_USED_AUXPTR_LE_CODED								2


extern unsigned char	cur_llPhy;






/******************************* Macro & Enumeration variables for User ************************************/

typedef enum {
	BLE_PHY_1M 			= 0x01,
	BLE_PHY_2M 			= 0x02,
	BLE_PHY_CODED	 	= 0x03,
} le_phy_type_t;







/************************************ User Interface  ******************************************************/








/*********************************** Stack Interface, user can not use!!! **********************************/
void rf_ble_phy_switch(unsigned char phy);













#endif /* PHY_H_ */
