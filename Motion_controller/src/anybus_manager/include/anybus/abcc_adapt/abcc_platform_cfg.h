/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Starter Kit version 3.05.02 (2018-08-30)                              **
**                                                                            **
** Delivered with:                                                            **
**    ABP            7.59.01 (2018-05-17)                                     **
**    ABCC Driver    5.05.02 (2018-08-30)                                     **
**                                                                            */
/*******************************************************************************
********************************************************************************
** COPYRIGHT NOTIFICATION (c) 2015 HMS Industrial Networks AB                 **
**                                                                            **
** This code is the property of HMS Industrial Networks AB.                   **
** The source code may not be reproduced, distributed, or used without        **
** permission. When used together with a product from HMS, permission is      **
** granted to modify, reproduce and distribute the code in binary form        **
** without any restrictions.                                                  **
**                                                                            **
** THE CODE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. HMS DOES NOT    **
** WARRANT THAT THE FUNCTIONS OF THE CODE WILL MEET YOUR REQUIREMENTS, OR     **
** THAT THE OPERATION OF THE CODE WILL BE UNINTERRUPTED OR ERROR-FREE, OR     **
** THAT DEFECTS IN IT CAN BE CORRECTED.                                       **
********************************************************************************
********************************************************************************
** Platform specific product configuration file.
**
** A symbol defined in this file will override the default product
** configurations in abcc_identification.h, abcc_obj_cfg.h, and abcc_drv_cfg.h.
** It can be used whenever the target platform needs to override a
** default configuration to realize its implementation.
********************************************************************************
********************************************************************************
** Services:
********************************************************************************
********************************************************************************
*/
#ifndef ABCC_PLATFORM_CFG_H_
#define ABCC_PLATFORM_CFG_H_

/*******************************************************************************
** Constants
********************************************************************************
*/

#define ETN_OBJ_ENABLE                          TRUE
#define ETN_IA_IP_CONFIGURATION_ENABLE          TRUE
#define ETN_OBJ_USE_SET_ATTR_SUCCESS_CALLBACK   TRUE

#define APP_IA_SER_NUM_ENABLE                   TRUE
#define APP_IA_SER_NUM_VALUE                    0x11223344

#define SAFE_OBJ_ENABLE                         TRUE
#define SAFE_IA_SAFETY_ENABLED_ENABLE           TRUE
#define SAFE_IA_SAFETY_ENABLED_VALUE            FALSE
#define SAFE_IA_BAUD_RATE_ENABLE                TRUE
#define SAFE_IA_BAUD_RATE_VALUE                 1020000L

#define SAFE_IA_CYCLE_TIME_ENABLE               TRUE
#define SAFE_IA_CYCLE_TIME_VALUE                2

#endif /* #ifndef ABCC_PLATFORM_CFG_H_ */
