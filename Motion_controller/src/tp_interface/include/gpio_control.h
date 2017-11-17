/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       gpio_control.h
Author:     Feng.Wu 
Create:     13-Sep-2017
Modify:     13-Sep-2017
Summary:    dealing with gpio
**********************************************/
#ifndef TP_INTERFACE_GPIO_CONTROL_H_
#define TP_INTREFACE_GPIO_CONTROL_H_

#define ERR_GPIO_CONTROL_INIT (unsigned long long int)0x00110002008F044D   /*fail to initialize the GPIO control.*/

#ifdef __cplusplus
extern "C" {
#endif

//  -----------------------------------------------------------------------
//  Function:		openGPIO
//  Description:	initialize the GPIO control.
//  In:             None.
//  Out:            None.
//  Return:			0: no error
//					ERR_GPIO_CONTROL_INIT: fail to initialize the GPIO contorl.
//  -----------------------------------------------------------------------
unsigned long long int openGPIO(void);

//  -----------------------------------------------------------------------
//  Function:		shutdownPower
//  Description:	send shutdown signal to GPIO when shutting down.
//  In:             None.
//  Out:            None.
//  Return:			None.
//  -----------------------------------------------------------------------
void shutdownPower(void);

//  -----------------------------------------------------------------------
//  Function:		revokeShutdown
//  Description:	revoke shutdown signal to GPIO.
//  In:             None.
//  Out:            None.
//  Return:			None.
//  -----------------------------------------------------------------------
void revokeShutdown(void); 

//  -----------------------------------------------------------------------
//  Function:		closeGPIO
//  Description:	close mmaping.
//  In:             None.
//  Out:            None.
//  Return:			None.
//  -----------------------------------------------------------------------
void closeGPIO(void);

#ifdef __cplusplus
}
#endif

#endif
