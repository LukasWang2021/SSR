/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       gpio_control.cpp
Author:     Feng.Wu 
Create:     13-Sep-2017
Modify:     13-Sep-2017
Summary:    dealing with gpio
**********************************************/
#ifndef TP_INTERFACE_GPIO_CONTROL_CPP_
#define TP_INTERFACE_GPIO_CONTROL_CPP_

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "gpio_control.h"

#define SHUTDOWN_LEN    0x40         
#define SHUTDOWN_BASE   0xFF300000     
#define SHUTDOWN_BASE_OFFSET    0x20    

static int fd;
static char *shutdown_ptr;        
static char *shutdown_write_ptr;   

//  -----------------------------------------------------------------------
//  Function:		openGPIO
//  Description:	initialize the GPIO control.
//  In:             None.
//  Out:            None.
//  Return:			0: no error
//					ERR_GPIO_CONTROL_INIT: fail to initialize the GPIO contorl.
//  -----------------------------------------------------------------------
unsigned long long int openGPIO() {
	unsigned long long int ret = 0;
    fd = open("/dev/mem", O_RDWR);
	if (fd == -1) {
		return ERR_GPIO_CONTROL_INIT;
	}

    shutdown_ptr = (char *)mmap(NULL, SHUTDOWN_LEN, PROT_WRITE|PROT_READ, MAP_SHARED, fd, SHUTDOWN_BASE); 
    if (shutdown_ptr == MAP_FAILED){
        close(fd);
        return ERR_GPIO_CONTROL_INIT; 
    }
    shutdown_write_ptr = (char *)(shutdown_ptr + SHUTDOWN_BASE_OFFSET);  

	return ret;
}

//  -----------------------------------------------------------------------
//  Function:		shutdownPower
//  Description:	send shutdown signal to GPIO when shutting down.
//  In:             None.
//  Out:            None.
//  Return:			None.
//  -----------------------------------------------------------------------
void shutdownPower(void)            
{
    *shutdown_write_ptr = 0x01;    
}

//  -----------------------------------------------------------------------
//  Function:		revokeShutdown
//  Description:	revoke shutdown signal to GPIO.
//  In:             None.
//  Out:            None.
//  Return:			None.
//  -----------------------------------------------------------------------
void revokeShutdown(void)         
{
    *shutdown_write_ptr = 0x00; 
}

//  -----------------------------------------------------------------------
//  Function:		closeGPIO
//  Description:	close mmaping.
//  In:             None.
//  Out:            None.
//  Return:			None.
//  -----------------------------------------------------------------------
void closeGPIO() {
    munmap(shutdown_ptr, SHUTDOWN_LEN); 
	close(fd);
}

#endif
