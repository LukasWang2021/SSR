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
** COPYRIGHT NOTIFICATION (c) 2014 HMS Industrial Networks AB                 **
**                                                                            **
** This program is the property of HMS Industrial Networks AB.                **
** It may not be reproduced, distributed, or used without permission          **
** of an authorized company official.                                         **
********************************************************************************
********************************************************************************
** file_description
********************************************************************************
********************************************************************************
*/

#include "abcc_drv_cfg.h"
#include "abcc_sys_adapt.h"
#include "abcc_sys_adapt_par.h"
#include "abcc.h"


/*******************************************************************************
** Public Services
********************************************************************************
*/

void ABCC_SYS_HWReset( void )
{
    printf("Reset Start\n");
    *(volatile UINT8*)(pMemSys) &= 0x00;
}

void ABCC_SYS_HWReleaseReset( void )
{
    printf("Start Release\n");
    *(volatile UINT8*)(pMemSys) |= 0x01;
}

BOOL ABCC_SYS_HwInit( void )
{
   ABCC_SYS_HWReset();

   return TRUE;
}

BOOL ABCC_SYS_Init( void )
{
   return TRUE;
}

void ABCC_SYS_Close( void )
{
}


