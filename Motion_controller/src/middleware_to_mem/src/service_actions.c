/**********************************************
File: service_action.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: the actions according to corresponding service id.
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_C_
#define MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_C_

#include "service_actions/service_actions.h"
#include  "stdio.h"

//------------------------------------------------------------
// Function:  searchServiceTableIndex
// Summary: Search the index of the service function table.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  result -> the seq of structure in the service function table.
//          -1 -> no match id.
//------------------------------------------------------------
int  searchServiceTableIndex(const ServiceResponse *res)
{ 
    int i;
    int result = -1; 
    int sid = (*res).res_id;
    for (i = 0; i < SERVICE_TABLE_LEN; ++i)
    {
        if (sid == service_table[i].sid)
        {
            result = i;
            break;
        }
    }
//    if (result == -1) printf("\nInfo in SearchServiceTableIndex(): no match id!\n");
    return result;
}

//------------------------------------------------------------
// Function:  interruptInfo
// Summary: print the info if core1 cannot access interrupt.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int interruptInfo(const ServiceResponse *res)
{
    printf("CANNOT ACCESS INTERRUPT\n");
    return 1;
}

//------------------------------------------------------------
// Function:  versionInfo
// Summary: print the version info of CORE1
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int versionInfo(const ServiceResponse *res)
{
    printf("Info from CORE1:\n%s\n", (*res).res_buff);
    return 1;
}

//------------------------------------------------------------
// Function:  heartbeatResponse
// Summary: print the info from core1.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int heartbeatResponse(const ServiceResponse *res)
{   
    if ((*res).res_buff[0] == 1)
    {
        printf("Heartbeat:\n%s\n", &((*res).res_buff[1]));
    }
    return 1;
}

//------------------------------------------------------------
// Function:  resetCore
// Summary: do nothing.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int resetCore(const ServiceResponse *res)
{
    return 1;
}


#endif  //MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_C_
