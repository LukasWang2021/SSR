/**********************************************
File: response_action.cpp
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: the actions according to corresponding service id.
Author: Feng.Wu 14-Nov-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_CPP_
#define MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_CPP_

#include "service_actions/response_actions.h"
#include "stdio.h"

namespace fst_response_action
{

//3.Bellow add the new service item.
const ResponseTable ResponseAction::response_table[SERVICE_TABLE_LEN] = 
{
    CREATE_SERVICE_ITEM(NEGATIVE_RESPONSE_SID, abnormalInfo),
    CREATE_SERVICE_ITEM(READ_VERSION_SID, versionInfo),
    CREATE_SERVICE_ITEM(HEARTBEAT_INFO_SID, heartbeatResponse),    
};

ResponseAction::ResponseAction()
{
}

//------------------------------------------------------------
// Function:  searchServiceTableIndex
// Summary: Search the index of the service function table.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  result -> the seq of structure in the service function table.
//          -1 -> no match id.
//------------------------------------------------------------
int ResponseAction::searchServiceTableIndex(const ServiceResponse *res)
{ 
    int i;
    int result = -1; 
    int sid = (*res).res_id;
    for (i = 0; i < SERVICE_TABLE_LEN; ++i)
    {
        if (sid == response_table[i].sid)
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
int ResponseAction::abnormalInfo(const ServiceResponse *res)
{
    printf("Abnormal response code from CORE1: %s\n", (*res).res_buff);
    return true;
}

//------------------------------------------------------------
// Function:  versionInfo
// Summary: print the version info of CORE1
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int ResponseAction::versionInfo(const ServiceResponse *res)
{
    printf("Info from CORE1:\n%s\n", (*res).res_buff);
    return true;
}

//------------------------------------------------------------
// Function:  heartbeatResponse
// Summary: print the info from core1.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int ResponseAction::heartbeatResponse(const ServiceResponse *res)
{  
    int i;
    for (i = 0; i < 4; ++i)
    {
        if ((*res).res_buff[i] != 0)
        {
            printf("Heartbeat Error Code: %d \n", (*res).res_buff[0]);
        }
    }
    if ((*res).res_buff[4] == 1)
    {
        printf("Heartbeat Info:\n%s\n", &((*res).res_buff[5]));
    }
    return true;
}

ResponseAction::~ResponseAction()
{
}

}

#endif  //MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_C_
