/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       response_action.h
Author:     Feng.Wu / Yan.He
Create:     14-Nov-2016
Modify:     02-Dec-2016
Summary:    The actions according to corresponding service id.
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_CPP_
#define MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_CPP_

#include "service_actions/response_actions.h"
#include "stdio.h"
#include <iostream>
#include "error_code.h"

namespace fst_response_action
{

//3.Bellow add the new service item.
const ResponseTable ResponseAction::response_local_table[RESPONSE_LOCAL_TABLE_LEN] = 
{
    CREATE_SERVICE_ITEM(READ_VERSION_SID, versionInfo),
    CREATE_SERVICE_ITEM(HEARTBEAT_INFO_SID, heartbeatResponse), 
    CREATE_SERVICE_ITEM(READ_DTC_SID, dtcResponse),
};

const int ResponseAction::service_table[SERVICE_TABLE_LEN] = 
{
    JTAC_CMD_SID,
    READ_VERSION_SID,
    HEARTBEAT_INFO_SID,
    READ_SERVO_DATA_BY_ADDR,
    READ_DATA_BY_ID,
    WRITE_SERVO_DATA_BY_ADDR,
    WRTIE_DATA_BY_ID,
    READ_DTC_SID,
    LOG_CONTROL_SID,
    LOG_GETLIST_SID,
    MONITOR_HEARTBEAT_SID,
};

int ResponseAction::dtc_flag = 0;
fst_log::Logger* ResponseAction::log_ptr_ = NULL;

//------------------------------------------------------------
// Function:  ResponseAction
// Summary: The constructor of class
// In:      None.
// Out:     None.
// Return:  None. 
//------------------------------------------------------------
ResponseAction::ResponseAction(fst_log::Logger *logger)
{
    log_ptr_ = logger;
}

//------------------------------------------------------------
// Function:  ~ResponseAction
// Summary: The destructor of class
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
ResponseAction::~ResponseAction()
{
}

//------------------------------------------------------------
// Function:  searchResponseLocalTableIndex
// Summary: Search the index of the service function table.
// In:      sid -> the id of the service response.
// Out:     None.
// Return:  result -> the seq of structure in the service function table.
//          -1 -> no match id.
//------------------------------------------------------------
int ResponseAction::searchResponseLocalTableIndex(int sid)
{ 
    int i, result = -1;
    for (i = 0; i < RESPONSE_LOCAL_TABLE_LEN; ++i)
    {
        if (sid == response_local_table[i].sid)
        {
            result = i;
            break;
        }
    }
//    if (result == -1) printf("\nInfo in SearchServiceTableIndex(): no match id!\n");
    return result;
}

//------------------------------------------------------------
// Function:  searchServiceTableIndex
// Summary: Search the index of the service id table.
// In:      id -> the id of the service response.
// Out:     None.
// Return:  result -> the seq of structure in the service function table.
//          -1 -> no match id.
//------------------------------------------------------------
int ResponseAction::searchServiceTableIndex(int sid)
{ 
    int i, result = -1;
    for (i = 0; i < SERVICE_TABLE_LEN; ++i)
    {
        if (sid == service_table[i])
        {
            result = i;
            break;
        }
    }
//    if (result == -1) printf("\nInfo in SearchServiceTableIndex(): no match id!\n");
    return result;
}

//------------------------------------------------------------
// Function:  getDtcFlag
// Summary: Return the value of dtc_flag.
// In:      None.
// Out:     None.
// Return:  dtc_flag -> the dtc status.
//------------------------------------------------------------
int ResponseAction::getDtcFlag(void)
{
    return dtc_flag;
}

//------------------------------------------------------------
// Function:  clearDtcFlag
// Summary: Set the value of dtc_flag to be zero.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void ResponseAction::clearDtcFlag(void)
{
    dtc_flag = 0;
}

//------------------------------------------------------------
// Function:  versionInfo
// Summary: Print the version info of BARE CORE.
// In:      *resp -> the address of the service response.
// Out:     None
// Return:  1
//------------------------------------------------------------
int ResponseAction::versionInfo(const ServiceResponse *resp)
{
    FST_INFO("CORE1 Version:%s", (*resp).res_buff);
    return true;
}

//------------------------------------------------------------
// Function:  heartbeatResponse
// Summary: Print the heartbeat info from BARE CORE.
// In:      *resp -> the address of the service response.
// Out:     None
// Return:  1
//------------------------------------------------------------
int ResponseAction::heartbeatResponse(const ServiceResponse *resp)
{  
    if((*resp).res_buff[0] !=0)
    {
        FST_INFO("New Diagnostice infomation!");
        dtc_flag = 1;
    }
    if ((*resp).res_buff[1] == 1)
    {
        FST_INFO(&((*resp).res_buff[2]));
        //std::cout << &((*resp).res_buff[2])<<std::endl;
    }
    
    return true;
}

//------------------------------------------------------------
// Function:  dtcResponse
// Summary: Print the error code from BARE CORE.
// In:      *resp -> the address of the service response.
// Out:     None
// Return:  1.
//------------------------------------------------------------
int ResponseAction::dtcResponse(const ServiceResponse *resp)
{
    unsigned int size = *(int*)(&(*resp).res_buff[4]);
    const char *buf;
    if (size > 0)
    {
        FST_ERROR("%d Diagnostice infomation(s)!",size);
    }
    for (unsigned int i = 0; i < size; ++i)
    {
        buf = &(*resp).res_buff[8 + i*8];
        FST_ERROR(": %02X%02X%02X%02X%02X%02X%02X%02X ", (unsigned char)buf[7], (unsigned char)buf[6], (unsigned char)buf[5], (unsigned char)buf[4], (unsigned char)buf[3], (unsigned char)buf[2], (unsigned char)buf[1], (unsigned char)buf[0]);
    }
    return true;
}


}

#endif  //MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_CPP_
