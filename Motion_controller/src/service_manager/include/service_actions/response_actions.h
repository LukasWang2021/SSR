/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       response_action.h
Author:     Feng.Wu
Create:     14-Nov-2016
Modify:     22-Apr-2019
Summary:    The actions according to corresponding service id.
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_
#define MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_

#include "struct_to_mem/struct_service_response.h"

//Bellow add the sevice id number.
enum ServiceID
{
    JTAC_CMD_SID = 0x01,
    READ_VERSION_SID = 0x10,
    HEARTBEAT_INFO_SID = 0x11,
    READ_SERVO_DATA_BY_ADDR = 0x14,
    READ_DATA_BY_ID = 0x1D,
    WRITE_SERVO_DATA_BY_ADDR = 0x24,
    WRTIE_DATA_BY_ID = 0x2D,
    READ_DTC_SID = 0x30,
    LOG_CONTROL_SID = 0x40,
    LOG_GETLIST_SID = 0x41,
    MONITOR_HEARTBEAT_SID = 0xA1,

};

#endif  
