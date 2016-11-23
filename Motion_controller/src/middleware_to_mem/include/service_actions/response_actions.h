/**********************************************
File: response_action.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: the actions according to corresponding service id.
Author: Feng.Wu 14-Nov-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_
#define MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_

#include "struct_to_mem/struct_service_response.h"

#define CREATE_SERVICE_ITEM(sid, service_fun) {((int)sid), (fst_response_action::ResponseAction::service_fun)}

typedef struct
{
    int sid;
    int (*function)(const ServiceResponse *res);
}ResponseTable;
//1.Bellow add the sevice id number.
enum ServiceID
{
    NEGATIVE_RESPONSE_SID = 0,
    JTAC_CMD_SID = 0x01,
    READ_VERSION_SID = 0x10,
    HEARTBEAT_INFO_SID = 0x11,
    READ_SERVO_DATA_BY_ADDR = 0x14,
    READ_DATA_BY_ID = 0x1D,
    WRITE_SERVO_DATA_BY_ADDR = 0x24,
    WRTIE_DATA_BY_ID = 0x2D,
};
//2.SERVICE_TABLE_LEN plus one if add new item.
#define SERVICE_TABLE_LEN 3

namespace fst_response_action
{

class ResponseAction
{
public:
    ResponseAction();
    ~ResponseAction();
    int searchServiceTableIndex(const ServiceResponse *res);
    static int abnormalInfo(const ServiceResponse *res);
    static int versionInfo(const ServiceResponse *res);
    static int heartbeatResponse(const ServiceResponse *res);
    static const ResponseTable response_table[SERVICE_TABLE_LEN];  
};

}//namespace fst_response_action

#endif  //MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_
