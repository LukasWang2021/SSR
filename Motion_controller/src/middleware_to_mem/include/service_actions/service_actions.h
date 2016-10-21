/**********************************************
File: service_action.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: the actions according to corresponding service id.
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_H_
#define MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_H_

#include "struct_to_mem/struct_service_response.h"

#define CREATE_SERVICE_ITEM(sid, service_fun) {((int)sid), (service_fun)}

typedef struct
{
    int sid;
    int (*function)(const ServiceResponse *res);

}ServiceTable;

//1.Bellow add the new service item, and modify SERVICE_TABLE_LEN.
#define SERVICE_TABLE_LEN 4
static const ServiceTable service_table[SERVICE_TABLE_LEN] = 
{
    CREATE_SERVICE_ITEM(0, interruptInfo),
    CREATE_SERVICE_ITEM(0x10, versionInfo),
    CREATE_SERVICE_ITEM(0x11, heartbeatResponse),    
    CREATE_SERVICE_ITEM(0x01, resetCore),

};

//2.Bellow add the sevice id number.
enum ServiceID
{
    INTERRUPT_SID = 0,
    JTAC_RESET_SID = 0x01,
    READ_VERSION_SID = 0x10,
    HEARTBEAT_INFO_SID = 0x11,
    READ_SERVO_DATA_BY_ADDR = 0x14,
    READ_DATA_BY_ID = 0x1D,
    WRITE_SERVO_DATA_BY_ADDR = 0x24,
    WRTIE_DATA_BY_ID = 0x2D,
};

#ifdef __cplusplus
	extern "C"{
#endif

//------------------------------------------------------------
// Function:  searchServiceTableIndex
// Summary: Search the index of the service function table.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  result -> the seq of structure in the service function table.
//          -1 -> no match id.
//------------------------------------------------------------
int searchServiceTableIndex(const ServiceResponse *res);

//------------------------------------------------------------
// Function:  interruptInfo
// Summary: print the info if core1 cannot access interrupt.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int interruptInfo(const ServiceResponse *res);

//------------------------------------------------------------
// Function:  versionInfo
// Summary: print the version info of CORE1
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int versionInfo(const ServiceResponse *res);

//------------------------------------------------------------
// Function:  heartbeatResponse
// Summary: print the info from core1.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int heartbeatResponse(const ServiceResponse *res);

//------------------------------------------------------------
// Function:  resetCore
// Summary: do nothing.
// In:      *res -> the address of the service response variable.
// Out:     None
// Return:  1
//------------------------------------------------------------
int resetCore(const ServiceResponse *res);

#ifdef __cplusplus
}
#endif


#endif  //MIDDLEWARE_TO_MEM_SERVICE_ACTIONS_H_
