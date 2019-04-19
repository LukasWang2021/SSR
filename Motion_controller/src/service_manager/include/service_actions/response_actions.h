/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       response_action.h
Author:     Feng.Wu / Yan.He
Create:     14-Nov-2016
Modify:     02-Dec-2016
Summary:    The actions according to corresponding service id.
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_
#define MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_

#include "struct_to_mem/struct_service_response.h"
#include "common_log.h"

#define CREATE_SERVICE_ITEM(sid, service_fun) {((int)sid), (fst_response_action::ResponseAction::service_fun)}

typedef struct
{
    int sid;
    int (*function)(const ServiceResponse *resp);
}ResponseTable;
//1.Bellow add the sevice id number.
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
//2.SERVICE_TABLE_LEN plus one if add new item.
#define RESPONSE_LOCAL_TABLE_LEN 3
#define SERVICE_TABLE_LEN 11
namespace fst_response_action
{

class ResponseAction
{
public:
    //------------------------------------------------------------
    // Function:  ResponseAction
    // Summary: The constructor of class
    // In:      None.
    // Out:     None.
    // Return:  None. 
    //------------------------------------------------------------
    ResponseAction(fst_log::Logger *logger);

    //------------------------------------------------------------
    // Function:  ~ResponseAction
    // Summary: The destructor of class
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    ~ResponseAction();

    //------------------------------------------------------------
    // Function:  searchResponseLocalTableIndex
    // Summary: Search the index of the service function table.
    // In:      *resp -> the address of the service response.
    // Out:     None.
    // Return:  result -> the seq of structure in the service function table.
    //          -1 -> no match id.
    //------------------------------------------------------------
    int searchResponseLocalTableIndex(int sid);

    //------------------------------------------------------------
    // Function:  searchServiceTableIndex
    // Summary: Search the index of the service id table.
    // In:      id -> the id of the service response.
    // Out:     None.
    // Return:  result -> the seq of structure in the service function table.
    //          -1 -> no match id.
    //------------------------------------------------------------
    int searchServiceTableIndex(int sid);

    //------------------------------------------------------------
    // Function:  getDtcFlag
    // Summary: Return the value of dtc_flag.
    // In:      None.
    // Out:     None.
    // Return:  dtc_flag -> the dtc status.
    //------------------------------------------------------------
    int getDtcFlag(void);

    //------------------------------------------------------------
    // Function:  clearDtcFlag
    // Summary: Set the value of dtc_flag to be zero.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void clearDtcFlag(void);

    //------------------------------------------------------------
    // Function:  versionInfo
    // Summary: Print the version info of BARE CORE.
    // In:      *resp -> the address of the service response.
    // Out:     None
    // Return:  1
    //------------------------------------------------------------
    static int versionInfo(const ServiceResponse *resp);

    //------------------------------------------------------------
    // Function:  heartbeatResponse
    // Summary: Print the heartbeat info from BARE CORE.
    // In:      *resp -> the address of the service response.
    // Out:     None
    // Return:  1
    //------------------------------------------------------------
    static int heartbeatResponse(const ServiceResponse *resp);

    //------------------------------------------------------------
    // Function:  dtcResponse
    // Summary: Print the error code from BARE CORE.
    // In:      *resp -> the address of the service response.
    // Out:     None
    // Return:  1.
    //------------------------------------------------------------
    static int dtcResponse(const ServiceResponse *resp);

    // The responses that monitor process deal with locally.
    static const ResponseTable response_local_table[RESPONSE_LOCAL_TABLE_LEN];

    // All the service ID table.
    static const int service_table[SERVICE_TABLE_LEN];

private:
    
    static fst_log::Logger* log_ptr_;
    // To mark whether there is error code in BARE CORE.
    static int dtc_flag;
};

}//namespace fst_response_action

#endif  //MIDDLEWARE_TO_MEM_RESPONSE_ACTIONS_H_
