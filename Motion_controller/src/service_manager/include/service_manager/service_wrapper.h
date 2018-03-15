/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_wrapper.h
Author:     Feng.Wu 
Create:     19-Dec-2016
Modify:     20-Dec-2016
Summary:    lib to send service to BARE CORE
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_SERVICE_WRAPPER_H_
#define MIDDLEWARE_TO_MEM_SERVICE_WRAPPER_H_

#include "error_code/error_code.h"
#include "comm_interface/comm_interface.h"
#include "service_actions/response_actions.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

namespace fst_service_wrapper
{

class ServiceWrapper
{
public:
    //------------------------------------------------------------
    // Function:  ServiceWrapper
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ServiceWrapper();

    //------------------------------------------------------------
    // Function:  ~ServiceWrapper
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~ServiceWrapper();

    //------------------------------------------------------------
    // Function:  init
    // Summary: create a specific channel to communicate with monitor
    // In:      None
    // Out:     None
    // Return:  0 -> success.
    //          CREATE_CHANNEL_FAIL -> fail to create a channel.
    //------------------------------------------------------------
    ERROR_CODE_TYPE init(void);

    //------------------------------------------------------------
    // Function:  sendHeartbeatRequest
    // Summary: send a request for the heartbeat of monitor
    // In:      None
    // Out:     None
    // Return:  0 -> success.
    //          SEND_MSG_FAIL -> fail.
    //------------------------------------------------------------
    ERROR_CODE_TYPE sendHeartbeatRequest(ServiceResponse &resp);

    //------------------------------------------------------------
    // Function:  sendResetRequest
    // Summary: send a reset request to BARE CORE.
    // In:      None
    // Out:     None
    // Return:  0 -> success.
    //          SEND_MSG_FAIL -> fail.
    //------------------------------------------------------------
    ERROR_CODE_TYPE sendResetRequest(void);

    //------------------------------------------------------------
    // Function:  sendStopRequest
    // Summary: send a stop request.
    // In:      None
    // Out:     None
    // Return:  0 -> success.
    //          SEND_MSG_FAIL -> fail.
    //------------------------------------------------------------
    ERROR_CODE_TYPE sendStopRequest(void);

    // The attempt number to send a request or receive a response.
    static const int ATTEMPTS = 100;

private:
    // Used to communicate with monitor.
    fst_comm_interface::CommInterface comm_;
       
};
} //namespace fst_service_wrapper

#endif //MIDDLEWARE_TO_MEM_SERVICE_WRAPPER_H_
