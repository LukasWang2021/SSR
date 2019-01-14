/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_wrapper.cpp
Author:     Feng.Wu 
Create:     19-Dec-2016
Modify:     20-Dec-2016
Summary:    lib to send service to BARE CORE
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_SERVICE_WRAPPER_CPP_
#define MIDDLEWARE_TO_MEM_SERVICE_WRAPPER_CPP_

#include "service_manager/service_wrapper.h"
#include <string.h>
#include <unistd.h>
#include <iostream>
#include "error_code.h"

namespace fst_service_wrapper
{

//------------------------------------------------------------
// Function:  ServiceWrapper
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
ServiceWrapper::ServiceWrapper()
{
}

//------------------------------------------------------------
// Function:  ~ServiceWrapper
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
ServiceWrapper::~ServiceWrapper()
{
}

//------------------------------------------------------------
// Function:  init
// Summary: create a specific channel to communicate with monitor
// In:      None
// Out:     None
// Return:  0 -> success.
//          CREATE_CHANNEL_FAIL -> fail to create a channel.
//------------------------------------------------------------
ErrorCode ServiceWrapper::init(void)
{
    ErrorCode result = comm_.createChannel(COMM_REQ, COMM_IPC, "mcs");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error in ServiceWrapper::init(): fail to create mcs channel."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }
    return SUCCESS;
}

//------------------------------------------------------------
// Function:  sendHeartbeatRequest
// Summary: send a request for the heartbeat of monitor
// In:      None
// Out:     None
// Return:  0 -> success.
//          SEND_MSG_FAIL -> fail.
//          RECV_MSG_FAIL
//------------------------------------------------------------
ErrorCode ServiceWrapper::sendHeartbeatRequest(ServiceResponse &resp)
{
    // attempt to send heartbeat request
    int count = 0;
    ErrorCode result = SEND_MSG_FAIL;
    ServiceRequest req = {MONITOR_HEARTBEAT_SID, ""};
    while (result != SUCCESS)
    {
        result = comm_.send(&req, sizeof(req), COMM_DONTWAIT);
        ++count;
        if (count > ATTEMPTS)
        {
            printf("write heartbeat out\n");
            return result;
        }
    }
    std::cout<<"send heartbeat count = "<<count<<std::endl;

    // attempt to recv heartbeat response.
    count = 0;
    result = RECV_MSG_FAIL;
    while (result != SUCCESS)
    {
        usleep(500);
        result = comm_.recv(&resp, sizeof(resp), COMM_DONTWAIT);
        ++count;
        if (count > ATTEMPTS)
        {
            printf("read heartbeat out\n");
            return result;
        }
    }
    std::cout<<"recv heartbeat count = "<<count<<std::endl;
    return result;
}

//------------------------------------------------------------
// Function:  sendResetRequest
// Summary: send a reset request to BARE CORE.
// In:      None
// Out:     None
// Return:  0 -> success.
//          SEND_MSG_FAIL -> fail to send a msg.
//          RECV_MSG_FAIL -> fail to recv a msg within limited time.
//------------------------------------------------------------
ErrorCode ServiceWrapper::sendResetRequest(void)
{
    // attempt to send reset request
    int count = 0;
    ErrorCode result = SEND_MSG_FAIL;
    ServiceRequest req = {JTAC_CMD_SID, ""};
    while (result != SUCCESS)
    {
        result = comm_.send(&req, sizeof(req), COMM_DONTWAIT);
        ++count;
        if (count > ATTEMPTS)
            return result;
    }

    // attempt to recv reset response
    count = 0;
    result = RECV_MSG_FAIL;
    ServiceResponse resp;
    while (result != SUCCESS)
    {
        usleep(500);
        result = comm_.recv(&resp, sizeof(resp), COMM_DONTWAIT);
        ++count;
        if (count > ATTEMPTS)
            return result;
    }
//    std::cout<<"recv reset count = "<<count<<std::endl;
    return SUCCESS;
}
   

//------------------------------------------------------------
// Function:  sendStopRequest
// Summary: send a stop request.
// In:      None
// Out:     None
// Return:  0 -> success.
//          SEND_MSG_FAIL -> fail.
//------------------------------------------------------------
ErrorCode ServiceWrapper::sendStopRequest(void)
{
    // attempt to send stop request.
    int count = 0;
    ErrorCode result = SEND_MSG_FAIL;
    ServiceRequest req = {JTAC_CMD_SID, ""};
    int stop = 1;
    memcpy(&req.req_buff[0], &stop, sizeof(stop));
    while (result != SUCCESS)
    {
        result = comm_.send(&req, sizeof(req), COMM_DONTWAIT);
        ++count;
        if (count > ATTEMPTS)
            return result;
    }

    // attempt to recv reset response
    count = 0;
    result = RECV_MSG_FAIL;
    ServiceResponse resp;
    while (result != SUCCESS)
    {
        usleep(500);
        result = comm_.recv(&resp, sizeof(resp), COMM_DONTWAIT);
        ++count;
        if (count > ATTEMPTS)
            return result;
    }
//    std::cout<<"recv stop count = "<<count<<std::endl;
    return SUCCESS;
}

} //namespace fst_service_wrapper

#endif //MIDDLEWARE_TO_MEM_SERVICE_WRAPPER_CPP_
