/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_manager_main.cpp
Author:     Feng.Wu 
Create:     07-Nov-2016
Modify:     22-Apr-2016
Summary:    dealing with service
**********************************************/
#ifndef SERVICE_MANAGER_CPP_
#define SERVICE_MANAGER_CPP_

#include "service_manager.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>

using namespace fst_service_manager;
using namespace fst_mc;

//------------------------------------------------------------
// Function:  addBareCoreRequest
// Summary: Setup the heartbeat request to BARE CORE after a number of loop. 
//          setup the stop request to BARE CORE if heartbeat_mcs missing.
// In:      None
// Out:     None
// Return:  true -> a request to BARE CORE is setup.
//          false -> didn't setup request.
//------------------------------------------------------------
bool ServiceManager::addBareCoreHeartbeatRequest(void)
{   
    // Caculate the loop number, setup the heartbeat request to BARE CORE.
    if (loop_count_core_ >= heartbeat_with_barecore_count_)
    {
        request_fifo_.push_back(heartbeat_core_req_);
        loop_count_core_ = 0;
        return true;
    }
    // Check whether there is error code in BARE CORE.
    return dtcSurvey();
}

//------------------------------------------------------------
// Function:  dtcSurvey
// Summary: Create diagnostic request if there are errors in BARE CORE. 
// In:      None
// Out:     None
// Return:  true -> error codes are available.
//          false -> no error codes.
//------------------------------------------------------------
bool ServiceManager::dtcSurvey(void)
{
    if (dtc_flag_ == false)
    {
        return false;
    }
    else if (dtc_flag_ == true)
    {
        ServiceRequest dtc_req = {READ_DTC_SID,""};
        request_fifo_.push_back(dtc_req);
        dtc_flag_ = false; 
    }
    return true;
}


//------------------------------------------------------------
// Function:  addRequest
// Summary: Mainly check if the request id is defined. 
// In:      None
// Out:     None
// Return:  true -> the request id is available.
//          false -> invailid request id.
//------------------------------------------------------------
bool ServiceManager::addRequest(ServiceRequest req)
{
    // Check whether it is heartbeat request from the other process.
    if (req.req_id == MONITOR_HEARTBEAT_SID)
    {
        // Fill the heartbeat context with error_fifo_.
        fillLocalHeartbeat();
        response_fifo_.push_back(heartbeat_local_resp_);
        return true;
    }

    request_fifo_.push_back(req);
    return true;
}

//------------------------------------------------------------
// Function:  fillLocalHeartbeat
// Summary: error codes are filled into the heartbeat response.
// In:      None
// Out:     None
// Return:  true -> error codes are filled into the response.
//          false -> No error codes to be filled.
//------------------------------------------------------------
bool ServiceManager::fillLocalHeartbeat(void)
{
    size_t size = error_fifo_.size();
    if (size > 0)
    {
        memcpy(&(heartbeat_local_resp_.res_buff[0]), &size, sizeof(size));
        for (size_t i = 0; i < size; ++i)
        {
            ErrorCode error = error_fifo_[0];
            memcpy(&(heartbeat_local_resp_.res_buff[8 + i*8]), &error, sizeof(error));
            deleteFirstElement(&error_fifo_);
            //FST_INFO("local heartbeat:id = 0x%X, %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", 
            //          heartbeat_local_resp_.res_id, 
            //          (unsigned char)heartbeat_local_resp_.res_buff[7+8+ i*8],(unsigned char)heartbeat_local_resp_.res_buff[6+8+ i*8],
            //          (unsigned char)heartbeat_local_resp_.res_buff[5+8+ i*8],(unsigned char)heartbeat_local_resp_.res_buff[4+8+ i*8],
            //          (unsigned char)heartbeat_local_resp_.res_buff[3+8+ i*8],(unsigned char)heartbeat_local_resp_.res_buff[2+8+ i*8],
            //          (unsigned char)heartbeat_local_resp_.res_buff[1+8+ i*8],(unsigned char)heartbeat_local_resp_.res_buff[0+8+ i*8]);
        }
    } else if (size == 0)
    {
        memset(&(heartbeat_local_resp_.res_buff[0]), 0, sizeof(heartbeat_local_resp_.res_buff));
        return false;
    }
    return true;
}


//------------------------------------------------------------
// Function:  sendRequest
// Summary: send the service request to BARE CORE. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool ServiceManager::sendBareCoreRequest(void)
{
    if (request_fifo_.empty()) 
        return false;

    ServiceRequest temp_request = request_fifo_[0];
    bool result = sendRequestToBareCore(temp_request);
    if (result == true)
    {
        running_sid_ = temp_request.req_id;
        deleteFirstElement(&request_fifo_);
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  getResponse
// Summary: get the service response from BARE CORE. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool ServiceManager::getBareCoreResponse(void)
{
    if (!response_fifo_.empty()) 
        return false;
    ServiceResponse response;  

    bool result = recvResponseFromBareCore(response);
    if (result == true && (response.res_id == running_sid_))
    {
        response_fifo_.push_back(response);
        return true;
    }
    else if(result == true && (response.res_id != running_sid_))
    {
        //for debug.
        FST_ERROR("Wrong Service ID(0x%x) from BareCore response(0x%x)", response.res_id, running_sid_);
    }
    return false;
}


//------------------------------------------------------------
// Function:  storeError
// Summary: Push the error code into error_fifo_. 
// In:      None
// Out:     None
// Return:  true -> A error is pushed into error_fifo_.
//          false -> did nothing. 
//------------------------------------------------------------
bool ServiceManager::storeError(ErrorCode error)
{
    if (error == 0 || doesErrorExist(error))
        return false;
    error_fifo_.push_back(error);
    return true;
}

//------------------------------------------------------------
// Function:  doesErrorExist
// Summary: Check whether the error code is stored in error_fifo_. 
// In:      None
// Out:     None
// Return:  true -> the error code exists in fifo.
//          false -> the error code doesn't exist in fifo. 
//------------------------------------------------------------
bool ServiceManager::doesErrorExist(ErrorCode error)
{
    for (std::vector<ErrorCode>::iterator iter = error_fifo_.begin(); iter !=error_fifo_.end(); ++iter)
    {
        if (*iter == error)
            return true;
    }
    return false;
}


//------------------------------------------------------------
// Function:  manageLocalResponse
// Summary: deal with local response. 
// In:      None
// Out:     None
// Return:  true -> the local response is disposed.
//          false -> did nothing. 
//------------------------------------------------------------
bool ServiceManager::manageLocalResponse(void)
{
    ServiceResponse temp_response = response_fifo_[0];
    int id = temp_response.res_id;
    if (READ_VERSION_SID == id)
    {
        FST_INFO("CORE1 Version:%s", (temp_response).res_buff);
    }
    else if (HEARTBEAT_INFO_SID == id)
    {
        if((temp_response).res_buff[0] !=0)
        {
            FST_INFO("New Diagnostice infomation!");
            dtc_flag_ = true;
        }
        if ((temp_response).res_buff[1] == 1)
        {
            FST_INFO(&((temp_response).res_buff[2]));
        }
    }
    else if (READ_DTC_SID == id)
    {
        ErrorCode error_code = 0;
        unsigned int size = *(int*)(&(temp_response).res_buff[4]);
        if (size > 0)
        {
            FST_ERROR("%d Diagnostice infomation(s)!",size);
        }
        for (unsigned int i = 0; i < size; ++i)
        {                     
            memcpy(&error_code, &temp_response.res_buff[8 + i*8], 8);
            storeError(error_code);
            FST_ERROR("error code = %016llX", error_code);
        }
    }

    deleteFirstElement(&response_fifo_);
    return true;
}

//------------------------------------------------------------
// Function:  transmitResponse
// Summary: feedback the response to the other process. 
// In:      comm -> the object used to send/recv msg.
// Out:     None
// Return:  true -> success to send msg.
//          false -> fail to send msg.
//------------------------------------------------------------
bool ServiceManager::transmitResponse(fst_comm_interface::CommInterface &comm)
{
    ServiceResponse temp_response = response_fifo_[0];

    int count = 0;
    ErrorCode send_result;
    do
    {
        send_result = comm.send(&temp_response, sizeof(temp_response), COMM_DONTWAIT);
        ++count;
        if (count >= max_send_resp_count_)
        {
            FST_ERROR("Error transmitResponse():fail to send back response id(0x%x) within limit tries. It was dropped.", temp_response.res_id);
            ErrorCode send_error = SEND_RESP_FAIL;
            storeError(send_error);
            deleteFirstElement(&response_fifo_);
            return false;
        }
    } while (send_result != SUCCESS);
    deleteFirstElement(&response_fifo_);
    return true;
}

//------------------------------------------------------------
// Function:  sendRequestToBareCore
// Summary: send the request to BARE CORE. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool ServiceManager::sendRequestToBareCore(ServiceRequest &req)
{
    return clientSendRequest(handle_core_, &req);
}

//------------------------------------------------------------
// Function:  recvResponseFromBareCore
// Summary: get the response from BARE CORE. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool ServiceManager::recvResponseFromBareCore(ServiceResponse &resp)
{
    return clientGetResponse(handle_core_, &resp);
}

//------------------------------------------------------------
// Function:  deleteFirstElement
// Summary: delete the first element of a fifo. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
template<typename T>
bool ServiceManager::deleteFirstElement(T *fifo)
{
    if (fifo->size() == 0)
    {
        FST_ERROR("Error when delete fifo fist element.");
        return false;
    }
    fifo->erase(fifo->begin());
    return true;
}


#endif //SERVICE_MANAGER_CPP_
