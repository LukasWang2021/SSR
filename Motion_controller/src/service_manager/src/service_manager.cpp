/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_manager.cpp
Author:     Feng.Wu 
Create:     07-Nov-2016
Modify:     12-Dec-2016
Summary:    dealing with service
**********************************************/
#ifndef SERVICE_MANAGER_SERVICE_MANAGER_CPP_
#define SERVICE_MANAGER_SERVICE_MANAGER_CPP_

#include "service_manager/service_manager.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include "error_code.h"


namespace fst_service_manager
{

//------------------------------------------------------------
// Function:  ServiceManager
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
ServiceManager::ServiceManager()
{
    handle_core_ = 0;
    loop_count_core_ = 0;
    loop_count_mcs_ = 0;
    check_mcs_enable_ = false;
    running_sid_ = 0;
}

//------------------------------------------------------------
// Function:  ~ServiceManager
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
ServiceManager::~ServiceManager()
{
}

//------------------------------------------------------------
// Function:    init
// Summary: Initialize the communication channel and the sharedmem of cores.
// In:      None
// Out:     None
// Return:  0 -> success.
//          ERROR_CODE -> failed.
//------------------------------------------------------------
ErrorCode ServiceManager::init(void)
{
    handle_core_ = openMem(MEM_CORE);
    if (handle_core_ == -1) return OPEN_CORE_MEM_FAIL;
    clearSharedmem(MEM_CORE);
    
    // Establish the communication channel with other processes.
    ErrorCode result = comm_test_.createChannel(COMM_REP, COMM_IPC, "test");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error in CommMonito::init(): fail to create modbus channel."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }
    result = comm_mcs_.createChannel(COMM_REP, COMM_IPC, "JTAC");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error in CommMonito::init(): fail to create mcs channel."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }
    result = comm_param_.createChannel(COMM_REP, COMM_IPC, "servo_param");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error in CommMonito::init(): fail to create servo param channel."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }

    result = comm_tp_heartbeat_.createChannel(COMM_REP, COMM_IPC, "heartbeat");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error in ServiceManager::init(): fail to create tp heartBeat channel."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }

    // Init the heartbeat_info request
    heartbeat_core_.req_id = HEARTBEAT_INFO_SID;
    request_fifo_.push_back(heartbeat_core_);

    // Init the version request
    ServiceRequest version_info = {READ_VERSION_SID, ""};
    request_fifo_.push_back(version_info);

    // Init the local heartbeat response
    heartbeat_local_.res_id = MONITOR_HEARTBEAT_SID;

    channel_flag_ = LOCAL_CHANNEL;
/*//for independent test
ServiceResponse test = {READ_DTC_SID, ""};
const ErrorCode test_code = SEND_MSG_FAIL;
const ErrorCode test_code2 = OPEN_CORE_MEM_FAIL;
int s = 2;
memcpy(&test.res_buff[4], &s, 4);
memcpy(&test.res_buff[8], &test_code, sizeof(test_code));
memcpy(&test.res_buff[16], &test_code2, sizeof(test_code2));
response_fifo_.push_back(test);
*/
    
    return SUCCESS;
}

//------------------------------------------------------------
// Function:  receiveRequest
// Summary: Receive a request. 
// In:      None
// Out:     None
// Return:  true -> a request needs to be deal with.
//          false -> did nothing. 
//------------------------------------------------------------
bool ServiceManager::receiveRequest(void)
{
    // Caculate the loop number for the heartbeat request to BARE CORE.
    ++loop_count_core_;
    // Caculate the loop number for the heartbeat check of motion controller.
    if (check_mcs_enable_ == true)
        ++loop_count_mcs_;

    if (request_fifo_.size())
    printf("request size:%d\n", request_fifo_.size());
    // Stop receive any request if there is any service in fifo.
    if (!response_fifo_.empty() || !request_fifo_.empty()) 
        return false;

    // Setup the heartbeat request to BARE CORE after a number of loop time.
    if (addRequest())
    {
        channel_flag_ = LOCAL_CHANNEL;
        return true;
    }

    ServiceRequest request;
    ErrorCode result = comm_mcs_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == 0)
    {
        printf("recv servo mcs success with %X\n", request.req_id);
        //push the request into fifo.
        if (checkRequest(request)) 
        {
            channel_flag_ = MCS_CHANNEL;
            return true;
        }
    }

    result = comm_param_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == 0)
    {
        printf("recv servo param success with %X\n", request.req_id);
        //push the request into fifo.
        if (checkRequest(request)) 
        {
            channel_flag_ = PARAM_CHANNEL;
            return true;
        }
    }
	
    result = comm_tp_heartbeat_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == 0)
    {
        // printf("recv servo heartbeat success with %X\n", request.req_id);
        //push the request into fifo.
        if (checkRequest(request)) 
        {
            channel_flag_ = TP_HEARTBEAT_CHANNEL;
            return true;
        }
    }

    result = comm_test_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == 0)
    {
        printf("recv servo test success with %X\n", request.req_id);
        //push the request into fifo.
        if (checkRequest(request)) 
        {
            channel_flag_ = TEST_CHANNEL;
            return true;
        }
    }
    return false;
}

//------------------------------------------------------------
// Function:  addRequest
// Summary: Setup the heartbeat request to BARE CORE after a number of loop. 
//          setup the stop request to BARE CORE if heartbeat_mcs missing.
// In:      None
// Out:     None
// Return:  true -> a request to BARE CORE is setup.
//          false -> didn't setup request.
//------------------------------------------------------------
bool ServiceManager::addRequest(void)
{  
    // Caculate the loop number, setup the stop request to BARE CORE.
    if (loop_count_mcs_ >= HEARTBEAT_INTERVAL_MCS)
    {
        ServiceRequest req = {JTAC_CMD_SID, ""};
        int stop = 1;
        memcpy(&req.req_buff[0], &stop, sizeof(stop));
        request_fifo_.push_back(req);
        std::cout<<"||====No heartbeat from MCS, a stop command was sent.====||"<<std::endl;

                    
        //store the motion controller timeout error.
        ErrorCode mcs_timeout = MCS_TIMEOUT;
        storeError(mcs_timeout);

        check_mcs_enable_ = false;
        loop_count_mcs_ = 0;

        return true;
    }
 
    // Caculate the loop number, setup the heartbeat request to BARE CORE.
    if (loop_count_core_ >= HEARTBEAT_INTERVAL_CORE)
    {
        request_fifo_.push_back(heartbeat_core_);
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
    if (response_action_.getDtcFlag() == 0)
    {
        return false;
    }
    else if (response_action_.getDtcFlag() == 1)
    {
        ServiceRequest dtc_req = {READ_DTC_SID,""};
        request_fifo_.push_back(dtc_req);
        response_action_.clearDtcFlag();
    }
    return true;
}

//------------------------------------------------------------
// Function:  checkRequest
// Summary: Mainly check if the request id is defined. 
// In:      None
// Out:     None
// Return:  true -> the request id is available.
//          false -> invailid request id.
//------------------------------------------------------------
bool ServiceManager::checkRequest(ServiceRequest req)
{
    // Check whether it is heartbeat request from the other process.
    if (isLocalRequest(req)) 
        return true;
    request_fifo_.push_back(req);
    return true;
    
    // Push the non-heartbeat request into this fifo.
/*    if (response_action_.searchServiceTableIndex(req.req_id) != -1)
    {
        request_fifo_.push_back(req);
        return true;
    }
    // Record the error if unexpected request is received.
    ErrorCode invalid_sid = INVALID_SERVICE_ID;
    storeError(invalid_sid);
    std::cout<<"Error in CommMonitor::checkRequest(): The request id(0x"<<std::hex<<req.req_id<<std::dec<<") is not available"<<std::endl;
    return false;
*/
}

//------------------------------------------------------------
// Function:  isLocalRequest
// Summary: push local heartbeat request into local fifo if receive it.
//          Fill the local heartbeart response with error_fifo_.
// In:      None
// Out:     None
// Return:  true -> receive a heartbeat request.
//          false -> not heartbeat request.
//------------------------------------------------------------
bool ServiceManager::isLocalRequest(ServiceRequest req)
{
    if (req.req_id == MONITOR_HEARTBEAT_SID)
    {
        // Fill the heartbeat context with error_fifo_.
        fillLocalHeartbeat();
        response_fifo_.push_back(heartbeat_local_);
        check_mcs_enable_ = true;
        loop_count_mcs_ = 0;
        return true;
    }
    return false;
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
        memcpy(&(heartbeat_local_.res_buff[0]), &size, sizeof(size));
        for (size_t i = 0; i < size; ++i)
        {
            ErrorCode error = error_fifo_[0];
            memcpy(&(heartbeat_local_.res_buff[8 + i*8]), &error, sizeof(error));
            deleteFirstElement(&error_fifo_);
            printf("local heartbeat:id = %d, %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", heartbeat_local_.res_id, (unsigned char)heartbeat_local_.res_buff[7+8+ i*8],(unsigned char)heartbeat_local_.res_buff[6+8+ i*8],(unsigned char)heartbeat_local_.res_buff[5+8+ i*8],(unsigned char)heartbeat_local_.res_buff[4+8+ i*8],(unsigned char)heartbeat_local_.res_buff[3+8+ i*8],(unsigned char)heartbeat_local_.res_buff[2+8+ i*8],(unsigned char)heartbeat_local_.res_buff[1+8+ i*8],(unsigned char)heartbeat_local_.res_buff[0+8+ i*8]);

        }
    } else if (size == 0)
    {
        memset(&(heartbeat_local_.res_buff[0]), 0, sizeof(heartbeat_local_.res_buff));
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
bool ServiceManager::sendRequest(void)
{
    if (request_fifo_.empty()) 
        return false;

    ServiceRequest temp_request = request_fifo_[0];
    //if (temp_request.req_id == JTAC_CMD_SID)
    //{
        //int *a = (int*)(temp_request.req_buff);
        //std::cout<<"==service to core1 id = "<<*a<<std::endl;
    /*}*/
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
bool ServiceManager::getResponse(void)
{
    if (!response_fifo_.empty()) 
        return false;
    ServiceResponse response;  

    bool result = recvResponseFromBareCore(response);
    //printf("response resul:%d\n", result);
    if (result == true && (response.res_id == running_sid_))
    {
        response_fifo_.push_back(response);
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  runService
// Summary: Send the request and wait for the response from BARE CORE, including heartbeat_core_. 
//          Create timeout error if abnormal.
// In:      None
// Out:     None
// Return:  0 -> success to send a request and receive a response.
//          BARE_CORE_TIMEOUT -> timeout. 
//------------------------------------------------------------
ErrorCode ServiceManager::interactBareCore(void)
{
    if (request_fifo_.empty())
        return SUCCESS;  

    if (!response_fifo_.empty())
        return SUCCESS;

    ServiceRequest request = request_fifo_[0];
    int req_result = false, res_result = false;
    struct timeval t_start, t_end;
    long cost_time = 0;
    gettimeofday(&t_start, NULL);
    while (res_result == false)
    {
        if (req_result == false) req_result = sendRequest();
        //wait for core1 response 
        usleep(200);
        if (req_result == true) res_result = getResponse();

        gettimeofday(&t_end, NULL);
        cost_time = (t_end.tv_sec - t_start.tv_sec) * SEC_TO_USEC + (t_end.tv_usec - t_start.tv_usec);
        if (cost_time > HEARTBEAT_CORE_TIMEOUT)
        {
            std::cout<<"\033[31m"<<"||====No heartbeat from CORE1====||"<<"\033[0m"<<std::endl;
            std::cout<<"BARE CORE response "<<request.req_id<<" time is "<<cost_time<<" us. ";
            std::cout<<"send="<<req_result<<". recv="<<res_result<<std::endl;
            ErrorCode timeout_error = BARE_CORE_TIMEOUT;
            storeError(timeout_error);
            return BARE_CORE_TIMEOUT;
        }
    }// end while (res_result == false)
    // For debug
    if (cost_time > 5000)
        std::cout<<"BARE CORE response "<<request.req_id<<" time is "<<cost_time<<" us."<<std::endl;
    return FST_SUCCESS;
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
// Function:  manageResponse
// Summary: deal with the service response from BARE CORE. 
//          Some responses are deal with locally. 
//          Some are sent to the other processes.
// In:      None
// Out:     None
// Return:  true -> a response is disposed.
//          false -> did nothing. 
//------------------------------------------------------------
bool ServiceManager::manageResponse(void)
{
    if (response_fifo_.empty()) 
        return false;

    switch (channel_flag_)
    {
        case LOCAL_CHANNEL:
            manageLocalResponse();
            break;
        case MCS_CHANNEL:
            transmitResponse(comm_mcs_);
            break;
        case PARAM_CHANNEL:
            transmitResponse(comm_param_);
            break;
        case TEST_CHANNEL:
            transmitResponse(comm_test_);
            break;
        case TP_HEARTBEAT_CHANNEL:
            transmitResponse(comm_tp_heartbeat_);
            break;
        default:
            break;
    }
    return true;
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
    int index = response_action_.searchResponseLocalTableIndex(temp_response.res_id);
    if (index != -1) 
    {
        extractErrorCode(temp_response);
        response_action_.response_local_table[index].function(&temp_response);
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
        if (count >= SEND_RESP_ATTEMPTS)
        {
            std::cout<<"Error in CommMonitor::transmitResponse():fail to send response id(0x"<<std::hex<<temp_response.res_id<<std::dec<<") within limit tries. It was dropped."<<std::endl;
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
// Function:  extractErrorCode
// Summary: Extract error codes from dtc response.. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> did nothing. 
//------------------------------------------------------------
bool ServiceManager::extractErrorCode(ServiceResponse resp)
{
    if (resp.res_id == READ_DTC_SID)
    {
        ErrorCode error_code = 0;
        unsigned int size = *(int*)(&(resp.res_buff[4]));

        for (unsigned int i = 0;  i < size; ++i)
        {
            memcpy(&error_code, &resp.res_buff[8 + i*8], 8);
            printf("extract error code = %016llX\n", error_code);
            storeError(error_code);
        }
        return true;
    }
    return false;
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
        std::cout<<"Error when delete fifo fist element."<<std::endl;
        return false;
    }
    fifo->erase(fifo->begin());
    return true;
}

//------------------------------------------------------------
// Function:  runLoop
// Summary: The main loop to run this process. 
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void ServiceManager::runLoop(void)
{
    while (true)
    {
        receiveRequest();
        interactBareCore();
        manageResponse();
        usleep(LOOP_TIME);
    }
}

} //namespace fst_service_manager


int main(int argc, char** argv)
{
/* //for independent test
    if(!fork())//mimic the mcs process. 
    {
        fst_comm_interface::CommInterface comm;

        int fd = comm.createChannel(IPC_REQ, "mcs");
        if (fd == CREATE_CHANNEL_FAIL)
        {
            std::cout<<"Error in Child process: fail to create mcs channel."<<std::endl;
        }
        ServiceRequest req2 = {0x123, ""};
        comm.send(&req2, sizeof(req2), IPC_DONTWAIT);

        ServiceRequest req = {MONITOR_HEARTBEAT_SID, ""};
        while (true)
        {
//            sleep(1);
            usleep(50000);
            ErrorCode result = comm.send(&req, sizeof(req), IPC_DONTWAIT);
            if (result == 0)
            {
//                std::cout<<"send heartbeat req ok."<<std::endl;
            }

            ServiceResponse resp;
            ErrorCode rec = comm.recv(&resp, sizeof(resp), IPC_WAIT);
            if (rec == 0)
            {
                size_t size;
                memcpy(&size, &resp.res_buff[0], sizeof(size));
//                printf("recv size = %zu\n", (size_t)size);
//                printf("recv heartbeat:id = %d, %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", resp.res_id, (unsigned char)resp.res_buff[7+8],(unsigned char)resp.res_buff[6+8],(unsigned char)resp.res_buff[5+8],(unsigned char)resp.res_buff[4+8],(unsigned char)resp.res_buff[3+8],(unsigned char)resp.res_buff[2+8],(unsigned char)resp.res_buff[1+8],(unsigned char)resp.res_buff[0+8]);
            }
        }

    }else
*/    {

    fst_service_manager::ServiceManager cm;
    ErrorCode init_result = cm.init();
    if (init_result != 0) 
        return false;

    cm.runLoop();

    }

}


#endif //SERVICE_MANAGER_SERVICE_MANAGER_CPP_
