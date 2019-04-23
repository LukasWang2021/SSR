/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_manager_main.cpp
Author:     Feng.Wu 
Create:     07-Nov-2016
Modify:     22-Apr-2019
Summary:    dealing with service
**********************************************/
#ifndef SERVICE_MANAGER_MAIN_CPP_
#define SERVICE_MANAGER_MAIN_CPP_

#include "service_manager.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>


using namespace fst_service_manager;
using namespace fst_mc;

ServiceManager::ServiceManager():
    log_ptr_(NULL),
    param_ptr_(NULL),
    handle_core_(0),
    loop_count_core_(0),
    dtc_flag_(false),
    running_sid_ (0),
    is_exit_(false)
{
    log_ptr_ = new fst_log::Logger();
    FST_LOG_INIT("ServiceManager");
    param_ptr_ = new ServiceManagerParam();
}

ServiceManager::~ServiceManager()
{
    if(log_ptr_ != NULL){
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL){
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ErrorCode ServiceManager::init(void)
{
    handle_core_ = openMem(MEM_CORE);
    if (handle_core_ == -1) return OPEN_CORE_MEM_FAIL;
    clearSharedmem(MEM_CORE);

    if(!param_ptr_->loadParam()){
        FST_ERROR("Failed to load service_manager component parameters");
        return 1;
    }else{
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        FST_INFO("Success to load service_manager component parameters");
    }
    cycle_time_ = param_ptr_->cycle_time_;
    max_barecore_timeout_count_ = param_ptr_->max_barecore_timeout_count_;
    heartbeat_with_barecore_count_ = param_ptr_->heartbeat_with_barecore_count_;
    max_send_resp_count_ = param_ptr_->max_send_resp_count_;
 
    // Establish the communication channel with motion_control processes.
    ErrorCode result = comm_mcs_.createChannel(COMM_REP, COMM_IPC, "JTAC");
    if (result == CREATE_CHANNEL_FAIL)
    {
        FST_ERROR("Error in ServiceManager::init(): fail to create mcs channel.");
        return CREATE_CHANNEL_FAIL;
    }
    // Establish the communication channel with controller heartbeat processes.
    result = comm_controller_heartbeat_.createChannel(COMM_REP, COMM_IPC, "heartbeat");
    if (result == CREATE_CHANNEL_FAIL)
    {
        FST_ERROR("Error in ServiceManager::init(): fail to create controller heartBeat channel.");
        return CREATE_CHANNEL_FAIL;
    }
    // Establish the communication channel with servo_diag processes.
    result = comm_servo_diag_.createChannel(COMM_REP, COMM_IPC, "servo_diag");
    if (result == CREATE_CHANNEL_FAIL)
    {
        FST_ERROR("Error in ServiceManager::init(): fail to create servo_diag channel.");
        return CREATE_CHANNEL_FAIL;
    }
    
    //not used. todo delete
    result = comm_param_.createChannel(COMM_REP, COMM_IPC, "servo_param");
    if (result == CREATE_CHANNEL_FAIL)
    {
        FST_ERROR("Error in ServiceManager::init(): fail to create servo param channel.");
        return CREATE_CHANNEL_FAIL;
    }
    
    // Init the heartbeat_info request
    heartbeat_core_req_.req_id = HEARTBEAT_INFO_SID;
    request_fifo_.push_back(heartbeat_core_req_);

    // Init the version request
    ServiceRequest version_info = {READ_VERSION_SID, ""};
    request_fifo_.push_back(version_info);

    // Init the local heartbeat response
    heartbeat_local_resp_.res_id = MONITOR_HEARTBEAT_SID;

    channel_flag_ = LOCAL_CHANNEL;
    
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

    // Stop receive any request if there is any service in fifo.
    if (!response_fifo_.empty() || !request_fifo_.empty()) 
        return false;

    ServiceRequest request;
    //receive service request from the motion_control processes.
    ErrorCode result = comm_mcs_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == SUCCESS)
    {
        FST_INFO("recv controller request success with 0x%X, buff[0-3] = 0x%X", request.req_id, request.req_buff[0]);
        if (addRequest(request)) 
        {
            channel_flag_ = MCS_CHANNEL;
            return true;
        }
    }
    //receive service request from the controller heartbeat processes.
    result = comm_controller_heartbeat_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == SUCCESS)
    {
        //printf("recv controller heartbeat success with 0x%X\n", request.req_id);
        if (addRequest(request)) 
        {
            channel_flag_ = CONTROLLER_HEARTBEAT_CHANNEL;
            return true;
        }
    }
    //receive service request from the servo_diag processes.
    result = comm_servo_diag_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == SUCCESS)
    {
        //FST_INFO("recv servo_diag success with 0x%X", request.req_id);
        if (addRequest(request)) 
        {
            channel_flag_ = SERVO_DIAG_CHANNEL;
            return true;
        }
    }

    //todo delete
    result = comm_param_.recv(&request, sizeof(request), COMM_DONTWAIT);
    if (result == SUCCESS)
    {
        FST_INFO("---------------recv unkonw servoparam success with 0x%X------------", request.req_id);
        if (addRequest(request)) 
        {
            channel_flag_ = PARAM_CHANNEL;
            return true;
        }
    }

    // Setup the heartbeat request to BARE CORE after a number of loop time.
    if (addBareCoreHeartbeatRequest())
    {
        channel_flag_ = LOCAL_CHANNEL;
        return true;
    }

    return false;
}


//------------------------------------------------------------
// Function:  interactBareCore
// Summary: Send the request and wait for the response from BARE CORE, including heartbeat_core_req_. 
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
    int req_result = false;
    int res_result = false;
    int timeout_count = 0;

    while (res_result == false)
    {
        if (req_result == false)
        {
            req_result = sendBareCoreRequest();
        } 
        usleep(200);//wait for core1 response 
        if (req_result == true)
        {
            res_result = getBareCoreResponse();
        }

        timeout_count++;
        if ((timeout_count >= max_barecore_timeout_count_) && (res_result == false)) 
        {
            FST_ERROR("||====No heartbeat from CORE1====||");
            FST_ERROR("interactBareCore: handle request(0x%x), send result = %d, receive result = %d", request.req_id, req_result, res_result);
            ErrorCode timeout_error = BARE_CORE_TIMEOUT;
            storeError(timeout_error);
            return BARE_CORE_TIMEOUT;
        }
    }
    return FST_SUCCESS;
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
        case CONTROLLER_HEARTBEAT_CHANNEL:
            transmitResponse(comm_controller_heartbeat_);
            break;
        case SERVO_DIAG_CHANNEL:
            transmitResponse(comm_servo_diag_);
            break;
        case PARAM_CHANNEL://todo delete
            transmitResponse(comm_param_);
            break;
        default:
            break;
    }
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
    receiveRequest();
    interactBareCore();
    manageResponse();
    usleep(cycle_time_);

}

void ServiceManager::setExit(void)
{
    is_exit_ = true;
}
bool ServiceManager::isExit(void)
{
    return is_exit_;
}


fst_service_manager::ServiceManager* g_service_manager_ptr_ = NULL;

void serviceManagerOnExit(int dunno)
{
    g_service_manager_ptr_->setExit();
}


int main(int argc, char** argv)
{
    fst_service_manager::ServiceManager* service_manager_ptr= new fst_service_manager::ServiceManager();
    ErrorCode ret = service_manager_ptr->init();
    if (ret != 0) 
        return false;

    g_service_manager_ptr_ = service_manager_ptr;

    signal(SIGINT, serviceManagerOnExit);
    signal(SIGTERM, serviceManagerOnExit);

    while(!service_manager_ptr->isExit())
    {
        service_manager_ptr->runLoop();
    }
 
    delete service_manager_ptr;
    std::cout<<"service_manager exit"<<std::endl;
}


#endif //SERVICE_MANAGER_MAIN_CPP_
