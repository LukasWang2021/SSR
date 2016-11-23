/**********************************************
File: comm_monitor.cpp
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: service handling
Author: Feng.Wu 07-Nov-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_COMM_MONITOR_CPP_
#define MIDDLEWARE_TO_MEM_COMM_MONITOR_CPP_

#include "comm_monitor/comm_monitor.h"
#include <sys/time.h>
#include <iostream>

namespace fst_comm_monitor
{

CommMonitor::CommMonitor()
{
    handle_process_ = 0;
    handle_core_ = 0;
    time_count_ = 0;
}

//------------------------------------------------------------
// Function:    init
// Summary: Open the shared memory area.
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed.
//------------------------------------------------------------
int CommMonitor::init()
{
    handle_process_ = openMem(MEM_PROCESS);
    if (handle_process_ == -1) return false;
    clearSharedmem(MEM_PROCESS);
    handle_core_ = openMem(MEM_CORE);
    if (handle_core_ == -1) return false;
    clearSharedmem(MEM_CORE);

    //init the heartbeat_info request
    heartbeat_info_.req_id = HEARTBEAT_INFO_SID;
    request_fifo_.push_back(heartbeat_info_);

    //init the version request
    ServiceRequest version_info = {READ_VERSION_SID, "request for core1 version\n"};
    request_fifo_.push_back(version_info);

    return true;
}

//------------------------------------------------------------
// Function:  receiveRequest
// Summary: Receive the request from the motion controll system. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::receiveRequest()
{
    if (!response_fifo_.empty() || !request_fifo_.empty()) return false;

    ServiceRequest request;
    int read_result = serverGetRequest(handle_process_, &request);
    if (read_result == true)
    { 
        //push the request into fifo.
        request_fifo_.push_back(request);
        time_count_ = 0;
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  sendRequest
// Summary: send the service request to CORE1. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::sendRequest()
{
    if (request_fifo_.empty()) return false;

    ServiceRequest temp_request = request_fifo_[0];   
    int result = sendCore(temp_request);
    if (result == true)
    {
        request_fifo_.erase(request_fifo_.begin());
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  getResponse
// Summary: get the service response from CORE1. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::getResponse()
{
    if (!response_fifo_.empty()) return false;

    ServiceResponse response;    
    int result = recvCore(response);
    if (result == true)
    {
        response_fifo_.push_back(response);
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  handleService
// Summary: the main method of the service handling. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::runService()
{
    if (request_fifo_.empty())
    {
        ++time_count_;
        if (time_count_ == HEARTBEAT_CYCLE)
        {
            request_fifo_.push_back(heartbeat_info_);
            time_count_ = 0;
        } else
        {
            return false;
        }
    }

    int req_result = false, res_result = false;
    struct timeval t_start, t_end;
    gettimeofday(&t_start, NULL);
    while (res_result == false)
    {
        if (req_result == false) req_result = sendRequest();
        //wait for core1 response 
        usleep(200);
        if (req_result == true) res_result = getResponse();

        gettimeofday(&t_end, NULL);
        long cost_time = (t_end.tv_sec - t_start.tv_sec) * SEC_TO_USEC + (t_end.tv_usec - t_start.tv_usec);
        if (cost_time > HEARTBEAT_LIMIT)
        {
            std::cout<<"\033[31m"<<"||====No heartbeat from CORE1====||"<<"\033[0m"<<std::endl;
            return false;
        }
    }// end while (res_result == false)
    return true;
}

//------------------------------------------------------------
// Function:  manageResponse
// Summary: deal with the service response from CORE1. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::manageResponse()
{
    if (response_fifo_.empty()) return false;

    ServiceResponse temp_response = response_fifo_[0];
    int index = response_action_.searchServiceTableIndex(&temp_response);
    if (index != -1) 
    {
        response_action_.response_table[index].function(&temp_response);
        response_fifo_.erase(response_fifo_.begin());
    } else
    {
        int send_result = serverSendResponse(handle_process_, &temp_response);
        if (send_result == true)
        {
            response_fifo_.erase(response_fifo_.begin());
        }
    }
    return true;
}

//------------------------------------------------------------
// Function:  sendCore
// Summary: send the request to CORE1. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::sendCore(ServiceRequest &request)
{
    return clientSendRequest(handle_core_, &request);
}

//------------------------------------------------------------
// Function:  recvCore
// Summary: get the response from CORE1. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool CommMonitor::recvCore(ServiceResponse &response)
{
    return clientGetResponse(handle_core_, &response);
}

CommMonitor::~CommMonitor()
{
}
} //namespace fst_trajectory_queue


int main(int argc, char** argv)
{
    fst_comm_monitor::CommMonitor cm;
    int init_result = cm.init();
    if (init_result == -1) return false;

    while (true)
    {
        cm.receiveRequest();
        cm.runService();
        cm.manageResponse();
        usleep(1000);
    }

}


#endif //MIDDLEWARE_TO_MEM_COMM_MONITOR_CPP_
