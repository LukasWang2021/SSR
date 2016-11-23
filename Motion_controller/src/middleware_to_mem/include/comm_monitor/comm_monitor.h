/**********************************************
File: comm_monitor.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: service handling
Author: Feng.Wu 07-Nov-2016
Modifier:
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_COMM_MONITOR_H_
#define MIDDLEWARE_TO_MEM_COMM_MONITOR_H_

#include <vector>
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include "service_actions/response_actions.h"

namespace fst_comm_monitor
{
/*fst trajectory being sent to the shared memory*/

class CommMonitor
{
public:
    CommMonitor();
    ~CommMonitor();
    int init();
    bool receiveRequest();
    bool sendRequest();
    bool getResponse();
    bool runService();
    bool manageResponse();
    bool sendCore(ServiceRequest &request);
    bool recvCore(ServiceResponse &response);

    static const int HEARTBEAT_CYCLE = 100; // 1ms times 100.
    static const int HEARTBEAT_LIMIT = 100000; //100ms.
    static const unsigned int SEC_TO_USEC = 1000000; //converting from second to micorsecond
 
private:
    int handle_process_;
    int handle_core_;
    int time_count_;
    ServiceRequest heartbeat_info_;
    std::vector<ServiceRequest> request_fifo_;
    std::vector<ServiceResponse> response_fifo_;
    fst_response_action::ResponseAction response_action_;
};
} //namespace fst_comm_monitor

#endif //MIDDLEWARE_TO_MEM_COMM_MONITOR_H_
