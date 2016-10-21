/**********************************************
File: trajectory_queue.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: trajectory queue to be sent
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_H_
#define MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_H_

#include <vector>
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

namespace fst_trajectory_queue
{
/*fst trajectory being sent to the shared memory*/

class TrajectoryQueue
{
public:
    TrajectoryQueue();
    ~TrajectoryQueue();
    int init();
    void initTrajectory();
    bool receiveTrajectory();
    bool sendTrajectory();
    bool readJointStates();
    bool sendJointStates();
    void setTimeInterval(double interval);

    bool receiveRequest();
    bool sendRequest();
    bool getResponse();
    bool manageResponse();
    bool startServiceThread();
    static void *serviceThreadHelper(void *self);
    static void *heartbeatThreadHelper(void *self);
    void runService();
    void runHeartbeat();

    static const int FIFO_LEN = 50;
    static const int ATTEMPTS = 2;
    static const int SERVICE_CYCLE = 20000; //the unit is us.
    static const int HEARTBEAT_CYCLE = 200000; //the unit is us.
    static const int HEARTBEAT_LIMIT = 200000; //the unit is us.
    static const unsigned int SEC_TO_NSEC = 1000000000; //converting from second to microsecond
    

private:
    int handle_process_;
    int handle_core_;
    std::vector<Points> joints_in_fifo_;
    JointCommand jc_;
    TrajectorySegment ts_;
    FeedbackJointState fbjs_;
    double interval_;
    unsigned int sec_;
    unsigned int nsec_;
    unsigned int time_step_;
    int flag_;
    ServiceRequest request_;
    ServiceResponse response_;
    std::vector<ServiceRequest> request_fifo_;
    std::vector<ServiceResponse> response_fifo_;
};
} //namespace fst_trajectory_queue

#endif //MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_H_
