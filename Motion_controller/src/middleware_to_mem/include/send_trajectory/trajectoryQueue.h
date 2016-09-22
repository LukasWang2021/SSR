/**********************************************
File: TrajectoryQueue.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: trajectory queue to be sent
Author: 16-Aug-2016
Modifier:
**********************************************/

#ifndef _TRAJECTORY_QUEUE_H_
#define _TRAJECTORY_QUEUE_H_

#include <vector>
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_bare_core_version.h"

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
    bool receivedTrajectory();
    bool sendTrajectory();
    bool readJointStates();
    bool sendJointStates();
    void setTimeInterval(double interval);
    void versionInfo();
    static const int FIFO_LEN = 50;
    static const int ATTEMPTS = 2;
    static const unsigned int SEC_TO_NSEC = 1000000000;

private:
    int handle_process;
    int handle_core;
    std::vector<Points> joints_in_fifo_;
    JointCommand jc_;
    TrajectorySegment ts_;
    FeedbackJointState fbjs_;
    double interval_;
    unsigned int sec_;
    unsigned int nsec_;
    unsigned int time_step_;
    int flag_;
};
}

#endif
