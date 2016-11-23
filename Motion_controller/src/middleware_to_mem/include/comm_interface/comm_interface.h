/**********************************************
File: comm_interface.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: lib to communicate with core1
Author: Feng.Wu 04-Nov-2016
Modifier:
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_
#define MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_

#include <vector>
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_feedback_joint_states.h"

#define REQ 48
#define REP 49
#define PUB 32
#define SUB 33
#define IPC_BLOCK 0
#define IPC_NONBLOCK 1 

namespace fst_comm_interface
{

class CommInterface
{
public:
    CommInterface();
    ~CommInterface();
    bool init();
    void initTrajectory(void);
    bool sendBareCore(JointCommand jc);
    bool recvBareCore(FeedbackJointState &fbjs);
    bool sendBareCoreFake(JointCommand jc); // for fake test
    bool recvBareCoreFake(FeedbackJointState &fbjs); // for fake test
    int createChannel(int type, const char *name = "");
//    int waitForRep(int fd);
//    int waitForReq(int fd);
    int send(int fd, void *buff, int buff_size);
    int recv(int fd, void *buff, int buff_size);
    void close(int fd);
    void setTimeInterval(double interval);

    static const unsigned int SEC_TO_NSEC = 1000000000; //converting from second to microsecond
    
private:
    int handle_core_;
    TrajectorySegment ts_;
    unsigned int sec_;
    unsigned int nsec_;
    unsigned int prev_sec_;
    unsigned int prev_nsec_;
    unsigned int time_step_;
    std::vector<Points> joints_in_fifo_; // for fake test
    FeedbackJointState fbjs_; // for fake test
};
} //namespace fst_comm_interface

#endif //MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_
