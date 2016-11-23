/**********************************************
File: comm_interface.cpp
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: lib to communicate with core1
Author: Feng.Wu 04-Nov-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_
#define MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_

#include "comm_interface/comm_interface.h"
#include <iostream>
#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>

namespace fst_comm_interface
{

CommInterface::CommInterface()
{
    handle_core_ = 0;
    sec_ = 0;
    nsec_ = 0;
    prev_sec_ = 0;
    prev_nsec_ = 0;
    time_step_ = 1000000; //the unit is ns.
    //init the joint states fbjs_ for fake
    for (int j = 0; j<JOINT_NUM; ++j)
    {
        fbjs_.position[j] = 0;
        fbjs_.velocity[j] = 0;
        fbjs_.effort[j] = 0;
    }
    fbjs_.state = STATE_READY;

}

bool CommInterface::init()
{
    handle_core_ = openMem(MEM_CORE);
    if (handle_core_ == -1) return false;
    clearSharedmem(MEM_CORE);
    return true;
}

//------------------------------------------------------------
// Function:  initTrajectory
// Summary: Initial the trajectory variable. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void CommInterface::initTrajectory(void)
{
    //init the TrajectorySegment ts.
    for (int i = 0; i < TS_POINT_NUM; ++i)
    {
        for (int j = 0; j < JOINT_NUM; ++j)
        {
            ts_.points[i].positions[j] = 0;
            ts_.points[i].velocities[j] = 0;
            ts_.points[i].accelerations[j] = 0;
            ts_.points[i].effort[j] = 0;
        }
        ts_.points[i].valid_level = 1;
        ts_.points[i].time_from_start.sec = 0;
        ts_.points[i].time_from_start.nsec = 0;
    }
    ts_.stamp.sec = 0;
    ts_.stamp.nsec = 0;
    ts_.total_points = 0;
    ts_.seq = 0;
    ts_.last_fragment = 0;
}

//------------------------------------------------------------
// Function:  sendBareCore
// Summary: Add the time stamp for each points. 
//          Send the trajectory segments to CORE1.
// In:      jc -> the desired values of all joints
// Out:     None
// Return:  true -> success.
//          false -> failed to send the trajectory to CORE1
//------------------------------------------------------------
bool CommInterface::sendBareCore(JointCommand jc)
{   
    prev_sec_ = sec_;
    prev_nsec_ = nsec_;
    initTrajectory();
    for (int i = 0; i < jc.total_points; ++i)
    {
        for (int j = 0; j < JOINT_NUM; ++j)
        {
            ts_.points[i].positions[j] = jc.points[i].positions[j];
        }
        //Adding time stamp for trajectory. 
        if (jc.points[i].point_position == START_POINT)
        {
            sec_ = 0;
            nsec_ = time_step_; 
        }
        else
        {
            nsec_ += time_step_;
            if (nsec_ >= SEC_TO_NSEC)
            {
                nsec_ -= SEC_TO_NSEC;
                sec_ += 1;
            }
        }
        ts_.points[i].time_from_start.sec = sec_;
        ts_.points[i].time_from_start.nsec = nsec_;

        //Set the flag last_fragment to indicate the last point of trajectory.  
        if (jc.points[i].point_position == END_POINT) 
        {
            ts_.last_fragment = 1;
            sec_ = 0;
            nsec_ = 0;
        }
    } // end for (int i = 0; i < jc.total_points; ++i)
    ts_.total_points = jc.total_points;

    int write_result = readWriteSharedMem(handle_core_, &ts_, "TrajectorySegment", MEM_WRITE);
    if (write_result == false)
    {
        sec_ = prev_sec_;
        nsec_ = prev_nsec_;
        return false;
    }
    return true;
}

//------------------------------------------------------------
// Function:  recvBareCore
// Summary: Read the actual joint states from CORE1
// In:      None
// Out:     fbjs -> the actual joint states.
// Return:  true -> success.
//          false -> failed to read the actural joint states from the servo core(lower level) 
//------------------------------------------------------------
bool CommInterface::recvBareCore(FeedbackJointState &fbjs)
{
    return readWriteSharedMem(handle_core_, &fbjs, "FeedbackJointState", MEM_READ);
}


//------------------------------------------------------------
// Function:  sendBareCoreFake
// Summary: Send the trajectory segments to CORE1. (fake for test) 
// In:      jc -> the desired values of all joints
// Out:     None
// Return:  true -> success.
//          false -> failed to send the trajectory to CORE1
//------------------------------------------------------------
bool CommInterface::sendBareCoreFake(JointCommand jc)
{
    if (joints_in_fifo_.size() > 40) return false;
    //push the trajectory joint states into fifo.
    for(int i = 0; i < jc.total_points; ++i)
    {
        joints_in_fifo_.push_back(jc.points[i]);
    }
    return true;
}

//------------------------------------------------------------
// Function:  recvBareCoreFake
// Summary: Read the actual joint states from CORE1 (fake for test)
// In:      None
// Out:     fbjs -> the actual joint states.
// Return:  true -> success.
//          false -> failed to read the actural joint states from the servo core(lower level) 
//------------------------------------------------------------
bool CommInterface::recvBareCoreFake(FeedbackJointState &fbjs)
{
    //if no trajectory, feedback the last states.
    if (joints_in_fifo_.empty()) 
    {
        for (int i = 0; i < JOINT_NUM; ++i)
        {
            fbjs.position[i] = fbjs_.position[i];
        }
        fbjs_.state = STATE_READY;
        fbjs.state = STATE_READY;
        return true;
    }

    //set trajectory joint states as feedback joint states
    for (int j = 0; j < JOINT_NUM; ++j)
    {
        fbjs_.position[j] = joints_in_fifo_[0].positions[j];
        fbjs.position[j] = fbjs_.position[j];
    }
    int num;
    if (joints_in_fifo_.size() > 10)
        num = 10;
    else
        num = joints_in_fifo_.size();
    joints_in_fifo_.erase(joints_in_fifo_.begin(), joints_in_fifo_.begin()+num-1);
    fbjs.state = STATE_RUNNING;
    return true;
}

//------------------------------------------------------------
// Function:  createChannel
// Summary: Create the channel for IPC communication
// In:      type -> REQ:request, REP:response, PUB:publisher, SUB:subscriber
//          name -> name for each channel
// Out:     None
// Return:  fd -> succeed to get handle.
//          -1 -> failed to create channel 
//------------------------------------------------------------
int CommInterface::createChannel(int type, const char *name)
{
    //create a socket.
    int fd = nn_socket(AF_SP, type);
    if(fd < 0)
    {
        std::cout<<"Error in CommInterface::createChannel() for socket: "<<nn_strerror(nn_errno())<<std::endl;
        return -1;
    }

    //bind or connect to url.
    char url[64];
    sprintf(url, "ipc:///tmp/msg_%s.ipc", name);
    if (type == NN_REP || type == NN_PUB)
    {
        if (nn_bind(fd, url) < 0)
        {
            std::cout<<"Error in CommInterface::createChannel() to bind: "<<nn_strerror(nn_errno())<<std::endl;
            return -1;
        }
    } else if (type == NN_REQ || type == NN_SUB)
    {
        if (nn_connect(fd, url) < 0)
        {
            std::cout<<"Error in CommInterface::createChannel() to connect: "<<nn_strerror(nn_errno())<<std::endl;
            return -1;
        }
    } else
    {
        std::cout<<"Info in CommInterface::createChannel(): Please enter a type(REQ, REP, PUB, SUB)."<<std::endl;
        return -1;
    }
    //set sockopt specially for NN_SUB, otherwise no receive.
    if (type == NN_SUB)
    {
        if (nn_setsockopt(fd, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0)
        {
            std::cout<<"Error in CommInterface::createChannel() to set opt: "<<nn_strerror(nn_errno())<<std::endl;
            return -1;
        };
    }

    return fd;
}
/*
int CommInterface::waitForRep(int fd)
{
    int check = -1;
    char send_buff[64] = "Is rep server started up???";
    char recv_buff[64];
    std::cout<<"CLIENT: Waiting for the response server to start up before sending msg"<<std::endl;
    while (check == -1)
    {
        send(fd, send_buff, sizeof(send_buff));
        usleep(100000);
        check = recv(fd, recv_buff, sizeof(recv_buff));
    }
    std::cout<<recv_buff<<std::endl;
    return true;
}

int CommInterface::waitForReq(int fd)
{
    int check = -1;
    char recv_buff[64];
    char send_buff[64] = "CLINET: The response server is started up, msg can be sent.";
    std::cout<<"SERVER: Waiting for the request client to start up."<<std::endl;
    while (check == -1)
    {
        usleep(1000);
        if (recv(fd, recv_buff, sizeof(recv_buff) >= 0))
        {
            check = send(fd, send_buff, sizeof(send_buff));
        }
    }
    std::cout<<"SERVER: The request client is started up."<<std::endl;      
    usleep(100000);

    return true;
}
*/
//------------------------------------------------------------
// Function:  send
// Summary: send data
// In:      fd -> handle from method createChannel().
//          buff -> data to be sent.
//          buff_size -> date size
// Out:     None
// Return:  1 -> succeed to send data.
//          -1 -> failed to send data.
//------------------------------------------------------------
int CommInterface::send(int fd, void *buff, int buff_size)
{
    if (fd == -1)
    {
        std::cout<<"Error in CommInterface::send(): Wrong fd, please check createChannel()."<<std::endl;
        return -1;
    }
    if (nn_send(fd, buff, buff_size, 0) < 0)
    {
        std::cout<<"Error in CommInterface::send(): "<<nn_strerror(nn_errno())<<std::endl;
        return -1;
    }
    return 1;
}

//------------------------------------------------------------
// Function:  recv
// Summary: receive data
// In:      fd -> handle from method createChannel().
//          buff -> data to be read.
//          buff_size -> date size
// Out:     None
// Return:  1 -> succeed to get data.
//          -1 -> failed to get data.
//------------------------------------------------------------
int CommInterface::recv(int fd, void *buff, int buff_size)
{
    if (fd == -1)
    {
        std::cout<<"Error in CommInterface::recv(): Wrong fd, please check createChannel()."<<std::endl;
        return -1;
    }
/*    int recvfd;
    size_t sz = sizeof(recvfd);
    int ret = nn_getsockopt(fd, NN_SOL_SOCKET, NN_RCVFD, &recvfd, &sz);
    std::cout<<"recvfd:"<<recvfd<<std::endl;
*/         
    if (nn_recv(fd, buff, buff_size, NN_DONTWAIT) < 0)
    {
        if (nn_errno() != EAGAIN)
        {
            std::cout<<"Error in CommInterface::recv(): "<<nn_strerror(nn_errno())<<std::endl;
        }
        return -1;
    }
    
    return 1;
}

//------------------------------------------------------------
// Function:  close
// Summary: closeChannel
// In:      fd -> handle from method createChannel().
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void CommInterface::close(int fd)
{
    nn_close(fd);
}

//------------------------------------------------------------
// Function:  setTimeStamp
// Summary: set the time stamp. default is 0.001s 
// In:      interval -> 0.001 is default
// Out:     None
// Return:  None
//------------------------------------------------------------
void CommInterface::setTimeInterval(double interval)
{
    time_step_ = interval * SEC_TO_NSEC;
}

CommInterface::~CommInterface()
{
}

} //namespace fst_comm_interface

#endif //MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_
