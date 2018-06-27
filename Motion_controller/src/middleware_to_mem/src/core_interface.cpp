/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       core_interface.cpp
Author:     Feng.Wu 
Create:     04-Nov-2016
Modify:     09-Jun-2017
Summary:    lib to communicate with core1
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_CORE_INTERFACE_CPP_
#define MIDDLEWARE_TO_MEM_CORE_INTERFACE_CPP_

#include "comm_interface/core_interface.h"
#include <iostream>
#include <sstream>
#include "middleware_to_mem_version.h"

namespace fst_core_interface
{

//------------------------------------------------------------
// Function:  getVersion
// Summary: get the version. 
// In:      None
// Out:     None
// Return:  std::string -> the version.
//------------------------------------------------------------
std::string getVersion(void)
{
     std::stringstream ss;
    ss<<middleware_to_mem_VERSION_MAJOR<<"."
        <<middleware_to_mem_VERSION_MINOR<<"."
        <<middleware_to_mem_VERSION_PATCH;
    std::string s = ss.str();

    return s;
}

//------------------------------------------------------------
// Function:  CoreInterface
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
CoreInterface::CoreInterface()
{
    handle_core_ = 0;
    error_flag_ = OPEN_CORE_MEM_FAIL;
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
    std::cout<<"lib_core_interface version:"<<getVersion()<<std::endl;
}

//------------------------------------------------------------
// Function:  CoreInterface
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
CoreInterface::~CoreInterface()
{
}

//------------------------------------------------------------
// Function:  init
// Summary: Initialize the shared memory of cores
// In:      None
// Out:     None
// Return:  0 -> succeed to initialize the shared memory.
//          OPEN_CORE_MEM_FAIL -> failed 
//------------------------------------------------------------
ERROR_CODE_TYPE CoreInterface::init(void)
{
    handle_core_ = openMem(MEM_CORE);
    if (handle_core_ == -1) 
    {
        return OPEN_CORE_MEM_FAIL;
    }
    clearSharedmem(MEM_CORE);
    error_flag_ = 0;
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  initTrajectory
// Summary: Initialize the trajectory variable. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void CoreInterface::initTrajectory(void)
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
// Return:  0 -> success to send the trajectory to CORE1.
//          WRITE_CORE_MEM_FAIL -> failed.
//          OPEN_CORE_MEM_FAIL -> failed
//------------------------------------------------------------
ERROR_CODE_TYPE CoreInterface::sendBareCore(JointCommand jc)
{   
    if (error_flag_ != 0)
        return error_flag_;

    prev_sec_ = sec_;
    prev_nsec_ = nsec_;
    initTrajectory();
    for (int i = 0; i < jc.total_points; ++i)
    {
        for (int j = 0; j < JOINT_NUM; ++j)
        {
            ts_.points[i].positions[j] = jc.points[i].positions[j];
            ts_.points[i].velocities[j] = jc.points[i].omega[j];
            ts_.points[i].effort[j] = jc.points[i].inertia[j];
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
        return WRITE_CORE_MEM_FAIL;
    }
    //printf("first:%d, level:%d, last:%d, level:%d\n", (ts_.points[0].time_from_start.sec*1000 +ts_.points[0].time_from_start.nsec/1000000),jc.points[0].point_position, (ts_.points[ts_.total_points-1].time_from_start.sec*1000 +ts_.points[ts_.total_points-1].time_from_start.nsec/1000000),jc.points[ts_.total_points-1].point_position);


    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  recvBareCore
// Summary: Read the actual joint states from CORE1
// In:      None
// Out:     fbjs -> the actual joint states.
// Return:  0 -> success to read the actual joint states.
//          READ_CORE_MEM_FAIL -> failed. 
//          OPEN_CORE_MEM_FAIL -> failed.
//------------------------------------------------------------
ERROR_CODE_TYPE CoreInterface::recvBareCore(FeedbackJointState &fbjs)
{
    if (error_flag_ != 0)
        return error_flag_;

    int result = readWriteSharedMem(handle_core_, &fbjs, "FeedbackJointState", MEM_READ);
    if (result == false) 
        return READ_CORE_MEM_FAIL;
    return FST_SUCCESS;
}


//------------------------------------------------------------
// Function:  sendBareCoreFake
// Summary: Send the trajectory segments to CORE1. (fake for test) 
// In:      jc -> the desired values of all joints
// Out:     None
// Return:  0 -> success to send the trajectory to CORE1.
//          WRITE_CORE_MEM_FAIL -> failed.
//------------------------------------------------------------
ERROR_CODE_TYPE CoreInterface::sendBareCoreFake(JointCommand jc)
{

    if (joints_in_fifo_.size() > 40) 
        return WRITE_CORE_MEM_FAIL;
    //push the trajectory joint states into fifo.
    for(int i = 0; i < jc.total_points; ++i)
    {
        joints_in_fifo_.push_back(jc.points[i]);
    }
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  recvBareCoreFake
// Summary: Read the actual joint states from CORE1 (fake for test)
// In:      None
// Out:     fbjs -> the actual joint states.
// Return:  0 -> success to read the actural joint states.
//------------------------------------------------------------
ERROR_CODE_TYPE CoreInterface::recvBareCoreFake(FeedbackJointState &fbjs)
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
        return 0;
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
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  setTimeStamp
// Summary: set the time stamp. default is 0.001s 
// In:      interval -> the unit is second.
// Out:     None
// Return:  None
//------------------------------------------------------------
void CoreInterface::setTimeInterval(double interval)
{
    time_step_ = interval * SEC_TO_NSEC;
}

} //namespace fst_core_interface

#endif //MIDDLEWARE_TO_MEM_CORE_INTERFACE_CPP_

