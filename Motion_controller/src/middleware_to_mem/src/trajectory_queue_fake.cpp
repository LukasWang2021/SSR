/**********************************************
File: TrajectoryQueueFake.cpp
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: fake joint_States feedback
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_FAKE_CPP_
#define MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_FAKE_CPP_

#include "send_trajectory/trajectory_queue_fake.h"
#include <dlfcn.h>
#include <sys/time.h>
#include <iostream>

namespace fst_trajectory_queue
{

TrajectoryQueueFake::TrajectoryQueueFake()
{
    handle_process_ = 0;
    handle_core_ = 0;
    interval_ = 0.001;
    sec_ = 0;
    nsec_ = 0;
    time_step_ = 1000000; //ns
    flag_ = 0;
}

//------------------------------------------------------------
// Function:    init
// Summary: Open the shared memory area.
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed.
//------------------------------------------------------------
int TrajectoryQueueFake::init()
{
    handle_process_ = openMem(MEM_PROCESS);
    if (handle_process_ == -1) return(-1);
    clearSharedmem(MEM_PROCESS);

    //init the TrajectorySegment ts.
    initTrajectory();

    //init the joint states fbjs_
    for (int j = 0; j<JOINT_NUM; ++j)
    {
        fbjs_.position[j] = 0;
        fbjs_.velocity[j] = 0;
        fbjs_.effort[j] = 0;
    }
    fbjs_.state = 0;

    //init the time_step_
    time_step_ = interval_ * SEC_TO_NSEC;
}

//------------------------------------------------------------
// Function:  initTrajectory
// Summary: Reset the trajectory variable. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void TrajectoryQueueFake::initTrajectory()
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
// Function:  ReceiveTrajectory
// Summary: Read the joint command from the motion control(upper level).
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to read the joint command data. 
//------------------------------------------------------------
bool TrajectoryQueueFake::receivedTrajectory()
{ 
    //====test JointCommand====
    //Writing Joint Command
/*    JointCommand jc_w;
    for(int i = 0; i<SEG_POINT_NUM; ++i)
    {   
        for(int j = 0; j<JOINT_NUM; ++j)
        {
            jc_w.points[i].positions[j] = 0.1*i*j;
        }
    }
    jc_w.total_points = 10;
    readWriteSharedMem(handle_process, &jc_w, "JointCommand", MEM_WRITE);
*/

    //if fifo3 size is bigger than (50-10), do not read data from fifo2.
    if (joints_in_fifo_.size() > (FIFO_LEN - JC_POINT_NUM)) return false;
    JointCommand jc_r;
    //Read the data from fifo2 of another process. 
    int read_result = readWriteSharedMem(handle_process_, &jc_, "JointCommand", MEM_READ);
    if (read_result == true)
    {
        jc_r = jc_;  
    }
    else
    {
        return false;
    }
    //push the data into fifo3.
    for(int i = 0; i < jc_r.total_points; ++i)
    {
        joints_in_fifo_.push_back(jc_r.points[i]);
    }

    return true;
}

//------------------------------------------------------------
// Function:  sendTrajectory
// Summary: Push the point from the fifo3
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to send the trajectory to the servo core. 
//------------------------------------------------------------
bool TrajectoryQueueFake::sendTrajectory()
{
    if (joints_in_fifo_.empty()) 
    {
        fbjs_.state = STATE_READY;
        return false;
    }
  
	 for (int i = 0; i < JOINT_NUM; ++i)
    {
        fbjs_.position[i] = joints_in_fifo_[0].positions[i];
    }
    fbjs_.state = STATE_RUNNING;
    joints_in_fifo_.erase(joints_in_fifo_.begin());
	 return true;
}

//------------------------------------------------------------
// Function:  readJointStates
// Summary: None
// In:      None
// Out:     None
// Return:  true -> success.
//------------------------------------------------------------
bool TrajectoryQueueFake::readJointStates()
{
    return true;
}

//------------------------------------------------------------
// Function:  sendJointStates
// Summary: send the actual joint states for upper level. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to send the actual joint states to upper level.
//------------------------------------------------------------
bool TrajectoryQueueFake::sendJointStates()
{
    FeedbackJointState fbjs_w = fbjs_;  
    int count = 0;
    //Write the feedback joint states for another process, writing attempt is 2.
    while (count < ATTEMPTS)
    {
        int write_result = readWriteSharedMem(handle_process_, &fbjs_w, "FeedbackJointState", MEM_WRITE);
        if (write_result == true)
        {
            //Just for printing.
//            for(int i=0;i<JOINT_NUM; ++i){printf("fbjs_w.position[%d] = %f \n", i, fbjs_w.position[i]);}
//            printf(" fbjs_w.state = %d\n\n", fbjs_w.state);
            printf("||====fifo3 left %u points====|| \n",joints_in_fifo_.size());
            return true;
        }     
        count++;
        usleep(20);
    }
    return false;
}

//------------------------------------------------------------
// Function:  setTimeStamp
// Summary: set the time stamp. default is 0.001s 
// In:      interval -> 0.001 is default
// Out:     None
// Return:  None
//------------------------------------------------------------
void TrajectoryQueueFake::setTimeInterval(double interval)
{
    interval_ = interval;
    time_step_ = interval_ * SEC_TO_NSEC;
}

int TrajectoryQueueFake::versionInfo()
{

}

int TrajectoryQueueFake::startServiceThread()
{
    pthread_t id;
    int result = pthread_create(&id, NULL, threadHelper, this);
    if (result != 0)
    {
        std::cout<<"Create pthread error!"<<std::endl;
        return false;
    }
    return true;
}


void *TrajectoryQueueFake::threadHelper(void *self)
{
    TrajectoryQueueFake *ptr = (TrajectoryQueueFake *)self;
    ptr->heartbeatService();
}


int TrajectoryQueueFake::heartbeatService()
{
    int req_result = false, res_result = false;
    long cost_time = 0, start_time = 0, end_time = 0;
    struct timeval t_start, t_end;
    ServiceRequest heartbeat_request = {110, "heartbeat"};
    ServiceResponse heartbeat_response;

    while (true)
    {
        req_result = false;
        gettimeofday(&t_start, NULL);
        start_time = ((long)t_start.tv_sec) + (long)t_start.tv_usec/1000000; 
        while (res_result == false)
        {
            if (req_result == false) 
            {
                req_result = clientSendRequest(handle_process_, &heartbeat_request);
            }
            //wait for one microsecond to get response.
            usleep(1000);

            if (req_result == true) 
            {
                res_result = clientGetResponse(handle_process_, &heartbeat_response);
            }
 
            gettimeofday(&t_end, NULL);
            end_time = ((long)t_end.tv_sec) + (long)t_end.tv_usec/1000000;
            cost_time = end_time - start_time;
            if (cost_time > 1)
            {
                std::cout<<"\033[31m"<<"No heartbeat from core1"<<"\033[0m"<<std::endl;
            }
        }
        //send a heartbeat signal every one second.
        sleep(1);
    }
}


TrajectoryQueueFake::~TrajectoryQueueFake()
{

}
} //namespace fst_trajectory_queue

int main(int argc, char** argv)
{
    fst_trajectory_queue::TrajectoryQueueFake tq;
    int init_result = tq.init();
    if (init_result == -1) return false;

    //start another thread for heartbeat.
    tq.startServiceThread();
 
    while (true)
    {
        tq.receivedTrajectory();
        tq.sendTrajectory();
        tq.sendJointStates();

        usleep(1000);
    }
}


#endif //MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_FAKE_CPP_
