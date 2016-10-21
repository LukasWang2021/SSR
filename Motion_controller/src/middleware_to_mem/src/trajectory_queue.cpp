/**********************************************
File: trajectory_queue.cpp
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: trajectory queue to be sent
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_CPP_
#define MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_CPP_

#include "send_trajectory/trajectory_queue.h"
#include <dlfcn.h>
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include "service_actions/service_actions.h"

namespace fst_trajectory_queue
{

TrajectoryQueue::TrajectoryQueue()
{
    handle_process_ = 0;
    handle_core_ = 0;
    interval_ = 0.001; //the unit is s.
    sec_ = 0;
    nsec_ = 0;
    time_step_ = 1000000; //the unit is ns.
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
int TrajectoryQueue::init()
{
    handle_process_ = openMem(MEM_PROCESS);
    if (handle_process_ == -1) return false;
    clearSharedmem(MEM_PROCESS);
    handle_core_ = openMem(MEM_CORE);
    if (handle_core_ == -1) return false;
    clearSharedmem(MEM_CORE);

    //init the TrajectorySegment ts_.
    initTrajectory();

    //init the joint states fbjs_.
    for (int j = 0; j < JOINT_NUM; ++j)
    {
        fbjs_.position[j] = 0;
        fbjs_.velocity[j] = 0;
        fbjs_.effort[j] = 0;
    }
    fbjs_.state = 0;

    //init the time_step_.
    time_step_ = interval_ * SEC_TO_NSEC;

    //init the service request_fifo_
    ServiceRequest request_info = {0x10, "request for core1 version\n"};
    request_fifo_.push_back(request_info);

    return true;
}

//------------------------------------------------------------
// Function:  initTrajectory
// Summary: Initial the trajectory variable. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void TrajectoryQueue::initTrajectory()
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
bool TrajectoryQueue::receiveTrajectory()
{ 
    //if fifo3 size is bigger than (50-10), do not read data from fifo2.
    if (joints_in_fifo_.size() > (FIFO_LEN - JC_POINT_NUM)) return false;
    
    JointCommand jc_r;
    //Read the joint command from fifo2 of another process. 
    int read_result = readWriteSharedMem(handle_process_, &jc_, "JointCommand", MEM_READ);
    if (read_result == true)
    {
        jc_r = jc_;  
    }
    else
    {
        return false;
    }
    //push the point into fifo3.
    for (int i = 0; i < jc_r.total_points; ++i)
    {
        joints_in_fifo_.push_back(jc_r.points[i]);
    }

    return true;
}

//------------------------------------------------------------
// Function:  sendTrajectory
// Summary: Add the time stamp for each points. 
//          Send the trajectory segments to the servo core(lower level).
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to send the trajectory to the servo core 
//------------------------------------------------------------
bool TrajectoryQueue::sendTrajectory()
{
    if (joints_in_fifo_.empty()) return false;
    
    Points temp_point;
    int loop_num = 0;
    int fifo_size = joints_in_fifo_.size();
    if (flag_ == 0)
    {
        loop_num = fifo_size >= TS_POINT_NUM?TS_POINT_NUM:fifo_size;
        initTrajectory();

        for (int i = 0; i < loop_num; ++i)
        {
            temp_point = joints_in_fifo_[i];
            for (int j = 0; j < JOINT_NUM; ++j)
            {
                ts_.points[i].positions[j] = temp_point.positions[j];
            }   
            //Adding time stamp for trajectory. 
            if (temp_point.point_position == START_POINT)
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
            if (temp_point.point_position == END_POINT) 
            {
                ts_.last_fragment = 1;
                sec_ = 0;
                nsec_ = 0;
                loop_num = (i + 1);
                break;
            }
        } // end for (int i = 0; i < loop_num; ++i)
        ts_.total_points = loop_num;
    } // end if (flag_ == 0)

    int count = 0;
    //Write the trajectory for the core, writing attempt is 2.
    while (count < ATTEMPTS)
    {
        int write_result = readWriteSharedMem(handle_core_, &ts_, "TrajectorySegment", MEM_WRITE);
        if (write_result == true)
        {
            joints_in_fifo_.erase(joints_in_fifo_.begin(), joints_in_fifo_.begin()+ts_.total_points);
            flag_ = 0;
            printf("||====fifo3 left %u points====|| \n",joints_in_fifo_.size());
            return true;
        }
        flag_ = 1;
        count++;
        usleep(20);
    }
    return false;
}

//------------------------------------------------------------
// Function:  readJointStates
// Summary: Read the actual joint states from the servo core
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to read the actural joint states from the servo core(lower level) 
//------------------------------------------------------------
bool TrajectoryQueue::readJointStates()
{
    FeedbackJointState fbjs_r;
    int count = 0;
    //Read the feedback joint states from the core, reading attempt is 2.
    while (count < ATTEMPTS)
    {
        int read_result = readWriteSharedMem(handle_core_, &fbjs_r, "FeedbackJointState", MEM_READ);
        if (read_result == true)
        {
            fbjs_ = fbjs_r;
            return true;
        }        
        count++;
        usleep(20);        
    }
    return false;
}

//------------------------------------------------------------
// Function:  sendJointStates
// Summary: send the actual joint states for upper level 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to send the actual joint states to upper level 
//------------------------------------------------------------
bool TrajectoryQueue::sendJointStates()
{
    FeedbackJointState fbjs_w = fbjs_;  
    int count = 0;
    //Write the feedback joint states for another process, writing attempt is 2.
    while (count < ATTEMPTS)
    {
        int write_result = readWriteSharedMem(handle_process_, &fbjs_w, "FeedbackJointState", MEM_WRITE);
        if (write_result == true)
        {
            //Just for printing
/*            int i;
            for(i=0;i<JOINT_NUM; ++i){printf("fbjs_w.position[%d] = %f \n", i, fbjs_w.position[i]);}
            printf(" fbjs_w.state = %d\n\n", fbjs_w.state);
*/
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
void TrajectoryQueue::setTimeInterval(double interval)
{
    interval_ = interval;
    time_step_ = interval_ * SEC_TO_NSEC;
}

//------------------------------------------------------------
// Function:  receiveRequest
// Summary: Receive the request from the motion controll system. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool TrajectoryQueue::receiveRequest()
{
    if (request_fifo_.size() > FIFO_LEN) return false;

    ServiceRequest request;
    int read_result = serverGetRequest(handle_process_, &request_);
    if (read_result == true)
    {
        request = request_;  
        //push the request into fifo.
        request_fifo_.push_back(request);
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
bool TrajectoryQueue::sendRequest()
{
    if (request_fifo_.empty()) return false;

    ServiceRequest temp_request = request_fifo_[0];   
    int write_result = clientSendRequest(handle_core_, &temp_request);
    if (write_result == true)
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
bool TrajectoryQueue::getResponse()
{
    ServiceResponse response;
    
    int read_result = clientGetResponse(handle_core_, &response);
    if (read_result == true)
    {
        response_fifo_.push_back(response);
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  manageResponse
// Summary: deal with the service response from CORE1. 
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed. 
//------------------------------------------------------------
bool TrajectoryQueue::manageResponse()
{
    if (response_fifo_.empty()) return false;

    ServiceResponse temp_response = response_fifo_[0];
    int index = searchServiceTableIndex(&temp_response);
    if (index != -1) 
    {
        service_table[index].function(&temp_response);
        response_fifo_.erase(response_fifo_.begin());
    }
    else
    {
        int send_result = serverSendResponse(handle_process_, &temp_response);
        if (send_result == true) response_fifo_.erase(response_fifo_.begin());
    }
    return true;
}

//------------------------------------------------------------
// Function:  startServiceThread
// Summary: create two threads for service and heartbeat.
// In:      None
// Out:     None
// Return:  true -> success.
//          false -> failed to create the threads. 
//------------------------------------------------------------
bool TrajectoryQueue::startServiceThread()
{
    pthread_t service_thread, heartbeat_thread;
    int result = pthread_create(&service_thread, NULL, serviceThreadHelper, this);
    if (result != 0)
    {
        std::cout<<"Create service pthread error!"<<std::endl;
        return false;
    }
    result = pthread_create(&heartbeat_thread, NULL, heartbeatThreadHelper, this);
    if (result != 0)
    {
        std::cout<<"Create heartbeat pthread error!"<<std::endl;
        return false;
    }
    return true;
}

//------------------------------------------------------------
// Function:  serviceThreadHelper
// Summary: a bridge to run the thread method. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void *TrajectoryQueue::serviceThreadHelper(void *self)
{
    TrajectoryQueue *ptr = (TrajectoryQueue *)self;
    ptr->runService();
}

//------------------------------------------------------------
// Function:  heartbeatThreadHelper
// Summary: a bridge to run the thread method. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void *TrajectoryQueue::heartbeatThreadHelper(void *self)
{
    TrajectoryQueue *ptr = (TrajectoryQueue *)self;
    ptr->runHeartbeat();
}

//------------------------------------------------------------
// Function:  runService
// Summary: the main method of the service thread. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void TrajectoryQueue::runService()
{
    int req_result = false, res_result = false;
    long cost_time = 0, start_time = 0, end_time = 0;
    struct timeval t_start, t_end;
    //deal with the request which being sent to CORE1
    while (true)
    {
        usleep(SERVICE_CYCLE); 
        if (request_fifo_.empty())
        {
            getResponse();
            continue;
        }        
        req_result = false;
        res_result = false;
        gettimeofday(&t_start, NULL);
        start_time = ((long)t_start.tv_sec) + (long)t_start.tv_usec/1000000;
        //in the loop, the elapsed time is evaluated until the service is done successfully.
        while (res_result == false)
        {
            if (req_result == false) req_result = sendRequest();
            //wait for core1 response 
            usleep(1000);
            if (req_result == true) res_result = getResponse();

            gettimeofday(&t_end, NULL);
            end_time = ((long)t_end.tv_sec) + (long)t_end.tv_usec/1000000;
            cost_time = end_time - start_time;
            if (cost_time > HEARTBEAT_LIMIT)
            {
                std::cout<<"\033[31m"<<"||====No heartbeat from core1====||"<<"\033[0m"<<std::endl;
                res_result = true;
            }
        } // end while (res_result == false)
    } // end while (true)
}

//------------------------------------------------------------
// Function:  runHeartbeat
// Summary: the main method of the heartbeat thread. 
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void TrajectoryQueue::runHeartbeat()
{
    ServiceRequest heartbeat_request = {0x11, "heartbeat"};
    while (true)
    {      
        usleep(HEARTBEAT_CYCLE);  
        //push the heartbeat request into fifo every one second.
        if (request_fifo_.empty()) request_fifo_.push_back(heartbeat_request);               
    }
}


TrajectoryQueue::~TrajectoryQueue()
{

}
} //namespace fst_trajectory_queue


int main(int argc, char** argv)
{
    fst_trajectory_queue::TrajectoryQueue tq;
    int init_result = tq.init();
    if (init_result == -1) return false;

    //start the threads for service.
    tq.startServiceThread();

    while (true)
    {
    //compute the time of interval  
/*      struct timeval t_start, t_end;
        long cost_time = 0;
        gettimeofday(&t_start, NULL);
        long start = ((long)t_start.tv_sec)*1000000+(long)t_start.tv_usec;
*/
        tq.receiveTrajectory();
        tq.sendTrajectory();

        tq.readJointStates();
        tq.sendJointStates();

        tq.receiveRequest();
        tq.manageResponse();

        //end time
/*        gettimeofday(&t_end, NULL);
        long end = ((long)t_end.tv_sec)*1000000+(long)t_end.tv_usec;
        cost_time = end - start;
        std::cout<<" ====Cost time : "<<cost_time<<"us"<<std::endl;
*/
        usleep(1000);
    }

}


#endif //MIDDLEWARE_TO_MEM_TRAJECTORY_QUEUE_CPP_
