/**********************************************
File: TrajectoryQueue.cpp
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: trajectory queue to be sent
Author: 16-Aug-2016
Modifier:
**********************************************/
#include <iostream>
#include <dlfcn.h>
#include <sys/time.h>
#include "send_trajectory/trajectoryQueue.h"

namespace fst_trajectory_queue
{

TrajectoryQueue::TrajectoryQueue()
{
    interval_ = 0.001;
    sec_ = 0;
    nsec_ = 0;
    time_step_ = 0;
    flag_ = 0;
}

//------------------------------------------------------------
// Function:    init
// Summary: Load the .so libs and functions. Initialize some variables.
//------------------------------------------------------------
int TrajectoryQueue::init()
{
    handle_process = openMem(MEM_PROCESS);
    if (handle_process == -1) return(-1);
    handle_core = openMem(MEM_CORE);
    if (handle_core == -1) return(-1);

    //init the TrajectorySegment ts.
    initTrajectory();

    //init the joint states fbjs_
    for(int j = 0; j<JOINT_NUM;j++)
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
//------------------------------------------------------------
void TrajectoryQueue::initTrajectory()
{
    //init the TrajectorySegment ts.
    for(int i = 0; i<TS_POINT_NUM; i++)
    {
        for(int j = 0; j<JOINT_NUM;j++)
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
// Summary: Read the ik joint states from another process via middleware. 
//------------------------------------------------------------
bool TrajectoryQueue::receivedTrajectory()
{ 
    //if fifo3 size is bigger than (50-10), do not read data from fifo2
//    printf("||====fifo3 %u points====|| \n", joints_in_fifo_.size());
    if(joints_in_fifo_.size() > (FIFO_LEN - JC_POINT_NUM)) return false;
    
    JointCommand jc_r;
    //Read the data from fifo2 of another process 
    int read_result = readWriteSharedMem(handle_process, &jc_,"JointCommand", MEM_READ);
    if(read_result == true)
    {
        jc_r = jc_;  
    }else
    {
        return false;
    }
    //push the data into fifo3
    for(int i = 0; i < jc_r.total_points; i++)
    {
        joints_in_fifo_.push_back(jc_r.points[i]);
    }

    return true;
}

//------------------------------------------------------------
// Function:  sendTrajectory
// Summary: Send the trajectory segments to the bare core via shared memory 
//------------------------------------------------------------
bool TrajectoryQueue::sendTrajectory()
{
    if(joints_in_fifo_.empty()) return false;
    
    Points temp_point;
    int loop_num, i, j;
    int fifo_size = joints_in_fifo_.size();
    if(flag_ == 0)
    {
        loop_num = fifo_size >= TS_POINT_NUM?TS_POINT_NUM:fifo_size;
        initTrajectory();

        for(i = 0; i < loop_num; i++)
        {
            temp_point = joints_in_fifo_[i];
            for(j = 0; j < JOINT_NUM;j++)
            {
                ts_.points[i].positions[j] = temp_point.positions[j];
            }   
            //Adding time stamp for trajectory 
            if(temp_point.point_position == START_POINT)
            {
                sec_ = 0;
                nsec_ = time_step_;    
            }else
            {
                nsec_ += time_step_;
                if(nsec_ >= SEC_TO_NSEC)
                {
                    nsec_ -= SEC_TO_NSEC;
                    sec_ += 1;
                }
            }
            ts_.points[i].time_from_start.sec = sec_;
            ts_.points[i].time_from_start.nsec = nsec_;
        
            //Set the last point of trajectory  
            if(temp_point.point_position == END_POINT) 
            {
                ts_.last_fragment = 1;
                sec_ = 0;
                nsec_ = 0;
                loop_num = (i+1);
                break;
            }
        } 
        ts_.total_points = loop_num;
    }

    int count = 0;
    //Write the trajectory for the core, writing attempt is 2.
    while(count < ATTEMPTS)
    {
        int write_result = readWriteSharedMem(handle_core, &ts_,"TrajectorySegment", MEM_WRITE);
        if(write_result == true)
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
// Summary: Read the actual joint states from the bare core via shared memory 
//------------------------------------------------------------
bool TrajectoryQueue::readJointStates()
{
    FeedbackJointState fbjs_r;
    int count = 0;
    //Read the feedback joint states from the core, reading attempt is 2;
    while(count < ATTEMPTS)
    {
        int read_result = readWriteSharedMem(handle_core, &fbjs_r,"FeedbackJointState", MEM_READ);
        if(read_result == true)
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
// Summary: send the actual joint states for upper ware 
//------------------------------------------------------------
bool TrajectoryQueue::sendJointStates()
{
    FeedbackJointState fbjs_w = fbjs_;  
    int count = 0;
    //Write the feedback joint states for another process, writing attempt is 2.
    while(count < ATTEMPTS)
    {
        int write_result = readWriteSharedMem(handle_process, &fbjs_w,"FeedbackJointState", MEM_WRITE);
        if(write_result == true)
        {
            //Just for printing
/*            int i;
            for(i=0;i<JOINT_NUM; i++){printf("fbjs_w.position[%d] = %f \n", i, fbjs_w.position[i]);}
//            for(i=0;i<JOINT_NUM; i++){printf("fbjs_w.velocity[%d] = %f \n", i, fbjs_w.velocity[i]);}
//            for(i=0;i<JOINT_NUM; i++){printf("fbjs_w.effort[%d] = %f \n", i, fbjs_w.effort[i]);}
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
//------------------------------------------------------------
void TrajectoryQueue::setTimeInterval(double interval)
{
    interval_ = interval;
    time_step_ = interval_ * SEC_TO_NSEC;
}

void TrajectoryQueue::versionInfo()
{
    //reading the version of bared metal core
    BareCoreVersion version;
    int read_result = false;
    while(read_result == false)
    {
        read_result = readWriteSharedMem(handle_core, &version, "BareCoreVersion", MEM_READ);
        if(read_result == true)
        {
            break;
        }else
        {
            usleep(20);
        }
    }
    std::cout<<"||============Bared metal core version ============||"<<std::endl;
    std::cout<<"||modified date: "<<version.year<<"."<<version.month<<"."<<version.day<<std::endl;
    std::cout<<"||modified version: "<<version.major<<"."<<version.minor<<"."<<version.revision<<std::endl;
    std::cout<<"||modified description: "<<version.description<<std::endl;
    std::cout<<"||servo description: "<<version.servoVersion<<std::endl;
    std::cout<<"||============Bared metal core version ============||"<<std::endl;

}

TrajectoryQueue::~TrajectoryQueue()
{

}
}

int main(int argc, char** argv)
{
    fst_trajectory_queue::TrajectoryQueue tq;
    int init_result = tq.init();
    if (init_result == -1) return false;

    tq.versionInfo();
 
    while(true)
    {
        //compute the time of interval  
/*        struct timeval t_start, t_end;
        long cost_time = 0;
        gettimeofday(&t_start, NULL);
        long start = ((long)t_start.tv_sec)*1000000+(long)t_start.tv_usec;
*/
        tq.receivedTrajectory();
        tq.sendTrajectory();
        tq.readJointStates();
        tq.sendJointStates();

        //end time
/*        gettimeofday(&t_end, NULL);
        long end = ((long)t_end.tv_sec)*1000000+(long)t_end.tv_usec;
        cost_time = end - start;
        std::cout<<" ====Cost time : "<<cost_time<<"us"<<std::endl;
*/
        usleep(1000);
    }

}



