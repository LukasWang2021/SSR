/**
 * @file controller.cpp
 * @brief
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-9
 */
#include <signal.h>
#include "proto_parse.h"

#ifndef CROSS_PLATFORM
#include <gperftools/profiler.h>
#endif
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "sub_functions.h"
#include "common.h"
#include "rt_timer.h"
#include "tp_interface_version.h"
#include "log_manager/log_manager_logger.h"



#define BASIC_INTERVAL_TIME		(1) //basic cycle time(ms)

#define STATE_TIMER_ID			(0)
#define PROCESS_TIMER_ID		(1)


#define MAIN_PRIORITY           (80)

fst_log::Logger glog;
bool gs_running_flag = true;

/**
 * @brief: callback of SIGINT 
 *
 * @param dunno
 */
void sigroutine(int dunno)
{
    if (dunno == SIGSEGV)
    {
        FST_INFO("SIGSEGV...");
        gs_running_flag = false;
    }
    else
    {
	    FST_INFO("stop....\n");
	    gs_running_flag = false;
    }
//	exit(0);
}

/**
 * @brief: set thread priority 
 *
 * @param prio: input==> priority
 *
 * @return: true if success 
 */
bool setPriority(int prio)
{
    struct sched_param param;
    param.sched_priority = prio; 
    if (sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1) //set priority
    { 
        FST_ERROR("sched_setscheduler() failed"); 
        return false;  
    } 
    return true;

}


/**
 * @brief get message from TP and publish things that TP want to get repeatedly
 */
void commuProc(ProtoParse *proto_parse)
{	
    while (!proto_parse->getRobotMotionPtr()->shutdown_)
	{
       //auto t1 = std::chrono::system_clock::now(); 
		proto_parse->StartParser(); //get message from TP and parse the message
/*auto t2 = std::chrono::system_clock::now();*/
        //int x = std::chrono::duration_cast<std::chrono::milliseconds>( t2-t1 ).count();
        //if (x > 10)
            /*FST_ERROR("111delay too much===:%d", x);*/
        proto_parse->updateParams();
		proto_parse->pubParamList();	 // publish parameters the TP subscibed  
		//mSleep(INTERVAL_PROPERTY_UPDATE);
        rtMsSleep(INTERVAL_PROPERTY_UPDATE);
	}
}

void heartBeatProc(ProtoParse *proto_parse)
{	
    int err_size;
    U64 result;
    U64 err_list[128];
    RobotMotion *rob_motion = proto_parse->getRobotMotionPtr(); 
    
    while (!rob_motion->shutdown_)
	{
        err_size = rob_motion->motionHeartBeart(err_list);
        for (int i = 0; i < err_size; i++)
        {
            result = err_list[i];

            proto_parse->storeErrorCode(result);
        }
                
        result = rob_motion->getIOInterfacrPtr()->getIOError();
        if (result != TPI_SUCCESS)
        {
            proto_parse->storeErrorCode(result);
        }
        
        mSleep(INTERVAL_HEART_BEAT_UPDATE);         
    }
}


void* stateMachine(void *params)
{
    ProtoParse * proto_parse = (ProtoParse*)params;
    RobotMotion *robot_motion = proto_parse->getRobotMotionPtr(); //class RobotMotion
    //long cur_time = getCurTime();
    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
            exit(-2);
    }
    setPriority(MAIN_PRIORITY);
    using mseconds = std::chrono::duration<int, std::chrono::milliseconds::period>;
    
    struct timeval t1, t2, tre;
    gettimeofday(&t1, NULL);
    while (!robot_motion->shutdown_)
	{
        gettimeofday(&t1, NULL);
                //t1 = t2;
        //auto next_cycle = std::chrono::steady_clock::now() + mseconds(6);
		U64 result = robot_motion->checkProgramState();
        if (result != TPI_SUCCESS)
        {
            proto_parse->storeErrorCode(result);
        }

        gettimeofday(&t2, NULL);
        timeval_subtract(&tre, &t1, &t2);
        if (tre.tv_usec > 5000)
        {

            FST_ERROR("+++++++main delay too much===:%d, %ld, %ld,%ld, %ld", tre.tv_usec/1000, t1.tv_sec, t1.tv_usec, t2.tv_sec, t2.tv_usec);
        }

        //std::this_thread::sleep_until(next_cycle);
        //usleep(5000);
        rtMsSleep(INTERVAL_PROCESS_UPDATE);
    }// while (gs_running_flag)

}

int main(int argc, char **argv)
{    
	ros::init(argc, argv, "controller");	
	ros::NodeHandle n;	
    LOG_INIT();

    RosBasic ros_basic(&n); //init ros basic setting

	signal(SIGINT, sigroutine);	 
	signal(SIGTERM, sigroutine);           
   // signal(SIGSEGV, sigroutine);
    FST_INFO("VERSION:%d.%d.%d", tp_interface_VERSION_MAJOR, tp_interface_VERSION_MINOR, tp_interface_VERSION_PATCH);
    FST_INFO("BUILD TIME:%s", tp_interface_BUILD_TIME);

    ProtoParse proto_parse(&ros_basic);    
    
    
	//=========================================
	//ProtoParse proto_parse; //init class ProtoParse
//	ProfilerStart("my.prof"); //
	//thread to receive and reply messages from TP
	//publish message to TP
	boost::thread thrd_Sock_Server(boost::bind(commuProc, &proto_parse));
    boost::thread thrd_heart_beat(boost::bind(heartBeatProc, &proto_parse));
	//======start timer========================
    

    RobotMotion *robot_motion = proto_parse.getRobotMotionPtr(); //class RobotMotion
    //long cur_time = getCurTime();
    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) 
    {
            printf("mlockall failed: %m\n");
    }
    setPriority(MAIN_PRIORITY);
    using mseconds = std::chrono::duration<int, std::chrono::milliseconds::period>;
    
    struct timeval t1, t2, tre;
    gettimeofday(&t1, NULL);
    while (!robot_motion->shutdown_)
	{
        gettimeofday(&t1, NULL);
                //t1 = t2;
        //auto next_cycle = std::chrono::steady_clock::now() + mseconds(6);
		U64 result = robot_motion->checkProgramState();
        if (result != TPI_SUCCESS)
        {
            proto_parse.storeErrorCode(result);
        }

        gettimeofday(&t2, NULL);
        timeval_subtract(&tre, &t1, &t2);
        if (tre.tv_usec > 5000)
        {

            FST_ERROR("+++++++main delay too much===:%d, %ld, %ld,%ld, %ld", tre.tv_usec/1000, t1.tv_sec, t1.tv_usec, t2.tv_sec, t2.tv_usec);
        }

        //std::this_thread::sleep_until(next_cycle);
        //usleep(5000);
        rtMsSleep(5);
        //rtMsSleep(INTERVAL_PROCESS_UPDATE);
    }// while (gs_running_flag)

        
    //==============================================
	thrd_Sock_Server.join();
	thrd_heart_beat.join();

    //	ProfilerStop(); // stop profiling
		
	return 0;
}
