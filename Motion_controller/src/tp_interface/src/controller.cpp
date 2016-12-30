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

#define BASIC_INTERVAL_TIME		(1) //basic cycle time(ms)

#define STATE_TIMER_ID			(0)
#define PROCESS_TIMER_ID		(1)


#define MAIN_PRIORITY           (89)

static bool gs_running_flag = true;

inline void mSleep(int ms)
{
	usleep(ms*1000);
}

void rtMsSleep(int ms)
{
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);

    // Add the time you want to sleep
    deadline.tv_nsec += 1000000*ms;

    // Normalize the time to account for the second boundary
    if(deadline.tv_nsec >= 1000000000) {
        deadline.tv_nsec -= 1000000000;
        deadline.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
}

/**
 * @brief: callback of SIGINT 
 *
 * @param dunno
 */
void sigroutine(int dunno)
{
	FST_INFO("stop....\n");
	gs_running_flag = false;
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
	while (gs_running_flag)
	{
		proto_parse->StartParser(); //get message from TP and parse the message

        proto_parse->updateParams();
		proto_parse->pubParamList();	 // publish parameters the TP subscibed        

		mSleep(INTERVAL_PROPERTY_UPDATE);
	}
}

void heartBeatProc(ProtoParse *proto_parse)
{	
    int err_size;
    U64 err_list[128];
    while (gs_running_flag)
	{
        proto_parse->getRobotMotionPtr()->motionHeartBeart(err_list);
        mSleep(INTERVAL_HEART_BEAT_UPDATE); 
    }
}

int main(int argc, char **argv)
{    
	ros::init(argc, argv, "controller");	
	ros::NodeHandle n;	

    RosBasic ros_basic(&n); //init ros basic setting

	signal(SIGINT, sigroutine);	 
	signal(SIGTERM, sigroutine);           

    ProtoParse proto_parse(&ros_basic);
    
	//=========================================
	//ProtoParse proto_parse; //init class ProtoParse
//	ProfilerStart("my.prof"); //
	//thread to receive and reply messages from TP
	//publish message to TP
	boost::thread thrd_Sock_Server(boost::bind(commuProc, &proto_parse));
    boost::thread thrd_heart_beat(boost::bind(heartBeatProc, &proto_parse));
	//======start timer========================
    setPriority(MAIN_PRIORITY);

   // rtTimerStart();
  //  rtTimerAdd(STATE_TIMER_ID, INTERVAL_TIME_STATE);
   // rtTimerAdd(PROCESS_TIMER_ID, INTERVAL_TIME_PROCESS);    
    

    RobotMotion *robot_motion = proto_parse.getRobotMotionPtr(); //class RobotMotion
    //long cur_time = getCurTime();
    while (gs_running_flag)
	{

        proto_parse.checkManualWdt();	
		U64 result = robot_motion->queueProcess();
        if (result != FST_SUCCESS)
        {
            proto_parse.storeErrorCode(result);
        }
        rtMsSleep(INTERVAL_PROCESS_UPDATE);
    }// while (gs_running_flag)
    

	thrd_Sock_Server.join();
	thrd_heart_beat.join();

//	ProfilerStop(); // stop profiling
		
	return 0;
}
