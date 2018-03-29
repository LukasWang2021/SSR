/**
 * @file main.cpp
 * @brief
 * @author WangWei
 * @version 1.0.0
 * @date 2017-08-9
 */
#include <signal.h>
 
#ifndef CROSS_PLATFORM
#include <gperftools/profiler.h>
#endif

#include "controller.h"
#include "tp_interface_version.h"


static bool gs_running_flag = true;

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
    Controller::exit();
}



int main(int argc, char **argv)
{    
	ros::init(argc, argv, "controller");	
	ros::NodeHandle n;	

    RosBasic ros_basic(&n);     //init ros basic setting
    ShareMem shm(&ros_basic);    //init ShareMem

	signal(SIGINT, sigroutine);	 
	signal(SIGTERM, sigroutine);           
   // signal(SIGSEGV, sigroutine);
    FST_INFO("VERSION:%d.%d.%d", tp_interface_VERSION_MAJOR, tp_interface_VERSION_MINOR, tp_interface_VERSION_PATCH);
    FST_INFO("BUILD TIME:%s", tp_interface_BUILD_TIME);

    Controller controller;  
    while (!controller.isTerminated())
    {
        usleep(100);
    }
//	ProfilerStop(); // stop profiling
		
	return 0;
}
