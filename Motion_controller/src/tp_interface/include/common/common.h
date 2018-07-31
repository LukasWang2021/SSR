/**
 * @file common.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-17
 */
#ifndef TP_INTERFACE_COMMON_H_
#define TP_INTERFACE_COMMON_H_
#include <ros/ros.h>
#include "log_manager/log_manager_logger.h"
#include <boost/thread/shared_mutex.hpp>

extern fst_log::Logger glog;

#define LOG_INIT()    \
    do {\
            glog.initLogger("robot controller");\
            glog.setDisplayLevel(fst_log::MSG_LEVEL_INFO);\
    }while(0)

#define FST_INFO    glog.info
#define FST_ERROR   glog.error
#define FST_WARN    glog.warn

#define FST_ASSERT  ROS_ASSERT


//#define PI  3.1415926
//
typedef unsigned int U32;
typedef unsigned long long int U64;

#endif
