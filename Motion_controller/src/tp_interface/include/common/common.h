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
            glog.setDisplayLevel(fst_log::MSG_LEVEL_ERROR);\
    }while(0)

#ifdef PRINT
#ifndef LOGGER
#define FST_ERROR ROS_ERROR
#define FST_WARN(format, ...) \
	do { \
		ROS_INFO("\033[33m" format "\033[0m", ##__VA_ARGS__); \
	} while (0) 
#define FST_INFO(format, ...) \
	do { \
		ROS_INFO("\033[36m" format "\033[0m", ##__VA_ARGS__); \
	} while (0) 
#else
#define FST_INFO(format, ...) do {glog.info("\033[36m" format "\033[0m", ##__VA_ARGS__);}while(0)
#define FST_ERROR   glog.error
#define FST_WARN    glog.warn
#endif
#else
#define FST_ERROR ROS_ERROR
#define FST_WARN(format, ...) \
	do { \
		ROS_INFO("\033[33m" format "\033[0m", ##__VA_ARGS__); \
	} while (0) 
#define FST_INFO(format, ...)
#endif


#define FST_PRINT(format, ...) \
	do { \
		printf("\033[46m" format "\033[0m", ##__VA_ARGS__); \
	} while (0) 

#define FST_ASSERT  ROS_ASSERT


//#define PI  3.1415926
//
typedef unsigned int U32;
typedef unsigned long long int U64;

#endif
