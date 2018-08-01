/*************************************************************************
	> File Name: motion_plan_reuse.h
	> Author:   Feng Yun
	> Mail:     yun.feng@foresight-robotics.com
	> Created Time: 2018年02月27日 星期二 10时20分34秒
 ************************************************************************/

#ifndef _TP_COMMON_H_
#define _TP_COMMON_H_

#include <ros/ros.h>
#include "log_manager/log_manager_logger.h"
extern fst_log::Logger glog;

// #define FST_LOG(fmt, ...)       glog.log(fmt, ##__VA_ARGS__)
// #define FST_INFO(fmt, ...)      glog.info(fmt, ##__VA_ARGS__)
// #define FST_WARN(fmt, ...)      glog.warn(fmt, ##__VA_ARGS__)
// #define FST_ERROR(fmt, ...)     glog.error(fmt, ##__VA_ARGS__)

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


#endif